"""Ensaio de hover comandado pela Pi: prepara, voa, grava e analisa.

Um ensaio é um comando só, feito para ser rodado por SSH e repetido mudando
um valor por vez:

    ~/drone/dronectl hover-test --motors                 # voo real (perfil: 30 cm, 20 s)
    ~/drone/dronectl hover-test --motors --set CTL_W0=1080 --label "w0 mais alto"
    ~/drone/dronectl hover-test --sim                    # drone simulado no STM32
    ~/drone/dronectl hover-test                          # a seco: sensores reais, motores parados

O que ele faz, em ordem:

1. deixa o STM32 com **os valores de fábrica + o perfil + os ``--set`` deste
   ensaio**, nada herdado de um ensaio anterior: na primeira vez em cada boot
   do STM32 ele lê os valores de fábrica dos parâmetros que toca e, nos
   seguintes, volta a eles antes de aplicar o perfil
   (``~/drone/perfil_hover.json``). Cada valor é conferido no que o STM32
   devolve. Com pesos Q (``--q``) diferentes dos do LQG ajustado, projeta Kd e
   Kfd (``tools/lqg_ajustado.py``, o mesmo dlqr da tabela do firmware) e envia
   com CRC; sem eles, volta aos ganhos de fábrica;
2. liga o DRONE_CTRL_DEBUG (50 Hz), confere os sensores e mede o chão;
3. arma, decola até a altura, segura o tempo pedido **depois de chegar** e
   pousa. Proteções da Pi, além das do STM32: altura 25 cm acima do alvo,
   saída da cerca (80 cm do ponto de decolagem), telemetria parada por 1 s,
   Ctrl+C ou a sessão SSH caindo (SIGHUP) → pouso; **Ctrl+C de novo corta os
   motores** (desarma onde estiver);
4. grava tudo em ``~/real_log_kit/hover_<data>/`` (incluindo o CSV completo
   ``real_log.csv`` para planilhas), e imprime a análise
   (``hover_report``) com a sugestão para o próximo voo, que fica guardada no
   perfil.

O perfil só muda quando alguém pede (``hover-profile --apply-suggestion``,
``hover-profile --set`` ou ``hover-test --save``): a sugestão é uma conta,
não uma decisão.
"""
from __future__ import annotations

import csv
import json
import math
import signal
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

from .gains import REPO_ROOT, Gains
from .hover_report import (ARMED, FAILSAFE, HOLD, LEG, MOTOR_OF_INPUT, STANDBY, TAKEOFF,
                           CSV_HEADER, Report, Sample, analyze, format_report, sample_row)
from .link import DroneLink, LinkError, state_name

#: Perfil padrão (fora de tools/: o deploy não apaga).
PROFILE_PATH = Path.home() / 'drone' / 'perfil_hover.json'
#: Kit autocontido de cada ensaio. Fica fora do deploy para não ser apagado.
RESULTS_DIR = Path.home() / 'real_log_kit'
#: Altura e tempo de hover de um perfil novo.
DEFAULT_HEIGHT_CM = 30.0
DEFAULT_TIME_S = 20.0
#: Teto de altura do firmware (FLIGHT_MAX_HEIGHT_M), em cm.
MAX_HEIGHT_CM = 50.0
#: Menor altura aceita, em cm (abaixo disso é efeito solo puro).
MIN_HEIGHT_CM = 5.0
#: Taxa do DRONE_CTRL_DEBUG durante o ensaio, em Hz (~70% do enlace a 115200).
DEBUG_RATE_HZ = 50.0
#: Proteções da Pi.
OVERSHOOT_M = 0.25
GEOFENCE_M = 0.80
TELEMETRY_TIMEOUT_S = 1.0
#: Verificações seguidas (a 20 Hz) com a altura ou a cerca estouradas antes de
#: pousar: um pico isolado do sensor não encerra o voo.
GUARD_CHECKS = 5
#: Inclinação máxima no chão antes de armar, em graus.
GROUND_TILT_MAX_DEG = 15.0
#: Partida dos motores antes da rampa de subida (FLIGHT_SPINUP + rampa do hover), em s.
SPINUP_S = 3.0
#: Folga para chegar na altura além da partida e da rampa, em s.
CLIMB_MARGIN_S = 20.0
#: Rampa de subida padrão do firmware (FLT_Z_RATE), em m/s.
Z_RATE_DEFAULT = 0.05
#: Tempo máximo esperando o desarme depois de pedir pouso, em s.
LAND_WAIT_S = 30.0
#: Distância do alvo que conta como "chegou", em m (a mesma da análise).
REACHED_M = 0.03
#: Altura abaixo da qual o drone conta como no chão, em m.
ON_GROUND_M = 0.05
# Durante a decolagem, Ctrl+C a poucos centímetros do chão precisa cortar
# antes que o pouso normal mantenha o hover por mais de meio segundo.
# O ensaio de 08/11 abortado a 8,4 cm ainda terminou em failsafe no pouso.
LOW_ABORT_MEAS_M = 0.10
LOW_ABORT_EST_M = 0.12
#: Ensaio a seco: tempo depois do tempo de hover até pousar, em s.
DRY_EXTRA_S = 3.0
#: Ensaios guardados no histórico do perfil.
HISTORY_MAX = 60
#: Bit de DRONE_PROTOCOL_INFO.capabilities: motores liberados (FLT_DRY_RUN = 0).
CAPABILITY_ACTUATION = 32
#: meas_flags que precisam estar presentes no chão: atitude, giro e altura.
MEAS_NEEDED = 1 | 2 | 4
#: Parâmetros gravados em todo ensaio (mesmo sem mudar), para a análise e o registro.
RECORDED = ('CTL_W0', 'CTL_FF_TH', 'CTL_FF_PHI', 'CTL_FF_PSI', 'SNS_LVL_PHI', 'SNS_LVL_TH',
            'CTL_XY', 'CTL_LQG_AJ', 'CTL_DIST', 'CTL_DIST_MAX', 'MOT_MAX', 'MOT_W_AT_0',
            'MOT_W_AT_100', 'FLT_Z_RATE', 'FLT_TILT_MAX')
#: Parâmetros que o próprio ensaio controla (não vêm do perfil).
MANAGED = ('FLT_SIM', 'FLT_DRY_RUN', 'DBG_RATE_HZ')


# ---- perfil --------------------------------------------------------------------

@dataclass
class Profile:
    """Valores com que o próximo ensaio começa (persistidos em JSON).

    ``boot_values`` guarda o valor de fábrica, no boot ``boot_id`` do STM32,
    de cada parâmetro que algum ensaio já tocou: é para ele que um parâmetro
    volta quando sai do perfil.
    """

    path: Path
    params: Dict[str, float] = field(default_factory=dict)
    q: Dict[str, float] = field(default_factory=dict)
    r: Optional[float] = None
    height_cm: float = DEFAULT_HEIGHT_CM
    time_s: float = DEFAULT_TIME_S
    suggestion: Dict[str, float] = field(default_factory=dict)
    history: List[Dict] = field(default_factory=list)
    boot_id: int = 0
    boot_values: Dict[str, float] = field(default_factory=dict)

    @classmethod
    def load(cls, path: Path = PROFILE_PATH) -> 'Profile':
        if not path.exists():
            return cls(path=path)
        data = json.loads(path.read_text(encoding='utf-8'))
        r = data.get('r')
        return cls(path=path,
                   params={k.upper(): float(v) for k, v in data.get('params', {}).items()},
                   q={k: float(v) for k, v in data.get('q', {}).items()},
                   r=float(r) if r is not None else None,
                   height_cm=float(data.get('height_cm', DEFAULT_HEIGHT_CM)),
                   time_s=float(data.get('time_s', DEFAULT_TIME_S)),
                   suggestion={k: float(v) for k, v in data.get('suggestion', {}).items()},
                   history=list(data.get('history', [])),
                   boot_id=int(data.get('boot_id', 0)),
                   boot_values={k: float(v) for k, v in data.get('boot_values', {}).items()})

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {'params': self.params, 'q': self.q, 'r': self.r, 'height_cm': self.height_cm,
                'time_s': self.time_s, 'suggestion': self.suggestion,
                'history': self.history[-HISTORY_MAX:], 'boot_id': self.boot_id,
                'boot_values': self.boot_values}
        tmp = self.path.with_suffix('.tmp')
        tmp.write_text(json.dumps(data, indent=1, ensure_ascii=False), encoding='utf-8')
        tmp.replace(self.path)

    def apply_suggestion(self, keys: Optional[List[str]] = None) -> Dict[str, float]:
        """Passa a última sugestão (toda, ou só ``keys``) para os parâmetros do perfil."""
        wanted = {k.upper() for k in keys} if keys else None
        chosen = {k: v for k, v in self.suggestion.items() if wanted is None or k in wanted}
        self.params.update(chosen)
        return chosen

    def text(self) -> str:
        lines = [f'perfil {self.path}', f'altura {self.height_cm:g} cm, tempo {self.time_s:g} s']
        lines.append('parâmetros: ' + (', '.join(f'{k}={v:g}' for k, v in sorted(self.params.items()))
                                       or '(os de fábrica do firmware)'))
        lines.append('pesos Q: ' + (', '.join(f'{k}={v:g}' for k, v in self.q.items())
                                    or '(os do LQG ajustado do firmware)')
                     + (f', R={self.r:g}' if self.r is not None else ''))
        if self.suggestion:
            lines.append('última sugestão: ' +
                         ', '.join(f'{k}={v:g}' for k, v in self.suggestion.items()))
        if self.history:
            lines.append('últimos ensaios:')
            for h in self.history[-8:]:
                label = f" [{h['label']}]" if h.get('label') else ''
                lines.append(f"  {h.get('folder')}  {h.get('mode')}{label}  {h.get('summary', '')}")
        return '\n'.join(lines)


def parse_assignments(items: Optional[List[str]], upper: bool = True) -> Dict[str, float]:
    """``['CTL_W0=1080', 'CTL_FF_TH=-40,CTL_XY=1']`` -> dicionário."""
    out: Dict[str, float] = {}
    for item in items or []:
        for part in item.split(','):
            if not part.strip():
                continue
            name, sep, value = part.partition('=')
            if not sep or not name.strip():
                raise ValueError(f'esperado NOME=VALOR, veio "{part}"')
            key = name.strip().upper() if upper else name.strip()
            out[key] = float(value.strip())
    return out


# ---- ganhos do LQG ajustado ---------------------------------------------------

def _ajustado():
    """Módulo tools/lqg_ajustado.py (Python puro, o mesmo que gera a tabela do firmware)."""
    tools = str(REPO_ROOT / 'tools')
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import lqg_ajustado
    return lqg_ajustado


def check_q_names(q: Dict[str, float]) -> None:
    """Confere os nomes dos estados nos pesos Q (x, y, z, phi, ..., r)."""
    names = _ajustado().lp.ESTADOS
    unknown = set(q) - set(names)
    if unknown:
        raise ValueError(f'estado(s) desconhecido(s) nos pesos Q: {sorted(unknown)} '
                         f'(use {", ".join(names)})')


def adjusted_gains(q_override: Dict[str, float],
                   r: Optional[float]) -> Tuple[Gains, List[float], float]:
    """Kd/Kfd do dlqr do ajustado com os pesos Q pedidos e o Kalman do ajustado.

    ``q_override`` usa os nomes dos estados (x, y, z, phi, theta, psi, vx, vy,
    vz, p, q, r); o que faltar fica com o Q do LQG ajustado do firmware. L e
    M são os do ajustado (``kalman_lqg(RN_AJUSTADO)``), os mesmos que o
    firmware usa com os ganhos de fábrica.

    Returns:
        Os ganhos, o Q completo usado e o R usado.
    """
    aj = _ajustado()
    check_q_names(q_override)
    q = [float(q_override.get(n, v)) for n, v in zip(aj.lp.ESTADOS, aj.Q_AJUSTADO)]
    rr = float(r) if r is not None else aj.R_TCC
    if rr <= 0 or any(v < 0 for v in q):
        raise ValueError('Q não pode ser negativo e R precisa ser positivo')
    Kd, Kfd = aj.ganhos_lqg(q, rr)
    M, L = aj.kalman_lqg(aj.RN_AJUSTADO)
    return Gains(Kd=Kd, Kfd=Kfd, L=L, M=M), q, rr


# ---- ensaio --------------------------------------------------------------------

@dataclass
class Options:
    """O que este ensaio vai fazer."""

    height_m: float
    time_s: float
    mode: str                              # 'motors', 'sim' ou 'dry'
    params: Dict[str, float] = field(default_factory=dict)
    q: Dict[str, float] = field(default_factory=dict)
    r: Optional[float] = None
    rate_hz: float = DEBUG_RATE_HZ
    geofence_m: float = GEOFENCE_M
    label: str = ''
    out_dir: Path = RESULTS_DIR
    countdown_s: float = 3.0

    def check(self) -> None:
        """Recusa um ensaio mal pedido antes de falar com o drone."""
        if self.mode not in ('motors', 'sim', 'dry'):
            raise ValueError(f'modo desconhecido: {self.mode}')
        if not MIN_HEIGHT_CM <= self.height_m * 100 <= MAX_HEIGHT_CM:
            raise ValueError(f'altura de {self.height_m * 100:g} cm fora de {MIN_HEIGHT_CM:g} a '
                             f'{MAX_HEIGHT_CM:g} cm (a altura é em cm)')
        if not 0 < self.time_s <= 300:
            raise ValueError('tempo de hover entre 0 e 300 s')
        if not 0 < self.rate_hz <= 50:
            raise ValueError('taxa do DRONE_CTRL_DEBUG entre 0 e 50 Hz')
        if self.geofence_m <= 0:
            raise ValueError('cerca precisa ser positiva')
        managed = sorted(set(self.params) & set(MANAGED))
        if managed:
            raise ValueError(f'{", ".join(managed)} é do próprio ensaio (use --motors ou --sim)')
        check_q_names(self.q)


def _safe_print(text: str) -> None:
    """print que não derruba o ensaio se o terminal sumir (SSH caiu)."""
    try:
        print(text, flush=True)
    except (OSError, ValueError):
        pass


class HoverTest:
    """Um ensaio de hover sobre um enlace já aberto."""

    def __init__(self, link: DroneLink, opts: Options, profile: Profile,
                 say: Callable[[str], None] = _safe_print) -> None:
        opts.check()
        self.link = link
        self.opts = opts
        self.profile = profile
        self.say = say
        #: (time.monotonic da chegada, DebugSample).
        self.samples: List[Tuple[float, object]] = []
        self.texts: List[Tuple[float, str]] = []
        self.stop_reason: Optional[str] = None
        self.kill = False
        self.applied: Dict[str, float] = {}
        #: Fotografia de todos os parâmetros do STM32 imediatamente antes de armar.
        self.params_snapshot: Dict[str, float] = {}
        self.gains_info: Dict = {}
        self.ground: Optional[float] = None
        self.armed_by_us = False
        #: Por que o voo terminou (pedido de pouso da Pi ou o STM32 encerrando).
        self.land_reason: Optional[str] = None
        self.error: Optional[str] = None
        self.folder: Optional[Path] = None
        self.report: Optional[Report] = None
        link.on_text = self._on_text
        link.subscribe(self._on_sample)

    # -- recepção (thread do enlace) --

    def _on_sample(self, s) -> None:
        self.samples.append((time.monotonic(), s))

    def _on_text(self, text: str) -> None:
        self.texts.append((time.monotonic(), text))
        self.say(f'  [STM32] {text}')

    # -- sinais --

    def _request_stop(self, signum, _frame) -> None:
        if self.stop_reason is None:
            self.stop_reason = {signal.SIGINT: 'Ctrl+C',
                                signal.SIGTERM: 'SIGTERM'}.get(signum, 'SIGHUP (SSH caiu)')
        elif signum == signal.SIGINT:
            self.kill = True

    # -- estado --

    def _last(self):
        return self.samples[-1][1] if self.samples else None

    def _state(self) -> int:
        """Estado de voo da amostra mais nova (50 Hz) ou, sem ela, do HEARTBEAT."""
        if self.samples and time.monotonic() - self.samples[-1][0] < 0.5:
            return self.samples[-1][1].state
        return self.link.flight_state

    def _fresh_state(self, timeout: float = 1.5) -> int:
        """Estado de voo de um HEARTBEAT recebido agora (o guardado pode ter 0,5 s)."""
        t0 = time.monotonic()
        end = t0 + timeout
        while time.monotonic() < end and self.link.last_fc_heartbeat <= t0:
            time.sleep(0.02)
        return self.link.flight_state

    def _height(self, s) -> Tuple[float, float]:
        """Altura medida e estimada acima do chão, em m."""
        return self.ground - s.y[0], self.ground - s.x_hat[2]

    # -- preparação --

    def _set(self, name: str, value: float) -> float:
        got = self.link.param_set(name, value)
        if not math.isclose(got, value, rel_tol=1e-4, abs_tol=1e-4):
            raise LinkError(f'{name}: pedido {value:g}, o STM32 ficou com {got:g} '
                            '(fora dos limites?)')
        self.applied[name] = got
        return got

    def _boot_values(self, names: List[str]) -> Dict[str, float]:
        """Valores de fábrica deste boot do STM32 para ``names`` (lê os que faltam)."""
        p = self.profile
        if p.boot_id != self.link.boot_id:
            # O STM32 reiniciou: o que está nele agora é o de fábrica.
            p.boot_id, p.boot_values = self.link.boot_id, {}
        for name in names:
            if name not in p.boot_values:
                p.boot_values[name] = self.link.param_get(name)
        return p.boot_values

    def _wait_samples(self, count: int, timeout: float) -> None:
        start = len(self.samples)
        end = time.monotonic() + timeout
        while time.monotonic() < end and len(self.samples) - start < count:
            time.sleep(0.02)

    def _apply_params(self) -> None:
        """Fábrica + perfil + este ensaio; FLT_SIM e FLT_DRY_RUN pelo modo."""
        o = self.opts
        wanted = {k: v for k, v in self.profile.params.items() if k not in MANAGED}
        wanted.update(o.params)
        touched = sorted(set(wanted) | set(self.profile.boot_values) | set(RECORDED))
        touched = [n for n in touched if n not in MANAGED]
        boot = self._boot_values(touched)
        self._set('FLT_SIM', 1.0 if o.mode == 'sim' else 0.0)
        self._set('FLT_DRY_RUN', 0.0 if o.mode == 'motors' else 1.0)
        for name in touched:
            self._set(name, wanted.get(name, boot[name]))
        changed = {k: self.applied[k] for k in touched
                   if not math.isclose(self.applied[k], boot[k], rel_tol=1e-4, abs_tol=1e-4)}
        self.say('fora de fábrica neste ensaio: ' +
                 (', '.join(f'{k}={v:g}' for k, v in sorted(changed.items())) or 'nada'))
        self.profile.save()  # guarda os valores de fábrica lidos

    def _apply_gains(self) -> None:
        q = dict(self.profile.q)
        q.update(self.opts.q)
        r = self.opts.r if self.opts.r is not None else self.profile.r
        if not q and r is None:
            crc = self.link.restore_gains()
            self.gains_info = {'custom': False, 'crc': f'{crc:08X}'}
            self.say(f'ganhos de fábrica (LQG ajustado do firmware), crc {crc:08X}')
            return
        if self.applied.get('CTL_LQG_AJ', 1.0) < 0.5:
            raise LinkError('os pesos Q são do LQG ajustado: precisa de CTL_LQG_AJ = 1')
        try:
            gains, q_used, r_used = adjusted_gains(q, r)
        except ValueError as exc:
            raise LinkError(str(exc)) from exc
        crc = self.link.upload_gains(gains)
        self.gains_info = {'custom': True, 'q_changed': q, 'q': q_used, 'r': r_used,
                           'crc': f'{crc:08X}'}
        self.say(f'ganhos do ajustado com Q {q or "de fábrica"} e R={r_used:g} em uso '
                 f'(crc {crc:08X})')

    def _snapshot_for_log(self) -> None:
        """Lê a configuração completa que ficará repetida no CSV do ensaio.

        A leitura acontece desarmado, depois de aplicar perfil, ``--set`` e
        ganhos, para que o arquivo prove exatamente com que configuração o
        voo começou. ``param_list`` também pega parâmetros que o hover-test
        normalmente não altera (sinais, limites, curva de motor etc.).
        """
        self.params_snapshot = self.link.param_list()
        self.applied = dict(self.params_snapshot)

    def prepare(self) -> None:
        o = self.opts
        state = self._fresh_state()
        if state == FAILSAFE:
            self.say('STM32 em FAILSAFE (motores parados): desarmando para voltar a STANDBY')
            self.link.arm(False)
            state = self._fresh_state()
        if state != STANDBY:
            raise LinkError(f'o drone precisa estar desarmado no chão (STANDBY); agora: '
                            f'{state_name(state)}')
        self._apply_params()
        self._apply_gains()
        if o.mode == 'motors':
            # O bit chega no DRONE_PROTOCOL_INFO (1 Hz) depois de FLT_DRY_RUN = 0.
            end = time.monotonic() + 2.5
            while time.monotonic() < end and not self.link.capabilities & CAPABILITY_ACTUATION:
                time.sleep(0.1)
            if not self.link.capabilities & CAPABILITY_ACTUATION:
                raise LinkError('o STM32 não liberou os motores (LQG_FLIGHT_ENABLED = 0 no '
                                'firmware?)')

        self._set('DBG_RATE_HZ', o.rate_hz)
        self._snapshot_for_log()
        self._wait_samples(int(o.rate_hz), 3.0)
        s = self._last()
        if s is None:
            raise LinkError('sem DRONE_CTRL_DEBUG: a telemetria não chegou')
        if (s.meas_flags & MEAS_NEEDED) != MEAS_NEEDED:
            raise LinkError(f'sensores incompletos no chão (meas=0x{s.meas_flags:X}: '
                            'precisa de atitude, giro e altura)')
        tilt = math.degrees(max(abs(s.y[1]), abs(s.y[2])))
        if tilt > GROUND_TILT_MAX_DEG:
            raise LinkError(f'drone inclinado {tilt:.0f}° no chão: nivele antes de decolar')
        pool = [x.y[0] for _, x in self.samples[-int(o.rate_hz):]
                if x.state == STANDBY and x.meas_flags & 4]
        if not pool:
            raise LinkError('sem altura medida no chão')
        self.ground = sum(pool) / len(pool)
        self.say(f'chão: sensor a {-self.ground * 100:.1f} cm do piso (variação '
                 f'{(max(pool) - min(pool)) * 100:.1f} cm em {len(pool)} amostras)')

    # -- voo --

    def _countdown(self) -> None:
        o = self.opts
        if o.mode == 'motors' and o.countdown_s > 0:
            self.say(f'MOTORES VÃO GIRAR: decolando em {o.countdown_s:.0f} s (Ctrl+C cancela; '
                     'perto do chão na decolagem corta; acima disso pousa; '
                     'Ctrl+C de novo corta os motores)')
            end = time.monotonic() + o.countdown_s
            while time.monotonic() < end and not self.stop_reason:
                time.sleep(0.05)
        if self.stop_reason:
            raise LinkError(f'cancelado antes de armar ({self.stop_reason})')

    def fly(self) -> None:
        o = self.opts
        self._countdown()
        self.link.start_session()
        self.link.arm(True)
        self.armed_by_us = True
        self.link.takeoff(o.height_m, timeout_s=120.0)
        t_takeoff = time.monotonic()
        z_rate = self.applied.get('FLT_Z_RATE') or Z_RATE_DEFAULT
        climb_limit = SPINUP_S + o.height_m / max(z_rate, 0.01) + CLIMB_MARGIN_S
        reached_at: Optional[float] = None
        last_print = 0.0
        land_reason: Optional[str] = None
        over = outside = 0
        while land_reason is None:
            time.sleep(0.05)
            now = time.monotonic()
            if self.stop_reason:
                land_reason = self.stop_reason
                break
            state = self._state()
            if state in (STANDBY, FAILSAFE) and now - t_takeoff > 1.0:
                self.land_reason = f'o STM32 encerrou o voo ({state_name(state)})'
                self.say(self.land_reason)
                return
            if not self.samples or now - self.samples[-1][0] > TELEMETRY_TIMEOUT_S:
                land_reason = 'telemetria parou'
                break
            s = self._last()
            h_meas, h = self._height(s)
            # A medida só vale com o MTF respondendo; a estimativa vale sempre.
            h_top = max(h, h_meas) if s.meas_flags & 4 else h
            off = math.hypot(s.x_hat[0] - s.ref[0], s.x_hat[1] - s.ref[1])
            over = over + 1 if h_top > o.height_m + OVERSHOOT_M else 0
            outside = outside + 1 if off > o.geofence_m else 0
            if over >= GUARD_CHECKS:
                land_reason = (f'altura {h_top * 100:.0f} cm, mais de '
                               f'{OVERSHOOT_M * 100:.0f} cm acima do alvo')
            elif outside >= GUARD_CHECKS:
                land_reason = (f'{off * 100:.0f} cm do ponto de decolagem '
                               f'(cerca de {o.geofence_m * 100:.0f} cm)')
            elif reached_at is None and abs(h - o.height_m) <= REACHED_M and \
                    s.state in (TAKEOFF, HOLD, LEG):
                reached_at = now
                self.say(f'chegou a {h * 100:.0f} cm em {now - t_takeoff:.1f} s: '
                         f'segurando {o.time_s:g} s')
            elif reached_at is not None and now - reached_at >= o.time_s:
                land_reason = 'fim do tempo de hover'
            elif reached_at is None and o.mode == 'dry' and \
                    now - t_takeoff >= o.time_s + DRY_EXTRA_S:
                land_reason = 'fim do ensaio a seco'
            elif reached_at is None and o.mode != 'dry' and now - t_takeoff > climb_limit:
                land_reason = f'não chegou a {o.height_m * 100:.0f} cm em {climb_limit:.0f} s'
            if now - last_print >= 1.0:
                last_print = now
                self.say(self._line(now - t_takeoff, s, h))
        self.land_reason = land_reason
        self.say(f'pousando: {land_reason}')
        self.land()

    @staticmethod
    def _line(t: float, s, h: float) -> str:
        motors = [0.0] * 4
        for i, m in enumerate(MOTOR_OF_INPUT):
            motors[m - 1] = s.throttle[i] * 100
        return (f'{t:5.1f}s {state_name(s.state):<8} h={h * 100:5.1f} cm  '
                f'x={s.x_hat[0] * 100:+5.1f} y={s.x_hat[1] * 100:+5.1f} cm  '
                f'φ={math.degrees(s.y[1]):+5.1f} θ={math.degrees(s.y[2]):+5.1f} '
                f'ψ={math.degrees(s.y[3]):+6.1f}  M1..M4=' +
                ' '.join(f'{v:3.0f}' for v in motors))

    def _disarm_now(self, why: str) -> None:
        self.say(f'CORTANDO OS MOTORES: {why}')
        try:
            self.link.arm(False)
        except LinkError as exc:
            self.say(f'desarme sem resposta ({exc}); sem a autorização o STM32 pousa sozinho')
            self.link.end_session()

    def land(self) -> None:
        """Pede pouso e espera o desarme (Ctrl+C de novo corta os motores)."""
        # Durante a decolagem, um cancelamento a poucos centímetros do chão
        # não deve manter o empuxo de hover enquanto o firmware aguarda a
        # referência de pouso e mais 500 ms de contato. No ensaio de 08/11 o
        # Ctrl+C veio a 2,6 cm e o failsafe ocorreu nesse intervalo. Exige
        # amostra recente, altura medida e estimada baixas; em voo mais alto,
        # o primeiro Ctrl+C continua pedindo pouso.
        s = self._last()
        if (self.stop_reason == 'Ctrl+C' and self.opts.mode == 'motors' and
                self._state() == TAKEOFF and self.ground is not None and s is not None and
                self.samples and time.monotonic() - self.samples[-1][0] < 0.2 and
                s.meas_flags & 4):
            h_meas, h_est = self._height(s)
            if h_meas < LOW_ABORT_MEAS_M and h_est < LOW_ABORT_EST_M:
                self._disarm_now('Ctrl+C durante a decolagem, perto do chão')
                end = time.monotonic() + 2.0
                while time.monotonic() < end and self._state() not in (STANDBY, FAILSAFE):
                    time.sleep(0.05)
                return
        try:
            self.link.land()
        except LinkError as exc:
            # Sem a autorização, o STM32 segura e pousa sozinho.
            self.say(f'pouso: {exc}; sem a autorização o STM32 pousa sozinho')
            self.link.end_session()
        end = time.monotonic() + LAND_WAIT_S
        while time.monotonic() < end and self._state() not in (STANDBY, FAILSAFE):
            if self.kill:
                self.kill = False
                self._disarm_now('Ctrl+C de novo')
                end = min(end, time.monotonic() + 3.0)
            time.sleep(0.05)
        if self._state() in (STANDBY, FAILSAFE):
            return
        s = self._last()
        if s is not None and self.ground is not None and self._height(s)[0] < ON_GROUND_M:
            self._disarm_now('no chão e ainda armado')
        else:
            self.say('ATENÇÃO: o drone não pousou no tempo esperado (dronectl land / disarm)')

    def abort(self) -> None:
        """Depois de um erro: desarma se ainda está no chão, pousa se está no ar."""
        state = self._state()
        if not self.armed_by_us or state in (STANDBY, FAILSAFE):
            return
        if state == ARMED:
            self._disarm_now('erro antes de decolar')
        else:
            self.land()

    def finish(self) -> None:
        """Volta ao estado seguro: sem autorização, sem telemetria rápida, a seco, sem simulado."""
        self.link.end_session()
        state = self._state()
        if state == FAILSAFE:
            # Motores já parados: desarmar volta a STANDBY, onde FLT_DRY_RUN e
            # FLT_SIM podem mudar (o motivo do FAILSAFE já ficou nos textos).
            try:
                self.link.arm(False)
            except LinkError:
                pass
            state = self._fresh_state()
        if state != STANDBY:
            self.say('aviso: o drone ainda está armado; FLT_SIM e FLT_DRY_RUN ficaram como estão')
            return
        for name, value in (('DBG_RATE_HZ', 0.0), ('FLT_DRY_RUN', 1.0), ('FLT_SIM', 0.0)):
            try:
                got = self.link.param_set(name, value)
            except LinkError:
                got = float('nan')
            if not math.isclose(got, value, abs_tol=1e-6):
                self.say(f'aviso: não consegui voltar {name} para {value:g}')

    # -- registro --

    def meta(self) -> Dict:
        o = self.opts
        return {'date': datetime.now().isoformat(timespec='seconds'), 'label': o.label,
                'mode': o.mode, 'height_m': o.height_m, 'time_s': o.time_s,
                'params': self.params_snapshot or self.applied, 'set_this_test': o.params,
                'gains': self.gains_info,
                'boot_id': self.link.boot_id, 'ground_y_z': self.ground,
                'land_reason': self.land_reason, 'stop_reason': self.stop_reason,
                'error': self.error}

    def save(self, folder: Path) -> Report:
        folder.mkdir(parents=True, exist_ok=True)
        samples = list(self.samples)
        texts = list(self.texts)
        with open(folder / 'telemetria.csv', 'w', newline='', encoding='utf-8') as f:
            w = csv.writer(f)
            w.writerow(CSV_HEADER)
            for t_pi, s in samples:
                w.writerow(sample_row(t_pi, s))
        self._save_real_log(folder, samples)
        self._save_gains(folder)
        (folder / 'textos.txt').write_text(''.join(f'{t:.3f}\t{txt}\n' for t, txt in texts),
                                           encoding='utf-8')
        meta = self.meta()
        (folder / 'meta.json').write_text(json.dumps(meta, indent=1, ensure_ascii=False),
                                          encoding='utf-8')
        report = analyze([Sample(t=s.time_ms / 1000.0, state=s.state, meas=s.meas_flags,
                                 flags=s.ctrl_flags, y=list(s.y), x=list(s.x_hat),
                                 du=list(s.du), ref=list(s.ref), thr=list(s.throttle), t_pi=t_pi)
                          for t_pi, s in samples], texts, meta)
        if self.land_reason and self.land_reason != 'fim do tempo de hover':
            report.notes.insert(0, f'fim do voo: {self.land_reason}')
        if self.error:
            report.notes.insert(0, f'ensaio interrompido: {self.error}')
        (folder / 'resumo.json').write_text(
            json.dumps({'metrics': report.metrics, 'suggestion': report.suggestion,
                        'notes': report.notes}, indent=1, ensure_ascii=False), encoding='utf-8')
        (folder / 'linha_do_tempo.txt').write_text('\n'.join(report.timeline) + '\n',
                                                   encoding='utf-8')
        return report

    def _gain_snapshot(self) -> Dict:
        """Descrição auditável dos ganhos esperados para o CRC que foi usado."""
        q_changed = self.gains_info.get('q_changed', {})
        r = self.gains_info.get('r')
        gains, q, r_used = adjusted_gains(q_changed, r)
        return {'custom': bool(self.gains_info.get('custom')), 'crc': self.gains_info.get('crc'),
                'q': q, 'r': r_used, 'state_names': _ajustado().lp.ESTADOS,
                'Kd': gains.Kd, 'Kfd': gains.Kfd, 'L': gains.L, 'M': gains.M}

    def _save_gains(self, folder: Path) -> None:
        """Salva as matrizes uma vez; o CSV guarda CRC, Q e R em cada linha."""
        (folder / 'gains.json').write_text(
            json.dumps(self._gain_snapshot(), indent=1, ensure_ascii=False), encoding='utf-8')

    def _save_real_log(self, folder: Path, samples: List[Tuple[float, object]]) -> None:
        """CSV de planilha, com cabeçalhos físicos e contexto repetido em cada amostra.

        ``telemetria.csv`` continua sendo o formato compacto interno do
        hover-report. Este arquivo é deliberadamente largo: pode ser aberto
        isoladamente no LibreOffice/Excel e não exige juntar meta ou ganhos.
        """
        state_names = _ajustado().lp.ESTADOS
        gain = self._gain_snapshot()
        params = self.params_snapshot or self.applied
        header = (
            ['t_pi_s', 't_stm32_ms', 'step', 'state', 'meas_flags', 'ctrl_flags',
             'mode', 'boot_id', 'target_height_m', 'ground_y_z_m',
             'm1_pct', 'm2_pct', 'm3_pct', 'm4_pct'] +
            ['y_z_m', 'y_phi_rad', 'y_theta_rad', 'y_psi_rad', 'y_vx_m_s', 'y_vy_m_s',
             'y_p_rad_s', 'y_q_rad_s', 'y_r_rad_s'] +
            ['x_m', 'y_m', 'z_m', 'phi_rad', 'theta_rad', 'psi_rad', 'vx_m_s', 'vy_m_s',
             'vz_m_s', 'p_rad_s', 'q_rad_s', 'r_rad_s'] +
            ['du_u1_rad_s', 'du_u2_rad_s', 'du_u3_rad_s', 'du_u4_rad_s',
             'ref_x_m', 'ref_y_m', 'ref_z_m', 'ref_psi_rad',
             'thr_u1_pct', 'thr_u2_pct', 'thr_u3_pct', 'thr_u4_pct',
             'gain_custom', 'gain_crc'] +
            [f'q_{name}' for name in state_names] + ['r_lqr'] +
            [f'param_{name}' for name in sorted(params)]
        )
        with open(folder / 'real_log.csv', 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f, delimiter=';')
            writer.writerow(header)
            for t_pi, s in samples:
                motors = [0.0] * 4
                for input_index, motor in enumerate(MOTOR_OF_INPUT):
                    motors[motor - 1] = s.throttle[input_index] * 100.0
                row = ([f'{t_pi:.6f}', s.time_ms, s.step, s.state, s.meas_flags, s.ctrl_flags,
                        self.opts.mode, self.link.boot_id, f'{self.opts.height_m:.6g}',
                        '' if self.ground is None else f'{self.ground:.6g}'] +
                       [f'{v:.6f}' for v in motors] +
                       [f'{v:.8g}' for v in s.y] + [f'{v:.8g}' for v in s.x_hat] +
                       [f'{v:.8g}' for v in s.du] + [f'{v:.8g}' for v in s.ref] +
                       [f'{v * 100.0:.6f}' for v in s.throttle] +
                       [int(gain['custom']), gain['crc'] or ''] +
                       [f'{v:.12g}' for v in gain['q']] + [f'{gain["r"]:.12g}'] +
                       [f'{params[name]:.9g}' for name in sorted(params)])
                writer.writerow(row)

    # -- tudo --

    def run(self) -> Tuple[Path, Report]:
        """Prepara, voa, volta ao estado seguro, grava, analisa e guarda no perfil."""
        self.folder = self.opts.out_dir / f'hover_{datetime.now():%Y%m%d_%H%M%S}'
        handlers = {sig: signal.signal(sig, self._request_stop)
                    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
        try:
            try:
                self.prepare()
                self.fly()
            except Exception as exc:  # qualquer falha: primeiro pôr o drone no chão
                self.error = str(exc) if isinstance(exc, (LinkError, OSError)) else \
                    f'{type(exc).__name__}: {exc}'
                self.say(f'erro: {self.error}')
                try:
                    self.abort()
                except LinkError as again:
                    self.say(f'erro ao pousar: {again}; sem a autorização o STM32 pousa sozinho')
            finally:
                self.finish()
        finally:
            for sig, old in handlers.items():
                signal.signal(sig, old)
        self.report = self.save(self.folder)
        self._remember()
        return self.folder, self.report

    def _remember(self) -> None:
        """Guarda a sugestão e uma linha do ensaio no perfil."""
        m = self.report.metrics
        parts = [f"máx {m.get('max_height_cm')} cm", f"janela {m.get('window_s')} s"]
        if 'height' in m:
            parts.append(f"h RMS {m['height']['err_rms_cm']} cm")
        if 'xy' in m:
            parts.append(f"XY médio {m['xy']['mean_cm']} RMS {m['xy']['rms_cm']} cm")
        if self.error:
            parts.append(f'erro: {self.error}')
        self.profile.suggestion = dict(self.report.suggestion)
        self.profile.history.append({'folder': self.folder.name, 'mode': self.opts.mode,
                                     'label': self.opts.label,
                                     'height_cm': round(self.opts.height_m * 100, 1),
                                     'set': self.opts.params, 'q': self.opts.q,
                                     'summary': ', '.join(parts),
                                     'suggestion': self.report.suggestion})
        self.profile.save()


def run_hover_test(link: DroneLink, opts: Options, profile: Profile,
                   say: Callable[[str], None] = _safe_print) -> HoverTest:
    """Roda um ensaio e imprime a análise; devolve o ensaio (pasta, relatório, erro)."""
    test = HoverTest(link, opts, profile, say)
    folder, report = test.run()
    say('')
    say(format_report(report))
    say(f'\ngravado em {folder}')
    return test
