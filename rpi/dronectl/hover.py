"""Ensaio de hover comandado pela Pi: decola, segura, grava e pousa.

``dronectl hover --height 30 --time 20`` arma pela autorização do operador, decola
até a altura, fica ``--time`` segundos em HOLD e pousa. Do armar ao desarmar grava,
numa pasta ``~/drone/ensaios/hover_<data>``:

- ``telemetry.csv``: cada DRONE_CTRL_DEBUG (10 Hz) com o relógio da Pi, medidas,
  estimativa, Δω, referência, motores, flags e a tensão do SYS_STATUS;
- ``texts.txt``: os avisos do STM32 (STATUSTEXT) e os da Pi, com hora;
- ``summary.json``: o resumo de :func:`analyze` e as condições do ensaio.

O controle roda no STM32; a Pi só pede os trechos e observa. Para segurar o drone
com cabo e IHM na USB, use a aba Controle da IHM; este comando é para o voo com a
Pi a bordo (ou para o STM32 em FLT_SIM, motores em zero).
"""

from __future__ import annotations

import csv
import json
import math
import os
import statistics
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional

from .link import DebugSample, DroneLink, LinkError
from .mission import MAX_HEIGHT_CM, MIN_HEIGHT_CM, Mission, parse

#: Onde os ensaios ficam (mesma pasta dos ensaios da submissão).
DEFAULT_ROOT = Path.home() / 'drone' / 'ensaios'
#: DRONE_FLIGHT_STATE de HOLD (estável no ar).
STATE_HOLD = 6
#: DRONE_CTRL_FLAG_SATURATED.
FLAG_SATURATED = 0x08

COLUMNS = (
    ['t_pi_s', 't_ms', 'step', 'state', 'meas_flags', 'ctrl_flags']
    + [f'y{i}' for i in range(9)]
    + [f'x{i}' for i in range(12)]
    + [f'du{i}' for i in range(4)]
    + [f'ref{i}' for i in range(4)]
    + [f'thr{i}' for i in range(4)]
    + ['vbat_v']
)


class Recorder:
    """Grava cada DRONE_CTRL_DEBUG numa lista e num CSV, a partir da thread do enlace."""

    def __init__(self, link: DroneLink, csv_path: Optional[Path]) -> None:
        self.link = link
        self.samples: List[DebugSample] = []
        self._t0 = time.monotonic()
        self._file = None
        self._writer = None
        self._texts = None
        self._last_sync = 0.0
        if csv_path is not None:
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            self._file = open(csv_path, 'w', newline='', encoding='utf-8')
            self._writer = csv.writer(self._file)
            self._writer.writerow(COLUMNS)
            self._texts = open(csv_path.parent / 'texts.txt', 'a', encoding='utf-8')

    def __call__(self, s: DebugSample) -> None:
        self.samples.append(s)
        if self._writer is not None:
            vbat = self.link.battery_v
            self._writer.writerow(
                [f'{time.monotonic() - self._t0:.3f}', s.time_ms, s.step, s.state, s.meas_flags]
                + [s.ctrl_flags]
                + [f'{v:.5g}' for v in s.y + s.x_hat + s.du + s.ref + s.throttle]
                + ['' if vbat is None else f'{vbat:.3f}']
            )
            self._sync()

    def note(self, text: str) -> None:
        """Aviso com hora em ``texts.txt``, gravado na hora (sobrevive a falta de energia)."""
        if self._texts is not None:
            self._texts.write(f'{time.monotonic() - self._t0:8.3f}  {text}\n')
            self._sync(force=True)

    def _sync(self, force: bool = False) -> None:
        """Leva o que foi escrito até o cartão a cada 0,5 s (e na hora para avisos).

        Sem isso o CSV fica no buffer e some se a Pi perde energia no voo, como no
        ensaio de 12/11 às 17:14, que ficou vazio.
        """
        now = time.monotonic()
        if not force and now - self._last_sync < 0.5:
            return
        self._last_sync = now
        for f in (self._file, self._texts):
            if f is not None:
                f.flush()
                os.fsync(f.fileno())

    def close(self) -> None:
        self._sync(force=True)
        for name in ('_file', '_texts'):
            f = getattr(self, name)
            if f is not None:
                f.close()
                setattr(self, name, None)


def _rms(values: List[float]) -> float:
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float('nan')


def analyze(samples: List[DebugSample]) -> Dict[str, float]:
    """Resumo dos passos em HOLD: erro de altura, desvio horizontal, guinada, motores.

    Altura = −z estimado (NED, relativo ao chão do armamento); erro em relação à
    referência do STM32. XY é a distância da posição estimada à referência: mede
    o quanto o drone segura o ponto, segundo o próprio EKF (não é verdade de solo).
    """
    hold = [s for s in samples if s.state == STATE_HOLD]
    if not hold:
        return {'hold_samples': 0}
    dh = [(-s.x_hat[2]) - (-s.ref[2]) for s in hold]
    dxy = [math.hypot(s.x_hat[0] - s.ref[0], s.x_hat[1] - s.ref[1]) for s in hold]
    dpsi = [
        math.atan2(math.sin(s.x_hat[5] - s.ref[3]), math.cos(s.x_hat[5] - s.ref[3])) for s in hold
    ]
    thr = [statistics.fmean(s.throttle) for s in hold]
    return {
        'hold_samples': len(hold),
        'hold_s': (hold[-1].time_ms - hold[0].time_ms) / 1000.0,
        'height_ref_m': -hold[-1].ref[2],
        'height_err_mean_m': statistics.fmean(dh),
        'height_err_std_m': statistics.pstdev(dh),
        'height_err_rms_m': _rms(dh),
        'xy_dev_rms_m': _rms(dxy),
        'xy_dev_max_m': max(dxy),
        'yaw_err_rms_deg': math.degrees(_rms(dpsi)),
        'throttle_mean': statistics.fmean(thr),
        'saturated_fraction': sum(1 for s in hold if s.ctrl_flags & FLAG_SATURATED) / len(hold),
    }


def format_summary(r: Dict[str, float]) -> str:
    """Resumo em uma linha para o terminal."""
    if not r.get('hold_samples'):
        return 'sem passos em HOLD: nada a resumir'
    return (
        f'HOLD {r["hold_s"]:.1f} s a {r["height_ref_m"] * 100:.0f} cm: '
        f'altura {r["height_err_mean_m"] * 100:+.1f} ± {r["height_err_std_m"] * 100:.1f} cm, '
        f'XY rms {r["xy_dev_rms_m"] * 100:.1f} cm (máx {r["xy_dev_max_m"] * 100:.1f}), '
        f'guinada rms {r["yaw_err_rms_deg"]:.1f}°, motores {r["throttle_mean"] * 100:.0f} %, '
        f'saturado {r["saturated_fraction"] * 100:.0f} % do tempo'
    )


def run_hover(
    link: DroneLink,
    height_cm: int,
    hold_s: float,
    root: Path = DEFAULT_ROOT,
    out: Callable[[str], None] = print,
    record: bool = True,
) -> Dict[str, float]:
    """Decola, segura @p hold_s s, pousa; grava e devolve o resumo.

    Qualquer erro (recusa, trecho sem fim, Ctrl+C) termina em pouso.
    """
    if not MIN_HEIGHT_CM <= height_cm <= MAX_HEIGHT_CM:
        raise ValueError(f'altura de {MIN_HEIGHT_CM} a {MAX_HEIGHT_CM} cm')
    folder = root / datetime.now().strftime('hover_%Y%m%d_%H%M%S') if record else None
    texts: List[str] = []
    t0 = time.monotonic()

    def say(text: str) -> None:
        texts.append(f'{time.monotonic() - t0:8.3f}  {text}')
        recorder.note(text)
        out(text)

    recorder = Recorder(link, folder / 'telemetry.csv' if folder else None)
    previous_on_text = link.on_text
    link.on_text = lambda text: say(f'[STM32] {text}')
    link.subscribe(recorder)
    mission = Mission(link, out=say)
    start_v = link.battery_v
    ok = False
    try:
        mission.run(parse(f'takeoff {height_cm}'))
        say(f'segurando {hold_s:.0f} s')
        end = time.monotonic() + hold_s
        while time.monotonic() < end:
            if not link.link_ok:
                raise LinkError('enlace com o STM32 perdido')
            time.sleep(min(1.0, max(0.0, end - time.monotonic())))
            say(mission.summary())
        ok = True
    except (LinkError, ValueError) as exc:
        say(f'erro: {exc}; pousando')
    except KeyboardInterrupt:
        mission.kill()
    finally:
        try:
            if not mission.killed:
                mission.land()
        finally:
            link.unsubscribe(recorder)
            recorder.close()
            link.on_text = previous_on_text
    result = analyze(recorder.samples)
    say(format_summary(result))
    if folder is not None:
        info = {
            'completed': ok,
            'height_cm': height_cm,
            'hold_s': hold_s,
            'boot_id': link.boot_id,
            'protocol': list(link.protocol),
            'simulated': link.simulated,
            'battery_start_v': start_v,
            'battery_end_v': link.battery_v,
            'rx_gaps': link.rx_gaps,
            'fc_link_errors': link.fc_link_errors,
            'summary': result,
        }
        (folder / 'summary.json').write_text(json.dumps(info, indent=2), encoding='utf-8')
        out(f'gravado em {folder}')
    result['completed'] = ok
    return result
