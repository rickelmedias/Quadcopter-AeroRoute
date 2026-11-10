"""Análise de um ensaio de hover gravado pela Pi (``hover-test``).

Lê as amostras do DRONE_CTRL_DEBUG (um passo do LQG cada) e as mensagens do
voo (STATUSTEXT) e responde, em números, o que importa para ajustar o hover:

- **fases**: armou, decolou, saiu do chão, chegou na altura, pousou;
- **altura** na janela de hover: erro médio, RMS e máximo em relação ao alvo;
- **XY** (posição estimada pelo Kalman a partir do fluxo do MTF, relativa ao
  ponto de decolagem): deslocamento médio, RMS, máximo e deriva;
- **atitude**: φ e θ médios (o nível que segura o drone) e desvio padrão;
  guinada: quanto girou;
- **motores**: aceleração média por motor, saturação e o hover equivalente;
- **oscilação**: frequência e amplitude dominantes de φ, θ, altura e XY;
- **perturbação estimada** (``dist z= th= phi= psi=``) no fim do voo.

E sugere o próximo voo:

- ``CTL_W0`` = CTL_W0 + d̂ de altura (o hover que o estimador achou);
- ``CTL_FF_TH/PHI/PSI`` = fixo + d̂ de cada canal (o desequilíbrio todo);
- ``SNS_LVL_PHI/TH`` = atual − φ/θ médio num hover parado. Parado no ar, sem
  acelerar, o drone está nivelado de verdade: a inclinação medida nesse
  momento é o erro do zero de nível, e é ela que deixa o drone fora do ponto
  (sem integrador de posição, 0,5° ≈ 14 cm).

Nada aqui fala com o drone: serve também para reanalisar uma pasta antiga
(``dronectl hover-report <pasta>``).
"""
from __future__ import annotations

import csv
import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

#: Estados do voo (DRONE_FLIGHT_STATE).
ARMED, TAKEOFF, HOLD, LEG, LAND, STANDBY, FAILSAFE = 4, 5, 6, 7, 8, 2, 9
#: Bits de meas_flags e ctrl_flags (DRONE_MEAS_FLAG, DRONE_CTRL_FLAG).
MEAS_RANGE, MEAS_FLOW = 4, 8
CTRL_SATURATED = 8
#: Entrada do modelo (u1..u4) -> motor físico (control/mixer.h).
MOTOR_OF_INPUT = (4, 2, 1, 3)
#: Altura que conta como "saiu do chão", em m (FLIGHT_LIFTED_HEIGHT_M).
LIFTED_M = 0.03
#: Distância do alvo que conta como "chegou", em m.
REACHED_M = 0.03
#: Depois de chegar, tempo de acomodação antes da janela de hover, em s.
SETTLE_S = 2.0
#: Menor janela de hover para sugerir hover e desequilíbrios, em s.
MIN_WINDOW_S = 5.0
#: Velocidade horizontal abaixo da qual o drone conta como parado (nível), em m/s.
STILL_SPEED = 0.05
#: Maior passo de correção de nível sugerido por voo, em graus.
LEVEL_STEP_MAX_DEG = 1.5
#: Limite do parâmetro de nível (SNS_LVL_*), em graus.
LEVEL_LIMIT_DEG = 5.0
#: Relato do estimador de perturbação no texto do voo.
DIST_RE = re.compile(r'dist z=(-?\d+) th=(-?\d+) phi=(-?\d+)(?: psi=(-?\d+))?')
#: Radianos para graus.
DEG = 180.0 / math.pi


@dataclass
class Sample:
    """Um passo do LQG gravado (unidades SI; ``t`` em s do relógio do STM32)."""

    t: float
    state: int
    meas: int
    flags: int
    y: List[float]
    x: List[float]
    du: List[float]
    ref: List[float]
    thr: List[float]
    t_pi: float = 0.0


@dataclass
class Report:
    """Resultado da análise."""

    metrics: Dict = field(default_factory=dict)
    suggestion: Dict[str, float] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)
    timeline: List[str] = field(default_factory=list)

    def text(self) -> str:
        return format_report(self)


# ---- leitura -------------------------------------------------------------------

CSV_HEADER = (['t_pi', 't_ms', 'state', 'meas', 'flags'] + [f'y{i}' for i in range(9)] +
              [f'x{i}' for i in range(12)] + [f'du{i}' for i in range(4)] +
              [f'ref{i}' for i in range(4)] + [f'thr{i}' for i in range(4)])


def sample_row(t_pi: float, s) -> List:
    """Linha do CSV a partir de um DebugSample (dronectl.link)."""
    return ([f'{t_pi:.4f}', s.time_ms, s.state, s.meas_flags, s.ctrl_flags] +
            [f'{v:.6g}' for v in s.y] + [f'{v:.6g}' for v in s.x_hat] +
            [f'{v:.6g}' for v in s.du] + [f'{v:.6g}' for v in s.ref] +
            [f'{v:.4f}' for v in s.throttle])


def load_samples(path: Path) -> List[Sample]:
    out: List[Sample] = []
    with open(path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            num = {k: float(v) for k, v in row.items()}
            out.append(Sample(t=num['t_ms'] / 1000.0, state=int(num['state']),
                              meas=int(num['meas']), flags=int(num['flags']),
                              y=[num[f'y{i}'] for i in range(9)],
                              x=[num[f'x{i}'] for i in range(12)],
                              du=[num[f'du{i}'] for i in range(4)],
                              ref=[num[f'ref{i}'] for i in range(4)],
                              thr=[num[f'thr{i}'] for i in range(4)], t_pi=num['t_pi']))
    return out


def load_texts(path: Path) -> List[Tuple[float, str]]:
    out = []
    if path.exists():
        for line in path.read_text(encoding='utf-8').splitlines():
            t, _, text = line.partition('\t')
            try:
                out.append((float(t), text))
            except ValueError:
                continue
    return out


# ---- contas --------------------------------------------------------------------

def _mean(v: Sequence[float]) -> float:
    return sum(v) / len(v) if v else float('nan')


def _rms(v: Sequence[float]) -> float:
    return math.sqrt(sum(a * a for a in v) / len(v)) if v else float('nan')


def _std(v: Sequence[float]) -> float:
    if len(v) < 2:
        return float('nan')
    m = _mean(v)
    return math.sqrt(sum((a - m) ** 2 for a in v) / (len(v) - 1))


def _slope(t: Sequence[float], v: Sequence[float]) -> float:
    """Inclinação da reta de mínimos quadrados (unidade de v por s)."""
    if len(t) < 3:
        return float('nan')
    mt, mv = _mean(t), _mean(v)
    den = sum((a - mt) ** 2 for a in t)
    return sum((a - mt) * (b - mv) for a, b in zip(t, v)) / den if den > 0 else float('nan')


def _unwrap(angles: Sequence[float]) -> List[float]:
    out, offset, prev = [], 0.0, None
    for a in angles:
        if prev is not None:
            d = a - prev
            if d > math.pi:
                offset -= 2 * math.pi
            elif d < -math.pi:
                offset += 2 * math.pi
        out.append(a + offset)
        prev = a
    return out


def dominant_oscillation(t: Sequence[float], v: Sequence[float]) -> Tuple[float, float]:
    """(frequência em Hz, amplitude RMS) do pico do espectro, sem a tendência.

    Usa numpy se houver; sem ele, devolve (nan, desvio padrão).
    """
    if len(v) < 32:
        return float('nan'), _std(v)
    try:
        import numpy as np
    except ImportError:  # pragma: no cover - a Pi tem numpy
        return float('nan'), _std(v)
    tt = np.asarray(t, dtype=float)
    vv = np.asarray(v, dtype=float)
    dt = float(np.median(np.diff(tt)))
    if not dt > 0:
        return float('nan'), float(np.std(vv))
    # Reamostra em passo fixo (pode faltar amostra no enlace) e tira a reta.
    grid = np.arange(tt[0], tt[-1], dt)
    vv = np.interp(grid, tt, vv)
    vv = vv - np.polyval(np.polyfit(grid, vv, 1), grid)
    spec = np.abs(np.fft.rfft(vv * np.hanning(len(vv))))
    freqs = np.fft.rfftfreq(len(vv), dt)
    spec[0] = 0.0
    k = int(np.argmax(spec[1:]) + 1) if len(spec) > 1 else 0
    return float(freqs[k]), float(np.std(vv))


def parse_dist(texts: Sequence[Tuple[float, str]]) -> Optional[Dict[str, float]]:
    """Último relato ``dist z= th= phi= psi=`` (em rad/s por motor)."""
    last = None
    for _, text in texts:
        m = DIST_RE.search(text)
        if m:
            last = {'z': float(m.group(1)), 'theta': float(m.group(2)),
                    'phi': float(m.group(3)),
                    'psi': float(m.group(4)) if m.group(4) is not None else 0.0}
    return last


# ---- análise -------------------------------------------------------------------

def analyze(samples: List[Sample], texts: Sequence[Tuple[float, str]], meta: Dict) -> Report:
    """Analisa um ensaio. ``meta`` traz ``height_m`` e os parâmetros usados (``params``)."""
    rep = Report()
    params = meta.get('params', {})
    target = float(meta.get('height_m', 0.3))
    rep.metrics['target_cm'] = round(target * 100, 1)
    if not samples:
        rep.notes.append('nenhuma amostra do DRONE_CTRL_DEBUG: confira DBG_RATE_HZ e o enlace')
        return rep

    # Chão: o que a Pi mediu antes de armar (média de 1 s) ou, numa gravação
    # sem ele, z medido armado no chão ou antes de armar.
    ground_pool = [s.y[0] for s in samples if s.state == ARMED and s.meas & MEAS_RANGE]
    if len(ground_pool) < 5:
        ground_pool = [s.y[0] for s in samples if s.state == STANDBY and s.meas & MEAS_RANGE][:50]
    if meta.get('ground_y_z') is not None:
        ground = float(meta['ground_y_z'])
    else:
        ground = _mean(ground_pool) if ground_pool else samples[0].y[0]
    for s in samples:
        s.h_meas = ground - s.y[0]      # type: ignore[attr-defined]
        s.h_est = ground - s.x[2]       # type: ignore[attr-defined]
        s.h_ref = ground - s.ref[2]     # type: ignore[attr-defined]

    def first(cond, start: float = -1e9) -> Optional[float]:
        for s in samples:
            if s.t >= start and cond(s):
                return s.t
        return None

    t0 = samples[0].t
    t_arm = first(lambda s: s.state >= ARMED and s.state != FAILSAFE)
    t_takeoff = first(lambda s: s.state == TAKEOFF)
    t_lift = first(lambda s: s.h_est >= LIFTED_M and s.state in (TAKEOFF, HOLD, LEG),
                   t_takeoff or t0)
    t_reach = first(lambda s: abs(s.h_est - target) <= REACHED_M and s.state in (TAKEOFF, HOLD, LEG),
                    t_lift or t0) if t_lift is not None else None
    t_land = first(lambda s: s.state == LAND, t_takeoff or t0)
    t_failsafe = first(lambda s: s.state == FAILSAFE, t_arm or t0)
    t_stop = min(v for v in (t_land, t_failsafe, samples[-1].t) if v is not None)

    def rel(v: Optional[float]) -> Optional[float]:
        return None if v is None or t_takeoff is None else round(v - t_takeoff, 2)

    # Saúde do enlace: amostras que faltaram pelo intervalo entre elas.
    dts = sorted(b.t - a.t for a, b in zip(samples, samples[1:]) if b.t > a.t)
    if dts:
        step = dts[len(dts) // 2]
        lost = sum(round((b.t - a.t) / step) - 1
                   for a, b in zip(samples, samples[1:]) if b.t - a.t > 1.5 * step)
        rep.metrics['link'] = {'samples': len(samples), 'rate_hz': round(1.0 / step, 1),
                               'lost_pct': round(100.0 * lost / (len(samples) + lost), 1)}
    rep.metrics['phases_s'] = {'arm': rel(t_arm), 'takeoff': 0.0 if t_takeoff else None,
                               'lifted': rel(t_lift), 'reached': rel(t_reach),
                               'land': rel(t_land), 'failsafe': rel(t_failsafe)}
    rep.metrics['max_height_cm'] = round(max(s.h_meas for s in samples) * 100, 1)
    if t_takeoff is None:
        rep.notes.append('não decolou (sem estado TAKEOFF)')
    elif t_lift is None:
        rep.notes.append('não saiu do chão: hover (CTL_W0) baixo, motor parado ou ensaio a seco')
    elif t_reach is None:
        rep.notes.append(f'não chegou a {target * 100:.0f} cm (máximo {rep.metrics["max_height_cm"]} cm)')
    if t_failsafe is not None:
        reasons = [txt for _, txt in texts if txt.startswith('FAILSAFE')]
        rep.notes.append('FAILSAFE' + (f': {reasons[-1]}' if reasons else ''))

    # Janela de hover: da chegada (mais a acomodação) até o pouso.
    if t_reach is not None:
        w0 = t_reach + SETTLE_S
    elif t_lift is not None:
        w0 = t_lift + 3.0
    else:
        w0 = None
    win = [s for s in samples if w0 is not None and w0 <= s.t < t_stop and
           s.state in (TAKEOFF, HOLD, LEG)]
    window_s = (win[-1].t - win[0].t) if len(win) > 1 else 0.0
    rep.metrics['window_s'] = round(window_s, 1)

    if len(win) > 10:
        t = [s.t for s in win]
        ez = [s.h_meas - s.h_ref for s in win]
        rep.metrics['height'] = {
            'mean_cm': round(_mean([s.h_meas for s in win]) * 100, 1),
            'err_mean_cm': round(_mean(ez) * 100, 1), 'err_rms_cm': round(_rms(ez) * 100, 1),
            'err_max_cm': round(max(abs(v) for v in ez) * 100, 1),
            'std_cm': round(_std([s.h_meas for s in win]) * 100, 1)}
        ex = [s.x[0] - s.ref[0] for s in win]
        ey = [s.x[1] - s.ref[1] for s in win]
        radius = [math.hypot(a, b) for a, b in zip(ex, ey)]
        rep.metrics['xy'] = {
            'mean_cm': [round(_mean(ex) * 100, 1), round(_mean(ey) * 100, 1)],
            'rms_cm': round(_rms(radius) * 100, 1), 'max_cm': round(max(radius) * 100, 1),
            'final_cm': [round(ex[-1] * 100, 1), round(ey[-1] * 100, 1)],
            'drift_cm_s': [round(_slope(t, ex) * 100, 2), round(_slope(t, ey) * 100, 2)],
            'speed_rms_cm_s': round(_rms([math.hypot(s.x[6], s.x[7]) for s in win]) * 100, 1),
            'flow_valid_pct': round(100 * _mean([1.0 if s.meas & MEAS_FLOW else 0.0
                                                 for s in win]), 1)}
        phi = [s.y[1] * DEG for s in win]
        theta = [s.y[2] * DEG for s in win]
        psi = _unwrap([s.y[3] for s in win])
        rep.metrics['attitude'] = {
            'phi_mean_deg': round(_mean(phi), 2), 'phi_std_deg': round(_std(phi), 2),
            'theta_mean_deg': round(_mean(theta), 2), 'theta_std_deg': round(_std(theta), 2),
            'psi_drift_deg': round((psi[-1] - psi[0]) * DEG, 1),
            'psi_range_deg': round((max(psi) - min(psi)) * DEG, 1),
            'rate_std_deg_s': [round(_std([s.y[i] * DEG for s in win]), 1) for i in (6, 7, 8)]}
        thr = [[s.thr[i] for s in win] for i in range(4)]
        per_input = [round(_mean(v) * 100, 1) for v in thr]
        per_motor = [0.0] * 4
        for i, motor in enumerate(MOTOR_OF_INPUT):
            per_motor[motor - 1] = per_input[i]
        w_at_0 = float(params.get('MOT_W_AT_0', 0.0))
        w_at_100 = float(params.get('MOT_W_AT_100', 1755.0))
        mean_thr = _mean([_mean(v) for v in thr])
        rep.metrics['motors'] = {
            'throttle_pct_M1_M4': per_motor,
            'throttle_max_pct': round(max(max(v) for v in thr) * 100, 1),
            'saturated_pct': round(100 * _mean([1.0 if s.flags & CTRL_SATURATED else 0.0
                                                for s in win]), 1),
            'hover_equiv_w0': round(w_at_0 + mean_thr * (w_at_100 - w_at_0))}
        osc = {}
        for name, values, unit in (('phi', phi, 'deg'), ('theta', theta, 'deg'),
                                   ('height', [v * 100 for v in ez], 'cm'),
                                   ('x', [v * 100 for v in ex], 'cm'),
                                   ('y', [v * 100 for v in ey], 'cm')):
            f, a = dominant_oscillation(t, values)
            osc[name] = {'hz': round(f, 2) if f == f else None, 'rms': round(a, 2), 'unit': unit}
        rep.metrics['oscillation'] = osc

        # Nível: média de φ e θ nos trechos parados (sem acelerar = nivelado).
        still = [s for s in win if math.hypot(s.x[6], s.x[7]) < STILL_SPEED]
        rep.metrics['still_pct'] = round(100 * len(still) / len(win), 1)
        use_xy = float(params.get('CTL_XY', 1.0)) >= 0.5
        if use_xy and window_s >= MIN_WINDOW_S and len(still) >= 0.3 * len(win):
            for key, idx, name in (('SNS_LVL_PHI', 1, 'phi'), ('SNS_LVL_TH', 2, 'theta')):
                mean_deg = _mean([s.y[idx] * DEG for s in still])
                step = max(-LEVEL_STEP_MAX_DEG, min(LEVEL_STEP_MAX_DEG, -mean_deg))
                current = float(params.get(key, 0.0))
                new = max(-LEVEL_LIMIT_DEG, min(LEVEL_LIMIT_DEG, current + step))
                if abs(step) >= 0.1:
                    rep.suggestion[key] = round(new, 2)
                rep.metrics.setdefault('level_still_mean_deg', {})[name] = round(mean_deg, 2)
        elif not use_xy:
            rep.notes.append('CTL_XY = 0: sem posição na malha, o nível não pode ser medido em voo')
    else:
        rep.notes.append('janela de hover curta demais para medir (chegou na altura?)')

    # O relato que vale é o de antes do pouso: no pouso o d̂ de altura é
    # descontado de propósito (land_bleed) e não diz mais o hover.
    t_land_pi = next((s.t_pi for s in samples if s.state == LAND and s.t_pi > 0), None)
    before_land = [tx for tx in texts if t_land_pi is None or tx[0] <= t_land_pi]
    dist = parse_dist(before_land) or parse_dist(texts)
    rep.metrics['dist_final'] = dist
    if dist is not None and window_s >= MIN_WINDOW_S:
        w0_now = float(params.get('CTL_W0', 0.0))
        if w0_now > 0:
            rep.suggestion['CTL_W0'] = round(w0_now + dist['z'])
        for key, ch in (('CTL_FF_TH', 'theta'), ('CTL_FF_PHI', 'phi'), ('CTL_FF_PSI', 'psi')):
            if key in params:
                rep.suggestion[key] = round(float(params[key]) + dist[ch])
    rep.timeline = timeline(samples, texts, target)
    return rep


def timeline(samples: Sequence[Sample], texts: Sequence[Tuple[float, str]], target: float,
             every_s: float = 1.0) -> List[str]:
    """Uma linha por segundo: estado, altura, XY, inclinação, guinada e motores."""
    names = {2: 'STANDBY', 4: 'ARMED', 5: 'TAKEOFF', 6: 'HOLD', 7: 'LEG', 8: 'LAND',
             9: 'FAILSAFE'}
    out = ['    t  estado    h(cm) alvo   x(cm)  y(cm)   φ(°)   θ(°)   ψ(°)  motores % (M1..M4)']
    t_start = next((s.t for s in samples if s.state >= ARMED), samples[0].t) if samples else 0.0
    last = -1e9
    for s in samples:
        if s.t < t_start - 2.0 or s.t - last < every_s:
            continue
        last = s.t
        motors = [0.0] * 4
        for i, m in enumerate(MOTOR_OF_INPUT):
            motors[m - 1] = s.thr[i] * 100
        out.append(f'{s.t - t_start:5.1f}  {names.get(s.state, str(s.state)):<8} '
                   f'{getattr(s, "h_meas", 0.0) * 100:5.1f} {getattr(s, "h_ref", 0.0) * 100:5.1f} '
                   f'{s.x[0] * 100:6.1f} {s.x[1] * 100:6.1f} {s.y[1] * DEG:6.1f} '
                   f'{s.y[2] * DEG:6.1f} {s.y[3] * DEG:6.1f}  '
                   + ' '.join(f'{m:4.0f}' for m in motors))
    return out


# ---- texto ---------------------------------------------------------------------

def _fmt(v, unit='') -> str:
    if v is None or (isinstance(v, float) and v != v):
        return '—'
    return f'{v}{unit}'


def format_report(rep: Report) -> str:
    m = rep.metrics
    lines = [f"alvo {m.get('target_cm')} cm · altura máxima {m.get('max_height_cm')} cm · "
             f"janela de hover {m.get('window_s')} s"]
    ph = m.get('phases_s') or {}
    lines.append('fases (s desde a decolagem): ' +
                 ', '.join(f'{k} {_fmt(v)}' for k, v in ph.items()))
    if 'height' in m:
        h = m['height']
        lines.append(f"altura: média {h['mean_cm']} cm, erro médio {h['err_mean_cm']:+} cm, "
                     f"RMS {h['err_rms_cm']} cm, máx {h['err_max_cm']} cm")
    if 'xy' in m:
        xy = m['xy']
        lines.append(f"XY: deslocamento médio ({xy['mean_cm'][0]:+}, {xy['mean_cm'][1]:+}) cm, "
                     f"RMS {xy['rms_cm']} cm, máx {xy['max_cm']} cm, final "
                     f"({xy['final_cm'][0]:+}, {xy['final_cm'][1]:+}) cm, deriva "
                     f"({xy['drift_cm_s'][0]:+}, {xy['drift_cm_s'][1]:+}) cm/s, "
                     f"fluxo válido {xy['flow_valid_pct']}%")
    if m.get('nav'):
        from .nav_report import nav_line
        line = nav_line(m['nav'])
        if line:
            lines.append(line)
    if 'attitude' in m:
        a = m['attitude']
        lines.append(f"atitude: φ {a['phi_mean_deg']:+}° ± {a['phi_std_deg']}, "
                     f"θ {a['theta_mean_deg']:+}° ± {a['theta_std_deg']}, ψ girou "
                     f"{a['psi_drift_deg']:+}° (faixa {a['psi_range_deg']}°), "
                     f"p/q/r ± {a['rate_std_deg_s']} °/s")
    if 'motors' in m:
        mo = m['motors']
        lines.append(f"motores M1..M4: {mo['throttle_pct_M1_M4']} %, máx {mo['throttle_max_pct']}%, "
                     f"saturado {mo['saturated_pct']}% do tempo, hover equivalente "
                     f"CTL_W0 ≈ {mo['hover_equiv_w0']}")
    if 'oscillation' in m:
        lines.append('oscilação dominante: ' + ', '.join(
            f"{k} {_fmt(v['hz'], ' Hz')} ({v['rms']} {v['unit']} RMS)"
            for k, v in m['oscillation'].items()))
    if m.get('level_still_mean_deg'):
        lv = m['level_still_mean_deg']
        lines.append(f"parado ({m.get('still_pct')}% da janela): φ médio {lv.get('phi')}°, "
                     f"θ médio {lv.get('theta')}° (o nível real)")
    if m.get('dist_final'):
        d = m['dist_final']
        lines.append(f"perturbação estimada no fim: z {d['z']:+.0f}, θ {d['theta']:+.0f}, "
                     f"φ {d['phi']:+.0f}, ψ {d['psi']:+.0f} rad/s")
    if m.get('link'):
        ln = m['link']
        lines.append(f"telemetria: {ln['samples']} amostras a {ln['rate_hz']} Hz, "
                     f"{ln['lost_pct']}% perdidas")
    for note in rep.notes:
        lines.append(f'! {note}')
    if rep.suggestion:
        lines.append('sugestão para o próximo voo: ' +
                     ', '.join(f'{k}={v:g}' for k, v in rep.suggestion.items()))
    return '\n'.join(lines)


def report_folder(folder: Path) -> Report:
    """Analisa uma pasta gravada por ``hover-test`` (telemetria.csv, textos.txt, meta.json)."""
    meta = json.loads((folder / 'meta.json').read_text(encoding='utf-8'))
    samples = load_samples(folder / 'telemetria.csv')
    texts = load_texts(folder / 'textos.txt')
    return analyze(samples, texts, meta)
