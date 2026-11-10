"""Análise do EKF de navegação (DRONE_NAV_DEBUG) de um ensaio de hover.

O STM32 manda, a ``NAV_DBG_HZ``, o que o EKF de navegação (``control/nav_ekf.h``)
estimou: posição, velocidade e viés do acelerômetro em x, y (fluxo) e z
(telêmetro filtrado), com as razões de teste e o que foi recusado. Este módulo
grava isso em ``nav_log.csv`` e compara com o LQG (``DRONE_CTRL_DEBUG``) para
responder duas perguntas:

- **O z do LQG é coerente?** O Kalman do LQG estima vz pelo modelo do empuxo; o
  EKF estima vz pela distância e pelo acelerômetro, sem depender do empuxo. Em
  voos de 08/11 o LQG via ~2,5 cm/s de subida com a altura parada.
- **Qual a qualidade do Z e do XY?** Quanto o filtro de distância e os gates
  recusaram, quanto o x, y do EKF difere do do LQG, quanto o viés vale.
"""
from __future__ import annotations

import csv
import math
import statistics
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

#: Bits do campo flags do DRONE_NAV_DEBUG (control/meas.h, meas_diag_flag_t).
NEW_FRAME, FLOW_FUSED, FLOW_REJECTED, XY_USABLE = 1, 2, 4, 8
XY_TO_LQG, RANGE_REJECTED, Z_REJECTED, Z_USABLE, Z_TO_LQG = 16, 32, 64, 128, 256

#: Estados do voo em que o drone está (ou deveria estar) no ar.
AIRBORNE_STATES = (5, 6, 7, 8)


@dataclass
class NavSample:
    """Um DRONE_NAV_DEBUG recebido."""

    time_ms: int
    flow_raw: List[float]
    flow_body: List[float]
    range_raw: float
    range_filt: float
    pos: List[float]
    vel: List[float]
    bias: List[float]
    accel: List[float]
    flow_ratio: float
    z_ratio: float
    flow_rejected: int
    z_rejected: int
    range_rejected: int
    flags: int
    quality: int

    @classmethod
    def from_msg(cls, msg) -> 'NavSample':
        """Converte a mensagem MAVLink."""
        return cls(msg.time_boot_ms, list(msg.flow_raw), list(msg.flow_body), msg.range_raw,
                   msg.range_filt, list(msg.pos), list(msg.vel), list(msg.bias),
                   list(msg.accel), msg.flow_ratio, msg.z_ratio, msg.flow_rejected,
                   msg.z_rejected, msg.range_rejected, msg.flags, msg.quality)


#: Colunas do nav_log.csv.
NAV_HEADER = ['t_pi_s', 't_stm32_ms', 'flow_raw_x_rad_s', 'flow_raw_y_rad_s',
              'flow_fwd_m_s', 'flow_right_m_s', 'range_raw_m', 'range_filt_m',
              'ekf_x_m', 'ekf_y_m', 'ekf_z_m', 'ekf_vx_m_s', 'ekf_vy_m_s', 'ekf_vz_m_s',
              'bias_x_m_s2', 'bias_y_m_s2', 'bias_z_m_s2',
              'accel_x_m_s2', 'accel_y_m_s2', 'accel_down_m_s2',
              'flow_ratio', 'z_ratio', 'flow_rejected', 'z_rejected', 'range_rejected',
              'flags', 'quality', 'xy_usable', 'xy_to_lqg', 'z_usable', 'z_to_lqg']


def nav_row(t_pi: float, n: NavSample) -> List:
    """Uma linha do nav_log.csv."""
    return ([f'{t_pi:.4f}', n.time_ms] + [f'{v:.5g}' for v in n.flow_raw + n.flow_body] +
            [f'{n.range_raw:.4f}', f'{n.range_filt:.4f}'] +
            [f'{v:.5g}' for v in n.pos + n.vel + n.bias + n.accel] +
            [f'{n.flow_ratio:.3g}', f'{n.z_ratio:.3g}', n.flow_rejected, n.z_rejected,
             n.range_rejected, n.flags, n.quality, int(bool(n.flags & XY_USABLE)),
             int(bool(n.flags & XY_TO_LQG)), int(bool(n.flags & Z_USABLE)),
             int(bool(n.flags & Z_TO_LQG))])


def save_nav_log(path: Path, samples: Sequence[Tuple[float, NavSample]]) -> None:
    """Grava o nav_log.csv (separador ';', como o real_log.csv)."""
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f, delimiter=';')
        w.writerow(NAV_HEADER)
        for t_pi, n in samples:
            w.writerow(nav_row(t_pi, n))


def _count_since_reset(values: Sequence[int]) -> int:
    """Quantas ocorrências um contador de 16 bits registrou, com zeramentos no meio.

    O firmware zera os contadores do EKF ao armar, e o EKF já roda desarmado, então
    o primeiro valor pode ser maior que o último. Conta a partir do último zeramento
    (uma queda) e trata a volta dos 16 bits como soma.
    """
    total, prev = 0, values[0]
    for v in values[1:]:
        if v >= prev:
            total += v - prev
        elif prev - v > 0x8000:  # deu a volta dos 16 bits
            total += v + 0x10000 - prev
        else:                    # zeramento: o que veio antes é de outro voo
            total = v
        prev = v
    return total


def _rms(values: Sequence[float]) -> float:
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float('nan')


def analyze_nav(nav: Sequence[NavSample], ctrl: Sequence) -> Dict:
    """Métricas do EKF de navegação, comparadas com o LQG no mesmo instante.

    Args:
        nav: amostras do DRONE_NAV_DEBUG, em ordem.
        ctrl: amostras do DRONE_CTRL_DEBUG (com ``time_ms``, ``state``, ``y`` e
            ``x_hat``), em ordem.

    Returns:
        Dicionário vazio sem amostras; senão as métricas (alturas em cm,
        velocidades em cm/s).
    """
    if not nav:
        return {}
    times = [c.time_ms for c in ctrl]

    def at(t_ms: int):
        if not ctrl:
            return None
        i = bisect_left(times, t_ms)
        best = min((j for j in (i - 1, i) if 0 <= j < len(ctrl)),
                   key=lambda j: abs(times[j] - t_ms))
        return ctrl[best] if abs(times[best] - t_ms) <= 50 else None

    pairs = [(n, at(n.time_ms)) for n in nav]
    air = [(n, c) for n, c in pairs if c is not None and c.state in AIRBORNE_STATES]
    use = air or [(n, c) for n, c in pairs if c is not None]
    out: Dict = {'samples': len(nav)}
    span_s = (nav[-1].time_ms - nav[0].time_ms) / 1000.0
    out['rate_hz'] = round((len(nav) - 1) / span_s, 1) if span_s > 0 else None
    out['airborne_samples'] = len(air)

    frames = sum(1 for n in nav if n.flags & NEW_FRAME)
    out['range_rejected'] = _count_since_reset([n.range_rejected for n in nav])
    out['z_rejected'] = _count_since_reset([n.z_rejected for n in nav])
    out['flow_rejected'] = _count_since_reset([n.flow_rejected for n in nav])
    out['xy_usable_pct'] = round(100.0 * sum(1 for n in nav if n.flags & XY_USABLE) / len(nav), 1)
    out['z_usable_pct'] = round(100.0 * sum(1 for n in nav if n.flags & Z_USABLE) / len(nav), 1)
    out['xy_to_lqg_pct'] = round(100.0 * sum(1 for n in nav if n.flags & XY_TO_LQG) / len(nav), 1)
    out['z_to_lqg_pct'] = round(100.0 * sum(1 for n in nav if n.flags & Z_TO_LQG) / len(nav), 1)
    out['new_frame_pct'] = round(100.0 * frames / len(nav), 1)
    out['bias_final_m_s2'] = [round(v, 3) for v in nav[-1].bias]

    if use:
        # z: o EKF contra a medida (y_z) e contra o Kalman do LQG (x̂_z, x̂_vz).
        dz_meas = [(n.pos[2] - c.y[0]) * 100.0 for n, c in use]
        dz_lqg = [(n.pos[2] - c.x_hat[2]) * 100.0 for n, c in use]
        vz_ekf = [-n.vel[2] * 100.0 for n, _ in use]
        vz_lqg = [-c.x_hat[8] * 100.0 for _, c in use]
        out['z_ekf_minus_meas_rms_cm'] = round(_rms(dz_meas), 2)
        out['z_ekf_minus_lqg_rms_cm'] = round(_rms(dz_lqg), 2)
        out['climb_ekf_mean_cm_s'] = round(statistics.fmean(vz_ekf), 2)
        out['climb_lqg_mean_cm_s'] = round(statistics.fmean(vz_lqg), 2)
        out['climb_lqg_minus_ekf_rms_cm_s'] = round(
            _rms([a - b for a, b in zip(vz_lqg, vz_ekf)]), 2)
        heights = [-c.y[0] * 100.0 for _, c in use]
        t = [n.time_ms / 1000.0 for n, _ in use]
        if len(t) > 2 and t[-1] > t[0]:
            mt, mh = statistics.fmean(t), statistics.fmean(heights)
            den = sum((a - mt) ** 2 for a in t)
            out['height_trend_cm_s'] = round(
                sum((a - mt) * (b - mh) for a, b in zip(t, heights)) / den, 2) if den else None
        # x, y: o EKF contra o Kalman do LQG.
        dxy = [math.hypot(n.pos[0] - c.x_hat[0], n.pos[1] - c.x_hat[1]) * 100.0 for n, c in use]
        out['xy_ekf_minus_lqg_max_cm'] = round(max(dxy), 1)
        out['xy_ekf_final_cm'] = [round(v * 100.0, 1) for v in use[-1][0].pos[:2]]
        out['flow_ratio_mean'] = round(statistics.fmean(n.flow_ratio for n, _ in use), 3)
        out['z_ratio_mean'] = round(statistics.fmean(n.z_ratio for n, _ in use), 3)
    return out


def nav_notes(m: Dict) -> List[str]:
    """Avisos legíveis a partir das métricas (vazio sem nada a dizer)."""
    notes: List[str] = []
    if not m:
        return notes
    lqg, ekf = m.get('climb_lqg_mean_cm_s'), m.get('climb_ekf_mean_cm_s')
    trend = m.get('height_trend_cm_s')
    if lqg is not None and ekf is not None and abs(lqg - ekf) > 1.0:
        notes.append(f'vz: o Kalman do LQG viu {lqg:+.1f} cm/s de subida e o EKF '
                     f'{ekf:+.1f} cm/s (altura medida {trend:+.2f} cm/s)'
                     if trend is not None else
                     f'vz: o Kalman do LQG viu {lqg:+.1f} cm/s e o EKF {ekf:+.1f} cm/s')
    rr = m.get('range_rejected', 0)
    if rr:
        notes.append(f'distância: {rr} leituras recusadas pelo filtro (saltos ou inválidas)')
    if m.get('xy_usable_pct', 100.0) < 50.0:
        notes.append(f'EKF x, y utilizável só em {m["xy_usable_pct"]:.0f}% do tempo')
    return notes


def nav_line(m: Dict) -> Optional[str]:
    """Uma linha para o relatório do terminal."""
    if not m:
        return None
    parts = [f'EKF nav: {m["samples"]} amostras']
    if 'climb_lqg_mean_cm_s' in m:
        parts.append(f'subida LQG {m["climb_lqg_mean_cm_s"]:+.1f} / EKF '
                     f'{m["climb_ekf_mean_cm_s"]:+.1f} cm/s')
        parts.append(f'z EKF-medida RMS {m["z_ekf_minus_meas_rms_cm"]:.1f} cm')
        parts.append(f'x,y EKF final ({m["xy_ekf_final_cm"][0]:+.1f}, '
                     f'{m["xy_ekf_final_cm"][1]:+.1f}) cm')
    parts.append(f'distância recusada {m["range_rejected"]}, fluxo recusado '
                 f'{m["flow_rejected"]}, xy ok {m["xy_usable_pct"]:.0f}%')
    return ' · '.join(parts)
