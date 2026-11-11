"""Gráfico de um ensaio de hover (``dronectl hover-report <pasta> --plot``).

Roda no PC (precisa de matplotlib; a Pi Zero não tem): cinco painéis com o
mesmo eixo de tempo (desde a decolagem) - altura, XY, inclinação, guinada e
motores - com as fases do voo e a janela de hover sombreadas. Grava
``grafico.png`` na pasta do ensaio.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from .hover_report import (
    ARMED,
    DEG,
    HOLD,
    LAND,
    LEG,
    MOTOR_OF_INPUT,
    SETTLE_S,
    TAKEOFF,
    analyze,
    load_samples,
    load_texts,
)

#: Cores das séries, em ordem fixa (paleta categórica validada para daltonismo).
SERIES = ('#2a78d6', '#eb6834', '#1baf7a', '#eda100')
#: Referência e textos secundários.
NEUTRAL = '#6b6a66'
#: Fundo das fases: decolagem, hover (HOLD/trecho) e pouso.
PHASE_COLORS = {TAKEOFF: '#e8eef8', HOLD: '#eef6ee', LEG: '#eef6ee', LAND: '#f8ece6'}
PHASE_NAMES = {TAKEOFF: 'decolagem', HOLD: 'HOLD', LEG: 'trecho', LAND: 'pouso'}
#: Legenda à direita do painel, fora dos dados.
LEGEND = dict(fontsize=8, loc='upper left', bbox_to_anchor=(1.005, 1.0), frameon=False)


def _spans(t, states):
    """Trechos contínuos de cada estado: [(estado, início, fim)]."""
    out = []
    start = 0
    for i in range(1, len(states) + 1):
        if i == len(states) or states[i] != states[start]:
            out.append((states[start], t[start], t[i - 1]))
            start = i
    return out


def plot_folder(folder: Path, out: Path | None = None) -> Path:
    """Desenha o ensaio gravado em ``folder``; devolve o caminho do PNG."""
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    folder = Path(folder)
    meta = json.loads((folder / 'meta.json').read_text(encoding='utf-8'))
    samples = load_samples(folder / 'telemetria.csv')
    texts = load_texts(folder / 'textos.txt')
    rep = analyze(samples, texts, meta)
    if not samples:
        raise ValueError('ensaio sem amostras')
    # Do chão pouco antes de armar até o fim.
    t_arm = next((s.t for s in samples if s.state >= ARMED), samples[0].t)
    samples = [s for s in samples if s.t >= t_arm - 2.0]

    t_takeoff = next((s.t for s in samples if s.state == TAKEOFF), samples[0].t)
    t = [s.t - t_takeoff for s in samples]
    states = [s.state for s in samples]

    fig, axes = plt.subplots(
        5,
        1,
        figsize=(11, 11.5),
        sharex=True,
        gridspec_kw={'height_ratios': [1.3, 1.1, 1, 0.8, 1.1]},
    )
    for ax in axes:
        for state, a, b in _spans(t, states):
            if state in PHASE_COLORS:
                ax.axvspan(a, b, color=PHASE_COLORS[state], lw=0, zorder=0)
        ax.grid(True, color='#dddcd8', lw=0.6)
        ax.tick_params(colors=NEUTRAL, labelsize=8)
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
        for side in ('left', 'bottom'):
            ax.spines[side].set_color('#b5b4ae')

    reached = (rep.metrics.get('phases_s') or {}).get('reached')
    if reached is not None:
        for ax in axes:
            ax.axvline(reached + SETTLE_S, color=NEUTRAL, lw=0.8, ls=':')
        axes[-1].annotate(
            'início da janela',
            (reached + SETTLE_S, 0.0),
            xycoords=('data', 'axes fraction'),
            fontsize=7,
            color=NEUTRAL,
            ha='left',
            va='bottom',
            xytext=(3, 2),
            textcoords='offset points',
        )
    for state, a, b in _spans(t, states):
        if state in PHASE_NAMES and b - a > 1.0:
            axes[0].annotate(
                PHASE_NAMES[state],
                ((a + b) / 2, 1.0),
                xycoords=('data', 'axes fraction'),
                fontsize=7,
                color=NEUTRAL,
                ha='center',
                va='bottom',
            )

    lw = 1.1
    ax = axes[0]
    ax.plot(t, [s.h_meas * 100 for s in samples], color=SERIES[0], lw=lw, label='medida (MTF)')
    ax.plot(t, [s.h_est * 100 for s in samples], color=SERIES[1], lw=lw, label='estimada (Kalman)')
    ax.plot(t, [s.h_ref * 100 for s in samples], color=NEUTRAL, lw=1.0, ls='--', label='referência')
    ax.set_ylabel('altura (cm)', fontsize=9)
    ax.legend(**LEGEND)

    ax = axes[1]
    ax.plot(t, [s.x[0] * 100 for s in samples], color=SERIES[0], lw=lw, label='x (frente)')
    ax.plot(t, [s.x[1] * 100 for s in samples], color=SERIES[1], lw=lw, label='y (direita)')
    ax.axhline(0.0, color=NEUTRAL, lw=1.0, ls='--')
    ax.set_ylabel('posição (cm)', fontsize=9)
    ax.legend(**LEGEND)

    ax = axes[2]
    ax.plot(t, [s.y[1] * DEG for s in samples], color=SERIES[0], lw=lw, label='φ (rolagem)')
    ax.plot(t, [s.y[2] * DEG for s in samples], color=SERIES[1], lw=lw, label='θ (arfagem)')
    ax.axhline(0.0, color=NEUTRAL, lw=1.0, ls='--')
    ax.set_ylabel('inclinação (°)', fontsize=9)
    ax.legend(**LEGEND)

    ax = axes[3]
    psi = [s.y[3] for s in samples]
    unwrapped = [psi[0]]
    for v in psi[1:]:
        d = (v - unwrapped[-1] + math.pi) % (2 * math.pi) - math.pi
        unwrapped.append(unwrapped[-1] + d)
    ax.plot(t, [(v - unwrapped[0]) * DEG for v in unwrapped], color=SERIES[0], lw=lw)
    ax.set_ylabel('guinada (°)', fontsize=9)

    ax = axes[4]
    # Na ordem dos motores físicos (M1..M4), não das entradas do modelo.
    order = sorted(range(4), key=lambda i: MOTOR_OF_INPUT[i])
    for k, i in enumerate(order):
        ax.plot(
            t,
            [s.thr[i] * 100 for s in samples],
            color=SERIES[k],
            lw=lw,
            label=f'M{MOTOR_OF_INPUT[i]}',
        )
    ax.set_ylabel('motores (%)', fontsize=9)
    ax.set_xlabel('tempo desde a decolagem (s)', fontsize=9)
    ax.legend(**LEGEND)

    m = rep.metrics
    title = f"{folder.name}  ·  {meta.get('mode')}  ·  alvo {m.get('target_cm')} cm"
    if meta.get('label'):
        title += f"  ·  {meta['label']}"
    sub = []
    if 'height' in m:
        sub.append(f"h RMS {m['height']['err_rms_cm']} cm")
    if 'xy' in m:
        sub.append(
            f"XY médio ({m['xy']['mean_cm'][0]:+}, {m['xy']['mean_cm'][1]:+}) cm, "
            f"RMS {m['xy']['rms_cm']} cm"
        )
    if 'attitude' in m:
        sub.append(f"φ {m['attitude']['phi_mean_deg']:+}°  θ {m['attitude']['theta_mean_deg']:+}°")
    fig.suptitle(title, fontsize=10, x=0.01, ha='left', y=0.995)
    if sub:
        fig.text(0.01, 0.975, '  ·  '.join(sub), fontsize=8.5, color=NEUTRAL, ha='left')
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out = Path(out) if out else folder / 'grafico.png'
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out
