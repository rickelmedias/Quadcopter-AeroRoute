"""Margem de estabilidade da malha de atitude/altura com atraso e motor de 1ª ordem.

O Kd do LQR (``lqg_gains.h``) é projetado para a planta ideal; no voo há atraso de amostragem
e a dinâmica de motor e hélice. Este script fecha a malha com Kd × multiplicadores (os da aba
Controle, ``lqg_ctrl.c``) sobre a planta linear no hover, com ``delay`` passos de 10 ms e um
motor de 1ª ordem (``tau``), e mostra o maior módulo de polo (< 1: estável).

Uso (de ``stm32/``, ambiente ``aeroroute``)::

    conda run -n aeroroute python tools/loop_margin.py                       # multiplicadores atuais
    conda run -n aeroroute python tools/loop_margin.py --att 2 --rate 2 --z 10 --w 5

Atraso e ``tau`` reais do drone não foram medidos: a tabela varre ``tau`` 20 a 60 ms e atraso
de 1 a 3 passos, e ±20 % em kt e ±25 % na inércia no pior caso.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lqr_design as d

PARAMS = Path(__file__).resolve().parent / 'drone_params.json'
TAUS = (0.02, 0.04, 0.06)
DELAYS = (1, 2, 3)


def load_params(path: Path) -> d.Params:
    """Tensor de inércia do JSON de ``inertia_calc.py`` com o CG no centro dos motores.

    O CG do JSON veio de uma foto e foi refutado pelos voos (ver control-lqg.md): só o tensor
    vale como ordem de grandeza. Sem o arquivo, usa o modelo da submissão.
    """
    if not path.exists():
        return d.Params()
    raw = json.loads(path.read_text())
    return d.Params(inertia_tensor=tuple(raw['inertia_tensor']), body_kg=raw['body_kg'])


def scaled_gains(kd: np.ndarray, g: dict) -> np.ndarray:
    """Kd × multiplicadores por coluna, como ``ekf_control`` (xy e vel são relativos a att)."""
    s = np.array(
        [
            g['xy'] * g['att'],
            g['xy'] * g['att'],
            g['z'],
            g['att'],
            g['att'],
            g['yaw'],
            g['vel'] * g['att'],
            g['vel'] * g['att'],
            g['w'],
            g['rate'],
            g['rate'],
            g['yaw_rate'],
        ]
    )
    return kd * s[None, :]


def spectral_radius(ad: np.ndarray, bd: np.ndarray, k: np.ndarray, tau: float, delay: int) -> float:
    """Maior |polo| de x+ = A x + B·motor, com motor de 1ª ordem e ``delay`` passos de atraso."""
    n, m = bd.shape
    a = np.exp(-d.TS / tau)
    size = n + m + (m * delay)
    mat = np.zeros((size, size))
    mat[:n, :n] = ad
    mat[:n, n : n + m] = bd
    mat[n : n + m, n : n + m] = a * np.eye(m)
    q0 = n + m
    mat[n : n + m, q0 + (m * (delay - 1)) : q0 + (m * delay)] = (1.0 - a) * np.eye(m)
    mat[q0 : q0 + m, :n] = -k
    for i in range(delay - 1):
        mat[q0 + m * (i + 1) : q0 + m * (i + 2), q0 + m * i : q0 + m * (i + 1)] = np.eye(m)
    return float(np.abs(np.linalg.eigvals(mat)).max())


def worst_case(params: d.Params, k: np.ndarray) -> float:
    """Pior polo em tau, atraso, kt (±20 %) e inércia (±25 %)."""
    worst = 0.0
    for skt in (0.8, 1.0, 1.2):
        for si in (0.75, 1.0, 1.25):
            tensor = tuple((params.inertia_matrix * si).flatten())
            p = replace(params, kt=params.kt * skt, inertia_tensor=tensor)
            ad, bd = d.discretize(*d.continuous(p))
            for tau in TAUS:
                for delay in DELAYS:
                    worst = max(worst, spectral_radius(ad, bd, k, tau, delay))
    return worst


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    defaults = dict(z=10, w=5, att=5, rate=5, yaw=5, yaw_rate=5, xy=1, vel=1)
    for key, val in defaults.items():
        ap.add_argument(f'--{key.replace("_", "-")}', type=float, default=val)
    ap.add_argument('--params', default=str(PARAMS), help='JSON de inertia_calc.py (só o tensor)')
    args = ap.parse_args(argv)
    g = {key: getattr(args, key) for key in defaults}
    kd = scaled_gains(d.load_generated()['Kd'], g)
    p = load_params(Path(args.params))
    ad, bd = d.discretize(*d.continuous(p))
    print('multiplicadores:', g)
    print('polo máximo (planta nominal), linhas: atraso 1..3 passos; colunas: tau 20, 40, 60 ms')
    for delay in DELAYS:
        row = [spectral_radius(ad, bd, kd, tau, delay) for tau in TAUS]
        print(f'  atraso {delay}: ' + '  '.join(f'{v:.4f}' for v in row))
    wc = worst_case(p, kd)
    print(f'pior caso (kt ±20 %, inércia ±25 %, todo tau e atraso): {wc:.4f}')
    print('ESTÁVEL em todos os casos' if wc < 1.0 else 'INSTÁVEL em algum caso')
    return 0 if wc < 1.0 else 1


if __name__ == '__main__':
    sys.exit(main())
