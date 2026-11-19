"""Tensor de inércia e centro de massa do drone montado, por corpos compostos (Steiner).

Entrada: um JSON com os componentes (veja ``inertia_components.example.json``). Referencial FRD
do corpo, origem no centro geométrico dos quatro motores: x para a frente, y para a direita,
z para baixo; posições em mm, massas em g. Cada componente é uma caixa homogênea
(``dims_mm`` = [x, y, z]), uma massa pontual (``shape: "point"``) ou um aro fino no plano xy
(``shape: "ring"``, ``radius_mm``) ou uma peça de CAD (``shape: "tensor"``, ``inertia_kgm2``:
tensor 3x3 em torno do próprio CG, nos eixos do corpo). Campos ``null`` impedem o cálculo: nada é estimado por
suposição.

Uso::

    cd stm32
    conda run -n aeroroute python tools/inertia_calc.py tools/inertia_components.json
    conda run -n aeroroute python tools/inertia_calc.py tools/inertia_components.json --write-params p.json

A saída é o tensor em torno do CG (kg·m²), o CG em relação ao centro dos motores, os eixos
principais e o empuxo de hover de cada motor com esse CG. ``--write-params`` grava o JSON que
``lqr_design.py emit --physics --params`` lê.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lqr_design


def load(path: Path) -> list:
    """Lista de componentes; falha se algum campo necessário for nulo."""
    comps = json.loads(path.read_text())['components']
    missing = []
    for c in comps:
        need = [c.get('mass_g'), *(c.get('pos_mm') or [None])]
        if c.get('shape', 'box') == 'box':
            need += c.get('dims_mm') or [None]
        if c.get('shape') == 'ring':
            need.append(c.get('radius_mm'))
        if c.get('shape') == 'tensor':
            need += np.array(c.get('inertia_kgm2') or [[None]]).flatten().tolist()
        if any(v is None for v in need):
            missing.append(c['name'])
    if missing:
        raise SystemExit('Preencha massa, posição e dimensões de: ' + ', '.join(missing))
    return comps


def own_inertia(c: dict) -> np.ndarray:
    """Inércia do componente em torno do próprio CG, nos eixos do corpo (kg·m²)."""
    m = c['mass_g'] / 1000.0
    shape = c.get('shape', 'box')
    if shape == 'point':
        return np.zeros((3, 3))
    if shape == 'tensor':
        return np.array(c['inertia_kgm2'], dtype=float)
    if shape == 'ring':
        r = c['radius_mm'] / 1000.0
        return m * r**2 * np.diag([0.5, 0.5, 1.0])
    ax, by, cz = (d / 1000.0 for d in c['dims_mm'])
    return m / 12.0 * np.diag([by**2 + cz**2, ax**2 + cz**2, ax**2 + by**2])


def compose(comps: list) -> dict:
    """Massa, CG e tensor em torno do CG pelo teorema dos eixos paralelos."""
    m = np.array([c['mass_g'] for c in comps], dtype=float) / 1000.0
    r = np.array([c['pos_mm'] for c in comps], dtype=float) / 1000.0
    mass = float(m.sum())
    cg = (m[:, None] * r).sum(axis=0) / mass
    tensor = np.zeros((3, 3))
    for ci, mi, ri in zip(comps, m, r):
        d = ri - cg
        tensor += own_inertia(ci) + mi * ((d @ d) * np.eye(3) - np.outer(d, d))
    return {'mass_kg': mass, 'cg_m': cg, 'tensor': tensor}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('components', type=Path)
    ap.add_argument(
        '--write-params', type=Path, help='grava o JSON de parâmetros para lqr_design.py'
    )
    args = ap.parse_args(argv)
    comps = load(args.components)
    res = compose(comps)
    t = res['tensor']
    print(f'Massa total: {res["mass_kg"] * 1000:.1f} g')
    print('CG [x y z] em relação ao centro dos motores (mm):', np.round(res['cg_m'] * 1000, 2))
    print('Tensor de inércia em torno do CG (kg·m²):')
    print(np.array2string(t, precision=6, suppress_small=False))
    w = np.linalg.eigvalsh(t)
    print('Inércias principais (kg·m²):', np.array2string(w, precision=6))
    off = np.abs(t - np.diag(np.diag(t))).max() / np.diag(t).max()
    print(f'Maior produto de inércia / maior diagonal: {off:.3f}  (> 0,1: usar o tensor completo)')
    body = res['mass_kg'] - 4 * lqr_design.Params().motor_kg
    p = lqr_design.Params(
        body_kg=body,
        inertia_tensor=tuple(t.flatten()),
        cg_m=(float(res['cg_m'][0]), float(res['cg_m'][1])),
    )
    try:
        trim = p.hover_trim_permille()
        print('Empuxo de hover por motor u1..u4 (N):', np.round(p.thrust_hover, 4))
        print('Trim por motor em relação à média (‰, curva provisória):', np.round(trim, 1))
    except ValueError as e:
        print('Hover:', e)
    if args.write_params:
        args.write_params.write_text(
            json.dumps(
                {
                    'body_kg': body,
                    'inertia_tensor': t.flatten().tolist(),
                    'cg_m': [float(res['cg_m'][0]), float(res['cg_m'][1])],
                },
                indent=2,
            )
        )
        print('escrito', args.write_params)
    return 0


if __name__ == '__main__':
    sys.exit(main())
