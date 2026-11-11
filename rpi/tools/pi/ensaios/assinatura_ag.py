"""Mesma rota no PC e na Raspberry Pi: assinatura SHA-256 e tempo do AG.

    python3 tools/pi/ensaios/assinatura_ag.py            # no PC
    cd ~/drone/tools/pi && ~/drone-venv/bin/python ensaios/assinatura_ag.py   # na Pi

Para cada instância, roda o algoritmo genético (parâmetros padrão, semente
2026) e imprime a ordem, o comprimento e o SHA-256 de (pontos, partida, ordem,
comprimento). Se o hash for igual nas duas máquinas, a Pi calcula exatamente a
mesma rota que o PC: o benchmark feito no PC vale para o algoritmo da Pi, e só
o tempo de cálculo muda.
"""

import hashlib
import json
import platform
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dronectl import rotas  # noqa: E402

#: (nome, pontos em cm, partida). O quadrado é o do ensaio na UART (decolagem a 31 cm).
INSTANCIAS = [
    ('quadrado 40 cm (ensaio)', [(40, 0, 31), (40, 40, 31), (0, 40, 31)], (0.0, 0.0, 31.0)),
    ('10 pontos (benchmark, semente 2)', rotas.pontos_aleatorios(10, 2), rotas.ORIGEM),
    ('30 pontos (benchmark, semente 1)', rotas.pontos_aleatorios(30, 1), rotas.ORIGEM),
]


def main() -> int:
    print(f'máquina: {platform.machine()}  Python {platform.python_version()}')
    for nome, pontos, origem in INSTANCIAS:
        t0 = time.perf_counter()
        ordem = rotas.ordenar('genetico', pontos, origem)
        dt = time.perf_counter() - t0
        custo = round(rotas.custo(ordem, pontos, origem), 3)
        texto = json.dumps(
            {
                'pontos': [list(p) for p in pontos],
                'origem': list(origem),
                'ordem': ordem,
                'custo_cm': custo,
            },
            sort_keys=True,
        )
        sha = hashlib.sha256(texto.encode()).hexdigest()
        print(
            f'{nome}: ordem {" ".join(str(i + 1) for i in ordem)}  {custo:.1f} cm  '
            f'{dt:.2f} s  sha256 {sha}'
        )
    return 0


if __name__ == '__main__':
    sys.exit(main())
