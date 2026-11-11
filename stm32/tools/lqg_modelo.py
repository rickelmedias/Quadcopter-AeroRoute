"""Modelo do TCC e dlqr em Python puro (sem numpy), para as ferramentas do LQG.

A e B são lidos de App/Src/control/lqg_ert_model.c, a mesma tabela do
firmware (exportada do Simulink). O ``dlqr`` é o algoritmo de duplicação, que
converge em dezenas de passos; com ele, ``tools/lqg_ajustado.py`` refaz o
projeto do TCC (LQR e Kalman) com outros pesos.
"""

from __future__ import annotations

import math
import re
from pathlib import Path
from typing import List

RAIZ = Path(__file__).resolve().parents[1]
#: Tabelas do modelo do TCC no firmware.
MODELO_C = RAIZ / "App/Src/control/lqg_ert_model.c"

#: Estados do modelo, na ordem do firmware.
ESTADOS = ["x", "y", "z", "phi", "theta", "psi", "vx", "vy", "vz", "p", "q", "r"]
#: Período do controle, em s.
TS = 0.01
#: Entradas (motores do modelo).
NU = 4

Matriz = List[List[float]]


# --------------------------------------------------------------------------
# Álgebra (matrizes pequenas como listas de listas)
# --------------------------------------------------------------------------


def _mul(a: Matriz, b: Matriz) -> Matriz:
    bt = list(zip(*b))
    return [[sum(x * y for x, y in zip(linha, col)) for col in bt] for linha in a]


def _t(a: Matriz) -> Matriz:
    return [list(c) for c in zip(*a)]


def _inv(a: Matriz) -> Matriz:
    """Inversa por Gauss-Jordan com pivoteamento (como lqr_inverse)."""
    n = len(a)
    m = [list(linha) + [1.0 if i == j else 0.0 for j in range(n)] for i, linha in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-300:
            raise ArithmeticError("matriz singular")
        m[c], m[p] = m[p], m[c]
        d = m[c][c]
        m[c] = [v / d for v in m[c]]
        for r in range(n):
            if r != c and m[r][c] != 0.0:
                f = m[r][c]
                m[r] = [v - f * w for v, w in zip(m[r], m[c])]
    return [linha[n:] for linha in m]


def dlqr(A: Matriz, B: Matriz, q: List[float], r: List[float]) -> Matriz:
    """K = dlqr(A, B, diag(q), diag(r)), com u = -K x (algoritmo de duplicação).

    Converge em dezenas de passos mesmo com modos de integrador (autovalor 1).
    """
    n = len(A)
    Bt = _t(B)
    G = [
        [sum(B[i][t] * Bt[t][j] / r[t] for t in range(len(r))) for j in range(n)] for i in range(n)
    ]
    H = [[q[i] if i == j else 0.0 for j in range(n)] for i in range(n)]
    Ak = [list(linha) for linha in A]
    for _ in range(200):
        W = _mul(G, H)
        for i in range(n):
            W[i][i] += 1.0
        Wi = _inv(W)
        At = _t(Ak)
        An = _mul(Ak, _mul(Wi, Ak))
        Gn = _mul(_mul(Ak, _mul(Wi, G)), At)
        Hn = _mul(At, _mul(_mul(H, Wi), Ak))
        variacao = escala = 0.0
        for i in range(n):
            for j in range(n):
                Gn[i][j] += G[i][j]
                Hn[i][j] += H[i][j]
                variacao = max(variacao, abs(Hn[i][j] - H[i][j]))
                escala = max(escala, abs(Hn[i][j]))
        H = [[0.5 * (Hn[i][j] + Hn[j][i]) for j in range(n)] for i in range(n)]
        G = [[0.5 * (Gn[i][j] + Gn[j][i]) for j in range(n)] for i in range(n)]
        Ak = An
        if not math.isfinite(variacao):
            raise ArithmeticError("dlqr divergiu")
        if variacao <= 1e-13 * (escala if escala > 0.0 else 1.0):
            break
    else:
        raise ArithmeticError("dlqr não convergiu")
    BtX = _mul(Bt, H)
    S = _mul(BtX, B)
    for i in range(len(r)):
        S[i][i] += r[i]
    return _mul(_inv(S), _mul(BtX, A))


# --------------------------------------------------------------------------
# Tabelas do firmware
# --------------------------------------------------------------------------


def _ler_tabela(texto: str, nome: str, linhas: int, colunas: int) -> Matriz:
    """Lê os ``linhas x colunas`` números depois de ``nome =`` num .c gerado.

    Os comentários são tirados antes (eles têm números, como ``du1``).
    """
    sem_comentarios = re.sub(r"/\*.*?\*/", "", texto, flags=re.S)
    inicio = re.search(re.escape(nome) + r"[^=]*=", sem_comentarios)
    if inicio is None:
        raise ValueError(f"{nome} não encontrado")
    numeros = [
        float(v)
        for v in re.findall(r"[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?", sem_comentarios[inicio.end() :])
    ]
    if len(numeros) < linhas * colunas:
        raise ValueError(f"{nome}: {len(numeros)} valores, esperava {linhas * colunas}")
    numeros = numeros[: linhas * colunas]
    return [numeros[i * colunas : (i + 1) * colunas] for i in range(linhas)]


def modelo() -> tuple[Matriz, Matriz]:
    """A (12 x 12) e B (12 x 4) do firmware."""
    texto = MODELO_C.read_text(encoding="utf-8")
    return _ler_tabela(texto, "LQG_ERT_A", 12, 12), _ler_tabela(texto, "LQG_ERT_B", 12, NU)
