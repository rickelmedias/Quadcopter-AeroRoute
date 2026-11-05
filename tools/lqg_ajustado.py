"""Projeto do LQG ajustado e do estimador de perturbação (Python puro).

Gera App/Src/control/lqg_ajustado_gains.c, usado pelos modos "LQG ajustado" e
"LQG melhorado" do teste de controle (ver docs/controle_lqg.md):

- **LQG ajustado:** o mesmo projeto do TCC (modelo A, B do Simulink, ``dlqr``
  com R = 0,1 I e ``Kfd = inv(Cr (I - A + B Kd)^-1 B)``), só com outros pesos
  Q. O Kalman (L, M) continua o do Simulink.
- **Estimador de perturbação (LQG melhorado):** para cada canal (altura, φ, θ)
  um Kalman de regime com o estado de posição, o de velocidade e uma
  perturbação constante d nos motores, no padrão daquele canal:

      [pos; vel; d][k+1] = [A_pp A_pv -4 b_p; A_vp A_vv -4 b_v; 0 0 1] [pos; vel; d][k]
                           + [b_p; b_v; 0] U[k],      U = Σ_j padrão_j · u_j

  com o A, B do Simulink e os ruídos Qn e Rn do Kalman do TCC; só a variância
  de d (``QD_ALTURA``, ``QD_ATITUDE``) é nova. O controle soma d̂ · padrão a
  cada motor e cancela a perturbação estimada (hover errado, peso fora do
  centro, motores diferentes).

    python3 tools/lqg_ajustado.py            # grava a tabela
    python3 tools/lqg_ajustado.py --check    # confere (tests/run.sh)
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Tuple

import lqi_projeto as lp

RAIZ = Path(__file__).resolve().parents[1]
#: Tabela gerada.
SAIDA = RAIZ / "App/Src/control/lqg_ajustado_gains.c"

#: Pesos Q do TCC, na ordem dos estados (x y z φ θ ψ vx vy vz p q r).
Q_TCC = [5.0, 5.0, 500.0, 500.0, 500.0, 50.0, 5.0, 5.0, 5.0, 10.0, 10.0, 5.0]
#: Pesos Q do LQG ajustado: o TCC com altura e atitude mais firmes (05/11).
Q_AJUSTADO = [5.0, 5.0, 5000.0, 8000.0, 8000.0, 50.0, 5.0, 5.0, 50.0, 100.0, 100.0, 5.0]
#: R do TCC.
R_TCC = 0.1
#: Ruído de processo do Kalman do TCC: posições e ângulos, depois velocidades.
QN_POSICAO, QN_VELOCIDADE = 1e-5, 1e-3
#: Variância das medidas do Kalman do TCC (ordem de y: z φ θ ψ vx vy p q r).
RN = [1.15e-4, 5.0e-8, 1.94e-7, 6.04e-8, 1.28e-2, 2.2e-3, 2.06e-6, 3.24e-7, 2.38e-7]
#: Variância (por passo) da perturbação de altura, em (rad/s)²: rápida o
#: bastante para achar um hover 30 rad/s errado em poucos segundos.
QD_ALTURA = 0.1
#: Variância da perturbação de atitude: lenta, para a malha continuar estável
#: com o motor atrasando 120 ms (com 1e-2 já oscila em simulação).
QD_ATITUDE = 1e-3

#: Índices dos estados e das medidas no modelo do firmware.
X = {n: i for i, n in enumerate(lp.ESTADOS)}
Y = {n: i for i, n in enumerate(["z", "phi", "theta", "psi", "vx", "vy", "p", "q", "r"])}
#: Canais do estimador: (nome, posição, velocidade, medidas, padrão nos motores u1..u4).
CANAIS = [
    ("altura", "z", "vz", ["z"], [1, 1, 1, 1]),
    ("rolagem", "phi", "p", ["phi", "p"], [1, -1, -1, 1]),
    ("arfagem", "theta", "q", ["theta", "q"], [1, 1, -1, -1]),
]

Matriz = List[List[float]]


def ganhos_lqg(q: List[float]) -> Tuple[Matriz, Matriz]:
    """Kd = dlqr(A, B, diag(q), R) e Kfd = inv(Cr (I - A + B Kd)^-1 B), como o TCC."""
    A, B = lp.modelo()
    n = len(A)
    Kd = lp.dlqr(A, B, q, [R_TCC] * lp.NU)
    BK = lp._mul(B, Kd)
    malha = [[(1.0 if i == j else 0.0) - A[i][j] + BK[i][j] for j in range(n)] for i in range(n)]
    ganho_dc = lp._mul(lp._inv(malha), B)
    Cr = [X["x"], X["y"], X["z"], X["psi"]]
    Kfd = lp._inv([ganho_dc[i] for i in Cr])
    return Kd, Kfd


def kalman_canal(pos: str, vel: str, medidas: List[str], padrao: List[int],
                 qd: float) -> Tuple[Matriz, List[float], Matriz]:
    """A (3x3), B (3) e M (3x2) de regime de um canal do estimador."""
    A, B = lp.modelo()
    p, v = X[pos], X[vel]
    bp, bv = B[p][0] * padrao[0], B[v][0] * padrao[0]
    Ac = [[A[p][p], A[p][v], -4.0 * bp], [A[v][p], A[v][v], -4.0 * bv], [0.0, 0.0, 1.0]]
    Bc = [bp, bv, 0.0]
    C = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]][:len(medidas)]
    Qn = [QN_POSICAO, QN_VELOCIDADE, qd]
    Rn = [RN[Y[m]] for m in medidas]
    P = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
    for _ in range(100000):
        # M = P C' (C P C' + Rn)^-1;  Pf = (I - M C) P;  P = A Pf A' + Qn
        PCt = lp._mul(P, lp._t(C))
        S = lp._mul(C, PCt)
        for k, r in enumerate(Rn):
            S[k][k] += r
        M = lp._mul(PCt, lp._inv(S))
        MC = lp._mul(M, C)
        Pf = [[P[i][j] - sum(MC[i][m] * P[m][j] for m in range(3)) for j in range(3)] for i in range(3)]
        Pn = lp._mul(lp._mul(Ac, Pf), lp._t(Ac))
        for i in range(3):
            Pn[i][i] += Qn[i]
        mudou = max(abs(Pn[i][j] - P[i][j]) for i in range(3) for j in range(3))
        P = Pn
        if mudou <= 1e-15 * max(abs(v) for linha in P for v in linha):
            break
    else:
        raise ArithmeticError("Kalman do estimador não convergiu")
    M = [linha + [0.0] * (2 - len(linha)) for linha in M]
    return Ac, Bc, M


def _num(v: float) -> str:
    return repr(float(v))


def _linha(valores) -> str:
    return "{ " + ", ".join(_num(v) for v in valores) + " }"


def gerar() -> str:
    """Texto de App/Src/control/lqg_ajustado_gains.c."""
    Kd, Kfd = ganhos_lqg(Q_AJUSTADO)
    pesos = " ".join(f"{n}={q:g}" for n, q in zip(lp.ESTADOS, Q_AJUSTADO))
    out = [
        "/**",
        " * @file lqg_ajustado_gains.c",
        " * @brief Ganhos do LQG ajustado e do estimador de perturbação (contrato em",
        " *        control/lqg_dist.h).",
        " *",
        " * GERADO por tools/lqg_ajustado.py; não edite à mão.",
        " *",
        f" * - LQG ajustado: dlqr do modelo do TCC com Q: {pesos}",
        f" *   e R = {R_TCC:g} I; Kfd = inv(Cr (I - A + B Kd)^-1 B). Kalman: o do Simulink.",
        f" * - Estimador: Qn = {QN_POSICAO:g} (posição), {QN_VELOCIDADE:g} (velocidade), "
        f"variância de d = {QD_ALTURA:g} (altura), {QD_ATITUDE:g} (atitude); Rn do TCC.",
        " */",
        '#include "control/lqg_dist.h"',
        "",
        "const lqg_real_t LQG_AJ_KD[LQG_NU][LQG_NX] = {",
    ]
    out += [f"  {_linha(l)},  /* u{i + 1} */" for i, l in enumerate(Kd)]
    out += ["};", "", "const lqg_real_t LQG_AJ_KFD[LQG_NU][LQG_NR] = {"]
    out += [f"  {_linha(l)},  /* u{i + 1} */" for i, l in enumerate(Kfd)]
    out += ["};", "", "const lqg_dist_channel_t LQG_DIST_CHANNELS[LQG_DIST_NC] = {"]
    for nome, pos, vel, medidas, padrao in CANAIS:
        qd = QD_ALTURA if pos == "z" else QD_ATITUDE
        Ac, Bc, M = kalman_canal(pos, vel, medidas, padrao, qd)
        ys = [f"LQG_Y_{m.upper()}" for m in medidas] + ["0"] * (2 - len(medidas))
        out += [
            f"  {{ /* {nome} */",
            f"    .pos = LQG_X_{pos.upper()}, .vel = LQG_X_{vel.upper()}, .ny = {len(medidas)}U,",
            f"    .y = {{{', '.join(ys)}}},",
            f"    .pattern = {{{', '.join(f'{p:+d}.0F' for p in padrao)}}},",
            f"    .A = {{{', '.join(_linha(l) for l in Ac)}}},",
            f"    .B = {_linha(Bc)},",
            f"    .M = {{{', '.join(_linha(l) for l in M)}}},",
            "  },",
        ]
    out += ["};", ""]
    return "\n".join(out)


def main() -> int:
    """Grava a tabela, ou com ``--check`` só confere se ela está em dia."""
    texto = gerar()
    if "--check" in sys.argv[1:]:
        if SAIDA.read_text(encoding="utf-8") != texto:
            print(f"{SAIDA} desatualizado: rode python3 tools/lqg_ajustado.py")
            return 1
        print("lqg_ajustado_gains.c confere com tools/lqg_ajustado.py")
        return 0
    SAIDA.write_text(texto, encoding="utf-8")
    print(f"gerado {SAIDA}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
