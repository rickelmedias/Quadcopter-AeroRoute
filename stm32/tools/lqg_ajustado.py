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

import struct
import sys
import zlib
from pathlib import Path
from typing import List, Tuple

import lqg_modelo as lp

RAIZ = Path(__file__).resolve().parents[1]
#: Tabela gerada.
SAIDA = RAIZ / "App/Src/control/lqg_ajustado_gains.c"

#: Pesos Q do TCC, na ordem dos estados (x y z φ θ ψ vx vy vz p q r).
Q_TCC = [5.0, 5.0, 500.0, 500.0, 500.0, 50.0, 5.0, 5.0, 5.0, 10.0, 10.0, 5.0]
#: Pesos Q do LQG ajustado: o TCC com altura e atitude mais firmes (05/11) e,
#: desde 06/11, posição e guinada mais firmes. Com x, y = 5 e vx, vy = 5 o
#: ganho de posição era 3 rad/s por metro: um erro de 0,5° no nível do BNO
#: deixava o drone ~40 cm fora do ponto. Com 50 e 20 ele vai a ~10 rad/s/m
#: (~14 cm por 0,5°) e 20 cm de erro voltam em ~3,4 s. Mais que isso (100/50,
#: 200/100) também endurece φ e θ (o dlqr é acoplado) e, com o motor
#: atrasando 120 ms, a atitude deixa de ser estável em simulação. ψ = 2000 e
#: r = 50 são para o modelo com a guinada corrigida (GUINADA_ALPHA): Kψ ~70 e
#: Kr ~85 rad/s; a guinada é desacoplada e não mexe nas outras colunas.
Q_AJUSTADO = [50.0, 50.0, 5000.0, 8000.0, 8000.0, 2000.0, 20.0, 20.0, 50.0, 100.0, 100.0, 50.0]
#: R do TCC.
R_TCC = 0.1
#: Ruído de processo do Kalman do TCC: posições e ângulos, depois velocidades.
QN_POSICAO, QN_VELOCIDADE = 1e-5, 1e-3
#: Variância das medidas do Kalman do TCC (ordem de y: z φ θ ψ vx vy p q r).
RN = [1.15e-4, 5.0e-8, 1.94e-7, 6.04e-8, 1.28e-2, 2.2e-3, 2.06e-6, 3.24e-7, 2.38e-7]
#: Variância das medidas do Kalman do LQG ajustado: a do TCC com vy igual a vx.
#: O TCC supunha vy 2,4 vezes menos ruidoso (σ 0,047 contra 0,113 m/s), mas nos
#: voos de 06/11 o fluxo do MTF teve o mesmo ruído nos dois eixos (0,09 m/s com
#: o sensor a menos de 13 cm, 0,15 m/s acima; o σ de vx do TCC fica no meio):
#: o Kalman seguia o ruído em vy e a posição y andava com ele. Só as linhas de
#: y e vy do Kalman mudam (M de vy por vy: 0,48 -> 0,24, igual ao de vx).
RN_AJUSTADO = RN[:5] + [RN[4]] + RN[6:]
#: Desvio da velocidade do fluxo depois da fusão com o acelerômetro (FLOW_EKF),
#: em m/s. No voo de 08/11 o filtro deixou ~0,6 cm/s de ruído amostra a
#: amostra; 3 cm/s deixa margem para o que sobra em baixa frequência e para a
#: correlação entre amostras (a saída do filtro não é ruído branco).
SIGMA_V_FUNDIDO = 0.03
#: Variância das medidas do Kalman com a velocidade fundida: a do ajustado com
#: vx e vy em SIGMA_V_FUNDIDO² (o LQG passa a confiar mais na velocidade).
RN_FUNDIDO = RN_AJUSTADO[:4] + [SIGMA_V_FUNDIDO**2] * 2 + RN_AJUSTADO[6:]
#: Variância (por passo) da perturbação de altura, em (rad/s)²: rápida o
#: bastante para achar um hover 30 rad/s errado em poucos segundos.
QD_ALTURA = 1.0
#: Variância da perturbação de atitude: lenta, para a malha continuar estável
#: com o motor atrasando 120 ms (com 1e-2 já oscila em simulação).
QD_ATITUDE = 1e-3
#: Variância da perturbação de guinada (torque constante: hélice ou motor
#: diferente). Com a autoridade real (GUINADA_ALPHA) de 0,01 a 0,04 e o motor
#: atrasando até 120 ms, um torque de 40 rad/s por motor fica com menos de 1°
#: de erro em 20 s; com 1e-2 sobram ~10° e com 1e-3, ~23°.
QD_GUINADA = 0.1

#: Autoridade real de guinada, em rad/s² por rad/s de diferença média entre os
#: pares de motores (U = média de ±u_j no padrão de r). O B do TCC dá 61,9:
#: ~2000 vezes mais que o drone. Nos voos de 06/11 a guinada respondeu com
#: ~0,013 (regressão de dr/dt nos voos, malha de guinada fraca) e a física dá
#: ~0,03 (550 g, empuxo 1,35 N por motor, torque/empuxo ~1,5 cm, Izz ~0,005
#: kg·m²). Com o B do TCC o LQR achava que 1 rad/s girava o drone e quase não
#: comandava guinada: o drone girava 60–190° sozinho. Os ganhos e o
#: estimador de guinada são projetados com 0,02 e simulados de 0,01 a 0,04.
GUINADA_ALPHA = 0.02
#: Rotação de hover real, em rad/s do painel (CTL_W0). No modelo, o efeito de
#: 1 rad/s a mais em vz é g / (2 ω0), e o B do TCC foi exportado com ω0 = 702.
#: Com o chumbo (550 g) os voos de 06/11 pediram ~1000–1100 (W0 + estimador +
#: LQR, motores a 62–69%): o drone responde ~30% menos que o modelo. Com o B do
#: TCC o Kalman via o drone "subindo" (vz estimado +5 a +29 cm/s com a altura
#: parada) e o termo de vz do LQR tirava empuxo justamente na subida.
HOVER_OMEGA0 = 1000.0
#: Índices dos estados e das medidas no modelo do firmware.
X = {n: i for i, n in enumerate(lp.ESTADOS)}
Y = {n: i for i, n in enumerate(["z", "phi", "theta", "psi", "vx", "vy", "p", "q", "r"])}
#: Canais do estimador: (nome, posição, velocidade, medidas, padrão nos motores u1..u4).
CANAIS = [
    ("altura", "z", "vz", ["z"], [1, 1, 1, 1]),
    ("rolagem", "phi", "p", ["phi", "p"], [1, -1, -1, 1]),
    ("arfagem", "theta", "q", ["theta", "q"], [1, 1, -1, -1]),
    ("guinada", "psi", "r", ["psi", "r"], [-1, 1, -1, 1]),
]

Matriz = List[List[float]]


def modelo_corrigido() -> Tuple[Matriz, Matriz]:
    """A e B do TCC com a guinada (``GUINADA_ALPHA``) e a altura (``HOVER_OMEGA0``) reais.

    A guinada é desacoplada no modelo (só ψ e r dependem do padrão de
    guinada), então a correção só muda as colunas ψ, r do LQR.
    """
    A, B = lp.modelo()
    r = X["r"]
    alvo = GUINADA_ALPHA * lp.TS / lp.NU
    escala = alvo / abs(B[r][0])
    B = [list(linha) for linha in B]
    # ψ também recebe o padrão de guinada (meio passo de integração de r).
    for linha in (X["psi"], r):
        B[linha] = [v * escala for v in B[linha]]
    # z e vz só respondem ao coletivo (os quatro motores iguais).
    for linha in (X["z"], X["vz"]):
        B[linha] = [v * _escala_altura() for v in B[linha]]
    return A, B


def _escala_altura() -> float:
    """Resposta vertical real dividida pela do B do TCC (ω0 do TCC / ``HOVER_OMEGA0``)."""
    _, B = lp.modelo()
    omega0_tcc = 9.81 / (2.0 * abs(B[X["vz"]][0]) / lp.TS)
    return omega0_tcc / HOVER_OMEGA0


def _escala_guinada() -> float:
    """Autoridade de guinada real dividida pela do B do TCC."""
    _, B = lp.modelo()
    return GUINADA_ALPHA * lp.TS / lp.NU / abs(B[X["r"]][0])


def ganhos_lqg(q: List[float], r: float = R_TCC) -> Tuple[Matriz, Matriz]:
    """Kd = dlqr(A, B, diag(q), r I) e Kfd = inv(Cr (I - A + B Kd)^-1 B), como o TCC.

    A e B são os do TCC com a guinada corrigida (``modelo_corrigido``).
    """
    A, B = modelo_corrigido()
    n = len(A)
    Kd = lp.dlqr(A, B, q, [r] * lp.NU)
    BK = lp._mul(B, Kd)
    malha = [[(1.0 if i == j else 0.0) - A[i][j] + BK[i][j] for j in range(n)] for i in range(n)]
    ganho_dc = lp._mul(lp._inv(malha), B)
    Cr = [X["x"], X["y"], X["z"], X["psi"]]
    Kfd = lp._inv([ganho_dc[i] for i in Cr])
    return Kd, Kfd


def kalman_lqg(rn: List[float]) -> Tuple[Matriz, Matriz]:
    """M e L de regime do Kalman do TCC (A, B, C, Qn do Simulink) com as variâncias ``rn``.

    É o dual do LQR: o ganho do ``dlqr(A', C', Qn, Rn)`` transposto é L, e
    M = A^-1 L (estimador corrente, sem termo cruzado). Com o Rn do TCC
    reproduz o L e o M exportados pelo Simulink (``--check``).
    """
    A, _ = lp.modelo()
    C = lp._ler_tabela(lp.MODELO_C.read_text(encoding="utf-8"), "LQG_ERT_C", len(RN), len(A))
    qn = [QN_POSICAO] * 6 + [QN_VELOCIDADE] * 6
    L = lp._t(lp.dlqr(lp._t(A), lp._t(C), qn, rn))
    return lp._mul(lp._inv(A), L), L


def conferir_kalman_tcc() -> float:
    """Maior erro relativo entre ``kalman_lqg(RN)`` e o M do Simulink."""
    M, _ = kalman_lqg(RN)
    Mt = lp._ler_tabela(lp.MODELO_C.read_text(encoding="utf-8"), ".M", len(M), len(RN))
    escala = max(abs(v) for linha in Mt for v in linha)
    return max(abs(a - b) for la_, lb in zip(M, Mt) for a, b in zip(la_, lb)) / escala


def _float32(v: float) -> float:
    return struct.unpack("<f", struct.pack("<f", v))[0]


def crc32(Kd: Matriz, Kfd: Matriz) -> int:
    """CRC do envio: zlib.crc32 de Kd e Kfd (linha por linha) em float32 little-endian."""
    valores = [_float32(v) for m in (Kd, Kfd) for linha in m for v in linha]
    return zlib.crc32(struct.pack(f"<{len(valores)}f", *valores))


def comandos(Kd: Matriz, Kfd: Matriz) -> List[str]:
    """Linhas do console: ``lq begin``, os 64 ``lq set`` e o commit com o CRC.

    Cada valor vai arredondado para float32 com 9 algarismos, que identificam
    um float32 sem ambiguidade: o strtof do STM32 chega no mesmo número do CRC.
    """
    linhas = ["lq begin"]
    for nome, m in (("kd", Kd), ("kfd", Kfd)):
        for i, linha in enumerate(m):
            for j, v in enumerate(linha):
                linhas.append(f"lq set {nome} {i} {j} {_float32(v):.9g}")
    linhas.append(f"lq commit {crc32(Kd, Kfd):08X}")
    return linhas


def kalman_canal(
    pos: str, vel: str, medidas: List[str], padrao: List[int], qd: float
) -> Tuple[Matriz, List[float], Matriz]:
    """A (3x3), B (3) e M (3x2) de regime de um canal do estimador."""
    A, B = modelo_corrigido()
    p, v = X[pos], X[vel]
    bp, bv = B[p][0] * padrao[0], B[v][0] * padrao[0]
    Ac = [[A[p][p], A[p][v], -4.0 * bp], [A[v][p], A[v][v], -4.0 * bv], [0.0, 0.0, 1.0]]
    Bc = [bp, bv, 0.0]
    C = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]][: len(medidas)]
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
        Pf = [
            [P[i][j] - sum(MC[i][m] * P[m][j] for m in range(3)) for j in range(3)]
            for i in range(3)
        ]
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
        f" *   e R = {R_TCC:g} I; Kfd = inv(Cr (I - A + B Kd)^-1 B).",
        " * - Kalman (M, L): o do TCC (Qn, Rn do Simulink) com o ruído de vy igual ao de vx.",
        f" * - Estimador: Qn = {QN_POSICAO:g} (posição), {QN_VELOCIDADE:g} (velocidade), "
        f"variância de d = {QD_ALTURA:g} (altura), {QD_ATITUDE:g} (atitude), "
        f"{QD_GUINADA:g} (guinada); Rn do TCC.",
        f" * - Guinada: B de ψ e r na autoridade real ({GUINADA_ALPHA:g} rad/s² por rad/s),",
        f" *   {_escala_guinada():.6g} vezes a do TCC; altura com ω0 = {HOVER_OMEGA0:g} rad/s",
        f" *   ({_escala_altura():.4g} vezes a resposta vertical do TCC).",
        " */",
        '#include "control/lqg_dist.h"',
        "",
        "const lqg_real_t LQG_AJ_KD[LQG_NU][LQG_NX] = {",
    ]
    out += [f"  {_linha(l)},  /* u{i + 1} */" for i, l in enumerate(Kd)]
    out += ["};", "", "const lqg_real_t LQG_AJ_KFD[LQG_NU][LQG_NR] = {"]
    out += [f"  {_linha(l)},  /* u{i + 1} */" for i, l in enumerate(Kfd)]
    M, L = kalman_lqg(RN_AJUSTADO)
    out += ["};", "", f"const float LQG_AJ_YAW_SCALE = {_num(_escala_guinada())}F;"]
    out += [f"const float LQG_AJ_Z_SCALE = {_num(_escala_altura())}F;"]
    out += ["", "const lqg_real_t LQG_AJ_M[LQG_NX][LQG_NY] = {"]
    out += [f"  {_linha(l)},  /* {n} */" for l, n in zip(M, lp.ESTADOS)]
    out += ["};", "", "const lqg_real_t LQG_AJ_L[LQG_NX][LQG_NY] = {"]
    out += [f"  {_linha(l)},  /* {n} */" for l, n in zip(L, lp.ESTADOS)]
    Mf, Lf = kalman_lqg(RN_FUNDIDO)
    out += [
        "};",
        "",
        f"/* Kalman com a velocidade fundida (FLOW_EKF): vx, vy com desvio de {SIGMA_V_FUNDIDO:g} m/s. */",
    ]
    out += ["const lqg_real_t LQG_AJ_M_FUSED[LQG_NX][LQG_NY] = {"]
    out += [f"  {_linha(l)},  /* {n} */" for l, n in zip(Mf, lp.ESTADOS)]
    out += ["};", "", "const lqg_real_t LQG_AJ_L_FUSED[LQG_NX][LQG_NY] = {"]
    out += [f"  {_linha(l)},  /* {n} */" for l, n in zip(Lf, lp.ESTADOS)]
    out += ["};", "", "const lqg_dist_channel_t LQG_DIST_CHANNELS[LQG_DIST_NC] = {"]
    for nome, pos, vel, medidas, padrao in CANAIS:
        qd = {"z": QD_ALTURA, "psi": QD_GUINADA}.get(pos, QD_ATITUDE)
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
        erro = conferir_kalman_tcc()
        if erro > 1e-9:
            print(f"kalman_lqg(RN) não reproduz o M do Simulink (erro relativo {erro:.1e})")
            return 1
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
