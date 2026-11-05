"""Projeto do LQI em Python puro, igual ao de tools/lqi (sem numpy).

O painel (``controle_gui.py``) usa este módulo para testar outros pesos Q/R:
recalcula Kx, Kfd e Ki com o mesmo ``dlqr`` e monta os comandos ``lqi`` que o
console USB do STM32 aceita (control/lqi_tuning.h).

O cálculo é o de ``tools/lqi/lqi_design.c``: o subsistema de atitude e altura
(z, φ, θ, ψ, vz, p, q, r) do mesmo A, B do Simulink, aumentado com ∫(z - r_z),
∫φ e ∫θ, e ``[Kx Ki] = dlqr(Aa, Ba, diag(Q, Qi), R)`` pelo algoritmo de
duplicação. A e B são lidos de App/Src/control/lqg_ert_model.c, a mesma tabela
do firmware. Com os pesos padrão, o resultado é a tabela de
App/Src/control/lqi_gains.c (``--check`` confere).

    python3 tools/lqi_projeto.py --check
"""
from __future__ import annotations

import math
import re
import struct
import sys
import zlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List

RAIZ = Path(__file__).resolve().parents[1]
#: Tabelas do modelo do TCC no firmware.
MODELO_C = RAIZ / "App/Src/control/lqg_ert_model.c"
#: Tabela padrão do LQI no firmware.
GANHOS_C = RAIZ / "App/Src/control/lqi_gains.c"

#: Estados do modelo, na ordem do firmware.
ESTADOS = ["x", "y", "z", "phi", "theta", "psi", "vx", "vy", "vz", "p", "q", "r"]
#: Estados do subsistema de atitude e altura (índices no modelo).
SUBSISTEMA = [2, 3, 4, 5, 8, 9, 10, 11]
#: Posição de z, φ e θ dentro do subsistema (o que cada integrador integra).
INTEGRADOS = [0, 1, 2]
#: Colunas de Kx copiadas para Kfd (r_z e r_ψ); r_x e r_y ficam em zero.
KFD_DE_KX = {2: 2, 3: 5}
#: Período do controle, em s.
TS = 0.01
#: Entradas (motores do modelo).
NU = 4

Matriz = List[List[float]]


@dataclass
class Pesos:
    """Pesos do projeto, iguais a ``lqi_default_weights()`` de tools/lqi.

    São os do TCC mais os dos integradores, com a atitude mais firme
    (φ, θ = 8000 e p, q = 100 no lugar de 500 e 10) desde os ensaios de 05/11.
    """

    q: Dict[str, float] = field(default_factory=lambda: {
        "z": 500.0, "phi": 8000.0, "theta": 8000.0, "psi": 50.0,
        "vz": 5.0, "p": 100.0, "q": 100.0, "r": 5.0})
    r: float = 0.1
    q_int_z: float = 1000.0
    q_int_phi: float = 10000.0
    q_int_theta: float = 10000.0

    def resumo(self) -> str:
        """Uma linha com todos os pesos, para o log do ensaio."""
        q = " ".join(f"{k}={v:g}" for k, v in self.q.items())
        return (f"Q: {q} | R={self.r:g} | int_z={self.q_int_z:g} "
                f"int_phi={self.q_int_phi:g} int_theta={self.q_int_theta:g}")


@dataclass
class Ganhos:
    """Kx (4 x 12), Kfd (4 x 4) e Ki (4 x 3), como em control/lqi.h."""

    Kx: Matriz
    Kfd: Matriz
    Ki: Matriz

    def valores(self) -> List[float]:
        """Os 76 valores na ordem do envio (Kx, Kfd, Ki, linha por linha)."""
        return [v for m in (self.Kx, self.Kfd, self.Ki) for linha in m for v in linha]

    def crc32(self) -> int:
        """CRC do envio: zlib.crc32 dos valores em float32 little-endian."""
        return zlib.crc32(struct.pack(f"<{len(self.valores())}f", *self.valores()))

    def resumo(self) -> str:
        """Os ganhos que mais importam, em rad/s por unidade (motor u1)."""
        k, ki = self.Kx[0], self.Ki[0]
        return (f"z {abs(k[2]):.1f} · φ/θ {abs(k[3]):.1f}/{abs(k[4]):.1f} · vz {abs(k[8]):.1f} · "
                f"p/q {abs(k[9]):.1f}/{abs(k[10]):.1f} · ∫z {abs(ki[0]):.1f} · "
                f"∫φ/∫θ {abs(ki[1]):.1f}/{abs(ki[2]):.1f}")


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

    O mesmo de tools/lqi/lqr_design.c: converge em dezenas de passos mesmo com
    os modos de integrador (autovalor 1).
    """
    n = len(A)
    Bt = _t(B)
    G = [[sum(B[i][t] * Bt[t][j] / r[t] for t in range(len(r))) for j in range(n)] for i in range(n)]
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
    numeros = [float(v) for v in re.findall(r"[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?",
                                            sem_comentarios[inicio.end():])]
    if len(numeros) < linhas * colunas:
        raise ValueError(f"{nome}: {len(numeros)} valores, esperava {linhas * colunas}")
    numeros = numeros[:linhas * colunas]
    return [numeros[i * colunas:(i + 1) * colunas] for i in range(linhas)]


def modelo() -> tuple[Matriz, Matriz]:
    """A (12 x 12) e B (12 x 4) do firmware."""
    texto = MODELO_C.read_text(encoding="utf-8")
    return _ler_tabela(texto, "LQG_ERT_A", 12, 12), _ler_tabela(texto, "LQG_ERT_B", 12, NU)


def ganhos_firmware() -> Ganhos:
    """A tabela padrão do LQI do firmware (lqi_gains.c)."""
    texto = GANHOS_C.read_text(encoding="utf-8")
    return Ganhos(Kx=_ler_tabela(texto, ".Kx", NU, 12), Kfd=_ler_tabela(texto, ".Kfd", NU, 4),
                  Ki=_ler_tabela(texto, ".Ki", NU, 3))


# --------------------------------------------------------------------------
# Projeto e envio
# --------------------------------------------------------------------------

def projetar(pesos: Pesos) -> Ganhos:
    """Kx, Kfd e Ki para os pesos dados (lqi_design.c)."""
    A, B = modelo()
    ns, ni = len(SUBSISTEMA), len(INTEGRADOS)
    na = ns + ni
    Aa = [[0.0] * na for _ in range(na)]
    Ba = [[0.0] * NU for _ in range(na)]
    for i, si in enumerate(SUBSISTEMA):
        for j, sj in enumerate(SUBSISTEMA):
            Aa[i][j] = A[si][sj]
        Ba[i] = list(B[si])
    for k, integrado in enumerate(INTEGRADOS):
        Aa[ns + k][ns + k] = 1.0
        Aa[ns + k][integrado] = TS
    q = [pesos.q[ESTADOS[s]] for s in SUBSISTEMA] + [pesos.q_int_z, pesos.q_int_phi, pesos.q_int_theta]
    if min(q) < 0.0 or not pesos.r > 0.0:
        raise ValueError("pesos de Q precisam ser >= 0 e R > 0")
    K = dlqr(Aa, Ba, q, [pesos.r] * NU)
    Kx = [[0.0] * 12 for _ in range(NU)]
    Kfd = [[0.0] * 4 for _ in range(NU)]
    Ki = [[0.0] * ni for _ in range(NU)]
    for i in range(NU):
        for j, sj in enumerate(SUBSISTEMA):
            Kx[i][sj] = K[i][j]
        for k in range(ni):
            Ki[i][k] = K[i][ns + k]
        for col_kfd, col_kx in KFD_DE_KX.items():
            Kfd[i][col_kfd] = Kx[i][col_kx]
    return Ganhos(Kx=Kx, Kfd=Kfd, Ki=Ki)


def _float32(v: float) -> float:
    return struct.unpack("<f", struct.pack("<f", v))[0]


def comandos(ganhos: Ganhos) -> List[str]:
    """Linhas do console: begin, os 76 ``lqi set`` e o commit com o CRC.

    Cada valor vai já arredondado para float32 e com 9 algarismos, que
    identificam um float32 sem ambiguidade: o strtof do STM32 chega no mesmo
    número que entrou no CRC daqui.
    """
    g32 = Ganhos(Kx=[[_float32(v) for v in l] for l in ganhos.Kx],
                 Kfd=[[_float32(v) for v in l] for l in ganhos.Kfd],
                 Ki=[[_float32(v) for v in l] for l in ganhos.Ki])
    linhas = ["lqi begin"]
    for nome, m in (("kx", g32.Kx), ("kfd", g32.Kfd), ("ki", g32.Ki)):
        for i, linha in enumerate(m):
            for j, v in enumerate(linha):
                linhas.append(f"lqi set {nome} {i} {j} {v:.9g}")
    linhas.append(f"lqi commit {g32.crc32():08X}")
    return linhas


def conferir_firmware() -> float:
    """Maior erro relativo entre o projeto com os pesos padrão e lqi_gains.c."""
    a, b = projetar(Pesos()), ganhos_firmware()
    escala = max(abs(v) for v in b.valores())
    return max(abs(x - y) for x, y in zip(a.valores(), b.valores())) / escala


def main() -> int:
    """``--check``: confere o projeto com a tabela do firmware."""
    if "--check" in sys.argv[1:]:
        erro = conferir_firmware()
        print(f"lqi_projeto.py vs lqi_gains.c: erro relativo {erro:.1e}")
        return 0 if erro < 1e-9 else 1
    g = projetar(Pesos())
    print(g.resumo())
    print(f"crc={g.crc32():08X}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
