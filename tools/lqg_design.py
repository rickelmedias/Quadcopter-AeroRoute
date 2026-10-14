"""Projeto do LQG do quadricóptero e geração de App/Inc/lqg_gains.h e App/Src/lqg_gains.c.

Reproduz a modelagem do TCC (LaTeXTCC/partes/textuais/metodologias.tex e
resultados.tex): modelo linearizado no hover com 12 estados e as quatro
rotações dos motores como entradas, LQR com as mesmas Q e R, e as variâncias
medidas nos sensores como Rv do Filtro de Kalman. O que muda é só o período:
o firmware roda a malha a 400 Hz (Ts = 2,5 ms), então o modelo é discretizado
(ZOH) nesse período e os ganhos são recalculados para ele.

Uso (no PC, só precisa de numpy):

    python tools/lqg_design.py              # gera os arquivos e imprime os polos
    python tools/lqg_design.py --ts 0.005   # outro período

Equivalente no MATLAB, para conferir:  sys = ss(A,B,eye(12),0);
sysd = c2d(sys,Ts);  K = dlqr(sysd.A, sysd.B, Q, R).
"""
import argparse
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

# ---- Parâmetros do TCC (Quadros de parâmetros da metodologia) ----------------
M = 0.313        # kg
L_ARM = 0.142    # m, centro de massa ao motor
IXX = 0.0011     # kg.m^2
IYY = 0.0011
IZZ = 0.0022
KT = 3.6e-5      # empuxo:  T = KT * w^2
KQ = 1.2e-5      # torque:  tau = KQ * w^2
G = 9.81

# ---- Pesos do LQR (resultados.tex, eq. valores_Q_R_sim) ----------------------
Q_LQR = [5, 5, 500, 500, 500, 50, 5, 5, 5, 10, 10, 5]
R_LQR = [0.1, 0.1, 0.1, 0.1]

# ---- Ruído de medição Rv (resultados.tex, Quadro estatistica_sensores) -------
# Ordem das saídas: z, vx, vy, phi, theta, psi, p, q, r
RV = [1.15e-4, 1.28e-2, 2.20e-3, 5.00e-8, 1.94e-7, 6.04e-8, 2.06e-6, 3.24e-7, 2.38e-7]
Y_STATE = [2, 6, 7, 3, 4, 5, 9, 10, 11]
# Teste de inovação, em desvios padrão (0 = desligado). Só nas medidas do MTF,
# que podem ter saltos; o BNO entra sempre.
GATE = [6, 6, 6, 0, 0, 0, 0, 0, 0]

# ---- Ruído de processo Qw ----------------------------------------------------
# O TCC ajustou Qw em simulação sem publicar os valores. Estes são pontos de
# partida em intensidade contínua (unidade de estado^2 por segundo); a
# discretização usa Qd = Qc * Ts. Ajuste aqui e gere de novo.
QC = [1e-4, 1e-4, 1e-4,       # x, y, z
      1e-4, 1e-4, 1e-4,       # phi, theta, psi
      0.25, 0.25, 0.25,       # vx, vy, vz   (~0,5 m/s^2 de aceleração não modelada)
      4.0, 4.0, 4.0]          # p, q, r      (~2 rad/s^2)
P0 = [1e-2] * 12
P_MAX = [100.0, 100.0] + [1e3] * 10   # x e y não são observáveis
ANGLE_STATES = [3, 4, 5]

# Saturação das rotações, em múltiplos de w0 (ajustar com a curva medida).
U_MIN_FACTOR = 0.3
U_MAX_FACTOR = 2.0


def model():
    """Matrizes A e B contínuas do TCC (eq. matriz_A_linear e matriz_B_linear)."""
    w0 = np.sqrt(M * G / (4 * KT))
    A = np.zeros((12, 12))
    A[0:6, 6:12] = np.eye(6)
    A[6, 4] = -G
    A[7, 3] = G
    b_z = 2 * KT * w0 / M
    b_r = np.sqrt(2) * L_ARM * KT * w0 / IXX
    b_p = np.sqrt(2) * L_ARM * KT * w0 / IYY
    b_y = 2 * KQ * w0 / IZZ
    B = np.zeros((12, 4))
    B[8] = [-b_z, -b_z, -b_z, -b_z]
    B[9] = [b_r, -b_r, -b_r, b_r]
    B[10] = [b_p, b_p, -b_p, -b_p]
    B[11] = [-b_y, b_y, -b_y, b_y]
    return A, B, w0


def expm(X):
    """Exponencial de matriz por escalonamento e quadratura (Taylor)."""
    norm = np.linalg.norm(X, 1)
    s = max(0, int(np.ceil(np.log2(norm))) + 1) if norm > 0 else 0
    Y = X / (2 ** s)
    E = np.eye(X.shape[0])
    term = np.eye(X.shape[0])
    for k in range(1, 20):
        term = term @ Y / k
        E = E + term
    for _ in range(s):
        E = E @ E
    return E


def c2d(A, B, ts):
    """Discretização com segurador de ordem zero."""
    n, m = B.shape
    big = np.zeros((n + m, n + m))
    big[:n, :n] = A
    big[:n, n:] = B
    E = expm(big * ts)
    return E[:n, :n], E[:n, n:]


def dare(A, B, Q, R, iters=80):
    """Equação de Riccati discreta pelo algoritmo de duplicação (SDA)."""
    Ak = A.copy()
    Gk = B @ np.linalg.solve(R, B.T)
    Hk = Q.copy()
    I = np.eye(A.shape[0])
    for _ in range(iters):
        W = I + Gk @ Hk
        Winv = np.linalg.inv(W)
        A1 = Ak @ Winv @ Ak
        G1 = Gk + Ak @ Winv @ Gk @ Ak.T
        H1 = Hk + Ak.T @ Hk @ Winv @ Ak
        done = np.linalg.norm(H1 - Hk) <= 1e-12 * max(1.0, np.linalg.norm(H1))
        Ak, Gk, Hk = A1, G1, H1
        if done:
            break
    return Hk


def c_array(name, data, ctype='float'):
    data = np.atleast_1d(np.asarray(data))
    def fmt(v):
        return f'{int(v)}' if ctype != 'float' else f'{float(v):.9e}F'
    if data.ndim == 1:
        body = ', '.join(fmt(v) for v in data)
        return f'const {ctype} {name}[{data.shape[0]}] = {{{body}}};\n'
    rows = ',\n  '.join('{' + ', '.join(fmt(v) for v in row) + '}' for row in data)
    return f'const {ctype} {name}[{data.shape[0]}][{data.shape[1]}] = {{\n  {rows}\n}};\n'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ts', type=float, default=0.0025, help='período da malha, em s')
    args = parser.parse_args()
    ts = args.ts

    A, B, w0 = model()
    Ad, Bd = c2d(A, B, ts)
    C = np.zeros((len(Y_STATE), 12))
    for i, s in enumerate(Y_STATE):
        C[i, s] = 1.0

    Q = np.diag(Q_LQR).astype(float)
    R = np.diag(R_LQR).astype(float)
    P = dare(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)

    ctrb = np.hstack([np.linalg.matrix_power(A, k) @ B for k in range(12)])
    obsv = np.vstack([C @ np.linalg.matrix_power(A, k) for k in range(12)])
    poles = np.linalg.eigvals(Ad - Bd @ K)
    print(f'w0 = {w0:.2f} rad/s, Ts = {ts*1000:.2f} ms')
    print(f'posto da controlabilidade = {np.linalg.matrix_rank(ctrb)}, '
          f'da observabilidade = {np.linalg.matrix_rank(obsv)} (x e y não observáveis)')
    print(f'maior |polo| de Ad - Bd K = {np.max(np.abs(poles)):.6f} (precisa ser < 1)')
    print('K (rad/s por unidade de estado):')
    np.set_printoptions(precision=3, suppress=True, linewidth=160)
    print(K)

    u0 = [w0] * 4
    header = f"""/**
 * @file lqg_gains.h
 * @brief Modelo e ganhos do LQG. GERADO por tools/lqg_design.py; não edite à mão.
 *
 * Ts = {ts*1000:.3f} ms, w0 = {w0:.4f} rad/s.
 * Estados (desvios do hover, eixos NED: x frente, y direita, z para baixo):
 *   0 x, 1 y, 2 z, 3 phi, 4 theta, 5 psi, 6 vx, 7 vy, 8 vz, 9 p, 10 q, 11 r
 * Entradas: rotação de cada motor, em rad/s (motor 1..4, ver docs/lqg_firmware.md).
 * Saídas: 0 z, 1 vx, 2 vy, 3 phi, 4 theta, 5 psi, 6 p, 7 q, 8 r.
 */
#ifndef LQG_GAINS_H
#define LQG_GAINS_H
#include <stdbool.h>

#define LQG_NX 12U
#define LQG_NU 4U
#define LQG_NY 9U
#define LQG_TS_S {ts:.9e}F

enum {{ LQG_X_X, LQG_X_Y, LQG_X_Z, LQG_X_PHI, LQG_X_THETA, LQG_X_PSI,
       LQG_X_VX, LQG_X_VY, LQG_X_VZ, LQG_X_P, LQG_X_Q, LQG_X_R }};
enum {{ LQG_Y_Z, LQG_Y_VX, LQG_Y_VY, LQG_Y_PHI, LQG_Y_THETA, LQG_Y_PSI,
       LQG_Y_P, LQG_Y_Q, LQG_Y_R }};

extern const float LQG_AD[LQG_NX][LQG_NX];
extern const float LQG_BD[LQG_NX][LQG_NU];
extern const float LQG_C[LQG_NY][LQG_NX];
extern const float LQG_K[LQG_NU][LQG_NX];
extern const float LQG_QW[LQG_NX];
extern const float LQG_RV[LQG_NY];
extern const float LQG_GATE_SIGMA[LQG_NY];
extern const float LQG_P0[LQG_NX];
extern const float LQG_P_MAX[LQG_NX];
extern const float LQG_U0[LQG_NU];
extern const float LQG_U_MIN[LQG_NU];
extern const float LQG_U_MAX[LQG_NU];
extern const bool LQG_ANGLE_STATE[LQG_NX];
#endif
"""
    source = '/* GERADO por tools/lqg_design.py; não edite à mão. Ver lqg_gains.h. */\n'
    source += '#include "lqg_gains.h"\n\n'
    source += c_array('LQG_AD', Ad) + c_array('LQG_BD', Bd) + c_array('LQG_C', C)
    source += c_array('LQG_K', K)
    source += c_array('LQG_QW', np.array(QC) * ts) + c_array('LQG_RV', RV)
    source += c_array('LQG_GATE_SIGMA', GATE)
    source += c_array('LQG_P0', P0) + c_array('LQG_P_MAX', P_MAX)
    source += c_array('LQG_U0', u0)
    source += c_array('LQG_U_MIN', [U_MIN_FACTOR * w0] * 4)
    source += c_array('LQG_U_MAX', [U_MAX_FACTOR * w0] * 4)
    flags = [1 if i in ANGLE_STATES else 0 for i in range(12)]
    source += c_array('LQG_ANGLE_STATE', flags, 'bool')
    (ROOT / 'App/Inc/lqg_gains.h').write_text(header, encoding='utf-8', newline='\n')
    (ROOT / 'App/Src/lqg_gains.c').write_text(source, encoding='utf-8', newline='\n')
    print('gerados App/Inc/lqg_gains.h e App/Src/lqg_gains.c')


if __name__ == '__main__':
    main()
