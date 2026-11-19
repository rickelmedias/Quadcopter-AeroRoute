"""Projeto do LQR discreto do controle de voo, em Python (substitui o MATLAB para Kd e Kfd).

Reproduz ``references/DroneLQG_revisao/projetar_drone.m`` e o Simulink Coder da
submissão: planta linear no hover, ZOH a Ts, LQR discreto (equação de Riccati),
mapa de referência ``Kfd``. Estados ``[x y z phi theta psi u v w p q r]``, SI,
NED/FRD; entradas: Δω de cada motor em rad/s em torno do hover (u1..u4).

Uso (no PC, ambiente conda ``aeroroute``)::

    cd stm32
    conda run -n aeroroute python tools/lqr_design.py check      # reproduz os ganhos gerados
    conda run -n aeroroute python tools/lqr_design.py identify   # parâmetros do modelo gerado
    conda run -n aeroroute python tools/lqr_design.py emit       # reescreve control/lqg_gains.h

``emit`` sem opções usa o ``A`` e o ``B`` do modelo gerado e o ``Q``/``R`` da
submissão: o resultado é o mesmo ``Kd`` (erro < 1e-9). Para outro projeto, passe
``--params``, ``--q`` e ``--r`` (ver ``--help``). O firmware só lê
``control/lqg_gains.h``; trocar os ganhos nunca exige MATLAB.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import scipy.linalg as sl

#: Raiz do firmware (stm32/).
ROOT = Path(__file__).resolve().parents[1]
GENERATED = ROOT / 'App/Src/control/lqg_model/STM32_data.c'
GAINS_H = ROOT / 'App/Inc/control/lqg_gains.h'

N = 12
NU = 4
TS = 0.01
G = 9.81
#: Pesos da submissão (``projetar_drone.m``): referência de simulação, não sintonia de voo.
Q_SUBMISSION = (5, 5, 500, 500, 500, 50, 5, 5, 5, 10, 10, 5)
R_SUBMISSION = 0.1
#: Estados que a referência controla: x, y, z, psi.
REF_STATES = (0, 1, 2, 5)


#: Sinal de cada entrada u1..u4 nos eixos y (esquerda/direita), x (frente/trás) e no torque de guinada.
#: u1, u2 na frente; u1, u4 à esquerda. Mesmo mapa de B do modelo gerado (u1→M4, u2→M2, u3→M1, u4→M3).
MOTOR_SIGN_X = np.array([1.0, 1.0, -1.0, -1.0])
MOTOR_SIGN_Y = np.array([-1.0, 1.0, 1.0, -1.0])
MOTOR_SIGN_YAW = np.array([-1.0, 1.0, -1.0, 1.0])
#: Curva provisória do firmware: 1755 rad/s a 1000 ‰ (control/lqg_config.h, LQG_W_PER_PERMILLE = 1,755).
W_PER_PERMILLE = 1.755


@dataclass(frozen=True)
class Params:
    """Parâmetros físicos do drone.

    Os padrões reproduzem o modelo da submissão (Simulink Coder): corpo 455 g, motor mais
    hélice 24 g, massa total 551 g, inércia de caixa. kt e kq são candidatos da bancada
    (kq é hipótese não validada). Para a montagem real, preencher ``inertia_tensor`` e
    ``cg_m`` (ver ``tools/inertia_calc.py``).
    """

    #: Massa do corpo e de cada conjunto motor + hélice, em kg; metade da diagonal entre
    #: motores (m); caixa equivalente do corpo (m). Só alimentam a inércia por fórmula de caixa.
    body_kg: float = 0.455
    motor_kg: float = 0.024
    arm_m: float = 0.142 / 2
    box_a_m: float = 0.104
    box_b_m: float = 0.104
    box_c_m: float = 0.05
    #: Empuxo T = kt·ω² (N·s²) e torque Q = kq·ω² (N·m·s²). kq é hipótese não validada.
    kt: float = 2.0901595e-6
    kq: float = 9.5080563e-8
    #: Vazamento artificial de x e y do modelo gerado (1/s); o script de revisão usa 0.
    xy_leak: float = 0.0
    #: Tensor de inércia completo em torno do CG, eixos FRD do corpo (kg·m², 3x3);
    #: None = caixa + motores (diagonal).
    inertia_tensor: tuple | None = field(default=None)
    #: Centro de massa em relação ao centro dos quatro motores (x frente, y direita), em m.
    cg_m: tuple = (0.0, 0.0)

    @property
    def mass(self) -> float:
        return self.body_kg + 4 * self.motor_kg

    @property
    def inertia_matrix(self) -> np.ndarray:
        """Tensor de inércia 3x3 em torno do CG (kg·m²)."""
        if self.inertia_tensor is not None:
            return np.array(self.inertia_tensor, dtype=float).reshape(3, 3)
        md, mm, ll = self.body_kg, self.motor_kg, self.arm_m
        a, b, c = self.box_a_m, self.box_b_m, self.box_c_m
        return np.diag(
            [
                md * (b**2 + c**2) / 12 + 2 * mm * ll**2,
                md * (a**2 + c**2) / 12 + 2 * mm * ll**2,
                md * (a**2 + b**2) / 12 + 4 * mm * ll**2,
            ]
        )

    @property
    def ixx_iyy_izz(self) -> tuple:
        """Inércias principais da diagonal (kg·m²)."""
        return tuple(np.diag(self.inertia_matrix))

    @property
    def motor_xy(self) -> np.ndarray:
        """Posição de u1..u4 em relação ao CG (m): colunas x (frente) e y (direita)."""
        half = self.arm_m / np.sqrt(2)
        return np.stack(
            [MOTOR_SIGN_X * half - self.cg_m[0], MOTOR_SIGN_Y * half - self.cg_m[1]], axis=1
        )

    @property
    def thrust_hover(self) -> np.ndarray:
        """Empuxo de cada motor (N) que equilibra peso e momentos em torno do CG (tem de ser > 0).

        Equilíbrio de Σ T = m·g, Σ y·T = 0, Σ x·T = 0 e do torque de guinada Σ ±kq/kt·T = 0.
        Com o CG no centro dos motores sai m·g/4 em todos.
        """
        xy = self.motor_xy
        mat = np.vstack([np.ones(4), -xy[:, 1], xy[:, 0], MOTOR_SIGN_YAW * self.kq / self.kt])
        return np.linalg.solve(mat, np.array([self.mass * G, 0.0, 0.0, 0.0]))

    @property
    def w_hover(self) -> np.ndarray:
        """Velocidade de cada rotor no hover (rad/s)."""
        t = self.thrust_hover
        if np.any(t <= 0):
            raise ValueError('CG fora da área dos motores: não há equilíbrio com empuxo positivo')
        return np.sqrt(t / self.kt)

    @property
    def w0(self) -> float:
        """Velocidade média do rotor no hover (rad/s); com CG centrado, igual para todos."""
        return float(np.mean(self.w_hover))

    def hover_trim_permille(self) -> np.ndarray:
        """Diferença de cada motor para a média no hover, em ‰ (curva provisória de 1755 rad/s)."""
        w = self.w_hover
        return (w - w.mean()) / W_PER_PERMILLE


def continuous(p: Params) -> tuple:
    """A e B contínuos da planta linear no hover (guinada zero).

    A linearização usa a velocidade de hover de cada motor (``w_hover``): com o CG deslocado
    elas diferem, e o B de torque usa o braço real (posição do motor menos o CG) e a inversa do
    tensor de inércia completo (produtos de inércia acoplam os eixos).
    """
    w = p.w_hover
    xy = p.motor_xy
    a = np.zeros((N, N))
    a[0:6, 6:12] = np.eye(6)
    a[6, 4] = -G
    a[7, 3] = G
    a[0, 0] = -p.xy_leak
    a[1, 1] = -p.xy_leak
    b = np.zeros((N, NU))
    dthrust = 2 * p.kt * w  # N por rad/s
    b[8, :] = -dthrust / p.mass
    torque = np.vstack(
        [-xy[:, 1] * dthrust, xy[:, 0] * dthrust, MOTOR_SIGN_YAW * p.kq / p.kt * dthrust]
    )
    b[9:12, :] = np.linalg.solve(p.inertia_matrix, torque)
    return a, b


def discretize(ac: np.ndarray, bc: np.ndarray, ts: float = TS) -> tuple:
    """ZOH exato: expm([[A B],[0 0]]·Ts)."""
    n, m = bc.shape
    aug = np.zeros((n + m, n + m))
    aug[:n, :n] = ac
    aug[:n, n:] = bc
    e = sl.expm(aug * ts)
    return e[:n, :n], e[:n, n:]


def dlqr(ad: np.ndarray, bd: np.ndarray, q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Ganho K de u = -K x que minimiza Σ xᵀQx + uᵀRu."""
    pm = sl.solve_discrete_are(ad, bd, q, r)
    return np.linalg.solve(r + bd.T @ pm @ bd, bd.T @ pm @ ad)


def feedforward(ad: np.ndarray, bd: np.ndarray, k: np.ndarray) -> np.ndarray:
    """Kfd tal que a saída [x y z psi] em regime segue r: Gd·Kfd = I."""
    cref = np.eye(N)[list(REF_STATES), :]
    gd = cref @ np.linalg.solve(np.eye(N) - ad + bd @ k, bd)
    if np.linalg.matrix_rank(gd) < 4:
        raise ValueError('mapa de referência singular')
    return np.linalg.inv(gd)


def design(ad: np.ndarray, bd: np.ndarray, q: np.ndarray, r: np.ndarray) -> tuple:
    """Kd, Kfd e o maior módulo de polo de malha fechada (deve ser < 1)."""
    k = dlqr(ad, bd, q, r)
    pole = float(np.abs(np.linalg.eigvals(ad - bd @ k)).max())
    if pole >= 1.0:
        raise ValueError(f'malha fechada instável (polo {pole:.6f})')
    return k, feedforward(ad, bd, k), pole


def load_generated(path: Path = GENERATED) -> dict:
    """Matrizes do Simulink Coder (``STM32_data.c``): A, B (discretas), Kd, Kfd, Q de saída."""
    parts = re.split(r'/\* Expression: (\S+)', path.read_text())
    out = {}
    for name, body in zip(parts[1::2], parts[2::2]):
        block = body.split('}', 1)[0].split('*/', 1)[1].split('{', 1)[1]
        out[name] = np.array(
            [float(x) for x in re.findall(r'[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?', block)]
        )
    return {
        'A': out['pInitialization.A'].reshape(N, N, order='F'),
        'B': out['pInitialization.B'].reshape(N, NU, order='F'),
        'Kd': out['Kd'].reshape(NU, N, order='F'),
        'Kfd': out['Kfd'].reshape(NU, NU, order='F'),
    }


def identify(ad: np.ndarray, bd: np.ndarray, ts: float = TS) -> dict:
    """Parâmetros que ``A`` e ``B`` determinam (o modelo contínuo implícito, por logaritmo da matriz).

    Só quatro números físicos aparecem em B: ``w0`` (de B[w] e da gravidade), ``l·kt/Ixx``,
    ``l·kt/Iyy`` e ``kq/Izz``. Massa, ``kt``, inércias e ``kq`` separados NÃO são identificáveis
    só por A e B: precisam de medida (balança, CAD, bancada).
    """
    aug = np.zeros((N + NU, N + NU))
    aug[:N, :N] = ad
    aug[:N, N:] = bd
    aug[N:, N:] = np.eye(NU)
    lg = sl.logm(aug).real / ts
    ac, bc = lg[:N, :N], lg[:N, N:]
    bw = bc[8, 0]
    w0 = -G / (2 * bw)  # com m·g = 4·kt·ω0²: B[w] = -2·kt·ω0/m = -g/(2·ω0)
    return {
        'xy_leak': float(-ac[0, 0]),
        'w0': float(w0),
        'b_w': float(bw),
        'b_p': float(abs(bc[9, 0])),
        'b_q': float(abs(bc[10, 0])),
        'b_r': float(abs(bc[11, 0])),
        'ac': ac,
        'bc': bc,
    }


def implied_physics(ident: dict, mass_kg: float, arm_m: float) -> dict:
    """Com a massa pesada e o braço medido, fecha kt, inércias e kq implícitos em B."""
    w0 = ident['w0']
    kt = mass_kg * G / (4 * w0**2)
    s2l = np.sqrt(2) * arm_m * kt * w0
    izz_kq = 2 * w0 / ident['b_r']  # Izz / kq
    return {
        'kt': kt,
        'ixx': s2l / ident['b_p'],
        'iyy': s2l / ident['b_q'],
        'izz_over_kq': izz_kq,
    }


def format_header(k: np.ndarray, kfd: np.ndarray, note: str, trim: np.ndarray | None = None) -> str:
    """Cabeçalho C com Kd, Kfd (layout do Simulink Coder, coluna a coluna) e o trim de hover."""
    trim = np.zeros(NU) if trim is None else np.asarray(trim, dtype=float)
    trim_c = ', '.join(f'{v:.4f}F' for v in trim)

    def block(name: str, mat: np.ndarray) -> str:
        flat = mat.flatten(order='F')
        rows = ',\n  '.join(
            ', '.join(f'{v:.17g}' for v in flat[i : i + 4]) for i in range(0, len(flat), 4)
        )
        return f'static const double {name}[{flat.size}] = {{\n  {rows}\n}};\n'

    return (
        '/**\n'
        ' * @file lqg_gains.h\n'
        ' * @brief Ganhos do LQR discreto: Kd (4x12) e Kfd (4x4). GERADO por tools/lqr_design.py.\n'
        ' *\n'
        ' * Não editar à mão. Layout do Simulink Coder (coluna a coluna): Kd[(k * 4) + i] é o\n'
        ' * ganho da entrada i (u1..u4, Δω em rad/s) sobre o estado k de\n'
        ' * [x y z phi theta psi u v w p q r] (m, rad, m/s, rad/s; NED/FRD). Ts = 0,01 s.\n'
        f' * {note}\n'
        ' */\n'
        '#ifndef CONTROL_LQG_GAINS_H\n'
        '#define CONTROL_LQG_GAINS_H\n\n'
        f'{block("LQG_KD", k)}\n{block("LQG_KFD", kfd)}\n'
        '/** Trim de hover de u1..u4 (‰ da aceleração, soma ~0) que o CG fora do centro dos motores\n'
        ' *  exige; zero com o modelo da submissão. Aplicado em lqg_ctrl.c (LQG_CG_TRIM_ENABLE). */\n'
        f'static const float LQG_HOVER_TRIM_PERMILLE[4] = {{{trim_c}}};\n\n'
        '#endif\n'
    )


def _weights(args: argparse.Namespace) -> tuple:
    q = np.diag(args.q if args.q else Q_SUBMISSION).astype(float)
    r = args.r * np.eye(NU)
    return q, r


def _params(args: argparse.Namespace) -> Params:
    """Params do JSON de ``inertia_calc.py`` (``--params``) ou os padrões."""
    extra = json.loads(Path(args.params).read_text()) if args.params else {}
    for key in ('inertia_tensor', 'cg_m'):
        if key in extra:
            extra[key] = tuple(extra[key])
    return Params(xy_leak=args.xy_leak, **extra)


def _model(args: argparse.Namespace) -> tuple:
    """Ad, Bd, origem e trim de hover (‰, u1..u4).

    Com ``--physics``, ganhos da planta física. Sem ele, ganhos do modelo gerado; se houver
    ``--params``, o trim vem do CG dessa montagem (os ganhos não mudam).
    """
    trim = _params(args).hover_trim_permille() if args.params else np.zeros(NU)
    if args.physics:
        ac, bc = continuous(_params(args))
        return (*discretize(ac, bc), 'modelo físico (Params)', trim)
    gen = load_generated()
    return gen['A'], gen['B'], 'A e B do modelo gerado (STM32_data.c)', trim


def cmd_check(args: argparse.Namespace) -> int:
    gen = load_generated()
    q, r = _weights(args)
    k, kfd, pole = design(gen['A'], gen['B'], q, r)
    e_k = float(np.abs(k - gen['Kd']).max())
    e_f = float(np.abs(kfd - gen['Kfd']).max())
    print(f'Kd  gerado vs Python: erro máx {e_k:.3e} (|Kd| máx {np.abs(gen["Kd"]).max():.2f})')
    print(f'Kfd gerado vs Python: erro máx {e_f:.3e}')
    print(f'maior polo de malha fechada: {pole:.9f}')
    ok = e_k < 1e-7 and e_f < 1e-7
    print('OK: o fluxo em Python reproduz o Simulink Coder' if ok else 'DIVERGE')
    return 0 if ok else 1


def cmd_identify(args: argparse.Namespace) -> int:
    gen = load_generated()
    ident = identify(gen['A'], gen['B'])
    p = Params()
    _, bc = continuous(p)
    print('Modelo gerado (STM32_data.c), implícito em A e B:')
    print(
        f'  vazamento x,y         {ident["xy_leak"]:.4g} 1/s  (o script de revisão e Params usam 0)'
    )
    print(f'  w0 (hover)            {ident["w0"]:.2f} rad/s')
    print(f'  B[w]                  {ident["b_w"]:.5g}  (m/s)/(rad/s)/s')
    print(
        f'  |B[p]| |B[q]| |B[r]|  {ident["b_p"]:.5g} {ident["b_q"]:.5g} {ident["b_r"]:.5g}  rad/s² por rad/s'
    )
    print('Params padrão (corpo 455 g, motor + hélice 24 g; reproduz o modelo gerado):')
    print(
        f'  w0 {p.w0:.2f} rad/s; B[w] {bc[8, 0]:.5g}; B[p] {bc[9, 0]:.5g}; B[q] {bc[10, 1]:.5g}; B[r] {abs(bc[11, 0]):.5g}'
    )
    print(
        f'Razão gerado/revisão: B[w] {ident["b_w"] / bc[8, 0]:.4f}  B[p] {ident["b_p"] / bc[9, 0]:.4f}  '
        f'B[q] {ident["b_q"] / bc[10, 1]:.4f}  B[r] {ident["b_r"] / abs(bc[11, 0]):.4f}'
    )
    if args.mass:
        ph = implied_physics(ident, args.mass, args.arm)
        print(
            f'Com massa {args.mass} kg e braço {args.arm} m: kt {ph["kt"]:.4e}, Ixx {ph["ixx"]:.4e}, '
            f'Iyy {ph["iyy"]:.4e} kg·m², Izz/kq {ph["izz_over_kq"]:.4e}'
        )
    print(
        'Só estes 4 números físicos aparecem em A e B; massa, kt, inércias e kq separados precisam de medida.'
    )
    return 0


def cmd_emit(args: argparse.Namespace) -> int:
    ad, bd, origin, trim = _model(args)
    if args.trim:
        trim = np.array(args.trim, dtype=float)
    q, r = _weights(args)
    k, kfd, pole = design(ad, bd, q, r)
    note = f'{origin}; trim de hover {np.round(trim, 1).tolist()} ‰ (u1..u4); Q = diag{tuple(np.diag(q).tolist())}; R = {args.r} I; polo {pole:.6f}.'
    text = format_header(k, kfd, note, trim)
    out = Path(args.out)
    if args.dry_run:
        print(text)
    else:
        out.write_text(text)
        print(f'escrito {out}')
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name in ('check', 'emit'):
        sp = sub.add_parser(name)
        sp.add_argument('--q', type=float, nargs=N, help='diagonal de Q (12 valores)')
        sp.add_argument('--r', type=float, default=R_SUBMISSION, help='R = r·I (4x4)')
        if name == 'emit':
            sp.add_argument(
                '--physics', action='store_true', help='planta de Params em vez do modelo gerado'
            )
            sp.add_argument('--xy-leak', type=float, default=0.0)
            sp.add_argument(
                '--params',
                help='JSON de inertia_calc.py (body_kg, inertia_tensor, cg_m): trim de hover do CG; '
                'com --physics também os ganhos',
            )
            sp.add_argument(
                '--trim',
                type=float,
                nargs=4,
                metavar=('U1', 'U2', 'U3', 'U4'),
                help='trim de hover por entrada u1..u4 em ‰ (u1=M4, u2=M2, u3=M1, u4=M3); '
                'substitui o do CG',
            )
            sp.add_argument('--out', default=str(GAINS_H))
            sp.add_argument('--dry-run', action='store_true')
    si = sub.add_parser('identify')
    si.add_argument('--mass', type=float, help='massa total pesada, kg')
    si.add_argument('--arm', type=float, default=Params().arm_m, help='braço motor-centro, m')
    args = ap.parse_args(argv)
    return {'check': cmd_check, 'identify': cmd_identify, 'emit': cmd_emit}[args.cmd](args)


if __name__ == '__main__':
    sys.exit(main())
