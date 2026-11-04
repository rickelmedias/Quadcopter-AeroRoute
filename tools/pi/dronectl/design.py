"""Projeto do LQR e do Kalman do TCC, igual ao do MATLAB (precisa de numpy).

Os ganhos exportados do Simulink são reproduzidos por (erro relativo < 1e-9):

- ``Kd = dlqr(A, B, Q, R)`` com Q = diag(5, 5, 500, 500, 500, 50, 5, 5, 5, 10,
  10, 5) e R = 0,1 I;
- ``Kfd = inv(Cr (I - A + B Kd)^-1 B)``, Cr escolhendo x, y, z e ψ;
- Kalman de regime com G = I, Qn = diag(1e-5 x6, 1e-3 x6) e Rn = variâncias
  medidas dos sensores: ``M = P C' (C P C' + Rn)^-1`` e ``L = A M``.

A, B e C vêm da exportação do Simulink (reference/STM32_ert_rtw), então só os
pesos mudam durante o ajuste: o modelo continua sendo o do TCC.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Sequence, Tuple, Union

import numpy as np

from .gains import Gains, reference_matrices

#: Nomes dos estados (ordem do modelo).
STATES = ['x', 'y', 'z', 'phi', 'theta', 'psi', 'vx', 'vy', 'vz', 'p', 'q', 'r']
#: Nomes das medidas (ordem da matriz C).
OUTPUTS = ['z', 'phi', 'theta', 'psi', 'vx', 'vy', 'p', 'q', 'r']
#: Estados que a referência controla (x, y, z, ψ).
REF_STATES = [0, 1, 2, 5]


def _tcc_q() -> List[float]:
    return [5.0, 5.0, 500.0, 500.0, 500.0, 50.0, 5.0, 5.0, 5.0, 10.0, 10.0, 5.0]


def _tcc_r() -> List[float]:
    return [0.1] * 4


def _tcc_qn() -> List[float]:
    return [1e-5] * 6 + [1e-3] * 6


def _tcc_rn() -> List[float]:
    return [1.15e-4, 5.0e-8, 1.94e-7, 6.04e-8, 1.28e-2, 2.2e-3, 2.06e-6, 3.24e-7, 2.38e-7]


@dataclass
class Weights:
    """Pesos do projeto. O padrão são os do TCC (reproduzem o Simulink)."""

    q: List[float] = field(default_factory=_tcc_q)
    r: List[float] = field(default_factory=_tcc_r)
    qn: List[float] = field(default_factory=_tcc_qn)
    rn: List[float] = field(default_factory=_tcc_rn)

    def set(self, kind: str, which: Union[str, int, None], value: float) -> str:
        """Altera um peso.

        Args:
            kind: 'q' e 'qn' (por estado), 'r' (por motor), 'rn' (por medida).
            which: Nome ou índice; None ou 'all' altera todos.
            value: Novo valor (> 0, ou >= 0 em q e qn).

        Returns:
            Descrição da mudança, para mostrar ao operador.
        """
        names = {'q': STATES, 'qn': STATES, 'rn': OUTPUTS, 'r': ['1', '2', '3', '4']}[kind]
        vec = getattr(self, kind)
        if value < 0 or (kind in ('r', 'rn') and value == 0):
            raise ValueError(f'{kind} precisa ser positivo')
        if which is None or which == 'all':
            for i in range(len(vec)):
                vec[i] = float(value)
            return f'{kind}[todos] = {value:g}'
        index = which if isinstance(which, int) else (
            names.index(which) if which in names else int(which))
        old = vec[index]
        vec[index] = float(value)
        return f'{kind}[{names[index]}]: {old:g} -> {value:g}'

    def to_dict(self) -> Dict[str, List[float]]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Sequence[float]]) -> 'Weights':
        w = cls()
        for key in ('q', 'r', 'qn', 'rn'):
            if key in data:
                values = [float(v) for v in data[key]]
                if len(values) != len(getattr(w, key)):
                    raise ValueError(f'{key} precisa ter {len(getattr(w, key))} valores')
                setattr(w, key, values)
        return w


def dare(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray,
         iterations: int = 100) -> np.ndarray:
    """Equação de Riccati discreta pelo algoritmo de duplicação (SDA).

    Resolve P = A' P A - A' P B (R + B' P B)^-1 B' P A + Q sem depender do
    scipy, que nem sempre está disponível na Raspberry Pi.
    """
    n = A.shape[0]
    I = np.eye(n)
    Ak = A.copy()
    Gk = B @ np.linalg.solve(R, B.T)
    Hk = Q.copy()
    for _ in range(iterations):
        W = np.linalg.inv(I + Gk @ Hk)
        A1 = Ak @ W @ Ak
        G1 = Gk + Ak @ W @ Gk @ Ak.T
        H1 = Hk + Ak.T @ Hk @ W @ Ak
        converged = np.linalg.norm(H1 - Hk) <= 1e-13 * max(1.0, np.linalg.norm(H1))
        Ak, Gk, Hk = A1, G1, H1
        if converged:
            break
    return (Hk + Hk.T) / 2.0


@dataclass
class Model:
    """A, B e C discretos do Simulink."""

    A: np.ndarray
    B: np.ndarray
    C: np.ndarray

    @classmethod
    def from_reference(cls) -> 'Model':
        m = reference_matrices()
        return cls(A=np.array(m['A']), B=np.array(m['B']), C=np.array(m['C']))


def design(weights: Weights, model: Model | None = None) -> Tuple[Gains, Dict[str, float]]:
    """Calcula Kd, Kfd, L e M para os pesos dados.

    Returns:
        Os ganhos e um resumo (maior polo da malha do controle e do estimador).
    """
    model = model or Model.from_reference()
    A, B, C = model.A, model.B, model.C
    Q, R = np.diag(weights.q), np.diag(weights.r)
    Qn, Rn = np.diag(weights.qn), np.diag(weights.rn)

    P = dare(A, B, Q, R)
    Kd = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)

    Cr = np.zeros((4, A.shape[0]))
    for i, s in enumerate(REF_STATES):
        Cr[i, s] = 1.0
    Kfd = np.linalg.inv(Cr @ np.linalg.solve(np.eye(A.shape[0]) - A + B @ Kd, B))

    S = dare(A.T, C.T, Qn, Rn)
    M = S @ C.T @ np.linalg.inv(C @ S @ C.T + Rn)
    L = A @ M

    info = {
        'controle_maior_polo': float(np.max(np.abs(np.linalg.eigvals(A - B @ Kd)))),
        'estimador_maior_polo': float(np.max(np.abs(np.linalg.eigvals(A - L @ C)))),
    }
    gains = Gains(Kd=Kd.tolist(), Kfd=Kfd.tolist(), L=L.tolist(), M=M.tolist())
    return gains, info


def compare(a: Gains, b: Gains) -> Dict[str, float]:
    """Maior diferença relativa de cada matriz (|a-b| / max|b|)."""
    out = {}
    for name in ('Kd', 'Kfd', 'L', 'M'):
        x, y = np.array(getattr(a, name)), np.array(getattr(b, name))
        out[name] = float(np.max(np.abs(x - y)) / max(np.max(np.abs(y)), 1e-30))
    return out


def load_profile(path: Path) -> Tuple[Weights, Dict[str, float]]:
    """Lê um perfil de ajuste (pesos e parâmetros do STM32)."""
    data = json.loads(Path(path).read_text())
    return Weights.from_dict(data.get('weights', {})), dict(data.get('params', {}))


def save_profile(path: Path, weights: Weights, params: Dict[str, float]) -> None:
    """Grava um perfil de ajuste em JSON."""
    data = {'weights': weights.to_dict(), 'params': params}
    Path(path).write_text(json.dumps(data, indent=2) + '\n')
