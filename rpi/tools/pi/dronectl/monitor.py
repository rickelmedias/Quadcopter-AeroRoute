"""Estatísticas da filtragem e gravação da telemetria do LQG.

Para cada medida y_i, compara o sinal medido com a estimativa do Kalman do
mesmo estado numa janela deslizante:

- desvio padrão de y (ruído + movimento);
- desvio padrão de x̂ (o que o filtro deixou passar);
- desvio padrão de y - x̂ (o ruído que o filtro removeu). Com o drone parado,
  a variância de y - x̂ é uma boa estimativa da variância do ruído do sensor,
  ou seja, do Rn daquela medida: a coluna "Rn sugerido" ajuda a ajustar o
  Kalman com dados reais.
"""
from __future__ import annotations

import csv
import math
import threading
from collections import deque
from pathlib import Path
from typing import Deque, List, Optional, Sequence

from .design import OUTPUTS, STATES
from .link import DebugSample, state_name

#: Estado medido por cada saída (colunas da matriz C).
OUTPUT_STATE = [STATES.index(name) for name in OUTPUTS]
#: Bits de DRONE_MEAS_FLAG que validam cada saída.
OUTPUT_FLAG = [4, 1, 1, 1, 8, 8, 2, 2, 2]


def _std(values: Sequence[float]) -> float:
    n = len(values)
    if n < 2:
        return float('nan')
    mean = sum(values) / n
    return math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1))


class FilterStats:
    """Janela deslizante de y e x̂ para cada medida."""

    def __init__(self, window: int = 250) -> None:
        self._lock = threading.Lock()
        self._y: List[Deque[float]] = [deque(maxlen=window) for _ in OUTPUTS]
        self._x: List[Deque[float]] = [deque(maxlen=window) for _ in OUTPUTS]
        self.samples = 0
        self.last: Optional[DebugSample] = None

    def add(self, s: DebugSample) -> None:
        """Acrescenta um passo (chamado pela thread de recepção)."""
        with self._lock:
            self.samples += 1
            self.last = s
            for i, state in enumerate(OUTPUT_STATE):
                if s.meas_flags & OUTPUT_FLAG[i]:
                    self._y[i].append(s.y[i])
                    self._x[i].append(s.x_hat[state])

    def table(self, rn: Optional[Sequence[float]] = None) -> str:
        """Tabela de texto com as estatísticas atuais."""
        with self._lock:
            rows = []
            for i, name in enumerate(OUTPUTS):
                y, x = list(self._y[i]), list(self._x[i])
                residual = [a - b for a, b in zip(y, x)]
                std_res = _std(residual)
                rows.append((name, y[-1] if y else float('nan'), x[-1] if x else float('nan'),
                             _std(y), _std(x), std_res, std_res ** 2,
                             rn[i] if rn else float('nan')))
            last = self.last
        lines = []
        if last is not None:
            lines.append(f'passo {last.step}  estado {state_name(last.state)}  '
                         f'meas=0x{last.meas_flags:X} ctrl=0x{last.ctrl_flags:X}  '
                         f'du=[{", ".join(f"{v:+.1f}" for v in last.du)}] rad/s  '
                         f'acel=[{", ".join(f"{v * 100:.0f}" for v in last.throttle)}]%')
        lines.append(f'{"medida":>7} {"y":>10} {"x_hat":>10} {"dp(y)":>10} {"dp(x_hat)":>10} '
                     f'{"dp(y-x)":>10} {"Rn sugerido":>12} {"Rn atual":>10}')
        for name, y, x, sy, sx, sr, var, cur in rows:
            lines.append(f'{name:>7} {y:>10.4f} {x:>10.4f} {sy:>10.2e} {sx:>10.2e} '
                         f'{sr:>10.2e} {var:>12.2e} {cur:>10.2e}')
        return '\n'.join(lines)

    def suggested_rn(self) -> List[float]:
        """Variância de y - x̂ de cada medida (NaN sem amostras)."""
        with self._lock:
            out = []
            for i in range(len(OUTPUTS)):
                residual = [a - b for a, b in zip(self._y[i], self._x[i])]
                out.append(_std(residual) ** 2)
            return out


class CsvRecorder:
    """Grava cada DRONE_CTRL_DEBUG numa linha de CSV."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open('w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            ['time_ms', 'step', 'state', 'meas_flags', 'ctrl_flags'] +
            [f'y_{n}' for n in OUTPUTS] + [f'xhat_{n}' for n in STATES] +
            [f'du{i + 1}' for i in range(4)] + ['ref_x', 'ref_y', 'ref_z', 'ref_psi'] +
            [f'throttle{i + 1}' for i in range(4)])
        self._lock = threading.Lock()

    def add(self, s: DebugSample) -> None:
        with self._lock:
            if self._file.closed:
                return
            self._writer.writerow([s.time_ms, s.step, s.state, s.meas_flags, s.ctrl_flags,
                                   *s.y, *s.x_hat, *s.du, *s.ref, *s.throttle])

    def close(self) -> None:
        with self._lock:
            self._file.close()
