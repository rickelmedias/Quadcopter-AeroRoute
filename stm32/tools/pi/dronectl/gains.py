"""Ganhos do LQG no formato do envio para o STM32 (só biblioteca padrão).

O formato precisa ser idêntico ao de ``App/Src/control/ctrl_gains.c``:
matrizes na ordem Kd, Kfd, L, M, cada uma linha por linha, e CRC-32 (zlib)
dos valores em float32 little-endian. ``tests/run.sh`` confere que os dois
lados calculam o mesmo CRC para os ganhos do Simulink.
"""
from __future__ import annotations

import struct
import sys
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, List, Tuple

#: Raiz do repositório (tools/pi/dronectl -> raiz).
REPO_ROOT = Path(__file__).resolve().parents[3]

#: Ordem canônica das matrizes, com o id de DRONE_GAIN_MATRIX e a forma.
MATRICES: Tuple[Tuple[str, int, int, int], ...] = (
    ('Kd', 0, 4, 12),
    ('Kfd', 1, 4, 4),
    ('L', 2, 12, 9),
    ('M', 3, 12, 9),
)

Matrix = List[List[float]]


@dataclass
class Gains:
    """Kd, Kfd, L e M como listas de linhas."""

    Kd: Matrix
    Kfd: Matrix
    L: Matrix
    M: Matrix

    def __post_init__(self) -> None:
        for name, _, rows, cols in MATRICES:
            m = getattr(self, name)
            if len(m) != rows or any(len(r) != cols for r in m):
                raise ValueError(f'{name} precisa ser {rows}x{cols}')

    def rows(self) -> Iterator[Tuple[int, int, List[float]]]:
        """Percorre as 32 linhas na ordem do envio: (id da matriz, linha, valores)."""
        for name, matrix_id, _, _ in MATRICES:
            for index, row in enumerate(getattr(self, name)):
                yield matrix_id, index, [float(v) for v in row]

    def values(self) -> List[float]:
        """Os 280 valores na ordem canônica."""
        return [v for _, _, row in self.rows() for v in row]

    def crc32(self) -> int:
        """CRC-32 do envio (o mesmo de ctrl_gains_crc no firmware)."""
        data = b''.join(struct.pack('<f', v) for v in self.values())
        return zlib.crc32(data) & 0xFFFFFFFF


def reference_matrices(export_dir: Path | None = None) -> dict:
    """Lê A, B, C, Kd, Kfd, L e M da exportação do Simulink.

    Usa o mesmo leitor de ``tools/ert_import.py``, que gera as tabelas do
    firmware, então a Pi e o STM32 partem exatamente dos mesmos números.

    Args:
        export_dir: Pasta com o STM32_data.c (padrão: reference/STM32_ert_rtw).

    Returns:
        Dicionário nome -> lista de linhas (floats).
    """
    sys.path.insert(0, str(REPO_ROOT / 'tools'))
    try:
        import ert_import
    finally:
        sys.path.pop(0)
    folder = export_dir or REPO_ROOT / 'reference/STM32_ert_rtw'
    blocks = ert_import.parse_const_p((folder / 'STM32_data.c').read_text())
    out = {}
    for (_, name, rows, cols, _), values in zip(ert_import.FIELDS, blocks):
        key = name.replace('LQG_ERT_', '')
        out[key] = [[float(v) for v in row] for row in ert_import.to_rows(values, rows, cols)]
    return out


def default_gains() -> Gains:
    """Ganhos exportados do Simulink (os que o firmware usa ao ligar)."""
    m = reference_matrices()
    return Gains(Kd=m['Kd'], Kfd=m['Kfd'], L=m['L'], M=m['M'])
