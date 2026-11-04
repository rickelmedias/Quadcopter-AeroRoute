"""Importa as matrizes do código gerado pelo Simulink (Embedded Coder, ert.tlc).

O modelo do TCC foi simulado e validado no MATLAB/Simulink e exportado como
``STM32_ert_rtw``. O ``STM32_data.c`` gerado guarda todas as matrizes numa
única struct ``STM32_ConstP``, em ordem de coluna (column-major), que é difícil
de ler e de conferir. Este script lê esse arquivo e gera
``App/Src/control/lqg_ert_model.c`` com as mesmas matrizes em ordem de linha
(``A[i][j]``), **preservando o texto exato de cada número**, para que o
firmware use literalmente os mesmos valores que o MATLAB.

Uso (só biblioteca padrão):

    python tools/ert_import.py                         # usa reference/STM32_ert_rtw
    python tools/ert_import.py ../STM32_ert_rtw        # outra exportação
    python tools/ert_import.py --check                 # só confere se está atualizado

Depois de exportar o modelo de novo no Simulink, copie os fontes para
``reference/STM32_ert_rtw`` (o teste ``tests/test_lqg_ert.c`` compara o
firmware com eles) e rode este script.
"""
import argparse
import re
import sys
from pathlib import Path

#: Raiz do projeto.
ROOT = Path(__file__).resolve().parents[1]
#: Arquivo gerado.
OUTPUT = ROOT / 'App/Src/control/lqg_ert_model.c'

#: Campos de STM32_ConstP, na ordem em que aparecem, com nome no firmware,
#: dimensões (linhas, colunas) e origem no modelo Simulink.
FIELDS = [
    ('A_Value', 'LQG_ERT_A', 12, 12, "pInitialization.A, bloco '<S2>/A'"),
    ('B_Value', 'LQG_ERT_B', 12, 4, "pInitialization.B, bloco '<S2>/B'"),
    ('C_Value', 'LQG_ERT_C', 9, 12, "pInitialization.C, bloco '<S2>/C'"),
    ('KalmanGainL_Value', 'L', 12, 9, "pInitialization.L, bloco '<S3>/KalmanGainL'"),
    ('KalmanGainM_Value', 'M', 12, 9, "pInitialization.M, bloco '<S3>/KalmanGainM'"),
    ('Gain7_Gain', 'Kfd', 4, 4, "Kfd, bloco '<S1>/Gain7'"),
    ('Gain5_Gain', 'Kd', 4, 12, "Kd, bloco '<S1>/Gain5'"),
]

STATE_NAMES = ['x', 'y', 'z', 'phi', 'theta', 'psi', 'vx', 'vy', 'vz', 'p', 'q', 'r']
OUTPUT_NAMES = ['z', 'phi', 'theta', 'psi', 'vx', 'vy', 'p', 'q', 'r']
INPUT_NAMES = ['du1', 'du2', 'du3', 'du4']
REF_NAMES = ['x_ref', 'y_ref', 'z_ref', 'psi_ref']
#: Nome de cada linha, para o comentário ao fim dela.
ROW_NAMES = {
    'LQG_ERT_A': STATE_NAMES, 'LQG_ERT_B': STATE_NAMES, 'LQG_ERT_C': OUTPUT_NAMES,
    'L': STATE_NAMES, 'M': STATE_NAMES, 'Kfd': INPUT_NAMES, 'Kd': INPUT_NAMES,
}


def parse_const_p(text):
    """Extrai os vetores de STM32_ConstP como listas de literais (texto).

    Args:
        text: Conteúdo de STM32_data.c.

    Returns:
        Lista com um vetor de literais por campo, na ordem do arquivo.
    """
    start = text.index('STM32_ConstP = {')
    body = re.sub(r'/\*.*?\*/', '', text[start:], flags=re.S)
    blocks = re.findall(r'\{([^{}]*)\}', body)
    return [[tok.strip() for tok in block.split(',') if tok.strip()] for block in blocks]


def to_rows(values, rows, cols):
    """Converte um vetor column-major do Simulink em linhas (row-major)."""
    if len(values) != rows * cols:
        raise ValueError(f'esperado {rows * cols} valores, veio {len(values)}')
    return [[values[c * rows + r] for c in range(cols)] for r in range(rows)]


def literal(token):
    """Confere que o literal é um número e o devolve sem alterar o texto."""
    float(token)
    return token


def emit_matrix(lines, name, rows, origin, indent):
    """Escreve uma matriz como inicializador C, uma linha por linha do modelo."""
    lines.append(f'{indent}/* {origin} */')
    lines.append(f'{indent}{{')
    names = ROW_NAMES[name]
    for i, row in enumerate(rows):
        sep = ',' if i + 1 < len(rows) else ''
        lines.append(f'{indent}  {{ ' + ', '.join(literal(v) for v in row) + f' }}{sep}  /* {names[i]} */')
    lines.append(f'{indent}}}')


def generate(source):
    """Gera o texto de lqg_ert_model.c a partir de um STM32_data.c."""
    blocks = parse_const_p(source.read_text())
    if len(blocks) != len(FIELDS):
        raise ValueError(f'STM32_ConstP tem {len(blocks)} campos, esperado {len(FIELDS)}')
    mats = {name: to_rows(vals, r, c) for (_, name, r, c, _), vals in zip(FIELDS, blocks)}
    origin = {name: o for (_, name, _, _, o) in FIELDS}

    out = [
        '/**',
        ' * @file lqg_ert_model.c',
        ' * @brief Matrizes do modelo do TCC exportadas pelo Simulink (contrato em',
        ' *        lqg_ert_model.h).',
        ' *',
        ' * GERADO por tools/ert_import.py a partir de',
        ' * reference/STM32_ert_rtw/STM32_data.c; não edite à mão. Os números são o',
        ' * texto exato do código gerado, só reorganizados de coluna para linha.',
        ' */',
        '#include "control/lqg_ert_model.h"',
        '',
    ]
    for name, rows_macro, cols_macro in (('LQG_ERT_A', 'LQG_NX', 'LQG_NX'),
                                         ('LQG_ERT_B', 'LQG_NX', 'LQG_NU'),
                                         ('LQG_ERT_C', 'LQG_NY', 'LQG_NX')):
        out.append(f'const lqg_real_t {name}[{rows_macro}][{cols_macro}] =')
        emit_matrix(out, name, mats[name], origin[name], '')
        out[-1] += ';'
        out.append('')
    out.append('const lqg_gains_t LQG_ERT_DEFAULT_GAINS = {')
    for i, name in enumerate(('Kd', 'Kfd', 'L', 'M')):
        out.append(f'  .{name} =')
        emit_matrix(out, name, mats[name], origin[name], '  ')
        if i < 3:
            out[-1] += ','
    out.append('};')
    out.append('')
    return '\n'.join(out)


def main():
    """Lê a exportação do Simulink e grava (ou confere) lqg_ert_model.c."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('export_dir', nargs='?', default=str(ROOT / 'reference/STM32_ert_rtw'),
                        help='pasta com o STM32_data.c gerado pelo Simulink')
    parser.add_argument('--check', action='store_true',
                        help='não grava; sai com erro se o arquivo estiver desatualizado')
    args = parser.parse_args()
    text = generate(Path(args.export_dir) / 'STM32_data.c')
    if args.check:
        if not OUTPUT.exists() or OUTPUT.read_text() != text:
            print(f'{OUTPUT.relative_to(ROOT)} está desatualizado; rode tools/ert_import.py')
            return 1
        print(f'{OUTPUT.relative_to(ROOT)} confere com {args.export_dir}')
        return 0
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(text)
    print(f'gerado {OUTPUT.relative_to(ROOT)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
