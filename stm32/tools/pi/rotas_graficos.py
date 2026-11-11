"""Gráficos do benchmark de rotas para o TCC (roda no PC; precisa de matplotlib).

    python3 tools/pi/rotas_graficos.py docs/resultados_rotas/pc

Lê ``resumo.json``, ``convergencia.json`` e ``otimos_locais.csv`` da pasta e
grava PNGs em ``<pasta>/graficos``. Cada algoritmo tem sempre a mesma cor e o
mesmo marcador (a impressão em preto e branco continua legível) e as linhas
levam o nome no fim, sem depender só da cor.
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

#: Algoritmo -> (rótulo, cor, marcador). Cores da paleta categórica validada
#: (ordem fixa; a mesma em todos os gráficos).
ESTILO = {
    'guloso': ('Guloso', '#2a78d6', 'o'),
    'guloso+2opt': ('Guloso + 2-opt', '#eb6834', 's'),
    '2opt-multi': ('2-opt multipartida (20)', '#1baf7a', '^'),
    'genetico': ('Genético', '#eda100', 'D'),
    'exato': ('Exato (Held-Karp)', '#e87ba4', 'v'),
    'forca-bruta': ('Exato (força bruta)', '#6250d6', 'P'),
    'aleatorio': ('Aleatório', '#008300', 'x'),
}
TEXTO, TEXTO2, GRADE, EIXO, FUNDO = '#0b0b0b', '#52514e', '#e1e0d9', '#c3c2b7', '#fcfcfb'


def _eixo(ax, titulo: str, xlabel: str, ylabel: str) -> None:
    ax.set_facecolor(FUNDO)
    ax.set_title(titulo, loc='left', color=TEXTO, fontsize=11, fontweight='bold')
    ax.set_xlabel(xlabel, color=TEXTO2)
    ax.set_ylabel(ylabel, color=TEXTO2)
    ax.grid(True, color=GRADE, linewidth=0.6)
    ax.set_axisbelow(True)
    for lado in ('top', 'right'):
        ax.spines[lado].set_visible(False)
    for lado in ('left', 'bottom'):
        ax.spines[lado].set_color(EIXO)
    ax.tick_params(colors=TEXTO2)


def _linhas(ax, tabela, nomes, campo, rotular=True) -> None:
    fins = []
    for nome in nomes:
        pts = [(t['n'], t[campo]) for t in tabela if t['algoritmo'] == nome]
        if not pts:
            continue
        rotulo, cor, marca = ESTILO[nome]
        xs, ys = zip(*pts)
        ax.plot(
            xs,
            ys,
            color=cor,
            linewidth=2,
            marker=marca,
            markersize=5,
            markeredgecolor=FUNDO,
            markeredgewidth=0.8,
            label=rotulo,
        )
        fins.append((ys[-1], xs[-1], rotulo))
    if rotular:
        for y, x, rotulo in fins:
            ax.annotate(
                rotulo,
                (x, y),
                xytext=(6, 0),
                textcoords='offset points',
                va='center',
                fontsize=8,
                color=TEXTO,
            )


def gap(resumo, pasta: Path) -> None:
    tab, ultimo_otimo = (
        resumo['tabela'],
        max(t['n'] for t in resumo['tabela'] if t['referencia'] == 'otimo'),
    )
    fig, ax = plt.subplots(figsize=(8.5, 4.6), facecolor=FUNDO)
    _eixo(
        ax,
        'Quanto a rota fica acima da referência (média de 10 instâncias por N)',
        'pontos a visitar (N)',
        'gap médio (%)',
    )
    _linhas(ax, tab, ['guloso', 'guloso+2opt', 'genetico', '2opt-multi'], 'gap_medio_pct')
    ax.axvline(ultimo_otimo + 0.5, color=EIXO, linestyle='--', linewidth=1)
    # Faixa vazia do gráfico: acima das linhas de baixo, abaixo do guloso.
    ax.text(
        ultimo_otimo + 0.3,
        5.0,
        '← ref. = ótimo global',
        ha='right',
        va='center',
        fontsize=8,
        color=TEXTO2,
    )
    ax.text(
        ultimo_otimo + 0.7,
        5.0,
        'ref. = melhor encontrado →',
        ha='left',
        va='center',
        fontsize=8,
        color=TEXTO2,
    )
    ax.set_xlim(2.5, 34)
    ax.legend(loc='upper left', bbox_to_anchor=(0.0, 0.9), frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(pasta / 'gap_por_n.png', dpi=200)
    plt.close(fig)


def tempo(resumo, pasta: Path) -> None:
    fig, ax = plt.subplots(figsize=(8.5, 4.6), facecolor=FUNDO)
    _eixo(
        ax,
        'Tempo de cálculo por instância (média, PC)',
        'pontos a visitar (N)',
        'tempo (ms, escala log)',
    )
    ax.set_yscale('log')
    _linhas(
        ax,
        resumo['tabela'],
        ['forca-bruta', 'exato', 'genetico', '2opt-multi', 'guloso+2opt', 'guloso'],
        'tempo_medio_ms',
    )
    ax.set_xlim(2.5, 35)
    ax.legend(loc='upper left', frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(pasta / 'tempo_por_n.png', dpi=200)
    plt.close(fig)


def convergencia(conv, pasta: Path) -> None:
    ns = sorted(conv, key=int)
    fig, eixos = plt.subplots(1, len(ns), figsize=(11, 3.4), facecolor=FUNDO, sharey=True)
    for ax, n in zip(eixos, ns):
        c = conv[n]
        ref = c['referencia_cm']
        hist = [100 * (v / ref - 1) for v in c['genetico']]
        _eixo(
            ax,
            f'N = {n} (ref. = {"ótimo" if c["referencia"] == "otimo" else "melhor"})',
            'geração',
            'gap (%)' if n == ns[0] else '',
        )
        rot, cor, _ = ESTILO['genetico']
        ax.plot(range(len(hist)), hist, color=cor, linewidth=2, label=rot)
        for nome, chave, estilo in (
            ('guloso', 'guloso_cm', ':'),
            ('guloso+2opt', 'guloso+2opt_cm', '--'),
        ):
            r, cor2, _ = ESTILO[nome]
            ax.axhline(
                100 * (c[chave] / ref - 1), color=cor2, linestyle=estilo, linewidth=1.5, label=r
            )
        ax.set_ylim(-1, 60)
    eixos[0].legend(loc='upper right', frameon=False, fontsize=7)
    fig.suptitle(
        'Convergência do genético (melhor da população) contra guloso e 2-opt',
        x=0.01,
        ha='left',
        color=TEXTO,
        fontsize=11,
        fontweight='bold',
    )
    fig.tight_layout()
    fig.savefig(pasta / 'convergencia_genetico.png', dpi=200)
    plt.close(fig)


def otimos_locais(resumo, pasta: Path) -> None:
    locais = resumo['otimos_locais']
    ns = [l['n'] for l in locais]
    _, cor, _ = ESTILO['2opt-multi']
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 3.8), facecolor=FUNDO)
    _eixo(a1, 'Ótimos locais diferentes em 20 partidas', 'pontos (N)', 'média por instância')
    a1.bar(
        ns,
        [l['distintos_medio'] for l in locais],
        color=cor,
        width=0.75,
        edgecolor=FUNDO,
        linewidth=1,
    )
    _eixo(a2, 'Partidas do 2-opt que chegam à referência', 'pontos (N)', '% das partidas')
    a2.bar(
        ns,
        [l['acharam_referencia_pct'] for l in locais],
        color=cor,
        width=0.75,
        edgecolor=FUNDO,
        linewidth=1,
    )
    a2.set_ylim(0, 100)
    for ax in (a1, a2):
        ax.axvline(
            max(l['n'] for l in locais if l['referencia'] == 'otimo') + 0.5,
            color=EIXO,
            linestyle='--',
            linewidth=1,
        )
    fig.tight_layout()
    fig.savefig(pasta / 'otimos_locais.png', dpi=200)
    plt.close(fig)


def main() -> int:
    pasta = Path(sys.argv[1] if len(sys.argv) > 1 else 'docs/resultados_rotas/pc')
    resumo = json.loads((pasta / 'resumo.json').read_text(encoding='utf-8'))
    conv = json.loads((pasta / 'convergencia.json').read_text(encoding='utf-8'))
    saida = pasta / 'graficos'
    saida.mkdir(exist_ok=True)
    gap(resumo, saida)
    tempo(resumo, saida)
    convergencia(conv, saida)
    otimos_locais(resumo, saida)
    print(f'gráficos em {saida}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
