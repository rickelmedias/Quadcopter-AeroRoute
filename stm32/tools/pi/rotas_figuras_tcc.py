"""Figuras das rotas para o TCC (roda no PC; precisa de matplotlib).

    python3 tools/pi/rotas_figuras_tcc.py ~/Projects/facens/drone/LaTeXTCC/images/rotas

Diferenças para ``rotas_apresentacao.py`` (slides):

- **sem título dentro da imagem**: no TCC o título é a legenda da Figura;
- desenhadas no **tamanho real da página** (16 cm de largura, fonte de 9 pt), para
  ficarem legíveis impressas com ``width=\\textwidth``;
- comparam só aleatório, guloso, algoritmo genético, força bruta e Held-Karp
  (a busca local 2-opt fica numa tabela à parte no texto).

Gera: ``tempo_resolver.png``, ``memoria_exato.png``, ``qualidade_rotas.png``,
``rotas_3d_n10.png``, ``rotas_3d_n20.png``, ``rotas_3d_n25.png`` e
``convergencia_genetico.png``. Os números vêm de ``docs/resultados_rotas/pc``.
"""

from __future__ import annotations

import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

RAIZ = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RAIZ / 'tools/pi'))
from dronectl import rotas  # noqa: E402

DADOS = RAIZ / 'docs/resultados_rotas/pc'

#: Algoritmo -> (rótulo, cor, marcador). Paleta validada em todos os pares
#: (força bruta × Held-Karp fica na faixa de CVD que exige marcador diferente: + e ▼).
ESTILO = {
    'aleatorio': ('Aleatório', '#008300', 'x'),
    'guloso': ('Guloso', '#2a78d6', 'o'),
    'genetico': ('Algoritmo genético', '#eda100', 'D'),
    'forca-bruta': ('Força bruta', '#e34948', 'P'),
    'exato': ('Held-Karp', '#1baf7a', 'v'),
}
TEXTO, TEXTO2, GRADE, EIXO, FUNDO = '#0b0b0b', '#52514e', '#e1e0d9', '#c3c2b7', '#ffffff'
#: Largura útil da página A4 com margens ABNT (3 cm + 2 cm), em polegadas.
LARGURA = 16 / 2.54
ANO_S = 365.25 * 86400

plt.rcParams.update(
    {
        'font.size': 9,
        'axes.titlesize': 9,
        'axes.labelsize': 9,
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        'legend.fontsize': 8,
    }
)


def _eixo(ax, xlabel, ylabel):
    ax.set_facecolor(FUNDO)
    ax.set_xlabel(xlabel, color=TEXTO2)
    ax.set_ylabel(ylabel, color=TEXTO2)
    ax.grid(True, color=GRADE, linewidth=0.6)
    ax.set_axisbelow(True)
    for lado in ('top', 'right'):
        ax.spines[lado].set_visible(False)
    for lado in ('left', 'bottom'):
        ax.spines[lado].set_color(EIXO)
    ax.tick_params(colors=TEXTO2)


def _salvar(fig, saida, nome):
    fig.savefig(saida / nome, dpi=300, facecolor=FUNDO)
    plt.close(fig)


def _dados():
    resumo = json.loads((DADOS / 'resumo.json').read_text(encoding='utf-8'))
    limite = json.loads((DADOS / 'limite_exato.json').read_text(encoding='utf-8'))
    with open(DADOS / 'resultados.csv', encoding='utf-8') as f:
        linhas = list(csv.DictReader(f))
    conv = json.loads((DADOS / 'convergencia.json').read_text(encoding='utf-8'))
    return resumo, limite, linhas, conv


# ---- tempo e memória ------------------------------------------------------------------


def tempo_resolver(saida, resumo, limite):
    tab = resumo['tabela']
    corte_hk = max(m['n'] for m in limite['medidos'] if m['tempo_s'] <= limite['tempo_max_s'])
    corte_fb = max(t['n'] for t in tab if t['algoritmo'] == 'forca-bruta')
    teto = 1e13
    fig, ax = plt.subplots(figsize=(LARGURA, 4.3), facecolor=FUNDO)
    _eixo(ax, 'pontos a visitar (N)', 'tempo por instância (s, escala log)')
    ax.set_yscale('log')
    for segundos, texto in (
        (1, '1 s'),
        (60, '1 min'),
        (3600, '1 h'),
        (86400, '1 dia'),
        (ANO_S, '1 ano'),
        (1000 * ANO_S, '1000 anos'),
    ):
        ax.axhline(segundos, color=EIXO, linewidth=0.8, linestyle=':')
        ax.text(30.9, segundos, texto, fontsize=7, color=TEXTO2, va='center')
    for nome in ('guloso', 'genetico'):
        rot, cor, marca = ESTILO[nome]
        pts = [(t['n'], t['tempo_medio_ms'] / 1000.0) for t in tab if t['algoritmo'] == nome]
        ax.plot(
            *zip(*pts),
            color=cor,
            linewidth=1.8,
            marker=marca,
            markersize=3.5,
            markeredgecolor=FUNDO,
            markeredgewidth=0.5,
            label=f'{rot} (medido)',
        )
    for nome, med, prev, modelo in (
        (
            'forca-bruta',
            limite['forca_bruta']['medidos'],
            limite['forca_bruta']['previsao'],
            'c·N!',
        ),
        ('exato', limite['medidos'], limite['previsao'], 'c·N²·2ᴺ'),
    ):
        rot, cor, marca = ESTILO[nome]
        ax.plot(
            [m['n'] for m in med],
            [m['tempo_s'] for m in med],
            color=cor,
            linewidth=1.8,
            marker=marca,
            markersize=4.5,
            markeredgecolor=FUNDO,
            markeredgewidth=0.5,
            label=f'{rot} (medido)',
        )
        pts = [(med[-1]['n'], med[-1]['tempo_s'])] + [
            (p['n'], p['tempo_s']) for p in prev if p['tempo_s'] <= teto * 10
        ]
        ax.plot(
            *zip(*pts),
            color=cor,
            linewidth=1.8,
            linestyle='--',
            label=f'{rot} (previsto, {modelo})',
        )
    fb20 = next(p['tempo_s'] for p in limite['forca_bruta']['previsao'] if p['n'] == 20)
    seta = dict(arrowstyle='->', color=TEXTO2, linewidth=0.7)
    ax.annotate(
        f'força bruta, 20 pontos:\n≈ {fb20 / ANO_S / 1000:.0f} mil anos',
        (20, fb20),
        xytext=(11.3, 3e11),
        ha='left',
        va='center',
        fontsize=7.5,
        color=TEXTO,
        arrowprops=seta,
    )
    hk30 = next(p['tempo_s'] for p in limite['previsao'] if p['n'] == 30)
    ax.annotate(
        f'Held-Karp, 30 pontos:\n≈ {hk30 / 3600:.0f} h e 290 GB de RAM',
        (30, hk30),
        xytext=(23.2, 3e6),
        ha='left',
        va='center',
        fontsize=7.5,
        color=TEXTO,
        arrowprops=seta,
    )
    for x, texto in (
        (corte_fb, f'força bruta\nsai (N ≥ {corte_fb + 1})'),
        (corte_hk, f'Held-Karp\nsai (N ≥ {corte_hk + 1})'),
    ):
        ax.axvline(x + 0.5, color=TEXTO2, linewidth=0.8, linestyle='--')
        ax.text(x + 0.7, 1.5e-6, texto, fontsize=7.5, color=TEXTO2, va='bottom')
    ax.set_xlim(2.5, 30.8)
    ax.set_ylim(1e-6, teto)
    ax.legend(
        frameon=False,
        loc='upper center',
        bbox_to_anchor=(0.5, -0.16),
        ncol=3,
        handlelength=2.4,
        columnspacing=1.2,
    )
    fig.tight_layout()
    _salvar(fig, saida, 'tempo_resolver.png')


def memoria_exato(saida, limite):
    ns = list(range(3, 31))
    fig, ax = plt.subplots(figsize=(LARGURA, 3.6), facecolor=FUNDO)
    _eixo(ax, 'pontos a visitar (N)', 'memória (MB, escala log)')
    ax.set_yscale('log')
    rot, cor, marca = ESTILO['exato']
    ax.plot(
        ns,
        [n * 2**n * 9 / 1e6 for n in ns],
        color=cor,
        linewidth=1.8,
        linestyle='--',
        label=f'{rot}: tabela de N·2ᴺ estados (calculada)',
    )
    med = [m for m in limite['medidos'] if m['n'] >= 16]
    ax.plot(
        [m['n'] for m in med],
        [m['pico_mb'] for m in med],
        color=cor,
        linewidth=0,
        marker=marca,
        markersize=5,
        markeredgecolor=FUNDO,
        label=f'{rot}: pico medido do processo',
    )
    rot, cor, marca = ESTILO['genetico']
    ax.plot(
        ns,
        [120 * 2 * n * 8 / 1e6 for n in ns],
        color=cor,
        linewidth=1.8,
        marker=marca,
        markersize=3,
        markeredgecolor=FUNDO,
        label=f'{rot}: duas populações de 120 rotas',
    )
    for mb, texto in (
        (512, 'RAM do Raspberry Pi Zero W (512 MB)'),
        (
            limite['ram_total_mb'],
            f"RAM total do computador ({limite['ram_total_mb'] / 1000:.1f} GB)".replace('.', ','),
        ),
    ):
        ax.axhline(mb, color=TEXTO2, linewidth=0.8, linestyle=':')
        ax.text(3.2, mb * 1.4, texto, fontsize=7.5, color=TEXTO2)
    ax.set_xlim(2.5, 30.5)
    ax.legend(frameon=False, loc='upper center', bbox_to_anchor=(0.5, -0.2), ncol=2)
    fig.tight_layout()
    _salvar(fig, saida, 'memoria_exato.png')


# ---- qualidade da rota ------------------------------------------------------------------


def qualidade_rotas(saida, linhas):
    """Qualidade = referência / distância × 100 %: 100 % é a rota ótima; maior é melhor."""
    soma, cont = defaultdict(float), defaultdict(int)
    for l in linhas:
        chave = (int(l['n']), l['algoritmo'])
        soma[chave] += 100.0 / (1.0 + float(l['gap_pct']) / 100.0)
        cont[chave] += 1
    media = {k: soma[k] / cont[k] for k in soma}
    ns = [5, 10, 15, 20, 22, 25, 30]
    nomes = ['aleatorio', 'guloso', 'genetico']
    fig, ax = plt.subplots(figsize=(LARGURA, 3.6), facecolor=FUNDO)
    _eixo(ax, 'pontos a visitar (N)', 'qualidade da rota (%)')
    largura = 0.26
    for k, nome in enumerate(nomes):
        rot, cor, _ = ESTILO[nome]
        xs = [i + (k - 1) * largura for i in range(len(ns))]
        ys = [media[(n, nome)] for n in ns]
        ax.bar(xs, ys, width=largura - 0.03, color=cor, label=rot)
        # Valor dentro do topo da barra, para não brigar com a linha dos 100 %.
        for x, y in zip(xs, ys):
            ax.text(
                x,
                y - 1.5,
                f'{y:.0f}',
                ha='center',
                va='top',
                fontsize=6.5,
                color=TEXTO if nome == 'genetico' else '#ffffff',
            )
    ax.axhline(
        100,
        color=TEXTO,
        linewidth=0.9,
        linestyle='--',
        label='rota ótima (ou melhor conhecida) = 100 %',
    )
    ax.set_xticks(range(len(ns)), [str(n) for n in ns])
    ax.set_ylim(0, 105)
    ax.legend(frameon=False, loc='upper center', bbox_to_anchor=(0.5, -0.2), ncol=4)
    fig.tight_layout()
    _salvar(fig, saida, 'qualidade_rotas.png')
    return {f'{nome}@{n}': round(media[(n, nome)], 1) for n in ns for nome in nomes}


# ---- rotas em 3D --------------------------------------------------------------------------


def _rota_3d(ax, pontos, ordem, nome, ref):
    rot, cor, _ = ESTILO[nome]
    origem = rotas.ORIGEM
    caminho = [origem] + [pontos[i] for i in ordem] + [origem]
    xs, ys, zs = ([p[k] / 100.0 for p in caminho] for k in range(3))
    ax.plot(xs, ys, zs, color=cor, linewidth=1.8)
    ax.plot(xs, ys, [0] * len(zs), color=EIXO, linewidth=0.7, linestyle=':')
    px, py, pz = ([p[k] / 100.0 for p in pontos] for k in range(3))
    ax.scatter(px, py, pz, color=TEXTO, s=8, depthshade=False)
    ax.scatter([0], [0], [origem[2] / 100.0], color=TEXTO, marker='s', s=26, depthshade=False)
    if len(pontos) <= 10:
        for passo, i in enumerate(ordem, 1):
            ax.text(px[i], py[i], pz[i] + 0.07, str(passo), fontsize=6.5, color=TEXTO2)
    d = rotas.custo(ordem, pontos) / 100.0
    extra = 'menor' if d <= ref + 1e-9 else f'+{100 * (d / ref - 1):.1f} %'
    ax.set_title(f'{rot}: {d:.2f} m ({extra})'.replace('.', ','), color=TEXTO, fontsize=8.5, pad=0)
    ax.set_xlabel('x (m)', color=TEXTO2, fontsize=7, labelpad=-6)
    ax.set_ylabel('y (m)', color=TEXTO2, fontsize=7, labelpad=-6)
    ax.set_zlabel('z (m)', color=TEXTO2, fontsize=7, labelpad=-8)
    ax.set_zlim(0, 1.6)
    ax.tick_params(labelsize=6, colors=TEXTO2, pad=-3)
    ax.view_init(elev=26, azim=-58)


def rotas_3d(saida, n, semente, nomes, arquivo, aviso=None):
    pontos = rotas.pontos_aleatorios(n, semente)
    ordens = {nome: rotas.ordenar(nome, pontos) for nome in nomes}
    ref = min(rotas.custo(o, pontos) for o in ordens.values()) / 100.0
    fig = plt.figure(figsize=(LARGURA, LARGURA * 0.86), facecolor=FUNDO)
    paineis = nomes + ([None] if aviso else [])
    for k, nome in enumerate(paineis, 1):
        if nome is None:
            ax = fig.add_subplot(2, 2, k)
            ax.axis('off')
            ax.text(
                0.5, 0.5, aviso, ha='center', va='center', fontsize=8.5, color=TEXTO2, wrap=True
            )
            continue
        ax = fig.add_subplot(2, 2, k, projection='3d')
        ax.set_facecolor(FUNDO)
        _rota_3d(ax, pontos, ordens[nome], nome, ref)
    fig.subplots_adjust(left=0.0, right=0.97, top=0.95, bottom=0.03, wspace=0.08, hspace=0.12)
    _salvar(fig, saida, arquivo)
    return {
        nome: {
            'distancia_m': round(rotas.custo(ordens[nome], pontos) / 100.0, 2),
            'ordem': [i + 1 for i in ordens[nome]],
        }
        for nome in nomes
    }


# ---- convergência do AG -------------------------------------------------------------------


def convergencia(saida, conv):
    ns = ['10', '15', '20', '30']
    fig, eixos = plt.subplots(2, 2, figsize=(LARGURA, 4.4), facecolor=FUNDO, sharey=True)
    for ax, n in zip(eixos.flat, ns):
        c = conv[n]
        ref = c['referencia_cm']
        hist = [100 * (v / ref - 1) for v in c['genetico']]
        _eixo(ax, 'geração', 'acima da referência (%)')
        ax.set_title(
            f'N = {n} (referência: {"ótimo global" if c["referencia"] == "otimo" else "melhor rota conhecida"})',
            color=TEXTO,
            fontsize=8.5,
        )
        rot, cor, _ = ESTILO['genetico']
        ax.plot(range(len(hist)), hist, color=cor, linewidth=1.8, label=rot)
        rot, cor, _ = ESTILO['guloso']
        ax.axhline(
            100 * (c['guloso_cm'] / ref - 1), color=cor, linewidth=1.4, linestyle=':', label=rot
        )
        ax.axhline(0, color=TEXTO, linewidth=0.8, linestyle='--')
        ax.set_ylim(-2, 60)
    eixos[0][0].legend(frameon=False, loc='upper right')
    fig.tight_layout()
    _salvar(fig, saida, 'convergencia_genetico.png')


def main() -> int:
    saida = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else RAIZ / 'docs/figuras_tcc'
    saida.mkdir(parents=True, exist_ok=True)
    resumo, limite, linhas, conv = _dados()
    tempo_resolver(saida, resumo, limite)
    memoria_exato(saida, limite)
    numeros = {'qualidade': qualidade_rotas(saida, linhas)}
    todos = ['aleatorio', 'guloso', 'genetico', 'exato']
    numeros['n10'] = rotas_3d(saida, 10, 2, todos, 'rotas_3d_n10.png')
    numeros['n20'] = rotas_3d(saida, 20, 10, todos, 'rotas_3d_n20.png')
    numeros['n25'] = rotas_3d(
        saida,
        25,
        3,
        ['aleatorio', 'guloso', 'genetico'],
        'rotas_3d_n25.png',
        aviso='Held-Karp não executado:\na tabela de estados precisaria\n'
        'de 7,5 GB de RAM\n(o computador tem 7,3 GB)',
    )
    convergencia(saida, conv)
    print(json.dumps(numeros, indent=1, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
