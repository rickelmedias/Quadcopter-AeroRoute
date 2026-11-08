"""Figuras de apresentação das rotas (roda no PC; precisa de matplotlib).

    python3 tools/pi/rotas_apresentacao.py

Gera em ``docs/apresentacao_rotas/``:

- ``rotas_3d_n10.png``, ``rotas_3d_n15.png``: a mesma instância resolvida pelo
  guloso, pelo genético e pelo exato, em 3D, com a distância de cada rota;
- ``rotas_3d_n25.png``: 25 pontos, onde o exato já não roda (aleatório, guloso,
  genético);
- ``distancia_media.png``: distância média do circuito por algoritmo e N;
- ``tempo_resolver.png``: tempo para resolver por N (escala log), com os dois
  exatos (força bruta e Held-Karp) medidos até onde rodam (``limite_exato.json``)
  e previstos daí em diante;
- ``memoria_exato.png``: memória do exato por N (calculada e pico medido)
  contra a RAM da Pi Zero e a livre e total do PC.

Os números vêm de ``docs/resultados_rotas/pc`` (benchmark ``rotas-bench``) e das
próprias funções de ``dronectl/rotas.py``.
"""
from __future__ import annotations

import csv
import json
import sys
import time
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

RAIZ = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RAIZ / 'tools/pi'))
from dronectl import rotas  # noqa: E402

DADOS = RAIZ / 'docs/resultados_rotas/pc'
SAIDA = RAIZ / 'docs/apresentacao_rotas'

#: Mesmas cores e marcadores dos gráficos do benchmark (paleta categórica validada).
ESTILO = {
    'aleatorio': ('Aleatório', '#008300', 'x'),
    'guloso': ('Guloso', '#2a78d6', 'o'),
    'genetico': ('Algoritmo genético', '#eda100', 'D'),
    'exato': ('TSP exato (Held-Karp)', '#e87ba4', 'v'),
    'forca-bruta': ('TSP exato (força bruta)', '#6250d6', 'P'),
}
TEXTO, TEXTO2, GRADE, EIXO, FUNDO = '#0b0b0b', '#52514e', '#e1e0d9', '#c3c2b7', '#fcfcfb'
#: Memória disponível para comparar com o exato, em MB.
RAM_PI_ZERO_MB = 512
RAM_PC_MB = 8 * 1024


def _eixo2d(ax, titulo, xlabel, ylabel):
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


# ---- rotas em 3D ------------------------------------------------------------------

def _rota_3d(ax, pontos, ordem, nome, distancia_ref=None):
    rot, cor, _ = ESTILO[nome]
    origem = rotas.ORIGEM
    caminho = [origem] + [pontos[i] for i in ordem] + [origem]
    xs, ys, zs = ([p[k] / 100.0 for p in caminho] for k in range(3))
    ax.plot(xs, ys, zs, color=cor, linewidth=2)
    # Sombra no chão: ajuda a ler a profundidade.
    ax.plot(xs, ys, [0] * len(zs), color=EIXO, linewidth=0.8, linestyle=':')
    px, py, pz = ([p[k] / 100.0 for p in pontos] for k in range(3))
    ax.scatter(px, py, pz, color=TEXTO, s=14, depthshade=False)
    ax.scatter([0], [0], [origem[2] / 100.0], color=TEXTO, marker='s', s=40, depthshade=False)
    ax.text(0, 0, origem[2] / 100.0 + 0.12, 'partida', fontsize=7, color=TEXTO)
    for passo, i in enumerate(ordem, 1):
        ax.text(px[i], py[i], pz[i] + 0.06, str(passo), fontsize=6, color=TEXTO2)
    d = rotas.custo(ordem, pontos) / 100.0
    extra = '' if distancia_ref is None else f'  (+{100 * (d / distancia_ref - 1):.1f} %)'
    ax.set_title(f'{rot}\n{d:.2f} m{extra}'.replace('.', ','), color=TEXTO, fontsize=10,
                 fontweight='bold')
    ax.set_xlabel('x (m)', color=TEXTO2, fontsize=8)
    ax.set_ylabel('y (m)', color=TEXTO2, fontsize=8)
    ax.set_zlabel('altura (m)', color=TEXTO2, fontsize=8)
    ax.set_zlim(0, 1.6)
    ax.tick_params(labelsize=7, colors=TEXTO2)
    ax.view_init(elev=24, azim=-58)


def figura_3d(n, semente, nomes, arquivo, titulo):
    pontos = rotas.pontos_aleatorios(n, semente)
    ordens, tempos = {}, {}
    for nome in nomes:
        t0 = time.perf_counter()
        ordens[nome] = rotas.ordenar(nome, pontos)
        tempos[nome] = time.perf_counter() - t0
    ref = min(rotas.custo(o, pontos) for o in ordens.values()) / 100.0
    fig = plt.figure(figsize=(5.2 * len(nomes), 5.2), facecolor=FUNDO)
    for k, nome in enumerate(nomes, 1):
        ax = fig.add_subplot(1, len(nomes), k, projection='3d')
        ax.set_facecolor(FUNDO)
        _rota_3d(ax, pontos, ordens[nome], nome, ref)
    fig.suptitle(titulo + '  ·  % = acima da rota mais curta desta figura', x=0.01, ha='left',
                 color=TEXTO, fontsize=12, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(SAIDA / arquivo, dpi=200)
    plt.close(fig)
    return {nome: {'distancia_m': round(rotas.custo(ordens[nome], pontos) / 100.0, 3),
                   'tempo_ms': round(tempos[nome] * 1000.0, 2),
                   'ordem': [i + 1 for i in ordens[nome]]} for nome in nomes}


# ---- gráficos do benchmark ------------------------------------------------------------

def _medias_distancia():
    soma = defaultdict(float)
    cont = defaultdict(int)
    with open(DADOS / 'resultados.csv', encoding='utf-8') as f:
        for linha in csv.DictReader(f):
            chave = (int(linha['n']), linha['algoritmo'])
            soma[chave] += float(linha['custo_cm']) / 100.0
            cont[chave] += 1
    return {k: soma[k] / cont[k] for k in soma}


def distancia_media(arquivo):
    medias = _medias_distancia()
    ns = [5, 10, 15, 20, 22, 25, 30]
    nomes = ['aleatorio', 'guloso', 'genetico', 'forca-bruta', 'exato']
    fig, ax = plt.subplots(figsize=(10, 4.8), facecolor=FUNDO)
    _eixo2d(ax, 'Distância média do circuito (10 instâncias por N)', 'pontos a visitar (N)',
            'distância (m)')
    largura = 0.16
    for k, nome in enumerate(nomes):
        rot, cor, _ = ESTILO[nome]
        xs = [i + (k - 2) * largura for i in range(len(ns))]
        ys = [medias.get((n, nome)) for n in ns]
        for x, y in zip(xs, ys):
            if y is None:
                ax.text(x, 1.0, 'não roda', ha='center', va='bottom', fontsize=6.5, color=TEXTO2,
                        rotation=90)
        ax.bar([x for x, y in zip(xs, ys) if y is not None], [y for y in ys if y is not None],
               width=largura, color=cor, edgecolor=FUNDO, linewidth=1, label=rot)
    ax.set_xticks(range(len(ns)), [str(n) for n in ns])
    ax.legend(frameon=False, fontsize=8, loc='upper left')
    fig.tight_layout()
    fig.savefig(SAIDA / arquivo, dpi=200)
    plt.close(fig)
    return {f'{nome}@{n}': round(medias[(n, nome)], 2) for n in ns for nome in nomes
            if (n, nome) in medias}


def _limite():
    """Experimento de limite do exato (``dronectl rotas-limite``)."""
    return json.loads((DADOS / 'limite_exato.json').read_text(encoding='utf-8'))


def _anos(segundos):
    anos = segundos / (365.25 * 86400)
    if anos >= 1e6:
        return f'{anos:.0e} anos'.replace('e+0', ' × 10^').replace('e+', ' × 10^')
    if anos >= 1000:
        return f'{anos / 1000:.0f} mil anos'
    return f'{anos:.0f} anos'


def tempo_resolver(arquivo, resumo, limite):
    tab = resumo['tabela']
    corte = max(m['n'] for m in limite['medidos'] if m['tempo_s'] <= limite['tempo_max_s'])
    corte_fb = max(t['n'] for t in tab if t['algoritmo'] == 'forca-bruta')
    fb = limite['forca_bruta']
    teto = 1e8
    fig, ax = plt.subplots(figsize=(10, 6.2), facecolor=FUNDO)
    _eixo2d(ax, 'Tempo para resolver uma instância (PC Intel i5-9300HF)', 'pontos a visitar (N)',
            'tempo (s, escala log)')
    ax.set_yscale('log')
    for nome in ('guloso', 'genetico'):
        rot, cor, marca = ESTILO[nome]
        pts = [(t['n'], t['tempo_medio_ms'] / 1000.0) for t in tab if t['algoritmo'] == nome]
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color=cor, linewidth=2, marker=marca, markersize=5,
                markeredgecolor=FUNDO, markeredgewidth=0.8, label=rot + ' (medido, média)')
    rot, cor, marca = ESTILO['exato']
    med = limite['medidos']
    ax.plot([m['n'] for m in med], [m['tempo_s'] for m in med], color=cor, linewidth=2,
            marker=marca, markersize=5, markeredgecolor=FUNDO, label=rot + ' (medido)')
    prev = [(med[-1]['n'], med[-1]['tempo_s'])] + [(p['n'], p['tempo_s']) for p in limite['previsao']]
    ax.plot([p[0] for p in prev], [p[1] for p in prev], color=cor, linewidth=2, linestyle='--',
            label='TSP exato, Held-Karp (previsto: c·N²·2ᴺ; não cabe na RAM)')
    rot, cor, marca = ESTILO['forca-bruta']
    fbm = fb['medidos']
    ax.plot([m['n'] for m in fbm], [m['tempo_s'] for m in fbm], color=cor, linewidth=2,
            marker=marca, markersize=6, markeredgecolor=FUNDO, label=rot + ' (medido)')
    fbp = [(fbm[-1]['n'], fbm[-1]['tempo_s'])] + [(p['n'], p['tempo_s']) for p in fb['previsao']]
    ax.plot([p[0] for p in fbp], [p[1] for p in fbp], color=cor, linewidth=2, linestyle='--',
            label='TSP exato, força bruta (previsto: c·N!)')
    # A previsão sai do gráfico por cima; o rótulo fica ao lado de onde ela sai.
    sai = next(p['n'] for p in fb['previsao'] if p['tempo_s'] > teto)
    fb20 = next(p['tempo_s'] for p in fb['previsao'] if p['n'] == 20)
    ax.text(sai - 0.4, teto / 4, f'força bruta, 20 pontos:\n≈ {_anos(fb20)}', ha='left', va='top',
            fontsize=8, color=TEXTO)
    for segundos, texto in ((1, '1 s'), (60, '1 min (critério de corte)'), (3600, '1 h'),
                            (86400, '1 dia'), (365.25 * 86400, '1 ano')):
        ax.axhline(segundos, color=EIXO, linewidth=1, linestyle=':')
        ax.text(3.2, segundos * 1.15, texto, fontsize=8, color=TEXTO2)
    for x, texto in ((corte_fb, f'força bruta\nfora do\ncomparativo\n(N ≥ {corte_fb + 1})'),
                     (corte, f'Held-Karp\nfora do\ncomparativo\n(N ≥ {corte + 1})')):
        ax.axvline(x + 0.5, color=TEXTO2, linewidth=1, linestyle='--')
        ax.text(x + 0.7, 1.5e-6, texto, fontsize=8, color=TEXTO2)
    ga30 = next(t['tempo_medio_ms'] for t in tab if t['algoritmo'] == 'genetico' and t['n'] == 30)
    ax.annotate(f'genético, 30 pontos: {ga30 / 1000:.2f} s'.replace('.', ','), (30, ga30 / 1000.0),
                xytext=(-6, 10), textcoords='offset points', ha='right', fontsize=8, color=TEXTO)
    t30 = next(p['tempo_s'] for p in limite['previsao'] if p['n'] == 30)
    ax.annotate(f'Held-Karp, 30 pontos: ≈ {t30 / 3600:.0f} h (e 290 GB)', (30, t30),
                xytext=(-10, 6), textcoords='offset points', ha='right', fontsize=8, color=TEXTO)
    ax.set_xlim(2.5, 31)
    ax.set_ylim(top=teto)
    ax.legend(frameon=False, fontsize=8, loc='upper center', bbox_to_anchor=(0.5, -0.13), ncol=2)
    fig.tight_layout()
    fig.savefig(SAIDA / arquivo, dpi=200)
    plt.close(fig)
    return {'corte_n': corte, 'corte_forca_bruta_n': corte_fb,
            'forca_bruta_medido_s': {m['n']: round(m['tempo_s'], 3) for m in fbm if m['n'] >= 8},
            'forca_bruta_previsto_s': {p['n']: p['tempo_s'] for p in fb['previsao']
                                       if p['n'] in (12, 13, 15, 20, 25, 30)},
            'genetico_30_s': round(ga30 / 1000.0, 3),
            'exato_medido_s': {m['n']: round(m['tempo_s'], 2) for m in med if m['n'] >= 15},
            'exato_previsto_s': {p['n']: round(p['tempo_s'], 1) for p in limite['previsao']
                                 if p['n'] in (24, 25, 30)}}


def memoria_exato(arquivo, limite):
    ns = list(range(3, 31))
    mem = [n * 2 ** n * 9 / 1e6 for n in ns]  # MB: n·2ⁿ estados de 8 B + 1 B do predecessor
    ga = [(120 * 2 * n * 8) / 1e6 for n in ns]  # duas populações de 120 permutações
    fig, ax = plt.subplots(figsize=(10, 4.8), facecolor=FUNDO)
    _eixo2d(ax, 'Memória necessária', 'pontos a visitar (N)', 'memória (MB, escala log)')
    ax.set_yscale('log')
    rot, cor, marca = ESTILO['exato']
    ax.plot(ns, mem, color=cor, linewidth=2, linestyle='--', label=rot + ' (N·2ᴺ estados, calculado)')
    med = [m for m in limite['medidos'] if m['n'] >= 16]  # abaixo disso domina o próprio Python
    ax.plot([m['n'] for m in med], [m['pico_mb'] for m in med], color=cor, linewidth=0,
            marker=marca, markersize=6, markeredgecolor=FUNDO, label=rot + ' (pico medido do processo)')
    rot, cor, marca = ESTILO['genetico']
    ax.plot(ns, ga, color=cor, linewidth=2, marker=marca, markersize=4, markeredgecolor=FUNDO,
            label=rot + ' (população)')
    for mb, texto in ((RAM_PI_ZERO_MB, 'RAM da Raspberry Pi Zero (512 MB)'),
                      (limite['ram_livre_mb'],
                       f"RAM livre do PC ({limite['ram_livre_mb'] / 1000:.1f} GB)".replace('.', ',')),
                      (limite['ram_total_mb'],
                       f"RAM total do PC ({limite['ram_total_mb'] / 1000:.1f} GB)".replace('.', ','))):
        ax.axhline(mb, color=TEXTO2, linewidth=1, linestyle=':')
        ax.text(3.2, mb * 1.3, texto, fontsize=8, color=TEXTO2)
    ax.legend(frameon=False, fontsize=8, loc='center left', bbox_to_anchor=(0.0, 0.42))
    ax.set_xlim(2.5, 31)
    fig.tight_layout()
    fig.savefig(SAIDA / arquivo, dpi=200)
    plt.close(fig)
    return {'exato_mb': {n: round(n * 2 ** n * 9 / 1e6, 1) for n in (15, 20, 22, 23, 24, 25, 30)},
            'pico_medido_mb': {m['n']: round(m['pico_mb'], 1) for m in med}}


def main() -> int:
    SAIDA.mkdir(parents=True, exist_ok=True)
    resumo = json.loads((DADOS / 'resumo.json').read_text(encoding='utf-8'))
    limite = _limite()
    numeros = {
        'rotas_3d_n10': figura_3d(10, 2, ['guloso', 'genetico', 'exato'], 'rotas_3d_n10.png',
                                  '10 pontos (semente 2): mesma instância, três algoritmos'),
        'rotas_3d_n15': figura_3d(15, 10, ['guloso', 'genetico', 'exato'], 'rotas_3d_n15.png',
                                  '15 pontos (semente 10): mesma instância, três algoritmos'),
        'rotas_3d_n25': figura_3d(25, 3, ['aleatorio', 'guloso', 'genetico'], 'rotas_3d_n25.png',
                                  '25 pontos (semente 3): o Held-Karp precisaria de 7,5 GB de RAM'),
        'distancia_media_m': distancia_media('distancia_media.png'),
        'tempo': tempo_resolver('tempo_resolver.png', resumo, limite),
        'memoria': memoria_exato('memoria_exato.png', limite),
    }
    (SAIDA / 'numeros.json').write_text(json.dumps(numeros, indent=2, ensure_ascii=False),
                                        encoding='utf-8')
    print(json.dumps(numeros, indent=1, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
