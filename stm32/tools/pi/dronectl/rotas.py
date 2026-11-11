"""Ordem de visita de N pontos 3D (problema do caixeiro-viajante, TSP).

O drone decola, fica estável no ponto de partida ``origem`` (x = y = 0, na
altura da decolagem), passa por todos os pontos (x, y, z) e volta à origem.
O custo de uma rota é o comprimento do circuito em 3D, em cm.

Algoritmos (Python puro, sem numpy, para rodar igual no PC e na Pi Zero):

- ``aleatorio``: uma permutação sorteada — a referência "sem algoritmo".
- ``guloso``: vizinho mais próximo. Construtivo, O(n²), sem garantia.
- ``guloso+2opt``: o guloso melhorado por 2-opt até um **ótimo local**
  (nenhuma inversão de trecho encurta mais a rota).
- ``2opt-multi``: 2-opt a partir de várias rotas aleatórias; cada partida cai
  num ótimo local, e fica o melhor deles. Mostra que ótimos locais diferentes
  existem e que a partida importa.
- ``genetico``: algoritmo genético (população de permutações, torneio,
  cruzamento OX, mutação por inversão, elitismo) — busca global estocástica.
- ``forca-bruta``: testa as n! ordens possíveis: o **ótimo global** pelo
  caminho mais direto, em O(n · n!) de tempo e memória desprezível. Só para
  poucos pontos (10 pontos = 3,6 milhões de ordens).
- ``exato``: Held-Karp (programação dinâmica sobre subconjuntos): o mesmo
  **ótimo global**, em O(n² 2ⁿ) de tempo e O(n 2ⁿ) de memória.

Todos os sorteios usam só ``random()`` e as distâncias ``math.sqrt``, que dão
o mesmo resultado em qualquer versão do Python 3 e em qualquer máquina: com a
mesma semente, PC e Raspberry Pi chegam nas mesmas rotas.
"""

from __future__ import annotations

import math
import random
from array import array
from dataclasses import asdict, dataclass
from itertools import permutations
from typing import Callable, Dict, List, Sequence, Tuple

Ponto = Tuple[float, float, float]
Ordem = List[int]

#: Ponto de partida padrão: em cima da decolagem, a 50 cm (cm).
ORIGEM: Ponto = (0.0, 0.0, 50.0)


def _p3(p: Sequence[float]) -> Ponto:
    """Aceita (x, y) ou (x, y, z); sem z, usa a altura da origem."""
    return (float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else ORIGEM[2])


def distancia(a: Sequence[float], b: Sequence[float]) -> float:
    """Distância 3D. sqrt é arredondada igual em qualquer máquina (IEEE 754)."""
    a, b = _p3(a), _p3(b)
    dx, dy, dz = a[0] - b[0], a[1] - b[1], a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _matriz(pontos: Sequence[Sequence[float]], origem: Ponto) -> List[List[float]]:
    """Distâncias entre a origem (índice 0) e os pontos (1..n)."""
    todos = [origem] + [_p3(p) for p in pontos]
    return [[distancia(a, b) for b in todos] for a in todos]


def custo(ordem: Sequence[int], pontos: Sequence[Sequence[float]], origem: Ponto = ORIGEM) -> float:
    """Comprimento do circuito origem -> pontos na ordem -> origem, em cm."""
    caminho = [origem] + [_p3(pontos[i]) for i in ordem] + [origem]
    return sum(distancia(a, b) for a, b in zip(caminho, caminho[1:]))


# ---- sorteios reprodutíveis ---------------------------------------------------
# O Python só garante a mesma sequência de random() entre versões; shuffle,
# sample e randint podem mudar. Tudo aqui sai só de random().


def _indice(rng: random.Random, n: int) -> int:
    """Inteiro uniforme em [0, n)."""
    return min(int(rng.random() * n), n - 1)


def _embaralhar(rng: random.Random, lista: List[int]) -> None:
    """Fisher-Yates."""
    for i in range(len(lista) - 1, 0, -1):
        j = _indice(rng, i + 1)
        lista[i], lista[j] = lista[j], lista[i]


def _dois_distintos(rng: random.Random, n: int) -> Tuple[int, int]:
    """Dois índices distintos em [0, n), em ordem crescente."""
    a = _indice(rng, n)
    b = _indice(rng, n - 1)
    if b >= a:
        b += 1
    return (a, b) if a < b else (b, a)


def _amostra(rng: random.Random, n: int, k: int) -> List[int]:
    """k índices distintos em [0, n) (Fisher-Yates parcial)."""
    pool = list(range(n))
    for i in range(k):
        j = i + _indice(rng, n - i)
        pool[i], pool[j] = pool[j], pool[i]
    return pool[:k]


def _comprimento(d: List[List[float]], ordem: Sequence[int]) -> float:
    """Comprimento com a matriz (ordem em índices de pontos 0..n-1)."""
    c, a = 0.0, 0
    for g in ordem:
        c += d[a][g + 1]
        a = g + 1
    return c + d[a][0]


# ---- referência e construtivos ------------------------------------------------


def aleatorio(pontos: Sequence[Sequence[float]], semente: int = 1) -> Ordem:
    """Uma permutação sorteada: o que se espera sem algoritmo nenhum."""
    ordem = list(range(len(pontos)))
    _embaralhar(random.Random(semente), ordem)
    return ordem


def guloso(pontos: Sequence[Sequence[float]], origem: Ponto = ORIGEM) -> Ordem:
    """Vizinho mais próximo: da origem, vai sempre ao ponto não visitado mais perto.

    Empate: o de menor índice (determinístico). O(n²).
    """
    d = _matriz(pontos, origem)
    restantes = set(range(len(pontos)))
    atual, ordem = 0, []
    while restantes:
        prox = min(restantes, key=lambda i: (d[atual][i + 1], i))
        ordem.append(prox)
        restantes.remove(prox)
        atual = prox + 1
    return ordem


# ---- busca local (ótimo local) ---------------------------------------------------


def dois_opt(
    ordem: Sequence[int], pontos: Sequence[Sequence[float]], origem: Ponto = ORIGEM
) -> Ordem:
    """Melhoria 2-opt até um ótimo local.

    Vizinhança 2-opt: remover duas arestas (a, b) e (c, e) e religar como
    (a, c) e (b, e), o que equivale a inverter o trecho b..c. Aplica a primeira
    inversão que encurta e repete até nenhuma encurtar: a rota final é um
    ótimo **local** (nenhum vizinho é melhor), não necessariamente o global.
    """
    d = _matriz(pontos, origem)
    tour = [0] + [i + 1 for i in ordem] + [0]
    melhorou = True
    while melhorou:
        melhorou = False
        for i in range(1, len(tour) - 2):
            for j in range(i + 1, len(tour) - 1):
                a, b, c, e = tour[i - 1], tour[i], tour[j], tour[j + 1]
                if d[a][c] + d[b][e] < d[a][b] + d[c][e] - 1e-9:
                    tour[i : j + 1] = reversed(tour[i : j + 1])
                    melhorou = True
    return [i - 1 for i in tour[1:-1]]


def dois_opt_multi(
    pontos: Sequence[Sequence[float]], partidas: int = 20, semente: int = 1, origem: Ponto = ORIGEM
) -> Tuple[Ordem, List[float]]:
    """2-opt a partir de ``partidas`` rotas aleatórias.

    Returns:
        (melhor ordem, comprimento do ótimo local de cada partida).
    """
    rng = random.Random(semente)
    melhor, otimos = None, []
    for _ in range(partidas):
        inicio = list(range(len(pontos)))
        _embaralhar(rng, inicio)
        ordem = dois_opt(inicio, pontos, origem)
        c = custo(ordem, pontos, origem)
        otimos.append(c)
        if melhor is None or c < melhor[1] - 1e-9:
            melhor = (ordem, c)
    return (melhor[0] if melhor else []), otimos


# ---- exatos (ótimo global) --------------------------------------------------------


def forca_bruta(pontos: Sequence[Sequence[float]], origem: Ponto = ORIGEM) -> Ordem:
    """Circuito ótimo (global) testando todas as n! ordens de visita.

    Sem atalho nenhum (nem a simetria de percorrer o circuito ao contrário):
    é a definição do problema virando código. Empate: a primeira ordem na
    ordem lexicográfica (determinístico).
    """
    n = len(pontos)
    if n == 0:
        return []
    d = _matriz(pontos, origem)
    volta = [linha[0] for linha in d]
    melhor_c, melhor = float('inf'), None
    for ordem in permutations(range(1, n + 1)):
        a = ordem[0]
        c = d[0][a]
        for b in ordem[1:]:
            c += d[a][b]
            a = b
        c += volta[a]
        if c < melhor_c - 1e-9:
            melhor_c, melhor = c, ordem
    return [i - 1 for i in melhor]


def held_karp(pontos: Sequence[Sequence[float]], origem: Ponto = ORIGEM) -> Ordem:
    """Circuito ótimo (global) por programação dinâmica de Held-Karp.

    ``C(S, j)`` = menor caminho que sai da origem, visita exatamente os pontos
    do subconjunto S e termina em j ∈ S:

        C({j}, j)  = d(0, j)
        C(S, j)    = min_{k ∈ S, k ≠ j} C(S \\ {j}, k) + d(k, j)
        ótimo      = min_j C(todos, j) + d(j, 0)

    São n·2ⁿ estados com n escolhas cada: O(n² 2ⁿ) de tempo. Os estados ficam
    num ``array('d')`` plano (8 bytes cada) e o predecessor num ``array('b')``.
    """
    n = len(pontos)
    if n == 0:
        return []
    d = _matriz(pontos, origem)
    total = 1 << n
    inf = float('inf')
    melhor = array('d', [inf]) * (total * n)
    pai = array('b', [-1]) * (total * n)
    for j in range(n):
        melhor[(1 << j) * n + j] = d[0][j + 1]
    for s in range(1, total):
        if s & (s - 1) == 0:
            continue
        base = s * n
        bits = [j for j in range(n) if s >> j & 1]
        for j in bits:
            anterior = (s ^ (1 << j)) * n
            dj = j + 1
            valor, de = inf, -1
            for k in bits:
                if k == j:
                    continue
                c = melhor[anterior + k] + d[k + 1][dj]
                if c < valor:
                    valor, de = c, k
            melhor[base + j] = valor
            pai[base + j] = de
    cheio = total - 1
    fim = min(range(n), key=lambda j: (melhor[cheio * n + j] + d[j + 1][0], j))
    ordem, s, j = [], cheio, fim
    while j != -1:
        ordem.append(j)
        s, j = s ^ (1 << j), pai[s * n + j]
    return ordem[::-1]


# ---- genético (busca global estocástica) ------------------------------------------


@dataclass(frozen=True)
class ParamsGA:
    """Parâmetros do algoritmo genético (todos registrados no benchmark)."""

    populacao: int = 120
    geracoes: int = 800
    #: Para antes se o melhor não melhorar por tantas gerações.
    paciencia: int = 200
    torneio: int = 3
    taxa_cruzamento: float = 0.9
    taxa_mutacao: float = 0.3
    elitismo: int = 2
    semente: int = 2026


def _ox(pai1: List[int], pai2: List[int], rng: random.Random) -> List[int]:
    """Cruzamento OX: o trecho [a, b] do pai 1 no lugar; o resto na ordem do pai 2."""
    n = len(pai1)
    a, b = _dois_distintos(rng, n)
    filho = [-1] * n
    filho[a : b + 1] = pai1[a : b + 1]
    usados = set(pai1[a : b + 1])
    resto = [g for g in pai2 if g not in usados]
    pos = [i for i in range(n) if filho[i] == -1]
    for i, g in zip(pos, resto):
        filho[i] = g
    return filho


def genetico(
    pontos: Sequence[Sequence[float]], params: ParamsGA = ParamsGA(), origem: Ponto = ORIGEM
) -> Tuple[Ordem, Dict]:
    """Algoritmo genético.

    1. População inicial: ``populacao`` permutações aleatórias.
    2. A cada geração: os ``elitismo`` melhores passam direto; o resto nasce de
       dois pais escolhidos por torneio (o melhor de ``torneio`` sorteados),
       cruzados por OX com probabilidade ``taxa_cruzamento`` e mutados por
       inversão de um trecho com probabilidade ``taxa_mutacao``.
    3. Para após ``geracoes`` ou ``paciencia`` gerações sem melhorar.

    Returns:
        (melhor ordem, {'geracoes', 'historico' (melhor custo por geração), 'params'}).
    """
    n = len(pontos)
    if n <= 2:
        return list(range(n)), {'geracoes': 0, 'historico': [], 'params': asdict(params)}
    d = _matriz(pontos, origem)
    rng = random.Random(params.semente)
    pop = []
    for _ in range(params.populacao):
        ind = list(range(n))
        _embaralhar(rng, ind)
        pop.append(ind)
    aptidao = [_comprimento(d, i) for i in pop]
    melhor_c = min(aptidao)
    melhor = list(pop[aptidao.index(melhor_c)])
    historico = [melhor_c]
    sem_melhora, geracao = 0, 0

    def torneio() -> List[int]:
        competidores = _amostra(rng, params.populacao, params.torneio)
        return pop[min(competidores, key=lambda i: (aptidao[i], i))]

    for geracao in range(1, params.geracoes + 1):
        ordem_pop = sorted(range(params.populacao), key=lambda i: (aptidao[i], i))
        nova = [list(pop[i]) for i in ordem_pop[: params.elitismo]]
        while len(nova) < params.populacao:
            p1, p2 = torneio(), torneio()
            filho = _ox(p1, p2, rng) if rng.random() < params.taxa_cruzamento else list(p1)
            if rng.random() < params.taxa_mutacao:
                a, b = _dois_distintos(rng, n)
                filho[a : b + 1] = reversed(filho[a : b + 1])
            nova.append(filho)
        pop = nova
        aptidao = [_comprimento(d, i) for i in pop]
        c = min(aptidao)
        if c < melhor_c - 1e-9:
            melhor_c, melhor, sem_melhora = c, list(pop[aptidao.index(c)]), 0
        else:
            sem_melhora += 1
        historico.append(melhor_c)
        if sem_melhora >= params.paciencia:
            break
    return melhor, {'geracoes': geracao, 'historico': historico, 'params': asdict(params)}


#: Nome -> função que devolve a ordem (origem padrão).
ALGORITMOS: Dict[str, Callable[[Sequence[Sequence[float]]], Ordem]] = {
    'aleatorio': aleatorio,
    'guloso': guloso,
    'guloso+2opt': lambda p: dois_opt(guloso(p), p),
    '2opt-multi': lambda p: dois_opt_multi(p)[0],
    'genetico': lambda p: genetico(p)[0],
    'forca-bruta': forca_bruta,
    'exato': held_karp,
}


def ordenar(nome: str, pontos: Sequence[Sequence[float]], origem: Ponto = ORIGEM) -> Ordem:
    """Ordem de visita pelo algoritmo ``nome`` (chave de :data:`ALGORITMOS`) a partir de ``origem``."""
    if nome == 'aleatorio':
        return aleatorio(pontos)
    if nome == 'guloso':
        return guloso(pontos, origem)
    if nome == 'guloso+2opt':
        return dois_opt(guloso(pontos, origem), pontos, origem)
    if nome == '2opt-multi':
        return dois_opt_multi(pontos, origem=origem)[0]
    if nome == 'genetico':
        return genetico(pontos, origem=origem)[0]
    if nome == 'forca-bruta':
        return forca_bruta(pontos, origem)
    if nome == 'exato':
        return held_karp(pontos, origem)
    raise ValueError(f'algoritmo desconhecido: {nome}')


def pontos_aleatorios(
    n: int, semente: int, lado_cm: int = 600, z_min_cm: int = 30, z_max_cm: int = 150
) -> List[Ponto]:
    """N pontos inteiros (cm): x, y num quadrado de ``lado_cm`` centrado na
    decolagem e altura z entre ``z_min_cm`` e ``z_max_cm`` (limites do STM32:
    acima da altura de pouso e no máximo 1,5 m)."""
    rng = random.Random(semente)
    meio = lado_cm // 2
    pontos = set()
    while len(pontos) < n:
        p = (
            _indice(rng, lado_cm + 1) - meio,
            _indice(rng, lado_cm + 1) - meio,
            z_min_cm + _indice(rng, z_max_cm - z_min_cm + 1),
        )
        if (p[0], p[1]) != (0, 0):
            pontos.add(p)
    return sorted(pontos)
