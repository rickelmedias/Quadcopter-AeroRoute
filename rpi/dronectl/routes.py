"""Ordem de visita de N pontos 3D (problema do caixeiro-viajante, TSP).

O drone decola, fica estável no ponto de partida ``origin`` (x = y = 0, na
altura da decolagem), passa por todos os pontos (x, y, z) e volta à origem.
O custo de uma rota é o comprimento do circuito em 3D, em cm.

Algoritmos (Python puro, sem numpy, para rodar igual no PC e na Pi Zero):

- ``random``: uma permutação sorteada, a referência "sem algoritmo".
- ``greedy``: vizinho mais próximo. Construtivo, O(n²), sem garantia.
- ``greedy+2opt``: o guloso melhorado por 2-opt até um **ótimo local**
  (nenhuma inversão de trecho encurta mais a rota).
- ``2opt-multi``: 2-opt a partir de várias rotas aleatórias; cada partida cai
  num ótimo local, e fica o melhor deles.
- ``genetic``: algoritmo genético (população de permutações, torneio,
  cruzamento OX, mutação por inversão, elitismo), busca global estocástica.
- ``brute-force``: testa as n! ordens possíveis: o **ótimo global** pelo
  caminho mais direto, em O(n · n!) de tempo. Só para poucos pontos.
- ``exact``: Held-Karp (programação dinâmica sobre subconjuntos): o mesmo
  **ótimo global**, em O(n² 2ⁿ) de tempo e O(n 2ⁿ) de memória.

Todos os sorteios usam só ``random()`` e as distâncias ``math.sqrt``, que dão
o mesmo resultado em qualquer versão do Python 3 e em qualquer máquina: com a
mesma semente, PC e Raspberry Pi chegam nas mesmas rotas. É a tradução direta
de ``rotas.py`` da submissão (mesmas operações na mesma ordem), com os nomes em
inglês: as rotas do trabalho se reproduzem iguais.
"""

from __future__ import annotations

import math
import random
from array import array
from dataclasses import asdict, dataclass
from itertools import permutations
from typing import Callable, Dict, List, Sequence, Tuple

Point = Tuple[float, float, float]
Order = List[int]

#: Ponto de partida padrão: em cima da decolagem, a 50 cm (cm).
ORIGIN: Point = (0.0, 0.0, 50.0)


def _p3(p: Sequence[float]) -> Point:
    """Aceita (x, y) ou (x, y, z); sem z, usa a altura da origem."""
    return (float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else ORIGIN[2])


def distance(a: Sequence[float], b: Sequence[float]) -> float:
    """Distância 3D. sqrt é arredondada igual em qualquer máquina (IEEE 754)."""
    a, b = _p3(a), _p3(b)
    dx, dy, dz = a[0] - b[0], a[1] - b[1], a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _matrix(points: Sequence[Sequence[float]], origin: Point) -> List[List[float]]:
    """Distâncias entre a origem (índice 0) e os pontos (1..n)."""
    every = [origin] + [_p3(p) for p in points]
    return [[distance(a, b) for b in every] for a in every]


def cost(order: Sequence[int], points: Sequence[Sequence[float]], origin: Point = ORIGIN) -> float:
    """Comprimento do circuito origem -> pontos na ordem -> origem, em cm."""
    path = [origin] + [_p3(points[i]) for i in order] + [origin]
    return sum(distance(a, b) for a, b in zip(path, path[1:]))


# ---- sorteios reprodutíveis ---------------------------------------------------
# O Python só garante a mesma sequência de random() entre versões; shuffle,
# sample e randint podem mudar. Tudo aqui sai só de random().


def _index(rng: random.Random, n: int) -> int:
    """Inteiro uniforme em [0, n)."""
    return min(int(rng.random() * n), n - 1)


def _shuffle(rng: random.Random, items: List[int]) -> None:
    """Fisher-Yates."""
    for i in range(len(items) - 1, 0, -1):
        j = _index(rng, i + 1)
        items[i], items[j] = items[j], items[i]


def _two_distinct(rng: random.Random, n: int) -> Tuple[int, int]:
    """Dois índices distintos em [0, n), em ordem crescente."""
    a = _index(rng, n)
    b = _index(rng, n - 1)
    if b >= a:
        b += 1
    return (a, b) if a < b else (b, a)


def _sample(rng: random.Random, n: int, k: int) -> List[int]:
    """k índices distintos em [0, n) (Fisher-Yates parcial)."""
    pool = list(range(n))
    for i in range(k):
        j = i + _index(rng, n - i)
        pool[i], pool[j] = pool[j], pool[i]
    return pool[:k]


def _length(d: List[List[float]], order: Sequence[int]) -> float:
    """Comprimento com a matriz (ordem em índices de pontos 0..n-1)."""
    c, a = 0.0, 0
    for g in order:
        c += d[a][g + 1]
        a = g + 1
    return c + d[a][0]


# ---- referência e construtivos ------------------------------------------------


def random_order(points: Sequence[Sequence[float]], seed: int = 1) -> Order:
    """Uma permutação sorteada: o que se espera sem algoritmo nenhum."""
    order = list(range(len(points)))
    _shuffle(random.Random(seed), order)
    return order


def greedy(points: Sequence[Sequence[float]], origin: Point = ORIGIN) -> Order:
    """Vizinho mais próximo: da origem, vai sempre ao ponto não visitado mais perto.

    Empate: o de menor índice (determinístico). O(n²).
    """
    d = _matrix(points, origin)
    left = set(range(len(points)))
    current, order = 0, []
    while left:
        nxt = min(left, key=lambda i: (d[current][i + 1], i))
        order.append(nxt)
        left.remove(nxt)
        current = nxt + 1
    return order


# ---- busca local (ótimo local) ---------------------------------------------------


def two_opt(
    order: Sequence[int], points: Sequence[Sequence[float]], origin: Point = ORIGIN
) -> Order:
    """Melhoria 2-opt até um ótimo local.

    Vizinhança 2-opt: remover duas arestas (a, b) e (c, e) e religar como
    (a, c) e (b, e), o que equivale a inverter o trecho b..c. Aplica a primeira
    inversão que encurta e repete até nenhuma encurtar: a rota final é um
    ótimo **local** (nenhum vizinho é melhor), não necessariamente o global.
    """
    d = _matrix(points, origin)
    tour = [0] + [i + 1 for i in order] + [0]
    improved = True
    while improved:
        improved = False
        for i in range(1, len(tour) - 2):
            for j in range(i + 1, len(tour) - 1):
                a, b, c, e = tour[i - 1], tour[i], tour[j], tour[j + 1]
                if d[a][c] + d[b][e] < d[a][b] + d[c][e] - 1e-9:
                    tour[i : j + 1] = reversed(tour[i : j + 1])
                    improved = True
    return [i - 1 for i in tour[1:-1]]


def two_opt_multi(
    points: Sequence[Sequence[float]], starts: int = 20, seed: int = 1, origin: Point = ORIGIN
) -> Tuple[Order, List[float]]:
    """2-opt a partir de ``starts`` rotas aleatórias.

    Returns:
        (melhor ordem, comprimento do ótimo local de cada partida).
    """
    rng = random.Random(seed)
    best, optima = None, []
    for _ in range(starts):
        start = list(range(len(points)))
        _shuffle(rng, start)
        order = two_opt(start, points, origin)
        c = cost(order, points, origin)
        optima.append(c)
        if best is None or c < best[1] - 1e-9:
            best = (order, c)
    return (best[0] if best else []), optima


# ---- exatos (ótimo global) --------------------------------------------------------


def brute_force(points: Sequence[Sequence[float]], origin: Point = ORIGIN) -> Order:
    """Circuito ótimo (global) testando todas as n! ordens de visita.

    Sem atalho nenhum (nem a simetria de percorrer o circuito ao contrário):
    é a definição do problema virando código. Empate: a primeira ordem na
    ordem lexicográfica (determinístico).
    """
    n = len(points)
    if n == 0:
        return []
    d = _matrix(points, origin)
    back = [row[0] for row in d]
    best_c, best = float('inf'), None
    for order in permutations(range(1, n + 1)):
        a = order[0]
        c = d[0][a]
        for b in order[1:]:
            c += d[a][b]
            a = b
        c += back[a]
        if c < best_c - 1e-9:
            best_c, best = c, order
    return [i - 1 for i in best]


def held_karp(points: Sequence[Sequence[float]], origin: Point = ORIGIN) -> Order:
    """Circuito ótimo (global) por programação dinâmica de Held-Karp.

    ``C(S, j)`` = menor caminho que sai da origem, visita exatamente os pontos
    do subconjunto S e termina em j ∈ S:

        C({j}, j)  = d(0, j)
        C(S, j)    = min_{k ∈ S, k ≠ j} C(S \\ {j}, k) + d(k, j)
        ótimo      = min_j C(todos, j) + d(j, 0)

    São n·2ⁿ estados com n escolhas cada: O(n² 2ⁿ) de tempo. Os estados ficam
    num ``array('d')`` plano (8 bytes cada) e o predecessor num ``array('b')``.
    """
    n = len(points)
    if n == 0:
        return []
    d = _matrix(points, origin)
    total = 1 << n
    inf = float('inf')
    best = array('d', [inf]) * (total * n)
    parent = array('b', [-1]) * (total * n)
    for j in range(n):
        best[(1 << j) * n + j] = d[0][j + 1]
    for s in range(1, total):
        if s & (s - 1) == 0:
            continue
        base = s * n
        bits = [j for j in range(n) if s >> j & 1]
        for j in bits:
            previous = (s ^ (1 << j)) * n
            dj = j + 1
            value, came = inf, -1
            for k in bits:
                if k == j:
                    continue
                c = best[previous + k] + d[k + 1][dj]
                if c < value:
                    value, came = c, k
            best[base + j] = value
            parent[base + j] = came
    full = total - 1
    end = min(range(n), key=lambda j: (best[full * n + j] + d[j + 1][0], j))
    order, s, j = [], full, end
    while j != -1:
        order.append(j)
        s, j = s ^ (1 << j), parent[s * n + j]
    return order[::-1]


# ---- genético (busca global estocástica) ------------------------------------------


@dataclass(frozen=True)
class GAParams:
    """Parâmetros do algoritmo genético (os do trabalho, semente 2026)."""

    population: int = 120
    generations: int = 800
    #: Para antes se o melhor não melhorar por tantas gerações.
    patience: int = 200
    tournament: int = 3
    crossover_rate: float = 0.9
    mutation_rate: float = 0.3
    elitism: int = 2
    seed: int = 2026


def _ox(parent1: List[int], parent2: List[int], rng: random.Random) -> List[int]:
    """Cruzamento OX: o trecho [a, b] do pai 1 no lugar; o resto na ordem do pai 2."""
    n = len(parent1)
    a, b = _two_distinct(rng, n)
    child = [-1] * n
    child[a : b + 1] = parent1[a : b + 1]
    used = set(parent1[a : b + 1])
    rest = [g for g in parent2 if g not in used]
    free = [i for i in range(n) if child[i] == -1]
    for i, g in zip(free, rest):
        child[i] = g
    return child


def genetic(
    points: Sequence[Sequence[float]], params: GAParams = GAParams(), origin: Point = ORIGIN
) -> Tuple[Order, Dict]:
    """Algoritmo genético.

    1. População inicial: ``population`` permutações aleatórias.
    2. A cada geração: os ``elitism`` melhores passam direto; o resto nasce de
       dois pais escolhidos por torneio (o melhor de ``tournament`` sorteados),
       cruzados por OX com probabilidade ``crossover_rate`` e mutados por
       inversão de um trecho com probabilidade ``mutation_rate``.
    3. Para após ``generations`` ou ``patience`` gerações sem melhorar.

    Returns:
        (melhor ordem, {'generations', 'history' (melhor custo por geração), 'params'}).
    """
    n = len(points)
    if n <= 2:
        return list(range(n)), {'generations': 0, 'history': [], 'params': asdict(params)}
    d = _matrix(points, origin)
    rng = random.Random(params.seed)
    pop = []
    for _ in range(params.population):
        ind = list(range(n))
        _shuffle(rng, ind)
        pop.append(ind)
    fitness = [_length(d, i) for i in pop]
    best_c = min(fitness)
    best = list(pop[fitness.index(best_c)])
    history = [best_c]
    stale, generation = 0, 0

    def tournament() -> List[int]:
        rivals = _sample(rng, params.population, params.tournament)
        return pop[min(rivals, key=lambda i: (fitness[i], i))]

    for generation in range(1, params.generations + 1):
        ranked = sorted(range(params.population), key=lambda i: (fitness[i], i))
        new = [list(pop[i]) for i in ranked[: params.elitism]]
        while len(new) < params.population:
            p1, p2 = tournament(), tournament()
            child = _ox(p1, p2, rng) if rng.random() < params.crossover_rate else list(p1)
            if rng.random() < params.mutation_rate:
                a, b = _two_distinct(rng, n)
                child[a : b + 1] = reversed(child[a : b + 1])
            new.append(child)
        pop = new
        fitness = [_length(d, i) for i in pop]
        c = min(fitness)
        if c < best_c - 1e-9:
            best_c, best, stale = c, list(pop[fitness.index(c)]), 0
        else:
            stale += 1
        history.append(best_c)
        if stale >= params.patience:
            break
    return best, {'generations': generation, 'history': history, 'params': asdict(params)}


#: Nome -> função que devolve a ordem (origem padrão).
ALGORITHMS: Dict[str, Callable[[Sequence[Sequence[float]]], Order]] = {
    'random': random_order,
    'greedy': greedy,
    'greedy+2opt': lambda p: two_opt(greedy(p), p),
    '2opt-multi': lambda p: two_opt_multi(p)[0],
    'genetic': lambda p: genetic(p)[0],
    'brute-force': brute_force,
    'exact': held_karp,
}


def order(name: str, points: Sequence[Sequence[float]], origin: Point = ORIGIN) -> Order:
    """Ordem de visita pelo algoritmo ``name`` (chave de :data:`ALGORITHMS`) a partir de ``origin``."""
    if name == 'random':
        return random_order(points)
    if name == 'greedy':
        return greedy(points, origin)
    if name == 'greedy+2opt':
        return two_opt(greedy(points, origin), points, origin)
    if name == '2opt-multi':
        return two_opt_multi(points, origin=origin)[0]
    if name == 'genetic':
        return genetic(points, origin=origin)[0]
    if name == 'brute-force':
        return brute_force(points, origin)
    if name == 'exact':
        return held_karp(points, origin)
    raise ValueError(f'algoritmo desconhecido: {name}')


def random_points(
    n: int, seed: int, side_cm: int = 600, z_min_cm: int = 30, z_max_cm: int = 150
) -> List[Point]:
    """N pontos inteiros (cm): x, y num quadrado de ``side_cm`` centrado na decolagem e
    altura z entre ``z_min_cm`` e ``z_max_cm`` (as instâncias do benchmark do trabalho)."""
    rng = random.Random(seed)
    half = side_cm // 2
    points = set()
    while len(points) < n:
        p = (
            _index(rng, side_cm + 1) - half,
            _index(rng, side_cm + 1) - half,
            z_min_cm + _index(rng, z_max_cm - z_min_cm + 1),
        )
        if (p[0], p[1]) != (0, 0):
            points.add(p)
    return sorted(points)
