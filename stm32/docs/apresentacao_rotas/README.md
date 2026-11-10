# Apresentação: ordem de visita dos pontos — viabilidade do AG × TSP exato

Figuras e números para o trabalho. Avaliação completa (implementação de cada
algoritmo, metodologia, ótimos locais e global, todas as tabelas):
[`docs/rotas_tsp.md`](../rotas_tsp.md). Dados brutos:
[`docs/resultados_rotas/pc/`](../resultados_rotas/pc/).

Regerar: `python3 tools/pi/rotas_apresentacao.py` (precisa de matplotlib; usa
`resumo.json` do `rotas-bench` e `limite_exato.json` do `rotas-limite`).

## O problema em uma frase

O drone sai do ponto de partida (em cima da decolagem, a 50 cm), visita N pontos
(x, y, z) e volta; o objetivo é a **menor distância total**. Há N! ordens
possíveis — 3,6 milhões com 10 pontos, 2,7 × 10³² com 30.

## Conclusão para o trabalho

> **O TSP foi resolvido de forma exata por dois algoritmos, e os dois ficam
> inviáveis cedo.** A **força bruta** (testa as N! ordens) só entra no
> comparativo até **10 pontos** (1,5 s; com 11 já são 17 s e com 20 seriam
> ~32 mil anos). O **Held-Karp** (programação dinâmica, N²·2ᴺ), que chega ao
> mesmo ótimo, vai até **22 pontos** (~1 min por instância); **com 23 passa de
> 2 min e 1,7 GB** e sai do comparativo; **com 25 precisa de 7,5 GB e não cabe
> na RAM do PC** — fica impossível neste computador. Na Raspberry Pi Zero
> (512 MB) o limite é ainda menor: a memória passa da metade da RAM a partir de
> 21 pontos.
>
> **De 23 pontos em diante, sobram só o guloso, o aleatório e o algoritmo
> genético.** O genético resolve de 3 a 30 pontos em 0,13 a 0,45 s por
> instância (média por N), com memória desprezível e rotas em média até 3,7 %
> acima da referência (o ótimo global em 150 de 200 instâncias até 22 pontos).

### As três fases do comparativo

| Pontos | Algoritmos comparados |
|---|---|
| 3 a 10 | aleatório, guloso, genético, **força bruta**, **Held-Karp** (os dois exatos deram o mesmo ótimo nas 80 instâncias) |
| 11 a 22 | aleatório, guloso, genético, **Held-Karp** |
| 23 a 30 | aleatório, guloso, genético |

## Figuras

| Arquivo | O que mostra |
|---|---|
| `rotas_3d_n10.png` | 10 pontos: guloso 24,42 m (+26,1 %) × **genético 19,36 m = ótimo** × exato 19,36 m |
| `rotas_3d_n15.png` | 15 pontos: guloso 31,73 m (+17,3 %) × **genético 27,15 m (+0,4 %)** × exato 27,04 m |
| `rotas_3d_n25.png` | 25 pontos, **o exato não cabe na memória** (7,5 GB): aleatório 89,61 m × guloso 34,14 m × **genético 30,35 m** |
| `distancia_media.png` | distância média do circuito por algoritmo (10 instâncias por N); "não roda" onde o exato saiu |
| `tempo_resolver.png` | tempo para resolver por N (escala log): força bruta medida até 11 pontos e prevista depois (c·N!), Held-Karp medido até 23 e previsto até 30, com os dois cortes |
| `memoria_exato.png` | memória do exato (calculada e pico medido do processo) × do genético, contra a RAM da Pi Zero e a livre e total do PC |

Nas figuras 3D, cada número é a **ordem de visita**; o quadrado é a partida; a
linha pontilhada é a sombra da rota no chão; o % é quanto a rota é mais longa
que a mais curta da mesma figura.

## Números para o texto

### Limite da força bruta (medido até 11, previsto depois por c · N!)

| N | ordens | tempo | situação |
|---|---|---|---|
| 5 | 120 | 0,1 ms | ok |
| 9 | 362 880 | 0,14 s | ok |
| 10 | 3 628 800 | 1,5 s | **último no comparativo** |
| 11 | 39 916 800 | 17 s | sai do comparativo (×11 em relação ao 10) |
| 12 | 4,8 × 10⁸ | ≈ 3 min (previsto) | |
| 13 | 6,2 × 10⁹ | ≈ 43 min (previsto) | |
| 15 | 1,3 × 10¹² | ≈ 6 dias (previsto) | |
| 20 | 2,4 × 10¹⁸ | ≈ 32 mil anos (previsto) | impossível |

A memória da força bruta é constante (~22 MB, só o Python): o limite dela é o
**tempo**. A do Held-Karp é o contrário: rápido, mas a **memória** dobra a cada
ponto.

### Limite do Held-Karp (medido, uma instância por N, processo separado)

| N | tempo | pico de RAM | situação |
|---|---|---|---|
| 15 | 0,24 s | 23 MB | ok |
| 20 | 12,6 s | 194 MB | ok |
| 21 | 26,7 s | 392 MB | ok (Pi Zero: mais da metade da RAM) |
| 22 | 59–64 s | 806 MB | **último no comparativo** (~1 min) |
| 23 | **125–131 s** | **1,7 GB** | **sai do comparativo** (> 1 min) |
| 24 | ≈ 5 min (previsto) | 3,6 GB | praticamente toda a RAM livre |
| 25 | ≈ 10 min (previsto) | 7,5 GB | **não cabe na RAM total — impossível neste PC** |
| 30 | ≈ 8 h (previsto) | 290 GB | impossível |

As previsões (t = c · N² · 2ᴺ) foram conferidas rodando 20–22 pontos: erro de 2
a 10 %. Para 24+ o tempo só existiria se houvesse a memória.

### Distância média do circuito (10 instâncias por N)

% acima do ótimo global (até 22) ou do melhor encontrado (acima):

| N | aleatório | guloso | **genético** | força bruta | Held-Karp |
|---|---|---|---|---|---|
| 5 | 18,86 m (+38 %) | 14,86 m (+7,9 %) | **13,78 m (ótimo)** | 13,78 m | 13,78 m |
| 10 | 35,59 m (+83 %) | 21,04 m (+7,3 %) | **19,64 m (ótimo)** | 19,64 m | 19,64 m |
| 15 | 54,52 m (+142 %) | 24,26 m (+6,4 %) | **22,98 m (+1,0 %)** | ≈ 6 dias | 22,76 m |
| 20 | 65,12 m | 29,82 m | **25,94 m** | ≈ 32 mil anos | 25,79 m |
| 22 | 68,38 m | 31,91 m | **27,30 m** | — | 26,93 m |
| 25 | 77,76 m | 34,34 m (+17 %) | **29,76 m (+1,6 %)** | — | não cabe na RAM |
| 30 | 98,44 m | 38,04 m (+18 %) | **32,96 m (+2,2 %)** | — | não cabe na RAM |

### Tempo para resolver (média por instância, PC Intel i5-9300HF)

| N | guloso | **genético** | força bruta | Held-Karp |
|---|---|---|---|---|
| 5 | 0,03 ms | **139 ms** | 0,06 ms | 0,07 ms |
| 10 | 0,07 ms | **162 ms** | **1,56 s** | 4,1 ms |
| 11 | 0,08 ms | **187 ms** | 17 s (medido) | 9,5 ms |
| 15 | 0,16 ms | **223 ms** | ≈ 6 dias (previsto) | 260 ms |
| 20 | 0,25 ms | **281 ms** | ≈ 32 mil anos (previsto) | 13,0 s |
| 22 | 0,32 ms | **282 ms** | ≈ 15 milhões de anos | 63,7 s |
| 25 | 0,39 ms | **376 ms** | ≈ 2 × 10¹¹ anos | não cabe na RAM |
| 30 | 0,54 ms | **454 ms** | ≈ 3,5 × 10¹⁸ anos | ≈ 8 h e 290 GB (previsto) |

A partir de **10 pontos** o genético já é mais rápido que a força bruta, e a
partir de **15**, mais rápido que o Held-Karp. Os tempos variam ~10 % entre
rodadas (carga do PC); as distâncias não mudam.

## Ressalvas (para o texto ficar defensável)

1. **Os inviáveis são a força bruta e o Held-Karp**, não "o TSP": o TSP sempre tem ótimo, e solvers
   de *branch-and-cut* (Concorde) resolvem 30 pontos rapidamente — mas dependem
   de um solver de programação linear e não têm tempo garantido, o que não cabe
   no objetivo de rodar embarcado na Pi Zero.
2. **Os tempos são do PC.** Na Pi Zero serão maiores; o benchmark lá
   (`rotas-bench`) fecha a tabela e confere, pela assinatura SHA-256
   (`af4482d3…`), que o algoritmo é o mesmo.
3. No benchmark, a busca local repetida — **2-opt com 20 partidas aleatórias** —
   foi ainda melhor que o genético (ótimo em 189 de 200 instâncias, 14 ms com 30
   pontos); está em `docs/rotas_tsp.md`. A conclusão sobre o exato não muda.
