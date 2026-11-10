Máquina: Intel(R) Core(TM) i5-9300HF CPU @ 2.40GHz · Python 3.14.7 · RAM total 7.3 GB, livre 3.7 GB · limite de tempo 70 s por instância

## Força bruta (N! ordens)

| N | ordens testadas | tempo | pico de RAM do processo |
|---|---|---|---|
| 3 | 6 | 0.0 ms | 22.4 MB |
| 4 | 24 | 0.0 ms | 22.4 MB |
| 5 | 120 | 0.1 ms | 22.4 MB |
| 6 | 720 | 0.4 ms | 22.4 MB |
| 7 | 5.040 | 1.6 ms | 22.4 MB |
| 8 | 40.320 | 13.7 ms | 22.4 MB |
| 9 | 362.880 | 149.0 ms | 22.4 MB |
| 10 | 3.628.800 | 1.5 s | 22.4 MB |
| 11 | 39.916.800 | 17.6 s | 22.4 MB |

Sem rodar (previsão t = c·N! ajustada nos 3 maiores medidos):

| N | ordens | tempo previsto |
|---|---|---|
| 12 | 4.79e+08 | 3 min |
| 13 | 6.23e+09 | 43 min |
| 14 | 8.72e+10 | 10.1 h |
| 15 | 1.31e+12 | 6 dias |
| 16 | 2.09e+13 | 101 dias |
| 17 | 3.56e+14 | 4.7 anos |
| 18 | 6.40e+15 | 84.6 anos |
| 19 | 1.22e+17 | 1.61e+03 anos |
| 20 | 2.43e+18 | 3.22e+04 anos |
| 21 | 5.11e+19 | 6.75e+05 anos |
| 22 | 1.12e+21 | 1.49e+07 anos |
| 23 | 2.59e+22 | 3.42e+08 anos |
| 24 | 6.20e+23 | 8.2e+09 anos |
| 25 | 1.55e+25 | 2.05e+11 anos |
| 26 | 4.03e+26 | 5.33e+12 anos |
| 27 | 1.09e+28 | 1.44e+14 anos |
| 28 | 3.05e+29 | 4.03e+15 anos |
| 29 | 8.84e+30 | 1.17e+17 anos |
| 30 | 2.65e+32 | 3.51e+18 anos |

- Primeiro N acima de 70 s por instância: **12**

## Held-Karp (programação dinâmica, N²·2ᴺ)

Parou em N = 24: N=23 já levou 125 s (> 70 s).

| N | tempo medido | pico de RAM do processo | memória do Held-Karp (N·2ᴺ·9 B) |
|---|---|---|---|
| 3 | 0.0 ms | 22.4 MB | 0.0 MB |
| 4 | 0.1 ms | 22.4 MB | 0.0 MB |
| 5 | 0.1 ms | 22.4 MB | 0.0 MB |
| 6 | 0.2 ms | 22.4 MB | 0.0 MB |
| 7 | 0.3 ms | 22.4 MB | 0.0 MB |
| 8 | 0.7 ms | 22.4 MB | 0.0 MB |
| 9 | 1.7 ms | 22.4 MB | 0.0 MB |
| 10 | 3.8 ms | 22.4 MB | 0.1 MB |
| 11 | 8.9 ms | 22.4 MB | 0.2 MB |
| 12 | 20.9 ms | 22.4 MB | 0.4 MB |
| 13 | 46.4 ms | 22.4 MB | 1.0 MB |
| 14 | 104.9 ms | 22.4 MB | 2.1 MB |
| 15 | 236.4 ms | 22.4 MB | 4.4 MB |
| 16 | 517.0 ms | 22.7 MB | 9.4 MB |
| 17 | 1.2 s | 32.8 MB | 20.1 MB |
| 18 | 2.6 s | 54.2 MB | 42.5 MB |
| 19 | 5.6 s | 99.1 MB | 89.7 MB |
| 20 | 12.5 s | 193.7 MB | 188.7 MB |
| 21 | 26.1 s | 391.4 MB | 396.4 MB |
| 22 | 63.7 s | 805.9 MB | 830.5 MB |
| 23 | 2 min | 1.7 GB | 1.7 GB |

Sem rodar (previsão t = c·N²·2ᴺ ajustada nos 5 maiores medidos):

| N | tempo previsto | memória necessária | cabe na RAM livre? | cabe na RAM total? |
|---|---|---|---|---|
| 24 | 5 min | 3.6 GB | sim | sim |
| 25 | 10 min | 7.5 GB | não | não |
| 26 | 22 min | 15.7 GB | não | não |
| 27 | 48 min | 32.6 GB | não | não |
| 28 | 103 min | 67.6 GB | não | não |
| 29 | 3.7 h | 140.1 GB | não | não |
| 30 | 7.9 h | 289.9 GB | não | não |

- Primeiro N acima de 70 s por instância: **23**
- Primeiro N que não cabe na RAM livre (3.7 GB): **25**
- Primeiro N que não cabe nem na RAM total (7.3 GB): **25**
- Raspberry Pi Zero (512 MB): passa da metade da RAM a partir de **21** pontos
