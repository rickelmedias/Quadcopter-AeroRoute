# LQI: o LQR do TCC com integradores (branch `controle-lqi`)

> **Removido em 06/11/2026.** O LQI saiu do firmware, do painel e das
> ferramentas (`CTL_LQI`, `lqi_*.c`, `tools/lqi`, `tools/lqi_projeto.py`). Este
> texto fica como registro dos ensaios; o código está no histórico do git (tag
> `lqg-improved-v1`).

## Por que

O controle exportado pelo Simulink (`STM32_ert_rtw`) é um LQG: Kalman de regime
mais `du = Kfd·r − Kd·x̂`. Ele é **só proporcional ao estado**, e o modelo supõe
o centro de gravidade no centro e os quatro motores iguais (matriz B simétrica).
No drone real há um desequilíbrio grande: nos ensaios de 04/11, sem apoio da
mão, a traseira pedia ~60% de aceleração contra ~45% da frente. Esse torque
constante deixava θ parado em 10–16°, e a altura ficava abaixo do alvo porque a
rotação de hover do modelo (702 rad/s) não é a real.

A branch `controle` resolvia isso com um "ajuste lento" feito por fora do LQR,
com ganhos à mão (`control/trim.h`). Ele funcionava parado, mas entrava num
ciclo lento de θ. O LQI põe a ação integral **dentro** do projeto LQR, com os
ganhos calculados pelo mesmo `dlqr`.

## Projeto

Mesmo modelo A, B do Simulink, mesmos pesos Q e R do TCC; só os pesos dos
integradores são novos.

```text
subsistema       z φ θ ψ vz p q r          (A_r, B_r = linhas/colunas de A, B)
aumentado        Aa = [A_r 0; Ts·Ci I]     Ba = [B_r; 0]
                 ξ[k+1] = ξ[k] + Ts·(z, φ, θ)[k]
pesos            Q_r = Q do TCC, R = 0,1·I, ∫z = 1000, ∫φ = ∫θ = 10000
ganhos           [Kx Ki] = dlqr(Aa, Ba, diag(Q_r, Q_i), R)
lei de controle  du = −Kx (x̂ − x_ref) − Ki ξ
```

**Por que o subsistema e não o modelo completo:** no A do TCC, x, y, vx e vy não
realimentam a atitude, e vx' = −g·θ, ou seja, a integral de θ já é (a menos de
constante) a própria vx. Um integrador de θ no modelo completo é redundante: o
`dlqr` dá ganho ~0 ou não converge. No projeto completo é o laço de posição
(fluxo óptico) quem segura um desequilíbrio, mas com peso 5 ele precisaria de
um deslocamento enorme. Por isso o LQI fica no modo só atitude e altura (x e y
fora da malha, como já era no teste).

| Ganho (por motor) | TCC (Simulink) | LQI |
|---|---|---|
| φ, θ | 50,6 | 71,3 |
| p, q | 11,4 | 13,1 |
| z | 35,1 | 97,5 |
| vz | 50,2 | 83,4 |
| ∫z | — | 49,4 |
| ∫φ, ∫θ | — | 148,4 |

O projeto conjunto também endurece um pouco o proporcional (a altura em
especial, que no TCC era macia: 10 rad/s de erro de hover davam ~30 cm de erro).

**Escolha dos pesos.** Pesos maiores em ∫φ/∫θ (50 000, 200 000) rejeitam
empurrões melhor, mas ficam instáveis com o atraso do motor/ESC, que o modelo
não tem:

| ∫φ, ∫θ | Kθ | Ki θ | atraso 40 ms | 80 ms | 120 ms |
|---|---|---|---|---|---|
| **10 000** | 71 | 148 | estável | estável | estável (oscila só com autoridade 0,6×) |
| 50 000 | 108 | 328 | estável | no limite | instável |
| 200 000 | 162 | 645 | estável | instável (0,6×) | instável |

## Implementação no firmware

- `control/lqi.h`: integradores, saturação e saída `−Ki ξ`; ganhos em
  `App/Src/control/lqi_gains.c`, **gerado** por `tools/lqi/lqi_gen.c` a partir de
  `tools/lqi/lqi_design.c` (o `dlqr` em C de `tools/lqi/lqr_design.c` reproduz o
  Kd do Simulink com erro relativo de 1,4·10⁻¹²). `tests/run.sh` confere que a
  tabela é a que o projeto gera.
- `CTL_LQI = 1` (padrão nesta branch) troca Kd e Kfd pelos do LQI e soma a saída
  dos integradores; o ajuste lento (`CTL_TRIM`) não é usado.
- **O Kalman continua o do Simulink** e recebe só a parte do LQR do `du`. A parte
  dos integradores compensa uma perturbação que o modelo não tem; passá-la ao
  filtro enviesava a estimativa (visto com o ajuste lento: vz estimado de
  −0,29 m/s com o drone parado).
- **Os integradores usam as medidas** (z do MTF, φ e θ do BNO), não x̂: o erro em
  regime vai a zero no que o sensor mede, mesmo com o modelo errado.
- **Anti-windup** (não muda o projeto linear perto do equilíbrio):
  - a correção de cada integrador em um motor fica ≤ `CTL_LQI_MAX` (250 rad/s);
  - congela com motor saturado, em ensaio a seco, na rampa de partida e na
    descida do pouso; a altura só integra com o MTF respondendo;
  - no ar, o erro de φ e θ integrado é limitado a ±0,08 rad
    (`LQI_ATT_ERROR_MAX_RAD`); no chão entra inteiro, para achar o desequilíbrio.
- Os integradores de φ e θ do último instante no ar, nivelado e assentado são
  lembrados e o próximo voo começa deles (até reiniciar a placa).
- O log mostra a cada 2 s `lqi z=.. th=.. phi=..` (correção em rad/s: hover,
  frente/trás, esquerda/direita) e `NO LIMITE` quando um integrador satura.

## Simulação (`tests/test_lqi.c`)

Planta do Simulink, Kalman do Simulink, chão, ruído nos sensores, motores com
atraso de primeira ordem, desequilíbrio medido (125 rad/s na arfagem) e um
empurrão de 30 rad/s por 1,5 s:

| Caso | Altura final | θ, φ finais | Rebote do empurrão |
|---|---|---|---|
| hover +30, sem atraso | 0,149 m | −0,01°, +0,01° | 14,4° |
| hover +30, atraso 40 ms | 0,149 m | −0,02°, 0,00° | 16,0° |
| hover +30, atraso 80 ms | 0,149 m | −0,04°, −0,02° | 22,3° |
| hover +170, atraso 40 ms | 0,149 m | −0,02°, 0,00° | 16,0° |

## No MATLAB

`tools/lqi/projeto_lqi.m` faz o mesmo cálculo a partir de `Ad`, `Bd` do TCC e
imprime Kx, Ki e Kfd para comparar com `lqi_gains.c`. Use `dlqr` no sistema
aumentado, não `lqi`: o `lqi` do MATLAB integra `r − y`, o que inverte o sinal
de Ki. Para mudar os pesos, altere `lqi_default_weights()` em
`tools/lqi/lqi_design.c` (e o `.m`), regenere com `tools/lqi/lqi_gen` e rode
`tests/run.sh`.

## Ajuste depois dos ensaios de 05/11

Três mudanças, todas a partir dos logs de 05/11 (`tools/logs/controle_20261105_*.log`):

1. **Atitude mais firme.** Os pesos padrão do LQI passaram a ser os do TCC com
   φ, θ = 8000 (eram 500) e p, q = 100 (eram 10): K_φ/θ vai de 71 para 150
   rad/s por rad e K_p/q de 13 para 23. Na simulação de `tests/test_lqi.c`
   (desequilíbrio de 125 rad/s, empurrão de 1,5 s), o rebote cai assim:

   | atraso do motor | TCC (500/10) | novo (8000/100) | 16000/100 |
   |---|---|---|---|
   | 40 ms | 16,0° | 6,7° | 5,9° |
   | 80 ms | 22,3° | 11,1° | 9,9° |
   | 120 ms | 28,5° | 10,7° | instável |

   O valor de 8000 deixa margem para um motor mais lento que o esperado.
   `lqi_tcc_weights()` continua com os pesos do TCC (os que reproduzem o
   Simulink); `lqi_default_weights()`, `projeto_lqi.m` e `tools/lqi_projeto.py`
   têm os novos.
2. **Desequilíbrio fixo (`CTL_FF_TH`, padrão −90 rad/s).** Em todo hover o
   integrador de θ parava em −82 a −105 rad/s (traseira ~10% de aceleração
   acima da frente). Sem esse valor desde o início, a decolagem saía com os
   quatro motores iguais e a frente levantava sozinha no fim da rampa: θ de +2°
   a +18° em 0,2 s, FAILSAFE em 7 de 13 voos. Agora ele entra nos motores desde
   a rampa de partida, fora do Kalman, e os integradores corrigem só o resto.
   `CTL_FF_PHI` faz o mesmo entre os lados (padrão 0).
3. **Sem equilíbrio lembrado no LQI.** Ele trouxe, duas vezes, valores de voos
   presos nas cordinhas. Cada voo começa com os integradores em zero, por cima
   do desequilíbrio fixo.

Para ajustar `CTL_FF_TH`: num hover estável, o log mostra `lqi z=.. th=..`. Se
`th` ficar longe de 0, some esse valor ao `CTL_FF_TH` (o painel tem o controle).

## Testar outros pesos sem regravar

O painel (`tools/controle_gui.py`) tinha o quadro "Pesos do LQI" (hoje o quadro é o de pesos Q do LQG, seção 13 de controle_lqg.md; o envio abaixo segue valendo pelo console), com os pesos de
Q (z, φ, θ, ψ, vz, p, q, r), R e os três integradores. "Calcular" mostra os
ganhos que resultam; "Enviar ao drone" manda Kx, Kfd e Ki ao STM32; "Padrão"
volta à tabela de `lqi_gains.c`.

- O cálculo é `tools/lqi_projeto.py`, o mesmo `dlqr` (algoritmo de duplicação)
  de `tools/lqi/lqr_design.c` em Python puro (sem numpy). Com os pesos padrão
  ele reproduz `lqi_gains.c` com erro relativo de 1e-15 (`tests/run.sh` confere).
- O envio usa o console USB: `lqi begin`, 76 linhas `lqi set kx|kfd|ki L C V` e
  `lqi commit <crc>`. O firmware (`control/lqi_tuning.h`) só troca os ganhos se
  os 76 valores chegaram, são finitos e o CRC-32 (o mesmo do envio pela Pi)
  confere. Responde `lqi ganhos do painel crc=..` e o voo escreve
  `lqi: ganhos do painel em uso`.
- Só com o drone desarmado: os integradores ξ guardam o equilíbrio na escala do
  Ki antigo, então trocar Ki em voo daria um degrau. Pelo mesmo motivo, o
  equilíbrio lembrado do voo anterior é descartado na troca.
- Os ganhos enviados valem até reiniciar a placa. O log do ensaio registra os
  pesos e os ganhos de cada envio (`painel: enviando ganhos do LQI ..`).

Mudar os pesos continua sendo o projeto do TCC (o mesmo modelo A, B e o mesmo
`dlqr`); só a escolha de Q e R muda.

## O que o LQI não resolve

- **Decolagem torta:** no chão a traseira está apoiada e nenhum controle vê o
  desequilíbrio antes de a frente levantar; o LQI corrige em seguida.
- **Margem do lado pesado:** a traseira trabalha ~15% mais carregada.
- **Parâmetros não medidos:** ω0 real, curva DShot → rpm, diferença entre
  motores, massa e inércias. O B do modelo pode não ser o do drone.
- **Cordinhas:** acima do teto delas o hover empurra contra elas.

Balancear o centro de gravidade e medir o rpm de cada motor continuam valendo a
pena.

## Como testar

`python3 tools/controle_gui.py`: o modo de compensação vem em **LQI**. Reinicie
a placa antes (zera o equilíbrio lembrado), `CTL_W0` ≈ 850, altura abaixo do
teto das cordinhas, ensaio a seco primeiro.
