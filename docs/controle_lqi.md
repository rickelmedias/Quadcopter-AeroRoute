# LQI: o LQR do TCC com integradores (branch `controle-lqi`)

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
