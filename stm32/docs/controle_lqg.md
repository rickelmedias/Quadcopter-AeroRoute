# Controle LQG do TCC no firmware

Este documento explica como o controle apresentado no TCC (Filtro de Kalman +
LQR, simulado e validado no MATLAB/Simulink e exportado como `STM32_ert_rtw`)
roda no STM32, como ele se liga ao BNO08x, ao MTF-01 e aos ESCs, e como
ajustá-lo em tempo real pela Raspberry Pi.

> **Os motores não giram com o controle por padrão.** `LQG_FLIGHT_ENABLED`
> vale 0 em `App/Inc/flight_config.h` e o parâmetro `FLT_DRY_RUN` começa em 1.
> Tudo funciona em ensaio a seco (estimativa, comandos, telemetria) até a
> seção 9 ser cumprida.

## 1. Visão geral

```text
 BNO08x ─ I2C ─┐                      ┌────────── control/ (100 Hz) ──────────┐
               ├─ leituras ─ meas.c ─ y ─► lqg_ert.c (Kalman + LQR do Simulink)│
 MTF-01 ─ UART ┘                      │        │ du                            │
                                      │  ω = ω0 + du ─ curva do motor ─ motors.c ─ DShot600 ─ ESCs
                                      └────────────────────────────────────────┘
        r = [x y z ψ] (rampa) ◄─ flight_ctrl.c ◄─ comandos ─┬─ usb_console.c (USB)
                                                           └─ DroneLink (UART) ◄─ Raspberry Pi
                                                                │  ▲
              parâmetros, ganhos novos, telemetria ◄────────────┘  └─ dronectl (tools/pi)
```

| Módulo | Papel |
|---|---|
| `control/lqg_ert.h/.c` | O `STM32_step()` do Simulink reescrito de forma legível: Kalman de regime com estimador corrente e LQR com pré-compensador. |
| `control/lqg_ert_model.h/.c` | A, B, C e os ganhos Kd, Kfd, L, M exportados do Simulink (gerado por `tools/ert_import.py`). |
| `control/meas.h/.c` | Converte BNO08x e MTF-01 no vetor de medidas `y` do modelo (unidades, eixos, geometria). Não filtra. |
| `control/flight_ctrl.h/.c` | Task de 100 Hz: estados do voo, referência em rampa, failsafes, saída para os motores e telemetria. |
| `control/ctrl_params.h/.c` | Parâmetros ajustáveis em tempo de execução (rotação de hover, curva do motor, sinais, rampas...). |
| `control/ctrl_gains.h/.c` | Ganhos em uso e troca atômica de ganhos recalculados pela Pi. |
| `control/flight_command.c` | Liga o contrato do DroneLink (`flight_command_api.h`) ao voo. |
| `motors.h/.c` | Dono do DShot600 (1 kHz) e arbitragem entre bancada e voo. |
| `usb_console.h/.c` | Comandos de texto pela USB: bancada, voo e parâmetros. |
| `power.h/.c` | Bateria (PA1) e corrente (PB0) pelo ADC1; **desligado** (`POWER_*_ENABLED 0`) até o circuito existir. |
| `dronelink/dronelink_tuning.c` | Parâmetros (PARAM_*) e envio de ganhos (DRONE_GAIN_*) pela Pi. |
| `dronelink/dronelink_telemetry.c` | ATTITUDE e LOCAL_POSITION_NED a 5 Hz e DRONE_CTRL_DEBUG na taxa pedida. |

## 2. Do Simulink para o firmware

O código gerado fica, sem alterações, em `reference/STM32_ert_rtw` (só os
fontes). Dele saem duas coisas:

1. **As matrizes.** `tools/ert_import.py` lê o `STM32_data.c` (que guarda tudo
   em ordem de coluna numa única struct) e gera `App/Src/control/lqg_ert_model.c`
   com as mesmas matrizes em ordem de linha, **preservando o texto exato de
   cada número**. Ao exportar o modelo de novo:

   ```bash
   cp ../STM32_ert_rtw/{STM32.c,STM32.h,STM32_data.c,STM32_private.h,STM32_types.h,rtwtypes.h} reference/STM32_ert_rtw/
   python tools/ert_import.py
   sh tests/run.sh
   ```

2. **O algoritmo.** `lqg_ert.c` faz as mesmas operações do `STM32_step()`, na
   mesma ordem, sem variáveis globais. `tests/test_lqg_ert.c` liga o código
   gerado e o do firmware no mesmo executável e roda os dois por milhares de
   passos, em malha aberta e em malha fechada: **com `double` a saída e o
   estado interno são idênticos bit a bit**. Com `float`
   (`-DLQG_ERT_SINGLE_PRECISION`) a diferença fica em ~2e-4 relativo.

O passo, a cada Ts = 10 ms:

```text
e        = y - C x̂[k|k-1]                 inovação (D = 0)
x̂[k|k]   = x̂[k|k-1] + M e                 estimativa corrente
du       = Kfd r - Kd x̂[k|k]              LQR com pré-compensador
x̂[k+1|k] = A x̂[k|k-1] + B du + L e        predição
```

O firmware usa `double` por padrão, como o `real_T` do Simulink. No
Cortex-M4 o `double` é emulado em software; o passo inteiro custa por volta de
0,5 ms a 96 MHz (5% da CPU). O tempo real aparece no comando `lqg` do console
(`passo max`).

## 3. O modelo

| Vetor | Conteúdo (eixos NED: x frente, y direita, z para baixo) |
|---|---|
| x (12) | x, y, z, φ, θ, ψ, vx, vy, vz, p, q, r — desvios do hover |
| y (9) | z, φ, θ, ψ, vx, vy, p, q, r — **nesta ordem** (a da matriz C) |
| r (4) | x, y, z, ψ desejados |
| du (4) | desvio da rotação de cada motor em relação ao hover, em rad/s |

**Rotação de hover.** O código gerado entrega só `du`. Na linearização, o
termo de B em vz é g / (2 ω0), independente da massa e do Kt. Com o B
exportado (6,9872e-3) isso dá **ω0 ≈ 702 rad/s** (~6700 rpm), que é o padrão
do parâmetro `CTL_W0`. *Confira esse valor com o ω0 do script do MATLAB.*

**Ordem dos motores** (numeração física, vista de cima, frente do drone para cima):

```text
   M4 (amarela)   M2 (preta)        M1 = PB6   M2 = PB7
        frente  ^                   M3 = PB8   M4 = PB9
   M3 (preta)     M1 (amarela)
```

Frente = M2 e M4, trás = M1 e M3, direita = M1 e M2, esquerda = M3 e M4.
M1 e M4 têm hélice amarela (invertida) e o ESC com sentido **invertido**; M2 e
M3 têm hélice preta e ESC **normal** (testado e gravado na bancada, ver
`docs/guia_teste_motores.md`). As colunas de B seguem outra ordem:
`control/mixer.h` leva u1→M4, u2→M2, u3→M1, u4→M3, que é o canto de cada
entrada no modelo (u1 frente/esquerda, u2 frente/direita, u3 trás/direita,
u4 trás/esquerda). O sinal de ψ de B supõe hélices pretas (M2, M3) girando no
sentido anti-horário visto de cima: confirme na bancada. Não é a numeração do
Betaflight. Use `beep 1` a `beep 4` no console.

**Como os ganhos foram projetados.** Os ganhos exportados são reproduzidos
por `tools/pi/dronectl/design.py` com erro relativo de ~1e-12:

| Ganho | Projeto |
|---|---|
| Kd | `dlqr(A, B, Q, R)`, Q = diag(5, 5, 500, 500, 500, 50, 5, 5, 5, 10, 10, 5), R = 0,1 I |
| Kfd | `inv(Cr (I - A + B Kd)^-1 B)`, Cr escolhe x, y, z, ψ (ganho DC unitário) |
| M, L | Kalman de regime, G = I, Qn = diag(1e-5 ×6, 1e-3 ×6), Rn = variâncias dos sensores (1,15e-4; 5e-8; 1,94e-7; 6,04e-8; 1,28e-2; 2,2e-3; 2,06e-6; 3,24e-7; 2,38e-7), M = P C'(C P C' + Rn)^-1, L = A M |

É isso que permite ajustar em tempo real: a Pi muda os pesos, recalcula os
mesmos ganhos que o MATLAB calcularia e os envia (seção 7).

## 4. Medidas (`meas.c`)

O modelo supõe sensores que medem os estados. A conversão não filtra nada:

| Medida | Origem |
|---|---|
| z | −(distância do MTF-01 × cos da inclinação) |
| φ, θ | Euler do BNO08x × `SNS_SGN_ROLL`, `SNS_SGN_PITCH` |
| ψ | guinada do BNO08x relativa ao instante em que armou, sem o salto de ±π, × `SNS_SGN_YAW` |
| vx, vy | (`FLOW_SCALE` × fluxo + `FLOW_GYRO` × giro) × distância, girada pela guinada, × `SNS_SGN_VX/VY` |
| p, q, r | giroscópio do BNO08x × `SNS_SGN_P/Q/R` |

Sem leitura válida, a medida repete o último valor (como uma entrada amostrada
no Simulink) e o bit correspondente fica zerado (`meas` na linha EST e
`meas_flags` no DRONE_CTRL_DEBUG). A escala e a compensação do fluxo vêm da
calibração de bancada já feita (`docs/acompanhamento_tecnico.md`).

## 5. A malha de voo (`flight_ctrl.c`)

A task roda a cada 10 ms (`vTaskDelayUntil`), com prioridade abaixo só da task
dos motores:

1. atende pedidos (do console USB ou da Pi);
2. lê os sensores e monta `y`;
3. confere os failsafes;
4. avança a referência `r` em rampa até o alvo;
5. `lqg_ert_output` → `du`; ω = `CTL_W0` + du → aceleração pela curva
   `MOT_W_AT_0`/`MOT_W_AT_100`, limitada a [`MOT_MIN`, `MOT_MAX`];
6. `lqg_ert_update` com a entrada que **de fato** foi aplicada: em voo é o
   próprio `du` (como no Simulink; só difere se o motor saturar); desarmado,
   no chão ou em ensaio a seco é zero, porque os motores não seguem o
   controle;
7. publica a telemetria.

O Kalman roda desde o boot, mesmo desarmado: dá para ver a filtragem sem voar.

**Estados** (os mesmos de `DRONE_FLIGHT_STATE`):

| Estado | O que acontece |
|---|---|
| STANDBY | Desarmado. Kalman rodando; `r` acompanha a estimativa. |
| ARMED | Kalman e guinada zerados (referencial local começa aqui); motores em `MOT_IDLE`. Desarma sozinho após 15 s sem decolar. |
| TAKEOFF | 1 s de aceleração inicial, depois `r_z` sobe em rampa (`FLT_Z_RATE`) até a altura pedida. |
| HOLD | Mantém x, y, z e ψ. |
| EXECUTING_LEG | Vai até o alvo de um trecho (girar, transladar, ir a um ponto). |
| LAND | `r_z` desce até o chão; perto do chão (`FLT_LAND_H`) por 0,5 s, desarma. |
| FAILSAFE | Motores cortados. Sai com um novo armar ou desarmar. |

**Failsafes:**

| Condição | Ação |
|---|---|
| BNO08x sem dado por 50 ms | FAILSAFE |
| Inclinação medida acima de `FLT_TILT_MAX` com o controle ligado | FAILSAFE |
| Armado pela USB e a porta fechou | FAILSAFE |
| Motores parados por fora (Enter no console, por exemplo) | FAILSAFE |
| MTF-01 sem altura por 300 ms no ar | Pouso (sem altura, desarma pelo tempo) |
| Enlace ou autorização da Pi perdidos | HOLD e depois pouso (DroneLink) |

## 6. Parâmetros

Todos ficam na RAM e voltam ao padrão (`flight_config.h`) a cada boot. A Pi
reaplica perfis salvos.

| Nome | Padrão | Significado |
|---|---|---|
| `CTL_W0` | 702 | Rotação de hover ω0, rad/s |
| `MOT_W_AT_0` / `MOT_W_AT_100` | 0 / 1755 | Curva rotação × aceleração DShot (provisória: ω0 em ~40%) |
| `MOT_MIN` / `MOT_MAX` | 0,03 / 1,0 | Limites da aceleração com o controle ligado |
| `MOT_IDLE` | 0,05 | Aceleração armado no chão |
| `SNS_SGN_*` | ver `flight_config.h` | Sinais dos sensores para NED (só desarmado) |
| `SNS_ZERO_ROLL` / `SNS_ZERO_PITCH` | −174,5 / −2,9 | Roll e pitch crus do BNO com o drone nivelado, em graus (só desarmado). O BNO está montado de cabeça para baixo; roll e pitch entram no modelo relativos a este zero |
| `FLOW_SCALE_X/Y` | 0,944 / 0,942 | Escala do fluxo óptico |
| `FLOW_GYRO_X/Y` | +1 / −1 | Compensação da rotação no fluxo (0 desliga) |
| `FLT_HOVER_H` | 0,5 | Altura padrão do hover, m |
| `FLT_Z_RATE` / `FLT_XY_RATE` / `FLT_YAW_RATE` | 0,3 / 0,3 / 0,5 | Rampas da referência (0 = degrau, como na simulação) |
| `FLT_TILT_MAX` | 0,785 | Inclinação de corte, rad |
| `FLT_LAND_H` | 0,08 | Fim do pouso, m acima do chão |
| `FLT_DRY_RUN` | 1 | 1 = motores parados (só desarmado) |
| `DBG_RATE_HZ` | 0 | Taxa do DRONE_CTRL_DEBUG para a Pi |
| `LOG_EST_HZ` | 10 | Taxa da linha EST na USB |

## 7. Ajuste em tempo real pela Raspberry Pi

A ferramenta `dronectl` (`tools/pi`, ver o README de lá) roda na Pi por SSH.
Ela fala o protocolo DroneLink, que ganhou:

| Mensagem | Sentido | Uso |
|---|---|---|
| PARAM_REQUEST_LIST / _READ / PARAM_SET → PARAM_VALUE | Pi ↔ STM32 | Parâmetros da seção 6, protocolo padrão do MAVLink |
| DRONE_GAIN_ROW (51010) | Pi → STM32 | Uma linha de Kd, Kfd, L ou M |
| DRONE_GAIN_COMMIT (51011) | Pi → STM32 | Confirma (com CRC-32), aborta, restaura os do Simulink ou consulta |
| DRONE_GAIN_ACK (51012) | STM32 → Pi | Resultado, CRC calculado e CRC dos ganhos em uso |
| DRONE_CTRL_DEBUG (51020) | STM32 → Pi | Um passo do LQG: y, x̂, du, r, aceleração e flags |

**Troca de ganhos.** As 32 linhas vão para uma área de preparo; os ganhos em
uso só mudam no commit, se todas as linhas chegaram, todos os valores são
finitos e o CRC confere. O voo copia os ganhos no começo de um passo, então um
passo nunca mistura ganhos velhos e novos.

**Roteiro de ajuste sugerido:**

1. `dronectl monitor` com o drone parado: a tabela mostra, para cada medida,
   o desvio padrão de y, de x̂ e de y − x̂. A coluna "Rn sugerido" é a variância
   do ruído de cada sensor medida agora.
2. `dronectl tune`: ajuste o Kalman no chão (`rn auto` usa as variâncias
   medidas, `qn vx 2e-3` deixa a velocidade mais ágil...), `apply`, e veja a
   filtragem mudar com `watch`.
3. `dronectl hover --dry --debug`: com o drone na mão, confira sinais e
   respostas sem motores.
4. `dronectl hover --debug` (depois da seção 9): ajuste o LQR em voo
   (`q z 800`, `r all 0.2`, `apply`), a rotação de hover (`set CTL_W0 690`) e
   salve o perfil (`save perfil.json`).

Toda sessão `--debug` grava a telemetria em `logs/hover_<data>.csv`.

## 8. Telemetria

- **USB, linha EST** (taxa `LOG_EST_HZ`):
  `EST;t_ms;estado;meas;flags;y[9];x_hat[12];du[4];ref[4]`, com y, x̂ e r em
  milésimos de SI e du em décimos de rad/s.
- **Pi, DRONE_CTRL_DEBUG** (taxa `DBG_RATE_HZ`, até 50 Hz): os mesmos dados
  em float. A 115200 bit/s cada mensagem ocupa ~13 ms do enlace; 25 Hz é o
  padrão da ferramenta.
- **Pi, ATTITUDE e LOCAL_POSITION_NED** a 5 Hz, sempre.

## 9. Roteiro de bancada (sem hélices)

1. Grave o firmware e abra o console (`tools\motor_console.ps1`) ou use
   `dronectl monitor` na Pi.
2. Confira os motores: `beep 1`..`beep 4` e o sentido de giro (guia em
   `docs/guia_teste_motores.md`).
3. `dry 0.5` no console (ou `dronectl hover --dry --debug`). Com o drone na
   mão:
   (colunas u1..u4 do modelo; o `mixer` leva u1→M4, u2→M2, u3→M1, u4→M3)
   - abaixe o lado direito: φ positivo, e u2/u3 (M2 e M1, direita) devem subir;
   - levante o nariz: θ positivo, u3/u4 (M1 e M3, trás) sobem;
   - gire no sentido horário visto de cima: ψ aumenta, u2/u4 (M2 e M3, hélices
     pretas) sobem;
   - suba o drone: z fica mais negativo, as quatro rotações caem.
   Sinal trocado: corrija o `SNS_SGN_*` correspondente (desarmado). Lembre
   que o BNO está girado: φ vem do pitch dele e θ do roll dele
   (`control/meas.h`), e a guinada é zerada na energização.
4. Meça a curva do motor e ajuste `MOT_W_AT_0`/`MOT_W_AT_100` (ou confirme
   `CTL_W0` com o MATLAB).
5. Só então, com o drone preso: `LQG_FLIGHT_ENABLED 1` em `flight_config.h`,
   grave, `FLT_DRY_RUN 0`, e teste `hover` sem hélices.

## 10. O que foi removido

O estimador de deslocamento anterior (EKF de fluxo, filtro de alcance,
calibrador de fluxo, histórico do BNO), o LQG genérico variante no tempo a
400 Hz e a captura binária com replay saíram: o controle apresentado é o do
Simulink. Tudo continua no histórico do Git (commit `16269e9`).

## 11. Pontos em aberto

- Confirmar ω0 com o script do MATLAB e medir a curva do motor.
- Conferir os sinais dos sensores na bancada (seção 9).
- O LQR do TCC não tem ação integral: um ω0 ou uma curva de motor errados
  aparecem como erro permanente de altura. O ajuste de `CTL_W0` em voo corrige.
- x e y não são observáveis (só a velocidade é medida): a posição deriva
  devagar, como no modelo.
- Os parâmetros não são gravados na flash; a Pi reaplica perfis.
- Bateria e corrente: `power.h` já lê PA1 e PB0, mas está desligado. Ao
  montar o divisor e ligar o CURR do ESC, ajuste `POWER_VBAT_DIVIDER` e
  `POWER_CURRENT_A_PER_V` e mude as chaves para 1. Ainda não há failsafe de
  bateria baixa.
