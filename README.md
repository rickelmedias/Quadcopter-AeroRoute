# DroneController

Firmware de uma controladora de drone baseada na **STM32F411CEU6 Black Pill**.
Ele roda o controle apresentado no TCC — **Filtro de Kalman + LQR, simulado e
validado no MATLAB/Simulink** e exportado como `STM32_ert_rtw` — sobre o
BNO08x (atitude e giroscópio), o MTF-01 (altura e fluxo óptico) e quatro ESCs
em DShot600. Uma Raspberry Pi comanda o drone por MAVLink (DroneLink) e
permite ajustar o controle e ver a filtragem dos sensores em tempo real.

O projeto ainda está em desenvolvimento. O controle roda inteiro em ensaio a
seco (estimativa, comandos e telemetria), mas os motores só seguem o LQG
depois de conferir sinais e curva do motor na bancada
([`docs/controle_lqg.md`](docs/controle_lqg.md), seção 9). Portanto, o
firmware atual serve para desenvolvimento e bancada, não para voo.

Perfis de bancada em branches separadas, cada um testável sozinho:

| Branch | O que roda |
|---|---|
| `mtf` | Só o MTF-01, uma linha por frame (100 Hz) |
| `bno` | Só o BNO08x a 400 Hz (I2C a 400 kHz) |
| `motors` | MTF-01 + BNO08x + teste dos motores por comandos USB, sem controle ([guia](docs/guia_teste_motores.md)) |
| `main` | Tudo integrado com o Kalman + LQR do Simulink |

## Estado da branch `comunicacao-pi-stm32` (07/11/2026)

Teste **isolado** da comunicação Raspberry Pi ↔ STM32 (MAVLink/DroneLink), a
partir do `main`. Para não depender do voo, o STM32 tem um **drone simulado**
(`FLT_SIM = 1`): motores parados e o mesmo LQG do TCC controlando o modelo do
TCC no lugar dos sensores. Detalhes: [`docs/teste_comunicacao.md`](docs/teste_comunicacao.md)
(protocolo, comandos, diagrama das mensagens) e [`docs/rotas_tsp.md`](docs/rotas_tsp.md)
(ordem de visita dos pontos, benchmark para o TCC).

### O que foi feito

| Onde | O quê |
|---|---|
| STM32 | `FLT_SIM` e `control/sim_plant.c` (drone simulado, `tests/test_sim_plant.c`) |
| STM32 | Trechos só terminam com o drone **estável** (no alvo e parado por 1 s); só então o próximo comando é aceito |
| STM32 | Trecho fora dos limites de altura é recusado **na primeira resposta** (antes: "em andamento" e depois "falhou") |
| STM32 | Correção (vinha do `main`): a supervisão de enlace da Pi agia também em voo armado pelo USB e pousava o **primeiro voo depois do boot** |
| Pi | `dronectl missao`: shell e arquivo de missão; medidas em **cm** inteiros; pontos **3D** (x, y, z); comandos com a nomenclatura aeronáutica: `takeoff`, `yaw`, `forward/back/right/left/climb/descend`, `goto x y [z]`, `waypoints`, `route` (ordem pelo AG; sem pontos lê `missoes/pontos.txt`), `rtl`, `hold`, `land` |
| Pi | `dronectl/rotas.py`: aleatório, guloso, guloso+2-opt, 2-opt multipartida, genético, força bruta e exato (Held-Karp), em 3D; `dronectl rotas-bench` e `rotas-limite`; figuras em `tools/pi/rotas_graficos.py` e `rotas_apresentacao.py` |
| Pi | Correções no enlace: estado de voo também pelo `DRONE_LEG_ACK`; rota calculada só com posição recebida depois do fim do trecho |

### Testes

- **Firmware real no STM32, drone simulado, pelo console USB:** decolar 50 cm
  estável em 6,3 s; girar 90° em 4,8 s; andar 1 m em 7,6 s (fim a 0,3 cm do
  ponto); recusas certas (andar antes de decolar = estado errado; comando
  durante outro = ocupado; fora dos limites = recusado já no pedido); primeiro
  voo depois do boot fica em HOLD sem Pi; pouso e desarme.
- **Rotas no STM32 (N = 6, 2D e 3D):** o STM32 executou as rotas do guloso e do
  exato como planejadas (erro máximo de 3,1 cm em cada ponto); em 3D a rota do
  exato foi 21 % mais curta (1493 cm × 1898 cm).
- **Benchmark 3D (PC, 21 min, N = 3 a 30, 10 instâncias, 1680 execuções):** a
  **força bruta** (N! ordens) entra até **10 pontos** (1,5 s; com 11 já leva
  17 s e com 20 levaria ~32 mil anos); o
  TSP exato (Held-Karp) entra até **22 pontos** (~1 min por instância); com 23
  passa de 2 min e 1,7 GB e sai do comparativo; com 25 não cabe nem na RAM do PC
  (7,5 GB). Até 22 pontos o genético achou o ótimo global em 150/200 instâncias
  (0,13–0,45 s por instância em toda a faixa) e o 2-opt multipartida em 189/200. Avaliação:
  [`docs/rotas_tsp.md`](docs/rotas_tsp.md); figuras para a apresentação (rotas
  em 3D, tempo e memória do AG × TSP exato):
  [`docs/apresentacao_rotas/`](docs/apresentacao_rotas/). Assinatura
  `af4482d38e122be77517d9b9b097ebbdda6414b8f72f5601382e9dcb978532f0`.
- **Pi ↔ UART ↔ STM32 real (drone simulado), `tools/pi/ensaios/ensaio_dronelink.py`:**
  60 s de enlace sem perdas nem erros de CRC (552 B/s); latência de ida e volta
  de 10 ms (mediana); as 10 recusas com o motivo certo; pedido repetido
  executado uma vez; cancelamento; `takeoff 30` / `route` / `land` com erro
  máximo de 1,4 cm por ponto; autorização cortada → HOLD em 0,9 s e pouso em
  6 s; processo da Pi morto → HOLD em 1,3 s e pouso em 5,3 s. Detalhes e as três
  correções que saíram deste ensaio (sequência MAVLink única, altura a partir
  do chão, resposta de trecho conferida pela missão) em
  [`docs/teste_comunicacao.md`](docs/teste_comunicacao.md).
- **Pi (contra um STM32 falso, `tools/pi/tests`):** 26 testes — comandos,
  cálculo de rota, algoritmos, missões 2D e 3D, recusas, resposta antiga de
  outra missão e pouso em erro. Também rodam na própria Pi.
- **Mesma rota no PC e na Pi** (`tools/pi/ensaios/assinatura_ag.py`): o AG dá a
  mesma ordem e o mesmo SHA-256 nas duas máquinas para o quadrado do ensaio e
  para instâncias de 10 e 30 pontos do benchmark; na Pi Zero leva de 10 a 24 s
  (60 a 70× o PC). O benchmark completo fica no PC, por poder computacional.
  Registro: [`docs/resultados_rotas/pi/assinatura_ag.md`](docs/resultados_rotas/pi/assinatura_ag.md).
- `sh tests/run.sh`: todas as suítes do firmware.

### Pendente

1. Ensaio pela UART com os motores girando e em voo.
2. Propostas ainda não feitas: vigia do operador na Pi (perdeu o Wi-Fi → HOLD →
   pouso) e watchdog de hardware (IWDG) no STM32.

## O que existe hoje

```text
 BNO08x ── I2C/SH-2 ──┐
                      ├── control/meas ── y ──► Kalman + LQR (Simulink, 100 Hz) ── du
 MTF-01 ─ UART/DMA ───┘                               ▲                            │
                                                      │ r = [x y z ψ]              ▼
 Raspberry Pi ── UART/MAVLink ── DroneLink ──► voo (flight_ctrl) ──► motores ── DShot600 ── ESCs
      ▲  (dronectl: hover, ajuste,                    │
      │   telemetria, parâmetros)                     ▼
      └──────────────────────────────────── telemetria ──► USB CDC ── PC/logs e console
```

- O **BNO08x** fornece quatérnio, velocidade angular e aceleração linear a 400 Hz.
- O **MTF-01** fornece fluxo óptico e distância até o piso a 100 Hz.
- `control/meas` converte as leituras no vetor de medidas do modelo, sem filtrar.
- O **Kalman + LQR** é o código exportado do Simulink, portado com resultado
  idêntico bit a bit (`tests/test_lqg_ert.c`).
- A **máquina de voo** arma, decola, segura, executa trechos e pousa, com
  failsafes.
- O **DroneLink** troca mensagens MAVLink 2 com a Raspberry Pi: comandos de
  voo, parâmetros, troca atômica de ganhos e telemetria do controle.
- A **ferramenta `dronectl`** (`tools/pi`) roda na Pi por SSH: `hover`,
  `hover --debug` com ajuste em tempo real, `monitor` da filtragem e perfis.
- A USB da Black Pill aparece como porta COM para logs e um console de
  bancada.

## Peças e interfaces

| Peça | Papel | Interface com o STM32 |
|---|---|---|
| STM32F411CEU6 Black Pill | Controladora principal | — |
| GY-BNO08X | IMU: orientação, giroscópio e aceleração | I2C2, INT e RST |
| MicoAir MTF-01 | Fluxo óptico e telêmetro | USART2, protocolo Micolink |
| Raspberry Pi Zero W | Comandos, ajuste e telemetria | USART1, MAVLink 2/DroneLink |
| W25Q64 | Identidade persistente de boot do DroneLink | SPI1 |
| Skystars KM55A II (AM32) e 4 motores | Atuação | TIM4, DShot600 |
| Divisor de tensão da bateria | Tensão da bateria (**ainda não ligado**) | ADC1_IN1 |
| Saída CURR do ESC | Corrente (**ainda não ligada**) | ADC1_IN8 |
| Computador de desenvolvimento | Gravação DFU, logs e console | USB da Black Pill |

Ligações:

| Pino STM32 | Função | Liga em |
|---|---|---|
| PB6 | TIM4_CH1 (DShot) | ESC do motor 1 |
| PB7 | TIM4_CH2 (DShot) | ESC do motor 2 |
| PB8 | TIM4_CH3 (DShot) | ESC do motor 3 |
| PB9 | TIM4_CH4 (DShot) | ESC do motor 4 |
| PA2 | USART2_TX | RX do MTF-01 |
| PA3 | USART2_RX | TX do MTF-01 |
| PB3 | I2C2_SDA | SDA do BNO08x |
| PB10 | I2C2_SCL | SCL do BNO08x |
| PB4 | EXTI4 | INT do BNO08x (ativo baixo) |
| PB5 | GPIO | RST do BNO08x (ativo baixo) |
| PA9 | USART1_TX | RXD da Raspberry Pi Zero W (GPIO15, pino 10) |
| PA10 | USART1_RX | TXD da Raspberry Pi Zero W (GPIO14, pino 8) |
| PA1 | ADC1_IN1 | divisor da bateria — **não ligado** (`POWER_VBAT_ENABLED 0`) |
| PB0 | ADC1_IN8 | CURR do ESC por 1 kΩ — **não ligado** (`POWER_CURRENT_ENABLED 0`) |
| PA11 / PA12 | USB D− / D+ | PC |

A leitura de bateria e corrente já existe em `power.h/.c`, mas fica
desligada pelas duas chaves acima até o circuito ser montado: com elas em 0,
o firmware não toca no ADC nem nos pinos e informa "desconhecido" à Pi.

Todos os módulos devem compartilhar o GND. O MTF-01 é alimentado com 5 V e
usa sinais UART de 3,3 V. No BNO08x, a tensão de alimentação depende da placa
GY-BNO08X utilizada; SDA e SCL devem ter pull-ups para 3,3 V. A posição de
cada motor (M1..M4) está em [`docs/controle_lqg.md`](docs/controle_lqg.md).

## Mapa do repositório

```text
DroneController/
├── App/                    código próprio do firmware
│   ├── Inc/                contratos e tipos públicos
│   │   ├── control/        Kalman + LQR do Simulink, medidas, voo, parâmetros, ganhos
│   │   └── dronelink/      comunicação com a Raspberry Pi
│   └── Src/                implementações (mesmas subpastas) e driver SH-2
├── Core/                   inicialização gerada pelo CubeMX
├── Drivers/                CMSIS e HAL da ST
├── Middlewares/            FreeRTOS, MAVLink (gerado) e USB Device
├── USB_DEVICE/             configuração da porta USB CDC
├── protocol/               fonte XML do dialeto MAVLink próprio
├── reference/              código exportado pelo Simulink (referência dos testes)
├── docs/                   documentação consolidada
├── tests/                  testes executados no computador
├── tools/                  build, importação do Simulink, MAVLink, logs
│   └── pi/                 dronectl: ferramenta da Raspberry Pi
├── Doxyfile                documentação do código (doxygen)
├── DroneController.ioc     configuração do CubeMX
└── README.md               entrada para novos colaboradores
```

`Debug/`, `Release/` e `build/` não aparecem no Git: são recriados localmente.

### Código próprio

| Arquivo | Para que serve |
|---|---|
| `control/lqg_ert.h/.c` | Kalman de regime + LQR com pré-compensador: o `STM32_step()` do Simulink, legível e sem globais. |
| `control/lqg_ert_model.h/.c` | A, B, C, Kd, Kfd, L e M do Simulink. O `.c` é gerado por `tools/ert_import.py`. |
| `control/meas.h/.c` | Leituras do BNO08x e do MTF-01 → medidas `y` do modelo. |
| `control/flight_ctrl.h/.c` | Task de 100 Hz: estados do voo, referência, failsafes, saída e telemetria. |
| `control/flight_command.c` | Implementa `dronelink/flight_command_api.h` com o voo. |
| `control/ctrl_params.h/.c` | Parâmetros ajustáveis em tempo de execução. |
| `control/ctrl_gains.h/.c` | Ganhos em uso e troca atômica de ganhos enviados pela Pi. |
| `control/ctrl_port.h` | Seção crítica (FreeRTOS no firmware, nada nos testes). |
| `motors.h/.c` | DShot600 a 1 kHz e arbitragem bancada/voo. |
| `usb_console.h/.c` | Console de texto pela USB: bancada, voo e parâmetros. |
| `flight_config.h` | Chave `LQG_FLIGHT_ENABLED` e valores padrão dos parâmetros. |
| `sensors.h/.c` | Inicia as tasks do MTF-01 e do BNO08x. |
| `power.h/.c` | Bateria (PA1) e corrente (PB0) pelo ADC1; desligado até o circuito existir. |
| `bno08x.h/.c`, `bno_i2c_header.h` | Driver do BNO08x, transporte I2C, reset e reconexão. |
| `mtf01.h/.c`, `mtf_parser.h/.c` | Recepção do MTF-01 por DMA e decodificação do Micolink. |
| `attitude.h/.c` | Quatérnio, Euler, inclinação e rotação pela guinada. |
| `dshot.h/.c` | Geração dos quadros DShot600 no TIM4 com DMA burst. |
| `sensor_log.h/.c` | Log de texto pela USB por fila (MTF, BNO, MOT, EST, SYS). |
| `dma_ring.h` | Regra compartilhada para consumir DMA circular das USARTs. |

Dentro de `dronelink/`:

| Arquivo | Para que serve |
|---|---|
| `dronelink_app` | Cria e executa a task do enlace. |
| `dronelink_transport` | USART1, DMA, filas e envio/recepção de bytes. |
| `dronelink_protocol` | Mensagens de voo, enlace, autorização e telemetria básica. |
| `dronelink_tuning` | Parâmetros (PARAM_*) e envio de ganhos (DRONE_GAIN_*). |
| `dronelink_telemetry` | ATTITUDE, LOCAL_POSITION_NED e DRONE_CTRL_DEBUG. |
| `dronelink_mavlink.h` | Ponto único de inclusão do MAVLink e envio de mensagens. |
| `dronelink_transaction` | Ordem, repetição e estado dos comandos de trecho. |
| `dronelink_boot_identity` | Número de boot persistido na W25Q64. |
| `dronelink_config` | IDs MAVLink, períodos, timeouts e buffers. |
| `flight_command_api.h` | Contrato entre o protocolo e o voo. |

`App/Src/sh2/` é o driver SH-2 da CEVA, código externo vendorizado; consulte
`UPSTREAM.txt` antes de alterá-lo.

### Ferramentas, testes e documentação

| Arquivo | Para que serve |
|---|---|
| `tools/build.py` | Compila todas as fontes ARM e gera ELF, HEX, BIN, mapa e manifesto. |
| `tools/ert_import.py` | Gera `lqg_ert_model.c` a partir da exportação do Simulink. |
| `tools/gen_mavlink.py` | Regenera os headers C do MAVLink a partir do `protocol/drone_controller.xml`. |
| `tools/pi/` | `dronectl`, a ferramenta da Raspberry Pi ([README](tools/pi/README.md)). |
| `tools/read_logs.ps1`, `tools/motor_console.ps1` | Log e console USB no Windows. |
| `tests/run.sh` | Roda todas as suítes de PC e as verificações cruzadas. |
| `tests/test_lqg_ert.c` | Firmware × código gerado pelo Simulink: idêntico em double, erro medido em float. |
| `tests/test_ctrl_gains.c`, `test_ctrl_params.c`, `test_meas.c` | Troca de ganhos, parâmetros e conversão das medidas. |
| `tests/test_mtf.c`, `test_bno_header.c`, `test_regressions.c` | Parser Micolink, cabeçalhos SHTP e DMA circular. |
| `docs/controle_lqg.md` | **Comece aqui para o controle**: modelo, malha, parâmetros, ajuste pela Pi, roteiro de bancada. |
| `docs/guia_teste_motores.md` | Teste dos motores pelo console USB. |
| `docs/acompanhamento_tecnico.md` | Histórico: bring-up dos sensores, calibrações e decisões anteriores. |

## Getting started

### 1. Abrir e compilar

Instale o STM32CubeIDE e o STM32CubeProgrammer. No CubeIDE, importe a pasta
raiz como projeto existente, selecione **Debug**, pressione **F5**, faça
**Project > Clean** e **Ctrl+B**. O HEX sai em `Debug/DroneController.hex`.

Sem a IDE (Windows ou Linux, com o compilador do CubeIDE instalado):

```bash
python tools/build.py           # build/Release/DroneController.hex
python tools/build.py --debug   # build/Debug
```

### 2. Gravar o HEX em modo DFU

1. Feche qualquer terminal que esteja usando a porta COM da placa.
2. Coloque a Black Pill em modo DFU usando BOOT0 e RESET.
3. No STM32CubeProgrammer, conecte por **USB** ao dispositivo DFU.
4. Abra o `.hex`, inicie **Download** e aguarde a verificação.
5. Retorne BOOT0 para 0 e reinicie a placa.

O HEX já carrega os endereços; `0x08000000` só é necessário para o BIN.

### 3. Logs e console USB

```powershell
.\tools\motor_console.ps1 -Port COM5 -OutputPath logs\bancada.txt
```

O console liga o DTR (sem ele nada arma), mostra o log e envia o que você
digitar. `help` lista os comandos; `log off` silencia as linhas de sensores;
`lqg` mostra o estado do voo e a estimativa; `dry 0.5` faz um ensaio a seco
completo. Enter vazio para tudo. As linhas do log estão descritas em
`App/Inc/sensor_log.h`.

### 4. Raspberry Pi

Siga [`tools/pi/README.md`](tools/pi/README.md). Em resumo, por SSH na Pi:

```bash
cd DroneSTM32/tools/pi
.venv/bin/python -m dronectl monitor           # filtragem dos sensores ao vivo
.venv/bin/python -m dronectl hover --dry --debug
```

### 5. Testes no computador

```bash
sh tests/run.sh
```

Compila as suítes C com AddressSanitizer e UndefinedBehaviorSanitizer, confere
o firmware contra o código do Simulink, confere que `lqg_ert_model.c` está em
dia com a exportação, que o firmware e a Pi calculam o mesmo CRC dos ganhos e,
com `numpy`, `pymavlink` e `pyserial` instalados, roda os testes da
ferramenta da Pi contra um STM32 simulado. Os testes não substituem ensaios
com o hardware.

### 6. Documentação do código

```bash
doxygen        # gera build/docs/html/index.html
```

## Por onde começar a ler o código

1. **Configuração física:** `DroneController.ioc`, `Core/Inc/main.h` e
   `Core/Src/freertos.c` (ordem em que as tasks são criadas).
2. **O controle do TCC:** [`docs/controle_lqg.md`](docs/controle_lqg.md),
   depois `control/lqg_ert.h` e `lqg_ert.c` lado a lado com
   `reference/STM32_ert_rtw/STM32.c`.
3. **Sensores → medidas:** `control/meas.h`, depois `bno08x.h` e `mtf01.h`.
4. **Voo:** `control/flight_ctrl.h` (estados e contrato) e só então o `.c`.
5. **Pi:** `protocol/drone_controller.xml`, `dronelink/flight_command_api.h`,
   `dronelink_tuning.h` e `tools/pi/dronectl/`.

Uma regra útil: arquivos `.h` explicam **o que** um módulo oferece; arquivos
`.c` explicam **como** ele faz.

## Cuidados ao modificar

- Não edite `App/Src/control/lqg_ert_model.c` à mão: exporte de novo no
  Simulink e rode `tools/ert_import.py`.
- Se alterar `protocol/drone_controller.xml`, rode `tools/gen_mavlink.py`; a
  Pi regenera o lado dela sozinha.
- Preserve alterações próprias dentro dos blocos `USER CODE` dos arquivos
  gerados pelo CubeMX.
- Trate `Drivers/`, `Middlewares/` e `App/Src/sh2/` como dependências externas.
- Teste motores sempre sem hélices; `LQG_FLIGHT_ENABLED` só vai para 1 depois
  do roteiro de bancada.
