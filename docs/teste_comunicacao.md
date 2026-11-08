# Teste de comunicação Raspberry Pi ↔ STM32 (branch `comunicacao-pi-stm32`)

Teste isolado do enlace: a Pi manda comandos de voo um de cada vez e só manda o
próximo depois de o STM32 responder que o drone chegou e **ficou estável**. Para
não depender do voo, o STM32 roda um **drone simulado** (`FLT_SIM = 1`): os
motores não giram, e o mesmo LQG do TCC controla o modelo do TCC no lugar dos
sensores. A Pi, a UART, o protocolo e a máquina de voo são os de verdade.

## Ligação

| Pi Zero W | STM32 (Black Pill) |
|---|---|
| GPIO14 TXD (pino 8) | PA10 (USART1_RX) |
| GPIO15 RXD (pino 10) | PA9 (USART1_TX) |
| GND | GND |

115200 baud, 3,3 V nos dois lados. Configuração da UART da Pi em
[`tools/pi/README.md`](../tools/pi/README.md).

## Como rodar

Na Pi, a partir de `tools/pi` (venv com `requirements.txt`):

```bash
.venv/bin/python -m dronectl missao --sim                 # shell interativo
.venv/bin/python -m dronectl missao --sim missao.txt      # arquivo de missão
.venv/bin/python -m dronectl missao --sim --desligar-sim  # volta FLT_SIM=0 ao sair
```

`--sim` liga `FLT_SIM = 1` no STM32 (só desarmado). Sem `--sim` e com
`FLT_DRY_RUN = 1`, os motores não giram e nada simula o drone: a decolagem nunca
termina (a ferramenta avisa).

Exemplo de arquivo (`#` comenta; uma linha por comando). Medidas em números
inteiros: **centímetros**, graus e segundos:

Os comandos usam a nomenclatura aeronáutica em inglês (a mesma do PX4, do
ArduPilot e do MAVLink); as mensagens da Pi continuam em português.

```text
takeoff 50                     # sobe 50 cm
yaw 90                         # nariz para a direita inicial
forward 100                    # anda 1 m
goto 100 0                     # gira de frente para (100, 0) cm e anda reto até lá
waypoints 200,0 200,100 0,0    # três waypoints, na ordem dada
goto 100 100 120               # vai a (100, 100) subindo para 120 cm
route 200,0,60 0,200,140 200,200,100   # ordem pelo algoritmo genético, volta à partida
land
```

Missão pronta para espaço pequeno: `tools/pi/missoes/quadrado_40cm.txt` decola a
30 cm, roda `route` sem pontos (que lê o quadrado de 40 cm de
`tools/pi/missoes/pontos.txt`), volta à partida e pousa:

```bash
python -m dronectl missao missoes/quadrado_40cm.txt --sim
```

No fim do arquivo, ou no primeiro erro, a Pi pousa o drone.

## Comandos

| Comando | O que faz | Precisa estar |
|---|---|---|
| `takeoff <altura cm>` | arma e sobe (até 150 cm); termina quando o drone está estável no ar | no chão (STANDBY) |
| `yaw <graus>` | gira no lugar; + = horário visto de cima (para a direita) | estável no ar (HOLD) |
| `forward` / `back <cm>` | anda no eixo do nariz | HOLD |
| `right` / `left <cm>` | anda de lado | HOLD |
| `climb` / `descend <cm>` | muda a altura | HOLD |
| `goto <x cm> <y cm> [z cm]` | **calcula o trecho**: gira para ficar de frente para o waypoint e anda reto até ele | HOLD |
| `waypoints <x,y[,z]> ...` | `goto` para cada waypoint, na ordem dada | HOLD |
| `route [alg] [x,y[,z] ...]` | **executa a rota**: escolhe a ordem de visita (caixeiro-viajante) com o algoritmo genético, visita todos e volta à partida ([`rotas_tsp.md`](rotas_tsp.md)). Sem pontos, lê `tools/pi/missoes/pontos.txt` (ou o arquivo de `--pontos`). `alg` opcional, só para comparação: `greedy`, `2opt`, `2optmulti`, `bruteforce` (até 9 pontos) ou `exact` (até 16) | HOLD |
| `rtl` | *return to launch*: `goto 0 0` (ponto de decolagem) | HOLD |
| `hold` | para onde está | no ar |
| `wait <s>` | espera mostrando estado e posição | — |
| `land` | pousa e desarma | — |
| `status`, `help`, `exit` | — | — |

A Pi converte os centímetros para metros, a unidade do MAVLink. Coordenadas de
`goto`, `waypoints` e `route`: centímetros a partir do ponto de decolagem, no
referencial da decolagem: **x = para a frente** (para onde o nariz apontava ao
energizar o STM32), **y = para a direita**, **z = altura acima do chão** (15 a
150 cm; sem z, mantém a altura). O cálculo é `yaw = atan2(Δy, Δx) − guinada
atual` (pelo lado mais curto) e `forward = √(Δx² + Δy²)` com `climb/descend = Δz`
no mesmo trecho (o STM32 faz as rampas horizontal e vertical juntas), usando a
posição e a guinada que o STM32 publica. A distância é recalculada depois do giro.

## Protocolo (DroneLink, MAVLink 2, `protocol/drone_controller.xml`)

```text
 Pi                                              STM32
 ── HEARTBEAT (4 Hz) ─────────────────────────►  enlace vivo
 ── DRONE_CONTROL_LEASE (5 Hz, durante o voo) ─►  autorização do operador
 ◄─────────────── HEARTBEAT (2 Hz: estado de voo em custom_mode, armado)
 ◄─────────────── DRONE_PROTOCOL_INFO (1 Hz: boot_id, capacidades, estado)
 ◄─────────────── ATTITUDE e LOCAL_POSITION_NED (5 Hz: estimativa do Kalman)

 takeoff 50:
 ── COMMAND_LONG ARM_DISARM (1) ──────────────►
 ◄─────────────────────────── COMMAND_ACK aceito (ou recusado + motivo)
 ── DRONE_LEG_COMMAND TAKEOFF_RELATIVE z=-0.5 m ►  (boot_id, mission_id, command_id, leg_index)
 ◄──────────────── DRONE_LEG_ACK IN_PROGRESS     estado TAKEOFF: sobe em rampa
            ... drone chega a 50 cm e fica 1 s parado dentro da tolerância ...
 ◄──────────────── DRONE_LEG_ACK ACCEPTED        estado HOLD: pronto para o próximo

 yaw 90:
 ── DRONE_LEG_COMMAND ROTATE_RELATIVE yaw=+π/2 ►
 ◄──────────────── DRONE_LEG_ACK IN_PROGRESS     estado LEG
 ◄──────────────── DRONE_LEG_ACK ACCEPTED        estável de novo: HOLD

 forward 100:
 ── DRONE_LEG_COMMAND TRANSLATE_RELATIVE (BODY_FRD) x=1.0 m ►
 ◄──────────────── IN_PROGRESS ... ACCEPTED
```

- **Estável** (firmware, `check_goal`): a referência chegou ao alvo, a
  estimativa está dentro da tolerância (5 cm, 0,05 rad), e vx, vy, vz < 5 cm/s e
  p, q, r < 0,1 rad/s por **1 s seguido** (`FLIGHT_SETTLE_MS`). Só então o
  trecho termina, o estado vira HOLD e o ACK final sai.
- **Recusas:** com um trecho em andamento (TAKEOFF ou LEG) o STM32 responde
  `TEMPORARILY_REJECTED` + `ocupado`; fora de HOLD, `DENIED` + `estado errado`;
  alvo fora dos limites (abaixo do pouso, acima de 1,5 m), `fora dos limites`.
  A Pi confere o estado antes de mandar, mas o STM32 é quem garante.
- **Transações:** cada trecho tem `mission_id`, `command_id` e `leg_index`
  crescentes. Mandar o mesmo de novo (resposta perdida) devolve a mesma resposta
  sem executar duas vezes; o `boot_id` errado (STM32 reiniciou) é recusado.
- **Tempo máximo:** cada trecho leva um `timeout_ms` (a Pi calcula pela rampa +
  20 s). Se estourar, o STM32 encerra o trecho como falho e fica em HOLD.
- **Perda de enlace:** sem HEARTBEAT da Pi por 1 s o STM32 segura (HOLD); sem a
  autorização por mais tempo, pousa sozinho.

## Drone simulado (`FLT_SIM`, `control/sim_plant.h`)

- Estado do modelo do TCC, `x[k+1] = A x[k] + B du[k]`, a 100 Hz; medidas
  `y = C x`; chão em 8 cm de altura do sensor (não afunda).
- Recebe o mesmo `du` que os motores receberiam (com saturação). Motores sempre
  parados (força ensaio a seco).
- No PC (`tests/test_sim_plant.c`, mesmo LQG e critério): decolar 0,5 m estável
  em 5,2 s; girar 90° em 4,7 s; andar 1 m em 7,5 s, terminando a 0 cm do ponto.
- Pelo console USB também dá para conferir: `param set FLT_SIM 1`, `hover 0.5`,
  `yaw 90`, `move 1 0`, `land`.

## Testes

- `sh tests/run.sh` (raiz): firmware, incluindo `test_sim_plant`.
- `tools/pi`: `python3 -m unittest discover -s tests` — interpretação dos
  comandos, cálculo de rota e a missão inteira contra o STM32 falso
  (`tests/fake_fc.py`, que também recusa trechos fora de HOLD).

## Ensaio integrado pela UART (07/11/2026)

Pi Zero W (`r1ddax.local`, código em `~/drone` instalado por
`sh tools/pi/deploy.sh r1ddax@r1ddax.local`) ligada ao STM32 pela USART1
(PA9 → pino 10 da Pi, PA10 → pino 8, GND comum), drone simulado (`FLT_SIM = 1`).
Roteiro: `tools/pi/ensaios/ensaio_dronelink.py` (na Pi:
`cd ~/drone/tools/pi && ~/drone-venv/bin/python ensaios/ensaio_dronelink.py`).
Registro: [`resultados_comunicacao/dronelink_20261107_225922.json`](resultados_comunicacao/dronelink_20261107_225922.json).

| Etapa | Resultado |
|---|---|
| Enlace parado, 60 s | HEARTBEAT 1,98 Hz, SYS_STATUS 2,00, PROTOCOL_INFO 1,00, ATTITUDE e LOCAL_POSITION_NED 4,97 Hz; 895 mensagens, 552 B/s; **0 perdidas, 0 erros de CRC**, 4 fora de ordem (fila crítica passa à frente) |
| Latência PARAM_REQUEST_READ → PARAM_VALUE (100) | mediana 10,2 ms, média 10,9, p95 17,0, máx 21,2 |
| Recusas | sem autorização, no chão, boot errado, destino errado, sequência, NaN, ocupado, abaixo do chão, 25 m, colisão de IDs: todas com o motivo certo |
| Pedido repetido | mesma resposta; andou 0,294 m (executou uma vez) |
| Cancelar trecho | cancelado, fica em HOLD |
| `takeoff 30` / `route` / `land` | AG em 11,5 s na Pi; 8 trechos, 41 s; erro máximo 1,4 cm por ponto |
| Autorização cortada no meio de um trecho | HOLD 0,88 s, pouso 5,98 s, desarmado 8,99 s |
| Processo da Pi morto no meio de um trecho | HOLD 1,30 s, pouso 5,29 s, desarmado 8,31 s |

Corrigido por causa deste ensaio:

- **Sequência MAVLink única** no firmware (`dronelink_mavlink.h`): o estado do
  canal ficava em `static` de função `inline` e cada `.c` tinha a sua cópia;
  telemetria e protocolo saíam com sequências independentes.
- **`LOCAL_POSITION_NED` com z a partir do chão** (`dronelink_telemetry.c`):
  antes saía o z do estimador (chão em −8 cm), e a Pi via 8 cm a mais.
- **Pi: resposta de trecho conferida pela missão** (`link.py`): um
  `DRONE_LEG_ACK` antigo, de outra missão com o mesmo `command_id`, encerrava o
  trecho atual. Teste de regressão em `tests/test_missao.py`.
