# Estado atual do sistema de sensoriamento e estimação

Documentação técnica do subsistema de sensores e estimação de deslocamento do
projeto DroneController. Descreve o que existe hoje, como está organizado,
como as partes se comunicam, e — principalmente — **de onde veio cada número
de configuração**, separando o que foi medido do que foi copiado de
referência e do que foi escolha de engenharia.

Escopo: MTF-01, BNO08x, estimadores de deslocamento, log e captura para
replay. O subsistema `dronelink` (comunicação MAVLink com Raspberry Pi) está
fora do escopo deste documento.

**Situação em 2026-09-16:** fase de percepção encerrada; resumo e pendências
na Seção 8.5.

Para a evolução histórica do algoritmo e os resultados comparativos de cada
estágio, ver `docs/evolucao_algoritmo_deslocamento.md`.

---

## 1. Visão geral

O sistema lê dois sensores, estima deslocamento horizontal e publica tudo via
USB. Não há controle de voo, motores ou malha fechada — este é o subsistema
de **percepção**, que produz estimativas para um controlador futuro consumir.

```
   MTF-01  ──UART/DMA──►  mtf01.c ──► mtf_parser.c ──┐
   (fluxo + LiDAR)                                    │
                                                      ├──► displacement.c ──► 3 estimadores
   BNO08x  ──I2C/SH-2──►  bno08x.c ──────────────────┘         │
   (IMU)                                                        ▼
                                                          sensor_log.c ──USB CDC──► PC
                                             (build Capture: capture.c ──USB CDC──► PC)
```

Existem dois firmwares gerados a partir do mesmo código:

| Build | Arquivo | Uso |
|---|---|---|
| Normal | `build/Release/DroneController.hex` | **Firmware de uso normal e base do firmware de voo.** Estimadores rodam desde o boot; logs de texto pela USB |
| Capture | `build/Capture/DroneController.hex` | **Só bancada.** Grava tudo em binário para replay; o estimador só roda enquanto o PC está gravando |

**Hardware:** STM32F411CEU6 ("Black Pill"), FreeRTOS (tick de 1000 Hz, heap de
32768 bytes), HAL F4 V1.28.3.

---

## 2. Mapa de arquivos

Todo código próprio vive em `App/`. O driver SH-2 da CEVA é vendorizado **sem
modificações** em `App/Src/sh2/`.

### 2.1. Módulos de sensor

| Arquivo | Linhas | Responsabilidade |
|---|---|---|
| `App/Inc/bno08x.h` / `App/Src/bno08x.c` | 57 / 298 | Task do IMU, I2C2, pinos RST/INT, camada `sh2_Hal_t` sobre o driver CEVA |
| `App/Inc/mtf01.h` / `App/Src/mtf01.c` | 27 / 71 | Task do MTF-01, USART2 com DMA circular e stream buffer |
| `App/Inc/mtf_parser.h` / `App/Src/mtf_parser.c` | 12 / 55 | Decodificador do protocolo Micolink — **puro**, sem HAL |
| `App/Inc/bno_i2c_header.h` | 13 | Regra de continuação de cabeçalho SHTP — header-only, **puro** |

### 2.2. Estimação

| Arquivo | Linhas | Responsabilidade |
|---|---|---|
| `App/Inc/displacement.h` | 180 | API pública dos três estimadores e tipos compartilhados |
| `App/Src/displacement.c` | 225 | Task que lê os dois sensores e alimenta os estimadores |
| `App/Src/disp_estimator.c` | 119 | Estimadores ingênuo e compensado — **puros** |
| `App/Inc/flow_ekf.h` / `App/Src/flow_ekf.c` | 79 / 142 | Filtro de Kalman de posição/velocidade — **puro** |
| `App/Inc/flow_calibrator.h` / `App/Src/flow_calibrator.c` | 80 / 63 | Calibração de escala referenciada por giroscópio — **puro** |
| `App/Inc/attitude.h` / `App/Src/attitude.c` | 36 / 41 | Quatérnio → Euler, cos(inclinação), rotações — **puro** |

### 2.3. Infraestrutura

| Arquivo | Linhas | Responsabilidade |
|---|---|---|
| `App/Inc/sensors.h` / `App/Src/sensors.c` | 11 / 15 | Agregador fino: só `sensors_init()`, que sobe a captura (se houver) e as tasks dos sensores |
| `App/Inc/sensor_log.h` / `App/Src/sensor_log.c` | 59 / 181 | Log USB CDC, com grupos liga/desliga em tempo de compilação; seleciona o perfil Capture por `CAPTURE_BUILD` |
| `App/Inc/capture.h` / `App/Src/capture.c` | 226 / 359 | Captura binária evento a evento (formato 3), fila, task de envio e DIAG — só ativa no build Capture |
| `App/Inc/capture_session.h` | 13 | Máquina de estados da sessão: começa no DTR, uma por boot — **puro** |
| `USB_DEVICE/App/usbd_cdc_if.c` | 61 | Interface CDC: transmissão e detecção de DTR do PC |

### 2.4. Ferramentas no PC

| Arquivo | Função |
|---|---|
| `tools/build.py` | Compila Release/Debug e, com `--capture`, o build Capture; gera `manifest.json` com hashes |
| `tools/read_logs.ps1` | Lê os logs de texto do firmware normal |
| `tools/capture_binary.ps1` | Grava a captura binária do build Capture (ativa DTR, lê bytes crus) |
| `tools/decode_capture.py` | Converte a captura em CSVs e aponta perdas |
| `tools/replay.c` | Reexecuta os estimadores sobre a captura e compara com a placa |
| `tools/make_synthetic_capture.c` / `tools/verify_replay.py` | Captura sintética e sabotagens que provam que o replay detecta erros |

### 2.5. Testes de host

Executam no PC via `gcc`, sem hardware, todos de uma vez com `sh tests/run.sh`
(ASan/UBSan ligados). Stubs do HAL em `tests/stubs/`.

| Arquivo | Cobre |
|---|---|
| `tests/test_displacement.c` | Atitude, estimadores ingênuo e compensado, todos os gates |
| `tests/test_flow_ekf.c` | Filtro de Kalman: predição, correção, gate, recuperação |
| `tests/test_flow_calibrator.c` | Calibração: recuperação de escala, rejeições, diagnóstico |
| `tests/test_mtf.c` | Parser Micolink: fragmentação, checksum, sequência, ruído |
| `tests/test_bno_header.c` | Regra de continuação SHTP |
| `tests/test_regressions.c` | Defeitos corrigidos após a revisão técnica: reinício do relógio do MTF, filtro zerado, convergência sem correção, validade, DMA repetido, canal SHTP fora da faixa |
| `tests/test_capture.c` | Formato de captura: enquadramento, floats exatos, corrupção, registro grande demais, ressincronização, tamanhos iguais aos do decoder |
| `tests/test_capture_session.c` | Sessão de captura: espera DTR, ordem de ativação, uma por boot |

Além das oito suítes, o `run.sh` roda `tools/verify_replay.py`: gera uma
captura sintética, exige reprodução exata e exige que 12 sabotagens sejam
detectadas.

**Princípio de separação:** toda lógica não-trivial e sem dependência de
hardware foi isolada em arquivos "puros" (sem FreeRTOS, sem HAL), permitindo
teste no host com sanitizers. As tasks são camadas finas que só fazem I/O e
chamam essas funções puras. Foi essa separação que permitiu reproduzir em
teste automatizado os defeitos encontrados em bancada.

---

## 3. Tasks do FreeRTOS

Criadas por `sensors_init()` (`App/Src/sensors.c`), chamada de
`MX_FREERTOS_Init()` em `Core/Src/freertos.c`.

| Task | Stack | Prioridade | Período | Bloqueia em |
|---|---|---|---|---|
| `MTF01` | 512 words (2 KB) | `IDLE+3` | evento | `xStreamBufferReceive`, timeout 5 ms |
| `BNO08X` | 1024 words (4 KB) | `IDLE+2` | evento | `vTaskDelay(1)` no laço de serviço |
| `Displace` | 512 words (2 KB) | `IDLE+2` | 5 ms | `vTaskDelay(5)` |
| `DroneLink` | — | `IDLE+2` | — | (fora do escopo) |
| `SensorLog` | 1024 words (4 KB) | `IDLE+1` | 500 ms | `vTaskDelay(500)` — **não é criada no build Capture** |
| `Capture` | 512 words (2 KB) | `IDLE+1` | evento | `xStreamBufferReceive`, timeout 20 ms — **só no build Capture** |
| `defaultTask` | 2 KB | `osPriorityNormal` | 500 ms | `osDelay(500)` — só pisca o LED |

### 3.1. Por que essas prioridades

A ordem reflete **o custo de atrasar cada uma**, não a importância conceitual:

- **`MTF01` é a mais alta (`IDLE+3`)** porque é a única que pode *perder
  dados*: o DMA escreve continuamente num buffer circular de 256 bytes, e se
  a task não drenar o stream buffer a tempo, bytes são descartados
  (contabilizados em `drop`). Ela bloqueia com timeout de 5 ms, então cede
  CPU constantemente apesar da prioridade alta — prioridade alta aqui
  significa "acorda rápido quando há dado", não "monopoliza a CPU".

- **`BNO08X` e `Displace` no meio (`IDLE+2`)** porque o BNO tem buffer próprio
  no chip (o driver SH-2 lê quando o pino INT indica dado pronto) e o
  `Displace` apenas consome snapshots já publicados — nenhum dos dois perde
  dado se atrasar alguns milissegundos.

- **`SensorLog` é a mais baixa (`IDLE+1`)** porque perder uma linha de log é
  inofensivo — o próprio código conta isso em `log_drop` e segue adiante. Log
  nunca deve competir com aquisição.

### 3.2. Por que esses tamanhos de stack

`BNO08X` e `SensorLog` têm 4 KB porque ambos têm chamadas profundas: o
primeiro entra no driver SH-2 vendorizado (várias camadas de callback), o
segundo usa `snprintf` com dezenas de argumentos. `MTF01` e `Displace` têm
2 KB porque são rasos — o buffer grande do log é `static`, não vive na pilha.

O heap total do FreeRTOS é de 32768 bytes. No build normal, com todas as
tasks criadas, o log reporta cerca de **11 KB livres** (`SYS heap=`). Essa
folga é o que permite criar novas tasks sem reconfigurar o heap.

No build Capture a fila de captura (8 KB) e a task `Capture` ocupam mais
heap. Medido pelo DIAG em bancada (2026-09-16): menor heap livre de **2688
bytes**, constante durante a gravação. Margens de pilha medidas no mesmo
build: `Displace` 1100 bytes, `MTF01` 1304, `Capture` 1460 e `BNO08X` 2972.

### 3.3. Tratativas de erro por task

| Situação | Tratativa |
|---|---|
| BNO não responde no I2C | `hub_open()` falha → `fault=NO_DEVICE` → retenta após 2 s com pulso de RST |
| Falha de I2C durante operação | `transport_failed` → encerra sessão SH-2 → recupera barramento → reinicializa periférico → retenta |
| Reports do BNO param de chegar | Detecção de stall (1 s sem dado) → `fault=STALLED` → reinicia sessão |
| UART do MTF com erro | `restart` → aborta DMA, reseta stream buffer e parser, reinicia recepção |
| Bytes perdidos no stream buffer | Parser é resetado (dado descontínuo não pode ser confiado) e contado em `drop` |
| Checksum Micolink inválido | Frame descartado, contado em `crc_err`; procura próximo header `0xEF` no que sobrou |
| USB ocupado/desconectado | Linha de log descartada, contada em `log_drop`; aquisição não para |
| Fila de captura cheia ou mutex ocupado (build Capture) | Registro descartado e contado no DIAG por tipo; fila cheia deixa buraco visível na sequência |

**Princípio:** nenhuma falha de sensor trava o sistema. Cada caminho de erro
reinicia o subsistema afetado e continua, com contadores no log para
diagnóstico posterior.

---

## 4. Fluxo de dados detalhado

### 4.1. MTF-01 → amostra

1. **DMA circular** preenche `dma_buffer[256]` continuamente via USART2 a
   115200 8N1.
2. **Callback de idle/half/complete** (`mtf_rx_event`) empurra os bytes novos
   para um **stream buffer** de 1024 bytes. Roda em contexto de interrupção.
3. **`mtf_task`** drena o stream buffer em blocos de até 128 bytes e alimenta
   `mtf_parser_byte()` byte a byte.
4. **Parser** monta o frame Micolink, valida checksum, extrai os campos e
   publica em `mtf_sample_t` sob seção crítica.

O parser aceita **somente** payload `0x51` de 20 bytes (mensagem de
fluxo+distância do MTF-01). Ele detecta reinício do sensor comparando o
relógio interno do sensor (se andar para trás, é reboot, não milhares de
amostras perdidas).

### 4.2. BNO08x → amostra

1. **`hub_open()`** pulsa RST, aguarda, e sonda os endereços I2C `0x4A`/`0x4B`.
2. **Driver SH-2 da CEVA** gerencia o protocolo SHTP; nossa camada
   `sh2_Hal_t` implementa leitura/escrita I2C e o pino INT.
3. **Três relatórios habilitados a 100 Hz cada**: `SH2_ROTATION_VECTOR`
   (quatérnio), `SH2_GYROSCOPE_CALIBRATED` (rad/s), `SH2_LINEAR_ACCELERATION`
   (m/s², gravidade já removida pela fusão do chip).
4. **Callbacks** decodificam e publicam em `bno_sample_t`.

**Detalhe não óbvio do I2C do BNO08x:** cada transação repete o cabeçalho
SHTP de 4 bytes. É preciso ler o cabeçalho, descobrir o tamanho, e então ler
o pacote **completo** (não `tamanho − 4`), verificando que o cabeçalho
repetido bate. Essa regra está isolada e testada em `bno_i2c_header.h`.

### 4.3. Amostras → deslocamento

`displacement_task` roda a 5 ms, mas **não integra por ciclo de polling** —
integra **por frame do sensor**, detectado pela mudança do contador
`m.frames`. O `dt` usado vem do **relógio interno do MTF-01** (`time_ms`), não
do nosso tick, o que mantém a integral correta mesmo se o polling jitterar.

Cada frame alimenta, na ordem:
1. Calibrador (se ativo)
2. Estimador ingênuo (`disp_raw_update`)
3. Estimador compensado (`disp_fused_update`)
4. Predição do EKF (sempre, mesmo com medida ruim)
5. Correção do EKF (só se o compensado aceitou o frame)

---

## 5. Técnicas empregadas

### 5.1. Compensação de taxa do corpo

Um sensor de fluxo óptico mede a soma de translação e rotação. Subtrai-se a
taxa do giroscópio para isolar a translação:

```
fluxo_compensado = fluxo_medido × escala + sinal × giroscópio
velocidade_corpo = fluxo_compensado × alcance
```

A escala é aplicada **antes** de misturar com o giroscópio. Aplicar depois
escalaria também o termo do giroscópio, que não é o que se está calibrando —
esse foi um defeito real, invisível enquanto a escala era 1,0.

### 5.2. Rotação para o referencial do mundo

A velocidade compensada está no referencial do corpo. Antes de integrar, é
rotacionada pelo yaw atual — sem isso, um veículo que gira embaralha a
própria trajetória acumulada. Verificado contra log: com yaw estável em
−141,23°, o vetor compensado foi exatamente o cru girado de −141,21° a
−141,24°, com magnitude preservada em 1 parte em 1600.

### 5.3. "Confiar no zero do sensor"

Se o fluxo bruto de um eixo foi quantizado a zero, aquele eixo não viu
movimento — e o giroscópio, que resolve muito mais fino, não tem direito de
opinar sobre isso. Sem essa regra, girar abaixo do degrau de quantização
fazia a compensação **inventar** deslocamento (medido: ~3 mm por frame).

Aplicada sobre o fluxo **bruto** (não o escalado), porque reflete o piso de
quantização físico do sensor, que uma constante de calibração não altera.

### 5.4. Filtro de Kalman com gate de inovação

Dois filtros independentes de 2 estados (posição, velocidade), um por eixo —
os eixos não se acoplam nem na dinâmica nem na medida, então isso é
matematicamente idêntico a um filtro 4×4, sem a escrituração matricial.

- **Predição:** aceleração linear do BNO rotacionada para o mundo. Roda
  **todo frame**, mesmo quando a medida é rejeitada.
- **Correção:** velocidade do estimador compensado, aceita só se a inovação
  passar no gate de consistência (3σ).
- **Recuperação:** se o gate rejeitar por 0,5 s seguidos, o filtro adota a
  medida e alarga a incerteza — um gate que rejeita tudo é pior que gate
  nenhum.

**Por que o gate mora no filtro e não no integrador:** rejeitar uma amostra
de uma soma corrente equivale a afirmar "velocidade era zero naquele
intervalo", o que enviesa o total. Num filtro, a rejeição apenas deixa a
predição de pé. Isso foi medido: o gate no integrador produziu 21 mm de
deriva contra 2 mm sem gate nenhum.

### 5.5. Calibração referenciada por giroscópio

Durante rotação pura, um sensor de fluxo perfeitamente calibrado leria
exatamente a taxa que o giroscópio mede. A discrepância média **é** o erro de
escala — sem depender de nenhuma medida humana de distância. Técnica copiada
do `AP_OpticalFlow_Calibrator.cpp` do ArduPilot.

---

## 6. Catálogo de constantes: de onde saiu cada número

Esta seção existe porque constantes sem procedência são impossíveis de
revisar depois. Cada valor está classificado por origem.

### 6.1. Medidos em bancada neste projeto

| Constante | Valor | Onde | Como foi determinado |
|---|---|---|---|
| Latência de boot do BNO | ~80 ms | `bno08x.c` (motiva `PROBE_WINDOW_MS`) | Medido pelo campo `probe_ms` do log: tempo entre soltar RST e o primeiro ACK no I2C |
| `DISP_FLOW_LSB_RAD_S` | 0,2 rad/s | `displacement.h` | 34 de 34 amostras de fluxo caíram em múltiplos exatos de 20 na unidade nativa do sensor |
| `flow_scale_x` | 0,944 | `displacement.c` | Calibrador do giroscópio, 3239 amostras, convergido e estável |
| `flow_scale_y` | 0,942 | `displacement.c` | Calibrador do giroscópio, 2215 amostras (0,21% de diferença do eixo X) |
| `gyro_sign_x` | +1,0 | `displacement.c` | Rolagem isolada: fluxo X respondeu com **sinal oposto** ao giro X em 214/218 amostras (98,2%) |
| `gyro_sign_y` | −1,0 | `displacement.c` | Arfagem isolada: fluxo Y respondeu com **mesmo sinal** que o giro Y em 142/143 amostras (99,3%) |
| Pareamento de eixo | direto (X↔X, Y↔Y) | `disp_estimator.c` | Mesmas medidas acima; contraria o modelo de livro-texto (cruzado), pois os dois sensores estão montados com rotação relativa de 90° |

### 6.2. Copiados do ArduPilot

Valores de referência de uma implementação madura e amplamente testada em
voo, adotados para não reinventar parâmetros com pouca base empírica.

| Constante | Valor | Equivalente no ArduPilot |
|---|---|---|
| `MAX_FLOW_RAD_S` | 4,2 rad/s | Teto de sanidade em `AP_NavEKF3_Measurements.cpp` |
| `flow_noise_rad_s` | 0,25 | `FLOW_M_NSE` (padrão) |
| `gate_sigma` | 3,0 | `FLOW_I_GATE` (padrão 300 = 3,00 × 100) |
| `FLOW_CAL_MIN_RATE_RAD_S` | 0,349066 (20°/s) | `AP_OPTICALFLOW_CAL_ROLLPITCH_MIN_RADS` |
| `FLOW_CAL_MAX_YAW_RAD_S` | 0,349066 (20°/s) | `AP_OPTICALFLOW_CAL_YAW_MAX_RADS` |
| `FLOW_CAL_SCALE_MIN` / `MAX` | 0,20 / 4,00 | `AP_OPTICALFLOW_CAL_SCALE_MIN` / `MAX` |

### 6.3. Do fabricante ou do protocolo

| Constante | Valor | Origem |
|---|---|---|
| `I2C2 ClockSpeed` | 100 kHz | Consenso para BNO08x (clock stretching atípico); confirmado por implementação de referência funcional |
| `USART2 BaudRate` | 115200 | Especificação do MTF-01 |
| `REPORT_INTERVAL_US` | 10000 (100 Hz) | Taxa escolhida por ser o que o barramento I2C a 100 kHz carrega com três relatórios simultâneos |
| `FLOW_CM_S_AT_1M_TO_RAD_S` | 0,01 | Conversão de unidade: o MTF-01 reporta "cm/s a 1 m"; a 1 m, v m/s produz v rad/s, logo ÷100 |
| `MAX_RANGE_M` | 8,0 m | Alcance máximo do telêmetro do MTF-01 |
| `MIN_RANGE_M` | 0,08 m | Piso de operação **óptica** (~8 cm), não do telêmetro (~2 cm). De documentação secundária/comunidade, adotado por precaução |
| Payload Micolink `0x51` | 20 bytes | Especificação do protocolo Micolink |

### 6.4. Escolhas de engenharia

Valores sem fonte externa, escolhidos por julgamento. São os candidatos
naturais a ajuste se algum comportamento precisar mudar.

| Constante | Valor | Raciocínio |
|---|---|---|
| `PROBE_WINDOW_MS` | 400 ms | 5× a latência medida de 80 ms — margem generosa para um chip lento sem travar a inicialização |
| `RETRY_DELAY_MS` | 2000 ms | Lento o bastante para não saturar o barramento em falha persistente, rápido o bastante para recuperar sem intervenção |
| `STALL_TIMEOUT_MS` | 1000 ms | A 100 Hz, 1 s sem dado é ~100 relatórios perdidos — inequivocamente uma falha, não um soluço |
| `ONLINE_STALE_MS` | 250 ms | Mais apertado que o stall de propósito: um consumidor deve ver `online=0` **antes** da task decidir reiniciar a sessão |
| `POLL_PERIOD_MS` | 5 ms | Metade do período do MTF-01 (~100 Hz) — garante que nenhum frame passe despercebido |
| `LOG_PERIOD_MS` | 500 ms | Legível por humano em terminal sem inundar o USB |
| `MIN_QUALITY` | 20 | Abaixo disso a imagem não tem textura utilizável; valor conservador |
| `MIN_COS_TILT` | 0,80 (~37°) | Além disso o alcance medido deixa de mapear para altura de forma útil |
| `MIN_DT_S` / `MAX_DT_S` | 0,001 / 0,25 s | Um intervalo maior que 250 ms significa frames perdidos; extrapolar sobre esse buraco seria inventar dado |
| `DISP_MAX_ACCEL_M_S2` | 40 m/s² (~4 g) | Acima do que um multirrotor produz, abaixo dos ~250 m/s² medidos quando uma mão passou sob o sensor |
| `accel_noise` (EKF) | 0,5 m/s² | Cobre ruído do acelerômetro mais o que o modelo não representa; é o ajuste de "quão rápido o filtro pode mudar de ideia" |
| `MAX_REJECT_RUN` | 50 (0,5 s a 100 Hz) | Tempo até o filtro concluir que quem está errado é a predição, não a medida |
| `CONVERGED_VEL_VAR` | 0,09 (σ = 0,3 m/s) | Limiar acima do qual a estimativa de velocidade passa a valer a leitura |
| `INITIAL_VEL_VAR` | 1,0 | Velocidade inicial desconhecida; as primeiras medidas é que a fixam |
| `INITIAL_POS_VAR` | 0,0 | Posição inicial é a origem **por definição** (navegação estimada: o referencial é ancorado onde se começou) |
| `DMA_SIZE` | 256 bytes | ~9 frames Micolink de 27 bytes — folga confortável contra atraso da task |
| Stream buffer | 1024 bytes | 4× o buffer de DMA |
| Buffer do log | 2048 bytes | Pior caso medido com todos os grupos ligados: 1656 bytes |

**Sobre o buffer do log:** este é um caso onde a margem importa mais do que
parece. O log inteiro é um único `snprintf`; se ele estourar, o código
descarta a **mensagem inteira** em silêncio (a verificação `n < sizeof(line)`).
Um buffer subdimensionado não trunca a linha — faz o log desaparecer. Por isso
o valor foi dimensionado contra o pior caso com todos os contadores saturados,
não contra a saída típica.

---

## 7. Como usar o log

Grupos ligados/desligados em tempo de compilação no topo de
`App/Inc/sensor_log.h`. Grupo desligado é compilado fora: não gasta CPU nem
banda de USB.

| Grupo | Padrão | Conteúdo |
|---|---|---|
| `LOG_BNO_STATUS` | 0 | online, fault, endereço, firmware, `probe_ms`, resets |
| `LOG_BNO_DATA` | 0 | quatérnio, giroscópio, aceleração linear, contadores por relatório |
| `LOG_MTF` | 0 | fluxo bruto, alcance, contadores de frame/CRC/UART |
| `LOG_ATTITUDE` | 1 | ângulos de Euler e quatérnio |
| `LOG_HEIGHT` | 1 | alcance, altura corrigida por inclinação, `v_lsb_mm_s` |
| `LOG_POSITION` | 1 | deslocamento acumulado dos três estimadores |
| `LOG_POSITION_D` | 1 | incremento por frame, velocidades, saúde do filtro |
| `LOG_CALIBRATION` | 0 | progresso do calibrador (usar com `DISP_CALIBRATE_FLOW`) |
| `LOG_SYS` | 0 | heap livre, descartes de link e de log |

**Campos de diagnóstico que valem conhecer:**

- **`v_lsb_mm_s`** — quanto vale um degrau de fluxo na altura atual. É o piso
  do que o par consegue resolver: nada mais lento que isso é mensurável, por
  mais que se integre. Primeiro número a olhar quando resultados parecem
  ruidosos.
- **`sigma_v_mm_s`** — incerteza que o próprio filtro declara sobre a
  velocidade. Se o filtro estiver mentindo, esse número denuncia.
- **`ratio_x100`** — razão de teste da inovação, ×100. Acima de 100 significa
  que a medida foi rejeitada pelo gate.
- **`resets`** (linha EKF) — quantas vezes o filtro concluiu que sua própria
  predição estava errada e adotou o sensor.

---

## 8. Estado atual: o que funciona e o que falta

### 8.1. Validado

| Item | Evidência |
|---|---|
| Comunicação com BNO08x | Streaming contínuo sem perdas (`reads == packets`), Product ID confirmado |
| Comunicação com MTF-01 | Parser testado com 1 milhão de bytes de ruído sem estouro; zero erros de CRC nos logs |
| Compensação de rotação (eixo X) | Redução de 95% no erro por rotação em ensaio isolado |
| Rotação por yaw para o mundo | Verificada contra log com precisão de 0,03°; teste de dois movimentos perpendiculares deu 85,2° (esperado 90°) |
| Filtro de Kalman | Redução de ruído de velocidade de 28,6× em teste sintético; sem viés sob rejeição |
| Calibração de escala | Dois eixos convergiram independentemente com 0,21% de diferença |
| Fechamento de percurso | 51,2 mm de erro, menor que a incerteza do próprio método de referência (80–100 mm) |
| Aquisição contínua sem perdas | 75 s de captura evento a evento (2026-09-16): 0 registros perdidos, BNO online e distância válida em 100% do tempo |
| Reprodução no PC | 7524 passos reproduzidos pelo replay, 0 divergências, maior diferença 1,0 × 10⁻⁷ |
| Placa parada | 3,8 mm de afastamento máximo em 30 s a ~0,87 m |

**O que ainda não foi validado:** a precisão contra uma distância conhecida.
Os ensaios com régua foram inconsistentes, o fechamento de percurso ficou
dentro da incerteza manual e o ensaio com movimento de 2026-09-16 não teve
referência medida. Detalhes em `evolucao_algoritmo_deslocamento.md`, Seção 6.

### 8.2. Limitações conhecidas

| Limitação | Natureza |
|---|---|
| Deriva de posição cresce sem limite | **Física, não corrigível por software.** Posição não é observável só a partir de velocidade; exige referência absoluta externa |
| Resolução do fluxo cresce com a altura | **Física.** A 0,8 m, um degrau vale 168 mm/s. Confirmado em 9032 leituras: a ~0,9 m, 443 de 601 frames de um movimento à mão leram zero e a volta foi quase toda perdida. Ensaios de precisão devem ser feitos a 15–25 cm; em voo a ~1 m, deslocamento lento não é detectado |
| EKF descarta parte do movimento rápido | **Em aberto (ajuste).** 6 medidas rejeitadas pelo gate, todas durante movimentos; o EKF somou 14% menos que o FUSED na ida |
| EKF confia nas primeiras medidas da sessão | **Em aberto (ajuste).** Fluxo de 1–2 degraus nos primeiros frames deixou ~10 mm de afastamento antes do movimento |
| Regra "fluxo zero = sem translação" | **Em aberto.** Questionada na revisão técnica (translação e rotação podem se cancelar); mantida até ser avaliada com capturas |
| Heap do build Capture | **Margem pequena.** 2688 bytes livres; nova task ou fila nesse build exige rever o heap |
| Movimento de cena engana o sensor | **Física.** Folhas ao vento, prop wash, sombras. O gate do filtro pega picos abruptos, não cena movendo devagar |
| Sinal do eixo Y não confirmado isoladamente | **Em aberto.** Manobra de arfagem contaminada por rolagem; ver `evolucao_algoritmo_deslocamento.md` §5.4 |
| Vetor de rotação do BNO com acurácia 0 | **Em aberto.** Magnetômetro não calibrado; yaw estável na prática mas não confiável em termos absolutos |
| Giroscópio não alinhado no tempo com o fluxo | **Aproximação aceita.** Usa-se a amostra mais recente; o ArduPilot alinha com buffer de atraso. A 100 Hz nos dois sensores o erro é pequeno em bancada |

### 8.3. Não implementado

Fora do escopo deste subsistema, listado para contexto:

- Controle de taxa angular e atitude (LQR/PID)
- Controle de velocidade e de posição
- Mixagem de motores e saída DShot
- Sequenciamento de missão / navegação por waypoints
- Comando para zerar a posição em tempo de execução (hoje a origem é onde a
  placa ligou; `displacement_reset()` existe, mas nada o chama)

### 8.4. Uso no dia a dia

- **Firmware:** grave `build/Release/DroneController.hex`. O build Capture é
  só para ensaios de bancada.
- **Calibração:** já está no código (escala 0,944/0,942, sinais +1/−1, em
  `App/Src/displacement.c`) e vale em todo boot. O procedimento de balançar a
  placa em rolagem e arfagem **não é repetido a cada uso**. Refazer só se o
  MTF-01 for trocado, se a montagem dele mudar (posição, rotação ou ângulo) ou
  se ensaios com distância medida mostrarem erro de escala sistemático.
- **BNO08x:** calibra-se sozinho durante o uso; não há procedimento no
  firmware. A acurácia do vetor de rotação continua em aberto (Seção 8.2).
- **Origem:** a posição começa em zero no boot. Ligar o drone parado no ponto
  de partida; movimento depois de ligado entra na conta.

### 8.5. Encerramento da fase de percepção (2026-09-16)

A fase BNO08x + MTF-01 fica encerrada neste estado: comunicação estável,
três estimadores funcionando, calibração aplicada, captura e replay validados
em hardware. Ficam registrados como pendências, para retomar quando o
controle de voo precisar de posição:

1. Ensaio de precisão com distância medida, a 15–25 cm, ida e volta até
   marcas (o único item que falta para afirmar "X mm medidos = X mm reais").
2. Ajuste do gate do EKF para movimento rápido e da confiança inicial,
   avaliado com replay sobre as capturas.
3. Compensação no eixo Y e acurácia do vetor de rotação do BNO.
4. Regra de fluxo zero e alinhamento temporal giroscópio × fluxo.

Próxima fase: estabilização (controle de taxa e atitude) e acionamento dos
motores.

---

## 9. Reproduzindo os testes

Todos os testes de host rodam sem hardware, no WSL:

```bash
sh tests/run.sh
```

A compilação para ARM usa `tools/build.py` (requer o toolchain do
STM32CubeIDE; `--capture` gera o build de ensaio) ou o próprio STM32CubeIDE.
Gravação por DFU com STM32CubeProgrammer. Replay de uma captura:
`docs/captura_replay_v3.md`.

---

## 10. Documentos relacionados

- `README.md` — arquitetura, ligações, procedimentos de bancada, instruções de
  compilação e gravação.
- `docs/acompanhamento_tecnico.md` — histórico amplo que antes ocupava o
  README, com ensaios, calibrações, diagnósticos e decisões acumuladas.
- `docs/evolucao_algoritmo_deslocamento.md` — evolução histórica do algoritmo,
  com resultados comparativos de cada estágio.
- `docs/captura_replay_v3.md` — formato da captura, procedimento de gravação e
  resultados dos ensaios de 2026-09-16.
- `docs/revisao_tecnica_codex.md` — revisão técnica externa de 2026-09-15 e
  situação de cada achado.
