# DroneController

Firmware de uma controladora de drone baseada na **STM32F411CEU6 Black Pill**.
O projeto já faz a aquisição do BNO08x e do MTF-01, estima atitude, altura e
deslocamento horizontal, produz logs pela USB e mantém uma comunicação
MAVLink própria com uma Raspberry Pi.

O projeto ainda está em desenvolvimento. A parte de percepção foi implementada
e ensaiada em bancada; o acionamento dos ESCs, a mixagem dos quatro motores e
o controle de voo ainda não foram implementados. Portanto, o firmware atual
serve para desenvolvimento e bancada, não para voo.

O histórico técnico detalhado, com resultados, calibrações e problemas já
investigados, foi movido para
[`docs/acompanhamento_tecnico.md`](docs/acompanhamento_tecnico.md). Este README
é a porta de entrada para uma pessoa nova no projeto.

## O que existe hoje

```text
 BNO08x ── I2C/SH-2 ──┐
                      ├── estimadores ── atitude, altura e deslocamento
 MTF-01 ─ UART/DMA ───┘                         │
                                                ├── USB CDC ── PC/logs
 Raspberry Pi ── UART/MAVLink ── DroneLink ─────┘
```

- O **BNO08x** fornece quatérnio, velocidade angular e aceleração linear.
- O **MTF-01** fornece fluxo óptico e distância até o piso.
- Os estimadores combinam essas leituras em atitude, altura e deslocamento.
- O **DroneLink** troca mensagens MAVLink 2 entre o STM32 e a Raspberry Pi.
- A USB da Black Pill aparece como porta COM para logs ou captura de ensaio.
- O FreeRTOS separa aquisição, estimação, telemetria e logs em tasks.

Os comandos de voo já possuem um contrato de software, mas usam uma
implementação provisória que recusa armar e mover com `NOT_IMPLEMENTED`. Isso
impede que a presença do protocolo seja confundida com controle de voo pronto.

## Peças e interfaces

| Peça | Papel | Interface com o STM32 |
|---|---|---|
| STM32F411CEU6 Black Pill | Controladora principal | — |
| GY-BNO08X | IMU: orientação, giroscópio e aceleração linear | I2C2, INT e RST |
| MicoAir MTF-01 | Fluxo óptico e telêmetro | USART2, protocolo Micolink |
| Raspberry Pi | Computador de alto nível e envio de comandos | USART1, MAVLink 2/DroneLink |
| W25Q64 | Identidade persistente de boot do DroneLink | SPI1 |
| Computador de desenvolvimento | Gravação DFU, logs e capturas | USB da Black Pill |
| ESCs e quatro motores | Atuação futura | Ainda não implementada |

Ligações usadas atualmente:

| Sinal | Pino STM32 |
|---|---|
| BNO SCL | PB10 |
| BNO SDA | PB3 |
| BNO INT, ativo baixo | PB4 |
| BNO RST, ativo baixo | PB5 |
| MTF TX → STM32 RX | PA3 |
| MTF RX ← STM32 TX | PA2 |
| Raspberry Pi TX → STM32 RX | PA10 |
| Raspberry Pi RX ← STM32 TX | PA9 |
| USB D− / D+ | PA11 / PA12 |

Todos os módulos devem compartilhar o GND. O MTF-01 é alimentado com 5 V e
usa sinais UART de 3,3 V. No BNO08x, a tensão de alimentação depende da placa
GY-BNO08X utilizada; SDA e SCL devem ter pull-ups para 3,3 V.

## Mapa do repositório

Estrutura resumida que deve aparecer no Git:

```text
DroneController/
├── App/                    código próprio do firmware
│   ├── Inc/                contratos e tipos públicos
│   └── Src/                implementações, DroneLink e driver SH-2
├── Core/                   inicialização gerada pelo CubeMX
├── Drivers/                CMSIS e HAL da ST
├── Middlewares/            FreeRTOS, MAVLink e USB Device
├── USB_DEVICE/             configuração da porta USB CDC
├── protocol/               fonte XML do dialeto MAVLink próprio
├── docs/                   documentação consolidada
│   └── references/         README público; chats completos ficam locais
├── captures/               README público; dados dos ensaios ficam locais
├── tests/                  testes executados no computador
├── tools/                  build, logs, captura, decoder e replay
├── DroneController.ioc     configuração do CubeMX
├── STM32F411CEUX_FLASH.ld  mapa de memória da aplicação em Flash
├── .project/.cproject      projeto do STM32CubeIDE
├── .gitignore              exclusões de build e dados locais
└── README.md               entrada para novos colaboradores
```

`Debug/`, `Release/`, `build/`, `tmp/` e os dados internos de `captures/` não
aparecem no Git: são recriados localmente ou são resultados de ensaio. A
estrutura completa no computador continua sendo explicada nas seções abaixo.

### Código próprio

| Pasta | Conteúdo |
|---|---|
| `App/Inc/` | Interfaces públicas, tipos, configurações e contratos dos módulos próprios. Comece pelos `.h` antes de abrir os `.c`. |
| `App/Src/` | Implementação dos sensores, estimadores, logs e captura. |
| `App/Inc/dronelink/` | Contratos da comunicação com a Raspberry Pi, transações e API que o futuro controle de voo deverá implementar. |
| `App/Src/dronelink/` | Task, transporte UART/DMA, protocolo MAVLink, transações e identidade de boot do DroneLink. |
| `App/Src/sh2/` | Driver SH-2 da CEVA usado pelo BNO08x. É código externo vendorizado; consulte `UPSTREAM.txt` antes de alterá-lo. |

Os principais arquivos de `App/` são:

| Arquivo | Para que serve |
|---|---|
| `sensors.h/.c` | Ponto único que inicia captura, MTF, BNO e estimação. |
| `bno08x.h/.c` | Driver do BNO08x, transporte I2C, reset, reconexão e publicação das amostras. |
| `bno_history.h/.c` | Histórico temporal que integra gyro/aceleração e interpola atitude no intervalo de cada frame MTF. |
| `mtf01.h/.c` | Recepção do MTF-01 pela USART2 com DMA circular. |
| `mtf_parser.h/.c` | Decodificação pura dos frames Micolink. |
| `displacement.h/.c` | Tipos públicos e task que associa as amostras dos sensores. |
| `disp_estimator.c` | Estimadores cru e compensado por giroscópio. |
| `flow_ekf.h/.c` | Filtro de Kalman para posição e velocidade horizontais. |
| `range_filter.h/.c` | Suavização do alcance e rejeição de saltos usados para escalar o fluxo. |
| `flow_calibrator.h/.c` | Calibração da escala do fluxo contra o giroscópio. |
| `attitude.h/.c` | Conversões de quatérnio, Euler, inclinação e referencial. |
| `sensor_log.h/.c` | Seleção dos grupos de log e saída textual pela USB. |
| `capture.h/.c` | Formato binário e gravação evento a evento para replay. |
| `capture_session.h` | Máquina de estados da sessão de captura controlada por DTR. |
| `dma_ring.h` | Regra compartilhada para consumir DMA circular das USARTs. |

Dentro de `dronelink/`:

| Arquivo | Para que serve |
|---|---|
| `dronelink_app` | Cria e executa a task principal do enlace. |
| `dronelink_transport` | USART1, DMA, filas e envio/recepção de bytes MAVLink. |
| `dronelink_protocol` | Validação e tratamento das mensagens recebidas. |
| `dronelink_transaction` | Ordem, repetição e estado dos comandos de trecho. |
| `dronelink_boot_identity` | Número de boot persistido na W25Q64. |
| `dronelink_config` | IDs MAVLink, períodos, timeouts e tamanhos de buffer. |
| `flight_command_api` | Interface que o futuro controle de voo deve fornecer. |
| `flight_command_stub.c` | Implementação provisória e segura: desarmar é aceito; atuação é negada. |

### Código gerado e dependências

| Pasta | Conteúdo e regra de uso |
|---|---|
| `Core/` | Inicialização do STM32 gerada pelo CubeMX: `main`, GPIO, DMA, I2C, SPI, timers, USART, interrupções e FreeRTOS. Alterações próprias devem ficar em blocos `USER CODE`. |
| `Drivers/` | CMSIS e STM32 HAL fornecidos pela ST. Normalmente não são editados. |
| `Middlewares/ST/` | Biblioteca USB Device da ST. |
| `Middlewares/Third_Party/FreeRTOS/` | Kernel e adaptação CMSIS-RTOS v2. |
| `Middlewares/Third_Party/mavlink/` | Headers MAVLink gerados, incluindo o dialeto `drone_controller`. |
| `USB_DEVICE/` | Configuração da classe USB CDC. `usbd_cdc_if.c` liga o firmware à porta COM e detecta DTR para iniciar captura. |

### Protocolo, documentação e dados

| Pasta | Conteúdo |
|---|---|
| `protocol/` | Fonte do dialeto MAVLink próprio. `drone_controller.xml` define comandos, respostas, estados e motivos de falha trocados com a Raspberry Pi. Os headers gerados correspondentes ficam em `Middlewares/Third_Party/mavlink/drone_controller/`. |
| `docs/` | Documentos técnicos: estado atual, evolução dos estimadores, captura/replay, revisão de código e acompanhamento histórico. Esses documentos explicam o “por quê”; o README explica “onde começar”. |
| `docs/references/` | Conversas brutas usadas durante o desenvolvimento. Os `.txt` são locais e ignorados pelo Git; o conteúdo válido deve estar consolidado nos demais documentos. |
| `captures/` | Ensaios reais. O `.bin` contém eventos crus; `.bin.json` guarda metadados e hash; `.bin.firmware.json` copia o manifesto do build; a subpasta com o mesmo nome contém os CSVs decodificados. |

Em cada captura decodificada:

| CSV | Conteúdo |
|---|---|
| `session.csv` | Versão do formato, identificação do firmware e parâmetros da sessão. |
| `bno.csv` | Eventos de quatérnio, giroscópio e aceleração do BNO. |
| `mtf.csv` | Frames de fluxo e distância do MTF-01. |
| `observation.csv` | Amostras do MTF efetivamente observadas pela task de estimação. |
| `step.csv` | Decisões e resultados de cada passo do estimador. |
| `diag.csv` | Diagnóstico de perdas, heap e margem de stack das tasks. |

### Compilação, testes e arquivos temporários

| Pasta | Conteúdo |
|---|---|
| `tools/` | Build independente, leitura de logs, gravação/decodificação de captura e replay. |
| `tests/` | Dez suítes C executadas no computador. `run.sh` também testa replay e detecção de sabotagens. |
| `build/` | Saída de `tools/build.py`, separada em `Debug`, `Release` e `Capture`. É a saída reproduzível usada nas verificações automatizadas. |
| `Debug/` e `Release/` | Saída gerada pelo STM32CubeIDE para a configuração ativa. Contém objetos, mapas, ELF e, após o conversor, HEX. Não edite arquivos aqui. |
| `tmp/` | Scripts e artefatos auxiliares de revisões. Não faz parte do firmware e não deve ser incluído por código de produção. |
| `.settings/` | Preferências compartilháveis do Eclipse/STM32CubeIDE. O cache local `language.settings.xml` é ignorado. |

Arquivos de `tools/`:

| Arquivo | Para que serve |
|---|---|
| `build.py` | Compila todas as fontes ARM e gera ELF, HEX, BIN, mapa e manifesto. |
| `read_logs.ps1` | Lê e opcionalmente salva o log textual da porta COM. |
| `capture_binary.ps1` | Grava bytes crus do firmware Capture e salva metadados e SHA-256. |
| `decode_capture.py` | Valida o arquivo de captura e o converte nos CSVs de eventos. |
| `replay.c` | Reexecuta os estimadores no computador e compara o resultado com os registros `STEP`. |
| `make_synthetic_capture.c` | Produz uma captura controlada para testar o caminho completo. |
| `verify_replay.py` | Compila o replay, cruza C/Python e sabota dados para provar que erros são detectados. |

Arquivos de `tests/`:

| Arquivo | Para que serve |
|---|---|
| `run.sh` | Compila e executa todas as suítes e a verificação do replay. |
| `test_mtf.c` | Parser Micolink, ruído, fragmentação, checksum e sequência. |
| `test_bno_header.c` | Validação dos cabeçalhos SHTP recebidos do BNO. |
| `test_displacement.c` | Atitude, associação de amostras e estimadores cru/fundido. |
| `test_flow_ekf.c` | Predição, correção, validade e gates do EKF. |
| `test_flow_calibrator.c` | Cálculo e rejeições da calibração de escala. |
| `test_capture.c` | Codificação, CRC, corrupção e ressincronização da captura. |
| `test_capture_session.c` | Máquina de estados controlada por DTR. |
| `test_regressions.c` | Casos que reproduzem defeitos encontrados nas revisões. |
| `stubs/` | Definições mínimas de HAL/FreeRTOS para testar no computador. |

Arquivos de `docs/`:

| Arquivo | Para que serve |
|---|---|
| `acompanhamento_tecnico.md` | Antigo README: histórico amplo, ensaios, calibração e decisões acumuladas. |
| `estado_atual_do_sistema.md` | Arquitetura e situação consolidada do sensoriamento e da estimação. |
| `evolucao_algoritmo_deslocamento.md` | Experimentos e mudanças sucessivas no cálculo de deslocamento. |
| `captura_replay_v3.md` | Formato binário, sessão DTR, decoder, replay e limitações. |
| `revisao_tecnica_codex.md` | Achados da revisão de código e correções relacionadas. |
| `pesquisa_fluxo_optico_e_plano.md` | Comparação com ArduPilot, PX4, INAV, Betaflight e Crazyflie; diagnóstico do erro de fechamento e plano de evolução. |

Arquivos locais de `docs/references/`:

| Arquivo | Para que serve |
|---|---|

Os arquivos produzidos por um build têm funções diferentes:

- `.hex`: imagem com endereços, indicada para gravar no STM32CubeProgrammer.
- `.bin`: bytes crus; ao gravar manualmente, usa endereço inicial `0x08000000`.
- `.elf`: programa com símbolos, usado pelo depurador.
- `.map`: mapa de memória e símbolos, útil para analisar Flash e RAM.
- `manifest.json`: variante, compilador e hashes do build feito por `build.py`.
- `.o`, `.d`, `.su` e `.cyclo`: intermediários e relatórios do compilador.

### Arquivos na raiz

| Arquivo | Para que serve |
|---|---|
| `DroneController.ioc` | Fonte da configuração do STM32CubeMX: pinos, clocks e periféricos. |
| `.project`, `.cproject`, `.mxproject` | Metadados do STM32CubeIDE/CubeMX. |
| `DroneController Debug.launch` | Configuração de depuração da IDE. |
| `STM32F411CEUX_FLASH.ld` | Script de ligação para executar a aplicação a partir da Flash. |
| `STM32F411CEUX_RAM.ld` | Script alternativo de ligação para execução pela RAM. |

## Getting started

### 1. Abrir e entender o projeto

Instale o STM32CubeIDE e o STM32CubeProgrammer. No CubeIDE, importe a pasta
raiz como projeto existente. O arquivo `.project` já descreve o projeto; não é
necessário criar outro projeto ou regenerar o `.ioc` para começar.

Antes do primeiro build:

1. Selecione a configuração **Debug** na barra do CubeIDE.
2. Pressione **F5** para atualizar os arquivos do projeto.
3. Se a IDE estiver usando objetos antigos, execute **Project > Clean**.
4. Pressione **Ctrl+B** para compilar.
5. Confirme no Console que o build terminou sem erros.

O conversor HEX está habilitado no projeto. No perfil Debug, o arquivo para
gravação deve aparecer em:

```text
Debug/DroneController.hex
```

No perfil Release, ele aparece em `Release/DroneController.hex`. As pastas
`Debug/` e `Release/` pertencem ao CubeIDE; a pasta `build/` pertence ao script
de build explicado mais abaixo.

### 2. Gravar o HEX em modo DFU

1. Feche qualquer terminal que esteja usando a porta COM da placa.
2. Coloque a Black Pill em modo DFU usando BOOT0 e RESET.
3. Conecte o cabo USB e abra o STM32CubeProgrammer.
4. Selecione a conexão **USB** e conecte ao dispositivo DFU.
5. Abra `Debug/DroneController.hex`.
6. Inicie **Download/Program** e aguarde a verificação terminar.
7. Retorne BOOT0 para 0 e reinicie a placa.

O arquivo HEX já carrega os endereços de gravação. O endereço `0x08000000` só
precisa ser informado quando se grava o arquivo BIN.

Depois do reset normal, o firmware deixa de aparecer como DFU e passa a
enumerar uma porta COM USB. O CubeProgrammer grava a placa, mas não é o monitor
dos logs da aplicação.

### 3. Ver os logs

Abra um PowerShell na raiz e execute:

```powershell
.\tools\read_logs.ps1
.\tools\read_logs.ps1 -Port COM3 -OutputPath logs\bancada.txt
```

O primeiro comando ajuda a listar as portas. Troque `COM3` pela porta da sua
placa. Também é possível usar Tera Term ou outro terminal serial em 115200,
8N1 e sem controle de fluxo. No USB CDC, o baud rate é virtual, mas manter
115200 evita confusão com as UARTs físicas.

Os grupos exibidos são escolhidos em `App/Inc/sensor_log.h`. Alterar um grupo
exige recompilar e gravar novamente.

O perfil normal atual mostra o deslocamento horizontal do EKF, em milímetros,
a cada 500 ms. Ele começa do zero em todo boot e mantém o diagnóstico na mesma
linha:

```text
DESLOCAMENTO_MM 0,0 height_mm=301 valid=0 health=2 corr_age=25 imu_align_rej=0 range_rej=0 range_reanchor=0 ekf_rej=0
DESLOCAMENTO_MM -24,1 height_mm=298 valid=1 health=0 corr_age=4 imu_align_rej=0 range_rej=0 range_reanchor=0 ekf_rej=0
DESLOCAMENTO_MM -83,0 height_mm=304 valid=1 health=0 corr_age=3 imu_align_rej=0 range_rej=0 range_reanchor=0 ekf_rej=1
```

O primeiro valor é X e o segundo é Y. `height_mm` é a altura atual filtrada e
corrigida pela inclinação; ela não é acumulada. `valid=1 health=0` indica
estimativa utilizável. Os contadores revelam perda de alinhamento temporal,
rejeições do telêmetro, mudanças persistentes de altura e rejeições do Kalman.
O sinal depende do sentido dos eixos na montagem do MTF-01.

### 4. Build independente da IDE

O script abaixo usa o compilador instalado junto do CubeIDE, recompila todas
as fontes e cria os resultados em `build/`:

```powershell
python .\tools\build.py             # build/Release: firmware normal
python .\tools\build.py --debug     # build/Debug
python .\tools\build.py --capture   # build/Capture: captura binária de bancada
```

Para uso normal, grave `build/Release/DroneController.hex`. O perfil Capture é
exclusivo para ensaios: desliga os logs textuais e envia registros binários,
que parecem caracteres ilegíveis em um terminal.

### 5. Executar os testes no computador

Os testes usam GCC, AddressSanitizer e UndefinedBehaviorSanitizer no WSL:

```bash
sh tests/run.sh
```

Eles verificam parsers, estimadores, regressões, formato de captura, sessão e
replay. Esses testes não substituem ensaios com o hardware.

## Por onde começar a ler o código

Para uma primeira leitura, não tente entender todas as pastas em sequência.
Siga o caminho que os dados percorrem:

1. **Configuração física:** abra `DroneController.ioc` no modo gráfico e veja
   os pinos e periféricos. Depois consulte `Core/Inc/main.h` para os nomes dos
   pinos e `Core/Src/freertos.c` para saber quais módulos são iniciados.
2. **Inicialização própria:** leia `App/Inc/sensors.h` e
   `App/Src/sensors.c`. São pequenos e mostram a ordem captura → MTF → BNO →
   estimação.
3. **Um sensor de cada vez:** leia primeiro `mtf01.h` e `mtf_parser.h`; depois
   seus `.c`. Em seguida faça o mesmo com `bno08x.h/.c`. Os headers descrevem
   o contrato e os `.c` mostram HAL, DMA, SH-2 e FreeRTOS.
4. **Cálculo:** leia `displacement.h`, depois `disp_estimator.c`, `attitude.c`
   e `flow_ekf.c`. Por último, leia a task em `displacement.c`, que conecta as
   peças.
5. **Saída para o computador:** leia `sensor_log.h/.c`. Para ensaios
   reproduzíveis, continue em `capture.h/.c` e
   [`docs/captura_replay_v3.md`](docs/captura_replay_v3.md).
6. **Comunicação com a Pi:** comece por `protocol/drone_controller.xml`, depois
   `dronelink_app.h`, `dronelink_protocol.h` e `flight_command_api.h`. Só então
   abra as implementações em `App/Src/dronelink/`.
7. **Histórico e decisões:** consulte
   [`docs/estado_atual_do_sistema.md`](docs/estado_atual_do_sistema.md) e
   [`docs/evolucao_algoritmo_deslocamento.md`](docs/evolucao_algoritmo_deslocamento.md).

Uma regra útil: arquivos `.h` explicam **o que** um módulo oferece; arquivos
`.c` explicam **como** ele faz. `Core/` conecta o código ao microcontrolador,
enquanto `App/` contém o comportamento específico deste drone.

## Cuidados ao modificar

- Preserve alterações próprias dentro dos blocos `USER CODE` dos arquivos
  gerados pelo CubeMX.
- Não edite `Debug/`, `Release/` ou `build/`; essas pastas são recriadas.
- Trate `Drivers/`, `Middlewares/` e `App/Src/sh2/` como dependências externas.
- Se alterar `protocol/drone_controller.xml`, regenere os headers MAVLink para
  STM32 e Raspberry Pi a partir da mesma versão do dialeto.
- Não misture logs de texto com captura binária no mesmo firmware.
- Teste motores sempre sem hélices durante a futura implementação de ESCs.

## Documentos para aprofundar

- [Acompanhamento técnico e histórico](docs/acompanhamento_tecnico.md)
- [Estado atual do sensoriamento e estimação](docs/estado_atual_do_sistema.md)
- [Evolução do algoritmo de deslocamento](docs/evolucao_algoritmo_deslocamento.md)
- [Formato de captura e replay](docs/captura_replay_v3.md)
- [Revisão técnica do código](docs/revisao_tecnica_codex.md)
