# Acompanhamento técnico do DroneController

Este documento reúne o estado detalhado do desenvolvimento, resultados de
ensaios, decisões de implementação, calibrações, limitações e hipóteses já
investigadas. Para a apresentação do projeto, mapa do repositório e primeiros
passos, consulte o [`README.md`](../README.md) na raiz.

## Estado atual (2026-09-16) e uso

A fase de **percepção (BNO08x + MTF-01) está encerrada**: os dois sensores
comunicam de forma estável, três estimadores de deslocamento rodam a 100 Hz,
a calibração está aplicada, e a captura com replay foi validada em hardware.
Ainda **não existe controle de voo**: estabilização, motores e controle de
posição são a próxima fase.

**Qual firmware gravar:**

| Arquivo | Uso |
|---|---|
| `build/Release/DroneController.hex` | **Uso normal e base do firmware de voo.** Sensores, estimadores, DroneLink e logs de texto pela USB |
| `build/Capture/DroneController.hex` | **Só ensaio de bancada.** Nele o estimador só roda enquanto o PC grava; nunca usar no drone |

**Calibração:** já está no código (`App/Src/displacement.c`: escala 0,944 e
0,942, sinal do giroscópio +1 e −1) e vale em todo boot. O balanço de rolagem
e arfagem **não é feito a cada uso**. Refazer só se trocar o MTF-01, mudar a
montagem dele, ou se ensaios com distância medida mostrarem erro de escala
sistemático. O BNO08x se calibra sozinho durante o uso.

**Origem da posição:** zero no momento em que a placa liga. Ligue o drone
parado no ponto de partida; nada no firmware zera a posição depois.

**Resultados medidos:**

- Placa parada: 3,8 mm de afastamento máximo em 30 s a ~0,87 m.
- Captura de 75 s sem nenhum registro perdido; o replay reproduziu os 7524
  passos da placa com diferença máxima de 1,0 × 10⁻⁷.
- Inclinar sem sair do lugar: erro reduzido 77% (eixo X 95%; eixo Y em aberto).
- Três movimentos de ~200 mm voltando ao início: 51 mm de fechamento, dentro
  da incerteza manual.
- **Precisão contra distância conhecida ainda não comprovada.** A ~0,9 m, um
  degrau de fluxo vale ~180 mm/s e movimentos lentos quase não aparecem. O
  ensaio de precisão deve ser feito a 15–25 cm, com marcas medidas.

Detalhes e pendências: `docs/estado_atual_do_sistema.md` (Seção 8) e
`docs/evolucao_algoritmo_deslocamento.md` (Seção 6).

## Captura binária para replay

Ferramenta de desenvolvimento: grava tudo o que os sensores entregaram e o que
o estimador decidiu, para reexecutar o mesmo cálculo no PC. Validada em
bancada em 2026-09-16 (`captures/ensaio01.bin` e `ensaio02.bin`, 0 perdas,
replay idêntico).

Use `python tools/build.py --capture` para gerar
`build/Capture/DroneController.hex`, sem alterar o perfil normal. Após gravar
esse firmware e reiniciar a placa, feche terminais que ocupem a COM e execute:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\capture_binary.ps1 -Port COM3 -Seconds 45 -OutputPath captures\ensaio03.bin
python tools/decode_capture.py captures\ensaio03.bin --out-dir captures\ensaio03
```

A gravação começa assim que o script abre a porta: combine o roteiro do
movimento antes. Use um nome de arquivo novo a cada ensaio.

A sessão só começa quando o gravador abre a porta e ativa DTR. Há uma sessão
por boot: reinicie a placa antes de repetir. O script prepara o arquivo antes
do DTR, lê bytes crus e drena a fila ao final. **Não use `read_logs.ps1` para
captura binária**: ele é exclusivo dos logs textuais do firmware normal.

O modo Capture espera amostras novas após DTR e começa o estimador do zero;
movimento anterior à sessão não entra no replay. Se houver perda, o replay
mostra o registro da interrupção e o prefixo conferido, retornando erro.
Consulte [formato, testes e limitações](captura_replay_v3.md).

## Arquitetura dos sensores

Cada sensor e um modulo independente: um par `.h`/`.c` dono do seu periferico,
da sua task e do seu tipo de amostra. Nenhum estado global e compartilhado
entre eles.

| Arquivo | Responsabilidade |
|---|---|
| `App/Inc/bno08x.h` + `App/Src/bno08x.c` | IMU BNO08x: task, I2C2, pinos RST/INT e a camada `sh2_Hal_t` sobre o driver CEVA |
| `App/Inc/mtf01.h` + `App/Src/mtf01.c` | MTF-01: task, USART2 com DMA circular e stream buffer |
| `App/Inc/mtf_parser.h` + `App/Src/mtf_parser.c` | Decodificador Micolink puro, sem HAL |
| `App/Inc/displacement.h` + `App/Src/displacement.c` | Task de deslocamento: le os dois sensores e alimenta os estimadores |
| `App/Src/disp_estimator.c` | Estimadores cru e fundido, puros, sem HAL |
| `App/Inc/flow_ekf.h` + `App/Src/flow_ekf.c` | Filtro de Kalman de posicao/velocidade horizontal; puro |
| `App/Inc/flow_calibrator.h` + `App/Src/flow_calibrator.c` | Calibracao de escala do fluxo contra o giroscopio, sem regua; puro |
| `App/Inc/attitude.h` + `App/Src/attitude.c` | Quaternio -> Euler, cos(inclinacao), rotacao por yaw; puro |
| `App/Inc/bno_i2c_header.h` | Regra de continuacao de header SHTP, header-only |
| `App/Inc/sensors.h` + `App/Src/sensors.c` | Agregador fino: so `sensors_init()`, que sobe a captura (no build Capture) e as tasks dos sensores |
| `App/Inc/sensor_log.h` + `App/Src/sensor_log.c` | Log USB CDC a cada 500 ms |
| `App/Inc/capture.h` + `App/Src/capture.c` + `App/Inc/capture_session.h` | Captura binaria para replay; so ativa no build Capture |
| `App/Src/sh2/` | Driver CEVA SH-2 vendorizado, **sem modificacoes** |

As pecas com logica nao-trivial e sem dependencia de hardware ficam isoladas
de proposito, e tem teste de host. No WSL, um comando roda as oito suites (com
ASan/UBSan) e a verificacao do replay:

```bash
sh tests/run.sh
```
 
## Deslocamento: como o nome disso e por que o MTF-01 sozinho erra

O sensor optico **nao mede translacao**. Ele mede a taxa angular com que a cena
varre a imagem, e essa taxa e a soma de duas coisas: o drone transladando sobre
o chao e o drone **girando parado**. Uma camera apontada pra baixo que inclina
ve o chao se mover exatamente como uma que deslizou de lado. Integrar a leitura
crua acumula cada rolagem e arfagem como se fosse viagem - e essa e a razao
usual de 300 mm virarem 350 mm ou 250 mm.

Os nomes corretos:

| O que | Nome tecnico |
|---|---|
| A grandeza instantanea | **velocidade linear no referencial do corpo** (body-frame linear velocity) |
| Subtrair o giro do fluxo | **compensacao de taxa do corpo** (*body-rate compensation* / *gyro compensation*) |
| Acumular velocidade em posicao | **dead reckoning** (navegacao estimada) / **odometria** |
| A tecnica completa fluxo+IMU+range | **odometria inercial-visual** (*visual-inertial odometry*, VIO), ou *optical flow-aided inertial navigation* |
| Como o ArduPilot faz | fusao num **EKF** (Extended Kalman Filter), em `libraries/AP_NavEKF3/AP_NavEKF3_OptFlowFusion.cpp` |

A equacao central, de `AP_NavEKF3_Measurements.cpp:227` e
`AP_NavEKF3_OptFlowFusion.cpp:190`:

```
fluxo_compensado = fluxo_medido - taxa_do_giroscopio     <- so sobra translacao
velocidade       = fluxo_compensado * alcance            <- rad/s * m = m/s
deslocamento    += R(yaw) * velocidade * dt              <- gira pro mundo, depois integra
```

O ArduPilot ainda corrige escala sistematica com os parametros
`FLOW_FXSCALER`/`FLOW_FYSCALER` (`AP_OpticalFlow.cpp:205`) e tem um calibrador
automatico - e o mesmo papel do `displacement_set_flow_scale()` aqui.

### Os tres estimadores

`App/Inc/displacement.h` roda os tres ao mesmo tempo, pra comparar com trena na
bancada usando o mesmo movimento fisico:

| | `RAW` | `FUSED` | `EKF` |
|---|---|---|---|
| Compensacao de rotacao | nao | sim | sim |
| Referencial | corpo | mundo | mundo |
| Rejeicao por qualidade/alcance | nao | sim | sim |
| Predicao pela IMU | nao | nao | **sim** |
| Rejeicao de outlier | nao | nao | **sim, com gate de inovacao** |
| Saida confiavel | - | - | **velocidade** |

O teste `tests/test_displacement.c` prova a diferenca: girando 0,4 rad/s parado
por 1 s, o cru acumula **400 mm** de viagem inexistente e o fundido acumula
**0 mm**.

### Procedimento de bancada

1. **Pareamento e sinal do giroscopio primeiro - e o sinal e independente por
   eixo.** O modelo de livro-texto diz que uma camera de eixos alinhados com a
   IMU tem pareamento cruzado (arfagem varre a imagem no eixo X, rolagem no
   eixo Y) - **nao e isso que essa placa mede**. Testes de bancada com um eixo
   isolado de cada vez deram, com centenas de amostras cada:

   | Manobra | Giro dominante | Resposta | Relacao |
   |---|---|---|---|
   | Rolagem pura | `gyro_x` | inteira em `flow_x` | **sinal oposto**, 214/218 (98%) |
   | Arfagem pura | `gyro_y` | inteira em `flow_y` | **mesmo sinal**, 142/143 (99%) |

   Ou seja: o pareamento e **direto** (X com X, Y com Y), nao cruzado - o BNO e
   o MTF-01 parecem montados com os eixos girados 90 graus um em relacao ao
   outro nesta placa. Mas o sinal necessario para cancelar cada eixo e
   **diferente entre eles** (X precisa de +1, Y precisa de -1) - uma rotacao
   relativa de 90 graus entre dois sensores inverte exatamente um dos dois
   eixos, nao os dois nem nenhum, entao isso e consistente e nao e coincidencia.
   Por isso `gyro_sign` deixou de ser um unico valor e virou `gyro_sign_x`/
   `gyro_sign_y` independentes em `disp_config_t` - um valor compartilhado
   jamais acertaria os dois eixos ao mesmo tempo nesta placa. Ja corrigido em
   `disp_estimator.c`, com os defaults medidos (`+1.0`/`-1.0`) em
   `App/Src/displacement.c`.

   **Como verificar/refazer:** segure a placa num ponto fixo (sem transladar) e
   incline **um eixo de cada vez**, devagar, segurando ~2 s em cada extremo,
   6-8 vezes por eixo, com pausa entre eixos - misturar arfagem e rolagem no
   mesmo movimento contaminou as primeiras tentativas e tornou os dados
   inuteis. Ligue `LOG_BNO_DATA` e `LOG_MTF` para ver `gyro_mrad_s` e
   `flow_cm_s_at_1m` lado a lado e comparar o sinal diretamente - isso provou
   ser mais confiavel que a media ponderada do calibrador quando os eixos nao
   estao bem isolados (ver secao do calibrador abaixo). Uma vez pareado e
   assinado corretamente, `fused_mm` deve ficar perto de zero durante a
   rotacao naquele eixo; se crescer (aproximadamente dobrando o cru daquele
   eixo), o sinal daquele eixo especificamente ainda esta invertido - use
   `displacement_set_gyro_sign(sign_x, sign_y)`.
2. **Escala com o calibrador do giroscopio, nao com regua** - ver secao
   dedicada abaixo. Empurrar uma caixa uma distancia "medida a olho" carrega
   tanta incerteza quanto o erro que se quer medir: dois ensaios de ~200 mm
   deram razoes de 0,79 e 0,55 que nao bateram entre si, contra 0,944/0,942
   medidos pelo giroscopio. **Atencao ao interpretar isso:** os dois eixos
   concordarem entre si (0,2%) nao prova que a regua estava errada - eles
   compartilham o mesmo procedimento e poderiam compartilhar o mesmo erro
   (atraso de sincronismo, geometria do centro de giro). A divergencia entre
   0,79-0,55 e 0,944 continua **sem explicacao fechada** e so uma medicao com
   referencia fisica controlada (guia com batentes, ambos os sentidos, alturas
   e yaw conhecidos) resolve qual dos dois esta certo.
3. Compare `raw_mm` e `fused_mm` na mesma passada. Sem rotacao os dois devem
   concordar; com rotacao so o fundido deve se manter honesto.

Referencia medida em bancada, placa imovel sobre superficie estavel, ~4000
quadros (40 s) a 0,8 m: **amplitude pico a pico de 4 mm no cru e 6 mm no
fundido** (ruido de base do sistema, nao deriva acumulada). Se um
ensaio parado der muito mais que isso, o problema e o apoio ou a textura do
chao, nao o estimador - vale corrigir antes de medir deslocamento.

A rotacao por yaw foi verificada contra o log: com yaw estavel em -141,23 graus,
seis quadros independentes mostraram o vetor fundido sendo exatamente o cru
girado de -141,21 a -141,24 graus, com a magnitude preservada em 1 parte em
1600. A matematica do referencial esta correta.

### Calibrar a escala do fluxo contra o giroscopio, sem regua

`App/Inc/flow_calibrator.h` + `App/Src/flow_calibrator.c` implementam a mesma
tecnica do `AP_OpticalFlow_Calibrator.cpp` do ArduPilot: durante uma rotacao
pura (a placa gira no lugar, sem transladar), um sensor de fluxo perfeitamente
calibrado leria exatamente a taxa que o giroscopio mede - e o giroscopio, ao
contrario de um humano empurrando uma caixa, e uma referencia precisa e
independente. A diferenca media entre o que o MTF-01 realmente reporta e o
que o giroscopio diz que deveria ser **e** o erro de escala, sem depender de
quanto alguem realmente moveu alguma coisa.

**Por que isso e melhor que medir com trena:** dois ensaios de deslocamento
linear (empurrar ~200 mm e comparar com a caixa) deram razoes de 0,79 e 0,55
que nao concordaram entre si - a incerteza de "quanto eu realmente movi de
olho" era do tamanho do proprio erro sendo medido. Girar no lugar elimina essa
incerteza: o giroscopio do BNO e muito mais preciso que qualquer medida manual
de distancia.

**Como rodar:**

1. Ligue `LOG_CALIBRATION` em `App/Inc/sensor_log.h` (`1`) e `DISP_CALIBRATE_FLOW`
   no mesmo arquivo (`1`). Recompile e grave.
2. **Isole um eixo por vez** - so rolagem primeiro, depois so arfagem, com
   pausa entre os dois. Segure a placa **parada no lugar** (sem transladar) e
   incline devagar, ~2 s em cada extremo, 6-8 vezes por eixo. Balancar os dois
   eixos junto (tipo um "oito" rapido) foi tentado primeiro e deu `valid=0,0`
   persistente mesmo com milhares de amostras - ver a limitacao abaixo.
3. Acompanhe a linha `CAL` no log. Resultado real, medido nesta placa depois
   de isolar sinal e pareamento por eixo (ver secao acima):
   ```
   CAL running=1 samples=3239,2215 valid=1,1 scalar_x1000=944,942 residual_mrad_s=473,256
   ```
   `scalar_x1000=944,942` = fatores de correcao de 0,944 (X) e 0,942 (Y) -
   proximos o bastante um do outro pra dar confianca (0,2% de diferenca), e
   **surpreendentemente proximos de 1,0**. Isso contraria o 0,79 medido antes
   por deslocamento linear (que sugeria uma correcao de ~1,27) - e o motivo e
   exatamente o que motivou construir esse calibrador: aquela medida por
   regua carregava tanto erro quanto o que tentava medir. O giroscopio, sendo
   uma referencia independente e precisa, e o numero em que confiar aqui.
4. Quando os dois `valid` estiverem em 1 e o `scalar_x1000` tiver parado de
   mudar por alguns segundos, anote os dois numeros. Troque os defaults em
   `App/Src/displacement.c` (`cfg.flow_scale_x`/`flow_scale_y`) por esses
   valores, desligue `DISP_CALIBRATE_FLOW` e `LOG_CALIBRATION` de novo, e
   recompile - a calibracao nao persiste sozinha entre reinicializacoes.
   (`0,944`/`0,942` ja estao aplicados como default atual.)

`residual_mrad_s` e quanto o sensor discordava do giroscopio **antes** da
correcao (em mrad/s); nao e erro, e a medida de quao grosseira foi a
diferenca que a calibracao esta corrigindo - quanto maior, mais a escala
estava errada.

**Limitacao real, descoberta na pratica: a media ponderada nao e robusta a
manobra mal isolada.** Nas primeiras tentativas, balancar arfagem e rolagem
juntas (tipo um "oito" rapido) produziu `valid=0,0` persistente mesmo com
milhares de amostras - o algoritmo (que pesa cada amostra pelo proprio
residuo, dando MAIS peso as amostras mais discordantes) e sensivel a
contaminacao por translacao residual e a mistura de eixos, podendo convergir
pra um numero sem sentido (incluindo negativo) mesmo com dado abundante.
`raw_scalar_x1000` no log mostra esse valor mesmo fora da faixa aceita
(0,2-4,0), especificamente para diagnosticar isso: negativo persistente e
sinal de pareamento/sinal errado, nao so de ruido. Quando isso acontecer,
**pare de confiar na media agregada** e va direto aos dados brutos: ligue
`LOG_BNO_DATA` + `LOG_MTF` e compare `gyro_mrad_s` contra `flow_cm_s_at_1m`
amostra a amostra, num movimento de UM EIXO ISOLADO por vez. Foi assim que se
descobriu que o pareamento desta placa e direto, nao cruzado (ver
"Pareamento e sinal do giroscopio" acima) - a inspecao direta deu 10 de 10
amostras consistentes onde a media ponderada so dava confusao.

**Um bug que apareceu ao implementar isso:** a versao anterior do estimador
`fused` aplicava a escala *depois* de misturar com o giroscopio, o que
escalaria incorretamente o proprio termo de compensacao de rotacao - o
giroscopio nao deveria ser corrigido, so o fluxo. Com escala 1,0 (o unico
valor usado ate aqui) isso nao tinha efeito nenhum, mas teria comprometido a
compensacao assim que uma escala real fosse aplicada. Corrigido junto com a
calibracao por eixo (`App/Src/disp_estimator.c`).

### A resolucao do MTF-01 e o piso do que da pra medir

Medido em bancada: **34 de 34 amostras de fluxo cairam em multiplos exatos de
20** na unidade "cm/s a 1 m" do sensor. Ou seja, o MTF-01 so reporta fluxo em
degraus de **0,2 rad/s**. Isso nao e ruido, e a resolucao do sensor, e o que
uma contagem vale em velocidade cresce com a altura:

| Altura | 1 contagem de fluxo vale |
|---|---|
| 0,84 m | **168 mm/s** |
| 0,50 m | 100 mm/s |
| 0,25 m | 50 mm/s |
| 0,15 m | 30 mm/s |
| 0,10 m | 20 mm/s |

A 0,84 m o sensor so consegue dizer "parado" ou "168 mm/s". Movimento mais
lento que isso fica praticamente invisivel. Por isso o ensaio de bancada deve
ser feito **baixo**: a 0,15 m a resolucao fica 5,6x melhor. O log mostra esse
piso em `v_lsb_mm_s`.

Confirmado na captura de 2026-09-16: as 9032 leituras de fluxo cairam em
degraus de 20. A ~0,9 m, durante um movimento feito a mao, **443 de 601
frames leram zero**, e a volta foi somada em so 52-61 mm pelos tres
estimadores, inclusive o cru. O limite estava no que o sensor reportou, nao no
algoritmo. Em voo a ~1 m, o mesmo vale: deslocamento lento nao e detectado.

Duas consequencias praticas:

1. **Deriva parado.** Cada quadro espurio de 1 contagem soma ~1,7 mm a 0,84 m.
   Integrado como passeio aleatorio, isso da `1,7 mm * raiz(N)` - centenas de
   quadros ja produzem dezenas de milimetros de deriva sem a placa sair do
   lugar. Nao ha filtro que recupere informacao que o sensor nao mediu.
2. **A compensacao podia inventar movimento.** O giroscopio resolve taxas muito
   mais finas que o fluxo. Com a placa girando a 0,15 rad/s, o fluxo reporta 0
   (abaixo do degrau) e o giro reporta 0,15 - a subtracao lia essa diferenca
   como translacao e criava viagem do nada. O log de bancada pegou quadros com
   `raw_d_um=0,0` e `fused_d_um=2631,-1412`: ~3 mm inventados por quadro.

   A correcao e **confiar no zero do sensor**: se o fluxo bruto de um eixo foi
   quantizado a zero, aquele eixo nao viu movimento, e o giroscopio nao tem
   direito de opinar sobre isso. Vale sempre, sem parametro.

   Uma primeira tentativa colocou um *deadband* de uma contagem na taxa ja
   compensada, e **piorou**: uma contagem e o valor mais comum na leitura, entao
   valores minusculos do giroscopio decidiam ao acaso quais amostras sobreviviam,
   e o ruido simetrico de +-1 contagem - que se cancela sozinho ao integrar - era
   justamente o que ia embora. Medido: 49 mm de deriva parado contra 47 mm sem
   filtro nenhum. Esse deadband continua existindo como `deadband_rad_s`, mas
   **desligado por padrao**, so como botao de experimento.

### Movimento de cena: quando o chao se mexe e o drone nao

O sensor optico mede movimento **relativo** entre ele e a cena. Ele nao tem
como distinguir "eu me movi sobre um chao parado" de "o chao se moveu debaixo
de mim parado". As duas situacoes produzem exatamente a mesma imagem.

Na pratica isso acontece com folhas e poeira levantadas pelo vento **ou pelo
proprio prop wash** do drone, grama balancando, agua, sombras em movimento, e
pessoas ou veiculos passando por baixo. Passar a mao sob o sensor e a mesma
falha em miniatura - e produziu 2496 mm/s num unico quadro de 10 ms, ~250 m/s2,
com a placa imovel.

Quem arbitra e a IMU, e o lugar certo pra isso e o filtro. O ArduPilot usa um
**gate de consistencia de inovacao** (`AP_NavEKF3_OptFlowFusion.cpp:684`): a
medida so e fundida se a diferenca entre ela e o que o estado inercial previa
couber no gate (`FLOW_I_GATE`, padrao 300 = 3 sigma). O `flow_ekf.c` faz o
mesmo, com os mesmos valores padrao.

**Por que nao um gate no integrador.** A primeira tentativa colocou esse gate
direto no estimador `fused`. Piorou: descartar uma amostra de uma soma corrente
afirma "a velocidade era zero naquele intervalo", e quando as rejeicoes nao sao
simetricas essa afirmacao enviesa o total. Medido em bancada com a placa parada
por 2 minutos: **21 mm de deriva no fundido com gate contra 2 mm no cru sem gate
nenhum**. Num filtro nao existe esse buraco - a medida rejeitada apenas deixa a
predicao de pe, e a predicao vem da IMU.

**O que o gate ainda nao resolve:** cena movendo devagar e de forma sustentada.
Folhas arrastando ou agua correndo produzem uma velocidade plausivel, passam
pelo gate, e o drone acredita. Nao ha defesa boa sem uma referencia absoluta -
voar sobre agua corrente ou vegetacao ao vento e caso conhecidamente ruim para
fluxo optico em qualquer piloto automatico.

### O filtro: o que ele entrega e o que nao entrega

`App/Inc/flow_ekf.h` sao dois filtros de Kalman de 2 estados (posicao,
velocidade), um por eixo horizontal - os eixos nao se acoplam nem na dinamica
nem na medida, entao um 4x4 seria a mesma matematica com escrituracao extra.
Predicao pela aceleracao linear da IMU rotacionada pro mundo, correcao pela
velocidade do fluxo, com gate de inovacao.

**Entrega: velocidade.** Alimentado com o mesmo ruido de quantizacao de
+-160 mm/s, o filtro estima velocidade com desvio de 5,6 mm/s - **reducao de
28x** - e sem vies quando medidas sao descartadas.

**Nao entrega: posicao sem deriva. E nao tem como.** Posicao nao e observavel
so a partir de medidas de velocidade, entao o erro dela e a integral do erro de
velocidade e cresce sem limite. No mesmo fluxo sintetico o filtro derivou 65 mm
em 60 s contra 59 mm do integrador simples - **nao e melhor**. Velocidade e a
saida honesta aqui, e velocidade e o que um controlador precisa; posicao
confiavel exige referencia absoluta (GPS, marcadores, ancoras).

**Recuperacao.** Um gate que rejeita tudo e pior que gate nenhum: o filtro
defenderia um estado que o sensor vem contradizendo. Depois de 0,5 s de
rejeicoes seguidas (`MAX_REJECT_RUN`) o filtro adota a medida e alarga a
incerteza. O log mostra isso em `resets=`.

### Limitacoes conhecidas

- O giroscopio usado e a amostra **mais recente** do BNO, nao a do instante
  exato do quadro de fluxo. O ArduPilot alinha no tempo com um buffer de atraso
  (`_flowDelay_ms`). A 100 Hz nos dois sensores o erro e pequeno na bancada, mas
  e real em voo.
- O EKF (`App/Src/flow_ekf.c`) melhora a estimativa de **velocidade**, nao a
  deriva de posicao: posicao nao e observavel so a partir de medidas de
  velocidade, entao o erro de posicao **acumula sem limite** com filtro ou sem.
  Nao ha estimativa de terreno nem referencia absoluta. Serve pra medir
  deslocamento em trechos curtos, nao pra manter posicao indefinidamente.
- `attitude_to_euler()` satura em +-90 graus de arfagem (gimbal lock). Para
  controle, prefira o quaternio, que tambem esta exposto.
- Com os parametros atuais, o gate do EKF rejeita parte do movimento rapido
  feito a mao: 6 rejeicoes, todas durante movimentos, e 14% a menos que o
  fundido na ida do ensaio de 2026-09-16. Ajuste pendente, a avaliar com
  replay.
- O EKF recem-iniciado aceita as primeiras medidas da sessao com pouca
  cautela: 1-2 degraus de fluxo nos primeiros frames deixaram ~10 mm de
  afastamento antes do movimento.
- A regra que zera o eixo quando o fluxo bruto e zero (secao acima) foi
  questionada na revisao tecnica, porque translacao e rotacao podem se
  cancelar. Continua ativa ate ser avaliada com capturas.
- A precisao contra uma distancia conhecida ainda nao foi medida com
  referencia confiavel.

## Historico: por que o BNO08x nao subia

O BNO08x demora **~80 ms** depois de soltar o RST ate responder no I2C. O
`hub_open()` sondava os enderecos **uma unica vez, 20 ms** apos soltar o RST, e
desistia. Como 20 ms < 80 ms, a falha era **deterministica** - dezenas de
tentativas seguidas, todas falhando no primeiro byte.

A correcao troca a sondagem unica por polling ate 400 ms, retornando no
instante em que o chip responde. O tempo medido aparece no log como
`probe_ms`, que nesta placa le 80.

Hipoteses investigadas e **descartadas** por evidencia, para nao voltarem:

| Hipotese | Por que foi descartada |
|---|---|
| Timing ou prioridade de task do RTOS | A falha era no `HAL_I2C_IsDeviceReady`, antes de qualquer janela de protocolo. Escalonamento nao causa NACK. |
| Barramento I2C travado com SDA em low | `HAL_ERROR` com `ErrorCode == 0` e NACK limpo; barramento preso devolveria `HAL_BUSY`. |
| `ADVERT_TIMEOUT_US` curto demais | Alargar de 200 ms para 2 s nao mudou nada. Revertido ao valor do upstream. |
| Fiacao do INT | `INT=0` nao provava nada: PB4 e `GPIO_NOPULL`, entao fio aberto flutua. |
| I2C em 400 kHz | Mudado para 100 kHz junto com a correcao real; mantido por ser o consenso para BNO08x, mas nao era a causa. |
| Dispositivo nao ser SH2 de verdade | `part=10004148`, `fw=3.2.13` e milhares de pacotes validos. E SH2 genuino. |

`App/Src/bno08x.c` mantem `i2c2_bus_recovery()` no caminho de erro: e pratica
padrao de I2C e custa nada, mas nunca foi comprovadamente necessario aqui.

## O que foi implementado

- BNO08X: SH-2/SHTP pelo driver oficial CEVA, I2C2 a 100 kHz; reset físico, sondagem dos endereços 0x4A/0x4B por até 400 ms, leitura do Product ID e tentativa de reconexão a cada 2 s após falhas.
- Reports BNO: Rotation Vector (0x05), giroscópio calibrado (0x02) e aceleração linear (0x04), solicitados a 100 Hz cada. Quaternion na ordem **w,x,y,z**, velocidade angular em rad/s e aceleração em m/s², **com a gravidade já removida** pelo próprio BNO.
- MTF-01: **Micolink**, USART2 a 115200 8N1, DMA circular e buffer de recepção. Verifica soma de verificação e tamanho do payload 0x51, preserva sinais negativos e trata sequência, reinício e perda de bytes.
- Snapshots por `bno_get_sample()` e `mtf_get_sample()`, protegidos contra leitura durante atualização. Dados sem atualização por mais de 250 ms ficam inválidos; os últimos valores permanecem disponíveis para diagnóstico.
- Logs USB CDC a cada 500 ms. USB ocupado/desconectado descarta o log e incrementa `log_drop`; não espera o computador para continuar a aquisição.

### Comunicação com Raspberry Pi existente

`App/Src/dronelink` já contém UART/DMA, MAVLink 2, heartbeat, identificação de boot em flash externa e transações de comandos. USART1 continua exclusiva para esse protocolo. O dialeto está em `protocol/drone_controller.xml`; ambos os lados precisam usar esse dialeto.

`flight_command_stub.c` ainda nega armamento e execução de voo com `NOT_IMPLEMENTED`, e informa atuação indisponível. A leitura nova não altera esse comportamento. Os dados brutos dos sensores ficam na API de sensores e nos logs USB: **não são anunciados como atitude NED ou posição válida na telemetria de voo**. Isso depende da transformação dos eixos e do estimador. O controle de motores/DShot não foi iniciado.

## Ligações

| Dispositivo | Ligação no STM32 |
|---|---|
| BNO SCL | PB10, I2C2 SCL |
| BNO SDA | PB3, I2C2 SDA |
| BNO INT/H_INTN | PB4, ativo baixo |
| BNO RST/NRST | PB5, ativo baixo |
| MTF TX | PA3, USART2 RX |
| MTF RX | PA2, USART2 TX |
| Raspberry Pi TX, GPIO14/pino 8 | PA10, USART1 RX |
| Raspberry Pi RX, GPIO15/pino 10 | PA9, USART1 TX |
| USB do computador | Conector USB da Black Pill, PA11/DM e PA12/DP |
| Referência | GND comum entre módulos |

MTF: alimentação de 5 V; interface UART LVTTL de 3,3 V conforme fabricante. No BNO, confirme a alimentação permitida pela **placa breakout** adquirida e selecione o modo I2C nos pinos PS0/PS1 conforme seu módulo. SDA/SCL precisam de pull-ups para 3,3 V (muitos módulos já os incluem). INT e RST são necessários nesta implementação. Ela é para BNO08X, não BNO055.

PB3 permanece no I2C; SWO também usaria esse pino. USART6 compartilha PA11/PA12 com USB no encapsulamento de 48 pinos. Por você gravar em DFU, os logs foram implementados como USB CDC, sem reservar uma terceira USART. VBUS sensing está desligado para preservar PA9/PA10 usados pela Raspberry Pi. O PLL já fornece os 48 MHz do USB.

## Configurar o MTF-01

O parser desta versão aceita **somente Micolink**. Se o módulo estiver em MAVLink ou MSP, a UART pode receber dados sem aparecer nenhuma amostra válida.

No [MicoAssistant](https://micoair.com/assistant/), conecte o MTF por um adaptador USB–TTL de lógica 3,3 V, selecione 115200 e altere o protocolo para Micolink. Grave a configuração e reinicie o sensor. Não confunda essa COM do adaptador com a COM de logs do STM32: a porta de logs não é uma ponte para configuração do MTF. Não enviamos comandos de reconfiguração desconhecidos ao sensor.

O [manual do fabricante](https://github.com/micoair/MTF-01_USER_MANUAL) explica o procedimento. Sem adaptador, se o sensor ainda não estiver em Micolink, será necessário configurar por uma ponte serial apropriada antes deste ensaio.

## Compilar e gravar por DFU

Os arquivos prontos desta revisão ficam em `build/Release/DroneController.hex`, `.bin` e `.elf`: **este é o firmware a gravar para uso normal.** Debug fica em `build/Debug`, e o firmware de ensaio de captura em `build/Capture` (só bancada). Os arquivos antigos em `Debug/` e `Release/` na raiz só serão atualizados quando você compilar novamente pelo CubeIDE.

1. Entre em DFU pelo procedimento BOOT0/RESET que você já usa.
2. No CubeProgrammer, conecte por USB e selecione **`build/Release/DroneController.hex`**. O HEX já inclui os endereços. Se usar o BIN, o endereço inicial é **0x08000000**.
3. Grave e verifique. Retorne BOOT0 a zero e reinicie a placa para executar o firmware.
4. Agora o dispositivo deve aparecer no Gerenciador de Dispositivos em **Portas (COM e LPT)** como porta serial USB. Em DFU ele aparece como bootloader, não como COM.

O USB CDC é parte da aplicação, e o DFU usado para gravar é o bootloader de fábrica. Para gravar novamente, feche o terminal e repita BOOT0/RESET. Mesmo se a aplicação não enumerar, o procedimento físico de DFU permanece disponível.

### STM32CubeIDE

Atualizei `.cproject` com as fontes USB e os includes necessários. Atualize o projeto com **Refresh/F5** e faça **Project > Clean**, depois **Build Project**. A árvore `App/` é compilada automaticamente pelo projeto. Não é necessário regenerar pelo CubeMX para compilar esta entrega.

Também há um build independente dos objetos antigos do IDE:

```powershell
python tools/build.py
python tools/build.py --debug
python tools/build.py --capture   # firmware de ensaio de captura, em build/Capture
# Se a instalação do compilador não for encontrada:
python tools/build.py --gcc "C:\caminho\arm-none-eabi-gcc.exe"
```

O script usa apenas a biblioteca padrão Python, compila todas as fontes e gera ELF/HEX/BIN, mais um `manifest.json` com os hashes. Não grava na placa.

### Depois de regenerar no CubeMX

O `.ioc` contém USB Device CDC, PA11/PA12, USB IRQ de prioridade 6, VBUS sensing desativado, heap RTOS de 32768 bytes e stack de 2048 bytes para defaultTask. Drivers de aplicação estão fora de `Core`; as chamadas em `freertos.c` estão em blocos USER CODE.

A parte USB foi integrada manualmente a partir do pacote ST instalado. **A regeneração pelo CubeMX ainda não foi executada/verificada nesta revisão.** Faça backup antes de regenerar e confira:

- Uma única chamada `MX_USB_DEVICE_Init()` em `StartDefaultTask`; um único `OTG_FS_IRQHandler` com `HAL_PCD_IRQHandler`.
- Manutenção dos includes e source folders `App`, `USB_DEVICE`, `Middlewares` e `Drivers` no IDE.
- `sensors_init()` e `sensor_log_start()` em `MX_FREERTOS_Init()`.
- Não ativar VBUS/ID em PA9/PA10, nem mudar a prioridade do SysTick usada pelo RTOS.
- A implementação gerada de CDC precisa responder GET/SET_LINE_CODING e rearmar RX. O logger mantém seu próprio buffer persistente e testa TxState antes de transmiti-lo.

## Ver logs no Windows

**A janela de log do CubeProgrammer mostra operações de gravação; para os logs da aplicação, abra a porta COM em um terminal serial.** O seletor UART do CubeProgrammer é para comunicação com o bootloader, não um monitor serial da aplicação.

Opção sem instalar biblioteca Python: abra PowerShell nesta pasta e execute:

```powershell
.\tools\read_logs.ps1
.\tools\read_logs.ps1 -Port COM5 -OutputPath logs\bancada.txt
```

Troque `COM5` pela porta mostrada no seu computador. O primeiro comando lista as portas. Ctrl+C encerra. Se a política de execução bloquear o script, execute apenas essa chamada com:

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\read_logs.ps1 -Port COM5
```

Também funciona em um terminal serial configurado para **115200, 8 bits, sem paridade, 1 stop bit, sem controle de fluxo**. No USB CDC o baudrate é uma configuração virtual. Não abra a mesma COM em dois programas ao mesmo tempo. Se reiniciar a placa ou voltar do DFU, pode ser necessário reabrir a COM. No Windows moderno a classe CDC normalmente usa o driver USB serial do sistema; o driver de DFU é diferente.

### Ligar e desligar grupos de log

Todos os grupos ficam listados juntos no topo de `App/Inc/sensor_log.h`. Troque
o `1` por `0` para silenciar, `0` por `1` para trazer de volta, recompile e
grave. Um grupo desligado é compilado fora: não gasta CPU nem banda de USB.

```c
#define LOG_BNO_STATUS  0 /* online, fault, address, firmware, probe_ms, resets   */
#define LOG_BNO_DATA    0 /* quaternion, gyro, accel, per-report counters and gaps */
#define LOG_MTF         0 /* raw flow, range, frame/CRC/UART counters              */
#define LOG_ATTITUDE    0 /* ângulos de Euler e quatérnio                           */
#define LOG_HEIGHT      0 /* alcance e altura corrigida pela inclinação             */
#define LOG_POSITION    1 /* DESLOCAMENTO_MM x,y do EKF e diagnóstico compacto       */
#define LOG_POSITION_D  0 /* incremento por frame, velocidades e saúde do filtro    */
#define LOG_CALIBRATION 0 /* flow_calibrator.h live progress - see DISP_CALIBRATE_FLOW below */
#define LOG_SYS         0 /* free heap, link drops, dropped log lines              */
```

O padrão atual é o perfil visual simples de deslocamento: somente
`LOG_POSITION`, que imprime `DESLOCAMENTO_MM x,y` a partir do EKF, seguido de
validade, idade e contadores de rejeição na mesma linha.
Atitude, altura, incrementos e diagnósticos continuam disponíveis pelas outras
chaves. `LOG_CALIBRATION` só faz sentido junto com `DISP_CALIBRATE_FLOW` - ver
"Calibrar a escala do fluxo contra o giroscopio" acima.

Com todos os grupos ligados a linha chega a 1656 bytes no pior caso, contra um
buffer de 2048. Se você adicionar campos, confira essa margem: o log é um
`snprintf` só, e no estouro a mensagem **inteira** é descartada em silêncio.

### Interpretar a saída

- `HEIGHT range_mm`: alcance bruto no eixo do sensor. `height_mm` é o mesmo valor corrigido pela inclinação (`range * cos(tilt)`), que é a altura real. Os dois só coincidem com a placa nivelada; `tilt_mdeg` mostra rolagem e arfagem em milidegraus.
- `DESLOCAMENTO_MM x,y`: deslocamento horizontal acumulado pelo EKF
  FUSED, em milímetros. Começa em `0,0` a cada boot; X e Y seguem a orientação
  física do sensor. Os campos comparativos RAW/EKF só aparecem na captura ou
  quando um perfil de diagnóstico próprio é habilitado.
- `POS_D raw_d_um` / `fused_d_um`: **o incremento que o último quadro somou** ao acumulado, em micrômetros (é pequeno: 5 mm/quadro já é 5000). `*_v_mm_s` é a velocidade instantânea e `dt_ms` o intervalo daquele quadro. Multiplicar velocidade por `dt` reproduz o incremento.
- `CAL scalar_x1000`: fator de correção que `displacement_set_flow_scale()` aplicaria agora, ×1000. Só significa algo com `running=1` e `valid=1`; ver "Calibrar a escala do fluxo contra o giroscopio" para o procedimento completo.
- `BNO online=1`: os três reports chegaram recentemente. `acc` informa qualidade 0..3 por report, separada da presença de dados; online não significa calibração suficiente para voo.
- `q_x10000`: divida por 10000, ordem w,x,y,z. `gyro_mrad_s` e `accel_mm_s2`: divida por 1000 para rad/s e m/s².
- `n`: contadores acumulados dos reports. Para taxa recebida, use diferença de contadores dividida pelo intervalo entre dois logs. `gaps`: lacunas de sequência; `age_ms`: idade da última leitura. O log resumido de 2 Hz não substitui captura evento a evento para as métricas completas do relatório.
- `fault`: motivo da última queda, do enum `bno_fault_t` em `App/Inc/bno08x.h`. `0` = nenhuma; `1` = nenhum ACK no I2C; `2` = falha de transporte; `3` = enquadramento SHTP; `4` = sem notificação de reset; `5` = Product ID; `6` = configuração; `7` = reports pararam.
- `probe_ms`: tempo entre soltar o RST e o primeiro ACK no I2C. Esta placa lê ~80. Valor muito acima disso indica módulo lento ou marginal; `0` com `online=0` significa que nunca respondeu.
- `part/fw/build`: identificação de software do hub via Product ID; registre também a marcação física para identificar o exemplar BNO08X.
- `MTF mm`: distância no eixo do sensor, **não altura vertical compensada**. `range_ok` exige status 1, faixa de 10..8000 mm e dado recente; `flow_ok` exige status 1, qualidade não nula e dado recente. São critérios de recepção para bancada, não de validação de voo.
- `flow_cm_s_at_1m`: fluxo normalizado, com unidade cm/s a 1 m. Não é velocidade já compensada do drone. A aproximação do fabricante multiplica esse valor pela distância em metros; inclinação, rotação, orientação e qualidade ainda precisam ser tratadas pelo estimador.
- `crc_err`, `uart_err`, `drop`: corrupção, falhas da UART e bytes descartados. `log_drop` pode crescer normalmente enquanto a COM está fechada.
- `seen=0`: nenhum frame Micolink válido recebido. Confira protocolo, alimentação, GND e cruzamento TX/RX. `seen=1` com idade crescendo: o sensor parou de atualizar.

## Ensaio inicial

1. Ligue sem ESC; confirme que a COM aparece e os logs chegam mesmo com sensores desconectados.
2. Conecte o BNO, confirme endereço e Product ID. Em repouso, giroscópio próximo de zero e módulo da aceleração próximo de 9,81 m/s². Gire lentamente e observe o quaternion.
3. Aponte o MTF para uma superfície texturizada e iluminada, inicialmente a 0,2..1 m. Compare distância com uma régua e mova lateralmente para observar os sinais do fluxo.
4. Desconecte/reconecte cada sensor. Dados devem se tornar inválidos; o BNO deve tentar reconectar e o MTF voltar a aceitar frames válidos. Evite alimentar módulos pelos pinos de sinal.
5. Abra/feche o terminal durante a aquisição. Confira que contadores continuam avançando. Registre os logs para avaliar taxas, perdas e reinícios.

Há estimativa de deslocamento (três estimadores), mas ela ainda não é enviada à Raspberry Pi como posição válida. Não há controlador LQR, driver de ESC ou validação em voo nesta etapa. Os motores permanecem sem partida.

## Revisão do relatório e evidências

As seções 2.6.2, 3.2.2 e 3.3.2 do REPORT5 são coerentes com a base: BNO08X por SH-2/I2C, MTF por UART e caracterização de taxas reais. O Quadro 1 pode ser atualizado para incluir PA2/PA3 (MTF) e PA11/PA12 (logs USB). O relatório distingue corretamente fluxo de velocidade do veículo e distância de altura; a API mantém essa distinção. O texto descreve projeto e procedimentos, e não demonstra que o controle de voo já esteja implementado.

Validações executadas: compilação completa ARM GCC 14.3 em Release e Debug, sem avisos; parser C no GCC/WSL com AddressSanitizer e UndefinedBehaviorSanitizer (fragmentação, valores negativos, checksum, sequência, duplicata, reset, timeout, tamanho inválido e 1 milhão de bytes de ruído). Sem ensaio físico ou gravação da placa nesta sessão.

Em 2026-09-16: builds Release e Capture sem avisos; oito suítes de host e a verificação do replay passando; dois ensaios físicos gravados com o build Capture (75 s, 0 perdas, replay idêntico à placa). Resultados em `docs/evolucao_algoritmo_deslocamento.md`, Seção 6.

```bash
gcc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -Itests/stubs -IApp/Inc tests/test_mtf.c App/Src/mtf_parser.c \
  -o /tmp/drone-test-mtf && /tmp/drone-test-mtf
```

Fontes técnicas: [driver CEVA SH-2](https://github.com/ceva-dsp/sh2), revisão fixada em `App/Src/sh2/UPSTREAM.txt`; [protocolo Micolink](https://micoair.com/docs/decoding-micolink-messages-from-mtf-01/); [MTF-01](https://micoair.com/optical_range_sensor_mtf-01/); [datasheet STM32F411](https://www.st.com/resource/en/datasheet/stm32f411ce.pdf); [CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html). USB/HAL provenientes do STM32Cube_FW_F4_V1.28.3 local; alterações de integração em `USB_DEVICE/`.
