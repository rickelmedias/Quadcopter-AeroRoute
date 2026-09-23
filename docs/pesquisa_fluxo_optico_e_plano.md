# Pesquisa de fluxo óptico para navegação indoor

## Objetivo

Este documento compara o estimador do DroneController com implementações de
fluxo óptico usadas em ArduPilot, PX4, INAV, Betaflight e Crazyflie. O foco é
explicar o erro observado no ensaio de ida e volta com o MTF-01 e definir uma
sequência de melhorias que caiba no STM32F411 e no hardware atual.

O resultado principal é simples: o erro não tem uma única causa. Há uma parte
física do MTF-01, uma parte de sincronização entre MTF e BNO, uma parte do
modelo de medição e uma limitação fundamental de observabilidade. O firmware
atual já tem uma boa infraestrutura de aquisição, diagnóstico, captura e
replay, mas ainda não associa as medições como os pilotos automáticos maduros.

## Sintoma observado

No ensaio manual, a caixa percorreu aproximadamente 20 cm, voltou à origem e o
estimador conservou cerca de 50 mm de erro por ciclo. Após quatro ciclos, o
erro acumulado chegou perto de 200 mm, inclusive com componente no outro eixo.
O MTF também exigiu mais luz e cerca de 30 cm de distância para produzir fluxo
útil.

Esses fatos são compatíveis com quatro mecanismos:

1. O fluxo do MTF-01 é quantizado. No protocolo usado pelo projeto, uma
   contagem equivale a 0,2 rad/s. A 30 cm, isso representa cerca de 60 mm/s.
   Movimentos lentos podem desaparecer em sequências de zeros.
2. O MTF mede o movimento durante um intervalo, mas o firmware compensa esse
   intervalo com a última leitura disponível do giroscópio. Um pequeno atraso
   muda de sinal justamente nas inversões do movimento.
3. A regra atual transforma fluxo bruto zero em velocidade translacional zero.
   Isso evita movimento falso em repouso, mas também apaga movimento verdadeiro
   abaixo da resolução e falha quando rotação e translação se cancelam na
   imagem.
4. Mesmo com uma velocidade sem viés aparente, integrar velocidade não cria
   uma referência absoluta. Qualquer erro residual vira erro de posição e não
   existe uma medida que o faça voltar ao zero verdadeiro.

Iluminação e distância também são limitações reais do sensor. A MicoAir
especifica alimentação de 5 V, mais de 60 lux, distância de trabalho do fluxo
acima de 8 cm, saída a 100 Hz e campo de visão de 42 graus. O limite de 8 cm é
um limite nominal; textura, foco, velocidade e iluminação determinam a margem
útil de cada bancada. Portanto, funcionar melhor a 30 cm e com uma luminária
não é, por si só, sinal de defeito no código.

Fonte: [especificações oficiais do MTF-01](https://micoair.com/optical_range_sensor_mtf-01/).

## O que os projetos maduros fazem

### INAV

O INAV acumula as amostras do giroscópio entre duas atualizações do fluxo e
usa a velocidade angular média desse mesmo período. Ele também trata o fluxo
como uma medição acumulada com `deltaTime`, aplica alinhamento configurável e
possui histerese de qualidade. Isso evita usar um giro instantâneo que pode
pertencer a outro momento.

O próprio guia do INAV avisa que o modo sem GPS deriva ao longo do tempo, exige
piso com textura e boa iluminação, e usa o telêmetro para converter o fluxo em
movimento horizontal. A documentação ainda oferece escala, alinhamento e pesos
separados para posição e velocidade.

Fontes:

- [código do processamento de fluxo do INAV](https://github.com/iNavFlight/inav/blob/master/src/main/sensors/opflow.c)
- [guia oficial de fluxo óptico e rangefinder](https://inavflight.github.io/docs/core-features/optical-flow-rangefinder/)

### ArduPilot EKF3

O ArduPilot entrega ao EKF a medição óptica bruta, a velocidade angular média
do sensor durante o intervalo, o instante estimado da medição e a posição do
sensor no corpo. Antes de enfileirar a observação, estima a diferença de bias
entre o giroscópio do sensor de fluxo e a IMU. A medição é atrasada até o
horizonte temporal correto do filtro.

O EKF mantém estados e informações que o projeto atual não mantém: bias dos
sensores, altura do terreno, covariâncias acopladas e deslocamento físico do
sensor em relação ao centro do veículo. O ArduPilot também permite usar fluxo
antes de o yaw absoluto estar alinhado; isso mostra que sustentação horizontal
por fluxo não precisa depender de norte magnético.

Fontes:

- [entrada e sincronização da medição no EKF3](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp)
- [estrutura da observação de fluxo](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_core.h)
- [fusão de fluxo do EKF3](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_OptFlowFusion.cpp)

### PX4 EKF2

O PX4 representa o fluxo como ângulo integrado durante um intervalo e também
transporta o giro integrado. Para uma rotação pura, fluxo integrado e giro
integrado devem concordar; essa relação é usada na calibração da escala. O
EKF2 trabalha num horizonte de fusão atrasado para alinhar sensores que chegam
com latências diferentes.

O PX4 permite informar X, Y e Z do ponto focal em relação ao centro do veículo.
Isso corrige a velocidade aparente causada por `omega x braço` quando o sensor
está afastado do centro de rotação. Seu EKF estima, entre outros estados,
atitude, velocidade, posição e biases de giroscópio e acelerômetro.

Fontes:

- [guia oficial de sensores de fluxo do PX4](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/sensor/optical_flow.md)
- [arquitetura e horizonte de fusão do EKF2](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/advanced_config/tuning_the_ecl_ekf.md)
- [mensagem de fluxo integrada do PX4](https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/msg_docs/VehicleOpticalFlow.md)

### Betaflight

O estimador atual do Betaflight usa filtros de três estados por eixo: posição,
velocidade e aceleração. Para o fluxo, calcula a variância da medição em função
da qualidade; qualidade maior reduz `R`, qualidade menor diminui o peso da
amostra até o limite de rejeição. Ele também usa o timestamp e a cadência
medida do sensor, em vez de confundir a frequência de consulta da task com a
frequência real da fonte.

Esta é uma melhoria pequena e valiosa para o nosso filtro: hoje a qualidade só
é um corte em 20. Uma amostra de qualidade 21 recebe exatamente o mesmo ruído
que uma de qualidade 95.

Fonte: [estimador de posição do Betaflight](https://github.com/betaflight/betaflight/blob/master/src/main/flight/position_estimator.c).

### Crazyflie Flow Deck

O Crazyflie combina um PMW3901 com um telêmetro ToF e usa um EKF quando o Flow
Deck está presente. Sua estrutura de medição conserva timestamp, deslocamento
de pixels acumulado, intervalo de integração e desvio padrão por eixo. O
modelo de medição converte fluxo e altura dentro do filtro.

Quando precisa de posição absoluta, o Crazyflie acrescenta outra fonte, como
Loco UWB, Lighthouse ou captura de movimento. O Flow Deck sozinho estabiliza e
mede movimento relativo; ele não cria uma coordenada absoluta da sala.

Fontes:

- [documentação dos estimadores do Crazyflie](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/functional-areas/sensor-to-control/state_estimators.md)
- [tipo da medição de fluxo](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/interface/stabilizer_types.h)

## Comparação com o DroneController

| Tema | Projeto atual | Projetos de referência | Avaliação |
|---|---|---|---|
| Aquisição MTF | UART DMA, parser com checksum, sequência e relógio do sensor | Equivalente | Bom |
| Saúde dos sensores | idade, validade, qualidade, gaps e contadores | Equivalente | Bom |
| Captura e replay | eventos independentes, IDs e reprodução das decisões | Blackbox/log/replay | Muito bom para investigação |
| Intervalo do MTF | `dt` do relógio do MTF | intervalo próprio da fonte | Bom |
| Associação IMU/MTF | último quatérnio, giro e aceleração disponíveis quando a task vê o frame | buffer temporal, integração/interpolação e atraso conhecido | Principal lacuna |
| Compensação angular | giro instantâneo somado ao fluxo | giro médio ou integrado no mesmo intervalo | Provável fonte de viés |
| Fluxo igual a zero | força translação igual a zero | medição com ruído/resolução | Regra fisicamente incompleta |
| Qualidade | rejeição abaixo de 20 | gate mais peso/variância contínua | Simplificação melhorável |
| Altura | alcance bruto do mesmo frame | altura/terreno filtrado e testado | Simplificação melhorável |
| Calibração XY | escala e sinal independentes | escala, rotação/alinhamento e convenção completa | Falta acoplamento entre eixos |
| Referencial horizontal | gira tudo pelo yaw absoluto do Rotation Vector | fluxo pode sustentar velocidade sem yaw absoluto alinhado | Dependência desnecessária |
| Bias do acelerômetro | não estimado | estado ou correção explícita | Deriva inercial residual |
| Braço do sensor | assume centro de rotação | posição 3D configurável | Pode gerar velocidade aparente |
| Medição no filtro | velocidade já compensada e girada | fluxo bruto em modelo de observação | Perde informação cedo |
| Posição de longo prazo | integral local | outra fonte absoluta quando necessário | Limite físico, não bug |

## Pontos incorretos ou frágeis no código atual

### 1. O pareamento temporal é o primeiro problema a corrigir

`displacement.c` lê um snapshot do BNO e usa seus últimos valores para todo
frame novo do MTF. Os três relatórios do BNO chegam independentemente e podem
ter timestamps diferentes. O código já captura `timestamp` e `delay` do SH-2,
mas esses dados não entram no estimador.

Em movimento uniforme, alguns milissegundos produzem pouco erro. Na parada e
na inversão, o sinal muda rapidamente: fluxo do intervalo anterior pode ser
combinado com giro ou atitude do intervalo seguinte. Repetir ida e volta pode
então repetir o mesmo erro com o mesmo sinal.

### 2. “Fluxo zero significa translação zero” não é uma observação válida

A regra em `disp_estimator.c` resolveu um sintoma real em repouso: subtrair um
giro pequeno de um fluxo quantizado para zero fabricava deslocamento. Porém,
zero significa apenas que o valor verdadeiro caiu na célula de quantização do
sensor. Aproximadamente, uma leitura zero informa um intervalo em torno de
zero; não informa um valor exato.

Além disso, translação e rotação podem se cancelar na imagem. Nesse caso o
fluxo bruto pode ser zero enquanto o veículo se move. Esse cenário é normal em
um drone, pois ele inclina para acelerar.

### 3. O yaw absoluto pode transformar erro de rumo em erro no outro eixo

O caminho FUSED gira toda velocidade para o mundo usando o yaw do
`SH2_ROTATION_VECTOR`. Nos ensaios documentados, a acurácia desse vetor ficou
em zero. Mesmo que o yaw relativo pareça estável, um erro de rumo gira parte do
movimento X para Y. Isso é compatível com o resíduo observado no outro eixo.

Para estabilizar um hover indoor, o controlador pode trabalhar em eixos do
corpo ou num referencial local cujo yaw é fixado quando o modo é ativado. Norte
magnético não é necessário para impedir deriva lateral de curto prazo.

### 4. O ruído da medida ignora informação disponível

`flow_ekf.c` usa um ruído angular fixo de 0,25 rad/s multiplicado pela
distância. Ele não inclui qualidade, quantização, incerteza de altura,
inclinação ou erro de sincronização. O gate distingue uma amostra absurda, mas
trata todas as amostras aceitas como igualmente confiáveis.

### 5. A correção chega tarde demais ao filtro

O EKF recebe uma velocidade que já passou por escala, sinais, soma com gyro,
regra de zero, multiplicação pela distância e rotação pelo yaw. Ele não pode
mais estimar qual parcela veio do bias do IMU ou da incerteza óptica. Um EKF
completo como PX4/ArduPilot resolve isso com muitos estados; no STM32 podemos
obter grande parte do benefício com um modelo menor.

### 6. Escala por eixo não corrige desalinhamento pequeno

Dois fatores de escala e dois sinais só representam uma matriz diagonal. Uma
rotação mecânica de poucos graus, assimetria ou troca parcial entre eixos
exige uma matriz 2 x 2. Esse erro também transforma uma ida em X num acúmulo em
Y.

### 7. A posição continuará derivando sem uma âncora

O próprio `flow_ekf.h` já documenta corretamente esta limitação. Fluxo mede
velocidade relativa ao piso. O telêmetro dá escala e altura. A IMU melhora a
predição. Nenhum deles observa a coordenada X/Y absoluta do ponto inicial.

O objetivo realista com o hardware atual é:

- velocidade horizontal boa para estabilizar o drone;
- posição relativa de curto prazo para pequenos deslocamentos;
- origem reiniciada ao armar ou entrar no modo de posição;
- validade que cai quando a observação deixa de ser confiável.

## Plano recomendado

### Fase 0 — produzir um conjunto de evidências repetível

Antes de mudar a matemática, gravar capturas binárias nas mesmas condições:

1. MTF alimentado em 5 V, sinais UART em 3,3 V, piso bem iluminado e texturizado.
2. Altura fixa de aproximadamente 30 cm.
3. Dez ciclos de ida e volta entre batentes físicos separados por uma distância
   medida.
4. Repetir em velocidades lenta, média e rápida, começando ora para +X, ora
   para -X.
5. Gravar também 30 s parado e rotações sem translação em roll, pitch e yaw.
6. Anotar no ensaio os instantes dos batentes, direção e distância real.

O analisador deve calcular por trecho: soma positiva, soma negativa, fechamento,
percentual de zeros, qualidade, distância, fluxo por eixo, gyro médio e atraso
que maximiza a correlação fluxo/gyro. Isso separa quantização, escala,
desalinhamento e atraso.

Critério: nenhuma mudança avança só porque “pareceu melhor” numa passagem. Ela
precisa reduzir o erro mediano de fechamento no mesmo arquivo capturado, sem
aumentar movimento falso nos 30 s parados.

### Fase 1 — alinhar o tempo da IMU com o MTF

Esta é a mudança com maior retorno esperado.

1. Guardar históricos circulares separados de gyro, quatérnio e aceleração,
   com timestamp de medição do SH-2 convertido para uma linha de tempo comum.
2. Para cada intervalo `[t0, t1]` do MTF, integrar o gyro nesse intervalo e
   obter a taxa média `delta_angulo / dt`.
3. Interpolar o quatérnio no ponto médio do intervalo.
4. Usar aceleração média ou interpolada no mesmo período.
5. Estimar offline um atraso fixo residual MTF/BNO pelas capturas de rotação e
   aplicá-lo como parâmetro, sem esconder o valor no código.
6. Ampliar `OBSERVATION`/`STEP` para registrar IDs ou resultados dessa
   associação, preservando o replay determinístico.

Não é necessário copiar o horizonte completo do PX4. Um buffer de algumas
centenas de milissegundos e interpolação linear já corrigem a maior diferença
arquitetural.

### Fase 2 — modelar quantização e qualidade

1. Remover a regra rígida `flow bruto zero -> translação zero`.
2. Tratar cada valor quantizado como um intervalo de largura aproximada de uma
   contagem. Para zero, só aplicar inovação quando a previsão estiver fora da
   faixa compatível; dentro dela, a medida não corrige nem inventa movimento.
3. Calcular `R` por amostra com parcelas de:
   - quantização do MTF;
   - qualidade da imagem;
   - distância e sua incerteza;
   - incerteza do gyro médio e do atraso.
4. Manter um limite mínimo de qualidade com histerese, evitando alternar válido
   e inválido a cada frame perto do limiar.

Essa atualização por intervalo é uma inovação adequada ao MTF-01: ela conserva
a informação verdadeira de uma leitura zero sem fingir precisão que o sensor
não possui.

### Fase 3 — corrigir geometria e referencial

1. Substituir escala/sinal por uma matriz de calibração 2 x 2:

   `fluxo_corpo = A * fluxo_sensor`.

2. Estimar `A` com translações e rotações controladas nos dois eixos.
3. Configurar o vetor entre centro do veículo e ponto focal do MTF e compensar
   `omega x braço`.
4. Para o primeiro controle de hover, manter velocidade num referencial local
   de yaw fixado ao ativar o modo, ou diretamente no corpo.
5. Testar `SH2_GAME_ROTATION_VECTOR` como fonte de atitude relativa indoor,
   pois ele não depende do magnetômetro para o rumo. A troca deve ser validada
   por atraso, estabilidade e replay; não deve ser feita só pelo nome do
   relatório.

### Fase 4 — melhorar o estimador sem copiar um EKF de autopiloto inteiro

Começar com um filtro pequeno por eixo, com estado:

`[posição, velocidade, bias_aceleração]`.

Predizer com a aceleração do BNO e corrigir com a observação óptica sincronizada.
O estado de bias impede que um pequeno erro constante da aceleração seja
integrado como velocidade. Em paralelo:

1. filtrar a altura com um filtro alfa-beta ou Kalman 1D;
2. aplicar gate a saltos do telêmetro;
3. fundir o fluxo como taxa angular ou velocidade do corpo antes da rotação
   final para o referencial local;
4. manter posição e velocidade separadas na API, com validade própria;
5. publicar covariância ou incerteza, e não apenas `valid`.

Se as fases 1 a 3 eliminarem a maior parte do viés, talvez esse filtro de três
estados seja suficiente. Um EKF de 15 a 24 estados só se justifica se os dados
mostrarem que atitude, biases e velocidade precisam ser estimados em conjunto.

### Fase 5 — usar o fluxo da forma certa no controlador

1. O primeiro modo deve controlar **velocidade horizontal**, que é diretamente
   observada pelo fluxo.
2. O position hold integra essa velocidade apenas a partir do instante de
   ativação e para uma janela curta.
3. Ao desarmar, pousar ou ativar o modo, reiniciar a origem local de forma
   explícita.
4. Quando desarmado e comprovadamente parado, usar uma atualização de velocidade
   zero para estimar bias. Não aplicar essa suposição durante o voo.
5. Se qualidade, altura, IMU ou idade da correção falhar, sair do position hold
   de modo controlado; não congelar a última posição como se ainda fosse válida.

### Fase 6 — referência absoluta futura

Para missões longas e retorno ao mesmo ponto físico, será necessária uma fonte
que observe posição. Opções usuais:

- AprilTag ou Visual-Inertial Odometry na Raspberry Pi;
- UWB com âncoras;
- Lighthouse ou captura de movimento em laboratório;
- câmera com SLAM e correções de pose;
- GPS quando houver ambiente externo.

A Raspberry Pi pode enviar correções lentas de posição ao STM32, enquanto o
MTF e a IMU mantêm a malha rápida. Essa arquitetura preserva o que o hardware
atual faz bem e evita exigir que a comunicação com a Pi rode na frequência do
controle.

## Priorização

| Prioridade | Mudança | Impacto esperado | Complexidade |
|---|---|---|---|
| P0 | ensaio repetível e métricas offline | impede ajuste por impressão | baixa |
| P0 | gyro médio e atitude no intervalo do MTF | reduz erro de inversão/rotação | média |
| P0 | remover zero rígido com modelo de quantização | recupera movimento lento sem inventar giro | média |
| P1 | `R` dependente de qualidade e distância | melhora peso de cada amostra | baixa |
| P1 | referencial local sem yaw magnético absoluto | reduz acoplamento X/Y | baixa/média |
| P1 | matriz 2 x 2 de alinhamento | reduz vazamento entre eixos | média |
| P1 | altura filtrada e gate de saltos | estabiliza escala da velocidade | baixa |
| P2 | compensação do braço do sensor | remove velocidade em rotações | baixa |
| P2 | estado de bias do acelerômetro | reduz deriva na predição | média |
| P3 | correção absoluta pela Raspberry Pi | limita posição a longo prazo | alta/novo sensor |

## Proposta de primeira entrega

A primeira implementação deve conter somente:

1. analisador das capturas de ida e volta;
2. histórico temporal do BNO;
3. gyro médio no intervalo do MTF;
4. quatérnio interpolado no ponto médio;
5. atraso configurável e medido;
6. replay comparando algoritmo antigo e novo no mesmo arquivo.

Depois de medir o efeito dessa entrega, implementar o modelo de quantização e
o `R` variável. Essa ordem evita alterar sincronização, geometria, ruído e
estado do filtro ao mesmo tempo, o que tornaria impossível saber qual mudança
realmente corrigiu o erro.

## Implementação da interface Euler para o futuro LQR

O modelo de espaço de estados e o LQR do projeto foram desenvolvidos em
ângulos de Euler. Essa escolha é adequada para uma linearização em torno do
hover e não exige refazer as matrizes. A implementação adotou uma separação
entre representação matemática interna e interface de controle:

- o BNO08x continua entregando e armazenando o quatérnio normalizado;
- interpolações temporais futuras e rotações de vetores continuam usando o
  quatérnio;
- `bno_sample_t` passa a publicar também roll, pitch e yaw na convenção ZYX;
- o mesmo estado expõe p, q e r do giroscópio e as derivadas exatas dos três
  ângulos de Euler;
- `flight_snapshot_t` passa a ter atitude Euler para o controlador, mantendo o
  quatérnio para telemetria MAVLink e transformações internas;
- o erro angular usado em `delta-x` é normalizado para `[-pi, pi)`, evitando
  que a passagem de +179 para -179 graus pareça um erro de 358 graus;
- a conversão de p,q,r para derivadas de Euler detecta a singularidade em
  pitch próximo de 90 graus e não entrega números inválidos ao controlador.

As funções puras ficam em `App/Inc/attitude.h` e `App/Src/attitude.c`:

- `attitude_build_euler_state()` cria a parcela de atitude do vetor de estados;
- `attitude_wrap_pi()` normaliza um ângulo;
- `attitude_euler_error()` calcula referência menos medição com retorno
  angular correto.

O contrato distingue duas formulações possíveis do modelo:

1. Se o vetor de estados usa p,q,r, consumir `body_rate` diretamente. Essa é a
   formulação usual do quadricóptero linearizado.
2. Se o vetor usa roll_dot, pitch_dot,yaw_dot, consumir `euler_rate`. Perto do
   hover os dois conjuntos quase coincidem, mas a implementação não depende
   dessa aproximação.

Foram adicionadas regressões para a igualdade p,q,r no hover, passagem de yaw
por +-180 graus e rejeição das derivadas na singularidade. Ainda não existe um
módulo LQR ou acionamento de motores neste repositório; esta alteração prepara
a entrada correta sem inventar a ordem do vetor, os ganhos ou os limites do
controlador que ainda serão incorporados.

## Conclusão

O comportamento observado é parcialmente normal para fluxo óptico barato, mas
o fechamento de aproximadamente 50 mm por ciclo é grande o bastante para
investigar o firmware. A maior diferença concreta em relação a INAV,
ArduPilot e PX4 é temporal: usamos snapshots recentes; eles usam medições
integradas ou alinhadas ao mesmo intervalo.

O caminho mais promissor não é ajustar mais um ganho. É preservar o tempo e a
incerteza da medição, compensar o giro correto, controlar velocidade no curto
prazo e tratar posição como relativa enquanto não houver uma referência
absoluta.
## Implementação realizada após a pesquisa

As recomendações que corrigem a entrada do estimador foram incorporadas ao
firmware. Esta seção separa código ativo, infraestrutura pronta e trabalho que
depende de ensaio ou do futuro controlador.

### Sincronização BNO–MTF — implementada

Foi criado `bno_history.h/.c`, um módulo puro com históricos de rotação,
giroscópio e aceleração. Os timestamps SH-2 já eram produzidos a partir da
interrupção marcada pelo relógio do STM32; antes eram apenas capturados. Agora,
para cada intervalo do MTF:

- gyro e aceleração são integrados por trapézios e divididos pelo intervalo;
- o quatérnio é interpolado e normalizado no ponto médio;
- dados sem cobertura temporal próxima são rejeitados;
- o wrap do contador de microssegundos é tratado.

O fim do intervalo óptico usa o horário de recepção do próprio frame MTF com
um atraso inicial configurado de 10 ms (`DISP_MTF_DELAY_US`), em vez do horário
posterior em que a task produziu o STEP. Nas capturas `ensaio01` e `ensaio02`,
isso reduziu as falhas de cobertura temporal de 28/101 para zero. Os 10 ms são
uma estimativa obtida dessas capturas e ainda devem ser calibrados em bancada.

Firmware e replay chamam a mesma implementação. A captura conserva os eventos
BNO independentes, permitindo experimentar outros alinhamentos offline.

### Quantização e confiança — implementadas

A regra `fluxo zero = translação exatamente zero` foi removida. O FUSED calcula
a meia largura da célula que uma contagem representa na velocidade local. A
primeira implementação tratava a faixa como medição censurada, mas capturas
reais mostraram que isso deixava o bias da aceleração caminhar livre dentro da
célula: o ensaio parado passou de 3,8 para 763 mm. A implementação corrigida
mantém a leitura no centro informado pelo sensor e soma a variância uniforme
da quantização (`h²/3`) ao ruído. Assim o zero continua corrigindo o bias, mas
com peso menor e fisicamente explícito.

A qualidade permanece no gate com histerese 30/15. Ela não escala o ruído
continuamente porque ainda não existe calibração que relacione a escala 0..255
do MTF a uma variância física; a fórmula anterior piorava os dados reais.

### Referencial local — implementado

O primeiro frame aceito fixa `yaw_origin`; os seguintes usam
`wrap_pi(yaw-yaw_origin)`. O deslocamento deixa de depender do norte magnético.
`displacement_reset()` cria uma nova origem na observação seguinte.

A aceleração da IMU também é girada para esse mesmo referencial local. Antes,
o fluxo usava yaw relativo e a aceleração yaw absoluto; no ensaio o erro de
origem era aproximadamente -165 graus, quase invertendo a predição inercial.

Comparar `ROTATION_VECTOR` com `GAME_ROTATION_VECTOR` continua exigindo duas
capturas reais equivalentes; trocar sem medir atraso e ruído não seria uma
correção demonstrada.

### Geometria — infraestrutura implementada

A escala virou matriz 2x2 sensor->corpo. Os valores históricos ficam na
diagonal e os termos cruzados começam em zero. `displacement_set_flow_matrix()`
aplica uma calibração medida. `displacement_set_sensor_offset()` e a correção
`omega x braço` levam a velocidade do ponto focal ao centro do veículo.

Os termos cruzados e o braço permanecem neutros até serem medidos fisicamente.

### Altura — implementada

`range_filter.h/.c` suaviza o alcance e rejeita saltos isolados maiores que uma
margem mais uma velocidade vertical extrema. Cinco leituras coerentes em um
novo patamar reancoram o filtro. Isso permite recuperar uma mudança real de
altura ou a volta do telêmetro após perda, em vez de rejeitar para sempre tudo
que difere da última leitura aceita.

### Saída e pendências

`DESLOCAMENTO_MM` mostra a posição do Kalman, a altura atual filtrada em
`height_mm` (não acumulada) e, na mesma linha, validade, saúde, idade da última
correção, rejeições de alinhamento, rejeições e reancoragens do alcance e
rejeições do EKF. Isso mantém uma linha legível sem esconder a causa quando a
posição para de atualizar.

Após as correções, o replay de `ensaio01.bin` (placa parada por 30 s) terminou
em 0,2 x 0,8 mm e não teve falhas de alinhamento BNO. A versão censurada havia
chegado a 763 mm. `ensaio02.bin` terminou em 102 x 8,5 mm e também sem falhas
de alinhamento; como esse ensaio não teve régua, ele demonstra ausência do
disparo do filtro, não exatidão da trajetória.

O bias de acelerômetro como terceiro estado deve ser avaliado depois de uma
captura real mostrar o resíduo restante. Controle de velocidade, LQR, mixagem
e position hold continuam pendentes porque o repositório ainda não contém a
matriz, os ganhos, a ordem completa do vetor nem o driver de ESC. O contrato
Euler está pronto, mas o stub continua negando atuação com segurança.

As dez suítes de host e o replay exercitam o código de produção. O próximo
critério é uma captura real de ida e volta: testes sintéticos provam coerência,
não melhoria do modelo físico.
