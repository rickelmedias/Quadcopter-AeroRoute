# Evolução do algoritmo de deslocamento horizontal (MTF-01 + BNO08x)

Este documento registra, para fins de TCC, a evolução do algoritmo de
estimação de deslocamento horizontal do projeto DroneController, do método
mais simples até o estado atual. Cada estágio é descrito com o problema que
resolveu, como funciona, e os resultados medidos — separando sempre **testes
sintéticos** (dados controlados por software, exatos e reproduzíveis) de
**testes reais de bancada** (hardware físico, sujeitos a erro de medição
humano, explicitado quando relevante).

Todos os valores numéricos aqui foram conferidos por recomputação a partir
dos dados brutos antes da escrita deste documento. Nenhum valor foi
arredondado além do que a fonte original permite, e nenhuma comparação é
feita entre condições de ensaio diferentes sem indicar essa diferença.

## 1. Sensores usados

- **MTF-01** (MicoAir): sensor de fluxo óptico + telêmetro laser (LiDAR/ToF),
  protocolo Micolink via UART.
- **BNO08x** (CEVA/Hillcrest): unidade de medição inercial (IMU) com fusão de
  sensor embarcada, protocolo SH-2/SHTP via I2C.
- Ambos conectados a um STM32F411CEU6 ("Black Pill"), FreeRTOS.

O bring-up de cada sensor (fazer a comunicação funcionar de forma confiável)
é pré-requisito para qualquer cálculo de deslocamento e não produz uma
métrica de deslocamento em si, mas é resumido aqui por ser parte do trabalho.

### 1.1. Bring-up do BNO08x

Causa raiz do driver não inicializar: o chip leva **~80 ms** entre soltar o
pino de reset (RST) e responder no barramento I2C. O código sondava o
barramento **uma única vez, 20 ms** após soltar o RST — como 20 ms < 80 ms, a
falha era determinística. Corrigido trocando a sondagem única por *polling*
até 400 ms.

Resultado após a correção (log de bancada): `part=10004148 fw=3.2.13
build=6 resets=1 err=0`, com `reads=7517` e `packets=7517` (sem perdas) em
uma sessão de streaming contínuo — confirma um BNO08x genuíno operando SH-2
corretamente.

### 1.2. Bring-up e caracterização do MTF-01

Medido em bancada: **34 de 34 amostras** de fluxo óptico caíram em múltiplos
exatos de 20 na unidade nativa do sensor ("cm/s a 1 m"). Isso caracteriza a
resolução do sensor como um degrau de **0,2 rad/s** — não é ruído, é o menor
incremento que o sensor consegue reportar. Como a velocidade equivalente a um
degrau escala com a altura (`v_lsb = 0,2 rad/s × altura`), isso define um
piso de detecção que cresce com a distância ao solo:

| Altura | 1 degrau de fluxo equivale a |
|---|---|
| 0,84 m | 168 mm/s |
| 0,50 m | 100 mm/s |
| 0,25 m | 50 mm/s |
| 0,15 m | 30 mm/s |
| 0,10 m | 20 mm/s |

Esse piso é a limitação física de base sobre a qual todos os estágios
seguintes atuam — nenhum algoritmo de fusão recupera informação que o sensor
não mediu.

## 2. Estágio 1 — Estimador ingênuo (sem giroscópio)

**Método:** velocidade = fluxo_óptico × altura, integrada diretamente no
tempo, sem nenhuma correção de rotação. É o método mais simples possível de
converter fluxo óptico em deslocamento — a mesma classe de abordagem usada em
implementações de referência simples que não fundem IMU (ex.: sketches
Arduino/ESP32 sem giroscópio). No código deste projeto, corresponde à função
`disp_raw_update()` (`App/Src/disp_estimator.c`), mantida como baseline de
comparação ao lado dos estágios seguintes, não como solução de produção.

**Problema:** um sensor de fluxo óptico não mede translação — mede a taxa
angular com que a cena varre a imagem, que é a soma de duas coisas: o veículo
se deslocando sobre o chão, e o veículo girando no lugar. Sem separar essas
duas componentes, qualquer rotação (inclusive a que ocorre ao segurar o
sensor na mão) é contabilizada como se fosse deslocamento real.

### Resultado (teste sintético controlado)

Rotação pura simulada de 0,4 rad/s por 1 segundo, sem nenhuma translação real
(`range = 1 m`):

| Estimador | Deslocamento acumulado |
|---|---|
| Ingênuo (sem giroscópio) | **400 mm** (inteiramente inventado) |
| Compensado (estágio 2) | 0 mm |

### Observação em bancada (medida)

Em um ensaio de rotação isolada (a placa foi girada em rolagem e arfagem
segurando-a no mesmo ponto, sem deslocá-la), o estimador ingênuo acumulou
**1324 mm** de deslocamento aparente ao final — deslocamento esse que
fisicamente não ocorreu, já que a placa voltou aproximadamente à posição e
orientação iniciais. O mesmo ensaio, com o estimador compensado, acumulou
307 mm (ver Seção 5.4). Isso confirma em hardware o que o teste sintético
mostra de forma exata: sem compensação de rotação, o método é inutilizável
fora de uma bancada perfeitamente livre de rotação.

## 3. Estágio 2 — Compensação por giroscópio (BNO08x)

**Método:** antes de integrar, subtrai-se do fluxo medido a taxa de rotação
do corpo, medida pelo giroscópio do BNO08x. O que sobra é apenas a componente
de translação. A técnica segue o mesmo princípio que o ArduPilot usa antes de
fundir fluxo óptico no seu EKF (`AP_NavEKF3_OptFlowFusion.cpp`,
`AP_NavEKF3_Measurements.cpp`). Implementado em `disp_fused_update()`.

Duas correções adicionais nesse estágio:

- **Rotação para o referencial do mundo pelo guinada (yaw):** a velocidade
  compensada está no referencial do corpo; sem rotacioná-la pelo yaw atual
  antes de integrar, o próprio giro do veículo embaralha a trajetória
  acumulada.
- **Rejeição por qualidade/alcance/inclinação:** amostras com pouca textura,
  alcance inválido ou inclinação extrema são descartadas antes de entrar no
  cálculo.

### Resultado (teste sintético controlado)

Mesma rotação pura de 0,4 rad/s por 1 s: o estimador compensado fica em
**0 mm** (contra 400 mm do estágio 1) — ver tabela acima.

### Resultado em bancada, parado (referência antes das correções do estágio 3)

Placa imóvel sobre superfície estável, ~4000 quadros (40 s) a 0,8 m de
altura: **amplitude de variação (pico a pico) de 4 mm no estimador ingênuo e
6 mm no compensado**. Ambos pequenos porque, com a placa parada e sem
rotação, não há rotação para o compensado corrigir — este ensaio mede o
ruído de base do sistema, não a qualidade da compensação.

### Falha descoberta neste estágio: rejeição de amostra enviesa o acumulado

Uma tentativa de rejeitar amostras fisicamente implausíveis (ex.: um pico de
velocidade impossível para um veículo real) **diretamente no integrador**
causou um efeito colateral sério: descartar uma amostra de uma soma corrente
equivale a afirmar "a velocidade era zero naquele intervalo", e quando as
rejeições não são simétricas essa afirmação enviesa o total.

Medido em bancada, placa parada por 2 minutos:

| Configuração | Deriva acumulada |
|---|---|
| Estimador ingênuo, sem nenhum filtro | 2 mm |
| Estimador compensado **com** o filtro de plausibilidade | **21 mm** |

Esse resultado (o filtro "mais cuidadoso" produzindo *mais* erro que nenhum
filtro) motivou a remoção desse mecanismo do integrador e sua substituição
pelo Filtro de Kalman do estágio 3, onde uma amostra rejeitada não abre
buraco no acumulado — apenas deixa a predição do filtro de pé.

## 4. Estágio 3 — Filtro de Kalman (EKF)

**Método:** dois filtros de Kalman independentes de 2 estados (posição,
velocidade), um por eixo horizontal — os eixos não se acoplam nem na
dinâmica nem na medida, então dois filtros 2D equivalem matematicamente a um
único filtro 4D, sem a escrituração matricial extra. A predição usa a
aceleração linear do BNO08x (já com a gravidade removida pela fusão interna
do sensor, relatório `SH2_LINEAR_ACCELERATION`) rotacionada para o referencial
do mundo; a correção usa a velocidade produzida pelo estágio 2, com um *gate*
de consistência estatística (mesmo princípio do `FLOW_I_GATE` do ArduPilot)
para rejeitar medidas incompatíveis com a predição. Implementado em
`App/Src/flow_ekf.c`.

### O que o filtro entrega (teste sintético controlado)

Alimentado com o mesmo ruído de quantização do MTF-01 (±160 mm/s, ver tabela
da Seção 1.2, altura de referência 0,8 m):

| Grandeza | Sem filtro (amostra bruta) | Com EKF |
|---|---|---|
| Desvio de velocidade | 160 mm/s | **5,6 mm/s** (redução de **28,6×**) |
| Viés sob rejeição assimétrica | presente (ver Seção 3) | ausente |

### O que o filtro **não** entrega, e não tem como entregar

Posição não é observável apenas a partir de medidas de velocidade — o erro
de posição é a integral do erro de velocidade e cresce sem limite, com
filtro ou sem. No mesmo fluxo sintético usado para medir a redução de ruído
de velocidade, ao longo de 60 s de integração parada:

| Estimador | Deriva de posição em 60 s |
|---|---|
| Integrador simples (sem filtro) | 59,2 mm |
| EKF | 64,8 mm (**não é melhor**) |

Esse resultado é esperado e documentado no próprio código
(`App/Inc/flow_ekf.h`): o EKF melhora a estimativa de **velocidade**, que é o
que um controlador de voo consome; não elimina a deriva de posição de longo
prazo, que exigiria uma referência absoluta externa (GPS, marcadores visuais,
etc. — fora do escopo deste sensoriamento).

## 5. Estágio 4 — Calibração de eixo, sinal e escala (estado atual)

Nem o MTF-01 nem o BNO08x documentam publicamente a convenção de eixos ou o
fator de escala do fluxo relatado. Essa etapa determinou empiricamente, em
bancada, os três parâmetros que faltavam.

### 5.1. Pareamento de eixo (qual eixo do giroscópio cancela qual eixo do fluxo)

O modelo de livro-texto prevê pareamento cruzado (arfagem varre a imagem no
eixo X, rolagem no eixo Y) para uma câmera de eixos alinhados com a IMU.
Testes de bancada com rotação isolada por eixo mostraram que **não é isso que
esta placa mede**:

| Manobra isolada | Giro dominante | Resposta no fluxo | Consistência |
|---|---|---|---|
| Rolagem pura | `gyro_x` | inteira em `flow_x`, **sinal oposto** | 214/218 amostras (98,2%) |
| Arfagem pura | `gyro_y` | inteira em `flow_y`, **mesmo sinal** | 142/143 amostras (99,3%) |

Conclusão: o pareamento é **direto** (X com X, Y com Y), não cruzado — os
dois sensores parecem montados com uma rotação relativa de 90° nesta placa
específica. Além disso, o **sinal necessário para cancelar cada eixo é
diferente entre eles** (X precisa de +1, Y precisa de −1): uma rotação
relativa de 90° entre dois sensores inverte exatamente um dos dois eixos,
nunca os dois nem nenhum — o padrão observado é consistente com essa
explicação física.

### 5.2. Escala do fluxo

Duas abordagens foram testadas:

**(a) Deslocamento linear medido com referência física (régua/caixa).**
Depender de um humano empurrando um objeto e medindo a distância percorrida
mostrou-se pouco confiável:

| Ensaio | Distância real | Distância medida | Razão medido/real |
|---|---|---|---|
| Deslocamento lateral | 200 mm (medida exata, aresta de caixa) | 157,7 mm | 0,789 |
| Deslocamento frontal | ~200 mm (aproximado, "a olho") | 109,3 mm | 0,546 |

As duas razões não concordam entre si (0,79 vs 0,55) porque a incerteza de
"quanto o objeto realmente se moveu" é da mesma ordem de grandeza do erro que
se está tentando medir — o segundo ensaio, com distância apenas aproximada,
tem uma fonte de erro que o primeiro não tem.

Um resultado desse mesmo par de ensaios: o ângulo entre os dois vetores de
deslocamento medidos foi de **85,2°**, contra um valor físico esperado de 90°
(os dois movimentos foram feitos em direções aproximadamente perpendiculares).

**O que isso valida e o que não valida:** uma rotação constante aplicada por
engano preserva o ângulo *entre* duas trajetórias. Portanto esse resultado
mostra que a transformação preserva a geometria relativa — mas **não** que a
orientação absoluta está correta. Validar isso exige comparar os sentidos
estimados contra eixos físicos conhecidos e marcados, não apenas o ângulo
entre dois movimentos.

**(b) Calibração referenciada pelo giroscópio, sem régua** (técnica do
`AP_OpticalFlow_Calibrator.cpp` do ArduPilot, implementada em
`App/Src/flow_calibrator.c`): durante rotação pura, um sensor perfeitamente
calibrado leria exatamente a taxa que o giroscópio mede — o giroscópio serve
de referência precisa e independente, sem depender de nenhuma medida humana.

Resultado, após corrigir pareamento e sinal (Seção 5.1):

| Eixo | Fator de correção | Amostras usadas |
|---|---|---|
| X | 0,944 | 3239 |
| Y | 0,942 | 2215 |

Os dois eixos concordam entre si com **0,21% de diferença**, uma consistência
que os ensaios com régua não alcançaram.

**Ressalva importante sobre essa comparação:** a concordância entre X e Y
**não prova** que a régua estava errada. Os dois eixos foram medidos pelo
mesmo procedimento, no mesmo ensaio, e portanto podem compartilhar uma mesma
fonte de erro sistemático — atraso de sincronismo entre os sensores, ou o
centro de rotação da manobra não coincidir com o sensor (girar apoiado numa
borda desloca fisicamente o sensor, e esse deslocamento real entraria na
conta como se fosse erro de escala). A divergência entre ~0,79 (régua) e
0,944 (giroscópio) **permanece sem explicação fechada**.

O valor do giroscópio foi adotado como padrão por ter procedimento mais
controlado e concordância interna, mas fechar essa questão exige uma medição
com referência física controlada: guia com batentes de distância conhecida,
ambos os sentidos, múltiplas alturas e orientações de yaw conhecidas.

### 5.3. Validação de fechamento de percurso (o teste mais rigoroso)

Com o EKF já em uso, mas **antes** das correções de pareamento/sinal
(Seção 5.1) e da calibração de escala (Seção 5.2), foi executado um percurso
de três movimentos — deslocamento lateral de 200 mm, deslocamento frontal de
~200 mm, e retorno manual ao ponto de partida.

Duas características medidas desse ensaio determinam o que ele valida e o
que não valida: a rolagem e a arfagem variaram apenas **2,5° e 3,0°**
(amplitude pico a pico), enquanto a guinada (yaw) variou **49,8°**. Como a
compensação por giroscópio atua sobre rolagem e arfagem — justamente os
eixos que quase não se moveram — o pareamento/sinal ainda incorretos na
época tiveram efeito desprezível sobre este resultado. Já a rotação do
referencial do corpo para o do mundo, que depende do yaw, foi exercitada de
forma significativa. Portanto este ensaio valida principalmente **a
integração de translação e a rotação por yaw**, não a compensação de
rotação.

| Grandeza | Valor |
|---|---|
| Erro de fechamento medido pelo sistema | **51,2 mm** |
| Incerteza estimada do reposicionamento manual do operador | 80–100 mm |

O erro medido pelo sistema é **menor** que a própria incerteza de o operador
voltar de olho ao ponto de partida — ou seja, não é possível, com esse
protocolo de ensaio, afirmar que o sistema errou mais do que o próprio
método de referência permite verificar.

### 5.4. Limitação em aberto

Após aplicar as correções de pareamento, sinal e escala (Seções 5.1–5.2), um
novo ensaio de rotação isolada (rolagem seguida de arfagem, segurando a placa
no mesmo ponto, com rotações de até ~87°) mostrou resultado **parcial**.
Valores acumulados ao final do ensaio, com a placa novamente em repouso:

| | Eixo X | Eixo Y | Módulo |
|---|---|---|---|
| Sem compensação (ingênuo) | −954 mm | −918 mm | 1324 mm |
| Compensado | −47 mm | 303 mm | 307 mm |

Redução de **77%** no módulo do erro acumulado. Decompondo por eixo, porém, o
comportamento não é uniforme: o eixo X cancelou de forma forte e consistente
(de −954 mm para −47 mm, 95%), enquanto o eixo Y praticamente não cancelou e
ainda inverteu de sinal (de −918 mm para +303 mm).

A causa provável é que a manobra de arfagem não ficou tão isolada quanto a de
rolagem — durante a fase de arfagem, a rolagem chegou a variar até ~27°,
contaminando o canal que se queria medir. Isso impede concluir, com a mesma
confiança estatística da Seção 5.1, se o sinal do eixo Y está definitivamente
correto ou se o resíduo observado é apenas vazamento entre eixos.
**Este ponto permanece como trabalho em aberto** no momento da escrita deste
documento.

### 5.5. Limitação em aberto: acurácia do vetor de rotação do BNO08x

O BNO08x reporta, junto de cada amostra, um índice de acurácia de 0 a 3 para
cada relatório. Em **100% das 2717 amostras** analisadas dos logs recentes, o
vetor de rotação (`SH2_ROTATION_VECTOR`) reportou acurácia **0 = não
confiável**, enquanto o acelerômetro reportou 3 (máxima).

A **hipótese** mais provável é o magnetômetro não calibrado — o vetor de
rotação do BNO08x usa o magnetômetro para referência absoluta de guinada, e o
procedimento de calibração (movimento em figura de oito) não foi executado.
Isso **não está confirmado**: o índice reporta que a saída não é confiável,
mas não diz a causa. A CEVA distingue qualidade de relatório e procedimentos
de calibração no datasheet do BNO080/085; fechar esse diagnóstico exige
consultar essa documentação e executar a calibração para ver se o índice
sobe, não apenas assumir a causa.

**Impacto sobre este trabalho:** o caminho compensado e o EKF dependem do yaw
para rotacionar a velocidade do referencial do corpo para o do mundo. Um yaw
com acurácia declarada como não confiável é, em princípio, uma fonte de erro.
Na prática, porém, o yaw observado mostrou-se **estável** nos ensaios (variação
da ordem de 0,001° entre amostras consecutivas com a placa parada), e o teste
da Seção 5.2(a) mediu 85,2° entre dois movimentos fisicamente perpendiculares —
evidência de que a referência de yaw está funcional para uso *relativo* de
curto prazo, ainda que não calibrada em termos absolutos (norte magnético).

Para voo, recomenda-se executar a calibração do magnetômetro e verificar se o
índice sobe de 0, antes de confiar no yaw para navegação de longa duração.

## 6. Estágio 5 — Captura binária e replay em bancada real (2026-09-16)

Os logs de texto a 2 Hz não permitiam reproduzir um ensaio: mostravam só um
resumo, sem as amostras que o estimador de fato usou. Este estágio não altera
a matemática dos estimadores. Acrescenta uma **captura evento a evento** (cada
leitura do MTF-01 e do BNO08x, cada frame observado pela task e cada passo do
estimador, com seus identificadores) e um **replay** no PC que reexecuta as
mesmas funções C e confere se chega aos mesmos resultados. Formato e
procedimento: `docs/captura_replay_v3.md`.

Foram feitos dois ensaios com o firmware `build/Capture`, placa segurada à
mão, a cerca de 0,85 m da superfície abaixo do sensor.

| Arquivo | Duração | Situação | SHA-256 (prefixo) |
|---|---|---|---|
| `captures/ensaio01.bin` | 30 s | placa parada | `B633E863…` |
| `captures/ensaio02.bin` | 45 s | ida, pausa, volta e outro movimento | `11BCBAD2…` |

### 6.1. Resultado da infraestrutura (medido)

| Grandeza | Ensaio 01 | Ensaio 02 |
|---|---|---|
| Registros / bytes | 18 158 / 750 947 | 27 228 / 1 126 025 |
| Buracos de sequência, bytes corrompidos, descartes | 0 / 0 / 0 | 0 / 0 / 0 |
| Passos reproduzidos pelo replay | 3010 de 3010 | 4514 de 4514 |
| Divergências de pareamento / decisão / valor | 0 / 0 / 0 | 0 / 0 / 0 |
| Maior diferença numérica replay × placa | 4,1 × 10⁻⁸ | 1,0 × 10⁻⁷ |
| BNO online / distância válida | 100% / 100% | 100% / 100% |
| Menor heap livre (build Capture) | 2688 bytes | 2688 bytes |
| Menor margem de pilha por task | 1100–2972 bytes | idem |

Conclusão: a placa é **reproduzida exatamente no PC**. A partir daqui,
mudanças de algoritmo podem ser avaliadas sobre o mesmo movimento físico
gravado, sem repetir a bancada. A taxa de dados (~25 KB/s) é suportada pela
USB sem perdas.

### 6.2. Placa parada (ensaio 01)

Com a placa imóvel a 850–892 mm por 30 s, o maior afastamento da origem foi
**3,8 mm** nos três estimadores. É coerente com a referência anterior
(Seção 3, 6 mm pico a pico em 40 s): o ruído de base continua na ordem de
milímetros.

### 6.3. Movimento (ensaio 02)

A distância real percorrida **não foi medida** neste ensaio, portanto os
valores abaixo descrevem o que cada estimador calculou, não o seu erro.
Inclinação durante todo o ensaio abaixo de 2°; yaw variou 3,6°. Os
deslocamentos de cada trecho são projetados na direção da ida de cada
estimador (RAW integra em eixos do corpo e FUSED/EKF em eixos do mundo, então
só as magnitudes são comparáveis entre eles).

| Trecho | RAW | FUSED | EKF |
|---|---|---|---|
| Parado, 0–7 s (maior afastamento) | 10 mm | 10 mm | 10 mm |
| Ida, 7–13 s | +187 mm | +197 mm | +170 mm |
| Parado, 13–25 s (variação por eixo) | 10 mm | 9 mm | 9 mm |
| Volta, 25,5–29,5 s | −52 mm | −55 mm | −61 mm |
| Outro movimento, 29,5–33 s | +21 mm | +24 mm | +12 mm |
| Parado, 33–45 s (variação por eixo) | 3 mm | 3 mm | 3 mm |
| Distância final até a origem | 146 mm | 159 mm | 115 mm |

As variações de ~10 mm nos trechos parados correspondem a poucos degraus
isolados de fluxo (cada degrau soma ~1,7 mm por frame a essa altura).

Observações, todas conferidas nos registros brutos:

- **Resolução do fluxo confirmada em escala maior.** As 9032 leituras de
  fluxo (X e Y) caíram em múltiplos exatos de 20 na unidade do sensor, ou
  seja, degraus de 0,2 rad/s. A ~0,9 m um degrau vale ~180 mm/s. Durante a
  ida, **443 dos 601 frames leram zero**; na volta, o sensor registrou quase
  tantos degraus +1 (38) quanto −1 (57) no eixo dominante. Por isso a volta
  somou só 52–61 mm nos três estimadores, inclusive no RAW, que integra o
  fluxo diretamente: o limite está no que o sensor reporta a essa altura, e
  não no algoritmo.
- **A altura mudou durante o movimento** (540–985 mm; 558 mm por um instante
  na ida) e terminou em ~670 mm, contra 850 mm no início. Isso indica que algo
  passou sob o sensor ou que a placa terminou sobre outra superfície. Como a
  velocidade é fluxo × distância, uma distância errada escala o trecho
  correspondente.
- **O EKF rejeitou 6 medidas, todas durante movimentos** (8,05 s, 9,59 s,
  9,61 s, 26,99 s, 30,32 s e 31,03 s), e somou 14% menos que o FUSED na ida.
  Com os parâmetros atuais, o gate de inovação descarta parte do movimento
  rápido feito à mão.
- **Fluxo nos primeiros frames.** Os quatro primeiros frames da sessão
  (1328–1331) trouxeram 1–2 degraus de fluxo, e os dois primeiros passos
  resultaram em ~340–370 mm/s no FUSED. Não é possível dizer se foi movimento
  real no instante do início ou ruído. O EKF recém-iniciado aceitou essas
  medidas, a velocidade dele decaiu ao longo de dezenas de passos, e isso
  contribuiu para o afastamento de ~10 mm antes do movimento principal.

### 6.4. Estado ao fim desta fase

O subsistema de percepção (BNO08x + MTF-01) está funcional, estável e
reproduzível. **Não está comprovada a precisão contra uma distância
conhecida**: os ensaios com régua da Seção 5.2 foram inconsistentes, o
fechamento da Seção 5.3 ficou dentro da incerteza manual e o ensaio 02 não
teve referência. O próximo ensaio de validação deve usar marcas de distância
medida, altura de 15–25 cm (um degrau vale 30–50 mm/s) e superfície com
textura, voltando exatamente à marca inicial.

Continuam em aberto: compensação no eixo Y (Seção 5.4), acurácia do vetor de
rotação (Seção 5.5), a regra que zera o eixo com fluxo quantizado em zero
(questionada na revisão técnica), o ajuste do gate do EKF para movimentos
rápidos, a confiança do EKF nas primeiras medidas da sessão e o alinhamento
temporal entre giroscópio e fluxo (o estimador ainda usa a amostra mais
recente de cada sensor).

## 7. Tabela-resumo comparativa

Os valores desta tabela vêm de ensaios com condições diferentes entre si
(alturas, durações, se é simulação ou hardware) — comparações diretas entre
linhas só são válidas quando a condição de ensaio é a mesma, indicada na
coluna "Condição".

| Estágio | Métrica | Condição | Resultado |
|---|---|---|---|
| 1. Ingênuo | Erro em rotação pura | Sintético, 0,4 rad/s, 1 s | 400 mm inventados |
| 2. Compensado (BNO) | Erro em rotação pura | Sintético, 0,4 rad/s, 1 s | 0 mm |
| 2. Compensado (BNO) | Deriva parado | Bancada real, 40 s, 0,8 m | 6 mm |
| 2. Compensado + filtro de plausibilidade (revertido) | Deriva parado | Bancada real, 120 s, 0,8 m | 21 mm (pior que sem filtro) |
| 3. EKF | Desvio de velocidade | Sintético, ruído de quantização | 5,6 mm/s (28,6× melhor que bruto) |
| 3. EKF | Deriva de posição parado | Sintético, 60 s | 64,8 mm (não melhor que integrador simples: 59,2 mm) |
| 4. Calibração (régua) | Razão medido/real | Bancada real, ~200 mm, 2 ensaios | 0,79 e 0,55 (inconsistentes entre si) |
| 4. Calibração (giroscópio) | Fator de escala por eixo | Bancada real, milhares de amostras | X=0,944, Y=0,942 (0,21% entre si; divergência com a régua **em aberto**) |
| 4. Fechamento de percurso | Erro de fechamento | Bancada real, 3 movimentos ~200 mm cada, yaw variando 49,8° | 51,2 mm (< 80–100 mm de incerteza manual) |
| — | Acurácia do vetor de rotação (BNO) | Bancada real, 2717 amostras | 0 = não confiável em 100% (magnetômetro não calibrado) |
| 4. Pós-calibração completa | Redução do erro por rotação | Bancada real, rolagem+arfagem até ~87° | 77% no módulo (X: 95%; Y: não cancelou — em aberto) |
| 5. Captura e replay | Reprodução da placa no PC | Bancada real, 2 ensaios, 7524 passos | 100%, 0 divergências, maior diferença 1,0 × 10⁻⁷ |
| 5. Captura e replay | Perdas na captura USB | Bancada real, 75 s, ~25 KB/s | 0 registros perdidos |
| 5. Captura e replay | Deriva parado | Bancada real, 30 s, ~0,87 m | 3,8 mm (EKF) |
| 5. Captura e replay | Resolução do fluxo | Bancada real, 9032 leituras | 100% em degraus de 0,2 rad/s (~180 mm/s a 0,9 m) |
| 5. Captura e replay | Ida sem referência | Bancada real, ~0,85 m | RAW 187, FUSED 197, EKF 170 mm (distância real não medida) |

## 8. Referências técnicas

- ArduPilot `AP_NavEKF3_OptFlowFusion.cpp` e `AP_NavEKF3_Measurements.cpp` —
  compensação de taxa do corpo e gate de inovação para fusão de fluxo óptico.
- ArduPilot `AP_OpticalFlow_Calibrator.cpp` — calibração de escala do fluxo
  óptico referenciada pelo giroscópio.
- MicoAir, documentação do MTF-01 — protocolo Micolink. O piso de operação
  óptica adotado (~8 cm, contra ~2 cm do telêmetro) provém de documentação
  secundária e de guias da comunidade, não de especificação formal do
  fabricante; foi adotado por precaução.
- CEVA/Hillcrest, driver SH-2 (`github.com/ceva-dsp/sh2`) — protocolo SHTP e
  relatórios de sensor do BNO08x, incluindo `SH2_LINEAR_ACCELERATION`.

## 9. Rastreabilidade

Todo o código citado está em `App/Src/` e `App/Inc/` do repositório
`DroneController`. Os testes automatizados que reproduzem os resultados
sintéticos desta seção estão em `tests/test_displacement.c`,
`tests/test_flow_ekf.c` e `tests/test_flow_calibrator.c`, executáveis via
`gcc` sem necessidade do hardware (ver `README.md`, seção "Por onde começar a
ler o código"). Os resultados de bancada (Seções 1, 5.2, 5.3, 5.4) vêm de logs
de sessão registrados durante o desenvolvimento e foram recomputados a partir
dos valores brutos antes de entrarem neste documento.

Os resultados da Seção 6 vêm dos arquivos binários em `captures/`, que
guardam também os metadados da gravação (`*.bin.json`) e o manifest do
firmware (`*.bin.firmware.json`). Para recalcular:

```sh
python3 tools/decode_capture.py captures/ensaio02.bin --out-dir captures/ensaio02
gcc -std=c11 -IApp/Inc tools/replay.c App/Src/disp_estimator.c \
    App/Src/flow_ekf.c App/Src/attitude.c -lm -o /tmp/replay
/tmp/replay captures/ensaio02.bin
```
