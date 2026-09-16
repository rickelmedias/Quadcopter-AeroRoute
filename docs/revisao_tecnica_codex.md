# Revisão técnica — sensores, estimação e próximos passos

Data: 2026-09-15. Revisão do estado local, sem alterar o firmware.

> **Documento histórico.** O texto abaixo descreve o código de 2026-09-15.
> Situação de cada achado em 2026-09-16, conferida no código:
>
> | Achado | Situação |
> |---|---|
> | 1. Reinício do relógio do MTF | **Corrigido.** `disp_frame_gap_usable()` descarta o frame e reancora; teste em `tests/test_regressions.c` |
> | 2. Filtro sem inicialização | **Corrigido.** `flow_ekf_reset()` no início da task e autoinicialização do filtro; teste de regressão |
> | 3. Fluxo zero ≠ translação zero | **Em aberto.** A regra continua em `disp_estimator.c`; será avaliada com capturas |
> | 4. Estimativa congelada anunciada como válida | **Corrigido quanto à validade.** Idade, saúde e validade publicadas e atualizadas mesmo sem frame novo; convergência recalculada na predição; testes de regressão. A predição continua ocorrendo só a cada frame do MTF |
> | 5. DMA reenviando buffer | **Corrigido.** `dma_ring.h` compartilhado por MTF e DroneLink; testes de regressão |
> | 6. Canal SHTP sem limite | **Mitigado.** A biblioteca continua sem modificação; `bno08x.c` valida o canal antes de entregar o pacote a ela; teste de regressão |
> | 7. Amostras sem alinhamento temporal | **Parcial.** A captura preserva os tempos de medição do SH-2 e quais amostras cada passo usou; o estimador ainda usa a amostra mais recente |
> | 8. Calibração válida com uma amostra | **Em aberto.** `flow_cal_result()` ainda aceita qualquer número de amostras > 0 |
>
> Do plano recomendado, o item 2 (captura binária e replay) foi feito e
> validado em bancada: `docs/captura_replay_v3.md`. O item 3 (guia com
> distância conhecida) continua pendente. Estado geral:
> `docs/estado_atual_do_sistema.md`, Seção 8.

## Parecer

A separação entre aquisição, parser, matemática e publicação é boa. O BNO tem
recuperação de comunicação, o MTF tem parser testável e o projeto preserva
estimadores comparáveis. Entretanto, os resultados de bancada ainda não
validam a estimação para fechar uma malha de posição. Há defeitos concretos
na integração temporal, inicialização, validade e transporte, além de hipóteses
geométricas que precisam de ensaios independentes.

Não atribuo todos os problemas abaixo à última alteração: não há histórico Git
disponível neste checkout que permita separar autoria. Parte está em código
de infraestrutura anterior e parte na biblioteca SH-2 incorporada.

## Verificação executada

Os cinco executáveis de testes existentes passaram com GCC, `-Wall -Wextra
-Werror -fsanitize=address,undefined`: cabeçalho BNO, parser MTF, deslocamento,
Kalman e calibrador. Também executei `tmp/review_probe.c`, ligado às funções
reais do estimador. Comando reproduzível, a partir do Windows:

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/c/STM32/DroneController/tmp/run_review_tests.sh
```

Resultados adicionais:

```text
sensor_reboot dt_s=4294957.500 position_m=9.223e+11 velocity_m_s=4.295e+05
first_update reset_v=0.192308 boot_v=0.000000 boot_converged=1
no_updates_for_100s converged=1 sigma_v=1.593
zero_flow_plus_rotation accepted=1 compensated_expected=1 reported_v=0.000
```

Essas reproduções documentam defeitos existentes; não são testes de regressão
que exigem o comportamento corrigido. Não regravei a placa, não fiz ensaios
físicos nem gerei um novo binário ARM. Não encontrei os CSV/logs brutos dos
ensaios no projeto; portanto, não pude recalcular as estatísticas documentadas.

## Achados prioritários

### 1. Reinício do MTF vira aproximadamente 49,7 dias de integração — alta

Em `App/Src/displacement.c:120`, a subtração de timestamps é unsigned. O parser
aceita corretamente um reinício do sensor, mas a task não reinicia sua época
temporal. Uma passagem de 10000 ms para 10 ms produz 4294957,5 segundos.
O caminho FUSED rejeita o intervalo, mas RAW e `flow_ekf_predict` recebem o mesmo
tempo sem limite superior. Com aceleração de apenas 0,1 m/s², a reprodução
produziu posição de 9,223e11 metros.

Correção: identificar reinício/época, validar intervalo antes de qualquer
integração, reancorar relógios e invalidar a estimativa quando necessário.
Preservar a distinção entre reinício e wrap normal do contador. Limitar o
intervalo também na API matemática como defesa adicional.

### 2. O filtro não recebe a inicialização usada pelos testes — alta

`App/Src/displacement.c:16` declara o estado estático zerado. O único
`flow_ekf_reset` dessa integração está em `displacement_reset`, que não tem
chamador no caminho de boot. `displacement_start:223` só cria a task.
Assim, P11 começa em zero, embora `flow_ekf_reset` estabeleça P11=1.

Com uma observação inicial de 0,2 m/s a 0,8 m, o estado zerado aceita a
observação, continua em 0 m/s e marca convergência. O estado inicializado vai
para 0,192308 m/s. O primeiro predict real aumenta um pouco P11, mas não
corrige essa certeza inicial indevida. Inicializar na task antes de consumir
sensores e testar o caminho real de inicialização.

### 3. Fluxo zero não significa translação zero — alta

`App/Src/disp_estimator.c:86` força cada componente compensada a zero quando o
fluxo bruto está abaixo de meio passo. Isso contradiz a própria equação do
modelo: translação e rotação podem produzir fluxos opostos e se cancelar.
No exemplo de um eixo, fluxo=0, gyro=1 rad/s, sinal=+1 e distância=1 m devem
deixar uma componente compensada de 1 m/s pelo modelo usado; o código retorna
zero. O problema não exige assumir um erro de montagem.

Uma leitura quantizada representa incerteza, não certeza de repouso. O teste
atual `test_quantized_flow_does_not_invent_travel` inclusive exige zero com
gyro=0,9 rad/s, cristalizando essa hipótese errada. Substituir a regra por
tratamento estatístico da quantização e adicionar casos de rotação+translação
simultâneas. Uma restrição de velocidade zero só deve entrar com evidência
independente de repouso; aceleração zero também ocorre em velocidade constante.

### 4. Estimativa congelada pode continuar anunciada como convergida — alta

`App/Src/displacement.c:110` condiciona toda a predição e publicação à chegada
de um novo frame MTF. Se o MTF parar, o estado fica congelado, inclusive sua
incerteza e sinalização. `displacement_get` não verifica idade. Separadamente,
`App/Src/flow_ekf.c:63` não recalcula `converged` durante a predição: após
convergir, 100 s de predição sem correção deixaram `converged=1` com sigma de
velocidade de 1,593 m/s, acima do critério nominal de 0,3 m/s.

Predizer por novas amostras IMU, corrigir por novas amostras MTF e publicar
idade, validade e motivo de degradação. A validade deve considerar tempo sem
correção, incerteza e saúde dos sensores; convergência estatística sozinha
não equivale a disponibilidade para controle.

### 5. DMA pode reenviar um buffer inteiro no evento IDLE após TC — alta

`App/Src/mtf01.c:30` e `App/Src/dronelink/dronelink_transport.c` normalizam
a posição final N para zero. A HAL local permite TC(N) seguido de IDLE(N)
sem bytes novos, explicitamente em
`Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2539`.
Depois do TC, last=0; o IDLE(N) envia novamente N bytes antigos.

Isso pode duplicar comandos/frames e contaminar timestamps do MTF. Seu parser
descarta repetição da sequência imediatamente anterior, mas um bloco repetido
com múltiplos frames não é uma única duplicata: contém sequências e tempos
antigos, podendo parecer reinício. Não afirmo que isso explique todos os
`link_rx_drop` observados; esse contador também soma erros UART.

Conservar o cursor N até o próximo evento e testar HT, TC, IDLE, coincidências
e wrap; considerar a ordem dos eventos DMA/UART. Este achado é por inspeção
do caminho HAL/callback, sem reprodução em hardware nesta revisão.

### 6. Biblioteca SHTP acessa canal antes de validar limite — alta

`App/Src/sh2/shtp.c:201` lê `pShtp->chan[chan]` antes do teste
`chan >= SHTP_MAX_CHANS`. Um cabeçalho corrompido com canal fora da faixa
pode acessar memória fora do array. A validação de igualdade entre os dois
cabeçalhos no transporte não garante que o canal seja válido.

Mover o teste de limite antes do primeiro acesso e testar cabeçalhos
malformados. Registrar a correção local ao upstream. É um defeito da
biblioteca incorporada, não evidência de que causou o problema original BNO.

### 7. Amostras não estão alinhadas no tempo e podem ser puladas — média/alta

`App/Src/displacement.c:104` lê snapshots recentes dos sensores; isso não
garante que gyro, quaternion, aceleração e fluxo sejam do mesmo instante.
O BNO decodifica timestamps SH-2, mas não os preserva no snapshot de cada
medição. Se chegarem vários frames MTF antes da task rodar, ela usa só o
último, multiplicado pelo tempo total. Em movimento variável, isso altera
a integral e a compensação; contar frames recebidos não conta frames usados.

Usar filas limitadas com timestamps, contador de perdas no estimador e
histórico IMU para alinhar as medidas. Calibrar também o atraso relativo,
antes de concluir que diferenças são erro de escala.

### 8. Calibração válida pode significar apenas uma amostra — média

`App/Src/flow_calibrator.c:35` exige amostras>0 e escala dentro de limites,
mas não exige qualidade de ajuste posterior nem melhora sobre escala=1.
O residual publicado é anterior ao ajuste. A semelhança entre escalas X/Y
não mede erro em relação ao deslocamento verdadeiro.

A implementação reproduz parte da média ponderada do ArduPilot, sob hipótese
de translação nula. O [calibrador original](https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_OpticalFlow/AP_OpticalFlow_Calibrator.cpp)
também usa amostras com predição de movimento, conjunto de amostras completo,
fitness posterior e comparação com o ajuste original. Logo, não há equivalência
completa de validação.

Exigir amostras suficientes em ambos os sentidos, validação em captura
separada, residual antes/depois, repetibilidade e sincronização. Não descartar
a referência de distância física apenas porque contradiz a escala do gyro.

## Geometria: hipótese importante ainda por validar

O código comenta uma rotação relativa de 90° entre sensores para justificar
pareamento gyro/fluxo no mesmo eixo. Entretanto, transforma essas componentes
diretamente como velocidades nos eixos do BNO e só aplica yaw. A aceleração
usa a rotação completa do quaternion. Correlação durante rotação não estabelece
por si só o mapeamento das componentes de translação.

Definir explicitamente sensor MTF → corpo → mundo, incluindo sinais e eixo
óptico. Testar translação pura nos dois eixos, frente/trás e esquerda/direita,
com yaw conhecido em 0°, 90°, 180° e 270°. Depois testar inclinação mantendo
trajetória horizontal. Aplicar só yaw é uma aproximação de voo nivelado,
não uma projeção geral para os ~37° atualmente aceitos.

Incluir distância do sensor ao centro de rotação: girar a montagem em torno
de uma borda produz velocidade real no sensor (`omega × r`). Não é correto
atribuir todo residual à calibração óptica. Fixar fisicamente um eixo próximo
do centro óptico ou medir e modelar esse braço de alavanca.

## Documentação e conclusões experimentais

- README:133 ainda chama 4/6 mm de deriva; a tabela em docs:349 também mantém
  6 mm como deriva. Se são amplitude pico a pico, corrigir todos os locais.
- README:324 diz que não há EKF, contradizendo o código atual. Tecnicamente,
  `flow_ekf.c` implementa um Kalman linear de posição/velocidade com a medida
  não linear pré-processada fora dele. Não é o EKF completo de navegação do
  ArduPilot; isso não é defeito, mas muda o que a covariância representa.
- A melhora de 28,6× vem de um sinal sintético alternando exatamente +passo
  e -passo, com descartes pseudoaleatórios. É um teste útil e favorável à
  suavização, não uma medida de precisão dinâmica real. Acrescentar rampas,
  reversões, viés IMU, atraso e erro de posição contra referência independente.
- 85,2° entre trajetórias quase perpendiculares não valida sozinho yaw ou
  orientação absoluta. Uma rotação constante errada preserva ângulos entre
  vetores. É necessário comparar o sentido estimado com eixos físicos conhecidos.
- 0,944 e 0,942 concordarem em 0,21% não prova acurácia de 0,21%. Ambos podem
  compartilhar o mesmo viés, atraso ou erro de geometria.
- RAW acumula em eixos móveis e FUSED em eixos fixos. Comparações por eixo
  precisam de referencial comum e comparação apenas dos mesmos intervalos
  válidos. Rejeição por inclinação também pode reduzir artificialmente o
  deslocamento integrado em testes de rotação grande.
- `reads=packets` não demonstra ausência de perda de amostras. Verificar
  sequências, timestamps e cadência de cada relatório.
- O tempo `probe_ms` começa depois dos 20 ms iniciais de espera. Não é o
  tempo total desde soltar reset. A narrativa de falha única em 20 ms deve
  identificar a versão a que se refere: logs anteriores desta conversa já
  mostravam ACK e leitura de cabeçalhos, portanto não eram todos a mesma falha.
- A documentação deve separar resolução observada em uma captura, unidade
  do protocolo e velocidade mínima detectável. A [MicoAir](https://micoair.com/docs/decoding-micolink-messages-from-mtf-01/)
  especifica cm/s a 1 m; isso não prova um piso universal de 0,2 rad/s a partir
  de 34 amostras. Quantização por amostra não impede necessariamente detectar
  movimentos menores em uma janela temporal.
- `acc=0` informa saída não confiável; não diagnostica sozinho magnetômetro
  não calibrado. O [datasheet CEVA, seções 3.1–3.4](https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf)
  descreve status e calibração. Conferir status por relatório e persistência
  da calibração. A hipótese de problema magnético merece ensaio, não certeza
  causal baseada apenas nesse campo.
- Guardar logs brutos, versão/configuração do firmware, fases do ensaio e
  script de análise junto de cada resultado. Hoje não consegui reproduzir
  os números de bancada a partir dos arquivos disponíveis.

## Outras melhorias de código

- `displacement_reset` escreve estados que a task modifica fora da seção
  crítica; uma chamada por outra task pode interromper uma atualização no
  meio. Preferir comandos processados pela task proprietária do estado.
- `displacement_set_ekf_config` guarda um ponteiro externo sem copiar ou
  validar os valores. Copiar uma configuração validada evita ponteiro para
  variável local já destruída e alterações concorrentes.
- O logger não incrementa `dropped_logs` quando snprintf ultrapassa o buffer,
  embora descarte a mensagem. Contabilizar truncamento separadamente de USB
  ocupado/desconectado; proteger conversão float→inteiro contra NaN/overflow.
- No protocolo, a supervisão marca HOLD/LAND como solicitado mesmo se a API
  rejeitar a solicitação. Ao substituir os stubs, tratar resultado, estado
  efetivamente atingido e política de repetição/escalonamento.
- `flight_get_snapshot` continua sendo stub. Os sensores funcionarem no log
  não significa que validade/posição/atitude já chegam ao protocolo de voo.

## Plano recomendado

1. Corrigir tempo, inicialização, validade, DMA e limite SHTP. Transformar os
   casos desta revisão em regressões e testar desconexão/reinício dos sensores.
2. Criar captura binária/CSV por amostra e replay no PC usando as mesmas funções
   C. Incluir timestamps, configurações, perdas e motivos de rejeição. Isso
   permite comparar mudanças com exatamente o mesmo movimento físico.
3. Fechar geometria X/Y, atraso e escala com guia linear, batentes de distância
   conhecida, três alturas e repetições nos dois sentidos. Reservar uma captura
   só para validação; não ajustar parâmetros nela.
4. Calibrar BNO, registrar qualidade e medir latência/cadência. Comparar yaw
   relativo e magnético em bancada. O Game Rotation Vector é uma alternativa
   a experimentar em campo magnético instável, com deriva de yaw como troca,
   conforme a CEVA; não trocar sem validar referenciais e efeitos no estimador.
5. Desenvolver acionamento de motores sem hélices em paralelo aos ensaios,
   com máquina de estados de armação, timeout de comando e saída segura.
   Escolher DShot conforme ESC real e mapear timers/DMA disponíveis.
6. Fechar primeiro controle de taxa angular e atitude com tempos medidos e
   limites. Um PID serve de referência experimental; LQR pode ser comparado
   depois sobre o mesmo modelo/ensaios. Não deixar um eixo de estimação sem
   validação para descobri-lo em voo com controle de posição.
7. Repetir aquisição com motores em bancada: analisar também espectro de
   vibração, atraso, saturação e campo magnético, além do deslocamento parado.
   Não escolher espuma arbitrariamente: a montagem muda ressonâncias e o
   alinhamento relativo dos sensores deve continuar conhecido.
8. Só então fechar velocidade e posição. Para posição com erro limitado ao
   longo do tempo, planejar referência externa apropriada ao cenário. Para
   trajetos curtos, quantificar o tempo/distância em que a odometria atende
   à tolerância da missão antes de adicionar complexidade.

A ideia de maior retorno imediato é uma bancada reproduzível com replay,
não aumentar o tamanho do filtro. Depois disso, estimar viés de aceleração
e modelar melhor incerteza de fluxo/gyro/altura pode valer a pena, se os logs
mostrarem que são as limitações dominantes.
