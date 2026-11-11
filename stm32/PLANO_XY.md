# Plano de estabilização XY — 08/11/2026

Atualização: a revisão dos kits posteriores 180436 e 181108 está em [ANALISE_LOGS_DRONE.md](ANALISE_LOGS_DRONE.md). A compensação cruzada de giroscópio já está implementada no código atual; as suspeitas anteriores sobre essa associação precisam ser lidas à luz dessa revisão. Ela também registra as correções locais do limitador de yaw e o diagnóstico adicional de Z, ainda não gravados no STM32.

Escopo: manter LQG/Kalman; revisar a cadeia de medição, estimativa e atuação antes de aumentar ganhos no drone. Base: código atual em `hover-lqg-pi` e kits 172447, 172704, 172833, 173119. Os ensaios tiveram intervenção manual sem marcação temporal. Não estimar estabilidade ou constante de tempo do drone livre a partir deles.

## O que os dados sustentam

- Os quatro kits usam XY=1 e FLOW_SWAP=1. Nos três primeiros FF_TH=-45 e W0=850/825/805; no último FF_TH=-35 e W0=900. FF_PSI=25.
- Nenhum registrou chegada ao alvo. Os três primeiros pediram pouso aos ~26 s por não atingir 15 cm; o último teve failsafe em 21,92 s. Duração de registro não equivale a duração de hover livre.
- Em 172833, o relatório calcula `lifted` aos 19,69 s, mas o texto do STM32 registra `XY: ligado` cerca de 5,77 s depois da decolagem. São critérios/origens distintos: não usar `phases_s.lifted` como instante exato de ativação de XY. A menor excursão XY desse ensaio não comprova melhor malha.
- No kit 173119, FLOW_GATE, FLOW_CENS, CTL_PSI_MAX e FLT_RAMP_CM estão em zero. Há implementações dessas funções no código, mas não estavam habilitadas nesse ensaio.
- Fluxo marcado válido significa amostra aceita; não demonstra escala, sinal, ausência de viés ou independência das amostras.
- Correlação de θ com x local e φ com y local não comprova o sentido da malha quando o yaw varia. O controlador trabalha com erros horizontais girados para a proa. Intervenção manual acrescenta outra causa aos ângulos.

## Ordem de trabalho e critérios de conclusão

### Prioridade conjunta: Z durante a decolagem e sua interação com XY

O objetivo é corrigir x/y enquanto a referência de altura sobe. Não é necessário terminar a subida para controlar posição. A implementação atual, porém, só libera XY após detectar desprendimento; logo parte da subida/partida pode ocorrer sem freio horizontal. Também não se deve ajustar XY supondo hover estável quando a altura continua muito abaixo do alvo.

Reanálise do `real_log.csv`, apenas TAKEOFF/HOLD/LEG, janela 5–15 s após a primeira amostra TAKEOFF. Altura medida = chão da Pi menos y_z; referência = mesmo chão menos ref_z. Valores arredondados. Há apoio manual não marcado nos kits.

| Kit | W0 | Altura medida mediana | Referência mediana | Tendência da altura medida | Subida estimada pelo Kalman (média -vz) | Comando médio dos motores |
|---|---:|---:|---:|---:|---:|---:|
| 172447 | 850 | 5,4 cm | 14,7 cm | -0,02 cm/s | +2,22 cm/s | 50,1% |
| 172704 | 825 | 2,7 cm | 14,7 cm | +0,13 cm/s | +2,52 cm/s | 49,3% |
| 172833 | 805 | 1,7 cm | 14,4 cm | +0,04 cm/s | +2,75 cm/s | 48,5% |
| 173119 | 900 | 2,5 cm | 14,8 cm | -0,02 cm/s | +2,60 cm/s | 53,7% |

A tendência medida é regressão linear da altura contra tempo, não derivada ruidosa de duas amostras. Os quatro intervalos tiveram 0% de flag de saturação; o maior comando individual nesses intervalos ficou entre 55,1% e 64,9%, abaixo de MOT_MAX=95%. Isso não mede empuxo real nem exclui limitação física do motor. Os valores de 15 cm são relativos ao chão calibrado pelo firmware; o CSV usa também a calibração independente da Pi, daí diferenças de alguns milímetros.

**Achado:** a referência sobe, mas a altura observada não a acompanha. O filtro prediz subida contínua incompatível com a tendência da distância. O termo de velocidade vertical pode então diminuir o pedido de subida antes de o drone subir de fato. Não atribuir automaticamente a W0 baixo: mão, contato com o chão, efeito solo, curva de motor, modelo vertical e tratamento da perturbação também entram nessa comparação.

Auditar nesta ordem:

1. `flight_ctrl.c`: confrontar `ref_z`, `y_z`, `xhat_z`, `xhat_vz`, `meas_z_smooth`/`meas_vz` e cada contribuição do comando coletivo. Verificar se o filtro precisa representar explicitamente a perturbação vertical que já é estimada; evitar dois estimadores com predições incompatíveis. Preservar Kalman/LQG e corrigir a consistência do modelo antes de substituir vz por uma derivada de sensor.
2. `update_dist()`: a liberação ocorre quando altura filtrada passa `TRIM_RELEASE_HEIGHT_M` (1 cm); `dist_released` permanece ativo. Esse limiar pode ser atingido por mão ou variação do telêmetro. Ao liberar, a rampa fixa de incremento de hover dá lugar à estimação. Nos textos d_z sobe lentamente: aproximadamente 6→91 no kit 172447 e 3→100 no kit 173119. Conferir o instante de liberação e a compatibilidade de entrada, FF, distúrbio e predição.
3. `tools/lqg_ajustado.py`, `lqg_dist_kalman_input()` e `compute_output()`: o modelo vertical foi escalado para um hover assumido próximo de 1000, enquanto W0 real de teste variou 805–900. W0 define comando de base na curva provisória; não é uma medição de velocidade dos motores. Reconstituir W0 + du coletivo + FF/estimador e separar erro de base, adaptação lenta e erro de ganho da planta.
4. Interação física: o componente vertical do empuxo cai com a inclinação; corrigir XY exige inclinar, sem perder sustentação. Perto do chão, contato, efeito solo e fluxo variam; esses ensaios não representam uma planta livre no ponto de hover. O sensor usa distância ao piso para escalar fluxo, incluindo a altura de montagem; não substituir essa distância apenas pela altura de subida.
5. Telemetria: publicar o chão do firmware e os eventos reais `lifted`, `xy_engaged` e `dist_released`, além da altura independente da Pi. O kit atual não permite assumir que o `lifted` calculado pelo relatório reproduz o estado interno. Separar máximo antes de pouso/failsafe dos picos durante contato/queda.

Critério para avançar a ajuste fino de XY: demonstrar subida sustentada sem apoio e estabilização próxima da referência, com estimativa de vz coerente com a evolução da altura, enquanto XY permanece ativo depois do desprendimento. Durante a revisão offline, testar Z e XY juntos: rampa de Z com erro/velocidade inicial horizontal, yaw variável, atraso dos motores e erro de empuxo de base. Não exigir controlar Z primeiro com XY desabilitado no voo livre; isso recriaria o afastamento já observado.

### 1. Corrigir/verificar a compensação angular com FLOW_SWAP

Arquivo principal: `App/Src/control/meas.c`; testes: `tests/test_meas.c`.

Atualmente trans_x recebe gyro[0] e trans_y recebe gyro[1]; só depois o código troca para frente=trans_y e direita=-trans_x. Pelo próprio mapeamento BNO do firmware, gyro[0] está associado a pitch e gyro[1] a roll. Assim, com swap=1, o giro de roll contamina o canal de avanço. Corrigir a transformação de fluxo e giroscópio em referenciais explicitamente definidos, incluindo sinais e braço do sensor; não apenas trocar dois índices sem conferir a geometria.

O teste existente de swap usa gyro[0]=gyro[1]=0: verifica translação, mas não essa compensação. Criar casos físicos de rotação sem translação, com pitch, roll e yaw, para as duas orientações do sensor. Critério: velocidade do centro próxima de zero depois da compensação, com residual explicado pela discretização e pelo braço do sensor. Reutilizar primeiro os ensaios manuais existentes.

### 2. Verificar a invariância à orientação e a guinada

Arquivos: `meas.c`, `attitude_mode.c`, `flight_ctrl.c`, `lqg_dist.c`.

O código gira fluxo para o referencial local e gira medidas/estados/referências para a proa ao calcular o LQG. Conferir erro e velocidade na proa antes de comparar com φ/θ. Um yaw real variando não deveria, por si só, inverter a correção XY se todas essas transformações estiverem certas.

Critério: o mesmo erro local, com yaw 0°, 90°, 180° e 270°, deve produzir aceleração corretiva local equivalente. Conferir também integração durante yaw variável e períodos sem fluxo. No simulador, a planta linear assume orientação próxima da origem; verificar se ela representa corretamente aceleração horizontal com yaw diferente de zero antes de usá-la para validar esses casos.

Reconstituir separadamente comando de yaw do LQG, FF_PSI, estimador de perturbação, eventual limitação e comando final. Não reduzir Qpsi/Qr apenas porque houve deriva: primeiro distinguir falta de autoridade, sinal, oscilação e erro de modelo. CTL_PSI_MAX=0 desabilita o limitador no ensaio analisado.

### 3. Medir autoridade XY nos ganhos reais e depois da saída

Arquivos: `tools/lqg_ajustado.py`, `tools/pi/dronectl/hover_test.py`, `flight_ctrl.c`, `throttle.c`.

Ganhos recalculados localmente, sem UART:

| Grandeza na primeira entrada | Atual | Qxy=250, Qvxy=60 |
|---|---:|---:|
| módulo Kx | 9,9298 | 22,1407 |
| módulo Kvx | 18,8126 | 29,7917 |
| Ktheta | 155,2429 | 170,4670 |
| Kq | 22,9525 | 23,5970 |
| ângulo equivalente estático para erro de 10 cm | 0,366° | 0,744° |

O ângulo equivalente é Kx/Ktheta vezes o erro, desprezando velocidade, taxa angular, FF e perturbações; não é um setpoint explícito nem previsão de voo. Confirma autoridade posicional pequena, mas também mostra que aumentar Qxy altera atitude. Comparar números de Q de metros e radianos diretamente não mede autoridade.

Decompor -Kd*xhat + Kfd*ref em posição, velocidade, atitude e taxas; reconstruir FF/perturbação e diferenças de motor depois da rampa e dessaturação. Conferir se o comando publicado é anterior às limitações. Critério: quantificar o pedido XY e quanto chega à saída, em vez de exigir correlação >0,4.

Antes de adotar Q maior, verificar resposta e estabilidade com atraso dos motores e incerteza da planta. O próprio `lqg_ajustado.py` registra perda de estabilidade para alguns pesos maiores com atraso de 120 ms. Essa anotação exige reprodução; não prova que o novo conjunto falha, mas impede tratá-lo como uma alteração apenas de posição.

### 4. Corrigir o que acontece antes de XY entrar

Arquivo: `flight_ctrl.c`.

XY entra depois de `lifted`, e o código zera x/y/vx/vy e a referência nesse instante. O ponto assim escolhido pode já ter se afastado do ponto físico onde começou a partida. Até essa entrada, a telemetria da malha mascara velocidades horizontais. Portanto não prometer segurar o ponto original durante toda a decolagem com a implementação atual.

Critério: registrar momento da entrada, deslocamento observado e velocidade existente; escolher uma origem explícita e uma transição sem apagar velocidade real. Só usar fluxo antes desse instante quando sua validade e compensação forem demonstradas. Registrar medidas originais separadas das substituídas pelo modo de controle.

### 5. Separar erro de posição estimada, nível e erro de modelo

Posição vem de fluxo integrado, sem medida absoluta. Um viés de 1 cm/s acumula 10 cm em 10 s. Em aproximação estática, os ganhos atuais deixam ~13,6 cm de erro para compensar 0,5° de erro de nível. Aumentar ganho pode reduzir esse desvio, mas não corrige um sensor que registra posição errada.

Conferir escala em deslocamento conhecido, zero de nível, braço de yaw FLOW_YAW_X/Y, idade de amostra e sincronização MTF/BNO. O driver aceita amostra MTF até 250 ms de idade: analisar repetição e atraso antes de alterar o filtro. Conferir necessidade de covariâncias adequadas e atualização com amostras novas, mantendo Kalman.

Critério: comparar posição estimada com marcações externas/medida independente e identificar trechos sem intervenção. Não tratar repetidas amostras aceitas como prova de fluxo perfeito. Massa de 551 g não fornece inércia, curva DShot/empuxo ou atraso do motor. A sustentação a 4–6 cm e com apoio manual não identifica W0 de hover livre.

## Sequência de validação

1. Resolver compensação angular e referenciais com testes determinísticos e dados já disponíveis.
2. Reproduzir os kits, decompor esforço XY/yaw e testar candidatos Q com atrasos e perturbações. Documentar limites da simulação.
3. Registrar no kit origem XY, medidas antes da máscara, idade/qualidade do fluxo, esforço por canal e motivo de limitação; marcar intervenções do operador.
4. Só então definir um ensaio físico curto que compare uma hipótese, com altura sustentada e medida independente de posição. Não usar segurar/soltar com a mão entre hélices em rotação como método de identificação.

Entrega pretendida: correções localizadas e testes que falhem antes delas, mantendo LQG/Kalman, seguidos de um candidato de configuração justificado. O comando `q-pos-forte` é candidato para avaliação offline, não uma configuração aprovada por esta revisão.
