/**
 * @file flight_config.h
 * @brief Chave de segurança do voo e valores padrão dos parâmetros.
 *
 * O LQG (control/lqg_ert_model.h) usa o modelo do TCC em eixos NED: x para a
 * frente, y para a direita, z para baixo; rolagem positiva abaixa o lado
 * direito, arfagem positiva levanta o nariz, guinada positiva gira no sentido
 * horário visto de cima. Os sinais abaixo levam as leituras dos sensores para
 * essa convenção. CONFIRA CADA UM NA BANCADA antes de voar (roteiro em
 * docs/controle_lqg.md).
 *
 * Os valores abaixo são só o ponto de partida: todos podem ser alterados em
 * tempo de execução (control/ctrl_params.h), sem regravar o firmware.
 */
#ifndef FLIGHT_CONFIG_H
#define FLIGHT_CONFIG_H

/**
 * @brief 0 = só ensaio a seco: o LQG roda e a telemetria sai, mas os motores
 *        nunca recebem o comando do controle, qualquer que seja FLT_DRY_RUN.
 *        1 = com FLT_DRY_RUN = 0, os motores seguem o LQG.
 *
 * Mudar para 1 é uma decisão consciente, com a curva do motor medida, os
 * sinais conferidos e o drone preso.
 *
 * Branch controle: 1, para testar o controle. Mesmo assim o firmware sobe com
 * FLT_DRY_RUN = 1 (motores parados) até a interface pedir o contrário.
 */
#define LQG_FLIGHT_ENABLED 1

/* ---- Motores ------------------------------------------------------------ */

/**
 * Rotação de hover ω0, em rad/s, na curva provisória MOT_W_AT_0/100 (ω =
 * aceleração × 1755). O TCC usa 702 (b_z = g / 2ω0); com o chumbo (550 g) os
 * voos de 06/11 pediram 870–940 perto do chão e 1170–1270 a 10–50 cm, já com a
 * bateria gasta (CTL_W0 + d̂ de altura no fim de cada voo). 1050 fica um pouco
 * abaixo do hover com bateria cheia: o estimador sobe o resto no chão
 * (TRIM_GROUND_RATE) e no ar. A ferramenta da Pi sugere o próximo valor.
 */
#define DEFAULT_CTL_W0 1050.0F

/**
 * Curva rotação -> aceleração DShot, linear entre os dois pontos.
 * VALORES PROVISÓRIOS: escolhidos para que ω0 caia em ~40% de aceleração.
 * Substitua pelos pontos medidos no dinamômetro (ou ajuste MOT_W_AT_100 em
 * voo até o hover ficar parado na altura pedida).
 */
#define DEFAULT_MOT_W_AT_0   0.0F
/** Rotação em 100% de aceleração, em rad/s (ver DEFAULT_MOT_W_AT_0). */
#define DEFAULT_MOT_W_AT_100 1755.0F

/** Aceleração mínima com o controle ligado (fração 0..1). */
#define DEFAULT_MOT_MIN  0.03F
/**
 * Aceleração máxima com o controle ligado (fração 0..1). Era 0,8; no voo de
 * 50 cm de 06/11 três motores ficaram em ~75% no hover, sem folga para o
 * controle com 0,8.
 */
#define DEFAULT_MOT_MAX  0.95F
/** Aceleração durante a partida de decolagem (fração 0..1); ARMADO envia zero. */
#define DEFAULT_MOT_IDLE 0.05F

/* ---- Zero (nível) do BNO08x -------------------------------------------
   O BNO está montado de cabeça para baixo por exigência da montagem: com o
   drone nivelado no chão ele lê roll ~ -174,5 graus e pitch ~ -2,9 graus.
   Esses valores são o zero: o controle usa roll e pitch medidos em relação a
   eles, então o drone nivelado vale 0. A guinada não é zerada (referência
   ao norte). Para recalibrar, leia roll e pitch crus com o drone nivelado
   (log EST/BNO) e ajuste ZERO_ROLL e ZERO_PITCH em tempo de execução. */
/** Roll cru lido com o drone nivelado, em graus. */
#define DEFAULT_ZERO_ROLL_DEG  (-174.5F)
/** Pitch cru lido com o drone nivelado, em graus. */
#define DEFAULT_ZERO_PITCH_DEG (-2.9F)
/**
 * Correção fina do nível, somada a φ e θ já no referencial do modelo, em graus.
 * O zero acima vem do drone parado no chão; o nível que importa é o do empuxo
 * no ar (frame e motores podem estar um pouco tortos). Sem integrador de
 * posição, 0,5° de erro deixa o drone ~14 cm fora do ponto: a ferramenta da Pi
 * mede, num hover parado, a inclinação média que o segura e sugere a correção.
 */
#define DEFAULT_LVL_PHI_DEG    0.0F
/** Correção fina do nível em θ, em graus (ver ::DEFAULT_LVL_PHI_DEG). */
#define DEFAULT_LVL_TH_DEG     0.0F

/* ---- Sinais dos sensores para NED ---------------------------------------
   O BNO está de cabeça para baixo e com o eixo x para o lado do drone, então
   o pitch do sensor é a rolagem φ do modelo e o roll do sensor é a arfagem θ
   (ver control/meas.h). Medido na bancada, com o zero de nível aplicado:
   - aumentar M2 e M4 (frente) e diminuir M1 e M3 faz o roll do sensor subir,
     e o nariz sobe: θ = +roll do sensor;
   - aumentar M1 e M2 (direita) e diminuir M3 e M4 faz o pitch do sensor
     descer, e o lado direito sobe (φ negativo): φ = +pitch do sensor;
   - girar para a esquerda (anti-horário visto de cima) aumenta a guinada do
     sensor, e ψ do modelo cresce no sentido horário: ψ = -guinada do sensor.
   Os giros seguem a cinemática de Euler com o roll do sensor perto de 180
   graus: p = -giro y, q = +giro x, r = +giro z. Confirme os giros na
   bancada (roteiro em docs/controle_lqg.md). */
/** Sinal de φ (vem do pitch do BNO08x). */
#define DEFAULT_SGN_ROLL  (+1.0F)
/** Sinal de θ (vem do roll do BNO08x). */
#define DEFAULT_SGN_PITCH (+1.0F)
/** Sinal da guinada do BNO08x. */
#define DEFAULT_SGN_YAW   (-1.0F)
/** Sinal de p (vem do giro y do BNO08x). */
#define DEFAULT_SGN_P     (-1.0F)
/** Sinal de q (vem do giro x do BNO08x). */
#define DEFAULT_SGN_Q     (+1.0F)
/** Sinal de r (vem do giro z do BNO08x). */
#define DEFAULT_SGN_R     (+1.0F)
/** Sinal da velocidade x do fluxo (já girada pela guinada) para NED. */
#define DEFAULT_SGN_VX    (+1.0F)
/** Sinal da velocidade y do fluxo para NED. */
#define DEFAULT_SGN_VY    (+1.0F)

/**
 * Escala do fluxo óptico do MTF-01, medida contra o giroscópio na bancada
 * (0,944 em X e 0,942 em Y, ver docs/acompanhamento_tecnico.md).
 */
#define DEFAULT_FLOW_SCALE_X 0.944F
/** Escala do fluxo em Y (ver DEFAULT_FLOW_SCALE_X). */
#define DEFAULT_FLOW_SCALE_Y 0.942F
/**
 * Compensação da rotação no fluxo pelo eixo de mesmo nome (pareamento direto).
 * Desligada: neste drone o fluxo responde ao giro do outro eixo (ver
 * ::DEFAULT_FLOW_GYRO_XY). Os ensaios antigos usavam X = +1 e Y = -1.
 */
#define DEFAULT_FLOW_GYRO_X  0.0F
/** Ver ::DEFAULT_FLOW_GYRO_X. */
#define DEFAULT_FLOW_GYRO_Y  0.0F
/**
 * O MTF-01 fica longe do centro do drone (na traseira): girando em guinada, ele
 * descreve um arco e mede uma velocidade que o centro não tem, r × distância.
 * Estes coeficientes, em m (m/s de velocidade falsa no corpo por rad/s de r do
 * modelo), saem dessa velocidade antes de ela ir para o local. Medidos nos
 * voos de 06/11 (só com o sensor a mais de 13 cm do chão, 503 amostras com
 * |r| > 0,4 rad/s, já nos eixos do corpo): y 0,093 ± 0,024; x 0,05 ± 0,03,
 * que não se separa do ruído e fica em 0. Medidos no mesmo referencial em que
 * são aplicados, não dependem de adivinhar sinais de eixo.
 */
#define DEFAULT_FLOW_YAW_X   0.0F
/** Ver ::DEFAULT_FLOW_YAW_X. */
#define DEFAULT_FLOW_YAW_Y   0.09F
/**
 * Eixos do fluxo trocados em relação ao corpo. Ensaios de mão de 08/11 (a seco,
 * nivelado): o movimento para o lado aparece no canal X do fluxo (direita
 * negativa, esquerda positiva); frente e trás parecem cair no canal Y (frente
 * positiva), com menos certeza. 0 mantém o mapeamento original (X = frente,
 * Y = direita); 1 usa Y = frente e X = esquerda (padrão: é o que os voos usam).
 */
#define DEFAULT_FLOW_SWAP    1.0F
/**
 * 1 = o fluxo só entra se a qualidade passa de 30 (liga) e não cai a 15
 * (desliga), a inclinação é menor que ~37° (cos ≥ 0,8) e |fluxo| ≤ 4,2 rad/s,
 * como no estimador da branch mtf-bno-test (que seguia o ArduPilot). 0 =
 * qualquer fluxo com qualidade > 0, como antes.
 */
#define DEFAULT_FLOW_GATE    0.0F
/**
 * 1 = o LQG recebe vx, vy do EKF de navegação (control/nav_ekf.h: fluxo +
 * acelerômetro do BNO, com viés, quantização e gate) e usa o Kalman
 * "fundido" (M, L com menos ruído em vx, vy; tools/lqg_ajustado.py). O EKF
 * roda sempre, como diagnóstico (DRONE_NAV_DEBUG); este parâmetro só decide
 * se o controle usa o que ele estima.
 */
#define DEFAULT_FLOW_EKF     0.0F
/** 1 = o LQG recebe o z do EKF de navegação em vez da distância crua corrigida. */
#define DEFAULT_Z_EKF        0.0F
/** Ruído da aceleração horizontal do BNO em voo, em m/s² (desvio medido: 0,4 a 0,5). */
#define DEFAULT_FEKF_ACC_N   1.0F
/** Ruído da aceleração vertical do BNO em voo, em m/s². */
#define DEFAULT_FEKF_ACCZ_N  1.0F
/** Passeio do viés do acelerômetro, em m/s² por raiz de s. */
#define DEFAULT_FEKF_BIAS_N  0.02F
/**
 * Ruído do fluxo, em rad/s. Vezes a distância dá m/s: 0,8 a 12 cm do chão são
 * ~0,10 m/s, o desvio amostra a amostra medido em voo (o ArduPilot usa 0,25,
 * mas o MTF em voo baixo é mais ruidoso).
 */
#define DEFAULT_FEKF_FLOW_N  0.8F
/** Ruído da distância, em m (desvio medido parado: 4 a 6 mm). */
#define DEFAULT_FEKF_RNG_N   0.006F
/** Gate de inovação, em desvios (o padrão do ArduPilot é 3). */
#define DEFAULT_FEKF_GATE    3.0F
/**
 * Taxa do DRONE_NAV_DEBUG (EKF de navegação) para a Pi, em Hz. O enlace é de
 * 115200 baud e o DRONE_CTRL_DEBUG a 50 Hz já ocupa ~65% dele; a 10 Hz esta
 * mensagem soma ~1 kB/s.
 */
#define DEFAULT_NAV_DBG_HZ   10.0F
/**
 * Integral do erro de posição x, y, em 1/s (0 desliga). O LQG só tem termo
 * proporcional em x, y, e fraco (0,37° de inclinação por 10 cm): um erro de
 * nível ou de equilíbrio de 1–2° deixava o drone 30–50 cm fora do ponto ou
 * derivando para um lado. O integral desloca a referência que o LQG recebe até
 * o erro médio zerar, como o I do laço de velocidade do ArduPilot. 0,3/s fica
 * abaixo da banda do laço de posição (oscilação de x a ~0,2 Hz nos voos).
 */
#define DEFAULT_CTL_KI_XY    0.3F
/** Limite do deslocamento integral de x, y, em m (anti-windup). */
#define DEFAULT_CTL_XY_I_MAX 0.30F
/**
 * Pareamento cruzado da compensação do giro (FLOW_GYRO_XY e FLOW_GYRO_YX).
 * Num voo de 08/11 (captura USB, hélices, 18 s de hover, ajuste por mínimos
 * quadrados sem atraso) o fluxo bruto respondeu a gy no canal X e a gx no
 * canal Y, não ao eixo do mesmo nome: flow_x ≈ -1,3 a -1,6·gy e
 * flow_y ≈ +1,6 a +1,7·gx. Os módulos acima de 1 incluem o braço do sensor.
 * Os ensaios a seco de 08/11 (222149, 220912) confirmaram flow_x ≈ -1,7·gy.
 * Padrão desde 08/11: em 21:08–22:21 o --set esqueceu estes valores e todos
 * os voos rodaram com o pareamento direto, errado.
 */
#define DEFAULT_FLOW_GYRO_XY 1.45F
/** Ver ::DEFAULT_FLOW_GYRO_XY. */
#define DEFAULT_FLOW_GYRO_YX (-1.65F)
/**
 * 1 = quando o MTF-01 devolve contagem zero num eixo (ele só reporta degraus
 * de 0,2 rad/s), o Kalman recebe a própria predição, limitada à faixa de
 * ±0,1 rad/s, no lugar de "velocidade = 0". Capturas de 08/11 mostraram que
 * o sensor parado leva a um desvio de ~0,65 rad/s por amostra.
 */
#define DEFAULT_FLOW_CENS    0.0F
/** Viés do fluxo X do sensor, em rad/s (medido parado: ~+0,047). 0 = sem correção. */
#define DEFAULT_FLOW_BIAS_X  0.0F
/** Viés do fluxo Y do sensor, em rad/s (medido parado: ~-0,024). 0 = sem correção. */
#define DEFAULT_FLOW_BIAS_Y  0.0F

/* ---- Voo ---------------------------------------------------------------- */

/**
 * Maior altura que qualquer pedido de decolagem ou trecho aceita, em m. O
 * teste de controle sobe no máximo 50 cm (lidos pelo MTF-01; era 25 até 06/11); o painel escolhe
 * a altura de cada voo abaixo disso.
 */
#define FLIGHT_MAX_HEIGHT_M       0.50F
/** Folga numérica ao comparar uma altura pedida com ::FLIGHT_MAX_HEIGHT_M, em m. */
#define FLIGHT_HEIGHT_TOL_M       0.001F
/**
 * Se a altura medida passar de ::FLIGHT_MAX_HEIGHT_M mais esta folga, o voo
 * pousa sozinho, em m.
 */
#define FLIGHT_OVERSHOOT_M        0.20F
/**
 * Ao encostar no chão, a aceleração desce em rampa até a marcha lenta durante
 * este tempo antes de desarmar, em ms (em vez de cortar de uma vez).
 */
#define FLIGHT_LAND_SPINDOWN_MS   1500U
/**
 * Depois da marcha lenta, a aceleração do controle entra em rampa durante
 * este tempo, em ms. Sem ela, o salto de 5% para o hover fazia o lado mais
 * leve pular e passar do corte de inclinação (ensaio de 04/11 com CTL_W0 = 800).
 */
#define FLIGHT_TAKEOFF_RAMP_MS    2000U
/** Altura estimada acima da qual o drone conta como "saiu do chão", em m. */
#define FLIGHT_LIFTED_HEIGHT_M    0.03F
/** Altura estimada abaixo da qual o drone conta como apoiado no chão (ZUPT do EKF), em m. */
#define FLIGHT_GROUNDED_HEIGHT_M  0.01F
/** Menor velocidade de descida no pouso, em m/s (a rampa do pouso nunca é um degrau). */
#define FLIGHT_LAND_MIN_RATE_M_S  0.03F
/**
 * No pouso, com o drone mais de ::FLIGHT_LAND_ABOVE_M acima da referência e
 * descendo a menos de ::FLIGHT_LAND_DESCENT_M_S, a correção de hover dos
 * correções lentas (estimador de perturbação ou ajuste lento) cai a esta taxa, em rad/s por s. O
 * integrador de altura sozinho desfaz devagar demais: no ensaio de 05/11 o
 * drone ficou 3–5 s parado no alto com a referência já no chão.
 */
#define FLIGHT_LAND_BLEED_RATE    20.0F
/** Drone acima da referência no pouso, em m (ver ::FLIGHT_LAND_BLEED_RATE). */
#define FLIGHT_LAND_ABOVE_M       0.01F
/** Descida que já conta como "está descendo" no pouso, em m/s (ver ::FLIGHT_LAND_BLEED_RATE). */
#define FLIGHT_LAND_DESCENT_M_S   0.03F
/**
 * Decolagem ou trecho com o alvo mais de ::FLIGHT_STUCK_ERROR_M acima da
 * altura medida e sem subir ::FLIGHT_STUCK_RISE_M em ::FLIGHT_STUCK_MS: o
 * drone está preso (cordinhas, teto). O voo passa a segurar na altura medida
 * e o empuxo acumulado desde o travamento é devolvido. Sem isso o integrador
 * de altura crescia até o corte por inclinação (ensaio de 05/11: motores a 57%
 * e FAILSAFE). Usa a altura do MTF filtrada, não a velocidade do Kalman: preso,
 * o filtro estimava subida de 4–5 cm/s com a altura parada.
 */
#define FLIGHT_STUCK_ERROR_M      0.04F
/**
 * 1 = detector de subida presa ligado (FLT_PRESO). Padrão desligado: ele serve
 * na bancada com cordinhas; sem elas, na qualificação de 05/11, disparou em 7
 * de 9 voos aos 7–10 s e trocou o alvo de 20 cm por "HOLD em 2 cm", porque fora
 * do efeito solo o drone sobe devagar enquanto o estimador acha o empuxo.
 */
#define DEFAULT_FLT_PRESO         0.0F
/**
 * 1 = a rampa de partida só reduz a aceleração média, e a diferença entre os
 * motores (atitude e desequilíbrio fixo) chega inteira (FLT_RAMP_CM). Em 7
 * voos reais o pitch disparou (14 a 20°) logo depois de o drone sair do chão,
 * quando a rampa ainda encolhia essa diferença a ~90%; o pico sempre cai no
 * fim da rampa. 0 = comportamento anterior, para comparar.
 */
#define DEFAULT_FLT_RAMP_CM       0.0F
/**
 * Limite da guinada total (LQG + FF + estimador), em rad/s (0 = sem limite).
 * Em 5 voos reais de 08/11 um giro súbito (|r| de 1,2 a 3,7 rad/s) fez o LQG
 * pedir ±200 a ±400 rad/s e levou roll e pitch ao failsafe.
 */
#define DEFAULT_CTL_PSI_MAX       0.0F
/** Subida que conta como progresso e recomeça a contagem, em m (ver ::FLIGHT_STUCK_ERROR_M). */
#define FLIGHT_STUCK_RISE_M       0.02F
/** Tempo travado até segurar na altura medida, em ms (ver ::FLIGHT_STUCK_ERROR_M). */
#define FLIGHT_STUCK_MS           3000U
/** Altura medida a partir da qual o drone conta como fora do chão para o travamento, em m. */
#define FLIGHT_STUCK_MIN_HEIGHT_M 0.015F
/**
 * Constante de tempo do filtro da altura medida (MTF) e da velocidade vertical
 * derivada dela, em s. Usados pelo travamento e pelo pouso no lugar da
 * velocidade do Kalman, que fica enviesada quando o drone não segue o modelo.
 */
#define FLIGHT_MEAS_Z_TAU_S       0.3F

/** Altura padrão do hover, em m (fora do efeito solo, abaixo do teto de 50 cm). */
#define DEFAULT_HOVER_HEIGHT_M 0.30F
/** Rampa da referência vertical, em m/s (0 = degrau, como na simulação). */
#define DEFAULT_Z_RATE_M_S     0.05F
/** Rampa padrão da referência horizontal, em m/s. */
#define DEFAULT_XY_RATE_M_S    0.3F
/** Rampa da referência de guinada, em rad/s. */
#define DEFAULT_YAW_RATE_RAD_S 0.5F
/** Inclinação acima da qual os motores são cortados, em rad (20 graus). */
#define DEFAULT_TILT_MAX_RAD   0.35F
/** Altura acima do chão em que o pouso termina, em m. */
#define DEFAULT_LAND_HEIGHT_M  0.03F
/**
 * 1 = o LQG também segura x e y (usa o fluxo óptico); 0 = só atitude e
 * altura (control/attitude_mode.h). Padrão desligado até a atitude e a altura
 * serem validadas. O ensaio de XY deve ser ativado explicitamente com CTL_XY=1.
 */
#define DEFAULT_CTL_XY         0.0F

/* ---- Ajuste lento do hover e do equilíbrio (control/trim.h) ---------------
   O LQR do TCC não tem integrador, então um hover errado ou um peso mal
   distribuído (frente leve, traseira pesada) deixam erro de altura e de
   inclinação. Valores calibrados em simulação com perturbações de até
   120 rad/s no hover e 30 rad/s de desequilíbrio. */
/* ---- LQG ajustado e estimador de perturbação ---------------------------- */
/* O LQI (LQR com integradores) que existia para comparação saiu em 06/11. */
/**
 * 1 = LQG ajustado: Kd e Kfd do mesmo projeto do TCC com Q mais firme em altura
 * e atitude (tools/lqg_ajustado.py); o Kalman continua o do Simulink.
 */
#define DEFAULT_CTL_LQG_AJ   1.0F
/**
 * 1 = estimador de perturbação ("LQG melhorado", control/lqg_dist.h): o
 * Kalman também estima o hover errado e os desequilíbrios, e o controle os
 * cancela.
 */
#define DEFAULT_CTL_DIST     1.0F
/**
 * Maior perturbação estimada de cada canal, em rad/s por motor. Era 250, o
 * limite que dividia com o LQI; com o chumbo (550 g, 06/11) o hover passou
 * do CTL_W0 do painel por mais que isso e o drone não saía do efeito solo.
 */
#define DEFAULT_CTL_DIST_MAX 400.0F

/**
 * Desequilíbrio frente/trás fixo somado aos motores desde a decolagem, em
 * rad/s por motor (+ na frente, − atrás). Até 05/11 a traseira era mais pesada
 * e o drone precisava de −90. Em 06/11 entrou o chumbo de verdade e o frame
 * ficou mais equilibrado; nos hovers de 06/11 o estimador de perturbação
 * parou perto de θ = −22 e φ = −40 rad/s. Usa-se o valor conservador de θ
 * anterior à integração Pi; o estimador continua livre para corrigir o restante.
 */
#define DEFAULT_CTL_FF_TH  (-20.0F)
/**
 * Desequilíbrio esquerda/direita fixo, em rad/s (+ na esquerda; mesmo padrão do
 * φ do estimador). Nos hovers de 06/11 o estimador parou em −40 em todos os
 * voos; sem isso o drone sai para um lado ao subir, até ele aprender.
 */
#define DEFAULT_CTL_FF_PHI (-40.0F)
/**
 * Torque de guinada fixo, em rad/s (mesmo padrão do ψ do estimador). Nos voos
 * de 06/11 com o estimador de guinada, ele parou em +49 a +61.
 *
 * Desligado por padrão: o torque fixo de guinada não foi qualificado no voo
 * livre desta integração. Pode ser testado isoladamente com CTL_FF_PSI.
 */
#define DEFAULT_CTL_FF_PSI 0.0F

/** 1 = liga o ajuste lento (CTL_TRIM). Padrão desligado: o painel liga. */
#define DEFAULT_CTL_TRIM         0.0F
/**
 * Ganho do hover, em rad/s por (m·s). Acima de ~50 a malha de altura oscila.
 */
#define DEFAULT_TRIM_KI_Z        20.0F
/** Ganho do equilíbrio de arfagem e rolagem, em rad/s por (rad·s). */
#define DEFAULT_TRIM_KI_ATT      80.0F
/**
 * Limite do termo de hover, em rad/s. No ensaio de 04/11 o hover médio passou
 * de 850 rad/s (48% de aceleração) sem o drone levantar por inteiro.
 */
#define DEFAULT_TRIM_MAX         250.0F
/**
 * Limite de cada termo de equilíbrio (arfagem e rolagem), em rad/s. No ensaio
 * de 04/11 a traseira precisou de ~125 rad/s a mais de cada lado (14% de
 * aceleração entre traseira e frente) e o limite de 120 saturou.
 */
#define DEFAULT_TRIM_ATT_MAX     250.0F
/**
 * Rampa do hover enquanto o drone ainda não se soltou do chão, em rad/s por s:
 * sem ela, sair do chão levaria dezenas de segundos. Era 10: com o CTL_W0
 * 350 abaixo do hover, o voo de 50 cm de 06/11 levou 13 s para sair do chão.
 * Para no desprendimento, então a folga ao sair é de poucos rad/s.
 */
#define TRIM_GROUND_RATE         20.0F
/** Altura acima do chão (estimada) que marca o desprendimento, em m. */
#define TRIM_RELEASE_HEIGHT_M    0.010F
/**
 * No ar, o erro de ângulo integrado pelo equilíbrio fica limitado a ± isto, em
 * rad (~4,6°). Em simulação, um empurrão que inclina o drone e some reduziu o
 * rebote de +57° para +12° com este limite.
 */
#define TRIM_ATT_ERROR_MAX_RAD   0.08F
/**
 * O equilíbrio é guardado para o próximo voo só com |φ| e |θ| abaixo disto,
 * em rad (~2°), com o drone no ar e assentado.
 */
#define TRIM_LEARN_LEVEL_RAD     0.035F
/**
 * ... e só em HOLD com a altura medida a menos disto do alvo, em m. Na
 * decolagem presa nas cordinhas (05/11) ele guardou θ = −128 e φ = −10 rad/s,
 * contra cerca de −85 e 0 do hover de verdade, e o voo seguinte saía torto.
 */
#define TRIM_LEARN_Z_ERROR_M     0.015F
/** O hover só integra no ar com |vz| abaixo disto, em m/s. */
#define TRIM_SETTLED_VZ          0.05F
/** Começa em ensaio a seco: os motores só giram depois de FLT_DRY_RUN = 0. */
#define DEFAULT_DRY_RUN          1.0F
/**
 * 1 = drone simulado (control/sim_plant.h): motores parados e as medidas vêm
 * do modelo do TCC controlado pelo próprio LQG. Para testar a comunicação com
 * a Pi na bancada.
 */
#define DEFAULT_FLT_SIM          0.0F
/**
 * Decolagem e trechos só terminam (e o drone só aceita o próximo trecho)
 * depois de ficar este tempo seguido dentro da tolerância e parado, em ms.
 */
#define FLIGHT_SETTLE_MS         1000U
/** Velocidade (vx, vy, vz) abaixo da qual o drone está parado, em m/s. */
#define FLIGHT_SETTLE_SPEED_M_S  0.05F
/** Taxa angular (p, q, r) abaixo da qual o drone está parado, em rad/s. */
#define FLIGHT_SETTLE_RATE_RAD_S 0.1F

/* ---- Telemetria --------------------------------------------------------- */

/** Taxa do DRONE_CTRL_DEBUG para a Pi, em Hz (0 = desligado até pedirem). */
#define DEFAULT_DEBUG_RATE_HZ 0.0F
/** Taxa da linha EST no log USB, em Hz. */
#define DEFAULT_LOG_EST_HZ    10.0F

#endif
