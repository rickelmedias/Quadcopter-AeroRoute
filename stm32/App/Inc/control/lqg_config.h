/**
 * @file lqg_config.h
 * @brief Constantes do controle LQR + Kalman de bancada (modo com cabo).
 *
 * Mapa de eixos, zero de nível e escalas do fluxo: copiados de
 * `flight_config.h` do firmware da submissão (branch `submission/2026-11-11`),
 * onde foram medidos no drone real. Não são suposições novas, mas a montagem
 * física pode ter mudado: confira o painel "Medidas do controle" da IHM com o
 * drone desarmado antes de decolar (nariz para baixo -> θ negativo; lado
 * direito para baixo -> φ positivo; giro horário visto de cima -> ψ positivo).
 */
#ifndef CONTROL_LQG_CONFIG_H
#define CONTROL_LQG_CONFIG_H

/* ---- Sensores -> NED/FRD ------------------------------------------------- */

/** Roll cru do BNO com o drone nivelado no chão, em graus (montado invertido). */
#define LQG_ZERO_ROLL_DEG  (-174.5F)
/** Pitch cru do BNO com o drone nivelado no chão, em graus. */
#define LQG_ZERO_PITCH_DEG (-2.9F)

/**
 * Montagem do BNO como quatérnio (w, x, y, z): q_corpo = WORLD ⊗ q_bno ⊗ MOUNT ⊗ conj(zero).
 *
 * O quatérnio do BNO leva do sensor para o mundo dele (ENU, z para cima). WORLD
 * é a rotação ENU -> NED (180° em torno de (x+y)/√2). MOUNT é a rotação do corpo
 * para o sensor (−90° em z): é a mesma do mapa do giroscópio (p = −gy, q = gx,
 * r = gz). Achados numericamente a partir do mapa de Euler de cima (único par entre
 * as 24 rotações de eixos; erro de segunda ordem em φθ, ~1° com 8,6° de cada).
 * O "zero" é a inclinação do corpo que o par produz na pose de ::LQG_ZERO_ROLL_DEG
 * e ::LQG_ZERO_PITCH_DEG, descontada à direita (no corpo).
 */
#define LQG_Q_WORLD {0.0F, 0.70710678F, 0.70710678F, 0.0F}
#define LQG_Q_MOUNT {0.70710678F, 0.0F, 0.0F, -0.70710678F}

/**
 * Multiplicador do ruído R da atitude pelo status de precisão do BNO (bits 1:0 do
 * relatório: 0 não confiável, 1 baixa, 2 média, 3 alta). Nunca anula a medida: sem
 * gravidade na aceleração, a atitude do BNO é a única referência de inclinação.
 */
#define LQG_ATT_R_SCALE {8.0F, 4.0F, 2.0F, 1.0F}

/** φ do modelo = LQG_SGN_ROLL * (pitch do sensor relativo ao zero). */
#define LQG_SGN_ROLL  (+1.0F)
/** θ do modelo = LQG_SGN_PITCH * (roll do sensor relativo ao zero). */
#define LQG_SGN_PITCH (+1.0F)
/** ψ do modelo = LQG_SGN_YAW * (guinada do sensor desde o armamento). */
#define LQG_SGN_YAW   (-1.0F)
/** p do modelo = LQG_SGN_P * giro y do sensor. */
#define LQG_SGN_P     (-1.0F)
/** q do modelo = LQG_SGN_Q * giro x do sensor. */
#define LQG_SGN_Q     (+1.0F)
/** r do modelo = LQG_SGN_R * giro z do sensor. */
#define LQG_SGN_R     (+1.0F)
/** Sinal da velocidade u (frente) obtida do fluxo. */
#define LQG_SGN_VX    (+1.0F)
/** Sinal da velocidade v (direita) obtida do fluxo. */
#define LQG_SGN_VY    (+1.0F)

/** Escala do fluxo do MTF-01 em x, medida contra o giroscópio. */
#define LQG_FLOW_SCALE_X (0.944F)
/** Escala do fluxo do MTF-01 em y. */
#define LQG_FLOW_SCALE_Y (0.942F)
/** Compensação da rotação no fluxo x (giro x do sensor), adimensional. */
#define LQG_FLOW_GYRO_X  (+1.0F)
/** Compensação da rotação no fluxo y (giro y do sensor), adimensional. */
#define LQG_FLOW_GYRO_Y  (-1.0F)
/** Velocidade falsa em x por rad/s de r (MTF fora do centro), em m. */
#define LQG_FLOW_YAW_X   (0.0F)
/** Velocidade falsa em y por rad/s de r (MTF fora do centro), em m. */
#define LQG_FLOW_YAW_Y   (0.09F)

/** Qualidade mínima do fluxo (0..255); abaixo disso a medida é descartada. */
#define LQG_FLOW_MIN_QUALITY (20U)

/**
 * Altura acima do chão do armamento (m, para cima) a partir da qual o fluxo
 * vale. O PMW3901 do MTF-01 não foca abaixo de ~8 cm: no chão (7–8 cm do
 * sensor) a qualidade fica em 25–28 e a velocidade sai com espúrios. ArduPilot
 * também ignora o fluxo antes da decolagem perto do chão.
 */
#define LQG_FLOW_MIN_HEIGHT_M  (0.10F)
/** Abaixo desta altura (m) o fluxo volta a ser ignorado (histerese de ::LQG_FLOW_MIN_HEIGHT_M). */
#define LQG_FLOW_OFF_HEIGHT_M  (0.07F)
/**
 * Passos de controle (10 ms) cujo giro é médio na compensação de rotação do
 * fluxo. O MTF-01 entrega o fluxo acumulado no intervalo do quadro (100 Hz) e
 * com atraso; o giro instantâneo do BNO compensava rotação que o quadro ainda
 * não tinha visto e virava velocidade falsa com vibração. 2 passos = 20 ms.
 */
#define LQG_FLOW_GYRO_AVG_STEPS (2U)

/**
 * Posição do MTF-01 em relação ao centro de massa, em m, eixos do corpo FRD
 * (x frente, y direita). Inclinado, o sensor fora do centro sobe ou desce e a
 * distância muda sem a altura mudar: no chão isso disparava o "descolou".
 * O sinal de x vem da mesma medida de ::LQG_FLOW_YAW_Y (velocidade lateral
 * falsa = x·r): 0,09 m à frente. CONFERIR com régua no drone; se o sensor
 * estiver atrás, o valor é negativo e ::LQG_FLOW_YAW_Y também troca de sinal.
 */
#define LQG_RANGE_POS_X_M (0.09F)
/** Posição lateral do MTF-01, em m (y para a direita). */
#define LQG_RANGE_POS_Y_M (0.0F)

/* ---- Entradas do modelo -> motores físicos -------------------------------- */

/**
 * Motor físico (índice 0 = M1) que recebe cada entrada Out1[i] do modelo
 * (mixer.c da submissão): u1 frente/esquerda -> M4, u2 frente/direita -> M2,
 * u3 trás/direita -> M1, u4 trás/esquerda -> M3. Vista de cima, frente para
 * cima: M4 M2 na frente, M3 M1 atrás. M1 e M4 têm sentido invertido no ESC.
 */
#define LQG_MOTOR_OF_INPUT {3U, 1U, 0U, 2U}

/* ---- Rotação -> aceleração dos motores ------------------------------------ */

/**
 * Rotação do motor por décimo de por cento de aceleração, em (rad/s)/‰.
 * Curva provisória da submissão (ω = 1755 rad/s em 100 %), linear pela origem.
 * O Δω do modelo vira Δ‰ = Δω / LQG_W_PER_PERMILLE em torno do hover.
 */
#define LQG_W_PER_PERMILLE (1.755F)

/* ---- Voo ------------------------------------------------------------------ */

/** Passo da malha, em s (Ts do modelo gerado). */
#define LQG_TS_S                (0.01F)
/** Aceleração com que a decolagem começa, em ‰ (5 %). */
#define LQG_IDLE_PERMILLE       (50.0F)
/** Menor aceleração enviada a um motor girando em voo, em ‰. */
#define LQG_MIN_PERMILLE        (30.0F)
/**
 * Altura acima do chão que conta como "descolou", em m, mantida por
 * ::LQG_LIFTOFF_STEPS. Com o braço do MTF compensado, a inclinação apoiada não
 * passa disso; e se passar, a base é a estimativa do hover e o controle de altura
 * segue subindo. A confirmação em 8 cm deixou o drone real quicando no efeito
 * solo sem controle de altura (12/11, 17:36).
 */
#define LQG_LIFTOFF_M           (0.03F)
/** Passos seguidos (10 ms) acima de ::LQG_LIFTOFF_M para valer como decolagem (50 ms). */
#define LQG_LIFTOFF_STEPS       (5U)
/**
 * Constante de tempo, em s, da leitura de base da altura na rampa. A decolagem conta
 * quando a altura passa ::LQG_LIFTOFF_M acima dessa base e não de zero: com os motores
 * girando o MTF passou a ler 2 a 4 cm sem o drone sair do chão (12/11, 18:02), e a
 * decolagem falsa a 47 % deixou o drone 12 s no chão com o controle "no ar".
 */
#define LQG_LIFTOFF_FLOOR_TAU_S (1.0F)
/**
 * Maior erro de guinada que entra no controle, em rad (~29°). Um erro de 170° pedia a
 * diferença máxima entre motores e gastava a folga de empuxo com guinada.
 */
#define LQG_YAW_ERR_MAX_RAD     (0.5F)
/**
 * Aceleração vertical do modelo por ‰ de aceleração coletiva, em m/s² por ‰
 * (4 motores × B[w] × ::LQG_W_PER_PERMILLE). Só estima quanto a rampa passou
 * do hover até a detecção da decolagem.
 */
#define LQG_ACCEL_PER_PERMILLE  (0.0428F)
/** Maior desconto do atraso da detecção da decolagem, em s. */
#define LQG_LIFTOFF_LAG_MAX_S   (1.0F)
/**
 * Diferença máxima entre motores na rampa rápida da decolagem (aceleração abaixo
 * de hover − ::LQG_SPOOL_FAST_MARGIN_PERMILLE) e no fim do pouso, em ‰. Apoiado, a
 * atitude não responde ao Δω e o LQR abria 40–70 ‰ entre motores. Da rampa lenta
 * em diante a correção é inteira: o drone pode estar leve e quicando.
 */
#define LQG_GROUND_AUTHORITY_PERMILLE (30.0F)
/** Altura abaixo da qual o pouso corta os motores, em m. */
#define LQG_TOUCHDOWN_M         (0.05F)
/** Inclinação que corta os motores, em rad (35 graus). */
#define LQG_TILT_MAX_RAD        (0.61F)
/** Se a altura passar do alvo por mais que isto, pousa sozinho, em m. */
#define LQG_OVERSHOOT_M         (0.30F)
/** Redução da aceleração por segundo durante o pouso, para garantir a descida, em ‰/s. */
#define LQG_LAND_BIAS_PERMILLE_S (15.0F)
/** Redução máxima da aceleração no pouso, em ‰. */
#define LQG_LAND_BIAS_MAX_PERMILLE (80.0F)
/**
 * 1: aplica o trim de hover por motor de lqg_gains.h (::LQG_HOVER_TRIM_PERMILLE, do CG medido);
 * 0: os quatro motores partem da mesma base (modelo da submissão, CG no centro).
 */
#ifndef LQG_CG_TRIM_ENABLE
#define LQG_CG_TRIM_ENABLE      (1)
#endif
/**
 * O trim por motor cresce com (base/::LQG_CG_TRIM_REF_PERMILLE)^4 (o torque de reação e o empuxo seguem a rotação ao
 * quadrado, e a diferença entre os pares cresceu mais rápido que isso nos voos de 14/11/2026:
 * +25 ‰ perto de 52 % e mais de +100 ‰ perto de 62 %). A razão sobe até este teto.
 */
#define LQG_CG_TRIM_RATIO_MAX   (1.25F)
/**
 * Aceleração em que os valores de ::LQG_HOVER_TRIM_PERMILLE valem (o trim é a tabela vezes
 * (base/esta)^4). Fixa: independe do parâmetro de hover, que o operador muda.
 */
#define LQG_CG_TRIM_REF_PERMILLE (550.0F)
/**
 * Abaixo desta altura (m) o drone ainda está no efeito solo e sem fluxo óptico (o controle de XY
 * só entra acima de ::LQG_FLOW_MIN_HEIGHT_M): o ajuste lento do hover integra ::LQG_TRIM_BOOST vezes
 * mais rápido (sempre limitado pelo teto de estabilidade) para sair dali.
 */
#define LQG_TRIM_BOOST_BELOW_M  (0.12F)
#define LQG_TRIM_BOOST          (2.0F)
/** Maior trim por motor aceito, em ‰ (CG a ~20 mm do centro dá ~90 ‰). */
#define LQG_CG_TRIM_MAX_PERMILLE (120.0F)
/** Limite do ajuste lento do hover (trim), em ‰. */
#define LQG_TRIM_MAX_PERMILLE   (250.0F)
/** Rampa de redução dos motores depois do toque, em ‰/s. */
#define LQG_SPINDOWN_PERMILLE_S (300.0F)
/** Aceleração até a qual a rampa de decolagem sobe rápida: hover - esta margem, em ‰. */
#define LQG_SPOOL_FAST_MARGIN_PERMILLE (100.0F)
/** Rampa rápida da decolagem, em ‰/s. */
#define LQG_SPOOL_FAST_PERMILLE_S (150.0F)
/** Limite do equivalente em posição do integrador XY, em m. */
#define LQG_INT_XY_LIMIT_M (1.0F)
/** Limite do equivalente em ângulo do integrador de guinada, em rad. */
#define LQG_INT_YAW_LIMIT_RAD (2.0F)
/** Passos sem IMU válido que cortam os motores (100 ms). */
#define LQG_IMU_BAD_STEPS       (10U)
/** Passos sem distância válida em voo que cortam os motores (250 ms). */
#define LQG_RANGE_BAD_STEPS     (25U)
/** Passos sem fluxo válido em voo que mandam pousar (2 s). */
#define LQG_FLOW_BAD_STEPS      (200U)
/** Amostras de distância no chão exigidas antes de decolar. */
#define LQG_GROUND_SAMPLES      (50U)
/**
 * Maior inclinação lida no chão que vira zero de nível do voo, em rad (5°).
 * Armado e parado, a média de φ e θ é tomada como nível: o zero fixo
 * (::LQG_ZERO_ROLL_DEG, ::LQG_ZERO_PITCH_DEG) veio da submissão, e no chão de
 * 12/11 a placa lia θ = −1,3 a −1,7°, o bastante para o drone sair andando antes
 * de o fluxo valer. Acima deste valor o chão não está nivelado: fica o zero fixo.
 */
#define LQG_LEVEL_MAX_RAD       (0.087F)

/** Repouso armado: amostras mínimas para usar a média do IMU como viés inicial do EKF3. */
#define LQG_REST_MIN_SAMPLES    (50U)
/** Viés de giro (rad/s) e de aceleração linear (m/s²) acima dos quais a média não é de repouso: ignorada. */
#define LQG_REST_GYRO_MAX_RAD_S (0.1F)
#define LQG_REST_ACCEL_MAX_M_S2 (0.5F)

/* ---- Ajuste lento do hover (integrador de altura) ------------------------- */

/**
 * O ganho efetivo do ajuste lento é o menor entre o parâmetro e esta fração do
 * limite de estabilidade da malha de altura. Com o integrador, a malha é
 * s³ + a·Kw·s² + a·Kz·s + a·ki (a = ::LQG_ACCEL_PER_PERMILLE, Kz e Kw em ‰ por m
 * e por m/s): estável se ki < a·Kz·Kw, ~26 ‰/(m·s) com os ganhos gerados e
 * 26·g.z·g.w com os multiplicadores. 0,1 deixa folga para o atraso dos motores,
 * que o modelo não tem.
 */
#define LQG_TRIM_STABILITY_FRACTION (0.1F)
/** a·Kz·Kw dos ganhos gerados (multiplicadores 1), em ‰/(m·s). */
#define LQG_TRIM_KI_LIMIT_1X        (26.2F)

/* ---- Bateria ---------------------------------------------------------------- */

/**
 * Tensão por célula em que o hover configurado vale, em V. O ESC é malha
 * aberta: a aceleração vira tensão média no motor, então a mesma rotação pede
 * aceleração proporcional a 1/V. Com a tensão medida (PA1 calibrado) a saída é
 * escalada por (células · este valor) / V filtrada.
 */
#define LQG_VBAT_REF_V_PER_CELL  (3.9F)
/** Constante de tempo do filtro da tensão, em s. */
#define LQG_VBAT_TAU_S           (1.0F)
/** Limites do fator de compensação da tensão. */
#define LQG_VBAT_FACTOR_MIN      (0.88F)
#define LQG_VBAT_FACTOR_MAX      (1.25F)
/** Abaixo desta tensão por célula, em voo, por ::LQG_LOW_BATT_STEPS: pousa. */
#define LQG_VBAT_LAND_V_PER_CELL (3.4F)
/** Aceleração base (já compensada) acima da qual, por ::LQG_LOW_BATT_STEPS, pousa: sem folga para a atitude, em ‰. */
#define LQG_BASE_LAND_PERMILLE   (850.0F)
/** Passos seguidos em bateria fraca ou base alta antes de pousar (2 s). */
#define LQG_LOW_BATT_STEPS       (200U)

/* ---- Valores ao ligar (voo só pela Pi, sem IHM) ------------------------------ */

/**
 * Parâmetros e multiplicadores que a task de controle aplica no boot. São os
 * das sessões com cabo de 11/11 (ganhos altura 10/5, atitude 5/5, guinada 5/5,
 * XY 1/1), com hover 55 % (descolou em ~55 %), limite 95 % e ajuste do hover
 * 100 ‰/(m·s) (era 60: nos voos da Pi de 14/11 o drone parou a ~8 cm, no efeito solo). A IHM, ao conectar, ainda sobrescreve tudo com o config.json.
 * Ordem de ::lqg_params_t: altura m, hover ‰, limite ‰, subida m/s, descida
 * m/s, rampa ‰/s, ajuste ‰/(m·s), integrador XY 1/s, integrador guinada 1/s,
 * ajuste ligado, EKF ligado.
 */
#define LQG_BOOT_PARAMS {0.30F, 550.0F, 950.0F, 0.10F, 0.10F, 80.0F, 100.0F, 0.05F, 0.3F, true, true}
/** Ordem de ::lqg_gains_t: z, w, att, rate, yaw, yaw_rate, xy, vel. */
#define LQG_BOOT_GAINS  {10.0F, 5.0F, 5.0F, 5.0F, 5.0F, 5.0F, 1.0F, 1.0F}

/* ---- Referência horizontal e de guinada (comandos da Raspberry Pi) ------------ */

/** Velocidade padrão da referência XY, em m/s. */
#define LQG_XY_SPEED_M_S       (0.10F)
/** Velocidade da referência de guinada, em rad/s. */
#define LQG_YAW_RATE_RAD_S     (0.5F)
/** Maior distância horizontal da origem aceita como alvo, em m. */
#define LQG_XY_LIMIT_M         (3.0F)

#endif
