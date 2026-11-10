/**
 * @file attitude.h
 * @brief Matemática de atitude a partir de quatérnios.
 *
 * Código puro: sem HAL e sem dependência de sensor, para compilar e rodar no
 * PC nos testes. Os quatérnios são sempre na ordem w,x,y,z, a mesma usada em
 * bno_sample_t::q.
 */
#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>

/**
 * @brief Ângulos de Euler na convenção ZYX (yaw-pitch-roll).
 */
typedef struct {
  /** Rolagem, em radianos. */
  float roll;
  /** Arfagem, em radianos. Satura em +-pi/2. */
  float pitch;
  /** Guinada (yaw), em radianos. */
  float yaw;
} euler_t;

/**
 * @brief Parcela de atitude do vetor de estados de um controlador em Euler.
 *
 * Os ângulos seguem a convenção ZYX e ficam em radianos. @c body_rate contém
 * exatamente o que o giroscópio mede nos eixos do corpo: p, q e r. Esses
 * valores são os estados usuais de um modelo de quadricóptero linearizado no
 * hover. @c euler_rate contém as derivadas exatas roll_dot, pitch_dot e
 * yaw_dot, calculadas a partir de p,q,r para modelos que usam essa outra
 * definição.
 *
 * O quatérnio não aparece nesta interface de controle, mas continua sendo a
 * fonte interna da orientação e deve ser preservado para interpolação e
 * rotação de vetores.
 */
typedef struct {
  /** Roll, pitch e yaw ZYX, em radianos. */
  euler_t angle;
  /** Velocidades angulares do corpo p, q, r, em rad/s. */
  float body_rate[3];
  /** Derivadas dos ângulos de Euler, em rad/s. */
  float euler_rate[3];
} attitude_euler_state_t;

/**
 * @brief Converte um quatérnio em ângulos de Euler intrínsecos ZYX.
 *
 * A arfagem satura em +-90 graus, onde rolagem e guinada deixam de ser
 * separáveis (gimbal lock). Nesse caso o resultado é válido, mas a divisão
 * entre rolagem e guinada é arbitrária.
 *
 * @param[in]  q    Quatérnio w,x,y,z.
 * @param[out] out  Ângulos resultantes.
 */
void attitude_to_euler(const float q[4], euler_t *out);

/**
 * @brief Monta a parcela de atitude de um vetor de estados em Euler.
 *
 * Converte o quatérnio para roll/pitch/yaw, conserva p,q,r sem aproximá-los e
 * calcula também as derivadas exatas dos ângulos. Perto do hover vale a
 * aproximação roll_dot ~= p, pitch_dot ~= q e yaw_dot ~= r, mas esta função
 * não depende dela.
 *
 * A conversão de p,q,r para derivadas de Euler é singular em pitch = +-90
 * graus. Nesse caso, ou se qualquer entrada não for finita, a função zera
 * @c euler_rate e retorna false. Os ângulos e p,q,r continuam preenchidos
 * quando são finitos, permitindo diagnóstico sem entregar derivadas
 * inválidas ao controlador.
 *
 * @param[in]  q          Quatérnio w,x,y,z.
 * @param[in]  body_rate  Velocidades p,q,r, em rad/s.
 * @param[out] out        Estado resultante.
 * @return true quando todos os campos, inclusive @c euler_rate, são válidos.
 */
bool attitude_build_euler_state(const float q[4], const float body_rate[3],
                                attitude_euler_state_t *out);

/**
 * @brief Normaliza um ângulo para o intervalo [-pi, pi).
 *
 * Necessário para que, por exemplo, referência -179 graus e medição +179
 * graus produzam erro +2 graus, e não -358 graus.
 *
 * @param[in] angle  Ângulo em radianos.
 * @return Ângulo normalizado; NaN se a entrada não for finita.
 */
float attitude_wrap_pi(float angle);

/**
 * @brief Calcula referência menos medição para roll, pitch e yaw.
 *
 * Cada componente é normalizada por ::attitude_wrap_pi. Essa é a forma que o
 * futuro LQR/PID deve usar para montar delta-x da atitude.
 *
 * @param[in]  reference  Atitude desejada.
 * @param[in]  measured   Atitude medida.
 * @param[out] error      Erro angular normalizado.
 */
void attitude_euler_error(const euler_t *reference, const euler_t *measured,
                          euler_t *error);

/**
 * @brief Mede a atitude em relação a uma posição de referência (zero).
 *
 * Roll e pitch saem como medido menos zero, normalizados por
 * ::attitude_wrap_pi, então um zero perto de +-180 graus funciona. A guinada
 * é copiada sem alteração: continua referida ao norte do sensor.
 *
 * @param[in]  zero      Roll e pitch lidos com o veículo nivelado (yaw ignorado).
 * @param[in]  measured  Atitude medida.
 * @param[out] relative  Atitude relativa ao zero; pode ser o próprio @p measured.
 */
void attitude_euler_relative(const euler_t *zero, const euler_t *measured,
                             euler_t *relative);

/**
 * @brief Calcula o cosseno da inclinação em relação à vertical.
 *
 * Corresponde ao termo (3,3) da matriz de rotação. Usado para rejeitar fluxo
 * óptico medido num ângulo em que a distância lida deixa de corresponder à
 * altura.
 *
 * @param[in] q  Quatérnio w,x,y,z.
 * @return 1.0 com a placa nivelada, 0.0 com a placa de lado.
 */
float attitude_cos_tilt(const float q[4]);

/**
 * @brief Gira um vetor horizontal do referencial do corpo para o do mundo.
 *
 * O fluxo óptico é medido nos eixos do corpo. Se o veículo gira enquanto se
 * move, cada amostra precisa ser levada a um referencial fixo antes de ser
 * integrada; caso contrário a trajetória acumulada vira lixo.
 *
 * @param[in]  yaw  Guinada atual, em radianos.
 * @param[in]  bx   Componente X no referencial do corpo.
 * @param[in]  by   Componente Y no referencial do corpo.
 * @param[out] wx   Componente X no referencial do mundo.
 * @param[out] wy   Componente Y no referencial do mundo.
 */
void attitude_yaw_rotate(float yaw, float bx, float by, float *wx, float *wy);

/**
 * @brief Gira um vetor 3D dos eixos do corpo para os eixos do mundo.
 *
 * Usado para a aceleração da IMU que alimenta a predição do filtro: esse
 * vetor é medido nos eixos do sensor, e um veículo inclinado espalha uma
 * aceleração puramente horizontal pelos três eixos.
 *
 * @param[in]  q      Quatérnio w,x,y,z.
 * @param[in]  body   Vetor x,y,z no referencial do corpo.
 * @param[out] world  Componentes horizontais x,y no referencial do mundo
 *                    (o z não é calculado).
 */
void attitude_rotate_to_world(const float q[4], const float body[3], float world[2]);

/** Gira corpo -> mundo e então expressa o plano horizontal no referencial
 * local cujo yaw zero é @p yaw_origin. Assim aceleração e fluxo usam os
 * mesmos eixos mesmo quando o BNO inicia apontando longe do norte. */
void attitude_rotate_to_local(const float q[4], float yaw_origin,
                              const float body[3], float local[2]);

/**
 * @brief Converte radianos em graus, para logs e leitura humana.
 *
 * @param[in] radians  Ângulo em radianos.
 * @return O mesmo ângulo em graus.
 */
float attitude_rad_to_deg(float radians);
#endif
