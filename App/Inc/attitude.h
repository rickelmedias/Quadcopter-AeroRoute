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

/**
 * @brief Converte radianos em graus, para logs e leitura humana.
 *
 * @param[in] radians  Ângulo em radianos.
 * @return O mesmo ângulo em graus.
 */
float attitude_rad_to_deg(float radians);
#endif
