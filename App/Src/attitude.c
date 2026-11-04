/**
 * @file attitude.c
 * @brief Implementação da matemática de atitude (contratos em attitude.h).
 *
 * Tudo em precisão simples: a FPU do Cortex-M4F só tem precisão simples, e um
 * double aqui cairia em silêncio numa emulação lenta por software.
 */
#include "attitude.h"
#include <math.h>

/** Pi em float, sem depender de M_PI, que não existe em C11 estrito. */
#define ATT_PI_F     3.14159265358979323846F
/** 2*pi em float. */
#define ATT_TWO_PI_F 6.28318530717958647692F
/** Abaixo deste |cos(pitch)|, a conversão de p,q,r fica numericamente singular. */
#define EULER_RATE_MIN_COS_PITCH 0.001F

/** @brief Quatérnio para Euler ZYX (ver attitude.h). */
void attitude_to_euler(const float q[4], euler_t *out) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  out->roll = atan2f(2.0F*(w*x + y*z), 1.0F - 2.0F*(x*x + y*y));
  /* Limita antes do asinf: o arredondamento de um quatérnio quase unitário
     pode passar o argumento um pouco de 1.0 e devolver NaN para uma atitude
     válida. */
  float sin_pitch = 2.0F*(w*y - z*x);
  if (sin_pitch > 1.0F) sin_pitch = 1.0F;
  if (sin_pitch < -1.0F) sin_pitch = -1.0F;
  out->pitch = asinf(sin_pitch);
  out->yaw = atan2f(2.0F*(w*z + x*y), 1.0F - 2.0F*(y*y + z*z));
}

/** @brief Monta estado Euler para controle (ver attitude.h). */
bool attitude_build_euler_state(const float q[4], const float body_rate[3],
                                attitude_euler_state_t *out) {
  attitude_to_euler(q, &out->angle);
  out->body_rate[0] = body_rate[0];
  out->body_rate[1] = body_rate[1];
  out->body_rate[2] = body_rate[2];
  out->euler_rate[0] = 0.0F;
  out->euler_rate[1] = 0.0F;
  out->euler_rate[2] = 0.0F;

  for (int i = 0; i < 4; ++i) {
    if (!isfinite(q[i])) return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (!isfinite(body_rate[i])) return false;
  }
  if (!isfinite(out->angle.roll) || !isfinite(out->angle.pitch) ||
      !isfinite(out->angle.yaw)) return false;

  const float sin_roll = sinf(out->angle.roll);
  const float cos_roll = cosf(out->angle.roll);
  const float cos_pitch = cosf(out->angle.pitch);
  if (fabsf(cos_pitch) < EULER_RATE_MIN_COS_PITCH) return false;
  const float tan_pitch = sinf(out->angle.pitch) / cos_pitch;
  const float p = body_rate[0], q_rate = body_rate[1], r = body_rate[2];

  out->euler_rate[0] = p + sin_roll*tan_pitch*q_rate +
                       cos_roll*tan_pitch*r;
  out->euler_rate[1] = cos_roll*q_rate - sin_roll*r;
  out->euler_rate[2] = (sin_roll*q_rate + cos_roll*r) / cos_pitch;
  return isfinite(out->euler_rate[0]) && isfinite(out->euler_rate[1]) &&
         isfinite(out->euler_rate[2]);
}

/** @brief Normaliza para [-pi,pi) (ver attitude.h). */
float attitude_wrap_pi(float angle) {
  if (!isfinite(angle)) return NAN;
  float wrapped = fmodf(angle + ATT_PI_F, ATT_TWO_PI_F);
  if (wrapped < 0.0F) wrapped += ATT_TWO_PI_F;
  return wrapped - ATT_PI_F;
}

/** @brief Erro angular referência - medição (ver attitude.h). */
void attitude_euler_error(const euler_t *reference, const euler_t *measured,
                          euler_t *error) {
  error->roll = attitude_wrap_pi(reference->roll - measured->roll);
  error->pitch = attitude_wrap_pi(reference->pitch - measured->pitch);
  error->yaw = attitude_wrap_pi(reference->yaw - measured->yaw);
}

/** @brief Cosseno da inclinação (ver attitude.h). */
float attitude_cos_tilt(const float q[4]) {
  const float x = q[1], y = q[2];
  return 1.0F - 2.0F*(x*x + y*y);
}

/** @brief Rotação horizontal pelo yaw (ver attitude.h). */
void attitude_yaw_rotate(float yaw, float bx, float by, float *wx, float *wy) {
  const float c = cosf(yaw), s = sinf(yaw);
  *wx = bx*c - by*s;
  *wy = bx*s + by*c;
}

/** @brief Rotação completa corpo -> mundo, componentes x e y (ver attitude.h). */
void attitude_rotate_to_world(const float q[4], const float body[3], float world[2]) {
  const float w = q[0], x = q[1], y = q[2], z = q[3];
  /* Duas primeiras linhas da matriz de rotação corpo -> mundo montada a
     partir de q. A terceira linha é o termo de attitude_cos_tilt(), e por
     isso os dois concordam por construção. */
  world[0] = (1.0F - 2.0F*(y*y + z*z))*body[0] + 2.0F*(x*y - w*z)*body[1] + 2.0F*(x*z + w*y)*body[2];
  world[1] = 2.0F*(x*y + w*z)*body[0] + (1.0F - 2.0F*(x*x + z*z))*body[1] + 2.0F*(y*z - w*x)*body[2];
}

/** @brief Rotação corpo -> referencial horizontal local (ver attitude.h). */
void attitude_rotate_to_local(const float q[4], float yaw_origin,
                              const float body[3], float local[2]) {
  float world[2];
  attitude_rotate_to_world(q, body, world);
  attitude_yaw_rotate(-yaw_origin, world[0], world[1], &local[0], &local[1]);
}

/** @brief Radianos para graus (ver attitude.h). */
float attitude_rad_to_deg(float radians) {
  return radians * (180.0F / ATT_PI_F);
}
