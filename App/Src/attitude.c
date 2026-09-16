/**
 * @file attitude.c
 * @brief Implementação da matemática de atitude (contratos em attitude.h).
 *
 * Tudo em precisão simples: a FPU do Cortex-M4F só tem precisão simples, e um
 * double aqui cairia em silêncio numa emulação lenta por software.
 */
#include "attitude.h"
#include <math.h>

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

/** @brief Radianos para graus (ver attitude.h). */
float attitude_rad_to_deg(float radians) {
  return radians * (180.0F / 3.14159265358979323846F);
}
