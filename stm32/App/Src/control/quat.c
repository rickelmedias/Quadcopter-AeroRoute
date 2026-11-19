/**
 * @file quat.c
 * @brief Matemática de quatérnios (contrato em control/quat.h).
 */
#include "control/quat.h"

#include <math.h>
#include <stddef.h>

#define HALF_PI_F 1.57079633F

void quat_identity(float q[4]) {
  q[0] = 1.0F;
  q[1] = 0.0F;
  q[2] = 0.0F;
  q[3] = 0.0F;
}

void quat_mul(const float a[4], const float b[4], float q[4]) {
  const float w = (a[0] * b[0]) - (a[1] * b[1]) - (a[2] * b[2]) - (a[3] * b[3]);
  const float x = (a[0] * b[1]) + (a[1] * b[0]) + (a[2] * b[3]) - (a[3] * b[2]);
  const float y = (a[0] * b[2]) - (a[1] * b[3]) + (a[2] * b[0]) + (a[3] * b[1]);
  const float z = (a[0] * b[3]) + (a[1] * b[2]) - (a[2] * b[1]) + (a[3] * b[0]);
  q[0] = w;
  q[1] = x;
  q[2] = y;
  q[3] = z;
}

void quat_conj(const float q[4], float out[4]) {
  out[0] = q[0];
  out[1] = -q[1];
  out[2] = -q[2];
  out[3] = -q[3];
}

void quat_normalize(float q[4]) {
  const float n2 = (q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]);
  if ((isfinite(n2) == 0) || (n2 < 1e-12F)) {
    quat_identity(q);
  } else {
    const float s = 1.0F / sqrtf(n2);
    for (int i = 0; i < 4; ++i) {
      q[i] *= s;
    }
  }
}

void quat_from_euler(float phi, float theta, float psi, float q[4]) {
  const float cr = cosf(0.5F * phi);
  const float sr = sinf(0.5F * phi);
  const float cp = cosf(0.5F * theta);
  const float sp = sinf(0.5F * theta);
  const float cy = cosf(0.5F * psi);
  const float sy = sinf(0.5F * psi);
  q[0] = (cr * cp * cy) + (sr * sp * sy);
  q[1] = (sr * cp * cy) - (cr * sp * sy);
  q[2] = (cr * sp * cy) + (sr * cp * sy);
  q[3] = (cr * cp * sy) - (sr * sp * cy);
}

float quat_yaw(const float q[4]) {
  return atan2f(2.0F * ((q[0] * q[3]) + (q[1] * q[2])),
                1.0F - (2.0F * ((q[2] * q[2]) + (q[3] * q[3]))));
}

void quat_to_euler(const float q[4], float *phi, float *theta, float *psi) {
  if (phi != NULL) {
    *phi = atan2f(2.0F * ((q[0] * q[1]) + (q[2] * q[3])),
                  1.0F - (2.0F * ((q[1] * q[1]) + (q[2] * q[2]))));
  }
  if (theta != NULL) {
    const float s = 2.0F * ((q[0] * q[2]) - (q[3] * q[1]));
    *theta = (s >= 1.0F) ? HALF_PI_F : ((s <= -1.0F) ? -HALF_PI_F : asinf(s));
  }
  if (psi != NULL) {
    *psi = quat_yaw(q);
  }
}

void quat_from_yaw(float psi, float q[4]) {
  q[0] = cosf(0.5F * psi);
  q[1] = 0.0F;
  q[2] = 0.0F;
  q[3] = sinf(0.5F * psi);
}

void quat_to_dcm(const float q[4], float r[3][3]) {
  const float w = q[0];
  const float x = q[1];
  const float y = q[2];
  const float z = q[3];
  r[0][0] = 1.0F - (2.0F * ((y * y) + (z * z)));
  r[0][1] = 2.0F * ((x * y) - (w * z));
  r[0][2] = 2.0F * ((x * z) + (w * y));
  r[1][0] = 2.0F * ((x * y) + (w * z));
  r[1][1] = 1.0F - (2.0F * ((x * x) + (z * z)));
  r[1][2] = 2.0F * ((y * z) - (w * x));
  r[2][0] = 2.0F * ((x * z) - (w * y));
  r[2][1] = 2.0F * ((y * z) + (w * x));
  r[2][2] = 1.0F - (2.0F * ((x * x) + (y * y)));
}

void quat_rotate(const float q[4], const float v[3], float out[3]) {
  float r[3][3];
  quat_to_dcm(q, r);
  for (int i = 0; i < 3; ++i) {
    out[i] = (r[i][0] * v[0]) + (r[i][1] * v[1]) + (r[i][2] * v[2]);
  }
}

void quat_rotate_inv(const float q[4], const float v[3], float out[3]) {
  float r[3][3];
  quat_to_dcm(q, r);
  for (int i = 0; i < 3; ++i) {
    out[i] = (r[0][i] * v[0]) + (r[1][i] * v[1]) + (r[2][i] * v[2]);
  }
}

void quat_from_rotvec(const float rv[3], float q[4]) {
  const float a2 = (rv[0] * rv[0]) + (rv[1] * rv[1]) + (rv[2] * rv[2]);
  float k;
  if (a2 < 1e-8F) {
    /* sen(a/2)/a por série: 1/2 - a²/48 */
    k = 0.5F - (a2 / 48.0F);
    q[0] = 1.0F - (a2 / 8.0F);
  } else {
    const float a = sqrtf(a2);
    k = sinf(0.5F * a) / a;
    q[0] = cosf(0.5F * a);
  }
  q[1] = k * rv[0];
  q[2] = k * rv[1];
  q[3] = k * rv[2];
}

void quat_to_rotvec(const float q[4], float rv[3]) {
  /* q e -q são a mesma rotação: fica no ramo w >= 0 (ângulo <= π). */
  const float sgn = (q[0] < 0.0F) ? -1.0F : 1.0F;
  const float vx = sgn * q[1];
  const float vy = sgn * q[2];
  const float vz = sgn * q[3];
  const float w = sgn * q[0];
  const float vn = sqrtf((vx * vx) + (vy * vy) + (vz * vz));
  /* ângulo = 2 atan2(|v|, w); fator = ângulo / |v|, com limite 2/w perto de zero */
  const float k = (vn < 1e-6F) ? (2.0F / ((w > 1e-6F) ? w : 1.0F)) : (2.0F * atan2f(vn, w) / vn);
  rv[0] = k * vx;
  rv[1] = k * vy;
  rv[2] = k * vz;
}
