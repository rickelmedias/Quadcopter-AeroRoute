/**
 * @file test_lqg_meas_q.c
 * @brief Quatérnio do BNO direto (sem Euler) contra o mapa de Euler da submissão.
 *
 * O sensor simulado usa o mapa inverso de control/lqg_config.h (mesmo de
 * lqg_sim.c): roll cru = θ + zero, pitch cru = φ + zero, yaw cru = −ψ. O caminho
 * em quatérnio (WORLD ⊗ q ⊗ MOUNT) tem de reproduzir os ângulos do mapa de Euler:
 * exato em um eixo, e com erro de segunda ordem (φθ/2) nas inclinações combinadas.
 */
#include "control/lqg_config.h"
#include "control/lqg_meas.h"
#include "control/quat.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEG 0.017453293F

/** @brief Sensor no estado (φ, θ, ψ): ângulos crus e quatérnio coerentes. */
static void sensors(float phi, float theta, float psi, float yaw0, lqg_sensors_t *in) {
  (void)memset(in, 0, sizeof(*in));
  in->pitch = (phi / LQG_SGN_ROLL) + (LQG_ZERO_PITCH_DEG * DEG);
  in->roll = (theta / LQG_SGN_PITCH) + (LQG_ZERO_ROLL_DEG * DEG);
  in->yaw = yaw0 + (psi / LQG_SGN_YAW);
  quat_from_euler(in->roll, in->pitch, in->yaw, in->q);
  in->attitude_valid = true;
  in->q_valid = true;
  in->att_accuracy = 3U;
}

/** @brief Ângulo (rad) entre duas atitudes. */
static float angle_between(const float a[4], const float b[4]) {
  float ai[4];
  float d[4];
  float rv[3];
  quat_conj(a, ai);
  quat_mul(ai, b, d);
  quat_to_rotvec(d, rv);
  return sqrtf((rv[0] * rv[0]) + (rv[1] * rv[1]) + (rv[2] * rv[2]));
}

/** @brief Compara o caminho nativo com o de Euler no estado dado; devolve o erro em rad. */
static float compare(float phi, float theta, float psi) {
  lqg_meas_t mq;
  lqg_meas_t me;
  lqg_meas_out_t oq;
  lqg_meas_out_t oe;
  lqg_sensors_t in;
  lqg_meas_reset(&mq);
  lqg_meas_reset(&me);
  /* Primeiro passo nivelado: fixa a guinada de referência dos dois caminhos. */
  sensors(0.0F, 0.0F, 0.0F, 0.7F, &in);
  lqg_meas_update(&mq, &in, &oq);
  in.q_valid = false;
  lqg_meas_update(&me, &in, &oe);
  assert(oq.q_native && !oe.q_native);
  assert(angle_between(oq.q, oe.q) < 1e-4F); /* parado e nivelado: identidade nos dois */
  sensors(phi, theta, psi, 0.7F, &in);
  lqg_meas_update(&mq, &in, &oq);
  in.q_valid = false;
  lqg_meas_update(&me, &in, &oe);
  assert(oq.q_native && !oe.q_native && ((oq.flags & LQG_MEAS_ATTITUDE) != 0U));
  /* O legado (y[]) e o caminho nativo descrevem a mesma atitude. */
  return angle_between(oq.q, oe.q);
}

/** Desvio de montagem do zero da submissão (φ0, θ0), em rad: entra no termo de segunda ordem. */
#define PHI0   (2.9F * DEG)
#define THETA0 (5.5F * DEG)

/** @brief Limite do erro de segunda ordem do mapa de Euler: (|φ|+φ0)(|θ|+θ0): o comutador de duas rotações. */
static float second_order_bound(float phi, float theta) {
  return (1.0F * (fabsf(phi) + PHI0) * (fabsf(theta) + THETA0)) + 5e-4F;
}

static void test_equivalence(void) {
  /* O erro é desvio de montagem × inclinação: some com a inclinação. */
  assert(compare(0.02F, 0.0F, 0.0F) < second_order_bound(0.02F, 0.0F));
  assert(compare(0.0F, 0.02F, 0.0F) < second_order_bound(0.0F, 0.02F));
  assert(compare(0.002F, 0.0F, 0.0F) < 2e-4F);
  /* Guinada pura: exata, qualquer valor. */
  assert(compare(0.0F, 0.0F, 0.5F) < 5e-4F);
  assert(compare(0.0F, 0.0F, -2.5F) < 5e-4F);
  const float angles[7] = {-0.25F, -0.15F, -0.05F, 0.0F, 0.08F, 0.15F, 0.25F};
  float worst = 0.0F;
  for (uint32_t i = 0U; i < 7U; ++i) {
    for (uint32_t j = 0U; j < 7U; ++j) {
      for (float psi = -3.0F; psi < 3.1F; psi += 1.5F) {
        const float e = compare(angles[i], angles[j], psi);
        assert(e < second_order_bound(angles[i], angles[j]));
        worst = (e > worst) ? e : worst;
      }
    }
  }
  printf("equivalência: pior erro %.4f rad (%.2f°) com até 14,3° de inclinação\n", worst,
         worst / DEG);
}

static void test_signs(void) {
  lqg_meas_t m;
  lqg_meas_out_t o;
  lqg_sensors_t in;
  float phi;
  float theta;
  float psi;
  lqg_meas_reset(&m);
  sensors(0.0F, 0.0F, 0.0F, -1.0F, &in);
  lqg_meas_update(&m, &in, &o);
  /* Lado direito para baixo -> φ > 0; nariz para cima -> θ > 0; horário visto de cima -> ψ > 0. */
  sensors(0.12F, 0.0F, 0.0F, -1.0F, &in);
  lqg_meas_update(&m, &in, &o);
  quat_to_euler(o.q, &phi, &theta, &psi);
  assert((fabsf(phi - 0.12F) < 1e-3F) && (fabsf(theta) < 1e-3F) && (fabsf(psi) < 1e-3F));
  sensors(0.0F, 0.09F, 0.0F, -1.0F, &in);
  lqg_meas_update(&m, &in, &o);
  quat_to_euler(o.q, &phi, &theta, &psi);
  assert((fabsf(phi) < 1e-3F) && (fabsf(theta - 0.09F) < 1e-3F));
  sensors(0.0F, 0.0F, 0.6F, -1.0F, &in);
  lqg_meas_update(&m, &in, &o);
  quat_to_euler(o.q, &phi, &theta, &psi);
  assert(fabsf(psi - 0.6F) < 1e-3F);
  puts("sinais ok");
}

static void test_level_and_fallback(void) {
  lqg_meas_t m;
  lqg_meas_out_t o;
  lqg_sensors_t in;
  lqg_meas_reset(&m);
  /* Nível do voo: a placa lê φ = 0,03 e θ = −0,02 no chão; o zero do armamento tira isso. */
  sensors(0.03F, -0.02F, 0.0F, 0.2F, &in);
  lqg_meas_update(&m, &in, &o);
  assert(o.q_native);
  assert(fabsf(o.tilt_raw[0] - 0.03F) < 2e-3F);
  assert(fabsf(o.tilt_raw[1] + 0.02F) < 2e-3F);
  const float rv[3] = {o.tilt_raw[0], o.tilt_raw[1], 0.0F};
  lqg_meas_set_level_q(&m, rv);
  lqg_meas_update(&m, &in, &o);
  float nil[4] = {1.0F, 0.0F, 0.0F, 0.0F};
  assert(angle_between(o.q, nil) < 1e-3F);
  /* Quatérnio fora da esfera unitária: volta ao caminho de Euler. */
  in.q[0] = 0.1F;
  in.q[1] = in.q[2] = in.q[3] = 0.1F;
  lqg_meas_update(&m, &in, &o);
  assert(!o.q_native && ((o.flags & LQG_MEAS_ATTITUDE) != 0U) && (o.att_accuracy == 3U));
  /* Sem q_valid: idem. */
  sensors(0.0F, 0.0F, 0.0F, 0.2F, &in);
  in.q_valid = false;
  lqg_meas_update(&m, &in, &o);
  assert(!o.q_native);
  /* A precisão do BNO passa para a saída (0..3), saturada. */
  sensors(0.0F, 0.0F, 0.0F, 0.2F, &in);
  in.att_accuracy = 1U;
  lqg_meas_update(&m, &in, &o);
  assert(o.q_native && (o.att_accuracy == 1U));
  in.att_accuracy = 9U;
  lqg_meas_update(&m, &in, &o);
  assert(o.att_accuracy == 3U);
  puts("nível e reserva ok");
}

int main(void) {
  test_equivalence();
  test_signs();
  test_level_and_fallback();
  puts("test_lqg_meas_q ok");
  return 0;
}
