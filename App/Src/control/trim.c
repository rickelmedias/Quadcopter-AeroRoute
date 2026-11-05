/**
 * @file trim.c
 * @brief Ajuste lento de hover e equilíbrio (contrato em control/trim.h).
 */
#include "control/trim.h"

#include <math.h>

/** Sinal do termo de arfagem em u1..u4: frente (u1, u2) +, atrás (u3, u4) -. */
static const float THETA_SIGN[TRIM_INPUTS] = {+1.0F, +1.0F, -1.0F, -1.0F};
/** Sinal do termo de rolagem em u1..u4: esquerda (u1, u4) +, direita (u2, u3) -. */
static const float PHI_SIGN[TRIM_INPUTS] = {+1.0F, -1.0F, -1.0F, +1.0F};

/** @brief Limita @p v a [-limit, limit]. */
static float clamp(float v, float limit) {
  if (v > limit) return limit;
  if (v < -limit) return -limit;
  return v;
}

void trim_reset(trim_state_t *t) {
  t->z = 0.0F;
  t->theta = 0.0F;
  t->phi = 0.0F;
}

/** Erro de altura que ainda conta como "alvo acima", em m, para a rampa de partida. */
#define GROUND_ERROR_M 0.02F

void trim_update(trim_state_t *t, const trim_config_t *cfg, bool enabled, float dt,
                 bool grounded, float vz, float z_error, float phi, float theta) {
  if (!enabled || !isfinite(z_error) || !isfinite(phi) || !isfinite(theta)) return;
  /* Abaixo do alvo (z_error > 0) pede mais rotação. Nariz para cima (theta > 0)
     pede menos na frente; lado direito para baixo (phi > 0) pede menos na esquerda. */
  /* No ar, a altura só integra com o drone assentado: enquanto sobe ou desce
     em rampa, acumular erro faria o hover passar do ponto (windup). */
  float z_step = 0.0F;
  if (grounded && z_error > GROUND_ERROR_M) {
    z_step = cfg->ground_rate * dt;
  } else if (!grounded && fabsf(vz) < cfg->settled_vz) {
    z_step = cfg->ki_z * z_error * dt;
  }
  /* Com a arfagem ou a rolagem no limite, mais empuxo igual nos quatro só
     iria para o lado leve (ensaio de 04/11: a frente subiu e a traseira
     voltou ao chão). O hover então só pode descer até elas saírem do limite. */
  const bool attitude_at_limit = fabsf(t->theta) >= cfg->max_att || fabsf(t->phi) >= cfg->max_att;
  if (attitude_at_limit && z_step > 0.0F) z_step = 0.0F;
  t->z = clamp(t->z + z_step, cfg->max_z);
  t->theta = clamp(t->theta - cfg->ki_att * theta * dt, cfg->max_att);
  t->phi = clamp(t->phi - cfg->ki_att * phi * dt, cfg->max_att);
}

void trim_values(const trim_state_t *t, float out[TRIM_INPUTS]) {
  for (unsigned i = 0U; i < TRIM_INPUTS; ++i) {
    out[i] = t->z + THETA_SIGN[i] * t->theta + PHI_SIGN[i] * t->phi;
  }
}
