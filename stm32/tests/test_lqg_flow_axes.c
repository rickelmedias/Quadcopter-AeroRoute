#include "control/lqg_meas.h"
#include "control/lqg_config.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void) {
  lqg_meas_t state;
  lqg_meas_reset(&state);
  lqg_sensors_t in = {0};
  lqg_meas_out_t out;
  in.roll = LQG_ZERO_ROLL_DEG * 0.017453293F;
  in.pitch = LQG_ZERO_PITCH_DEG * 0.017453293F;
  in.attitude_valid = true;
  in.gyro_valid = true;
  in.range_valid = true;
  in.flow_valid = true;
  in.flow_quality = 100U;
  in.range_m = 1.0F;

  /* Canais brutos MTF: Y+ frente, Y- trás, X- direita, X+ esquerda. */
  in.flow_y = 1.0F;
  lqg_meas_update(&state, &in, &out);
  assert((out.flags & LQG_MEAS_FLOW) && out.flow_body[0] > 0.9F);
  assert(fabsf(out.flow_body[1]) < 1e-5F && out.y[LQG_Y_U] > 0.9F);
  in.flow_y = -1.0F;
  lqg_meas_update(&state, &in, &out);
  assert(out.flow_body[0] < -0.9F);
  in.flow_y = 0.0F;
  in.flow_x = -1.0F;
  lqg_meas_update(&state, &in, &out);
  assert(out.flow_body[1] > 0.9F && out.y[LQG_Y_V] > 0.9F);
  assert(fabsf(out.flow_body[0]) < 1e-5F);
  in.flow_x = 1.0F;
  lqg_meas_update(&state, &in, &out);
  assert(out.flow_body[1] < -0.9F);

  /* Fluxo só acima de 10 cm do chão do armamento, com histerese até 7 cm. */
  lqg_meas_set_ground(&state, 0.08F);
  in.flow_x = 0.0F;
  in.flow_y = 1.0F;
  in.range_m = 0.12F; /* 4 cm acima do chão */
  lqg_meas_update(&state, &in, &out);
  assert(!(out.flags & LQG_MEAS_FLOW) && !(out.flags & LQG_MEAS_FLOW_HEIGHT));
  in.range_m = 0.19F;
  lqg_meas_update(&state, &in, &out);
  assert((out.flags & LQG_MEAS_FLOW) && (out.flags & LQG_MEAS_FLOW_HEIGHT));
  in.range_m = 0.165F; /* 8,5 cm: ainda vale (histerese) */
  lqg_meas_update(&state, &in, &out);
  assert(out.flags & LQG_MEAS_FLOW);
  in.range_m = 0.14F;
  lqg_meas_update(&state, &in, &out);
  assert(!(out.flags & LQG_MEAS_FLOW));

  /* Quadro repetido do MTF: medidas valem, mas não são novas (o filtro não reusa). */
  in.range_m = 0.30F;
  in.mtf_frames = 7U;
  lqg_meas_update(&state, &in, &out);
  assert((out.flags & LQG_MEAS_FRESH) && (out.flags & LQG_MEAS_FLOW));
  lqg_meas_update(&state, &in, &out);
  assert(!(out.flags & LQG_MEAS_FRESH) && !(out.flags & LQG_MEAS_FLOW) && (out.flags & LQG_MEAS_RANGE));

  /* Braço do MTF: o drone inclina no chão sem subir; o sensor fora do centro sobe
     LQG_RANGE_POS_X_M·sen θ e a altura corrigida não muda. */
  lqg_meas_reset(&state);
  in.mtf_frames = 0U;
  const float ground = 0.08F;
  const float th = 0.10F; /* nariz para cima (θ positivo) */
  in.roll = (LQG_ZERO_ROLL_DEG * 0.017453293F) + (th / LQG_SGN_PITCH);
  in.pitch = LQG_ZERO_PITCH_DEG * 0.017453293F;
  in.range_m = (ground + (LQG_RANGE_POS_X_M * sinf(th))) / cosf(th);
  lqg_meas_update(&state, &in, &out);
  assert(fabsf(out.y[LQG_Y_THETA] - th) < 1e-4F);
  assert(fabsf(out.range_corrected_m - ground) < 1e-4F);
  puts("test_lqg_flow_axes ok");
  return 0;
}
