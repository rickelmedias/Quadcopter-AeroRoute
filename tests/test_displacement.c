/**
 * @file test_displacement.c
 * @brief Testes de host da atitude e dos estimadores RAW e FUSED.
 */
#include "displacement.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/** Verdadeiro se @p a e @p b diferem no máximo @p tol. */
#define NEAR(a, b, tol) (fabsf((a)-(b)) <= (tol))
/** pi em float. M_PI não existe em C11 estrito; -std=c11 o esconde. */
#define PI_F 3.14159265358979323846F


/**
 * @brief Monta um frame válido: nivelado no yaw dado, 1 m de distância, 10 ms
 *        e qualidade 200.
 *
 * @param flow_x  Fluxo X, em rad/s.
 * @param flow_y  Fluxo Y, em rad/s.
 * @param gyro_x  Giro X, em rad/s.
 * @param gyro_y  Giro Y, em rad/s.
 * @param yaw     Yaw, em rad.
 * @return O frame.
 */
static disp_input_t frame(float flow_x, float flow_y, float gyro_x, float gyro_y, float yaw) {
  disp_input_t in = {0};
  in.flow_x = flow_x; in.flow_y = flow_y;
  in.gyro_x = gyro_x; in.gyro_y = gyro_y;
  /* Atitude nivelada no yaw dado: q = (cos(yaw/2), 0, 0, sin(yaw/2)). */
  in.q[0] = cosf(yaw*0.5F); in.q[1] = 0.0F; in.q[2] = 0.0F; in.q[3] = sinf(yaw*0.5F);
  in.range_m = 1.0F;
  in.dt_s = 0.01F;
  in.quality = 200;
  in.range_valid = true;
  return in;
}

/** @brief Funções de atitude primeiro: tudo o que vem depois confia nelas. */
static void test_attitude(void) {
  const float level[4] = {1.0F, 0.0F, 0.0F, 0.0F};
  euler_t e;
  attitude_to_euler(level, &e);
  assert(NEAR(e.roll, 0.0F, 1e-5F) && NEAR(e.pitch, 0.0F, 1e-5F) && NEAR(e.yaw, 0.0F, 1e-5F));
  assert(NEAR(attitude_cos_tilt(level), 1.0F, 1e-6F));

  /* Yaw de 90 graus. */
  const float yaw90[4] = {0.70710678F, 0.0F, 0.0F, 0.70710678F};
  attitude_to_euler(yaw90, &e);
  assert(NEAR(e.yaw, PI_F/2, 1e-4F));
  assert(NEAR(attitude_cos_tilt(yaw90), 1.0F, 1e-6F));

  /* Rolagem de 90 graus deixa o corpo de lado: cos(inclinação) tem de cair
     a 0. */
  const float roll90[4] = {0.70710678F, 0.70710678F, 0.0F, 0.0F};
  assert(NEAR(attitude_cos_tilt(roll90), 0.0F, 1e-6F));

  /* Um quatérnio um pouco fora da esfera unitária não pode gerar NaN. */
  const float over[4] = {0.0F, 0.0F, 0.7072F, 0.0F};
  attitude_to_euler(over, &e);
  assert(isfinite(e.roll) && isfinite(e.pitch) && isfinite(e.yaw));

  /* Com yaw de 90 graus, o +X do corpo vira o +Y do mundo. */
  float wx, wy;
  attitude_yaw_rotate(PI_F/2, 1.0F, 0.0F, &wx, &wy);
  assert(NEAR(wx, 0.0F, 1e-6F) && NEAR(wy, 1.0F, 1e-6F));
}

/**
 * @brief Translação pura: os dois estimadores precisam concordar entre si e
 *        com a física. 0,5 rad/s a 1 m por 1 s = 0,5 m.
 */
static void test_pure_translation(void) {
  disp_state_t raw = {0}, fused = {0};
  disp_input_t in = frame(0.5F, 0.0F, 0.0F, 0.0F, 0.0F);
  for (int i = 0; i < 100; ++i) {
    disp_raw_update(&raw, &in, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F });
    assert(disp_fused_update(&fused, &in, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));
  }
  assert(NEAR(raw.x, 0.5F, 1e-3F) && NEAR(raw.y, 0.0F, 1e-6F));
  assert(NEAR(fused.x, 0.5F, 1e-3F) && NEAR(fused.y, 0.0F, 1e-4F));
  assert(fused.rejected == 0 && fused.samples == 100);
  /* O incremento informado tem de ser exatamente o quanto o acumulado
     cresceu: 0,5 rad/s * 1 m * 0,01 s = 5 mm por frame. */
  assert(NEAR(raw.last_dx, 0.005F, 1e-6F) && NEAR(raw.last_dy, 0.0F, 1e-9F));
  assert(NEAR(fused.last_dx, 0.005F, 1e-5F));
  assert(NEAR(raw.x, raw.last_dx*100.0F, 1e-3F));
}

/**
 * @brief Rotação não é deslocamento - a razão de ser do módulo.
 *
 * O veículo gira no lugar: a câmera vê fluxo, mas nada transladou. O RAW
 * integra a rotação como deslocamento; o FUSED a cancela e fica parado.
 *
 * O pareamento é no mesmo eixo (gyro_x cancela flow_x, gyro_y cancela
 * flow_y), e não o pareamento cruzado de livro-texto de uma câmera voltada
 * para baixo com eixos alinhados à IMU - um teste de bancada rolando esta
 * montagem no lugar (gyro_x do BNO grande, gyro_y perto de zero) pôs a
 * resposta inteira em flow_x, com sinal oposto, 10 de 10 amostras sem
 * exceção. Ver disp_estimator.c.
 */
static void test_rotation_is_not_travel(void) {
  disp_state_t raw = {0}, fused = {0};
  /* Os dados de bancada pareiam fluxo e giro com sinais opostos nesta placa
     (fx ~= -gx em rolagem pura), então a rotação sintética daqui precisa ser
     montada do mesmo jeito - entradas com o mesmo sinal não seriam um teste
     fisicamente honesto. */
  /* X: os dados de bancada pareiam fluxo e giro com sinal OPOSTO (fx ~= -gx
     em rolagem pura), cancelado por gyro_sign_x=+1. */
  disp_input_t in = frame(0.4F, 0.0F, -0.4F, 0.0F, 0.0F);
  for (int i = 0; i < 100; ++i) {
    disp_raw_update(&raw, &in, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = 1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F });
    assert(disp_fused_update(&fused, &in, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = 1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));
  }
  assert(NEAR(raw.x, 0.4F, 1e-3F));     /* o RAW inventa 400 mm de deslocamento */
  assert(NEAR(fused.x, 0.0F, 1e-4F));   /* o FUSED corretamente informa nenhum */

  /* Y: os dados de bancada pareiam fluxo e giro com o MESMO sinal (fy ~= +gy
     em arfagem pura) - a relação oposta à do X - cancelado por
     gyro_sign_y=-1. Essa assimetria é real, não erro de digitação: ver
     disp_estimator.c. */
  disp_state_t r2 = {0}, f2 = {0};
  disp_input_t roll = frame(0.0F, 0.3F, 0.0F, 0.3F, 0.0F);
  for (int i = 0; i < 100; ++i) {
    disp_raw_update(&r2, &roll, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = 1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F });
    assert(disp_fused_update(&f2, &roll, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = 1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));
  }
  assert(NEAR(r2.y, 0.3F, 1e-3F));
  assert(NEAR(f2.y, 0.0F, 1e-4F));
}

/**
 * @brief Ida e volta no mesmo eixo do corpo, girando 180 graus no meio.
 *
 * No referencial do mundo o veículo termina onde começou; o RAW, que nunca
 * sai dos eixos do corpo, dobra a distância em vez de cancelá-la.
 */
static void test_yaw_rotation_into_world_frame(void) {
  disp_state_t raw = {0}, fused = {0};
  disp_input_t out = frame(0.5F, 0.0F, 0.0F, 0.0F, 0.0F);
  disp_input_t back = frame(0.5F, 0.0F, 0.0F, 0.0F, PI_F);
  for (int i = 0; i < 100; ++i) {
    disp_raw_update(&raw, &out, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F });
    assert(disp_fused_update(&fused, &out, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));
  }
  for (int i = 0; i < 100; ++i) {
    disp_raw_update(&raw, &back, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F });
    assert(disp_fused_update(&fused, &back, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));
  }
  assert(NEAR(raw.x, 1.0F, 1e-3F));   /* eixos do corpo: 0,5 + 0,5 */
  assert(NEAR(fused.x, 0.0F, 2e-3F)); /* eixos do mundo: 0,5 - 0,5 */
}

/**
 * @brief A escala corrige um erro sistemático: um movimento real de 300 mm
 *        lido como 350 mm é corrigido por 300/350, igualmente nos dois
 *        estimadores.
 */
static void test_flow_scale(void) {
  disp_state_t raw = {0}, fused = {0};
  disp_input_t in = frame(0.35F, 0.0F, 0.0F, 0.0F, 0.0F);
  const float scale = 300.0F/350.0F;
  for (int i = 0; i < 100; ++i) {
    disp_raw_update(&raw, &in, &(disp_config_t){ .flow_scale_x = scale, .flow_scale_y = scale, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F });
    assert(disp_fused_update(&fused, &in, &(disp_config_t){ .flow_scale_x = scale, .flow_scale_y = scale, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));
  }
  assert(NEAR(raw.x, 0.3F, 1e-3F));
  assert(NEAR(fused.x, 0.3F, 1e-3F));
}

/**
 * @brief Todo filtro de validade precisa rejeitar em vez de integrar, e a
 *        rejeição nunca pode mexer no acumulado. O RAW não tem filtros, por
 *        projeto.
 */
static void test_gates(void) {
  disp_state_t s = {0};
  const float ok_x = 0.5F;

  disp_input_t bad = frame(ok_x, 0.0F, 0.0F, 0.0F, 0.0F);
  bad.quality = 5;
  assert(!disp_fused_update(&s, &bad, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));

  bad = frame(ok_x, 0.0F, 0.0F, 0.0F, 0.0F); bad.range_valid = false;
  assert(!disp_fused_update(&s, &bad, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));

  bad = frame(ok_x, 0.0F, 0.0F, 0.0F, 0.0F); bad.range_m = 0.0F;
  assert(!disp_fused_update(&s, &bad, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));

  bad = frame(ok_x, 0.0F, 0.0F, 0.0F, 0.0F); bad.range_m = 50.0F;
  assert(!disp_fused_update(&s, &bad, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));

  bad = frame(ok_x, 0.0F, 0.0F, 0.0F, 0.0F); bad.dt_s = 2.0F;   /* frames perdidos */
  assert(!disp_fused_update(&s, &bad, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));

  bad = frame(ok_x, 0.0F, 0.0F, 0.0F, 0.0F); bad.dt_s = 0.0F;
  assert(!disp_fused_update(&s, &bad, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));

  bad = frame(99.0F, 0.0F, 0.0F, 0.0F, 0.0F);                    /* taxa corrompida */
  assert(!disp_fused_update(&s, &bad, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));

  bad = frame(NAN, 0.0F, 0.0F, 0.0F, 0.0F);
  assert(!disp_fused_update(&s, &bad, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));

  /* Rolagem de 90 graus: cos(inclinação) é 0, bem abaixo do limite. */
  bad = frame(ok_x, 0.0F, 0.0F, 0.0F, 0.0F);
  bad.q[0] = 0.70710678F; bad.q[1] = 0.70710678F; bad.q[2] = 0.0F; bad.q[3] = 0.0F;
  assert(!disp_fused_update(&s, &bad, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));

  assert(s.samples == 0 && s.rejected == 9);
  assert(s.x == 0.0F && s.y == 0.0F);
  /* Um frame rejeitado também não pode deixar um incremento velho para trás. */
  assert(s.last_dx == 0.0F && s.last_dy == 0.0F);
}

/**
 * @brief Com o sinal do giroscópio errado, a rotação quase dobra em vez de
 *        ser cancelada. É exatamente o que o procedimento de bancada procura.
 */
static void test_wrong_gyro_sign_is_visible(void) {
  disp_state_t right = {0}, wrong = {0};
  disp_input_t in = frame(0.4F, 0.0F, -0.4F, 0.0F, 0.0F);
  for (int i = 0; i < 100; ++i) {
    assert(disp_fused_update(&right, &in, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = +1.0F, .gyro_sign_y = +1.0F, .deadband_rad_s = 0.0F }));
    assert(disp_fused_update(&wrong, &in, &(disp_config_t){ .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F }));
  }
  assert(NEAR(right.x, 0.0F, 1e-4F));
  assert(NEAR(wrong.x, 0.8F, 1e-3F));
}

/**
 * @brief Regressão de um defeito exposto pelo log de bancada: fluxo
 *        quantizado a zero não pode inventar deslocamento.
 *
 * O MTF-01 só resolve o fluxo em passos de ::DISP_FLOW_LSB_RAD_S, então uma
 * placa girando mais devagar que isso informa fluxo 0,0 enquanto o
 * giroscópio ainda lê a taxa real. Sem proteção, a compensação lê essa
 * diferença como translação e inventa deslocamento a partir de um sensor que
 * disse que nada se moveu - frames reais produziam ~3 mm cada.
 */
static void test_quantized_flow_does_not_invent_travel(void) {
  disp_config_t cfg = { .flow_scale_x = 1.0F, .flow_scale_y = 1.0F, .gyro_sign_x = -1.0F, .gyro_sign_y = -1.0F, .deadband_rad_s = 0.0F };

  /* Girando a 0,15 rad/s, abaixo do passo de 0,2 rad/s do sensor, então o
     fluxo lê 0 enquanto o giroscópio ainda vê o giro. Isso não pode gerar
     deslocamento nenhum. */
  disp_state_t still = {0};
  disp_input_t turning = frame(0.0F, 0.0F, 0.0F, 0.15F, 0.0F);
  turning.range_m = 0.84F;
  for (int i = 0; i < 100; ++i) assert(disp_fused_update(&still, &turning, &cfg));
  assert(still.x == 0.0F && still.y == 0.0F);

  /* O mesmo para um giro grande o bastante para passar pela antiga zona
     morta por magnitude: uma leitura de fluxo zero continua sendo movimento
     zero. */
  disp_state_t fast = {0};
  disp_input_t spinning = frame(0.0F, 0.0F, 0.9F, 0.0F, 0.0F);
  spinning.range_m = 0.84F;
  for (int i = 0; i < 100; ++i) assert(disp_fused_update(&fast, &spinning, &cfg));
  assert(fast.x == 0.0F && fast.y == 0.0F);

  /* Confiar no zero do sensor não pode engolir movimento que ele realmente
     resolveu: uma contagem inteira de translação real ainda precisa ser
     integrada, e também uma contagem enquanto a placa gira. */
  disp_state_t real = {0};
  disp_input_t moving = frame(DISP_FLOW_LSB_RAD_S, 0.0F, 0.0F, 0.0F, 0.0F);
  moving.range_m = 0.84F;
  for (int i = 0; i < 100; ++i) assert(disp_fused_update(&real, &moving, &cfg));
  assert(NEAR(real.x, DISP_FLOW_LSB_RAD_S*0.84F, 1e-3F));

  /* E a compensação ainda precisa funcionar numa leitura resolvida: duas
     contagens de fluxo contra uma contagem de giro correspondente deixam uma
     contagem de deslocamento. */
  disp_state_t comp = {0};
  disp_input_t both = frame(2.0F*DISP_FLOW_LSB_RAD_S, 0.0F, DISP_FLOW_LSB_RAD_S, 0.0F, 0.0F);
  both.range_m = 0.84F;
  for (int i = 0; i < 100; ++i) assert(disp_fused_update(&comp, &both, &cfg));
  assert(NEAR(comp.x, DISP_FLOW_LSB_RAD_S*0.84F, 1e-3F));
}

/** @brief Roda todos os testes; qualquer falha interrompe pelo assert. */
int main(void) {
  test_attitude();
  test_pure_translation();
  test_rotation_is_not_travel();
  test_yaw_rotation_into_world_frame();
  test_flow_scale();
  test_gates();
  test_wrong_gyro_sign_is_visible();
  test_quantized_flow_does_not_invent_travel();
  puts("Displacement: attitude, pure translation, rotation rejection, yaw-to-world, "
       "scale correction, all gates, gyro-sign detection, quantization deadband and\n"
       "per-frame increments passed");
}
