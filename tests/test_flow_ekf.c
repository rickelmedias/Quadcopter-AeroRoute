/**
 * @file test_flow_ekf.c
 * @brief Testes de host do filtro de Kalman do fluxo.
 */
#include "flow_ekf.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/** Verdadeiro se @p a e @p b diferem no máximo @p tol. */
#define NEAR(a, b, tol) (fabsf((a)-(b)) <= (tol))
/** Intervalo entre frames nos testes, em s (100 Hz). */
#define DT 0.01F
/** Distância usada nos testes, em m. */
#define RANGE 0.8F

/** Parâmetros usados em todos os testes (os padrões). */
static flow_ekf_config_t cfg;

/**
 * @brief Confere que o estado é finito e a covariância simétrica e positiva.
 *
 * Se não for, todo número que o filtro produzir depois não tem significado.
 *
 * @param f  Filtro a conferir.
 */
static void assert_sane(const flow_ekf_t *f) {
  for (int i = 0; i < 2; ++i) {
    assert(isfinite(f->axis[i].p) && isfinite(f->axis[i].v));
    assert(f->axis[i].P00 >= 0.0F && f->axis[i].P11 >= 0.0F);
    /* A correlação não pode passar da média geométrica das variâncias. */
    assert(f->axis[i].P01*f->axis[i].P01 <= f->axis[i].P00*f->axis[i].P11 + 1e-6F);
  }
}

/** @brief Um veículo parado medido por um sensor parado precisa ficar na origem. */
static void test_stationary(void) {
  flow_ekf_t f;
  flow_ekf_reset(&f);
  for (int i = 0; i < 6000; ++i) {
    flow_ekf_predict(&f, 0.0F, 0.0F, DT, &cfg);
    flow_ekf_update(&f, 0.0F, 0.0F, RANGE, &cfg);
  }
  assert_sane(&f);
  assert(NEAR(f.axis[0].p, 0.0F, 1e-4F) && NEAR(f.axis[1].p, 0.0F, 1e-4F));
  assert(f.converged);
}

/**
 * @brief A afirmação em que o filtro se apoia: velocidade sem viés sob
 *        rejeição.
 *
 * Entra um sinal de média zero com a oscilação da quantização, e um
 * subconjunto pseudoaleatório é descartado para imitar o gate. Duas coisas
 * precisam valer: a velocidade estimada fica perto de zero e bem mais quieta
 * que as amostras brutas, e os descartes não a enviesam.
 *
 * A posição NÃO é conferida aqui, de propósito. Ela não é observável só pela
 * velocidade, então faz passeio aleatório tanto no filtro quanto no
 * integrador - medido em 65 mm contra 59 mm nestes mesmos 60 s. Afirmar o
 * contrário seria uma mentira em torno da qual os testes teriam de ser
 * escritos.
 */
static void test_velocity_is_unbiased_under_rejection(void) {
  flow_ekf_t f;
  flow_ekf_reset(&f);
  const float step = 0.2F * RANGE; /* uma contagem de fluxo, em m/s */
  uint32_t rng = 12345;
  int rejected = 0, accepted = 0;
  double sum_v = 0.0, sum_v2 = 0.0;

  for (int i = 0; i < 6000; ++i) {
    const float v = (i & 1) ? step : -step; /* média exatamente zero */
    rng = rng*1664525U + 1013904223U;
    const bool gated = (rng >> 28) == 0;    /* cerca de um frame em dezesseis */

    flow_ekf_predict(&f, 0.0F, 0.0F, DT, &cfg);
    if (gated) {
      ++rejected;
    } else {
      flow_ekf_update(&f, v, 0.0F, RANGE, &cfg);
      ++accepted;
    }
    sum_v += f.axis[0].v;
    sum_v2 += (double)f.axis[0].v * f.axis[0].v;
  }
  assert_sane(&f);
  assert(rejected > 200 && accepted > 5000);

  const double mean = sum_v / 6000.0;
  const double rms = sqrt(sum_v2 / 6000.0);
  /* Sem viés: a média estimada fica três ordens abaixo da oscilação de
     entrada. */
  assert(fabs(mean) < 0.01);
  /* E quieta: pelo menos dez vezes menos ruído que as amostras brutas. */
  assert(rms < step / 10.0);
}

/** @brief Movimento real constante precisa ser acompanhado, não filtrado fora. */
static void test_tracks_constant_velocity(void) {
  flow_ekf_t f;
  flow_ekf_reset(&f);
  const float v = 0.25F;
  for (int i = 0; i < 2000; ++i) {
    flow_ekf_predict(&f, 0.0F, 0.0F, DT, &cfg);
    flow_ekf_update(&f, v, 0.0F, RANGE, &cfg);
  }
  assert_sane(&f);
  assert(NEAR(f.axis[0].v, v, 0.02F));
  /* 20 s a 0,25 m/s são 5 m; com folga para o transitório de convergência. */
  assert(NEAR(f.axis[0].p, v*20.0F, 0.1F));
}

/** @brief A aceleração sentida pela IMU precisa mover a predição mesmo sem fluxo. */
static void test_prediction_uses_imu(void) {
  flow_ekf_t f;
  flow_ekf_reset(&f);
  for (int i = 0; i < 100; ++i) flow_ekf_predict(&f, 2.0F, 0.0F, DT, &cfg);
  assert_sane(&f);
  assert(NEAR(f.axis[0].v, 2.0F, 1e-3F));  /* 2 m/s^2 por 1 s */
  assert(NEAR(f.axis[0].p, 1.0F, 1e-2F));  /* 0,5*a*t^2 */
  /* Sem nada para corrigir, a incerteza tem de ter crescido. */
  assert(f.axis[0].P11 > 0.0F && !f.converged);
}

/**
 * @brief O gate precisa rejeitar uma medida absurda, e a rejeição precisa
 *        deixar a predição intacta em vez de arrastar o estado.
 */
static void test_gate_rejects_outlier(void) {
  flow_ekf_t f;
  flow_ekf_reset(&f);
  for (int i = 0; i < 1000; ++i) {
    flow_ekf_predict(&f, 0.0F, 0.0F, DT, &cfg);
    flow_ekf_update(&f, 0.0F, 0.0F, RANGE, &cfg);
  }
  assert(f.converged);
  const float v_before = f.axis[0].v;
  const uint32_t rejected_before = f.rejected;

  /* Uma mão passando sob o sensor: metros por segundo do nada. */
  flow_ekf_predict(&f, 0.0F, 0.0F, DT, &cfg);
  assert(!flow_ekf_update(&f, 2.5F, 0.0F, RANGE, &cfg));
  assert(f.rejected == rejected_before + 1);
  assert(f.test_ratio[0] > 1.0F);
  assert(NEAR(f.axis[0].v, v_before, 1e-3F));
  assert_sane(&f);
}

/**
 * @brief Uma velocidade nova e sustentada precisa acabar sendo aceita: o gate
 *        se alarga conforme a incerteza da predição cresce, então o filtro
 *        não consegue se trancar para fora.
 */
static void test_gate_eventually_admits_real_change(void) {
  flow_ekf_t f;
  flow_ekf_reset(&f);
  for (int i = 0; i < 1000; ++i) {
    flow_ekf_predict(&f, 0.0F, 0.0F, DT, &cfg);
    flow_ekf_update(&f, 0.0F, 0.0F, RANGE, &cfg);
  }
  int accepted = 0;
  for (int i = 0; i < 500; ++i) {
    flow_ekf_predict(&f, 0.0F, 0.0F, DT, &cfg);
    if (flow_ekf_update(&f, 1.2F, 0.0F, RANGE, &cfg)) ++accepted;
  }
  assert(accepted > 100);
  assert(NEAR(f.axis[0].v, 1.2F, 0.05F));
  assert_sane(&f);
}

/**
 * @brief Os dois eixos são uma observação de um único vetor, então uma
 *        medida ruim em qualquer eixo não pode ser fundida pela metade.
 */
static void test_gate_is_per_measurement_not_per_axis(void) {
  flow_ekf_t f;
  flow_ekf_reset(&f);
  for (int i = 0; i < 1000; ++i) {
    flow_ekf_predict(&f, 0.0F, 0.0F, DT, &cfg);
    flow_ekf_update(&f, 0.0F, 0.0F, RANGE, &cfg);
  }
  const float vy_before = f.axis[1].v;
  flow_ekf_predict(&f, 0.0F, 0.0F, DT, &cfg);
  assert(!flow_ekf_update(&f, 5.0F, 0.0F, RANGE, &cfg)); /* só o x é absurdo */
  assert(NEAR(f.axis[1].v, vy_before, 1e-4F));           /* o y fica intacto */
}

/** @brief Entradas sem sentido não podem envenenar a covariância. */
static void test_rejects_nonsense_inputs(void) {
  flow_ekf_t f;
  flow_ekf_reset(&f);
  flow_ekf_predict(&f, NAN, 0.0F, DT, &cfg);
  flow_ekf_predict(&f, 0.0F, 0.0F, 0.0F, &cfg);
  flow_ekf_predict(&f, 0.0F, 0.0F, -1.0F, &cfg);
  assert(!flow_ekf_update(&f, NAN, 0.0F, RANGE, &cfg));
  assert(!flow_ekf_update(&f, 0.0F, 0.0F, 0.0F, &cfg));
  assert(!flow_ekf_update(&f, 0.0F, 0.0F, -1.0F, &cfg));
  assert_sane(&f);
  assert(f.axis[0].p == 0.0F && f.axis[0].v == 0.0F);
}

/** @brief Roda todos os testes; qualquer falha interrompe pelo assert. */
int main(void) {
  cfg = FLOW_EKF_DEFAULTS;
  test_stationary();
  test_velocity_is_unbiased_under_rejection();
  test_tracks_constant_velocity();
  test_prediction_uses_imu();
  test_gate_rejects_outlier();
  test_gate_eventually_admits_real_change();
  test_gate_is_per_measurement_not_per_axis();
  test_rejects_nonsense_inputs();
  puts("Flow EKF: stationary, unbiased velocity under rejection, constant velocity,\n"
       "IMU prediction, "
       "outlier gating, recovery from a real change, whole-vector gating and bad input passed");
}
