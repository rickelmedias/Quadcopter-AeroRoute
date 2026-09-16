/**
 * @file test_flow_calibrator.c
 * @brief Testes de host da calibração de escala do fluxo.
 */
#include "flow_calibrator.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/** Verdadeiro se @p a e @p b diferem no máximo @p tol. */
#define NEAR(a, b, tol) (fabsf((a)-(b)) <= (tol))

/**
 * @brief Ruído pseudoaleatório determinístico.
 *
 * Mesmo estilo de gerador (LCG) usado em tests/test_mtf.c, para que as
 * execuções sejam reproduzíveis em qualquer máquina.
 *
 * @param rng        Estado do gerador (atualizado).
 * @param amplitude  Amplitude máxima.
 * @return Valor uniforme em [-amplitude, amplitude).
 */
static float noise(uint32_t *rng, float amplitude) {
  *rng = *rng * 1664525U + 1013904223U;
  float u = (float)(*rng >> 8) / (float)(1U << 24); /* [0,1) */
  return (u * 2.0F - 1.0F) * amplitude;
}

/** @brief Filtros básicos: rotação lenta demais ou yaw rápido demais não pode ser aceito. */
static void test_gates(void) {
  flow_calibrator_t c;
  flow_cal_reset(&c);

  /* Abaixo do piso de 20 graus/s nos dois lados. */
  assert(!flow_cal_add_sample(&c, 0, 0.1F, 0.1F, 0.0F));
  /* Rotação rápida o bastante, mas o yaw simultâneo a desqualifica. */
  assert(!flow_cal_add_sample(&c, 0, 0.5F, 0.5F, 0.6F));
  /* Bruto rápido o bastante, esperado (vindo do giro) lento demais: ainda
     filtrada. */
  assert(!flow_cal_add_sample(&c, 0, 0.5F, 0.1F, 0.0F));
  assert(c.axis[0].samples == 0);

  /* Agora uma amostra que passa por todos os filtros. */
  assert(flow_cal_add_sample(&c, 0, 0.5F, 0.5F, 0.1F));
  assert(c.axis[0].samples == 1);
}

/**
 * @brief Sem ruído e com erro de escala conhecido, a calibração precisa
 *        recuperar a correção exata.
 *
 * Inclui uma manobra que balança nos dois sentidos (sinal alternado), que é
 * como o procedimento real de bancada (balançar rolagem/arfagem para lá e
 * para cá) realmente aparece.
 */
static void test_recovers_known_scale_no_noise(void) {
  flow_calibrator_t c;
  flow_cal_reset(&c);
  const float true_correction = 1.27F; /* corresponde à leitura 0,79 medida na bancada */
  for (int i = 0; i < 200; ++i) {
    float expected = ((i % 2) ? 1.0F : -1.0F) * (0.5F + 0.02F * (float)(i % 10));
    float raw = expected / true_correction; /* o sensor lê a menos nesta proporção */
    assert(flow_cal_add_sample(&c, 0, raw, expected, 0.0F));
  }
  float scalar, residual;
  assert(flow_cal_result(&c, 0, &scalar, &residual));
  assert(NEAR(scalar, true_correction, 1e-4F));
  assert(residual > 0.0F); /* o fator 1 realmente errava, então isto não pode ser zero */
}

/**
 * @brief O mesmo cenário com ruído em cada leitura bruta: a média de muitas
 *        amostras ainda precisa cair perto da correção verdadeira.
 *
 * É o mesmo tipo de propriedade de "a média apaga o ruído de quantização" em
 * que o EKF se apoia.
 */
static void test_recovers_known_scale_with_noise(void) {
  flow_calibrator_t c;
  flow_cal_reset(&c);
  const float true_correction = 1.27F;
  uint32_t rng = 42;
  for (int i = 0; i < 2000; ++i) {
    float expected = ((i % 2) ? 1.0F : -1.0F) * (0.5F + 0.3F * (float)(i % 5));
    float raw = expected / true_correction + noise(&rng, 0.05F);
    flow_cal_add_sample(&c, 0, raw, expected, 0.0F);
  }
  float scalar;
  assert(flow_cal_result(&c, 0, &scalar, NULL));
  assert(NEAR(scalar, true_correction, 0.05F));
}

/**
 * @brief Uma manobra que implicaria correção absurda (sensor ligado ao
 *        contrário, eixo errado ou defeito) precisa ser rejeitada em vez de
 *        aceita em silêncio.
 */
static void test_rejects_implausible_scale(void) {
  flow_calibrator_t c;
  flow_cal_reset(&c);
  for (int i = 0; i < 50; ++i) {
    float expected = 0.5F;
    float raw = 0.05F; /* implica correção de 10x, fora de [0,2; 4,0] */
    flow_cal_add_sample(&c, 0, raw, expected, 0.0F);
  }
  float scalar;
  assert(!flow_cal_result(&c, 0, &scalar, NULL));
}

/** @brief Sem amostras não há opinião, e cada eixo é independente do outro. */
static void test_no_samples_and_axis_independence(void) {
  flow_calibrator_t c;
  flow_cal_reset(&c);
  float scalar;
  assert(!flow_cal_result(&c, 0, &scalar, NULL));
  assert(!flow_cal_result(&c, 1, &scalar, NULL));

  for (int i = 0; i < 50; ++i) {
    assert(flow_cal_add_sample(&c, 0, 0.5F, 0.5F * 1.5F, 0.0F));
  }
  /* O eixo 0 tem uma resposta clara; o eixo 1 continua sem nenhuma. */
  assert(flow_cal_result(&c, 0, &scalar, NULL));
  assert(NEAR(scalar, 1.5F, 1e-4F));
  assert(!flow_cal_result(&c, 1, &scalar, NULL));
}

/**
 * @brief Um sensor que já lê certo precisa calibrar para 1,0, sem se afastar
 *        por ruído de ponto flutuante na média ponderada.
 */
static void test_already_correct_scale_stays_at_one(void) {
  flow_calibrator_t c;
  flow_cal_reset(&c);
  for (int i = 0; i < 100; ++i) {
    float expected = ((i % 2) ? 1.0F : -1.0F) * 0.6F;
    flow_cal_add_sample(&c, 0, expected, expected, 0.0F);
  }
  float scalar;
  assert(flow_cal_result(&c, 0, &scalar, NULL));
  assert(NEAR(scalar, 1.0F, 1e-5F));
}

/**
 * @brief O diagnóstico para o qual flow_cal_raw_scalar() existe: sinal do
 *        giroscópio invertido.
 *
 * Com gyro_sign invertido, bruto e esperado discordam de sinal em toda
 * amostra. flow_cal_result() corretamente recusa (escala negativa não faz
 * sentido), mas flow_cal_raw_scalar() ainda precisa mostrar um número
 * negativo, justamente para distingui-lo de contaminação por translação real
 * (que espalha os valores, em vez de inverter o sinal de forma consistente).
 */
static void test_raw_scalar_diagnoses_sign_error(void) {
  flow_calibrator_t c;
  flow_cal_reset(&c);
  for (int i = 0; i < 100; ++i) {
    float expected = ((i % 2) ? 1.0F : -1.0F) * 0.6F;
    float raw = -expected; /* sinal oposto o tempo todo: erro de sinal, não ruído */
    flow_cal_add_sample(&c, 0, raw, expected, 0.0F);
  }
  float scalar;
  assert(!flow_cal_result(&c, 0, &scalar, NULL));
  assert(flow_cal_raw_scalar(&c, 0, &scalar));
  assert(scalar < 0.0F);
  assert(NEAR(scalar, -1.0F, 1e-4F));
}

/** @brief Roda todos os testes; qualquer falha interrompe pelo assert. */
int main(void) {
  test_gates();
  test_recovers_known_scale_no_noise();
  test_recovers_known_scale_with_noise();
  test_rejects_implausible_scale();
  test_no_samples_and_axis_independence();
  test_already_correct_scale_stays_at_one();
  test_raw_scalar_diagnoses_sign_error();
  puts("Flow calibrator: gates, exact recovery, noisy recovery, implausible-scale "
       "rejection, axis independence, already-correct scale and sign-error "
       "diagnosis passed");
}
