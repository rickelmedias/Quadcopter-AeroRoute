/**
 * @file test_regressions.c
 * @brief Regressões dos defeitos encontrados na revisão técnica externa.
 *
 * Todo teste aqui chama a MESMA função que o firmware roda - as proteções
 * compartilhadas ficam em headers (dma_ring.h, bno_i2c_header.h,
 * displacement.h) exatamente por isso. Uma versão antiga deste arquivo
 * reimplementava a lógica localmente, então apagar uma proteção de produção
 * não teria feito nenhum teste falhar. Onde uma verificação não pode ser
 * alcançada sem FreeRTOS, o teste diz isso em vez de testar uma cópia.
 */
#include "displacement.h"
#include "flow_ekf.h"
#include "dma_ring.h"
#include "bno_i2c_header.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/** Verdadeiro se @p a e @p b diferem no máximo @p tol. */
#define NEAR(a, b, tol) (fabsf((a)-(b)) <= (tol))

/* ---- 1. Reinício do relógio do MTF-01 --------------------------------- */

/**
 * @brief O reinício do relógio do sensor não pode virar um intervalo de tempo.
 *
 * O sensor carimba os frames com o próprio relógio, que volta a zero quando
 * ele reinicia. Subtrair dois carimbos uint32 através desse reinício dá ~49,7
 * dias em vez de um intervalo curto. disp_frame_gap_usable() é a proteção de
 * produção; este teste a chama diretamente.
 */
static void test_sensor_clock_restart_is_not_a_time_gap(void) {
  const uint32_t gap_across_reboot = 10U - 10000U; /* volta do unsigned */
  assert(gap_across_reboot > 4000000000U);
  assert(!disp_frame_gap_usable(gap_across_reboot));

  /* O que esse intervalo faria se chegasse ao filtro: uns suaves
     0,1 m/s^2 viram uma posição astronômica. */
  flow_ekf_config_t cfg = FLOW_EKF_DEFAULTS;
  flow_ekf_t unguarded;
  flow_ekf_reset(&unguarded);
  flow_ekf_predict(&unguarded, 0.1F, 0.0F, (float)gap_across_reboot * 0.001F, &cfg);
  assert(fabsf(unguarded.axis[0].p) > 1e11F);

  /* Intervalos normais passam; os degenerados não. */
  assert(disp_frame_gap_usable(10U));
  assert(disp_frame_gap_usable(DISP_MAX_FRAME_GAP_MS));
  assert(!disp_frame_gap_usable(0U));
  assert(!disp_frame_gap_usable(DISP_MAX_FRAME_GAP_MS + 1U));
}

/* ---- 2. Inicialização do filtro --------------------------------------- */

/**
 * @brief Um filtro zerado precisa se autoinicializar.
 *
 * O armazenamento estático entrega o filtro todo zerado, e variância de
 * velocidade zero não é neutra: zera o primeiro ganho de Kalman e ao mesmo
 * tempo satisfaz o teste de convergência. O filtro agora corrige isso
 * sozinho, então o risco não volta porque alguém esqueceu de chamar o reset.
 */
static void test_zeroed_filter_self_initialises(void) {
  flow_ekf_config_t cfg = FLOW_EKF_DEFAULTS;

  flow_ekf_t zeroed;
  memset(&zeroed, 0, sizeof zeroed); /* exatamente o que o boot entrega */
  assert(!zeroed.initialized);
  assert(zeroed.axis[0].P11 == 0.0F);

  /* Um filtro zerado não pode dizer que convergiu antes de ver qualquer
     coisa, e precisa aprender de fato com a primeira medida. */
  assert(flow_ekf_update(&zeroed, 0.2F, 0.0F, 0.8F, &cfg));
  assert(zeroed.initialized);
  assert(zeroed.axis[0].v > 0.15F);

  /* Chegar lá pela predição primeiro precisa corrigir do mesmo jeito. */
  flow_ekf_t zeroed2;
  memset(&zeroed2, 0, sizeof zeroed2);
  flow_ekf_predict(&zeroed2, 0.0F, 0.0F, 0.01F, &cfg);
  assert(zeroed2.initialized);
  assert(!zeroed2.converged); /* nada o ancorou ainda */

  /* O contrato documentado, exercitado como quem chama deve escrever.
     Repare no que NÃO é testado aqui e não tem como ser: uma variável
     automática `flow_ekf_t f;` não inicializada já é comportamento indefinido
     no momento em que .initialized é lido, e nenhuma proteção em tempo de
     execução resolve isso - ver o contrato em flow_ekf.h. */
  flow_ekf_t braced = {0};
  assert(!braced.initialized);
  flow_ekf_predict(&braced, 0.0F, 0.0F, 0.01F, &cfg);
  assert(braced.initialized);
}

/* ---- 3. Convergência velha e validade --------------------------------- */

/**
 * @brief Um filtro seguindo só por inércia precisa deixar de dizer que
 *        convergiu.
 *
 * A predição aumenta a covariância, então pode tirar o filtro da
 * convergência. Deixar o indicador só para a correção fazia um filtro sem
 * medidas anunciar convergência para sempre.
 *
 * Também mostra que @c converged é LENTO - dezenas de segundos com o ruído
 * de processo padrão. Lento demais para proteger um controlador sozinho, e
 * por isso disp_health() também usa a idade da correção.
 */
static void test_coasting_filter_stops_claiming_convergence(void) {
  flow_ekf_config_t cfg = FLOW_EKF_DEFAULTS;
  flow_ekf_t f;
  flow_ekf_reset(&f);

  for (int i = 0; i < 2000; ++i) {
    flow_ekf_predict(&f, 0.0F, 0.0F, 0.01F, &cfg);
    flow_ekf_update(&f, 0.0F, 0.0F, 0.8F, &cfg);
  }
  assert(f.converged);
  assert(sqrtf(f.axis[0].P11) < 0.1F);

  int seconds = 0;
  while (f.axis[0].P11 < 0.09F && seconds < 120) {
    for (int i = 0; i < 100; ++i) flow_ekf_predict(&f, 0.0F, 0.0F, 0.01F, &cfg);
    ++seconds;
  }
  assert(seconds < 120);
  assert(!f.converged);
  assert(seconds > 20); /* documenta a lentidão que motiva disp_health */
}

/**
 * @brief A validade exige uma correção aceita, não apenas um frame recebido.
 *
 * O defeito que sobreviveu à primeira rodada de correções: a validade usava
 * a idade do último FRAME, e um frame chega quer algo útil saia dele ou não.
 * Com a IMU offline o caminho FUSED rejeita todos os frames e o filtro só
 * segue por inércia - mas os frames continuam chegando, então um critério
 * por idade de frame continuaria dizendo que a estimativa está boa.
 * disp_health() usa a idade da última CORREÇÃO ACEITA.
 */
static void test_validity_requires_an_accepted_correction(void) {
  /* Convergido e recém-corrigido: utilizável. */
  assert(disp_health(true, 0U) == DISP_HEALTH_OK);
  assert(disp_health(true, DISP_MAX_CORRECTION_AGE_MS) == DISP_HEALTH_OK);

  /* Convergido, mas nada aceito há tempo demais - exatamente o estado que o
     critério por idade de frame chamava de válido. */
  assert(disp_health(true, DISP_MAX_CORRECTION_AGE_MS + 1U) == DISP_HEALTH_NO_CORRECTION);
  assert(disp_health(true, 60000U) == DISP_HEALTH_NO_CORRECTION);

  /* Sem convergência nunca é utilizável, por mais recentes que sejam os
     dados. */
  assert(disp_health(false, 0U) == DISP_HEALTH_NOT_CONVERGED);
}

/* ---- 4. Posição de DMA repetida --------------------------------------- */

/* A HAL pode informar a mesma posição do anel duas vezes na borda do buffer
   - um evento de fim de transferência seguido de um de linha ociosa.
   Normalizar a posição de buffer cheio para 0 fazia o segundo parecer
   "0 -> size" e reenviar o buffer inteiro como dado novo. Os dois caminhos de
   USART agora usam dma_ring_segments(); estes testes a chamam diretamente. */

/** Tamanho do anel usado nos testes. */
#define RING 256U

/**
 * @brief Soma o tamanho dos trechos.
 * @param seg    Trechos.
 * @param count  Quantidade de trechos.
 * @return Total de bytes.
 */
static size_t total_length(const dma_ring_segment_t *seg, unsigned count) {
  size_t n = 0;
  for (unsigned i = 0; i < count; ++i) n += seg[i].length;
  return n;
}

/** @brief Uma posição repetida na borda do buffer não pode reenviar nada. */
static void test_repeated_dma_position_forwards_nothing(void) {
  dma_ring_segment_t seg[2];
  uint16_t last = 200;

  /* Primeiro evento: a cauda realmente nova. */
  unsigned count = dma_ring_segments(last, RING, RING, seg);
  assert(count == 1 && seg[0].offset == 200 && seg[0].length == 56);
  last = RING; /* guardada sem alteração, de propósito sem normalizar */

  /* Repetição da mesma posição: nada novo. Esta é a regressão. */
  count = dma_ring_segments(last, RING, RING, seg);
  assert(count == 0 && total_length(seg, count) == 0);

  /* A volta do anel que vem depois ainda entrega o início, sem cauda
     fantasma. */
  count = dma_ring_segments(last, 30, RING, seg);
  assert(count == 1 && seg[0].offset == 0 && seg[0].length == 30);
}

/** @brief Casos comuns do anel e a garantia de nunca apontar fora do buffer. */
static void test_dma_ring_ordinary_cases(void) {
  dma_ring_segment_t seg[2];

  /* Avanço simples. */
  unsigned count = dma_ring_segments(10, 50, RING, seg);
  assert(count == 1 && seg[0].offset == 10 && seg[0].length == 40);

  /* Uma volta real no meio do buffer: cauda e depois início, nessa ordem. */
  count = dma_ring_segments(200, 20, RING, seg);
  assert(count == 2);
  assert(seg[0].offset == 200 && seg[0].length == 56);
  assert(seg[1].offset == 0 && seg[1].length == 20);
  assert(total_length(seg, count) == 76);

  /* Dar a volta exatamente até 0 tem cauda, mas não início. */
  count = dma_ring_segments(200, 0, RING, seg);
  assert(count == 1 && seg[0].offset == 200 && seg[0].length == 56);

  /* Sem movimento, e posições fora da faixa, não entregam nada. */
  assert(dma_ring_segments(42, 42, RING, seg) == 0);
  assert(dma_ring_segments(0, RING + 1, RING, seg) == 0);
  assert(dma_ring_segments(RING + 1, 0, RING, seg) == 0);

  /* Sejam quais forem as entradas, um trecho nunca pode apontar para fora do
     buffer. */
  for (unsigned last = 0; last <= RING; ++last) {
    for (unsigned pos = 0; pos <= RING; ++pos) {
      const unsigned n = dma_ring_segments((uint16_t)last, (uint16_t)pos, RING, seg);
      assert(n <= 2);
      for (unsigned i = 0; i < n; ++i) {
        assert(seg[i].length > 0);
        assert((size_t)seg[i].offset + seg[i].length <= RING);
      }
    }
  }
}

/* ---- 5. Canal SHTP sem limite ----------------------------------------- */

/**
 * @brief O canal SHTP precisa ser limitado antes de ser usado.
 *
 * O parser SHTP incorporado indexa o vetor por canal com um byte vindo
 * direto do barramento e só confere a faixa depois. hub_read() rejeita antes
 * um canal fora da faixa, usando esta verificação compartilhada.
 */
static void test_shtp_channel_is_bounded_before_use(void) {
  uint8_t header[4] = {0x10, 0x00, 0x00, 0x00};
  for (unsigned chan = 0; chan < BNO_SHTP_MAX_CHANS; ++chan) {
    header[2] = (uint8_t)chan;
    assert(bno_i2c_header_channel_valid(header));
  }
  /* Um cabeçalho corrompido pode trazer qualquer byte. */
  for (unsigned chan = BNO_SHTP_MAX_CHANS; chan <= 255U; ++chan) {
    header[2] = (uint8_t)chan;
    assert(!bno_i2c_header_channel_valid(header));
  }

  /* A leitura completa é comparada com o cabeçalho espiado, e essa
     comparação exige o byte de canal idêntico - então validar o cabeçalho
     espiado cobre o pacote que o parser realmente recebe. */
  const uint8_t peeked[4] = {0x0F, 0x00, 0x02, 0x9A};
  uint8_t repeated[4] = {0x0F, 0x80, 0x02, 0x9B};
  assert(bno_i2c_header_matches(peeked, repeated));
  repeated[2] = 0x09; /* um canal que a proteção teria rejeitado */
  assert(!bno_i2c_header_matches(peeked, repeated));
}

/** @brief Roda todos os testes; qualquer falha interrompe pelo assert. */
int main(void) {
  test_sensor_clock_restart_is_not_a_time_gap();
  test_zeroed_filter_self_initialises();
  test_coasting_filter_stops_claiming_convergence();
  test_validity_requires_an_accepted_correction();
  test_repeated_dma_position_forwards_nothing();
  test_dma_ring_ordinary_cases();
  test_shtp_channel_is_bounded_before_use();
  puts("Regressions: clock restart, self-initialising filter, stale convergence,\n"
       "correction-age validity, duplicated and ordinary DMA positions, and\n"
       "SHTP channel bounds - all driving the shared production guards");
}
