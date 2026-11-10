/**
 * @file test_capture.c
 * @brief Testes de host do formato da captura binária.
 *
 * Exercita o formato dos registros com o mesmo codificador do firmware e o
 * decodifica com um leitor independente. Um formato só é útil se algo além
 * de quem escreve conseguir lê-lo, então o leitor daqui foi escrito a partir
 * do layout documentado, e não reaproveitando as funções do escritor.
 */
#include "capture.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/** pi em float. M_PI não existe em C11 estrito; -std=c11 o esconde. */
#define PI_F 3.14159265358979323846F

/* ---- leitor little-endian independente --------------------------------- */

/**
 * @brief Cursor de leitura sobre um registro.
 */
typedef struct {
  /** Bytes do registro. */
  const uint8_t *buf;
  /** Tamanho total. */
  uint16_t len;
  /** Próximo byte a ler. */
  uint16_t pos;
} reader_t;

/** @brief Lê um u8 (falha no assert se passar do fim). */
static uint8_t r_u8(reader_t *r) { assert(r->pos + 1 <= r->len); return r->buf[r->pos++]; }
/** @brief Lê um u16 little-endian. */
static uint16_t r_u16(reader_t *r) { uint16_t lo = r_u8(r); return (uint16_t)(lo | (r_u8(r) << 8)); }
/** @brief Lê um u32 little-endian. */
static uint32_t r_u32(reader_t *r) { uint32_t lo = r_u16(r); return lo | ((uint32_t)r_u16(r) << 16); }
/** @brief Lê um i16 little-endian. */
static int16_t r_i16(reader_t *r) { return (int16_t)r_u16(r); }
/** @brief Lê um f32 IEEE-754 little-endian. */
static float r_f32(reader_t *r) { uint32_t b = r_u32(r); float f; memcpy(&f, &b, sizeof f); return f; }

/**
 * @brief Confere o enquadramento de um registro e devolve um leitor
 *        posicionado na carga útil.
 *
 * @param rec          Registro.
 * @param size         Tamanho total do registro.
 * @param expect_type  Tipo esperado.
 * @param seq_out      Opcional: recebe o número de sequência.
 * @param t_out        Opcional: recebe o t_ms.
 * @return Leitor no início da carga.
 */
static reader_t open_record(const uint8_t *rec, uint16_t size, uint8_t expect_type,
                            uint16_t *seq_out, uint32_t *t_out) {
  assert(size >= CAP_OVERHEAD);
  assert(rec[0] == CAP_SYNC);
  assert(rec[1] == expect_type);
  const uint8_t payload = rec[2];
  assert((uint16_t)(payload + CAP_OVERHEAD) == size);
  /* O checksum cobre cabeçalho e carga, e fica depois deles. */
  assert(cap_crc8(rec, (uint16_t)(size - CAP_CRC_BYTES)) == rec[size - 1]);

  reader_t r = { rec, size, 3 };
  const uint16_t seq = r_u16(&r);
  const uint32_t t = r_u32(&r);
  if (seq_out) *seq_out = seq;
  if (t_out) *t_out = t;
  assert(r.pos == CAP_HEADER_BYTES);
  return r;
}

/* ---- testes ------------------------------------------------------------ */

/** @brief Ida e volta do enquadramento: cabeçalho, campos e tamanho. */
static void test_framing_roundtrip(void) {
  uint8_t buf[CAP_MAX_RECORD];
  cap_writer_t w;
  cap_begin(&w, buf, sizeof buf, CAP_TYPE_MTF, 0x1234, 0xDEADBEEFU);
  cap_i16(&w, -320);
  cap_u32(&w, 812U);
  cap_u8(&w, 87U);
  const uint16_t size = cap_finish(&w);
  assert(size == CAP_OVERHEAD + 2 + 4 + 1);

  uint16_t seq; uint32_t t;
  reader_t r = open_record(buf, size, CAP_TYPE_MTF, &seq, &t);
  assert(seq == 0x1234);
  assert(t == 0xDEADBEEFU);
  assert(r_i16(&r) == -320);
  assert(r_u32(&r) == 812U);
  assert(r_u8(&r) == 87U);
  assert(r.pos == size - CAP_CRC_BYTES);
}

/**
 * @brief Floats precisam voltar idênticos bit a bit, inclusive os valores
 *        difíceis.
 *
 * Um leitor que assumisse o formato da própria máquina em vez de decodificar
 * IEEE-754 little-endian passaria num teste que só usasse valores amigáveis.
 */
static void test_float_encoding_is_exact(void) {
  const float values[] = {
    0.0F, -0.0F, 1.0F, -1.0F, 0.1F, -0.944F, 3.4e38F, 1.2e-38F,
    0.2F * 0.84F, PI_F
  };
  for (unsigned i = 0; i < sizeof values / sizeof values[0]; ++i) {
    uint8_t buf[CAP_MAX_RECORD];
    cap_writer_t w;
    cap_begin(&w, buf, sizeof buf, CAP_TYPE_BNO, (uint16_t)i, 0);
    cap_f32(&w, values[i]);
    const uint16_t size = cap_finish(&w);
    reader_t r = open_record(buf, size, CAP_TYPE_BNO, NULL, NULL);
    const float back = r_f32(&r);
    assert(memcmp(&back, &values[i], sizeof back) == 0); /* idêntico bit a bit, não "perto" */
  }
}

/**
 * @brief Um byte corrompido em qualquer ponto do registro precisa falhar no
 *        checksum; senão o replay consumiria números errados em silêncio.
 */
static void test_corruption_is_detected(void) {
  uint8_t buf[CAP_MAX_RECORD];
  cap_writer_t w;
  cap_begin(&w, buf, sizeof buf, CAP_TYPE_STEP, 7, 1000);
  cap_f32(&w, 0.25F);
  cap_u32(&w, 4242U);
  const uint16_t size = cap_finish(&w);
  assert(cap_crc8(buf, (uint16_t)(size - 1)) == buf[size - 1]);

  for (uint16_t i = 0; i < size - 1U; ++i) {
    uint8_t copy[CAP_MAX_RECORD];
    memcpy(copy, buf, size);
    copy[i] ^= 0x01U; /* inverte um único bit */
    assert(cap_crc8(copy, (uint16_t)(size - 1)) != copy[size - 1]);
  }
}

/**
 * @brief Um registro que não cabe precisa ser recusado por inteiro; emitir um
 *        truncado dessincronizaria todos os registros seguintes.
 */
static void test_oversized_record_is_refused(void) {
  uint8_t small[CAP_HEADER_BYTES + 4U];
  cap_writer_t w;
  cap_begin(&w, small, sizeof small, CAP_TYPE_DIAG, 1, 0);
  cap_u32(&w, 1U);           /* enche exatamente */
  cap_u32(&w, 2U);           /* não cabe */
  assert(w.overflow);
  assert(cap_finish(&w) == 0U);
}

/**
 * @brief O byte de sincronismo permite ao leitor se recuperar após uma perda.
 *
 * Alimenta um fluxo com um buraco no meio e confere que o registro seguinte
 * ainda é encontrado, e que o salto de sequência deixa a perda visível em vez
 * de parecer um período parado.
 */
static void test_resync_after_loss(void) {
  uint8_t stream[512];
  uint16_t used = 0;
  uint16_t sizes[3];
  for (uint16_t i = 0; i < 3; ++i) {
    cap_writer_t w;
    cap_begin(&w, stream + used, (uint16_t)(sizeof stream - used), CAP_TYPE_MTF,
              (uint16_t)(100 + i), 1000U + i);
    cap_i16(&w, (int16_t)(i * 20));
    cap_u32(&w, 800U);
    sizes[i] = cap_finish(&w);
    assert(sizes[i] > 0);
    used = (uint16_t)(used + sizes[i]);
  }

  /* Remove o registro do meio inteiro, como uma fila cheia faria. */
  uint8_t lossy[512];
  const uint16_t first = sizes[0], second = sizes[1];
  memcpy(lossy, stream, first);
  memcpy(lossy + first, stream + first + second, sizes[2]);
  const uint16_t lossy_len = (uint16_t)(first + sizes[2]);

  /* Procura os registros como um leitor faria. */
  uint16_t seqs[8]; unsigned found = 0;
  for (uint16_t p = 0; p + CAP_OVERHEAD <= lossy_len; ) {
    if (lossy[p] != CAP_SYNC) { ++p; continue; }
    const uint16_t size = (uint16_t)(lossy[p + 2] + CAP_OVERHEAD);
    if (p + size > lossy_len) { ++p; continue; }
    if (cap_crc8(lossy + p, (uint16_t)(size - 1)) != lossy[p + size - 1]) { ++p; continue; }
    uint16_t seq;
    (void)open_record(lossy + p, size, CAP_TYPE_MTF, &seq, NULL);
    seqs[found++] = seq;
    p = (uint16_t)(p + size);
  }
  assert(found == 2);
  assert(seqs[0] == 100 && seqs[1] == 102);
  /* O salto de 2 é o que diz ao replay que faltam dados, e não que o veículo
     ficou parado entre essas duas amostras. */
  assert((uint16_t)(seqs[1] - seqs[0]) == 2);
}

/**
 * @brief Campos de 64 bits voltam corretamente.
 *
 * Eles são montados com duas metades de 32 bits, justamente o tipo de lugar
 * onde um erro de ordem de bytes se esconde: o carimbo de medição do SH-2 é
 * em microssegundos desde o boot do hub, então a metade alta fica pequena e
 * um par trocado ainda parece um tempo plausível por muito tempo.
 */
static void test_u64_roundtrip(void) {
  const uint64_t values[] = {
    0U, 1U, 0xFFFFFFFFU, 0x100000000ULL, 0xDEADBEEFCAFEF00DULL,
    1005000U /* um carimbo realista do hub */
  };
  for (unsigned i = 0; i < sizeof values / sizeof values[0]; ++i) {
    uint8_t buf[CAP_MAX_RECORD];
    cap_writer_t w;
    cap_begin(&w, buf, sizeof buf, CAP_TYPE_BNO, (uint16_t)i, 0);
    cap_u64(&w, values[i]);
    const uint16_t size = cap_finish(&w);
    reader_t r = open_record(buf, size, CAP_TYPE_BNO, NULL, NULL);
    const uint64_t lo = r_u32(&r), hi = r_u32(&r);
    assert((lo | (hi << 32)) == values[i]);
  }
}

/**
 * @brief Strings são precedidas do tamanho e truncadas no limite.
 *
 * O leitor nunca procura um terminador. A identificação do firmware é a que
 * importa, e ela precisa ser truncada em vez de passar do limite ou gravar um
 * tamanho que a carga não contém.
 */
static void test_str_is_length_prefixed_and_truncates(void) {
  const char *longer = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  uint8_t buf[CAP_MAX_RECORD];
  cap_writer_t w;
  cap_begin(&w, buf, sizeof buf, CAP_TYPE_SESSION, 0, 0);
  cap_str(&w, longer, 8U);
  cap_str(&w, "ab", 8U);
  cap_str(&w, "", 8U);
  const uint16_t size = cap_finish(&w);
  reader_t r = open_record(buf, size, CAP_TYPE_SESSION, NULL, NULL);

  assert(r_u8(&r) == 8U);
  for (unsigned i = 0; i < 8U; ++i) assert(r_u8(&r) == (uint8_t)longer[i]);
  assert(r_u8(&r) == 2U);
  assert(r_u8(&r) == 'a' && r_u8(&r) == 'b');
  assert(r_u8(&r) == 0U);
  assert(r.pos == size - CAP_CRC_BYTES); /* nada escrito depois do último campo */
}

/**
 * @brief Os tamanhos de carga do escritor precisam bater com os do decoder.
 *
 * O decoder confere o tamanho de cada registro antes de confiar nele, então
 * o escritor e aquela tabela precisam concordar. Estes são os números da
 * PAYLOAD_EXACT de tools/decode_capture.py; se um campo for acrescentado de
 * um lado e não do outro, isto falha antes de uma sessão de gravação ser
 * desperdiçada.
 */
static void test_payload_sizes_match_the_decoder(void) {
  uint8_t buf[CAP_MAX_RECORD];
  cap_writer_t w;

  cap_begin(&w, buf, sizeof buf, CAP_TYPE_MTF, 0, 0);
  cap_i16(&w, 0); cap_i16(&w, 0); cap_u32(&w, 0);
  cap_u8(&w, 0); cap_u8(&w, 0); cap_u8(&w, 0); cap_u8(&w, 0); cap_u8(&w, 0);
  cap_u32(&w, 0); cap_u32(&w, 0);
  assert(cap_finish(&w) - CAP_OVERHEAD == 21U);

  cap_begin(&w, buf, sizeof buf, CAP_TYPE_BNO, 0, 0);
  cap_u8(&w, 0); cap_u8(&w, 0); cap_u8(&w, 0);
  for (int i = 0; i < 4; ++i) cap_f32(&w, 0.0F);
  cap_u32(&w, 0); cap_u64(&w, 0); cap_u32(&w, 0);
  assert(cap_finish(&w) - CAP_OVERHEAD == 35U);

  cap_begin(&w, buf, sizeof buf, CAP_TYPE_STEP, 0, 0);
  cap_u32(&w, 0); cap_u32(&w, 0); cap_u32(&w, 0); cap_u32(&w, 0);
  cap_f32(&w, 0.0F); cap_u8(&w, 0);
  for (int i = 0; i < 8; ++i) cap_f32(&w, 0.0F);
  assert(cap_finish(&w) - CAP_OVERHEAD == 53U);
}

/** @brief Roda todos os testes; qualquer falha interrompe pelo assert. */
int main(void) {
  test_framing_roundtrip();
  test_u64_roundtrip();
  test_str_is_length_prefixed_and_truncates();
  test_payload_sizes_match_the_decoder();
  test_float_encoding_is_exact();
  test_corruption_is_detected();
  test_oversized_record_is_refused();
  test_resync_after_loss();
  puts("Capture format: framing round trip, bit-exact floats, 64-bit and string\n"
       "encoding, payload sizes agreeing with the decoder, corruption detection,\n"
       "oversized-record refusal and resync with a visible sequence gap");
}
