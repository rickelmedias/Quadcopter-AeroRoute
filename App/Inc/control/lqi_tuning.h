/**
 * @file lqi_tuning.h
 * @brief Ganhos do LQI em uso e troca atômica pelo console USB.
 *
 * O firmware começa com a tabela gerada por tools/lqi (::LQI_GAINS). Para
 * testar outros pesos Q/R, o painel (tools/controle_gui.py) recalcula Kx, Kfd e
 * Ki com o mesmo `dlqr` (tools/lqi_projeto.py) e envia valor por valor
 * (`lqi set`). Os valores vão para uma área de preparo; os ganhos em uso só
 * mudam no commit (`lqi commit <crc>`), e somente se:
 *
 * - os 76 valores chegaram desde o último `lqi begin` (48 de Kx, 16 de Kfd,
 *   12 de Ki);
 * - todos são finitos;
 * - o CRC-32 calculado aqui é igual ao enviado.
 *
 * A troca só é pedida com o drone desarmado (o console recusa armado): os
 * integradores ξ guardam o equilíbrio na escala do Ki antigo.
 *
 * @par CRC
 * O mesmo de control/ctrl_gains.h (CRC-32 IEEE, o `zlib.crc32`), sobre os
 * valores em float32 little-endian, na ordem Kx, Kfd, Ki, cada matriz linha
 * por linha.
 *
 * Código puro (sem HAL), testado em tests/test_lqi.c.
 */
#ifndef CONTROL_LQI_TUNING_H
#define CONTROL_LQI_TUNING_H

#include "control/lqi.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief Matrizes que podem ser enviadas. */
typedef enum {
  LQI_TUNING_KX = 0,  /**< Kx, 4 x 12. */
  LQI_TUNING_KFD,     /**< Kfd, 4 x 4. */
  LQI_TUNING_KI,      /**< Ki, 4 x 3. */
  LQI_TUNING_COUNT
} lqi_tuning_matrix_t;

/** @brief Resultado do preparo e do commit. */
typedef enum {
  LQI_TUNING_OK = 0,         /**< Aceito. */
  LQI_TUNING_BAD_INDEX,      /**< Matriz, linha ou coluna inválida. */
  LQI_TUNING_INCOMPLETE,     /**< Commit sem todos os valores. */
  LQI_TUNING_CRC_MISMATCH,   /**< O CRC não confere com o enviado. */
  LQI_TUNING_NOT_FINITE      /**< Algum valor é NaN ou infinito. */
} lqi_tuning_status_t;

/** Total de valores de um conjunto (48 + 16 + 12). */
#define LQI_TUNING_VALUES 76U

/** @brief Restaura os ganhos padrão e descarta o preparo. */
void lqi_tuning_init(void);

/** @brief Geração dos ganhos em uso: muda a cada commit ou restauração. */
uint32_t lqi_tuning_generation(void);

/**
 * @brief Copia os ganhos em uso, de forma segura entre tasks.
 * @param[out] out         Destino.
 * @param[out] generation  Geração copiada (pode ser NULL).
 */
void lqi_tuning_copy_active(lqi_gains_t *out, uint32_t *generation);

/** @brief Os ganhos em uso são os padrão (::LQI_GAINS). */
bool lqi_tuning_is_default(void);

/** @brief CRC dos ganhos em uso. */
uint32_t lqi_tuning_active_crc(void);

/**
 * @brief CRC de um conjunto de ganhos, no formato do envio.
 * @param[in] g  Ganhos.
 */
uint32_t lqi_tuning_crc(const lqi_gains_t *g);

/** @brief Começa um envio novo (descarta o que estava preparado). */
void lqi_tuning_begin(void);

/**
 * @brief Prepara um valor.
 * @param[in] m      Matriz.
 * @param[in] row    Linha (entrada u1..u4, 0..3).
 * @param[in] col    Coluna.
 * @param[in] value  Valor.
 */
lqi_tuning_status_t lqi_tuning_stage(lqi_tuning_matrix_t m, unsigned row, unsigned col,
                                     float value);

/**
 * @brief Troca os ganhos em uso pelos preparados, se tudo conferir.
 * @param[in]  expected_crc  CRC enviado.
 * @param[out] computed_crc  CRC calculado aqui (pode ser NULL).
 */
lqi_tuning_status_t lqi_tuning_commit(uint32_t expected_crc, uint32_t *computed_crc);

/** @brief Volta aos ganhos padrão (::LQI_GAINS). */
void lqi_tuning_restore_default(void);

/** @brief Texto curto de um resultado, para o console. */
const char *lqi_tuning_status_text(lqi_tuning_status_t s);

#endif
