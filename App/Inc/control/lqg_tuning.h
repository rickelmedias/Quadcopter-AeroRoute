/**
 * @file lqg_tuning.h
 * @brief Ganhos Kd/Kfd do LQG ajustado em uso e troca atômica pelo console USB.
 *
 * O firmware começa com a tabela gerada por tools/lqg_ajustado.py
 * (::LQG_AJ_KD, ::LQG_AJ_KFD). Para testar outros pesos Q nos 12 estados, o
 * painel (tools/controle_gui.py) refaz o `dlqr` do TCC (mesmo A, B, R) e envia
 * valor por valor (`lqg set`). Os valores vão para uma área de preparo; os
 * ganhos em uso só mudam no commit (`lqg commit <crc>`), e somente se:
 *
 * - os 64 valores chegaram desde o último `lqg begin` (48 de Kd, 16 de Kfd);
 * - todos são finitos;
 * - o CRC-32 calculado aqui é igual ao enviado.
 *
 * O estimador de perturbação não depende de Q (só de A, B e dos ruídos), então
 * continua o gerado. A troca só vale desarmado: o voo copia os ganhos no
 * início.
 *
 * @par CRC
 * O mesmo de control/ctrl_gains.h (CRC-32 IEEE, o `zlib.crc32`), sobre os
 * valores em float32 little-endian, na ordem Kd, Kfd, cada matriz linha por
 * linha.
 *
 * Código puro (sem HAL), testado em tests/test_lqg_dist.c.
 */
#ifndef CONTROL_LQG_TUNING_H
#define CONTROL_LQG_TUNING_H

#include "control/lqg_ert.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief Ganhos que o painel pode trocar. */
typedef struct {
  lqg_real_t Kd[LQG_NU][LQG_NX];   /**< Ganho de estado, 4 x 12. */
  lqg_real_t Kfd[LQG_NU][LQG_NR];  /**< Ganho de referência, 4 x 4. */
} lqg_aj_gains_t;

/** @brief Matrizes que podem ser enviadas. */
typedef enum {
  LQG_TUNING_KD = 0,  /**< Kd, 4 x 12. */
  LQG_TUNING_KFD,     /**< Kfd, 4 x 4. */
  LQG_TUNING_COUNT
} lqg_tuning_matrix_t;

/** @brief Resultado do preparo e do commit. */
typedef enum {
  LQG_TUNING_OK = 0,         /**< Aceito. */
  LQG_TUNING_BAD_INDEX,      /**< Matriz, linha ou coluna inválida. */
  LQG_TUNING_INCOMPLETE,     /**< Commit sem todos os valores. */
  LQG_TUNING_CRC_MISMATCH,   /**< O CRC não confere com o enviado. */
  LQG_TUNING_NOT_FINITE      /**< Algum valor é NaN ou infinito. */
} lqg_tuning_status_t;

/** Total de valores de um conjunto (48 + 16). */
#define LQG_TUNING_VALUES 64U

/** @brief Restaura os ganhos gerados e descarta o preparo. */
void lqg_tuning_init(void);

/** @brief Geração dos ganhos em uso: muda a cada commit ou restauração. */
uint32_t lqg_tuning_generation(void);

/** @brief Copia os ganhos em uso (e a geração, se @p gen não for NULL). */
void lqg_tuning_copy_active(lqg_aj_gains_t *out, uint32_t *gen);

/** @brief Os ganhos em uso são os gerados. */
bool lqg_tuning_is_default(void);

/** @brief CRC dos ganhos em uso. */
uint32_t lqg_tuning_active_crc(void);

/** @brief CRC de um conjunto, na ordem canônica. */
uint32_t lqg_tuning_crc(const lqg_aj_gains_t *g);

/** @brief Descarta o que foi preparado. */
void lqg_tuning_begin(void);

/** @brief Prepara um valor. */
lqg_tuning_status_t lqg_tuning_stage(lqg_tuning_matrix_t m, unsigned row, unsigned col,
                                     float value);

/** @brief Confere e troca os ganhos em uso; @p computed recebe o CRC calculado. */
lqg_tuning_status_t lqg_tuning_commit(uint32_t expected_crc, uint32_t *computed);

/** @brief Volta aos ganhos gerados. */
void lqg_tuning_restore_default(void);

/** @brief Troca Kd e Kfd numa cópia dos ganhos do controle (L e M não mudam). */
void lqg_tuning_apply(const lqg_aj_gains_t *aj, lqg_gains_t *gains);

/** @brief Texto de um resultado. */
const char *lqg_tuning_status_text(lqg_tuning_status_t s);

#endif /* CONTROL_LQG_TUNING_H */
