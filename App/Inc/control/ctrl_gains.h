/**
 * @file ctrl_gains.h
 * @brief Ganhos do LQG em uso e troca atômica de ganhos enviados pela Pi.
 *
 * O firmware começa com os ganhos exportados do Simulink
 * (::LQG_ERT_DEFAULT_GAINS). Durante o ajuste, a Raspberry Pi recalcula Kd,
 * Kfd, L e M a partir de novos pesos e os envia linha por linha
 * (DRONE_GAIN_ROW). As linhas vão para uma área de preparo; os ganhos em uso
 * só mudam no commit (DRONE_GAIN_COMMIT), e somente se:
 *
 * - as 32 linhas da mesma sessão chegaram (4 de Kd, 4 de Kfd, 12 de L,
 *   12 de M);
 * - todos os valores são finitos;
 * - o CRC-32 calculado aqui é igual ao enviado pela Pi.
 *
 * O controle copia os ganhos uma vez por passo, quando a geração muda, então
 * um passo nunca mistura ganhos velhos e novos.
 *
 * @par CRC
 * CRC-32 IEEE 802.3 (o mesmo de `zlib.crc32`), calculado sobre os valores em
 * float32 little-endian, na ordem Kd, Kfd, L, M, cada matriz linha por linha.
 */
#ifndef CONTROL_CTRL_GAINS_H
#define CONTROL_CTRL_GAINS_H

#include "control/lqg_ert.h"

#include <stdbool.h>
#include <stdint.h>

/** @brief Matrizes que podem ser enviadas (mesmos valores de DRONE_GAIN_MATRIX). */
typedef enum {
  CTRL_GAIN_KD = 0,   /**< Kd, 4 x 12. */
  CTRL_GAIN_KFD = 1,  /**< Kfd, 4 x 4. */
  CTRL_GAIN_L = 2,    /**< L, 12 x 9. */
  CTRL_GAIN_M = 3,    /**< M, 12 x 9. */
  CTRL_GAIN_COUNT
} ctrl_gain_matrix_t;

/** @brief Resultado das operações de envio (mesmos valores de DRONE_GAIN_STATUS). */
typedef enum {
  CTRL_GAINS_OK = 0,            /**< Aceito. */
  CTRL_GAINS_BAD_SESSION = 1,   /**< Sessão diferente da que está sendo preparada. */
  CTRL_GAINS_BAD_INDEX = 2,     /**< Matriz, linha ou quantidade de colunas inválida. */
  CTRL_GAINS_INCOMPLETE = 3,    /**< Commit sem todas as linhas. */
  CTRL_GAINS_CRC_MISMATCH = 4,  /**< O CRC não confere com o enviado. */
  CTRL_GAINS_NOT_FINITE = 5     /**< Algum valor é NaN ou infinito. */
} ctrl_gains_status_t;

/** Total de linhas de um conjunto completo (4 + 4 + 12 + 12). */
#define CTRL_GAINS_TOTAL_ROWS 32U
/** Maior quantidade de colunas de uma linha (Kd). */
#define CTRL_GAINS_MAX_COLS 12U

/** @brief Restaura os ganhos do Simulink e descarta qualquer preparo. */
void ctrl_gains_init(void);

/**
 * @brief Linhas e colunas de uma matriz.
 * @param[in]  m     Matriz.
 * @param[out] rows  Linhas (pode ser NULL).
 * @param[out] cols  Colunas (pode ser NULL).
 * @return false se @p m é inválida.
 */
bool ctrl_gains_shape(ctrl_gain_matrix_t m, uint8_t *rows, uint8_t *cols);

/**
 * @brief Geração dos ganhos em uso: muda a cada commit ou restauração.
 *
 * Permite ao controle copiar os ganhos só quando eles mudam.
 */
uint32_t ctrl_gains_generation(void);

/**
 * @brief Copia os ganhos em uso, de forma segura entre tasks.
 * @param[out] out         Destino.
 * @param[out] generation  Geração copiada (pode ser NULL).
 */
void ctrl_gains_copy_active(lqg_gains_t *out, uint32_t *generation);

/** @brief Verdadeiro enquanto os ganhos em uso são os do Simulink. */
bool ctrl_gains_active_is_default(void);

/** @brief CRC-32 dos ganhos em uso (ver formato no topo do arquivo). */
uint32_t ctrl_gains_active_crc(void);

/**
 * @brief Recebe uma linha de uma sessão de envio.
 *
 * Uma sessão nova (id diferente da atual) descarta o preparo anterior. Linhas
 * repetidas são aceitas e sobrescrevem a anterior.
 *
 * @param[in] session  Identificador da sessão escolhido pela Pi (não zero).
 * @param[in] m        Matriz.
 * @param[in] row      Linha.
 * @param[in] values   Valores da linha.
 * @param[in] count    Quantidade de valores; precisa ser o número de colunas.
 * @return ::CTRL_GAINS_OK ou o motivo da recusa.
 */
ctrl_gains_status_t ctrl_gains_stage_row(uint32_t session, ctrl_gain_matrix_t m,
                                         uint8_t row, const float *values,
                                         uint8_t count);

/**
 * @brief Troca os ganhos em uso pelos preparados, se estiverem completos e
 *        íntegros.
 *
 * @param[in]  session        Sessão a confirmar.
 * @param[in]  expected_crc   CRC-32 calculado pela Pi.
 * @param[out] computed_crc   CRC-32 do preparo (pode ser NULL).
 * @param[out] rows_received  Máscara das linhas recebidas, bit i = linha i na
 *                            ordem Kd, Kfd, L, M (pode ser NULL).
 * @return ::CTRL_GAINS_OK se os ganhos foram trocados.
 */
ctrl_gains_status_t ctrl_gains_commit(uint32_t session, uint32_t expected_crc,
                                      uint32_t *computed_crc,
                                      uint32_t *rows_received);

/** @brief Descarta o preparo em andamento. */
void ctrl_gains_abort(void);

/** @brief Volta aos ganhos do Simulink. */
void ctrl_gains_restore_default(void);

/**
 * @brief CRC-32 de um conjunto de ganhos, no formato do envio.
 * @param[in] g  Ganhos.
 * @return O CRC.
 */
uint32_t ctrl_gains_crc(const lqg_gains_t *g);

/**
 * @brief CRC-32 IEEE incremental (compatível com zlib.crc32).
 * @param[in] crc   Valor anterior (0 no início).
 * @param[in] data  Bytes.
 * @param[in] len   Quantidade.
 * @return Novo valor.
 */
uint32_t ctrl_crc32(uint32_t crc, const void *data, uint32_t len);

#endif
