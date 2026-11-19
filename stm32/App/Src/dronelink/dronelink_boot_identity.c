/**
 * @file dronelink_boot_identity.c
 * @brief Registro persistente de boots na flash SPI W25Q64 (contrato em
 *        dronelink_boot_identity.h).
 *
 * A flash fica no SPI1, com chip select em FLASH_CS. O registro ocupa o
 * último setor (4 KB) e é uma sequência de registros de 16 bytes.
 */
#include "dronelink/dronelink_boot_identity.h"

#include <stddef.h>
#include <string.h>

#include "main.h"
#include "spi.h"

/** Comando: ler o identificador JEDEC. */
#define FLASH_CMD_JEDEC_ID       0x9FU
/** Comando: ler dados. */
#define FLASH_CMD_READ_DATA      0x03U
/** Comando: ler o registrador de status 1. */
#define FLASH_CMD_READ_STATUS_1  0x05U
/** Comando: habilitar escrita. */
#define FLASH_CMD_WRITE_ENABLE   0x06U
/** Comando: gravar uma página. */
#define FLASH_CMD_PAGE_PROGRAM   0x02U
/** Comando: apagar um setor de 4 KB. */
#define FLASH_CMD_SECTOR_ERASE   0x20U
/** Bit "ocupado" do registrador de status 1. */
#define FLASH_STATUS_BUSY        0x01U
/** Código de fabricante da Winbond. */
#define FLASH_JEDEC_WINBOND      0xEFU
/** Código de capacidade da W25Q64 (64 Mbit). */
#define FLASH_JEDEC_W25Q64       0x17U
/** Tamanho de um setor, em bytes. */
#define FLASH_SECTOR_SIZE        4096U
/** Endereço do setor do registro (o último da flash). */
#define FLASH_JOURNAL_ADDRESS    0x007FF000UL
/** Timeout de cada transferência SPI, em ms. */
#define FLASH_IO_TIMEOUT_MS      20U
/** Espera máxima por uma gravação, em ms. */
#define FLASH_PROGRAM_TIMEOUT_MS 20U
/** Espera máxima por um apagamento de setor, em ms. */
#define FLASH_ERASE_TIMEOUT_MS   1000U
/** Marcador de registro válido ("DLB1" em ASCII). */
#define BOOT_RECORD_MAGIC        0x444C4231UL

/** @brief Um registro de boot gravado na flash (16 bytes). */
typedef struct {
  /** Deve ser ::BOOT_RECORD_MAGIC. */
  uint32_t magic;
  /** Número do boot; 0 é inválido. */
  uint32_t boot_id;
  /** Complemento de @c boot_id, para detectar corrupção. */
  uint32_t boot_id_inverse;
  /** CRC-32 dos campos anteriores. */
  uint32_t checksum;
} boot_record_t;

/** @brief Seleciona a flash (CS em nível baixo). */
static void select_flash(void) {
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

/** @brief Libera a flash (CS em nível alto). */
static void release_flash(void) {
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief Envia bytes pelo SPI1.
 * @param[in] data    Bytes.
 * @param[in] length  Quantidade.
 * @return true se a transferência terminou sem erro.
 */
static bool transmit(const uint8_t *data, uint16_t length) {
  return HAL_SPI_Transmit(&hspi1, (uint8_t *)data, length,
                          FLASH_IO_TIMEOUT_MS) == HAL_OK;
}

/**
 * @brief Lê o registrador de status 1.
 * @param[out] status  Valor lido.
 * @return true se a leitura funcionou.
 */
static bool read_status(uint8_t *status) {
  const uint8_t command = FLASH_CMD_READ_STATUS_1;
  select_flash();
  const bool sent = transmit(&command, 1U);
  const bool received = sent &&
      HAL_SPI_Receive(&hspi1, status, 1U, FLASH_IO_TIMEOUT_MS) == HAL_OK;
  release_flash();
  return received;
}

/**
 * @brief Espera a flash terminar a operação em andamento.
 * @param[in] timeout_ms  Espera máxima, em ms.
 * @return false em timeout ou erro de leitura.
 */
static bool wait_ready(uint32_t timeout_ms) {
  const uint32_t started = HAL_GetTick();
  uint8_t status = FLASH_STATUS_BUSY;
  do {
    if (!read_status(&status)) {
      return false;
    }
    if ((status & FLASH_STATUS_BUSY) == 0U) {
      return true;
    }
    HAL_Delay(1U);
  } while ((uint32_t)(HAL_GetTick() - started) < timeout_ms);
  return false;
}

/**
 * @brief Habilita a próxima escrita ou apagamento.
 * @return true se o comando foi enviado.
 */
static bool write_enable(void) {
  const uint8_t command = FLASH_CMD_WRITE_ENABLE;
  select_flash();
  const bool result = transmit(&command, 1U);
  release_flash();
  return result;
}

/**
 * @brief Lê um bloco da flash.
 * @param[in]  address      Endereço inicial.
 * @param[out] destination  Destino.
 * @param[in]  length       Quantidade de bytes.
 * @return true se a leitura funcionou.
 */
static bool read_data(uint32_t address, void *destination, uint16_t length) {
  const uint8_t command[4] = {
      FLASH_CMD_READ_DATA,
      (uint8_t)(address >> 16U),
      (uint8_t)(address >> 8U),
      (uint8_t)address,
  };
  select_flash();
  const bool sent = transmit(command, sizeof(command));
  const bool received = sent &&
      HAL_SPI_Receive(&hspi1, destination, length, FLASH_IO_TIMEOUT_MS) == HAL_OK;
  release_flash();
  return received;
}

/**
 * @brief Grava um bloco numa página já apagada e espera terminar.
 * @param[in] address  Endereço inicial.
 * @param[in] source   Dados.
 * @param[in] length   Quantidade de bytes.
 * @return true se a gravação terminou dentro do prazo.
 */
static bool page_program(uint32_t address, const void *source, uint16_t length) {
  const uint8_t command[4] = {
      FLASH_CMD_PAGE_PROGRAM,
      (uint8_t)(address >> 16U),
      (uint8_t)(address >> 8U),
      (uint8_t)address,
  };
  if (!wait_ready(FLASH_PROGRAM_TIMEOUT_MS) || !write_enable()) {
    return false;
  }
  select_flash();
  const bool header_sent = transmit(command, sizeof(command));
  const bool data_sent = header_sent && transmit(source, length);
  release_flash();
  return data_sent && wait_ready(FLASH_PROGRAM_TIMEOUT_MS);
}

/**
 * @brief Apaga o setor do registro e espera terminar.
 * @return true se o apagamento terminou dentro do prazo.
 */
static bool erase_journal(void) {
  const uint8_t command[4] = {
      FLASH_CMD_SECTOR_ERASE,
      (uint8_t)(FLASH_JOURNAL_ADDRESS >> 16U),
      (uint8_t)(FLASH_JOURNAL_ADDRESS >> 8U),
      (uint8_t)FLASH_JOURNAL_ADDRESS,
  };
  if (!wait_ready(FLASH_PROGRAM_TIMEOUT_MS) || !write_enable()) {
    return false;
  }
  select_flash();
  const bool sent = transmit(command, sizeof(command));
  release_flash();
  return sent && wait_ready(FLASH_ERASE_TIMEOUT_MS);
}

/**
 * @brief Calcula o CRC-32 dos campos antes de @c checksum.
 * @param[in] record  Registro.
 * @return O CRC-32.
 */
static uint32_t checksum(const boot_record_t *record) {
  const uint8_t *bytes = (const uint8_t *)record;
  uint32_t value = 0xFFFFFFFFUL;
  for (size_t index = 0U; index < offsetof(boot_record_t, checksum); ++index) {
    value ^= bytes[index];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      value = (value >> 1U) ^ (0xEDB88320UL & (0U - (value & 1U)));
    }
  }
  return ~value;
}

/**
 * @brief Diz se o registro está apagado (todos os bytes 0xFF).
 * @param[in] record  Registro lido.
 * @return true se está livre para gravação.
 */
static bool record_is_blank(const boot_record_t *record) {
  const uint8_t *bytes = (const uint8_t *)record;
  for (size_t index = 0U; index < sizeof(*record); ++index) {
    if (bytes[index] != 0xFFU) {
      return false;
    }
  }
  return true;
}

/**
 * @brief Confere marcador, número não nulo, complemento e CRC.
 * @param[in] record  Registro lido.
 * @return true se o registro é válido.
 */
static bool record_is_valid(const boot_record_t *record) {
  return record->magic == BOOT_RECORD_MAGIC && record->boot_id != 0U &&
         record->boot_id_inverse == ~record->boot_id &&
         record->checksum == checksum(record);
}

/**
 * @brief Confere pelo ID JEDEC se a flash é uma Winbond W25Q64.
 * @return true se for.
 */
static bool flash_is_supported(void) {
  const uint8_t command = FLASH_CMD_JEDEC_ID;
  uint8_t id[3] = {0U};
  select_flash();
  const bool sent = transmit(&command, 1U);
  const bool received = sent &&
      HAL_SPI_Receive(&hspi1, id, sizeof(id), FLASH_IO_TIMEOUT_MS) == HAL_OK;
  release_flash();
  return received && id[0] == FLASH_JEDEC_WINBOND && id[2] == FLASH_JEDEC_W25Q64;
}

/** @brief Avança o registro de boots e devolve o novo número (ver dronelink_boot_identity.h). */
bool dronelink_boot_identity_init(uint32_t *boot_id) {
  if (boot_id == NULL) {
    return false;
  }
  *boot_id = 0U;
  if (!flash_is_supported()) {
    return false;
  }

  uint32_t newest_id = 0U;
  uint32_t write_address = 0U;
  for (uint32_t offset = 0U; offset < FLASH_SECTOR_SIZE;
       offset += sizeof(boot_record_t)) {
    boot_record_t record;
    if (!read_data(FLASH_JOURNAL_ADDRESS + offset, &record, sizeof(record))) {
      return false;
    }
    if (record_is_valid(&record) && record.boot_id > newest_id) {
      newest_id = record.boot_id;
    } else if (write_address == 0U && record_is_blank(&record)) {
      write_address = FLASH_JOURNAL_ADDRESS + offset;
    }
  }

  uint32_t next_id = newest_id + 1U;
  if (next_id == 0U) {
    next_id = 1U;
  }
  if (write_address == 0U) {
    if (!erase_journal()) {
      return false;
    }
    write_address = FLASH_JOURNAL_ADDRESS;
  }

  boot_record_t next = {
      .magic = BOOT_RECORD_MAGIC,
      .boot_id = next_id,
      .boot_id_inverse = ~next_id,
      .checksum = 0U,
  };
  next.checksum = checksum(&next);
  if (!page_program(write_address, &next, sizeof(next))) {
    return false;
  }

  boot_record_t verification;
  if (!read_data(write_address, &verification, sizeof(verification)) ||
      !record_is_valid(&verification) || verification.boot_id != next_id) {
    return false;
  }
  *boot_id = next_id;
  return true;
}
