/**
 * @file dronelink_boot_identity.h
 * @brief Identidade de boot persistente, guardada na flash SPI W25Q64.
 *
 * Cada boot recebe um número maior que o anterior. A Raspberry Pi repete esse
 * número nos comandos, e o firmware recusa comandos feitos para um boot
 * anterior (por exemplo, depois de um reinício no meio da missão).
 */
#ifndef DRONELINK_BOOT_IDENTITY_H
#define DRONELINK_BOOT_IDENTITY_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Avança o registro persistente de boots na W25Q64 e devolve o novo
 *        número.
 *
 * O registro fica no último setor da flash (0x7FF000), como uma sequência de
 * registros de 16 bytes com marcador, número, número invertido e CRC-32. O
 * novo número é o maior válido encontrado mais 1; quando o setor enche, ele é
 * apagado e o registro recomeça do início. A gravação é conferida por
 * releitura.
 *
 * @param[out] boot_id  Número deste boot. Em caso de falha vale zero, para
 *                      que o protocolo possa anunciar telemetria mas negue
 *                      movimento.
 * @return true se o número foi gravado e conferido na flash.
 */
bool dronelink_boot_identity_init(uint32_t *boot_id);

#endif
