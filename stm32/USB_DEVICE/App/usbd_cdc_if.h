/**
 * @file usbd_cdc_if.h
 * @brief Interface da porta serial USB (CDC) usada para logs e captura.
 */
#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H
#include "usbd_cdc.h"
#include <stdbool.h>

/** Callbacks da classe CDC registrados na pilha USB da ST. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/**
 * @brief Envia um bloco pela USB, sem esperar.
 *
 * Os dados são copiados para um buffer interno, então o chamador pode
 * reaproveitar o seu logo em seguida.
 *
 * @param[in] buffer  Dados a enviar.
 * @param[in] length  Tamanho, até 2048 bytes.
 * @return USBD_OK se o envio começou; USBD_BUSY se a USB não está configurada
 *         ou ainda envia o bloco anterior; USBD_FAIL se o bloco é grande
 *         demais.
 */
uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length);

/**
 * @brief Informa se o programa no PC ativou o DTR (porta aberta e pronta).
 * @return Estado atual do DTR; false após reset ou desconexão da USB.
 */
bool CDC_HostDTR(void);

/**
 * @brief Contador de fechamentos: soma 1 cada vez que o DTR passa de ativo
 *        para inativo.
 *
 * Permite perceber que a porta foi fechada e reaberta entre duas leituras.
 *
 * @return O contador.
 */
uint32_t CDC_CloseEpoch(void);

/**
 * @brief Bytes recebidos do PC (chamada na interrupção USB).
 *
 * A definição padrão é fraca e descarta os bytes. Um perfil que aceita
 * comandos (por exemplo, o teste de motores) a redefine; ela deve só copiar
 * os bytes para uma fila e voltar.
 *
 * @param[in] data    Bytes recebidos.
 * @param[in] length  Quantidade, até 64.
 */
void CDC_OnReceive(const uint8_t *data, uint32_t length);
#endif
