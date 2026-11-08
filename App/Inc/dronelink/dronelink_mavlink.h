/**
 * @file dronelink_mavlink.h
 * @brief Ponto único de inclusão do MAVLink no DroneLink e envio de mensagens.
 *
 * Os headers gerados (Middlewares/Third_Party/mavlink) disparam o aviso
 * "address of packed member" do GCC, que não se aplica ao Cortex-M4 (ele lê
 * palavras desalinhadas). O aviso é silenciado só aqui.
 */
#ifndef DRONELINK_MAVLINK_H
#define DRONELINK_MAVLINK_H

/* Estado do canal MAVLink (inclui o número de sequência de envio) único para
   todo o firmware. Sem isto, mavlink_get_channel_status() guarda o estado numa
   variável static de função inline, e cada .c teria a sua cópia: telemetria e
   protocolo sairiam com sequências independentes, e a Pi não conseguiria
   medir perdas pelos saltos da sequência. O vetor fica em dronelink_protocol.c. */
#define MAVLINK_EXTERNAL_RX_STATUS
struct __mavlink_status;
/**
 * @brief Estados dos canais MAVLink, compartilhados por todos os módulos.
 * @return Vetor de MAVLINK_COMM_NUM_BUFFERS estados.
 */
struct __mavlink_status *dronelink_mavlink_status(void);
#define m_mavlink_status (dronelink_mavlink_status())

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "drone_controller/mavlink.h"
#pragma GCC diagnostic pop

#include "dronelink/dronelink_transport.h"

/**
 * @brief Serializa e enfileira uma mensagem para a Pi.
 *
 * Nunca bloqueia: se a fila estiver cheia, a mensagem é descartada e contada
 * pelo transporte (aparece no SYS_STATUS).
 *
 * @param[in] message   Mensagem já empacotada.
 * @param[in] priority  Fila de destino.
 */
void dronelink_send(const mavlink_message_t *message, dronelink_tx_priority_t priority);

#endif
