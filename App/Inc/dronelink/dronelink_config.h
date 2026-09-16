/**
 * @file dronelink_config.h
 * @brief Constantes do DroneLink: identidades MAVLink, períodos, limites e
 *        tamanhos de buffer.
 */
#ifndef DRONELINK_CONFIG_H
#define DRONELINK_CONFIG_H

/** ID de sistema MAVLink do drone (compartilhado pelo STM32 e pela Pi). */
#define DRONELINK_SYSTEM_ID                 1U
/** ID de componente MAVLink da controladora de voo (este firmware). */
#define DRONELINK_FC_COMPONENT_ID           1U
/** ID de componente MAVLink da Raspberry Pi; mensagens de outras origens são ignoradas. */
#define DRONELINK_PI_COMPONENT_ID           191U
/** Versão principal do protocolo DroneLink. */
#define DRONELINK_PROTOCOL_MAJOR            1U
/** Versão secundária do protocolo DroneLink. */
#define DRONELINK_PROTOCOL_MINOR            0U
/** Maior timeout aceito num comando de trecho (leg), em ms. */
#define DRONELINK_MAX_LEG_TIMEOUT_MS         120000UL
/** Período de envio do HEARTBEAT, em ms. */
#define DRONELINK_HEARTBEAT_PERIOD_MS        500UL
/** Período de envio do SYS_STATUS, em ms. */
#define DRONELINK_STATUS_PERIOD_MS           500UL
/** Período de envio do DRONE_PROTOCOL_INFO, em ms. */
#define DRONELINK_PROTOCOL_INFO_PERIOD_MS    1000UL
/** Sem HEARTBEAT da Pi por este tempo, o enlace é considerado degradado: novos
    trechos são recusados e, em voo, é pedido HOLD. Em ms. */
#define DRONELINK_LINK_DEGRADED_MS           1000UL
/** Sem HEARTBEAT por este tempo e com navegação degradada, é pedido pouso. Em ms. */
#define DRONELINK_LINK_LOST_MS               2000UL
/** Sem enlace (ou sem autorização do operador) por este tempo, é pedido pouso
    em qualquer caso. Em ms. */
#define DRONELINK_FAILSAFE_LAND_MS           5000UL
/** Tamanho do buffer DMA de recepção da USART1, em bytes. */
#define DRONELINK_RX_DMA_SIZE                512U
/** Tamanho do stream buffer entre a interrupção e a task, em bytes. */
#define DRONELINK_RX_STREAM_SIZE             1024U
/** Frames em cada uma das duas filas de transmissão. */
#define DRONELINK_TX_QUEUE_DEPTH             6U

#endif
