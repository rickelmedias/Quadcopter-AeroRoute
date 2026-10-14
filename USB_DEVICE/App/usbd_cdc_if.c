/**
 * @file usbd_cdc_if.c
 * @brief Porta serial USB (CDC): envio, DTR e callbacks da classe
 *        (contratos em usbd_cdc_if.h).
 */
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
/** Memória da classe CDC. Instância única e alocação fixa, segura também
    durante o reset feito na interrupção USB. */
static USBD_CDC_HandleTypeDef class_memory;
/** Buffer de recepção; o conteúdo vai para CDC_OnReceive(). */
static uint8_t rx[64];
/** Buffer de envio; CDC_Transmit_FS copia os dados para cá. */
static uint8_t tx[2048];
/** Último estado do DTR informado pelo PC. */
static volatile bool host_dtr;
/** Quantas vezes o DTR passou de ativo para inativo. */
static volatile uint32_t close_epoch;

/** @brief Estado atual do DTR (ver usbd_cdc_if.h). */
bool CDC_HostDTR(void) { return host_dtr; }
/** @brief Contador de fechamentos da porta (ver usbd_cdc_if.h). */
uint32_t CDC_CloseEpoch(void) { return close_epoch; }

/**
 * @brief Atualiza o DTR e conta a transição de ativo para inativo.
 *
 * @param[in] value  Novo estado do DTR.
 */
static void set_dtr(bool value) {
  if (host_dtr && !value) ++close_epoch;
  host_dtr = value;
}

/** Configuração de linha informada ao PC: 115200 8N1 (virtual no CDC). */
static uint8_t line_coding[7] = {0x00,0xC2,0x01,0,0,0,8};

/**
 * @brief Alocador da pilha USB (USBD_malloc em usbd_conf.h).
 *
 * @param[in] size  Tamanho pedido.
 * @return A memória fixa da classe, ou NULL se o tamanho não couber.
 */
void *usb_class_alloc(uint32_t size) { return size <= sizeof(class_memory) ? &class_memory : NULL; }

/**
 * @brief Liberação da pilha USB (USBD_free em usbd_conf.h). Nada a fazer,
 *        porque a memória é fixa.
 *
 * @param[in] p  Não usado.
 */
void usb_class_free(void *p) { (void)p; }

/**
 * @brief Callback da classe: dispositivo configurado pelo PC.
 * @return USBD_OK.
 */
static int8_t init(void) {
  set_dtr(false);
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rx);
  return USBD_OK;
}

/**
 * @brief Callback da classe: dispositivo desconfigurado ou desconectado.
 * @return USBD_OK.
 */
static int8_t deinit(void) { set_dtr(false); return USBD_OK; }

/**
 * @brief Callback da classe: requisições de controle CDC do PC.
 *
 * Trata SET_CONTROL_LINE_STATE (DTR) e SET/GET_LINE_CODING.
 *
 * @param[in]     cmd  Código da requisição.
 * @param[in,out] p    Dados da requisição (ou a própria requisição de setup,
 *                     quando ela não tem dados).
 * @param[in]     n    Tamanho dos dados.
 * @return USBD_OK.
 */
static int8_t control(uint8_t cmd, uint8_t *p, uint16_t n) {
  /* Em requisições sem dados, esta HAL passa a própria requisição de setup. */
  if (cmd == CDC_SET_CONTROL_LINE_STATE && n == 0 && p) {
    const USBD_SetupReqTypedef *req = (const USBD_SetupReqTypedef *)p;
    set_dtr((req->wValue & 1U) != 0);
  }
  if (cmd == CDC_SET_LINE_CODING && n == 7) memcpy(line_coding,p,7);
  if (cmd == CDC_GET_LINE_CODING && n == 7) memcpy(p,line_coding,7);
  return USBD_OK;
}

/** @brief Padrão: descarta os bytes (ver usbd_cdc_if.h). */
__attribute__((weak)) void CDC_OnReceive(const uint8_t *data, uint32_t length) {
  (void)data; (void)length;
}

/**
 * @brief Callback da classe: dados recebidos do PC. São entregues a
 *        CDC_OnReceive() e a recepção é rearmada.
 *
 * @param[in] p  Bytes recebidos.
 * @param[in] n  Quantidade.
 * @return USBD_OK.
 */
static int8_t receive(uint8_t *p, uint32_t *n) {
  if (p && n) CDC_OnReceive(p, *n);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rx);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return USBD_OK;
}

/**
 * @brief Callback da classe: fim de um envio. Nada a fazer.
 *
 * @param[in] p   Não usado.
 * @param[in] n   Não usado.
 * @param[in] ep  Não usado.
 * @return USBD_OK.
 */
static int8_t complete(uint8_t *p, uint32_t *n, uint8_t ep) {
  (void)p; (void)n; (void)ep; return USBD_OK;
}

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {init,deinit,control,receive,complete};

/** @brief Envio sem espera, com cópia para o buffer interno (ver usbd_cdc_if.h). */
uint8_t CDC_Transmit_FS(uint8_t *buffer, uint16_t length) {
  if (length > sizeof(tx)) return USBD_FAIL;
  /* A IRQ do USB (prioridade 6) fica mascarada por esta seção crítica. */
  taskENTER_CRITICAL();
  USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassData;
  uint8_t result = USBD_BUSY;
  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc && !cdc->TxState) {
    memcpy(tx, buffer, length);
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx, length);
    result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  }
  taskEXIT_CRITICAL();
  return result;
}
