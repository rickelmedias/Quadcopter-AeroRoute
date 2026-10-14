/**
 * @file bno08x.c
 * @brief Driver do BNO08x (contratos em bno08x.h).
 *
 * Implementa a camada sh2_Hal_t (I2C2, pinos RST e INT) sobre a qual roda a
 * pilha SH-2 da CEVA, decodifica os relatórios e mantém a sessão viva,
 * reconectando após qualquer falha.
 */
#include "bno08x.h"
#include "bno_i2c_header.h"
#include "capture.h"
#include "sensor_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c.h"
#include "main.h"
#include "sh2/sh2.h"
#include "sh2/sh2_SensorValue.h"
#include "sh2/sh2_err.h"
#include <math.h>
#include <string.h>

/** Intervalo pedido para cada relatório, em us (400 Hz, o máximo do datasheet
    para rotação, giroscópio e aceleração linear). */
#define REPORT_INTERVAL_US 2500U
/** Tempo máximo sondando o I2C após soltar o RST, em ms; cobre os ~80 ms que
    esta placa realmente precisa. */
#define PROBE_WINDOW_MS    400U
/** Sem relatórios por este tempo, a task encerra e refaz a sessão, em ms. */
#define STALL_TIMEOUT_MS   1000U
/** Sem relatórios por este tempo, bno_get_sample() já informa offline, em ms. */
#define ONLINE_STALE_MS    250U
/** Espera antes de uma nova tentativa depois de uma falha, em ms. */
#define RETRY_DELAY_MS     2000U

/** Amostra em construção; usada só pela task. */
static bno_sample_t sample;
/** Última amostra publicada; protegida por seção crítica. */
static bno_sample_t published;
/** Histórico usado para associar cada frame MTF ao mesmo intervalo da IMU. */
static bno_history_t history;
/** O SH-2 avisou que o sensor reiniciou. */
static bool reset_seen;
/** Uma transferência I2C falhou; a sessão atual precisa ser refeita. */
static bool transport_failed;
/** Instante (us, derivado do tick) da última borda do pino INT. */
static volatile uint32_t irq_us;
/** Endereço I2C já deslocado como a HAL espera; 0 até o sensor ser encontrado. */
static uint16_t address;

/** Task que espera a transferência I2C em andamento. */
static TaskHandle_t i2c_waiter;
/** Resultado da última transferência I2C por interrupção. */
static volatile bool i2c_ok;

/** @brief Fim de recepção I2C (interrupção): acorda a task do BNO. */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *h) {
  if (h != &hi2c2 || !i2c_waiter) return;
  BaseType_t wake = pdFALSE;
  i2c_ok = true;
  vTaskNotifyGiveFromISR(i2c_waiter, &wake);
  portYIELD_FROM_ISR(wake);
}

/** @brief Erro I2C (interrupção): acorda a task do BNO com falha. */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *h) {
  if (h != &hi2c2 || !i2c_waiter) return;
  BaseType_t wake = pdFALSE;
  i2c_ok = false;
  vTaskNotifyGiveFromISR(i2c_waiter, &wake);
  portYIELD_FROM_ISR(wake);
}

/**
 * @brief Lê @p length bytes do BNO por interrupção, dormindo até o fim.
 *
 * Com o I2C a 400 kHz e os três relatórios a 400 Hz, o barramento fica ocupado
 * perto de metade do tempo. No modo bloqueante da HAL a CPU ficaria esse tempo
 * todo esperando; aqui ela fica livre para as outras tasks.
 *
 * @return true se a transferência terminou sem erro dentro de @p timeout_ms.
 */
static bool i2c_receive(uint8_t *buffer, uint16_t length, uint32_t timeout_ms) {
  i2c_waiter = xTaskGetCurrentTaskHandle();
  (void)ulTaskNotifyTake(pdTRUE, 0);
  i2c_ok = false;
  if (HAL_I2C_Master_Receive_IT(&hi2c2, address, buffer, length) != HAL_OK) return false;
  if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms))) {
    /* Sem resposta: a sessão será refeita, e o caminho de erro reinicia o
       periférico (HAL_I2C_DeInit/Init) antes da próxima tentativa. */
    return false;
  }
  return i2c_ok;
}

/**
 * @brief Copia a amostra em construção para a publicada.
 */
static void publish(void) {
  /* A interface externa do futuro controlador é Euler, mas a conversão é
     derivada aqui do quatérnio normalizado. O quatérnio continua publicado
     para interpolação temporal e rotação de vetores. Fazer a conversão antes
     da seção crítica mantém a trava curta. */
  bno_sample_t next = sample;
  next.euler_valid = next.rotation.seen && next.angular.seen &&
      attitude_build_euler_state(next.q, next.gyro, &next.euler);
  taskENTER_CRITICAL(); published = next; taskEXIT_CRITICAL();
}

/**
 * @brief Registra uma falha. Só a primeira falha da sessão fica em @c fault.
 *
 * @param[in] fault  Motivo.
 */
static void fail(bno_fault_t fault) {
  if (sample.fault == BNO_FAULT_NONE) sample.fault = fault;
  ++sample.errors;
}

/**
 * @brief Destrava o barramento I2C2 gerando pulsos de clock manualmente.
 *
 * Uma transferência que termina em timeout pode deixar um escravo segurando o
 * SDA em nível baixo no meio de um byte. HAL_I2C_DeInit()/Init() reiniciam o
 * periférico e soltam os pinos, mas nunca geram clock no SCL, então um
 * escravo preso assim continua preso. Aqui são gerados até nove pulsos de
 * clock e depois um STOP. Usado só no caminho de erro.
 */
static void i2c2_bus_recovery(void) {
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  /* Coloca os dois bits de saída em nível alto antes de mudar os pinos para
     saída open-drain. A mudança de modo vale na hora, então um bit baixo
     antigo puxaria o barramento para baixo no instante em que o pino vira
     saída. */
  HAL_GPIO_WritePin(BNO085_SCL_GPIO_Port, BNO085_SCL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BNO085_SDA_GPIO_Port, BNO085_SDA_Pin, GPIO_PIN_SET);
  gpio.Pin = BNO085_SCL_Pin;
  HAL_GPIO_Init(BNO085_SCL_GPIO_Port, &gpio);
  gpio.Pin = BNO085_SDA_Pin;
  HAL_GPIO_Init(BNO085_SDA_GPIO_Port, &gpio);
  for (int i = 0; i < 9 &&
       HAL_GPIO_ReadPin(BNO085_SDA_GPIO_Port, BNO085_SDA_Pin) == GPIO_PIN_RESET; ++i) {
    HAL_GPIO_WritePin(BNO085_SCL_GPIO_Port, BNO085_SCL_Pin, GPIO_PIN_RESET);
    vTaskDelay(1);
    HAL_GPIO_WritePin(BNO085_SCL_GPIO_Port, BNO085_SCL_Pin, GPIO_PIN_SET);
    vTaskDelay(1);
  }
  /* STOP: o SDA sobe enquanto o SCL já está alto. */
  HAL_GPIO_WritePin(BNO085_SDA_GPIO_Port, BNO085_SDA_Pin, GPIO_PIN_RESET);
  vTaskDelay(1);
  HAL_GPIO_WritePin(BNO085_SCL_GPIO_Port, BNO085_SCL_Pin, GPIO_PIN_SET);
  vTaskDelay(1);
  HAL_GPIO_WritePin(BNO085_SDA_GPIO_Port, BNO085_SDA_Pin, GPIO_PIN_SET);
  vTaskDelay(1);
}

/* ---- implementação de sh2_Hal_t --------------------------------------- */

/**
 * @brief Relógio em microssegundos exigido pela pilha SH-2 (resolução de 1 ms).
 *
 * @param[in] self  Não usado.
 * @return Tick atual convertido para us.
 */
static uint32_t now_us(sh2_Hal_t *self) { (void)self; return HAL_GetTick()*1000U; }

/**
 * @brief Callback de interrupção externa da HAL: registra o instante da borda
 *        do pino INT do BNO.
 *
 * @param[in] pin  Pino que gerou a interrupção.
 */
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  if (pin == BNO085_INT_Pin) irq_us = HAL_GetTick()*1000U;
}

/**
 * @brief Abre o sensor para a pilha SH-2: pulsa o RST e procura o endereço I2C.
 *
 * @param[in] self  Não usado.
 * @return SH2_OK se o sensor respondeu em 0x4A ou 0x4B dentro de
 *         ::PROBE_WINDOW_MS; SH2_ERR_IO caso contrário.
 */
static int hub_open(sh2_Hal_t *self) {
  (void)self;
  sample.address = 0;
  sample.fault = BNO_FAULT_NONE;
  HAL_GPIO_WritePin(BNO085_RST_GPIO_Port, BNO085_RST_Pin, GPIO_PIN_RESET);
  vTaskDelay(pdMS_TO_TICKS(10));
  HAL_GPIO_WritePin(BNO085_RST_GPIO_Port, BNO085_RST_Pin, GPIO_PIN_SET);
  vTaskDelay(pdMS_TO_TICKS(20));
  /* O tempo de boot após o RST varia com o firmware carregado, então a
     sondagem é repetida em vez de apostar num atraso fixo: sondar uma vez aos
     20 ms falha de vez num chip que precisa de ~80 ms. A repetição retorna no
     instante em que ele responde, então um boot rápido não custa nada e um
     lento ainda funciona. Responder logo importa porque a notificação de
     reset do SH-2 é enviada uma única vez. A HAL quer o endereço deslocado;
     as duas opções do pino SA0 são testadas. */
  sample.probe_ms = 0;
  uint32_t started = HAL_GetTick();
  do {
    for (unsigned a = 0x4A; a <= 0x4B; ++a) {
      if (HAL_I2C_IsDeviceReady(&hi2c2, a<<1, 2, 20) == HAL_OK) {
        address = (uint16_t)(a<<1);
        sample.address = (uint8_t)a;
        sample.probe_ms = (uint16_t)(HAL_GetTick()-started);
        publish();
        return SH2_OK;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  } while (HAL_GetTick()-started < PROBE_WINDOW_MS);
  fail(BNO_FAULT_NO_DEVICE); publish();
  return SH2_ERR_IO;
}

/**
 * @brief Fecha o sensor para a pilha SH-2, mantendo-o em reset.
 *
 * @param[in] self  Não usado.
 */
static void hub_close(sh2_Hal_t *self) {
  (void)self;
  HAL_GPIO_WritePin(BNO085_RST_GPIO_Port, BNO085_RST_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Lê um pacote SHTP do sensor para a pilha SH-2.
 *
 * Só lê quando o pino INT (ativo em nível baixo) indica dado pendente. Lê
 * primeiro o cabeçalho de 4 bytes e depois o pacote completo, conferindo que
 * o cabeçalho repetido combina.
 *
 * @param[in]  self       Não usado.
 * @param[out] buffer     Destino do pacote.
 * @param[in]  capacity   Tamanho do destino.
 * @param[out] timestamp  Instante da última interrupção, em us.
 * @return Tamanho do pacote, 0 se não havia nada, ou SH2_ERR_IO em falha
 *         (que marca a sessão para ser refeita).
 */
static int hub_read(sh2_Hal_t *self, uint8_t *buffer, unsigned capacity, uint32_t *timestamp) {
  (void)self;
  /* O INT é ativo em nível baixo: alto significa que o sensor não tem nada
     na fila. */
  if (transport_failed ||
      HAL_GPIO_ReadPin(BNO085_INT_GPIO_Port, BNO085_INT_Pin) == GPIO_PIN_SET) {
    vTaskDelay(1);
    return 0;
  }
  uint8_t header[4];
  *timestamp = irq_us;
  if (!i2c_receive(header, 4, 20)) {
    fail(BNO_FAULT_TRANSPORT); goto failed;
  }
  unsigned length = ((unsigned)header[0] | (unsigned)header[1]<<8) & 0x7FFFU;
  if (length == 0) { vTaskDelay(1); return 0; }
  if (length < 4 || length > capacity) { fail(BNO_FAULT_PROTOCOL); goto failed; }
  /* Limita o canal antes que o pacote chegue ao parser SHTP incorporado - ver
     bno_i2c_header_channel_valid(). A leitura completa abaixo é conferida
     contra este mesmo cabeçalho por bno_i2c_header_matches(), que exige o
     byte 2 idêntico, então validar o cabeçalho espiado cobre o pacote que o
     parser realmente recebe. */
  if (!bno_i2c_header_channel_valid(header)) { fail(BNO_FAULT_PROTOCOL); goto failed; }
  if (length == 4) { memcpy(buffer, header, 4); return 4; }
  /* Toda transferência I2C repete o cabeçalho SHTP, então se lê o tamanho
     completo, e não tamanho-4, e nunca como leitura de registrador
     (Mem_Read). */
  if (!i2c_receive(buffer, (uint16_t)length, 40)) {
    fail(BNO_FAULT_TRANSPORT); goto failed;
  }
  if (!bno_i2c_header_matches(header, buffer)) { fail(BNO_FAULT_PROTOCOL); goto failed; }
  return (int)length;
failed:
  transport_failed = true; publish();
  return SH2_ERR_IO;
}

/**
 * @brief Envia um pacote SHTP ao sensor para a pilha SH-2.
 *
 * @param[in] self    Não usado.
 * @param[in] buffer  Pacote.
 * @param[in] length  Tamanho do pacote.
 * @return Bytes enviados, ou SH2_ERR_IO em falha.
 */
static int hub_write(sh2_Hal_t *self, uint8_t *buffer, unsigned length) {
  (void)self;
  if (transport_failed) { vTaskDelay(1); return SH2_ERR_IO; }
  if (HAL_I2C_Master_Transmit(&hi2c2, address, buffer, length, 40) != HAL_OK) {
    fail(BNO_FAULT_TRANSPORT);
    transport_failed = true; publish();
    return SH2_ERR_IO;
  }
  return (int)length;
}

/** Tabela de funções de hardware entregue à pilha SH-2. */
static sh2_Hal_t hub = {hub_open, hub_close, hub_read, hub_write, now_us};

/* ---- decodificação dos relatórios ------------------------------------- */

/**
 * @brief Callback de eventos assíncronos do SH-2. Trata o aviso de reset do
 *        sensor.
 *
 * @param[in] cookie  Não usado.
 * @param[in] e       Evento.
 */
static void event(void *cookie, sh2_AsyncEvent_t *e) {
  (void)cookie;
  if (e->eventId == SH2_RESET) {
    reset_seen = true; ++sample.resets;
    sample.rotation.seen = sample.angular.seen = sample.acceleration.seen = false;
  }
}

/**
 * @brief Atualiza a saúde de um tipo de relatório com uma nova amostra.
 *
 * @param[in,out] s  Saúde do relatório.
 * @param[in]     v  Amostra decodificada.
 * @return false para uma duplicata (mesma sequência), que não deve
 *         sobrescrever a amostra.
 */
static bool update_stats(bno_report_stats_t *s, const sh2_SensorValue_t *v) {
  if (s->seen) {
    uint8_t delta = (uint8_t)(v->sequence - s->sequence);
    if (!delta) return false;
    if (delta < 128) s->gaps += delta - 1U; /* senão, assume reordenação e não perda */
  }
  s->seen = true;
  ++s->count;
  s->sequence = v->sequence;
  s->last_ms = HAL_GetTick();
  s->accuracy = v->status & 3;
  return true;
}

/**
 * @brief Callback de relatórios do SH-2: decodifica, valida, guarda na
 *        amostra e grava na captura.
 *
 * @param[in] cookie  Não usado.
 * @param[in] e       Evento de sensor ainda codificado.
 */
static void sensor_event(void *cookie, sh2_SensorEvent_t *e) {
  (void)cookie;
  sh2_SensorValue_t v;
  if (sh2_decodeSensorEvent(&v, e) != SH2_OK) { ++sample.errors; return; }
  if (v.sensorId == SH2_ROTATION_VECTOR) {
    sh2_RotationVectorWAcc_t *q = &v.un.rotationVector;
    /* Um quatérnio muito longe da esfera unitária está corrompido, não só
       impreciso. */
    float norm = q->real*q->real + q->i*q->i + q->j*q->j + q->k*q->k;
    if (!isfinite(norm) || norm < 0.8F || norm > 1.2F) { ++sample.errors; return; }
    if (!update_stats(&sample.rotation, &v)) return;
    float scale = 1.0F/sqrtf(norm);
    sample.q[0] = q->real*scale; sample.q[1] = q->i*scale;
    sample.q[2] = q->j*scale;    sample.q[3] = q->k*scale;
    taskENTER_CRITICAL();
    bno_history_push_rotation(&history, (uint32_t)v.timestamp,
                              sample.rotation.count, sample.q);
    taskEXIT_CRITICAL();
    capture_bno(CAP_BNO_ROTATION, sample.q, sample.rotation.accuracy,
                sample.rotation.sequence, sample.rotation.count,
                v.timestamp, v.delay);
  } else if (v.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    if (!update_stats(&sample.angular, &v)) return;
    sample.gyro[0] = v.un.gyroscope.x;
    sample.gyro[1] = v.un.gyroscope.y;
    sample.gyro[2] = v.un.gyroscope.z;
    taskENTER_CRITICAL();
    bno_history_push_gyro(&history, (uint32_t)v.timestamp,
                          sample.angular.count, sample.gyro);
    taskEXIT_CRITICAL();
    { const float g[4] = {sample.gyro[0], sample.gyro[1], sample.gyro[2], 0.0F};
      capture_bno(CAP_BNO_GYRO, g, sample.angular.accuracy,
                  sample.angular.sequence, sample.angular.count,
                  v.timestamp, v.delay); }
  } else if (v.sensorId == SH2_LINEAR_ACCELERATION) {
    if (!update_stats(&sample.acceleration, &v)) return;
    sample.lin_accel[0] = v.un.linearAcceleration.x;
    sample.lin_accel[1] = v.un.linearAcceleration.y;
    sample.lin_accel[2] = v.un.linearAcceleration.z;
    taskENTER_CRITICAL();
    bno_history_push_accel(&history, (uint32_t)v.timestamp,
                           sample.acceleration.count, sample.lin_accel);
    taskEXIT_CRITICAL();
    { const float a[4] = {sample.lin_accel[0], sample.lin_accel[1], sample.lin_accel[2], 0.0F};
      capture_bno(CAP_BNO_ACCEL, a, sample.acceleration.accuracy,
                  sample.acceleration.sequence, sample.acceleration.count,
                  v.timestamp, v.delay); }
  }
}

/**
 * @brief Habilita os três relatórios a 400 Hz: rotação, giroscópio calibrado
 *        e aceleração linear.
 *
 * @return true se os três foram aceitos pelo sensor.
 */
static bool configure(void) {
  sh2_SensorConfig_t c = {0};
  c.reportInterval_us = REPORT_INTERVAL_US;
  return sh2_setSensorConfig(SH2_ROTATION_VECTOR, &c) == SH2_OK &&
         sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &c) == SH2_OK &&
         /* Aceleração linear em vez do acelerômetro bruto: o hub remove a
            gravidade usando a própria solução de atitude, o que é melhor que
            reconstruí-la aqui. Três relatórios a 400 Hz ocupam cerca de
            metade do I2C a 400 kHz, então este substitui o bruto. */
         sh2_setSensorConfig(SH2_LINEAR_ACCELERATION, &c) == SH2_OK;
}

/**
 * @brief Diz se algum dos três relatórios está faltando ou velho demais.
 *
 * Quem lê usa o limite mais curto (::ONLINE_STALE_MS), para ver o sensor
 * ficar offline logo; a task usa ::STALL_TIMEOUT_MS antes de derrubar a
 * sessão por algo que pode ser só um soluço.
 *
 * @param[in] s      Amostra a avaliar.
 * @param[in] now    Tick atual, em ms.
 * @param[in] limit  Idade máxima aceita, em ms.
 * @return true se algum relatório nunca chegou ou tem mais que @p limit.
 */
static bool reports_stalled(const bno_sample_t *s, uint32_t now, uint32_t limit) {
  return !s->rotation.seen || !s->angular.seen || !s->acceleration.seen ||
         now - s->rotation.last_ms     > limit ||
         now - s->angular.last_ms      > limit ||
         now - s->acceleration.last_ms > limit;
}

/* ---- API pública ------------------------------------------------------ */

/** @brief Cópia segura da amostra, com @c online recalculado (ver bno08x.h). */
void bno_get_sample(bno_sample_t *out) {
  taskENTER_CRITICAL(); *out = published; taskEXIT_CRITICAL();
  out->online = out->online && !reports_stalled(out, HAL_GetTick(), ONLINE_STALE_MS);
}

bool bno_get_aligned(uint32_t end_us, uint32_t interval_us,
                     bno_aligned_sample_t *out) {
  bool ok;
  taskENTER_CRITICAL();
  ok = bno_history_align(&history, end_us, interval_us, out);
  taskEXIT_CRITICAL();
  return ok;
}

/**
 * @brief Task do BNO08x: abre a sessão SH-2, identifica e configura o sensor,
 *        atende os relatórios e refaz tudo após qualquer falha.
 *
 * @param[in] argument  Não usado.
 */
static void bno_task(void *argument) {
  (void)argument;
  uint32_t last_diag_ms = 0;
  for (;;) {
    capture_diag_poll(&last_diag_ms, CAP_CLOCK_RESYNCS_UNKNOWN);
    sample.online = false;
    sample.rotation.seen = sample.angular.seen = sample.acceleration.seen = false;
    taskENTER_CRITICAL(); bno_history_reset(&history); taskEXIT_CRITICAL();
    transport_failed = false;
    reset_seen = false;
    publish();

    if (sh2_open(&hub, event, NULL) == SH2_OK) {
      sh2_setSensorCallback(sensor_event, NULL);
      if (!reset_seen) {
        fail(BNO_FAULT_NO_RESET);
      } else {
        sh2_ProductIds_t ids = {0};
        if (sh2_getProdIds(&ids) != SH2_OK || !ids.numEntries) {
          fail(BNO_FAULT_IDENTITY);
        } else {
          sample.part = ids.entry[0].swPartNumber;
          sample.build = ids.entry[0].swBuildNumber;
          sample.version_major = ids.entry[0].swVersionMajor;
          sample.version_minor = ids.entry[0].swVersionMinor;
          sample.version_patch = ids.entry[0].swVersionPatch;
          reset_seen = false;
          publish();
          sample.online = configure();
          if (!sample.online) fail(BNO_FAULT_CONFIG);
        }
      }

      uint32_t settled = HAL_GetTick();
      uint32_t last_rotation = sample.rotation.count;
      uint32_t last_total = 0;
      while (sample.online && !transport_failed) {
        capture_diag_poll(&last_diag_ms, CAP_CLOCK_RESYNCS_UNKNOWN);
        sh2_service();
        /* Um reset durante a operação apaga os relatórios habilitados, então
           eles são reabilitados e a verificação de travamento ganha um novo
           período de carência. */
        if (reset_seen) {
          reset_seen = false;
          sample.online = configure();
          settled = HAL_GetTick();
        }
        uint32_t now = HAL_GetTick();
        if (now - settled > 2000U && reports_stalled(&sample, now, STALL_TIMEOUT_MS)) {
          fail(BNO_FAULT_STALLED);
          break;
        }
        /* Publica só quando chegou relatório novo. hub_read() já dorme 1 ms
           quando o INT diz que não há nada, então este laço não gira em vão,
           e com dado pendente ele lê o próximo pacote sem esperar o tick. */
        const uint32_t total = sample.rotation.count + sample.angular.count +
                               sample.acceleration.count;
        if (total != last_total) {
          last_total = total;
          publish();
          if (sample.rotation.count != last_rotation) {
            last_rotation = sample.rotation.count;
            log_bno(&sample);
          }
        }
      }
      sh2_close();
    }

    sample.online = false;
    publish();
    /* Dá ao periférico e a um barramento possivelmente travado um começo
       limpo antes do próximo reset físico. */
    HAL_I2C_DeInit(&hi2c2);
    i2c2_bus_recovery();
    MX_I2C2_Init();
    vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
  }
}

/** @brief Cria a task do BNO08x (ver bno08x.h). */
bool bno_start(void) {
  return xTaskCreate(bno_task, "BNO08X", 1024, NULL, tskIDLE_PRIORITY+2, NULL) == pdPASS;
}
