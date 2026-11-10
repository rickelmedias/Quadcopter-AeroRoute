/**
 * @file capture.c
 * @brief Captura binária para replay (contratos em capture.h).
 *
 * Só é compilada de verdade no build Capture (CAPTURE_ENABLED = 1). No build
 * normal, todas as funções públicas são vazias (bloco no fim do arquivo).
 */
#include "capture.h"
#include "capture_session.h"
#include "main.h"   /* HAL_GetTick */
#include "sensor_log.h"
#include "displacement.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "semphr.h"
#include <string.h>

#if CAPTURE_ENABLED

/** Tamanho da fila de registros, em bytes: cerca de um quarto de segundo da
    taxa esperada. Os produtores nunca esperam por ela: a aquisição roda na
    taxa do sensor e não pode ser cadenciada pela USB, então fila cheia custa
    um descarte contado em vez de uma amostra atrasada. O replay vê o buraco
    na sequência e sabe que faltam dados, em vez de supor que o veículo ficou
    parado. */
#define CAPTURE_QUEUE_BYTES  8192U
/** Bytes enviados à USB por vez. */
#define CAPTURE_CHUNK_BYTES  512U
/** Pilha da task de captura, em words. */
#define CAPTURE_TASK_STACK   512U
/** Espera máxima dos produtores pela trava, em ticks, antes de desistir e
    contar um descarte. Dois ticks é muito mais que os poucos microssegundos
    que um registro leva para ser montado e enfileirado, então a disputa nunca
    deveria chegar a isso; o limite existe para que uma task de sensor nunca
    fique presa atrás da captura. */
#define CAPTURE_LOCK_TICKS   2U

/** Fila de bytes entre os produtores e a task de captura. */
static StreamBufferHandle_t queue;
/** Trava dos produtores (mutex). */
static SemaphoreHandle_t lock_handle;
/** Estado da sessão de captura. */
static capture_session_t session;
/** Valor de CDC_CloseEpoch() quando a sessão começou; muda se a porta fechar. */
static uint32_t session_close_epoch;

/** @brief Diz se os produtores podem gravar agora (ver capture.h). */
bool capture_is_active(void) {
  return session.active && CDC_HostDTR() && session_close_epoch == CDC_CloseEpoch();
}

/** Próximo número de sequência; só é alterado com a trava. */
static uint16_t next_seq;
/** Descartes por fila cheia; consomem número de sequência. */
static uint32_t dropped_full;
/** Descartes por registro grande demais; consomem número de sequência. */
static uint32_t dropped_oversize;
/** Descartes por não conseguir a trava; NÃO consomem número de sequência. */
static volatile uint32_t dropped_contention;

/** @brief Soma dos três contadores de descarte (ver capture.h). */
uint32_t capture_dropped(void) {
  taskENTER_CRITICAL();
  const uint32_t d = dropped_full + dropped_oversize + dropped_contention;
  taskEXIT_CRITICAL();
  return d;
}

/**
 * @brief Tenta pegar a trava dos produtores, esperando no máximo
 *        ::CAPTURE_LOCK_TICKS.
 *
 * Os produtores rodam em tasks diferentes, e um stream buffer só aceita um
 * escritor, então é preciso exclusão mútua. Tem de ser um mutex, e não uma
 * seção crítica: xStreamBufferSend() é chamada do kernel, e o kernel proíbe
 * isso dentro de seção crítica (sbSEND_COMPLETED() suspende e retoma o
 * escalonador internamente).
 *
 * A trava também precisa cobrir o número de sequência, e não só o envio.
 * Atribuir o número separadamente deixaria dois produtores pegarem 5 e 6 e
 * enfileirarem o 6 antes do 5, e um leitor que calcula buracos pela sequência
 * acusaria uma perda que nunca aconteceu - destruindo o único sinal que
 * permite ao replay distinguir um período parado de dados faltando.
 *
 * Por isso next_seq é alterado num único lugar: emit(), com a trava. Nada mais
 * pode incrementá-lo. Uma versão antiga também o incrementava no caminho de
 * falha da trava, dentro de uma seção crítica, para que todo descarte
 * deixasse buraco. Estava errado duas vezes: uma seção crítica não exclui um
 * dono da trava que apenas foi preemptado, então os dois incrementos podiam
 * perder uma atualização; e prometia uma propriedade que é melhor declarar
 * com honestidade do que fingir. Descartes por disputa são contados à parte e
 * NÃO deixam buraco - ver capture_dropped().
 *
 * @return true se a trava foi obtida.
 */
static bool capture_lock(void) {
  return lock_handle != NULL &&
         xSemaphoreTake(lock_handle, CAPTURE_LOCK_TICKS) == pdTRUE;
}

/**
 * @brief Fecha o registro, consome o número de sequência e o coloca na fila.
 *
 * O número é consumido mesmo que o registro não caiba, para que um descarte
 * por fila cheia ou por tamanho deixe um buraco visível em vez de parecer um
 * momento calmo. Deve ser chamada com a trava.
 *
 * @param[in,out] w  Registro montado.
 */
static void emit(cap_writer_t *w) {
  ++next_seq;
  const uint16_t size = cap_finish(w);
  if (size == 0U) { ++dropped_oversize; return; }
  /* Um stream buffer aceita escrita parcial; meio registro dessincronizaria
     tudo o que vem depois, então o espaço é conferido antes. */
  if (xStreamBufferSpacesAvailable(queue) < size) { ++dropped_full; return; }
  if (xStreamBufferSend(queue, w->buf, size, 0) != size) ++dropped_full;
}

/**
 * @brief Conta um descarte por disputa da trava.
 *
 * Só conta - o número de sequência não pode ser tocado sem a trava, e fingir
 * um buraco aqui corromperia justamente o sinal que torna as perdas reais
 * legíveis. Por isso o DIAG informa este contador separadamente.
 */
static void drop_contended(void) {
  taskENTER_CRITICAL();
  ++dropped_contention;
  taskEXIT_CRITICAL();
}

/** @brief Grava uma fotografia observada pela task (ver capture.h). */
void capture_observation(uint32_t frames, uint32_t previous_sensor_ms,
                         bool have_previous, bool gap_usable,
                         bool range_valid, bool flow_valid) {
  const uint32_t now = HAL_GetTick();
  if (!queue || !capture_is_active()) return;
  if (!capture_lock()) { drop_contended(); return; }
  uint8_t buf[CAP_MAX_RECORD];
  cap_writer_t w;
  cap_begin(&w, buf, sizeof buf, CAP_TYPE_OBSERVATION, next_seq, now);
  cap_u32(&w, frames);
  cap_u32(&w, previous_sensor_ms);
  cap_u8(&w, (uint8_t)((have_previous ? 1U : 0U) | (gap_usable ? 2U : 0U) |
                       (range_valid ? 4U : 0U) | (flow_valid ? 8U : 0U)));
  emit(&w);
  xSemaphoreGive(lock_handle);
}

/** @brief Emite DIAG a cada 1 s (ver capture.h). */
void capture_diag_poll(uint32_t *last_ms, uint32_t clock_resyncs) {
  const uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - *last_ms) >= 1000U) {
    *last_ms = now;
    capture_diag(clock_resyncs);
  }
}

/** @brief Grava um frame do MTF-01 (ver capture.h). */
void capture_mtf(int16_t flow_x, int16_t flow_y, uint32_t distance_mm,
                 uint8_t quality, uint8_t distance_status, uint8_t strength,
                 uint8_t precision, bool range_valid, bool flow_valid,
                 uint32_t sensor_ms, uint32_t frames) {
  const uint32_t recv_ms = HAL_GetTick();
  if (!queue || !capture_is_active()) return;
  if (!capture_lock()) { drop_contended(); return; }
  uint8_t buf[CAP_MAX_RECORD];
  cap_writer_t w;
  cap_begin(&w, buf, sizeof buf, CAP_TYPE_MTF, next_seq, recv_ms);
  cap_i16(&w, flow_x);
  cap_i16(&w, flow_y);
  cap_u32(&w, distance_mm);
  cap_u8(&w, quality);
  cap_u8(&w, distance_status);
  cap_u8(&w, strength);
  cap_u8(&w, precision);
  cap_u8(&w, (uint8_t)((range_valid ? 1U : 0U) | (flow_valid ? 2U : 0U)));
  /* O relógio do próprio sensor, separado do nosso tick de recepção: a
     relação entre os dois é exatamente o que o replay precisa estudar. */
  cap_u32(&w, sensor_ms);
  cap_u32(&w, frames);
  emit(&w);
  xSemaphoreGive(lock_handle);
}

/** @brief Grava um relatório do BNO08x (ver capture.h). */
void capture_bno(cap_bno_report_t report, const float v[4], uint8_t accuracy,
                 uint8_t sequence, uint32_t count,
                 uint64_t sensor_us, uint32_t delay_us) {
  const uint32_t recv_ms = HAL_GetTick();
  if (!queue || !capture_is_active()) return;
  if (!capture_lock()) { drop_contended(); return; }
  uint8_t buf[CAP_MAX_RECORD];
  cap_writer_t w;
  cap_begin(&w, buf, sizeof buf, CAP_TYPE_BNO, next_seq, recv_ms);
  cap_u8(&w, (uint8_t)report);
  cap_u8(&w, accuracy);
  cap_u8(&w, sequence);
  /* Sempre quatro componentes, para que todo registro BNO tenha o mesmo
     formato; giro e aceleração deixam o quarto em zero. Custa 4 bytes e
     elimina um desvio na leitura e uma chance de leitor e escritor
     discordarem. */
  for (int i = 0; i < 4; ++i) cap_f32(&w, v[i]);
  cap_u32(&w, count);
  /* Quando o hub diz que mediu, contra o t_ms do registro, que diz quando
     recebemos. Três relatórios entregues num lote têm a mesma recepção e
     medições diferentes, e essa diferença é a base de qualquer tentativa
     futura de alinhamento temporal. */
  cap_u64(&w, sensor_us);
  cap_u32(&w, delay_us);
  emit(&w);
  xSemaphoreGive(lock_handle);
}

/** @brief Grava um passo do estimador (ver capture.h). */
void capture_step(uint32_t used_mtf_frames, uint32_t used_bno_rotation_count,
                  uint32_t used_bno_gyro_count, uint32_t used_bno_accel_count,
                  float dt_s, bool fused_accepted, bool ekf_accepted,
                  bool imu_online, float raw_px, float raw_py,
                  float fused_vx, float fused_vy, float ekf_vx, float ekf_vy,
                  float ekf_px, float ekf_py) {
  const uint32_t recv_ms = HAL_GetTick();
  if (!queue || !capture_is_active()) return;
  if (!capture_lock()) { drop_contended(); return; }
  uint8_t buf[CAP_MAX_RECORD];
  cap_writer_t w;
  cap_begin(&w, buf, sizeof buf, CAP_TYPE_STEP, next_seq, recv_ms);
  /* Nomear exatamente as amostras consumidas é o objetivo deste registro: o
     replay precisa reproduzir o mesmo pareamento, não adivinhá-lo. */
  cap_u32(&w, used_mtf_frames);
  cap_u32(&w, used_bno_rotation_count);
  cap_u32(&w, used_bno_gyro_count);
  cap_u32(&w, used_bno_accel_count);
  cap_f32(&w, dt_s);
  /* imu_online é gravado, não deduzido: com a IMU offline a predição segue
     com aceleração zero, e o último relatório do BNO continua na amostra
     parecendo perfeitamente plausível. Nenhuma análise dos valores dos
     sensores recuperaria qual dos dois aconteceu. */
  cap_u8(&w, (uint8_t)((fused_accepted ? 1U : 0U) | (ekf_accepted ? 2U : 0U) |
                       (imu_online ? 4U : 0U)));
  cap_f32(&w, raw_px);
  cap_f32(&w, raw_py);
  cap_f32(&w, fused_vx);
  cap_f32(&w, fused_vy);
  cap_f32(&w, ekf_vx);
  cap_f32(&w, ekf_vy);
  cap_f32(&w, ekf_px);
  cap_f32(&w, ekf_py);
  emit(&w);
  xSemaphoreGive(lock_handle);
}

/** @brief Grava um registro de perdas e saúde (ver capture.h). */
void capture_diag(uint32_t clock_resyncs) {
  const uint32_t recv_ms = HAL_GetTick();
  if (!queue || !capture_is_active()) return;
  if (!capture_lock()) { drop_contended(); return; }
  uint8_t buf[CAP_MAX_RECORD];
  cap_writer_t w;
  cap_begin(&w, buf, sizeof buf, CAP_TYPE_DIAG, next_seq, recv_ms);
  /* Separados porque significam coisas diferentes para o leitor: os dois
     primeiros deixaram buraco na sequência, o terceiro não. Juntá-los num só
     número faria a contagem de buracos e a de descartes discordarem por
     motivos que ninguém conseguiria reconstruir depois. */
  cap_u32(&w, dropped_full);
  cap_u32(&w, dropped_oversize);
  cap_u32(&w, dropped_contention);
  cap_u32(&w, clock_resyncs);
  /* Medido no sistema rodando, e não estimado pelo tamanho dos frames: a
     fila sai do heap do FreeRTOS, e os produtores são as tasks de sensor cuja
     margem realmente importa. uxTaskGetStackHighWaterMark retorna words, e
     quem chama é a task que emitiu este registro - o nome identifica o
     produtor; cada task chama capture_diag_poll(). */
  cap_u32(&w, (uint32_t)xPortGetMinimumEverFreeHeapSize());
  cap_u32(&w, (uint32_t)xPortGetFreeHeapSize());
  cap_u32(&w, (uint32_t)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
  cap_str(&w, pcTaskGetName(NULL), 12U);
  emit(&w);
  xSemaphoreGive(lock_handle);
}

/**
 * @brief Enfileira a SESSION: tudo o que um leitor precisa para interpretar o
 *        resto do arquivo sem adivinhar, e para distinguir gravações feitas
 *        com calibrações diferentes.
 */
static void emit_session(void) {
  const uint32_t recv_ms = HAL_GetTick();
  if (!capture_lock()) { drop_contended(); return; }
  uint8_t buf[CAP_MAX_RECORD];
  cap_writer_t w;
  cap_begin(&w, buf, sizeof buf, CAP_TYPE_SESSION, next_seq, recv_ms);
  cap_u8(&w, (uint8_t)CAP_FORMAT_VERSION);
  cap_str(&w, CAP_FIRMWARE_ID, CAP_FIRMWARE_ID_MAX);
  cap_f32(&w, displacement_get_flow_scale_x());
  cap_f32(&w, displacement_get_flow_scale_y());
  cap_f32(&w, displacement_get_gyro_sign_x());
  cap_f32(&w, displacement_get_gyro_sign_y());
  cap_f32(&w, displacement_get_deadband());
  /* Unidades e convenções são fixadas pelo formato, e gravadas para que o
     leitor possa conferir em vez de supor: fluxo e giro em rad/s, distância
     em metros, aceleração em m/s^2, quatérnio w,x,y,z, tempos em ms. */
  cap_f32(&w, DISP_FLOW_LSB_RAD_S);
  cap_u32(&w, DISP_MAX_FRAME_GAP_MS);
  cap_u32(&w, DISP_MAX_CORRECTION_AGE_MS);
  /* Os parâmetros do filtro, sem os quais o replay não reproduz uma única
     decisão de aceitar ou rejeitar - o gate é a questão toda. */
  const flow_ekf_config_t *k = displacement_get_ekf_config();
  cap_f32(&w, k->accel_noise);
  cap_f32(&w, k->flow_noise_rad_s);
  cap_f32(&w, k->gate_sigma);
  emit(&w);
  xSemaphoreGive(lock_handle);
}

/**
 * @brief Task de captura: controla a sessão pelo DTR e envia a fila pela USB.
 *
 * Espera o primeiro DTR do boot, enfileira a SESSION, libera os produtores e
 * então transmite a fila em blocos. Quando o DTR cai, os produtores param e
 * o que já estava na fila continua sendo enviado.
 *
 * @param[in] argument  Não usado.
 */
static void capture_task(void *argument) {
  (void)argument;
  static uint8_t chunk[CAPTURE_CHUNK_BYTES];
  uint32_t last_diag_ms = 0;
  for (;;) {
    const bool same_connection = !session.started || session_close_epoch == CDC_CloseEpoch();
    const capture_session_action_t action = capture_session_update(&session, CDC_HostDTR() && same_connection);
    if (action == CAP_SESSION_START) {
      session_close_epoch = CDC_CloseEpoch();
      /* Os produtores ainda estão desativados; a SESSION é o primeiro
         registro da fila. */
      emit_session();
      session.active = true;
    }
    if (!session.started) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
    capture_diag_poll(&last_diag_ms, CAP_CLOCK_RESYNCS_UNKNOWN);
    /* Espera até haver algo para enviar, então uma captura ociosa não gasta
       CPU. O timeout curto mantém o laço atento à volta da USB. */
    size_t n = xStreamBufferReceive(queue, chunk, sizeof chunk, pdMS_TO_TICKS(20));
    if (!n) continue;
    /* Espera o endpoint em vez de descartar: os bytes já saíram do caminho
       dos produtores, e perdê-los aqui corromperia um registro no meio em vez
       de numa fronteira. */
    for (;;) {
      taskENTER_CRITICAL();
      USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassData;
      const bool ready = (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) && cdc && !cdc->TxState;
      const bool sent = ready && CDC_Transmit_FS(chunk, (uint16_t)n) == USBD_OK;
      taskEXIT_CRITICAL();
      if (sent) break;
      if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED && !CDC_HostDTR()) {
        session.active = false;
        break; /* desconectado fisicamente: esta sessão não tem como continuar */
      }
      vTaskDelay(1);
    }
  }
}

/** @brief Cria a fila, a trava e a task de captura (ver capture.h). */
bool capture_start(void) {
  queue = xStreamBufferCreate(CAPTURE_QUEUE_BYTES, 1);
  if (!queue) return false;
  lock_handle = xSemaphoreCreateMutex();
  if (!lock_handle) return false;
  return xTaskCreate(capture_task, "Capture", CAPTURE_TASK_STACK, NULL,
                     tskIDLE_PRIORITY + 1, NULL) == pdPASS;
}

#else /* !CAPTURE_ENABLED */

/* Build normal: versões vazias, para que os produtores possam chamar a
   captura sem #if em cada ponto de chamada. */

/** @brief Versão vazia; sempre true. */
bool capture_start(void) { return true; }
/** @brief Versão vazia; sempre false. */
bool capture_is_active(void) { return false; }
/** @brief Versão vazia; sempre 0. */
uint32_t capture_dropped(void) { return 0; }
/** @brief Versão vazia; não grava nada. */
void capture_mtf(int16_t flow_x, int16_t flow_y, uint32_t distance_mm,
                 uint8_t quality, uint8_t distance_status, uint8_t strength,
                 uint8_t precision, bool range_valid, bool flow_valid,
                 uint32_t sensor_ms, uint32_t frames) {
  (void)flow_x; (void)flow_y; (void)distance_mm; (void)quality;
  (void)distance_status; (void)strength; (void)precision;
  (void)range_valid; (void)flow_valid; (void)sensor_ms; (void)frames;
}
/** @brief Versão vazia; não grava nada. */
void capture_bno(cap_bno_report_t report, const float v[4], uint8_t accuracy,
                 uint8_t sequence, uint32_t count,
                 uint64_t sensor_us, uint32_t delay_us) {
  (void)report; (void)v; (void)accuracy; (void)sequence; (void)count;
  (void)sensor_us; (void)delay_us;
}
/** @brief Versão vazia; não grava nada. */
void capture_step(uint32_t used_mtf_frames, uint32_t used_bno_rotation_count,
                  uint32_t used_bno_gyro_count, uint32_t used_bno_accel_count,
                  float dt_s, bool fused_accepted, bool ekf_accepted,
                  bool imu_online, float raw_px, float raw_py,
                  float fused_vx, float fused_vy, float ekf_vx, float ekf_vy,
                  float ekf_px, float ekf_py) {
  (void)used_mtf_frames; (void)used_bno_rotation_count; (void)used_bno_gyro_count;
  (void)used_bno_accel_count; (void)dt_s; (void)fused_accepted; (void)ekf_accepted;
  (void)imu_online; (void)raw_px; (void)raw_py;
  (void)fused_vx; (void)fused_vy; (void)ekf_vx; (void)ekf_vy;
  (void)ekf_px; (void)ekf_py;
}
/** @brief Versão vazia; não grava nada. */
void capture_diag(uint32_t clock_resyncs) { (void)clock_resyncs; }
/** @brief Versão vazia; não grava nada. */
void capture_diag_poll(uint32_t *last_ms, uint32_t clock_resyncs) {
  (void)last_ms; (void)clock_resyncs;
}
/** @brief Versão vazia; não grava nada. */
void capture_observation(uint32_t frames, uint32_t previous_sensor_ms,
                         bool have_previous, bool gap_usable,
                         bool range_valid, bool flow_valid) {
  (void)frames; (void)previous_sensor_ms; (void)have_previous;
  (void)gap_usable; (void)range_valid; (void)flow_valid;
}

#endif
