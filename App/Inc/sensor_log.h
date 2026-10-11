/**
 * @file sensor_log.h
 * @brief Log de texto pela USB (CDC) e chaves de compilação dos perfis.
 *
 * Perfil de bancada MTF: o firmware só inicializa o MTF-01 e imprime uma
 * linha CSV com a última leitura bruta a cada 100 ms. Não há BNO, deslocamento,
 * filtros ou estimadores de posição neste perfil.
 *
 * Com CAPTURE_BUILD definido (tools/build.py --capture), todos os grupos são
 * desligados e a captura binária é ligada, sem editar este arquivo.
 */
#ifndef SENSOR_LOG_H
#define SENSOR_LOG_H
#include <stdbool.h>

#if defined(CAPTURE_BUILD)
#define LOG_BNO_STATUS 0
#define LOG_BNO_DATA 0
#define LOG_MTF 0
#define LOG_ATTITUDE 0
#define LOG_HEIGHT 0
#define LOG_POSITION 0
#define LOG_POSITION_D 0
#define LOG_CALIBRATION 0
#define LOG_SYS 0
#else
/** Estado do BNO (desligado neste perfil). */
#define LOG_BNO_STATUS  0
/** Dados do BNO (desligado neste perfil). */
#define LOG_BNO_DATA    0
/** Dados brutos do MTF-01, em uma linha CSV sem cabeçalho. */
#define LOG_MTF         1
/** Ângulos de Euler (desligado neste perfil). */
#define LOG_ATTITUDE    0
/** Altura corrigida (desligado neste perfil). */
#define LOG_HEIGHT      0
/** Deslocamento EKF (desligado neste perfil). */
#define LOG_POSITION    0
/** Incremento por frame, velocidades e saúde do filtro. */
#define LOG_POSITION_D  0
/** Progresso ao vivo da calibração; ver DISP_CALIBRATE_FLOW abaixo. */
#define LOG_CALIBRATION 0
/** Heap livre, descartes do enlace e linhas de log descartadas. */
#define LOG_SYS         0
#endif

/**
 * @brief Mantido em zero: este perfil não cria estimadores de deslocamento.
 */
#define DISP_CALIBRATE_FLOW 0

/**
 * @brief Captura binária não é usada neste perfil de log textual MTF.
 */
#if defined(CAPTURE_BUILD)
#define CAPTURE_ENABLED 1
#else
#define CAPTURE_ENABLED 0
#endif

#if CAPTURE_ENABLED && (LOG_BNO_STATUS || LOG_BNO_DATA || LOG_MTF || LOG_ATTITUDE || LOG_HEIGHT || LOG_POSITION || LOG_POSITION_D || LOG_CALIBRATION || LOG_SYS)
#error "Binary capture and text logs cannot share CDC; disable LOG_* first"
#endif

/**
 * @brief Cria a task de log de texto.
 *
 * Mesmo com todos os grupos desligados a task é criada; ela só não emite
 * nada. No build Capture a task não é criada.
 *
 * @return true se a task foi criada (sempre true no build Capture).
 */
bool sensor_log_start(void);
#endif
