/**
 * @file sensor_log.h
 * @brief Log de texto pela USB (CDC) e chaves de compilação dos perfis.
 *
 * Cada grupo de log imprime uma linha a cada 500 ms. Troque 1 por 0 para
 * silenciar um grupo e 0 por 1 para trazê-lo de volta; um grupo desligado é
 * removido da compilação e não custa nada. A ordem abaixo é a ordem em que as
 * linhas aparecem no log.
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
/** Estado do BNO: online, falha, endereço, firmware, probe_ms, resets. */
#define LOG_BNO_STATUS  0
/** Dados do BNO: quatérnio, giro, aceleração, contadores e lacunas por relatório. */
#define LOG_BNO_DATA    0
/** MTF-01: fluxo bruto, distância e contadores de frame/CRC/UART. */
#define LOG_MTF         0
/** Ângulos de Euler e quatérnio; o yaw alimenta o caminho FUSED. */
#define LOG_ATTITUDE    1
/** Distância no eixo do sensor e altura corrigida pela inclinação. */
#define LOG_HEIGHT      1
/** Deslocamento acumulado dos três estimadores. */
#define LOG_POSITION    1
/** Incremento por frame, velocidades e saúde do filtro. */
#define LOG_POSITION_D  1
/** Progresso ao vivo da calibração; ver DISP_CALIBRATE_FLOW abaixo. */
#define LOG_CALIBRATION 0
/** Heap livre, descartes do enlace e linhas de log descartadas. */
#define LOG_SYS         0
#endif

/**
 * @brief Liga a calibração do fluxo desde o boot (1) em vez dos estimadores
 *        normais.
 *
 * Ver flow_calibrator.h e displacement_calibrate_start(). Ligue
 * LOG_CALIBRATION junto. Balance o veículo em rolagem e arfagem, sem
 * transladar nem girar no yaw, e acompanhe a linha CAL no log até convergir.
 * Anote os dois scalar_x1000 em que ela estabilizar, coloque-os como padrão
 * em displacement.c e volte esta chave para 0: o resultado não persiste
 * sozinho entre reinicializações.
 */
#define DISP_CALIBRATE_FLOW 0

/**
 * @brief Liga (1) a captura binária evento a evento pela USB, para replay.
 *
 * Ver capture.h. Não pode ser usada junto com o log de texto: os dois se
 * misturariam na mesma porta e nenhum seria legível, por isso a combinação é
 * recusada na compilação. Com 0, a fila e a task da captura nem são
 * compiladas.
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
