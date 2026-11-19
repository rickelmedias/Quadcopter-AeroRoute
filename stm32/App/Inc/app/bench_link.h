/**
 * @file bench_link.h
 * @brief Sessão USB da bancada: comandos bench-link v1 e telemetria periódica.
 *
 * Regras de segurança (o PC só pede; quem decide é este módulo e o árbitro):
 *
 * - Sessão nova a cada DTR ativo; fechar a porta ou trocar de sessão para os
 *   motores e descarta bytes e quadros pendentes da sessão anterior.
 * - ARM exige HEARTBEAT nos últimos 300 ms, recepção sem overflow e tensão
 *   acima do mínimo quando a IHM configurou um mínimo (power_cal.h).
 * - Só HEARTBEAT renova o prazo de 300 ms do árbitro; DTR ou outros comandos
 *   não renovam.
 * - Quadro com CRC/COBS inválido, comando desconhecido ou fora de faixa para
 *   a saída imediatamente (pode ser um STOP corrompido).
 * - Overflow da recepção para os motores e bloqueia comandos até reconectar.
 *
 * Periodicamente envia STATUS (10 Hz), POWER (10 Hz), SYS (1 Hz) e cada
 * quadro novo do ESC. Formato: link/bench_proto.h.
 */
#ifndef APP_BENCH_LINK_H
#define APP_BENCH_LINK_H

#include <stdbool.h>

/** @brief Cria o buffer de recepção e a task do enlace. */
bool bench_link_start(void);

#endif
