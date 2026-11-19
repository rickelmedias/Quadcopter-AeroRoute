/** @file startup.h @brief Composição de serviços por perfil da aplicação. */
#ifndef APP_STARTUP_H
#define APP_STARTUP_H
#include <stdbool.h>
bool app_services_start(void);

/**
 * @brief Por que o STM32 reiniciou da última vez (texto fixo), lido dos flags do RCC
 *        no início de app_services_start(): ligou, queda de tensão (brown-out),
 *        botão RESET/pino NRST, reinício por software, watchdog ou baixo consumo.
 *        Reinícios sozinhos com a bateria do ESC ligada deixam os fios do DShot
 *        soltos por um instante; este texto vai para a IHM e para a Pi.
 */
const char *app_reset_cause(void);
#endif
