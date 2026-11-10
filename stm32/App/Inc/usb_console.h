/**
 * @file usb_console.h
 * @brief Console de texto pela USB: teste de bancada dos motores.
 *
 * Uma linha por comando, terminada em Enter. As respostas saem no log
 * (linhas que começam com "# "). Abra a porta com DTR ligado
 * (tools/motores_gui.py faz isso): sem DTR, nada arma.
 *
 * SEM HÉLICES:
 *
 * @verbatim
 *   arm                      arma a bancada (todos os motores em 0)
 *   m <motor> <pct>          alvo de um motor (1..4) ou de todos (0), 0..100
 *   d <motor> <dshot>        alvo em valor DShot cru: 0 (parado) ou 48..2047
 *   sweep <motor> [max] [passo] [seg]
 *   rate <pct_por_s>         rampa da bancada (padrão 20 %/s)
 *   beep <motor>             beep do ESC (desarmado)
 *   dir <motor> n|r          sentido de giro do ESC, normal ou invertido
 *                            (desarmado; grava na memória do ESC)
 *   wd <ms>                  watchdog: para os motores se nenhuma linha chegar
 *                            em <ms> (200..5000); 0 desliga
 *   ping                     não faz nada; só mantém o watchdog vivo
 *   log on | log off         liga/silencia as linhas MOT
 *   status | help
 * @endverbatim
 *
 * stop, s ou Enter vazio param tudo na hora. Qualquer linha não reconhecida
 * também para (um erro de digitação nunca deixa motor girando). Fechar a
 * porta no PC corta os motores. A saída MOT (100 Hz) tem o formato
 * `MOT;t_ms;armado;pct1..pct4 (décimos);dshot1..dshot4`.
 */
#ifndef USB_CONSOLE_H
#define USB_CONSOLE_H

#include <stdbool.h>

/**
 * @brief Cria a task do console. Chamar depois de motors_start().
 * @return true se a task foi criada.
 */
bool usb_console_start(void);

#endif
