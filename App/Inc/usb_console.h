/**
 * @file usb_console.h
 * @brief Console de texto pela USB: bancada dos motores, voo e parâmetros.
 *
 * Uma linha por comando, terminada em Enter. As respostas saem no log
 * (linhas que começam com "# "). Abra a porta com DTR ligado
 * (tools/motor_console.ps1 faz isso): sem DTR, nada arma.
 *
 * Bancada (SEM HÉLICES):
 *
 * @verbatim
 *   arm                      arma a bancada (todos os motores em 0)
 *   m <motor> <pct>          alvo de um motor (1..4) ou de todos (0), 0..100
 *   sweep <motor> [max] [passo] [seg]
 *   rate <pct_por_s>         rampa da bancada (padrão 20 %/s)
 *   beep <motor>             beep do ESC (desarmado)
 *   log on | log off         liga/silencia as linhas MTF/BNO/MOT/EST
 *   status | help
 * @endverbatim
 *
 * Voo (o mesmo controle usado pela Raspberry Pi):
 *
 * @verbatim
 *   hover [altura_m]         arma, decola e segura (FLT_HOVER_H se omitida)
 *   dry [altura_m]           igual, mas força FLT_DRY_RUN = 1 (motores parados)
 *   land                     pousa e desarma
 *   hold                     para onde está
 *   goto <x> <y> <z>         vai ao ponto local, em m (z negativo = acima do chão)
 *   move <dx> <dy> [dz]      translada no referencial do corpo, em m
 *   yaw <graus>              gira em relação à guinada atual
 *   lqg                      estado do voo, estimativa e contadores
 * @endverbatim
 *
 * Ajuste:
 *
 * @verbatim
 *   param list               lista todos os parâmetros
 *   param get <NOME>         mostra um parâmetro
 *   param set <NOME> <valor> altera um parâmetro (control/ctrl_params.h)
 *   gains                    mostra se os ganhos são os do Simulink
 *   gains default            volta aos ganhos do Simulink
 * @endverbatim
 *
 * stop, disarm, s ou Enter vazio param tudo na hora. Qualquer linha não
 * reconhecida também para (um erro de digitação nunca deixa motor girando).
 * Fechar a porta no PC corta motores armados pela USB.
 */
#ifndef USB_CONSOLE_H
#define USB_CONSOLE_H

#include <stdbool.h>

/**
 * @brief Cria a task do console. Chamar depois de motors_start() e
 *        flight_ctrl_start().
 * @return true se a task foi criada.
 */
bool usb_console_start(void);

#endif
