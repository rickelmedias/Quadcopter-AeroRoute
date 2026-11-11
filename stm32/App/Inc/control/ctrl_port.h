/**
 * @file ctrl_port.h
 * @brief Seção crítica usada pelos módulos puros do controle.
 *
 * No firmware é a do FreeRTOS. Nos testes do PC (compilados com
 * `CTRL_HOST_BUILD`) não há outras tasks, então ela não faz nada. Assim os
 * módulos de control/ continuam testáveis sem stubs do RTOS.
 */
#ifndef CONTROL_CTRL_PORT_H
#define CONTROL_CTRL_PORT_H

#if defined(CTRL_HOST_BUILD)
/** Entra na seção crítica (nada no PC). */
#define CTRL_ENTER_CRITICAL() ((void)0)
/** Sai da seção crítica (nada no PC). */
#define CTRL_EXIT_CRITICAL()  ((void)0)
#else
#include "FreeRTOS.h"
#include "task.h"
/** Entra na seção crítica do FreeRTOS (sem troca de task nem interrupções). */
#define CTRL_ENTER_CRITICAL() taskENTER_CRITICAL()
/** Sai da seção crítica do FreeRTOS. */
#define CTRL_EXIT_CRITICAL()  taskEXIT_CRITICAL()
#endif

#endif
