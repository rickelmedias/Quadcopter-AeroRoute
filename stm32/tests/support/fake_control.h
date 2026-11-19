/**
 * @file fake_control.h
 * @brief Controle síncrono para os testes do DroneLink (ver fake_control.c).
 */
#ifndef TEST_FAKE_CONTROL_H
#define TEST_FAKE_CONTROL_H

#include "app/control_task.h"
#include "control/lqg_sim.h"
#include "motors.h"
#include "power.h"

extern lqg_ctrl_t ctrl;
extern lqg_sim_t sim;
extern uint32_t now_ms;
extern motors_owner_t owner;
extern uint32_t keepalives;
extern control_snapshot_t snap;
/** fake_step() também chama flight_command_tick() (o teste do protocolo deixa para dronelink_protocol_tick()). */
extern bool fake_runs_executor;

/** @brief Um passo de 10 ms: controle, planta e (se ligado) flight_command_tick(). */
void fake_step(void);

#endif
