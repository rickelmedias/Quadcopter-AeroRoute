/**
 * @file control_task.h
 * @brief Task do controle LQR + Kalman a 100 Hz (modo de teste com cabo).
 *
 * Liga os sensores (BNO085, MTF-01) ao controle puro (control/lqg_ctrl.h) e o
 * controle aos motores (dono ::MOTORS_OWNER_CONTROL). A task é a única dona do
 * estado do controle; as funções abaixo só deixam pedidos que ela atende no
 * próximo passo de 10 ms, e podem ser chamadas de qualquer task.
 *
 * Quem decide a segurança não é o PC: o árbitro de motores desarma sem
 * atualização do controle por 100 ms e sem HEARTBEAT da IHM por 300 ms, e o
 * controle corta os motores por inclinação, perda de sensor ou falha de
 * decolagem (control/lqg_ctrl.h). STOP (botão, espaço, DTR fechado) para tudo.
 */
#ifndef APP_CONTROL_TASK_H
#define APP_CONTROL_TASK_H

#include "control/lqg_ctrl.h"

#include <stdbool.h>
#include <stdint.h>

#ifndef FLT_SIM
/**
 * 1 = planta simulada (control/lqg_sim.h) no lugar do BNO e do MTF; os motores
 * reais recebem zero mesmo armados. Para testar sequência, DroneLink e Pi na
 * placa sem hélice girando (tools/build.py --sim).
 */
#define FLT_SIM 0
#endif

/** @brief Fotografia do controle depois de um passo (comandos e telemetria da Pi). */
typedef struct {
  lqg_state_t state;
  /** Saída armada para o controle. */
  bool armed;
  /** No ar (descolou e ainda não tocou). */
  bool airborne;
  /** Planta simulada (FLT_SIM). */
  bool simulated;
  /** Bits LQG_TLM_* do último passo. */
  uint8_t flags;
  /** Último motivo de parada (LQG_FAIL_*). */
  uint8_t fail;
  /** Estimativa, referência e alvo. */
  lqg_nav_t nav;
  /** Atitude medida φ, θ (rad) e taxas p, q (rad/s). */
  float phi, theta, p, q;
  float vbat_factor;
  /** Passos do controle desde o boot e o tick do último. */
  uint32_t step;
  uint32_t t_ms;
  /** Medidas y = [z φ θ ψ u v p q r], estimativa x̂[12] e motores M1..M4 (‰). */
  float y[9];
  float x_hat[12];
  float permille[4];
  /** Aceleração base antes do Δω, em ‰. */
  float base;
  /** Último aviso do controle (texto fixo) e quantos avisos já houve. */
  const char *note;
  uint32_t note_seq;
  /** O último aviso foi um corte dos motores (não o fim de um pouso). */
  bool note_critical;
} control_snapshot_t;

/** @brief Cria a task do controle. Chamar depois dos sensores e dos motores. */
bool control_task_start(void);

/**
 * @brief Arma os motores para o controle (motores em zero, mede o chão).
 *
 * Exige BNO e distância válidos agora e a saída livre. As checagens de
 * HEARTBEAT e de tensão são do enlace (link/bench_link).
 *
 * @return false se algum sensor está inválido ou a saída está ocupada.
 */
bool control_arm(void);

/** @brief Pede a decolagem. false se não está armado, sem chão medido ou sensor inválido. */
bool control_takeoff(void);

/**
 * @brief Pede a decolagem até @p height_m (m acima do chão do armamento), sem mudar
 *        o parâmetro de altura da IHM. Mesmas condições de ::control_takeoff.
 */
bool control_takeoff_to(float height_m);

/** @brief Pede o pouso. false se não está no ar nem subindo a aceleração. */
bool control_land(void);

/**
 * @brief Pede um novo alvo em voo (ver ::lqg_ctrl_goto): x, y local (m), altura (m,
 *        para cima), guinada (rad), velocidade XY (m/s, 0 = padrão).
 * @return false fora de subida/hover ou com valores fora dos limites.
 */
bool control_goto(float x, float y, float height_m, float psi, float speed);

/** @brief Em voo: segura onde a referência está agora. */
bool control_hold(void);

/** @brief Cópia da fotografia do último passo. */
void control_get_snapshot(control_snapshot_t *out);

/**
 * @brief Troca os parâmetros editáveis (valem a partir do próximo passo).
 * @return false se algum valor está fora dos limites.
 */
bool control_set_params(const lqg_params_t *params);

/**
 * @brief Troca os multiplicadores dos ganhos do LQR (valem no próximo passo, inclusive em voo).
 * @return false se algum valor está fora dos limites de ::lqg_gains_valid.
 */
bool control_set_gains(const lqg_gains_t *gains);

#endif
