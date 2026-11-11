/**
 * @file lqg_ert.h
 * @brief Filtro de Kalman + LQR do TCC, portado do código gerado pelo Simulink.
 *
 * Este módulo é o `STM32_step()` de `reference/STM32_ert_rtw/STM32.c`
 * (modelo Simulink "STM32", versão 2.17) reescrito de forma legível e sem
 * variáveis globais, mantendo **as mesmas operações na mesma ordem**. Com
 * ::lqg_real_t em `double` (o padrão, igual ao `real_T` do Simulink), o
 * resultado é idêntico bit a bit ao do código gerado; `tests/test_lqg_ert.c`
 * confere isso rodando os dois lado a lado.
 *
 * O bloco "Kalman Filter" do modelo é o de regime permanente (ganhos fixos L
 * e M) com a opção "Use current estimator". A cada período Ts = 10 ms:
 *
 * @code
 *   e        = y - C x̂[k|k-1]                  inovação (D = 0)
 *   x̂[k|k]   = x̂[k|k-1] + M e                  estimativa corrente
 *   du       = Kfd r - Kd x̂[k|k]               LQR com pré-compensador
 *   x̂[k+1|k] = A x̂[k|k-1] + B du + L e         predição para o próximo passo
 * @endcode
 *
 * - x (12): desvios do hover em NED: x, y, z, φ, θ, ψ, vx, vy, vz, p, q, r.
 * - y (9): medidas z, φ, θ, ψ, vx, vy, p, q, r (ordem do modelo, ::lqg_y_t).
 * - r (4): referência x, y, z, ψ (::lqg_ref_t).
 * - du (4): desvio da rotação de cada motor em relação ao hover, em rad/s.
 *   A rotação comandada é ω0 + du (ver control/ctrl_params.h).
 *
 * O cálculo foi dividido em duas chamadas, ::lqg_ert_output() e
 * ::lqg_ert_update(), só para que o voo possa informar a entrada que de fato
 * foi aplicada aos motores (por exemplo, zero no ensaio a seco). Chamar as
 * duas em seguida com o mesmo du é exatamente o `STM32_step()`, e é o que
 * ::lqg_ert_step() faz.
 *
 * Código puro: sem HAL e sem FreeRTOS, para rodar também nos testes do PC.
 */
#ifndef CONTROL_LQG_ERT_H
#define CONTROL_LQG_ERT_H

#include <stdbool.h>

/** Número de estados. */
#define LQG_NX    12U
/** Número de entradas (motores). */
#define LQG_NU    4U
/** Número de medidas. */
#define LQG_NY    9U
/** Número de referências. */
#define LQG_NR    4U
/** Período de amostragem do modelo, em s (taxa base do Simulink: 100 Hz). */
#define LQG_TS_S  0.01
/** Período de amostragem do modelo, em ms. */
#define LQG_TS_MS 10U

/**
 * @brief Tipo numérico do LQG.
 *
 * `double` por padrão, como o `real_T` do código gerado, para reproduzir o
 * MATLAB bit a bit. No Cortex-M4 o `double` é emulado em software: um passo
 * custa cerca de 0,5 ms a 96 MHz (5% da CPU a 100 Hz). Defina
 * `LQG_ERT_SINGLE_PRECISION` para usar a FPU em `float` (~20 us por passo);
 * o teste de equivalência mede a diferença: ~2e-4 relativo em malha fechada.
 */
#if defined(LQG_ERT_SINGLE_PRECISION)
typedef float lqg_real_t;
#else
typedef double lqg_real_t;
#endif

/** @brief Índices do vetor de estados (eixos NED, desvios do hover). */
typedef enum {
  LQG_X_X = 0, /**< Posição x (frente), em m. */
  LQG_X_Y,     /**< Posição y (direita), em m. */
  LQG_X_Z,     /**< Posição z (para baixo), em m. Altura h = -z. */
  LQG_X_PHI,   /**< Rolagem φ, em rad. */
  LQG_X_THETA, /**< Arfagem θ, em rad. */
  LQG_X_PSI,   /**< Guinada ψ, em rad. */
  LQG_X_VX,    /**< Velocidade vx, em m/s. */
  LQG_X_VY,    /**< Velocidade vy, em m/s. */
  LQG_X_VZ,    /**< Velocidade vz, em m/s. */
  LQG_X_P,     /**< Velocidade angular p, em rad/s. */
  LQG_X_Q,     /**< Velocidade angular q, em rad/s. */
  LQG_X_R      /**< Velocidade angular r, em rad/s. */
} lqg_x_t;

/** @brief Índices do vetor de medidas, na ordem da matriz C do modelo. */
typedef enum {
  LQG_Y_Z = 0, /**< Altura medida pelo MTF-01, como z NED (m). */
  LQG_Y_PHI,   /**< Rolagem do BNO08x (rad). */
  LQG_Y_THETA, /**< Arfagem do BNO08x (rad). */
  LQG_Y_PSI,   /**< Guinada do BNO08x (rad), relativa ao início. */
  LQG_Y_VX,    /**< Velocidade x do fluxo óptico (m/s). */
  LQG_Y_VY,    /**< Velocidade y do fluxo óptico (m/s). */
  LQG_Y_P,     /**< Giroscópio p (rad/s). */
  LQG_Y_Q,     /**< Giroscópio q (rad/s). */
  LQG_Y_R      /**< Giroscópio r (rad/s). */
} lqg_y_t;

/** @brief Índices do vetor de referência (entrada In1 do modelo). */
typedef enum {
  LQG_R_X = 0, /**< Posição x desejada (m). */
  LQG_R_Y,     /**< Posição y desejada (m). */
  LQG_R_Z,     /**< Posição z desejada (m, NED: negativa acima do chão). */
  LQG_R_PSI    /**< Guinada desejada (rad). */
} lqg_ref_t;

/**
 * @brief Ganhos que podem ser trocados em tempo de execução.
 *
 * A, B e C descrevem o drone e ficam fixos (control/lqg_ert_model.h). Os
 * ganhos dependem dos pesos escolhidos no projeto (Q e R do LQR, Qn e Rn do
 * Kalman) e podem ser recalculados na Raspberry Pi e enviados em voo
 * (control/ctrl_gains.h).
 */
typedef struct {
  /** Ganho do LQR, u = -Kd x̂ (bloco Gain5). */
  lqg_real_t Kd[LQG_NU][LQG_NX];
  /** Pré-compensador da referência (bloco Gain7), ganho DC unitário. */
  lqg_real_t Kfd[LQG_NU][LQG_NR];
  /** Ganho de predição do Kalman, L = A M. */
  lqg_real_t L[LQG_NX][LQG_NY];
  /** Ganho de correção do Kalman (estimador corrente). */
  lqg_real_t M[LQG_NX][LQG_NY];
} lqg_gains_t;

/**
 * @brief Estado do filtro (equivale a `DW_STM32_T` do código gerado).
 */
typedef struct {
  /** x̂[k|k-1]: o atraso "MemoryX" do Simulink. */
  lqg_real_t x_pred[LQG_NX];
  /** x̂[k|k]: estimativa corrente usada pelo LQR. Só para leitura. */
  lqg_real_t x_filt[LQG_NX];
  /** Inovação y - C x̂[k|k-1] do último passo. Só para leitura. */
  lqg_real_t innovation[LQG_NY];
  /** Verdadeiro até o primeiro passo: MemoryX começa em zero ("icLoad"). */
  bool ic_load;
} lqg_ert_t;

/**
 * @brief Inicializa o filtro (equivale a `STM32_initialize()`).
 *
 * A estimativa é zerada no primeiro passo, como o `icLoad` do Simulink.
 *
 * @param[out] s  Filtro.
 */
void lqg_ert_init(lqg_ert_t *s);

/**
 * @brief Correção do Kalman e lei de controle (primeira metade do passo).
 *
 * Calcula a inovação, x̂[k|k] e du = Kfd r - Kd x̂[k|k]. Não altera a
 * predição; chame ::lqg_ert_update() em seguida.
 *
 * @param[in,out] s   Filtro.
 * @param[in]     g   Ganhos em uso.
 * @param[in]     y   Medidas (::lqg_y_t).
 * @param[in]     r   Referência (::lqg_ref_t).
 * @param[out]    du  Desvio da rotação de cada motor, em rad/s.
 */
void lqg_ert_output(lqg_ert_t *s, const lqg_gains_t *g, const lqg_real_t y[LQG_NY],
                    const lqg_real_t r[LQG_NR], lqg_real_t du[LQG_NU]);

/**
 * @brief Predição do Kalman (segunda metade do passo).
 *
 * x̂[k+1|k] = A x̂[k|k-1] + B du + L e, usando a inovação calculada pela
 * última ::lqg_ert_output().
 *
 * @param[in,out] s           Filtro.
 * @param[in]     g           Ganhos em uso (os mesmos da saída).
 * @param[in]     du_applied  Entrada que os motores de fato receberam. Em voo
 *                            é o próprio du (como no Simulink); no ensaio a
 *                            seco, com os motores parados, é zero.
 */
void lqg_ert_update(lqg_ert_t *s, const lqg_gains_t *g, const lqg_real_t du_applied[LQG_NU]);

/**
 * @brief Um passo completo, idêntico a `STM32_step()`.
 *
 * @param[in,out] s   Filtro.
 * @param[in]     g   Ganhos.
 * @param[in]     y   Medidas.
 * @param[in]     r   Referência.
 * @param[out]    du  Saída Out1 do modelo.
 */
void lqg_ert_step(lqg_ert_t *s, const lqg_gains_t *g, const lqg_real_t y[LQG_NY],
                  const lqg_real_t r[LQG_NR], lqg_real_t du[LQG_NU]);

#endif
