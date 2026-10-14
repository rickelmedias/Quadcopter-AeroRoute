/**
 * @file lqg.h
 * @brief LQG genérico discreto: Filtro de Kalman + realimentação LQR.
 *
 * Código puro (sem HAL nem FreeRTOS), para rodar também nos testes do PC.
 * As dimensões são fixadas em tempo de compilação por LQG_NX, LQG_NU e
 * LQG_NY, e todos os números do modelo vêm de lqg_gains.h, gerado por
 * tools/lqg_design.py (ou colado a partir do MATLAB). Trocar o modelo, os
 * pesos ou o período não exige mexer neste código.
 *
 * Modelo (em torno do equilíbrio, com du = u - u0):
 *
 *   x[k+1] = Ad x[k] + Bd du[k] + w,   cov(w) = Qw
 *   y_i[k] = C_i x[k] + v_i,           var(v_i) = Rv_i
 *   u      = u0 - K (x_hat - x_ref),   saturado em [u_min, u_max]
 *
 * O filtro é o de Kalman variante no tempo, com a correção feita medida por
 * medida (sequencial). Assim cada sensor entra na sua própria taxa: o BNO a
 * cada passo e o MTF-01 só quando chega um frame novo. Com todas as medidas
 * presentes a cada passo, o ganho converge para o mesmo L do Kalman de
 * regime (dlqe/kalman do MATLAB). Os estados não observáveis (x e y, sem
 * medida de posição absoluta) seguem só pela predição; sua variância é
 * limitada por LQG_P_MAX para não crescer sem fim.
 */
#ifndef LQG_H
#define LQG_H
#include <stdbool.h>
#include <stdint.h>
#include "lqg_gains.h"

/** Estado do LQG. */
typedef struct {
  /** Estado estimado, nos desvios do equilíbrio. */
  float x[LQG_NX];
  /** Covariância do erro de estimação. */
  float P[LQG_NX][LQG_NX];
  /** Última entrada aplicada (absoluta), usada na predição seguinte. */
  float u[LQG_NU];
  /** Medidas rejeitadas pelo teste de inovação, por saída. */
  uint32_t rejected[LQG_NY];
} lqg_t;

/**
 * @brief Zera o estado, põe a covariância inicial e a entrada em u0.
 *
 * @param[out] f   LQG.
 * @param[in]  x0  Estado inicial (pode ser NULL para zero).
 */
void lqg_init(lqg_t *f, const float x0[LQG_NX]);

/**
 * @brief Predição de um período: x = Ad x + Bd (u - u0); P = Ad P Ad' + Qw.
 *
 * Usa a entrada guardada pela última chamada de lqg_control().
 */
void lqg_predict(lqg_t *f);

/**
 * @brief Correção com uma única medida (linha @p i de C).
 *
 * Medidas com inovação maior que LQG_GATE_SIGMA[i] desvios padrão (0 = sem
 * teste) são descartadas e
 * contadas em lqg_t::rejected, para que um valor absurdo de sensor não
 * contamine o estado. Para ângulos, passe @p wrap como true: a inovação é
 * normalizada para [-pi, pi).
 *
 * @return true se a medida foi usada.
 */
bool lqg_update(lqg_t *f, unsigned i, float y, bool wrap);

/**
 * @brief Lei de controle: u = u0 - K (x_hat - x_ref), saturada.
 *
 * Guarda u em f->u para a próxima predição.
 *
 * @param[in]  x_ref  Referência, nos mesmos desvios do estado.
 * @param[out] u      Entrada absoluta (por exemplo, rad/s de cada motor).
 */
void lqg_control(lqg_t *f, const float x_ref[LQG_NX], float u[LQG_NU]);
#endif
