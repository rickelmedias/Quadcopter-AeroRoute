/**
 * @file flow_ekf.c
 * @brief Implementação do filtro de Kalman de fluxo (contratos em flow_ekf.h).
 */
#include "flow_ekf.h"
#include <math.h>

const flow_ekf_config_t FLOW_EKF_DEFAULTS = {
  .accel_noise = 0.5F,
  .flow_noise_rad_s = 0.25F, /* padrão FLOW_M_NSE do ArduPilot */
  .gate_sigma = 3.0F,        /* padrão FLOW_I_GATE do ArduPilot (300) */
};

/** Variância de velocidade abaixo da qual o filtro é considerado convergido;
    cerca de 0,3 m/s de incerteza, a partir daí a estimativa vale a leitura. */
#define CONVERGED_VEL_VAR 0.09F

/** Variância inicial da posição. A posição começa perfeitamente conhecida
    porque é a origem por definição: é navegação estimada, então o
    referencial fica ancorado onde se começou. */
#define INITIAL_POS_VAR 0.0F
/** Variância inicial da velocidade. A velocidade começa desconhecida, e são
    as primeiras medidas de fluxo que a determinam. */
#define INITIAL_VEL_VAR 1.0F

/** Rejeições seguidas até o filtro adotar a medida (0,5 s a 100 Hz).
    Um gate que rejeita tudo é pior que nenhum gate: o filtro defenderia um
    estado que o sensor contradiz há meio segundo. Passado esse tanto, quem
    está errada é a predição, então a medida é adotada e a incerteza alargada
    para combinar. O ArduPilot faz o equivalente ao reiniciar os estados de
    velocidade quando a fusão do fluxo expira. */
#define MAX_REJECT_RUN 50

/** @brief Estado inicial (ver flow_ekf.h). */
void flow_ekf_reset(flow_ekf_t *f) {
  for (int i = 0; i < 2; ++i) {
    f->axis[i].p = 0.0F;
    f->axis[i].v = 0.0F;
    f->axis[i].P00 = INITIAL_POS_VAR;
    f->axis[i].P01 = 0.0F;
    f->axis[i].P11 = INITIAL_VEL_VAR;
    f->innov[i] = 0.0F;
    f->test_ratio[i] = 0.0F;
  }
  f->updates = 0;
  f->rejected = 0;
  f->resets = 0;
  f->reject_run = 0;
  f->converged = false;
  f->initialized = true;
}

/**
 * @brief Inicializa o filtro se ele ainda estiver zerado.
 *
 * O armazenamento estático entrega esta struct toda zerada, e variância de
 * velocidade zero não é um ponto de partida neutro: zera o ganho de Kalman e
 * ao mesmo tempo satisfaz o teste de convergência. Em vez de depender de todo
 * chamador lembrar de reiniciar antes, as funções de entrada corrigem isso.
 *
 * @param[in,out] f  Filtro.
 */
static void ensure_initialized(flow_ekf_t *f) {
  if (!f->initialized) flow_ekf_reset(f);
}

/**
 * @brief Predição de um eixo: x = F*x + B*a e P = F*P*F' + Q, com
 *        F = [1 dt; 0 1].
 *
 * @param[in,out] a          Estado do eixo.
 * @param[in]     accel      Aceleração no eixo, em m/s^2.
 * @param[in]     dt         Intervalo, em s.
 * @param[in]     accel_var  Variância da aceleração (accel_noise ao quadrado).
 */
static void predict_axis(flow_ekf_axis_t *a, float accel, float dt, float accel_var) {
  a->p += a->v*dt + 0.5F*accel*dt*dt;
  a->v += accel*dt;

  /* F*P*F' expandido para o caso 2x2. */
  const float P00 = a->P00 + dt*(2.0F*a->P01 + dt*a->P11);
  const float P01 = a->P01 + dt*a->P11;
  const float P11 = a->P11;

  /* Q para uma perturbação de aceleração constante durante dt. */
  const float dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt2*dt2;
  a->P00 = P00 + accel_var*dt4*0.25F;
  a->P01 = P01 + accel_var*dt3*0.5F;
  a->P11 = P11 + accel_var*dt2;
}

/**
 * @brief Recalcula @c converged a partir das variâncias de velocidade dos dois
 *        eixos.
 *
 * @param[in,out] f  Filtro.
 */
static void update_converged(flow_ekf_t *f) {
  f->converged = (f->axis[0].P11 < CONVERGED_VEL_VAR) &&
                 (f->axis[1].P11 < CONVERGED_VEL_VAR);
}

/** @brief Etapa de predição (ver flow_ekf.h). */
void flow_ekf_predict(flow_ekf_t *f, float ax, float ay, float dt,
                      const flow_ekf_config_t *cfg) {
  ensure_initialized(f);
  if (!(dt > 0.0F) || !isfinite(dt)) return;
  if (!isfinite(ax)) ax = 0.0F;
  if (!isfinite(ay)) ay = 0.0F;
  const float accel_var = cfg->accel_noise * cfg->accel_noise;
  predict_axis(&f->axis[0], ax, dt, accel_var);
  predict_axis(&f->axis[1], ay, dt, accel_var);
  /* A predição aumenta a covariância, então pode tirar o filtro da
     convergência tanto quanto uma correção o leva até ela. Deixar isso só
     para a correção fazia um filtro sem medidas continuar dizendo que
     convergiu enquanto a própria incerteza passava do limite. */
  update_converged(f);
}

/**
 * @brief Calcula a razão de teste da inovação de um eixo.
 *
 * A medida é a própria velocidade, então H = [0 1] e a correção vira
 * escalar. Quem chama decide o gate com base na razão.
 *
 * @param[in]  a           Estado do eixo.
 * @param[in]  z           Velocidade medida, em m/s.
 * @param[in]  R           Variância da medida.
 * @param[in]  gate_sigma  Gate em desvios padrão (<= 0 vira 1).
 * @param[out] innov_out   Inovação z - v.
 * @return Inovação normalizada; acima de 1 a medida está fora do gate.
 *         INFINITY se a variância da inovação não for positiva.
 */
static float axis_test_ratio(const flow_ekf_axis_t *a, float z, float R,
                             float gate_sigma, float *innov_out) {
  const float innov = z - a->v;
  const float S = a->P11 + R;
  *innov_out = innov;
  if (!(S > 0.0F)) return INFINITY;
  const float gate = (gate_sigma > 0.0F) ? gate_sigma : 1.0F;
  return (innov*innov) / (gate*gate*S);
}

/**
 * @brief Aplica a correção de Kalman num eixo.
 *
 * @param[in,out] a      Estado do eixo.
 * @param[in]     innov  Inovação já calculada.
 * @param[in]     R      Variância da medida.
 */
static void fuse_axis(flow_ekf_axis_t *a, float innov, float R) {
  const float S = a->P11 + R;
  const float K0 = a->P01 / S;
  const float K1 = a->P11 / S;

  a->p += K0*innov;
  a->v += K1*innov;

  /* P = (I - K*H)*P. Escrito por extenso, P se mantém simétrica por
     construção: as duas expressões fora da diagonal se reduzem a
     P01*(1 - P11/S). */
  const float P00 = a->P00 - K0*a->P01;
  const float P01 = a->P01 - K0*a->P11;
  const float P11 = a->P11 - K1*a->P11;
  a->P00 = (P00 > 0.0F) ? P00 : 0.0F;
  a->P01 = P01;
  a->P11 = (P11 > 0.0F) ? P11 : 0.0F;
}

/** @brief Etapa de correção (ver flow_ekf.h). */
bool flow_ekf_update(flow_ekf_t *f, float vx, float vy, float range_m,
                     const flow_ekf_config_t *cfg) {
  ensure_initialized(f);
  if (!isfinite(vx) || !isfinite(vy) || !isfinite(range_m) || range_m <= 0.0F) {
    ++f->rejected;
    return false;
  }
  /* O mesmo erro angular é um erro de velocidade maior quanto mais alto se
     está, então o ruído da medida precisa crescer com a distância. */
  const float sigma = cfg->flow_noise_rad_s * range_m;
  const float R = sigma*sigma;

  const float z[2] = {vx, vy};
  float ratio[2];
  for (int i = 0; i < 2; ++i) {
    ratio[i] = axis_test_ratio(&f->axis[i], z[i], R, cfg->gate_sigma, &f->innov[i]);
    f->test_ratio[i] = ratio[i];
  }

  /* Gate no par, não por eixo: a medida é uma observação de um único vetor
     de velocidade, e fundir metade dela puxaria a estimativa para o eixo que
     por acaso passou. */
  if (ratio[0] > 1.0F || ratio[1] > 1.0F) {
    ++f->rejected;
    if (++f->reject_run < MAX_REJECT_RUN) return false;
    /* Discordância persistente: acreditar no sensor, não em nós mesmos. */
    for (int i = 0; i < 2; ++i) {
      f->axis[i].v = z[i];
      f->axis[i].P11 = R;
      f->axis[i].P01 = 0.0F;
    }
    f->reject_run = 0;
    f->converged = false;
    ++f->resets;
    return true;
  }
  f->reject_run = 0;
  for (int i = 0; i < 2; ++i) fuse_axis(&f->axis[i], f->innov[i], R);
  ++f->updates;
  update_converged(f);
  return true;
}
