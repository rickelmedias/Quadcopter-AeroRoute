/**
 * @file disp_estimator.c
 * @brief Estimadores RAW e FUSED (contratos em displacement.h).
 *
 * Mantidos sem FreeRTOS e sem HAL para que os testes no PC os alimentem com
 * movimento sintético. A task que os alimenta fica em displacement.c, a mesma
 * divisão usada por mtf_parser.c e mtf01.c.
 *
 * Os filtros de validade seguem o espírito das verificações de fusão de fluxo
 * do ArduPilot, em libraries/AP_NavEKF3/AP_NavEKF3_OptFlowFusion.cpp.
 */
#include "displacement.h"
#include <math.h>

/** Qualidade de imagem mínima; abaixo disso a imagem não tem textura utilizável. */
#define QUALITY_HIGH   30U
#define QUALITY_LOW    15U
/** Cosseno mínimo da inclinação (~37 graus); além disso a distância deixa de
    corresponder à altura. */
#define MIN_COS_TILT   0.80F
/** Fluxo máximo aceito, em rad/s; o mesmo teto do ArduPilot para pegar frames
    corrompidos. */
#define MAX_FLOW_RAD_S 4.2F
/** Distância mínima, em m. O LiDAR mede até ~2 cm, mas o rastreamento óptico
    precisa de mais espaço antes de enxergar algo utilizável - a MicoAir indica
    cerca de 8 cm. Usar o limite do LiDAR em vez do da câmera deixava passar
    frames em que o fluxo não tinha significado. */
#define MIN_RANGE_M    0.08F
/** Distância máxima, em m (alcance do telêmetro). */
#define MAX_RANGE_M    8.0F
/** Menor intervalo aceito por frame, em s. */
#define MIN_DT_S       0.001F
/** Maior intervalo aceito por frame, em s; um intervalo maior significa frames
    perdidos, e não se extrapola sobre eles. */
#define MAX_DT_S       0.25F

/** @brief Estimador RAW (ver displacement.h). */
void disp_raw_update(disp_state_t *s, const disp_input_t *in, const disp_config_t *cfg) {
  /* Ingênuo de propósito, mantido como referência de bancada: fluxo vezes
     distância, integrado nos eixos do corpo. Nenhuma rotação removida, nenhum
     frame rejeitado, nenhum yaw considerado. É o comportamento com que o
     caminho FUSED é comparado. */
  s->vx = in->flow_x * in->range_m * cfg->flow_scale_x;
  s->vy = in->flow_y * in->range_m * cfg->flow_scale_y;
  s->last_dx = s->vx * in->dt_s;
  s->last_dy = s->vy * in->dt_s;
  s->x += s->last_dx;
  s->y += s->last_dy;
  ++s->samples;
}

/** @brief Estimador FUSED (ver displacement.h). */
bool disp_fused_update(disp_state_t *s, const disp_input_t *in, const disp_config_t *cfg) {
  const float cos_tilt = attitude_cos_tilt(in->q);
  if (s->quality_usable) {
    if (in->quality <= QUALITY_LOW) s->quality_usable = false;
  } else if (in->quality >= QUALITY_HIGH) {
    s->quality_usable = true;
  }
  if (!in->range_valid ||
      !s->quality_usable ||
      cos_tilt < MIN_COS_TILT ||
      in->range_m < MIN_RANGE_M || in->range_m > MAX_RANGE_M ||
      in->dt_s < MIN_DT_S || in->dt_s > MAX_DT_S ||
      !isfinite(in->flow_x) || !isfinite(in->flow_y) ||
      fabsf(in->flow_x) > MAX_FLOW_RAD_S || fabsf(in->flow_y) > MAX_FLOW_RAD_S) {
    ++s->rejected;
    return false;
  }

  /* A escala corrige a leitura do próprio sensor antes de ela ser combinada
     com qualquer outra coisa. Aplicar depois de somar o giroscópio (como uma
     versão antiga desta função fazia) escalaria também o termo do giroscópio,
     o que está errado - o giroscópio não é o que está sendo calibrado. Com
     escala 1.0 (o único valor usado até então) as duas ordens davam o mesmo
     resultado, e por isso o erro passou despercebido até a calibração por
     eixo tornar a ordem importante. */
  const float scaled_flow_x = in->flow_x * cfg->flow_scale_x +
                              in->flow_y * cfg->flow_cross_xy;
  const float scaled_flow_y = in->flow_x * cfg->flow_cross_yx +
                              in->flow_y * cfg->flow_scale_y;

  /* Compensação da rotação do corpo. Uma câmera voltada para baixo com eixos
     alinhados à IMU veria a rolagem varrer a imagem em Y e a arfagem em X
     (pareamento cruzado) - mas não é isso que esta placa mede. Testes
     isolados na bancada (um eixo girado por vez, placa segura no lugar)
     deram dois resultados limpos e com muitas amostras:
       - rolagem: gyro_x do BNO grande, gyro_y perto de zero -> a resposta
         caiu inteira em flow_x, com sinal OPOSTO, 214 de 218 amostras (98%).
       - arfagem: gyro_y do BNO grande, gyro_x perto de zero -> a resposta
         caiu inteira em flow_y, com o MESMO sinal, 142 de 143 amostras (99%).
     Então o pareamento é no mesmo eixo (não cruzado) nesta placa - os dois
     sensores estão montados com rotação relativa - mas o sinal NÃO é o mesmo
     nos dois eixos: X é invertido e Y não. Um único gyro_sign não atende os
     dois, por isso existem dois independentes. O que sobra depois da
     compensação é a parte do fluxo que só a translação poderia ter
     produzido. */
  float trans_x = scaled_flow_x + cfg->gyro_sign_x * in->gyro_x;
  float trans_y = scaled_flow_y + cfg->gyro_sign_y * in->gyro_y;

  /* Zero é uma célula de quantização, não velocidade exatamente nula. O
     centro continua aqui para o integrador diagnóstico; a meia largura é
     entregue ao Kalman, que aplica uma correção censurada por intervalo. */

  /* Zona morta extra opcional sobre a taxa compensada, desligada por padrão. */
  if (cfg->deadband_rad_s > 0.0F) {
    if (fabsf(trans_x) < cfg->deadband_rad_s) trans_x = 0.0F;
    if (fabsf(trans_y) < cfg->deadband_rad_s) trans_y = 0.0F;
  }

  /* Taxa angular vezes distância é uma velocidade linear, nos eixos do
     corpo. O ArduPilot divide pela distância inclinada, e não pela altura
     vertical, então a leitura bruta do telêmetro é o valor certo aqui. */
  float vx_body = trans_x * in->range_m;
  float vy_body = trans_y * in->range_m;
  /* O fluxo mede a velocidade no ponto focal. Remove omega x braço para
     recuperar a velocidade do centro do veículo. */
  vx_body -= in->gyro_y*cfg->sensor_offset_m[2] - in->gyro_z*cfg->sensor_offset_m[1];
  vy_body -= in->gyro_z*cfg->sensor_offset_m[0] - in->gyro_x*cfg->sensor_offset_m[2];

  /* Sem filtro de plausibilidade aqui, de propósito. Rejeitar uma amostra de
     uma soma corrente afirma "a velocidade foi zero naquele intervalo", e um
     ensaio de bancada mostrou essa afirmação derivando 21 mm em dois minutos
     com a placa intocada, enquanto a soma sem filtro ficou em 2 mm. A
     rejeição de valores absurdos pertence ao filtro, onde uma medida
     rejeitada deixa a predição de pé em vez de abrir um buraco na integral.
     Ver flow_ekf.h. */

  /* Girar para o referencial do mundo antes de integrar, senão o yaw vira
     deslocamento. */
  euler_t e;
  attitude_to_euler(in->q, &e);
  if (!s->yaw_origin_set) { s->yaw_origin=e.yaw; s->yaw_origin_set=true; }
  const float local_yaw=attitude_wrap_pi(e.yaw-s->yaw_origin);
  attitude_yaw_rotate(local_yaw, vx_body, vy_body, &s->vx, &s->vy);

  const bool qx=fabsf(in->flow_x)<0.5F*DISP_FLOW_LSB_RAD_S;
  const bool qy=fabsf(in->flow_y)<0.5F*DISP_FLOW_LSB_RAD_S;
  const float hx=in->range_m*0.5F*DISP_FLOW_LSB_RAD_S*
      (qx?fabsf(cfg->flow_scale_x):0.0F) + in->range_m*0.5F*DISP_FLOW_LSB_RAD_S*
      (qy?fabsf(cfg->flow_cross_xy):0.0F);
  const float hy=in->range_m*0.5F*DISP_FLOW_LSB_RAD_S*
      (qx?fabsf(cfg->flow_cross_yx):0.0F) + in->range_m*0.5F*DISP_FLOW_LSB_RAD_S*
      (qy?fabsf(cfg->flow_scale_y):0.0F);
  const float c=fabsf(cosf(local_yaw)), sn=fabsf(sinf(local_yaw));
  s->quant_half_width[0]=c*hx+sn*hy;
  s->quant_half_width[1]=sn*hx+c*hy;

  s->last_dx = s->vx * in->dt_s;
  s->last_dy = s->vy * in->dt_s;
  s->x += s->last_dx;
  s->y += s->last_dy;
  ++s->samples;
  return true;
}
