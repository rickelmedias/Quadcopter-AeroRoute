/** @file range_filter.c @brief Implementação de range_filter.h. */
#include "range_filter.h"
#include <math.h>
#include <string.h>
/** Constante de tempo: remove jitter sem esconder a dinâmica vertical. */
#define RANGE_TAU_S 0.05F
/** Salto admitido = margem fixa + velocidade vertical extrema * dt. */
#define RANGE_STEP_MARGIN_M 0.10F
#define RANGE_MAX_RATE_M_S  5.0F
#define RANGE_REANCHOR_SAMPLES 5U
void range_filter_reset(range_filter_t *f){memset(f,0,sizeof(*f));}
bool range_filter_update(range_filter_t *f,float z,float dt,bool valid,float *out){
  if(!f||!out||!valid||!isfinite(z)||!isfinite(dt)||z<0.08F||z>8.0F||dt<=0.0F){
    if(f) ++f->rejected;
    return false;
  }
  if(!f->initialized){f->value_m=f->last_raw_m=z;f->initialized=true;++f->accepted;*out=z;return true;}
  const float allowed=RANGE_STEP_MARGIN_M+RANGE_MAX_RATE_M_S*dt;
  if(fabsf(z-f->last_raw_m)>allowed){
    ++f->rejected;
    if(f->candidate_count==0U || fabsf(z-f->candidate_m)>allowed){
      f->candidate_m=z; f->candidate_count=1U;
    } else {
      f->candidate_m+=(z-f->candidate_m)/(float)(f->candidate_count+1U);
      if(f->candidate_count<UINT8_MAX) ++f->candidate_count;
    }
    if(f->candidate_count<RANGE_REANCHOR_SAMPLES) return false;
    f->value_m=f->last_raw_m=f->candidate_m;
    f->candidate_count=0U; ++f->reanchors; ++f->accepted; *out=f->value_m;
    return true;
  }
  f->candidate_count=0U;
  f->last_raw_m=z;
  const float alpha=dt/(RANGE_TAU_S+dt);
  f->value_m+=alpha*(z-f->value_m);
  ++f->accepted;*out=f->value_m;return true;
}
