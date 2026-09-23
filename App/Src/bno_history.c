/** @file bno_history.c @brief Implementação de bno_history.h. */
#include "bno_history.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static int32_t delta_us(uint32_t a, uint32_t b) { return (int32_t)(a - b); }

void bno_history_reset(bno_history_t *h) { memset(h, 0, sizeof(*h)); }

void bno_history_push_rotation(bno_history_t *h, uint32_t t, uint32_t id,
                               const float q[4]) {
  bno_history_quat_sample_t *s = &h->rotation[h->rotation_head];
  s->time_us=t; s->id=id; memcpy(s->v,q,sizeof s->v);
  h->rotation_head=(uint8_t)((h->rotation_head+1U)%BNO_HISTORY_CAPACITY);
  if (h->rotation_count<BNO_HISTORY_CAPACITY) ++h->rotation_count;
}
static void push_vec(bno_history_vec3_sample_t *a, uint8_t *head,
                     uint8_t *count, uint32_t t, uint32_t id,
                     const float v[3]) {
  bno_history_vec3_sample_t *s=&a[*head];
  s->time_us=t; s->id=id; memcpy(s->v,v,sizeof s->v);
  *head=(uint8_t)((*head+1U)%BNO_HISTORY_CAPACITY);
  if (*count<BNO_HISTORY_CAPACITY) ++*count;
}
void bno_history_push_gyro(bno_history_t *h,uint32_t t,uint32_t id,const float v[3]) {
  push_vec(h->gyro,&h->gyro_head,&h->gyro_count,t,id,v);
}
void bno_history_push_accel(bno_history_t *h,uint32_t t,uint32_t id,const float v[3]) {
  push_vec(h->accel,&h->accel_head,&h->accel_count,t,id,v);
}

static unsigned oldest(unsigned head,unsigned count) {
  return (head+BNO_HISTORY_CAPACITY-count)%BNO_HISTORY_CAPACITY;
}

static bool interpolate_vec(const bno_history_vec3_sample_t *a,unsigned head,
                            unsigned count,uint32_t target,float out[3],
                            uint32_t *id) {
  if (!count) return false;
  const bno_history_vec3_sample_t *before=NULL,*after=NULL;
  for (unsigned n=0,i=oldest(head,count);n<count;++n,i=(i+1U)%BNO_HISTORY_CAPACITY) {
    const bno_history_vec3_sample_t *s=&a[i];
    const int32_t d=delta_us(s->time_us,target);
    if (d<=0) before=s;
    if (d>=0 && !after) after=s;
  }
  if (!before) before=after;
  if (!after) after=before;
  if (!before || !after) return false;
  if ((uint32_t)abs(delta_us(before->time_us,target))>BNO_HISTORY_MAX_EXTRAPOLATION_US ||
      (uint32_t)abs(delta_us(after->time_us,target))>BNO_HISTORY_MAX_EXTRAPOLATION_US) return false;
  const int32_t span=delta_us(after->time_us,before->time_us);
  float u=0.0F;
  if (span>0) u=(float)delta_us(target,before->time_us)/(float)span;
  for (int k=0;k<3;++k) out[k]=before->v[k]+u*(after->v[k]-before->v[k]);
  if (id) *id=(delta_us(target,before->time_us)<=delta_us(after->time_us,target))?before->id:after->id;
  return true;
}

static bool average_vec(const bno_history_vec3_sample_t *a,unsigned head,
                        unsigned count,uint32_t start,uint32_t end,
                        float out[3],uint32_t *id) {
  const uint32_t duration=(uint32_t)(end-start);
  if (!duration) return false;
  float previous[3], endpoint[3], integral[3]={0};
  if (!interpolate_vec(a,head,count,start,previous,NULL) ||
      !interpolate_vec(a,head,count,end,endpoint,id)) return false;
  uint32_t previous_t=start;
  for (unsigned n=0,i=oldest(head,count);n<count;++n,i=(i+1U)%BNO_HISTORY_CAPACITY) {
    const bno_history_vec3_sample_t *s=&a[i];
    if (delta_us(s->time_us,start)<=0 || delta_us(s->time_us,end)>=0) continue;
    const float dt=(float)(uint32_t)(s->time_us-previous_t);
    for (int k=0;k<3;++k) integral[k]+=0.5F*(previous[k]+s->v[k])*dt;
    memcpy(previous,s->v,sizeof previous); previous_t=s->time_us;
  }
  const float dt=(float)(uint32_t)(end-previous_t);
  for (int k=0;k<3;++k) {
    integral[k]+=0.5F*(previous[k]+endpoint[k])*dt;
    out[k]=integral[k]/(float)duration;
  }
  return true;
}

static bool interpolate_quat(const bno_history_t *h,uint32_t target,
                             float out[4],uint32_t *id) {
  if (!h->rotation_count) return false;
  const bno_history_quat_sample_t *before=NULL,*after=NULL;
  for (unsigned n=0,i=oldest(h->rotation_head,h->rotation_count);n<h->rotation_count;
       ++n,i=(i+1U)%BNO_HISTORY_CAPACITY) {
    const bno_history_quat_sample_t *s=&h->rotation[i];
    const int32_t d=delta_us(s->time_us,target);
    if (d<=0) before=s;
    if (d>=0 && !after) after=s;
  }
  if (!before) before=after;
  if (!after) after=before;
  if (!before || !after) return false;
  if ((uint32_t)abs(delta_us(before->time_us,target))>BNO_HISTORY_MAX_EXTRAPOLATION_US ||
      (uint32_t)abs(delta_us(after->time_us,target))>BNO_HISTORY_MAX_EXTRAPOLATION_US) return false;
  float u=0.0F; const int32_t span=delta_us(after->time_us,before->time_us);
  if (span>0) u=(float)delta_us(target,before->time_us)/(float)span;
  float dot=0.0F; for(int k=0;k<4;++k) dot+=before->v[k]*after->v[k];
  const float sign=dot<0.0F?-1.0F:1.0F;
  float norm=0.0F;
  for(int k=0;k<4;++k){out[k]=(1.0F-u)*before->v[k]+u*sign*after->v[k];norm+=out[k]*out[k];}
  if (!(norm>0.0F) || !isfinite(norm)) return false;
  norm=1.0F/sqrtf(norm); for(int k=0;k<4;++k) out[k]*=norm;
  if(id)*id=(delta_us(target,before->time_us)<=delta_us(after->time_us,target))?before->id:after->id;
  return true;
}

bool bno_history_align(const bno_history_t *h,uint32_t end,uint32_t interval,
                       bno_aligned_sample_t *out) {
  if (!h || !out || !interval || interval>250000U) return false;
  const uint32_t start=end-interval, middle=start+interval/2U;
  bno_aligned_sample_t v={0};
  if (!interpolate_quat(h,middle,v.q,&v.rotation_id) ||
      !average_vec(h->gyro,h->gyro_head,h->gyro_count,start,end,v.gyro,&v.gyro_id) ||
      !average_vec(h->accel,h->accel_head,h->accel_count,start,end,v.accel,&v.accel_id)) return false;
  *out=v; return true;
}
