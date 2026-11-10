/** @file test_bno_history.c @brief Alinhamento temporal BNO-MTF. */
#include "bno_history.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#define NEAR(a,b,t) (fabsf((a)-(b))<=(t))

int main(void) {
  bno_history_t h; bno_history_reset(&h);
  for (uint32_t i=0;i<8;++i) {
    const uint32_t t=100000U+i*10000U;
    const float yaw=0.00001F*(float)(t-100000U); /* 0,1 rad por amostra */
    const float q[4]={cosf(yaw*0.5F),0,0,sinf(yaw*0.5F)};
    const float g[3]={(float)i,2.0F*(float)i,0.5F};
    const float a[3]={0.1F*(float)i,-0.2F*(float)i,0};
    bno_history_push_rotation(&h,t,i+1,q);
    bno_history_push_gyro(&h,t,i+1,g);
    bno_history_push_accel(&h,t,i+1,a);
  }
  bno_aligned_sample_t out;
  /* [120,140] ms: média de uma rampa 2..4 = 3; atitude no meio = 0,3 rad. */
  assert(bno_history_align(&h,140000U,20000U,&out));
  assert(NEAR(out.gyro[0],3.0F,1e-5F));
  assert(NEAR(out.gyro[1],6.0F,1e-5F));
  assert(NEAR(out.accel[0],0.3F,1e-5F));
  assert(NEAR(out.accel[1],-0.6F,1e-5F));
  assert(NEAR(out.q[0],cosf(0.15F),1e-5F));
  assert(NEAR(out.q[3],sinf(0.15F),1e-5F));

  /* Sem cobertura próxima, o módulo recusa em vez de usar uma fotografia velha. */
  assert(!bno_history_align(&h,300000U,10000U,&out));

  /* Wrap de uint32_t: timestamps curtos depois do wrap ainda formam intervalo. */
  bno_history_reset(&h);
  const float qi[4]={1,0,0,0}, one[3]={1,1,1};
  bno_history_push_rotation(&h,0xFFFFFF00U,1,qi);
  bno_history_push_gyro(&h,0xFFFFFF00U,1,one);
  bno_history_push_accel(&h,0xFFFFFF00U,1,one);
  bno_history_push_rotation(&h,0x00002610U,2,qi);
  bno_history_push_gyro(&h,0x00002610U,2,one);
  bno_history_push_accel(&h,0x00002610U,2,one);
  assert(bno_history_align(&h,0x00002610U,10000U,&out));
  assert(NEAR(out.gyro[0],1.0F,1e-6F));
  puts("BNO history: interval average, midpoint quaternion, stale rejection and timer wrap passed");
  return 0;
}
