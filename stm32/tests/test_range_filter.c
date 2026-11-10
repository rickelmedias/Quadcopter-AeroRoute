#include "range_filter.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void){
  range_filter_t f;range_filter_reset(&f);float z=0;
  assert(range_filter_update(&f,0.30F,0.01F,true,&z)&&fabsf(z-0.30F)<1e-6F);
  assert(range_filter_update(&f,0.31F,0.01F,true,&z));
  assert(z>0.30F&&z<0.31F);
  const float before=z;
  assert(!range_filter_update(&f,1.5F,0.01F,true,&z));
  assert(fabsf(f.value_m-before)<1e-7F&&f.rejected==1U);
  /* Um degrau real não pode travar o filtro para sempre. */
  for(int i=0;i<4;++i) assert(!range_filter_update(&f,1.0F,0.01F,true,&z));
  assert(range_filter_update(&f,1.0F,0.01F,true,&z));
  assert(fabsf(z-1.0F)<1e-6F && f.reanchors==1U);
  assert(range_filter_update(&f,1.01F,0.01F,true,&z));
  assert(!range_filter_update(&f,NAN,0.01F,true,&z));
  assert(!range_filter_update(&f,0.3F,0.01F,false,&z));
  puts("Range filter: smoothing, isolated step rejection, persistent-step recovery and invalid input passed");
  return 0;
}
