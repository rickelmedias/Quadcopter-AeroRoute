#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct {
  int unused;
} UART_HandleTypeDef;
/** Relógio do teste, em ms (definido pelo teste que o usa). */
uint32_t HAL_GetTick(void);
#endif
