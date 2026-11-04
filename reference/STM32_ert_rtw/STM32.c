/*
 * File: STM32.c
 *
 * Code generated for Simulink model 'STM32'.
 *
 * Model version                  : 2.17
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Mon Nov  2 13:38:02 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "STM32.h"
#include <string.h>
#include "rtwtypes.h"

/* Block states (default storage) */
DW_STM32_T STM32_DW;

/* External inputs (root inport signals with default storage) */
ExtU_STM32_T STM32_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_STM32_T STM32_Y;

/* Real-time model */
static RT_MODEL_STM32_T STM32_M_;
RT_MODEL_STM32_T *const STM32_M = &STM32_M_;

/* Model step function */
void STM32_step(void)
{
  real_T Product3[12];
  real_T tmp_0[12];
  real_T tmp_1[12];
  real_T tmp[9];
  real_T Product2;
  int32_T i;
  int32_T i_0;

  /* Delay: '<S2>/MemoryX' */
  if (STM32_DW.icLoad) {
    memset(&STM32_DW.MemoryX_DSTATE[0], 0, 12U * sizeof(real_T));
  }

  /* Outputs for Enabled SubSystem: '<S30>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S56>/Enable'
   */
  /* Sum: '<S56>/Add1' incorporates:
   *  Constant: '<S2>/C'
   *  Delay: '<S2>/MemoryX'
   *  Inport: '<Root>/y'
   *  Product: '<S56>/Product'
   */
  for (i = 0; i < 9; i++) {
    Product2 = 0.0;
    for (i_0 = 0; i_0 < 12; i_0++) {
      Product2 += STM32_ConstP.C_Value[9 * i_0 + i] *
        STM32_DW.MemoryX_DSTATE[i_0];
    }

    tmp[i] = STM32_U.y[i] - Product2;
  }

  /* End of Sum: '<S56>/Add1' */
  for (i = 0; i < 12; i++) {
    /* Product: '<S56>/Product2' incorporates:
     *  Constant: '<S3>/KalmanGainM'
     */
    Product2 = 0.0;
    for (i_0 = 0; i_0 < 9; i_0++) {
      Product2 += STM32_ConstP.KalmanGainM_Value[12 * i_0 + i] * tmp[i_0];
    }

    /* Sum: '<S30>/Add' incorporates:
     *  Delay: '<S2>/MemoryX'
     *  Product: '<S56>/Product2'
     */
    tmp_0[i] = Product2 + STM32_DW.MemoryX_DSTATE[i];
  }

  /* End of Outputs for SubSystem: '<S30>/Enabled Subsystem' */
  for (i = 0; i < 4; i++) {
    /* Gain: '<S1>/Gain5' */
    Product2 = 0.0;
    for (i_0 = 0; i_0 < 12; i_0++) {
      Product2 += STM32_ConstP.Gain5_Gain[(i_0 << 2) + i] * tmp_0[i_0];
    }

    /* Sum: '<S1>/Sum1' incorporates:
     *  Gain: '<S1>/Gain5'
     *  Gain: '<S1>/Gain7'
     *  Inport: '<Root>/In1'
     */
    STM32_Y.Out1[i] = (((STM32_ConstP.Gain7_Gain[i + 4] * STM32_U.In1[1] +
                         STM32_ConstP.Gain7_Gain[i] * STM32_U.In1[0]) +
                        STM32_ConstP.Gain7_Gain[i + 8] * STM32_U.In1[2]) +
                       STM32_ConstP.Gain7_Gain[i + 12] * STM32_U.In1[3]) -
      Product2;
  }

  /* Outputs for Enabled SubSystem: '<S23>/MeasurementUpdate' incorporates:
   *  EnablePort: '<S54>/Enable'
   */
  for (i = 0; i < 9; i++) {
    /* Product: '<S54>/C[k]*xhat[k|k-1]' incorporates:
     *  Constant: '<S2>/C'
     *  Delay: '<S2>/MemoryX'
     */
    Product2 = 0.0;
    for (i_0 = 0; i_0 < 12; i_0++) {
      Product2 += STM32_ConstP.C_Value[9 * i_0 + i] *
        STM32_DW.MemoryX_DSTATE[i_0];
    }

    /* Sum: '<S54>/Sum' incorporates:
     *  Constant: '<S2>/D'
     *  Inport: '<Root>/y'
     *  Product: '<S54>/C[k]*xhat[k|k-1]'
     *  Product: '<S54>/D[k]*u[k]'
     *  Sum: '<S54>/Add1'
     */
    tmp[i] = STM32_U.y[i] - ((((0.0 * STM32_Y.Out1[0] + 0.0 * STM32_Y.Out1[1]) +
      0.0 * STM32_Y.Out1[2]) + 0.0 * STM32_Y.Out1[3]) + Product2);
  }

  /* End of Outputs for SubSystem: '<S23>/MeasurementUpdate' */

  /* Update for Delay: '<S2>/MemoryX' */
  STM32_DW.icLoad = false;

  /* Outputs for Enabled SubSystem: '<S23>/MeasurementUpdate' incorporates:
   *  EnablePort: '<S54>/Enable'
   */
  for (i = 0; i < 12; i++) {
    /* Product: '<S54>/Product3' incorporates:
     *  Constant: '<S3>/KalmanGainL'
     */
    Product2 = 0.0;
    for (i_0 = 0; i_0 < 9; i_0++) {
      Product2 += STM32_ConstP.KalmanGainL_Value[12 * i_0 + i] * tmp[i_0];
    }

    Product3[i] = Product2;

    /* End of Product: '<S54>/Product3' */

    /* Product: '<S23>/B[k]*u[k]' incorporates:
     *  Constant: '<S2>/B'
     */
    tmp_0[i] = ((STM32_ConstP.B_Value[i + 12] * STM32_Y.Out1[1] +
                 STM32_ConstP.B_Value[i] * STM32_Y.Out1[0]) +
                STM32_ConstP.B_Value[i + 24] * STM32_Y.Out1[2]) +
      STM32_ConstP.B_Value[i + 36] * STM32_Y.Out1[3];

    /* Product: '<S23>/A[k]*xhat[k|k-1]' incorporates:
     *  Constant: '<S2>/A'
     *  Delay: '<S2>/MemoryX'
     */
    Product2 = 0.0;
    for (i_0 = 0; i_0 < 12; i_0++) {
      Product2 += STM32_ConstP.A_Value[12 * i_0 + i] *
        STM32_DW.MemoryX_DSTATE[i_0];
    }

    tmp_1[i] = Product2;

    /* End of Product: '<S23>/A[k]*xhat[k|k-1]' */
  }

  /* End of Outputs for SubSystem: '<S23>/MeasurementUpdate' */

  /* Update for Delay: '<S2>/MemoryX' incorporates:
   *  Sum: '<S23>/Add'
   */
  for (i = 0; i < 12; i++) {
    STM32_DW.MemoryX_DSTATE[i] = (tmp_0[i] + tmp_1[i]) + Product3[i];
  }
}

/* Model initialize function */
void STM32_initialize(void)
{
  /* InitializeConditions for Delay: '<S2>/MemoryX' */
  STM32_DW.icLoad = true;
}

/* Model terminate function */
void STM32_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
