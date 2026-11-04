/*
 * File: STM32.h
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

#ifndef STM32_h_
#define STM32_h_
#ifndef STM32_COMMON_INCLUDES_
#define STM32_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* STM32_COMMON_INCLUDES_ */

#include "STM32_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T MemoryX_DSTATE[12];           /* '<S2>/MemoryX' */
  boolean_T icLoad;                    /* '<S2>/MemoryX' */
} DW_STM32_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: pInitialization.A
   * Referenced by: '<S2>/A'
   */
  real_T A_Value[144];

  /* Expression: pInitialization.B
   * Referenced by: '<S2>/B'
   */
  real_T B_Value[48];

  /* Expression: pInitialization.C
   * Referenced by: '<S2>/C'
   */
  real_T C_Value[108];

  /* Expression: pInitialization.L
   * Referenced by: '<S3>/KalmanGainL'
   */
  real_T KalmanGainL_Value[108];

  /* Expression: pInitialization.M
   * Referenced by: '<S3>/KalmanGainM'
   */
  real_T KalmanGainM_Value[108];

  /* Expression: Kfd
   * Referenced by: '<S1>/Gain7'
   */
  real_T Gain7_Gain[16];

  /* Expression: Kd
   * Referenced by: '<S1>/Gain5'
   */
  real_T Gain5_Gain[48];
} ConstP_STM32_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T In1[4];                       /* '<Root>/In1' */
  real_T y[9];                         /* '<Root>/y' */
} ExtU_STM32_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Out1[4];                      /* '<Root>/Out1' */
} ExtY_STM32_T;

/* Real-time Model Data Structure */
struct tag_RTM_STM32_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_STM32_T STM32_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_STM32_T STM32_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_STM32_T STM32_Y;

/* Constant parameters (default storage) */
extern const ConstP_STM32_T STM32_ConstP;

/* Model entry point functions */
extern void STM32_initialize(void);
extern void STM32_step(void);
extern void STM32_terminate(void);

/* Real-time Model object */
extern RT_MODEL_STM32_T *const STM32_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S46>/Data Type Duplicate' : Unused code path elimination
 * Block '<S47>/Data Type Duplicate' : Unused code path elimination
 * Block '<S48>/Conversion' : Unused code path elimination
 * Block '<S48>/Data Type Duplicate' : Unused code path elimination
 * Block '<S49>/Data Type Duplicate' : Unused code path elimination
 * Block '<S6>/Data Type Duplicate' : Unused code path elimination
 * Block '<S7>/Data Type Duplicate' : Unused code path elimination
 * Block '<S8>/Data Type Duplicate' : Unused code path elimination
 * Block '<S9>/Data Type Duplicate' : Unused code path elimination
 * Block '<S11>/Data Type Duplicate' : Unused code path elimination
 * Block '<S12>/Data Type Duplicate' : Unused code path elimination
 * Block '<S13>/Data Type Duplicate' : Unused code path elimination
 * Block '<S14>/Conversion' : Unused code path elimination
 * Block '<S14>/Data Type Duplicate' : Unused code path elimination
 * Block '<S15>/Data Type Duplicate' : Unused code path elimination
 * Block '<S16>/Data Type Duplicate' : Unused code path elimination
 * Block '<S17>/Data Type Duplicate' : Unused code path elimination
 * Block '<S19>/Data Type Duplicate' : Unused code path elimination
 * Block '<S20>/Data Type Duplicate' : Unused code path elimination
 * Block '<S2>/G' : Unused code path elimination
 * Block '<S2>/H' : Unused code path elimination
 * Block '<S2>/N' : Unused code path elimination
 * Block '<S2>/P0' : Unused code path elimination
 * Block '<S2>/Q' : Unused code path elimination
 * Block '<S2>/R' : Unused code path elimination
 * Block '<S44>/CheckSignalProperties' : Unused code path elimination
 * Block '<S45>/CheckSignalProperties' : Unused code path elimination
 * Block '<S46>/Conversion' : Eliminate redundant data type conversion
 * Block '<S47>/Conversion' : Eliminate redundant data type conversion
 * Block '<S49>/Conversion' : Eliminate redundant data type conversion
 * Block '<S19>/Conversion' : Eliminate redundant data type conversion
 * Block '<S23>/Reshape' : Reshape block reduction
 * Block '<S2>/ReshapeX0' : Reshape block reduction
 * Block '<S2>/Reshapeu' : Reshape block reduction
 * Block '<S2>/Reshapexhat' : Reshape block reduction
 * Block '<S2>/Reshapey' : Reshape block reduction
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'STM32'
 * '<S1>'   : 'STM32/STM32'
 * '<S2>'   : 'STM32/STM32/Kalman Filter'
 * '<S3>'   : 'STM32/STM32/Kalman Filter/CalculatePL'
 * '<S4>'   : 'STM32/STM32/Kalman Filter/CalculateYhat'
 * '<S5>'   : 'STM32/STM32/Kalman Filter/CovarianceOutputConfigurator'
 * '<S6>'   : 'STM32/STM32/Kalman Filter/DataTypeConversionA'
 * '<S7>'   : 'STM32/STM32/Kalman Filter/DataTypeConversionB'
 * '<S8>'   : 'STM32/STM32/Kalman Filter/DataTypeConversionC'
 * '<S9>'   : 'STM32/STM32/Kalman Filter/DataTypeConversionD'
 * '<S10>'  : 'STM32/STM32/Kalman Filter/DataTypeConversionEnable'
 * '<S11>'  : 'STM32/STM32/Kalman Filter/DataTypeConversionG'
 * '<S12>'  : 'STM32/STM32/Kalman Filter/DataTypeConversionH'
 * '<S13>'  : 'STM32/STM32/Kalman Filter/DataTypeConversionN'
 * '<S14>'  : 'STM32/STM32/Kalman Filter/DataTypeConversionP'
 * '<S15>'  : 'STM32/STM32/Kalman Filter/DataTypeConversionP0'
 * '<S16>'  : 'STM32/STM32/Kalman Filter/DataTypeConversionQ'
 * '<S17>'  : 'STM32/STM32/Kalman Filter/DataTypeConversionR'
 * '<S18>'  : 'STM32/STM32/Kalman Filter/DataTypeConversionReset'
 * '<S19>'  : 'STM32/STM32/Kalman Filter/DataTypeConversionX'
 * '<S20>'  : 'STM32/STM32/Kalman Filter/DataTypeConversionX0'
 * '<S21>'  : 'STM32/STM32/Kalman Filter/DataTypeConversionu'
 * '<S22>'  : 'STM32/STM32/Kalman Filter/MemoryP'
 * '<S23>'  : 'STM32/STM32/Kalman Filter/Observer'
 * '<S24>'  : 'STM32/STM32/Kalman Filter/ReducedQRN'
 * '<S25>'  : 'STM32/STM32/Kalman Filter/Reset'
 * '<S26>'  : 'STM32/STM32/Kalman Filter/Reshapeyhat'
 * '<S27>'  : 'STM32/STM32/Kalman Filter/ScalarExpansionP0'
 * '<S28>'  : 'STM32/STM32/Kalman Filter/ScalarExpansionQ'
 * '<S29>'  : 'STM32/STM32/Kalman Filter/ScalarExpansionR'
 * '<S30>'  : 'STM32/STM32/Kalman Filter/UseCurrentEstimator'
 * '<S31>'  : 'STM32/STM32/Kalman Filter/checkA'
 * '<S32>'  : 'STM32/STM32/Kalman Filter/checkB'
 * '<S33>'  : 'STM32/STM32/Kalman Filter/checkC'
 * '<S34>'  : 'STM32/STM32/Kalman Filter/checkD'
 * '<S35>'  : 'STM32/STM32/Kalman Filter/checkEnable'
 * '<S36>'  : 'STM32/STM32/Kalman Filter/checkG'
 * '<S37>'  : 'STM32/STM32/Kalman Filter/checkH'
 * '<S38>'  : 'STM32/STM32/Kalman Filter/checkN'
 * '<S39>'  : 'STM32/STM32/Kalman Filter/checkP0'
 * '<S40>'  : 'STM32/STM32/Kalman Filter/checkQ'
 * '<S41>'  : 'STM32/STM32/Kalman Filter/checkR'
 * '<S42>'  : 'STM32/STM32/Kalman Filter/checkReset'
 * '<S43>'  : 'STM32/STM32/Kalman Filter/checkX0'
 * '<S44>'  : 'STM32/STM32/Kalman Filter/checku'
 * '<S45>'  : 'STM32/STM32/Kalman Filter/checky'
 * '<S46>'  : 'STM32/STM32/Kalman Filter/CalculatePL/DataTypeConversionL'
 * '<S47>'  : 'STM32/STM32/Kalman Filter/CalculatePL/DataTypeConversionM'
 * '<S48>'  : 'STM32/STM32/Kalman Filter/CalculatePL/DataTypeConversionP'
 * '<S49>'  : 'STM32/STM32/Kalman Filter/CalculatePL/DataTypeConversionZ'
 * '<S50>'  : 'STM32/STM32/Kalman Filter/CalculatePL/Ground'
 * '<S51>'  : 'STM32/STM32/Kalman Filter/CalculateYhat/Ground'
 * '<S52>'  : 'STM32/STM32/Kalman Filter/CovarianceOutputConfigurator/decideOutput'
 * '<S53>'  : 'STM32/STM32/Kalman Filter/CovarianceOutputConfigurator/decideOutput/SqrtUsedFcn'
 * '<S54>'  : 'STM32/STM32/Kalman Filter/Observer/MeasurementUpdate'
 * '<S55>'  : 'STM32/STM32/Kalman Filter/ReducedQRN/Ground'
 * '<S56>'  : 'STM32/STM32/Kalman Filter/UseCurrentEstimator/Enabled Subsystem'
 */
#endif                                 /* STM32_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
