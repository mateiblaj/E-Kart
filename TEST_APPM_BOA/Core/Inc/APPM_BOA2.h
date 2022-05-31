/*
 * File: APPM_BOA2.h
 *
 * Code generated for Simulink model 'APPM_BOA2'.
 *
 * Model version                  : 1.1
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Wed Sep 15 18:08:37 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_APPM_BOA2_h_
#define RTW_HEADER_APPM_BOA2_h_
#include <stddef.h>
#ifndef APPM_BOA2_COMMON_INCLUDES_
#define APPM_BOA2_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* APPM_BOA2_COMMON_INCLUDES_ */

#include "APPM_BOA2_types.h"
#include "MW_target_hardware_resources.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UD_DSTATE;                    /* '<S11>/UD' */
  int32_T chartAbsoluteTimeCounter;    /* '<S1>/Chart' */
  int32_T durationLastReferenceTick_1; /* '<S1>/Chart' */
  int32_T chartAbsoluteTimeCounter_n;  /* '<S1>/1sec debounce' */
  int32_T durationLastReferenceTick_1_p;/* '<S1>/1sec debounce' */
  int32_T durationLastReferenceTick_1_i;/* '<S1>/1sec debounce' */
  int32_T chartAbsoluteTimeCounter_nk; /* '<S1>/100ms debounce' */
  int32_T durationLastReferenceTick_1_m;/* '<S1>/100ms debounce' */
  int32_T durationLastReferenceTick_1_k;/* '<S1>/100ms debounce' */
  uint8_T is_active_c2_APPM_BOA2;      /* '<S1>/Chart' */
  uint8_T is_c2_APPM_BOA2;             /* '<S1>/Chart' */
  uint8_T is_active_c3_APPM_BOA2;      /* '<S1>/1sec debounce' */
  uint8_T is_c3_APPM_BOA2;             /* '<S1>/1sec debounce' */
  uint8_T is_active_c1_APPM_BOA2;      /* '<S1>/100ms debounce' */
  uint8_T is_c1_APPM_BOA2;             /* '<S1>/100ms debounce' */
  boolean_T condWasTrueAtLastTimeStep_1;/* '<S1>/Chart' */
  boolean_T condWasTrueAtLastTimeStep_1_k;/* '<S1>/1sec debounce' */
  boolean_T condWasTrueAtLastTimeStep_1_h;/* '<S1>/1sec debounce' */
  boolean_T condWasTrueAtLastTimeStep_1_e;/* '<S1>/100ms debounce' */
  boolean_T condWasTrueAtLastTimeStep_1_n;/* '<S1>/100ms debounce' */
} DW_APPM_BOA2_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T APP_R;                        /* '<Root>/APP_R' */
  real_T Acc_switch;                   /* '<Root>/Acc_switch' */
  real_T Brake_switch;                 /* '<Root>/Brake_switch' */
} ExtU_APPM_BOA2_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  boolean_T CUT_PWR;                   /* '<Root>/CUT_PWR' */
} ExtY_APPM_BOA2_T;

/* Parameters (default storage) */
struct P_APPM_BOA2_T_ {
  real_T Difference_ICPrevInput;       /* Mask Parameter: Difference_ICPrevInput
                                        * Referenced by: '<S11>/UD'
                                        */
  real_T CompareToConstant_const;     /* Mask Parameter: CompareToConstant_const
                                       * Referenced by: '<S5>/Constant'
                                       */
  real_T CompareToConstant_const_j; /* Mask Parameter: CompareToConstant_const_j
                                     * Referenced by: '<S10>/Constant'
                                     */
  real_T CompareToConstant1_const;   /* Mask Parameter: CompareToConstant1_const
                                      * Referenced by: '<S6>/Constant'
                                      */
  real_T CompareToConstant2_const;   /* Mask Parameter: CompareToConstant2_const
                                      * Referenced by: '<S7>/Constant'
                                      */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S9>/Constant'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_APPM_BOA2_T {
  const char_T *errorStatus;
};

/* Block parameters (default storage) */
extern P_APPM_BOA2_T APPM_BOA2_P;

/* Block states (default storage) */
extern DW_APPM_BOA2_T APPM_BOA2_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_APPM_BOA2_T APPM_BOA2_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_APPM_BOA2_T APPM_BOA2_Y;

/* Model entry point functions */
extern void APPM_BOA2_initialize(void);
extern void APPM_BOA2_step(void);
extern void APPM_BOA2_terminate(void);

/* Real-time Model object */
extern RT_MODEL_APPM_BOA2_T *const APPM_BOA2_M;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S8>/Scope' : Unused code path elimination
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
 * '<Root>' : 'APPM_BOA2'
 * '<S1>'   : 'APPM_BOA2/Subsystem'
 * '<S2>'   : 'APPM_BOA2/Subsystem/100ms debounce'
 * '<S3>'   : 'APPM_BOA2/Subsystem/1sec debounce'
 * '<S4>'   : 'APPM_BOA2/Subsystem/Chart'
 * '<S5>'   : 'APPM_BOA2/Subsystem/Compare To Constant'
 * '<S6>'   : 'APPM_BOA2/Subsystem/Compare To Constant1'
 * '<S7>'   : 'APPM_BOA2/Subsystem/Compare To Constant2'
 * '<S8>'   : 'APPM_BOA2/Subsystem/Gradient check'
 * '<S9>'   : 'APPM_BOA2/Subsystem/IsPositive'
 * '<S10>'  : 'APPM_BOA2/Subsystem/Gradient check/Compare To Constant'
 * '<S11>'  : 'APPM_BOA2/Subsystem/Gradient check/Difference'
 */
#endif                                 /* RTW_HEADER_APPM_BOA2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
