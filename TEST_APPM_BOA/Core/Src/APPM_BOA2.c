/*
 * File: APPM_BOA2.c
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

#include "APPM_BOA2.h"
#include "APPM_BOA2_private.h"

/* Named constants for Chart: '<S1>/100ms debounce' */
#define APPM_BOA2_IN_off               ((uint8_T)1U)
#define APPM_BOA2_IN_on                ((uint8_T)2U)

/* Named constants for Chart: '<S1>/Chart' */
#define APPM_BOA2_IN_Gradient_NOK      ((uint8_T)1U)
#define APPM_BOA2_IN_Gradient_OK       ((uint8_T)2U)

/* Block states (default storage) */
DW_APPM_BOA2_T APPM_BOA2_DW;

/* External inputs (root inport signals with default storage) */
ExtU_APPM_BOA2_T APPM_BOA2_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_APPM_BOA2_T APPM_BOA2_Y;

/* Real-time model */
static RT_MODEL_APPM_BOA2_T APPM_BOA2_M_;
RT_MODEL_APPM_BOA2_T *const APPM_BOA2_M = &APPM_BOA2_M_;

/* Model step function */
void APPM_BOA2_step(void)
{
  int32_T rtb_cut_pwr;
  int32_T rtb_cut_pwr_h;
  int32_T rtb_cut_pwr_i;
  boolean_T condIsTrue;
  boolean_T rtb_LogicalOperator1;
  boolean_T tmp;

  /* Logic: '<S1>/Logical Operator1' incorporates:
   *  Constant: '<S5>/Constant'
   *  Constant: '<S9>/Constant'
   *  Inport: '<Root>/APP_R'
   *  Inport: '<Root>/Acc_switch'
   *  RelationalOperator: '<S5>/Compare'
   *  RelationalOperator: '<S9>/Compare'
   */
  rtb_LogicalOperator1 = (APPM_BOA2_U.APP_R > APPM_BOA2_P.Constant_Value) ^
    (APPM_BOA2_U.Acc_switch == APPM_BOA2_P.CompareToConstant_const);

  /* Chart: '<S1>/100ms debounce' */
  APPM_BOA2_DW.chartAbsoluteTimeCounter_nk++;
  condIsTrue = !rtb_LogicalOperator1;
  if ((!condIsTrue) || (!APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_e)) {
    APPM_BOA2_DW.durationLastReferenceTick_1_m =
      APPM_BOA2_DW.chartAbsoluteTimeCounter_nk;
  }

  APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_e = condIsTrue;
  if ((!rtb_LogicalOperator1) || (!APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_n))
  {
    APPM_BOA2_DW.durationLastReferenceTick_1_k =
      APPM_BOA2_DW.chartAbsoluteTimeCounter_nk;
  }

  APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_n = rtb_LogicalOperator1;
  if (APPM_BOA2_DW.is_active_c1_APPM_BOA2 == 0U) {
    APPM_BOA2_DW.chartAbsoluteTimeCounter_nk = 0;
    APPM_BOA2_DW.is_active_c1_APPM_BOA2 = 1U;
    APPM_BOA2_DW.durationLastReferenceTick_1_k =
      APPM_BOA2_DW.chartAbsoluteTimeCounter_nk;
    APPM_BOA2_DW.is_c1_APPM_BOA2 = APPM_BOA2_IN_off;
    rtb_cut_pwr_h = 0;
    APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_n = rtb_LogicalOperator1;
  } else if (APPM_BOA2_DW.is_c1_APPM_BOA2 == APPM_BOA2_IN_off) {
    if ((!rtb_LogicalOperator1) || (!APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_n))
    {
      APPM_BOA2_DW.durationLastReferenceTick_1_k =
        APPM_BOA2_DW.chartAbsoluteTimeCounter_nk;
    }

    if (APPM_BOA2_DW.chartAbsoluteTimeCounter_nk -
        APPM_BOA2_DW.durationLastReferenceTick_1_k > 10) {
      APPM_BOA2_DW.durationLastReferenceTick_1_m =
        APPM_BOA2_DW.chartAbsoluteTimeCounter_nk;
      APPM_BOA2_DW.is_c1_APPM_BOA2 = APPM_BOA2_IN_on;
      rtb_cut_pwr_h = 1;
      APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_e = !rtb_LogicalOperator1;
    } else {
      rtb_cut_pwr_h = 0;
    }
  } else {
    /* case IN_on: */
    condIsTrue = !rtb_LogicalOperator1;
    if ((!condIsTrue) || (!APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_e)) {
      APPM_BOA2_DW.durationLastReferenceTick_1_m =
        APPM_BOA2_DW.chartAbsoluteTimeCounter_nk;
    }

    APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_e = condIsTrue;
    if (APPM_BOA2_DW.chartAbsoluteTimeCounter_nk -
        APPM_BOA2_DW.durationLastReferenceTick_1_m > 500) {
      APPM_BOA2_DW.durationLastReferenceTick_1_k =
        APPM_BOA2_DW.chartAbsoluteTimeCounter_nk;
      APPM_BOA2_DW.is_c1_APPM_BOA2 = APPM_BOA2_IN_off;
      rtb_cut_pwr_h = 0;
      APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_n = rtb_LogicalOperator1;
    } else {
      rtb_cut_pwr_h = 1;
    }
  }

  /* End of Chart: '<S1>/100ms debounce' */

  /* Chart: '<S1>/Chart' incorporates:
   *  Constant: '<S10>/Constant'
   *  Inport: '<Root>/APP_R'
   *  Inport: '<Root>/Acc_switch'
   *  RelationalOperator: '<S10>/Compare'
   *  Sum: '<S11>/Diff'
   *  UnitDelay: '<S11>/UD'
   *
   * Block description for '<S11>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S11>/UD':
   *
   *  Store in Global RAM
   */
  APPM_BOA2_DW.chartAbsoluteTimeCounter++;
  condIsTrue = (APPM_BOA2_U.Acc_switch == 0.0);
  if ((!condIsTrue) || (!APPM_BOA2_DW.condWasTrueAtLastTimeStep_1)) {
    APPM_BOA2_DW.durationLastReferenceTick_1 =
      APPM_BOA2_DW.chartAbsoluteTimeCounter;
  }

  APPM_BOA2_DW.condWasTrueAtLastTimeStep_1 = condIsTrue;
  if (APPM_BOA2_DW.is_active_c2_APPM_BOA2 == 0U) {
    APPM_BOA2_DW.chartAbsoluteTimeCounter = 0;
    APPM_BOA2_DW.is_active_c2_APPM_BOA2 = 1U;
    APPM_BOA2_DW.is_c2_APPM_BOA2 = APPM_BOA2_IN_Gradient_OK;
    rtb_cut_pwr = 0;
  } else if (APPM_BOA2_DW.is_c2_APPM_BOA2 == APPM_BOA2_IN_Gradient_NOK) {
    condIsTrue = (APPM_BOA2_U.Acc_switch == 0.0);
    if ((!condIsTrue) || (!APPM_BOA2_DW.condWasTrueAtLastTimeStep_1)) {
      APPM_BOA2_DW.durationLastReferenceTick_1 =
        APPM_BOA2_DW.chartAbsoluteTimeCounter;
    }

    APPM_BOA2_DW.condWasTrueAtLastTimeStep_1 = condIsTrue;
    if (APPM_BOA2_DW.chartAbsoluteTimeCounter -
        APPM_BOA2_DW.durationLastReferenceTick_1 > 300) {
      APPM_BOA2_DW.is_c2_APPM_BOA2 = APPM_BOA2_IN_Gradient_OK;
      rtb_cut_pwr = 0;
    } else {
      rtb_cut_pwr = 1;
    }

    /* case IN_Gradient_OK: */
  } else if ((APPM_BOA2_U.APP_R - APPM_BOA2_DW.UD_DSTATE >
              APPM_BOA2_P.CompareToConstant_const_j) && (APPM_BOA2_U.Acc_switch ==
              1.0)) {
    APPM_BOA2_DW.durationLastReferenceTick_1 =
      APPM_BOA2_DW.chartAbsoluteTimeCounter;
    APPM_BOA2_DW.is_c2_APPM_BOA2 = APPM_BOA2_IN_Gradient_NOK;
    rtb_cut_pwr = 1;
    APPM_BOA2_DW.condWasTrueAtLastTimeStep_1 = (APPM_BOA2_U.Acc_switch == 0.0);
  } else {
    rtb_cut_pwr = 0;
  }

  /* End of Chart: '<S1>/Chart' */

  /* Logic: '<S1>/Logical Operator3' incorporates:
   *  Constant: '<S6>/Constant'
   *  Constant: '<S7>/Constant'
   *  Inport: '<Root>/Acc_switch'
   *  Inport: '<Root>/Brake_switch'
   *  RelationalOperator: '<S6>/Compare'
   *  RelationalOperator: '<S7>/Compare'
   */
  rtb_LogicalOperator1 = ((APPM_BOA2_U.Acc_switch ==
    APPM_BOA2_P.CompareToConstant1_const) && (APPM_BOA2_U.Brake_switch ==
    APPM_BOA2_P.CompareToConstant2_const));

  /* Chart: '<S1>/1sec debounce' */
  APPM_BOA2_DW.chartAbsoluteTimeCounter_n++;
  condIsTrue = !rtb_LogicalOperator1;
  tmp = !condIsTrue;
  if (tmp || (!APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_k)) {
    APPM_BOA2_DW.durationLastReferenceTick_1_p =
      APPM_BOA2_DW.chartAbsoluteTimeCounter_n;
  }

  APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_k = condIsTrue;
  if (condIsTrue || (!APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_h)) {
    APPM_BOA2_DW.durationLastReferenceTick_1_i =
      APPM_BOA2_DW.chartAbsoluteTimeCounter_n;
  }

  APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_h = rtb_LogicalOperator1;
  if (APPM_BOA2_DW.is_active_c3_APPM_BOA2 == 0U) {
    APPM_BOA2_DW.chartAbsoluteTimeCounter_n = 0;
    APPM_BOA2_DW.is_active_c3_APPM_BOA2 = 1U;
    APPM_BOA2_DW.durationLastReferenceTick_1_i =
      APPM_BOA2_DW.chartAbsoluteTimeCounter_n;
    APPM_BOA2_DW.is_c3_APPM_BOA2 = APPM_BOA2_IN_off;
    rtb_cut_pwr_i = 0;
    APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_h = rtb_LogicalOperator1;
  } else if (APPM_BOA2_DW.is_c3_APPM_BOA2 == APPM_BOA2_IN_off) {
    if (condIsTrue || (!APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_h)) {
      APPM_BOA2_DW.durationLastReferenceTick_1_i =
        APPM_BOA2_DW.chartAbsoluteTimeCounter_n;
    }

    if (APPM_BOA2_DW.chartAbsoluteTimeCounter_n -
        APPM_BOA2_DW.durationLastReferenceTick_1_i > 100) {
      APPM_BOA2_DW.durationLastReferenceTick_1_p =
        APPM_BOA2_DW.chartAbsoluteTimeCounter_n;
      APPM_BOA2_DW.is_c3_APPM_BOA2 = APPM_BOA2_IN_on;
      rtb_cut_pwr_i = 1;
      APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_k = condIsTrue;
    } else {
      rtb_cut_pwr_i = 0;
    }
  } else {
    /* case IN_on: */
    if (tmp || (!APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_k)) {
      APPM_BOA2_DW.durationLastReferenceTick_1_p =
        APPM_BOA2_DW.chartAbsoluteTimeCounter_n;
    }

    APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_k = condIsTrue;
    if (APPM_BOA2_DW.chartAbsoluteTimeCounter_n -
        APPM_BOA2_DW.durationLastReferenceTick_1_p > 500) {
      APPM_BOA2_DW.durationLastReferenceTick_1_i =
        APPM_BOA2_DW.chartAbsoluteTimeCounter_n;
      APPM_BOA2_DW.is_c3_APPM_BOA2 = APPM_BOA2_IN_off;
      rtb_cut_pwr_i = 0;
      APPM_BOA2_DW.condWasTrueAtLastTimeStep_1_h = rtb_LogicalOperator1;
    } else {
      rtb_cut_pwr_i = 1;
    }
  }

  /* End of Chart: '<S1>/1sec debounce' */

  /* Outport: '<Root>/CUT_PWR' incorporates:
   *  Logic: '<S1>/Logical Operator'
   */
  APPM_BOA2_Y.CUT_PWR = ((rtb_cut_pwr_h != 0) || (rtb_cut_pwr != 0) ||
    (rtb_cut_pwr_i != 0));

  /* Update for UnitDelay: '<S11>/UD' incorporates:
   *  Inport: '<Root>/APP_R'
   *
   * Block description for '<S11>/UD':
   *
   *  Store in Global RAM
   */
  APPM_BOA2_DW.UD_DSTATE = APPM_BOA2_U.APP_R;
}

/* Model initialize function */
void APPM_BOA2_initialize(void)
{
  /* InitializeConditions for UnitDelay: '<S11>/UD'
   *
   * Block description for '<S11>/UD':
   *
   *  Store in Global RAM
   */
  APPM_BOA2_DW.UD_DSTATE = APPM_BOA2_P.Difference_ICPrevInput;
}

/* Model terminate function */
void APPM_BOA2_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
