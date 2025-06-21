/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: StateFeedbackController.c
 *
 * Code generated for Simulink model 'StateFeedbackController'.
 *
 * Model version                  : 1.282
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Mon Jun 16 12:38:20 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "StateFeedbackController.h"
#include "rtwtypes.h"

/* Block signals and states (default storage) */
DW rtDW;

/* Model step function */
real_T StateFeedbackController_step(real_T arg_ref_x, real_T arg_x, real_T
  arg_theta, real_T arg_gain_k[4])
{
  /* specified return value */
  real_T arg_out;

  /* Outport: '<Root>/out' incorporates:
   *  DiscreteIntegrator: '<Root>/Discrete-Time Integrator'
   *  DotProduct: '<Root>/Dot Product'
   *  Gain: '<Root>/Gain1'
   *  Inport: '<Root>/GainK'
   *  Inport: '<Root>/theta'
   *  Inport: '<Root>/x'
   *  Sum: '<Root>/Sum1'
   *  Sum: '<S1>/Diff'
   *  Sum: '<S2>/Diff'
   *  UnitDelay: '<S1>/UD'
   *  UnitDelay: '<S2>/UD'
   *
   * Block description for '<S1>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S2>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S1>/UD':
   *
   *  Store in Global RAM
   *
   * Block description for '<S2>/UD':
   *
   *  Store in Global RAM
   */
  arg_out = ((((arg_x - rtDW.UD_DSTATE) * arg_gain_k[1] + arg_x * arg_gain_k[0])
              + arg_theta * arg_gain_k[2]) + (arg_theta - rtDW.UD_DSTATE_p) *
             arg_gain_k[3]) + 0.0 * rtDW.DiscreteTimeIntegrator_DSTATE;

  /* Update for UnitDelay: '<S2>/UD' incorporates:
   *  Inport: '<Root>/x'
   *
   * Block description for '<S2>/UD':
   *
   *  Store in Global RAM
   */
  rtDW.UD_DSTATE = arg_x;

  /* Update for UnitDelay: '<S1>/UD' incorporates:
   *  Inport: '<Root>/theta'
   *
   * Block description for '<S1>/UD':
   *
   *  Store in Global RAM
   */
  rtDW.UD_DSTATE_p = arg_theta;

  /* Update for DiscreteIntegrator: '<Root>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/ref x'
   *  Inport: '<Root>/x'
   *  Sum: '<Root>/Sum'
   */
  rtDW.DiscreteTimeIntegrator_DSTATE += (arg_ref_x - arg_x) * 0.001;
  return arg_out;
}

/* Model initialize function */
void StateFeedbackController_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
