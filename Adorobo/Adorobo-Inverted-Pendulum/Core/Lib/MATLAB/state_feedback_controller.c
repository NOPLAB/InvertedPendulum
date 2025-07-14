/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: state_feedback_controller.c
 *
 * Code generated for Simulink model 'state_feedback_controller'.
 *
 * Model version                  : 1.193
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Mon Jun 30 12:14:24 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "state_feedback_controller.h"
#include "rtwtypes.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

/* Block signals and states (default storage) */
DW rtDW;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
real_T state_feedback_controller_step(real_T arg_y[2])
{
  /* local block i/o variables */
  real_T rtb_Sum3[5];
  real_T rtb_Gain1;

  /* specified return value */
  real_T arg_u;

  {
    real_T tmp_0[5];
    real_T tmp_1[5];
    real_T tmp[2];
    real_T tmp_2;
    real_T tmp_3;
    int32_T i;
    int32_T i_0;

    /* Sum: '<S1>/Sum2' incorporates:
     *  Gain: '<S1>/Cd matrix'
     *  Gain: '<S1>/Dd matrix'
     *  Inport: '<Root>/y'
     *  Memory: '<Root>/Memory'
     *  Sum: '<S1>/Sum'
     *  UnitDelay: '<S1>/Unit Delay'
     */
    for (i = 0; i < 2; i++) {
      tmp_2 = 0.0;
      for (i_0 = 0; i_0 < 5; i_0++) {
        tmp_2 += rtConstP.Cdmatrix_Gain[(i_0 << 1) + i] *
          rtDW.UnitDelay_DSTATE[i_0];
      }

      tmp[i] = arg_y[i] - (0.0 * rtDW.Memory_PreviousInput + tmp_2);
    }

    /* End of Sum: '<S1>/Sum2' */

    /* Gain: '<S1>/Ld matrix' */
    tmp_2 = tmp[1];
    tmp_3 = tmp[0];
    for (i = 0; i < 5; i++) {
      /* Sum: '<S1>/Sum1' incorporates:
       *  Gain: '<S1>/Bd matrix'
       *  Gain: '<S1>/Ld matrix'
       *  Memory: '<Root>/Memory'
       */
      tmp_0[i] = (rtConstP.Ldmatrix_Gain[i + 5] * tmp_2 +
                  rtConstP.Ldmatrix_Gain[i] * tmp_3) + rtConstP.Bdmatrix_Gain[i]
        * rtDW.Memory_PreviousInput;

      /* Gain: '<S1>/Ad matrix' */
      tmp_1[i] = 0.0;
    }

    /* Gain: '<S1>/Ad matrix' incorporates:
     *  UnitDelay: '<S1>/Unit Delay'
     */
    for (i = 0; i < 5; i++) {
      tmp_2 = rtDW.UnitDelay_DSTATE[i];
      for (i_0 = 0; i_0 < 5; i_0++) {
        tmp_1[i_0] += rtConstP.Admatrix_Gain[5 * i + i_0] * tmp_2;
      }
    }

    /* Gain: '<Root>/State Feedback' */
    tmp_2 = 0.0;
    for (i = 0; i < 5; i++) {
      /* Sum: '<S1>/Sum3' */
      rtb_Sum3[i] = tmp_0[i] + tmp_1[i];

      /* Gain: '<Root>/State Feedback' incorporates:
       *  UnitDelay: '<S1>/Unit Delay'
       */
      tmp_2 += rtConstP.StateFeedback_Gain[i] * rtDW.UnitDelay_DSTATE[i];
    }

    /* Gain: '<Root>/Gain1' */
    rtb_Gain1 = -tmp_2;

    /* Outport: '<Root>/u' */
    rtY.u = rtb_Gain1;
  }

  {
    int32_T i;

    /* Update for UnitDelay: '<S1>/Unit Delay' incorporates:
     *  Sum: '<S1>/Sum3'
     */
    for (i = 0; i < 5; i++) {
      rtDW.UnitDelay_DSTATE[i] = rtb_Sum3[i];
    }

    /* End of Update for UnitDelay: '<S1>/Unit Delay' */

    /* Update for Memory: '<Root>/Memory' */
    rtDW.Memory_PreviousInput = rtb_Gain1;
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  rtM->Timing.t[0] =
    ((time_T)(++rtM->Timing.clockTick0)) * rtM->Timing.stepSize0;

  {
    /* Update absolute timer for sample time: [0.001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.001, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     */
    rtM->Timing.clockTick1++;
  }

  /* Copy value for root outport '<Root>/u' since it is accessed globally */
  arg_u = rtY.u;
  return arg_u;
}

/* Model initialize function */
void state_feedback_controller_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, (&rtmGetErrorStatus(rtM)));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&rtM->solverInfo, false);
  rtsiSetIsContModeFrozen(&rtM->solverInfo, false);
  rtsiSetSolverName(&rtM->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.001;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
