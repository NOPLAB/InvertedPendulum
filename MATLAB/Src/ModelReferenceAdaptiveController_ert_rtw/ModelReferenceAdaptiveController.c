/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ModelReferenceAdaptiveController.c
 *
 * Code generated for Simulink model 'ModelReferenceAdaptiveController'.
 *
 * Model version                  : 1.19
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Jun 13 13:13:17 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "ModelReferenceAdaptiveController.h"
#include "rtwtypes.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static void rate_scheduler(void);

/*
 *         This function updates active task flag for each subrate.
 *         The function is called at model base rate, hence the
 *         generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (rtM->Timing.TaskCounters.TID[1])++;
  if ((rtM->Timing.TaskCounters.TID[1]) > 49) {/* Sample time: [0.05s, 0.0s] */
    rtM->Timing.TaskCounters.TID[1] = 0;
  }
}

/* Model step function */
real_T ModelReferenceAdaptiveController_step(real_T arg_r, real_T arg_x, real_T
  arg_theta)
{
  /* local block i/o variables */
  real_T rtb_Gain2[4];
  real_T rtb_Gain_tmp[16];
  real_T rtb_TmpSignalConversionAtMatr_0[16];
  real_T rtb_TmpSignalConversionAtMatrix[4];
  real_T tmp[4];
  real_T rtb_Gain2_0;
  real_T rtb_TmpSignalConversionAtMatr_1;
  real_T rtb_TmpSignalConversionAtMatr_2;
  real_T rtb_TmpSignalConversionAtMatr_3;
  real_T tmp_0;
  int32_T i;
  int32_T rtb_Gain_tmp_tmp;
  int32_T rtb_TmpSignalConversionAtMatr_4;

  /* specified return value */
  real_T arg_out;
  if (rtM->Timing.TaskCounters.TID[1] == 0) {
    /* SignalConversion generated from: '<S14>/Matrix Multiply' incorporates:
     *  Inport: '<Root>/theta'
     *  Inport: '<Root>/x'
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
    rtb_TmpSignalConversionAtMatrix[0] = arg_x;
    rtb_TmpSignalConversionAtMatrix[1] = arg_x - rtDW.UD_DSTATE;
    rtb_TmpSignalConversionAtMatrix[2] = arg_theta;
    rtb_TmpSignalConversionAtMatrix[3] = arg_theta - rtDW.UD_DSTATE_b;

    /* DiscreteStateSpace: '<S28>/Discrete State-Space' incorporates:
     *  Inport: '<Root>/r'
     */
    {
      rtb_Gain2[0] = (1.0)*rtDW.DiscreteStateSpace_DSTATE[0];
      rtb_Gain2[1] = (1.0)*rtDW.DiscreteStateSpace_DSTATE[1];
      rtb_Gain2[2] = (1.0)*rtDW.DiscreteStateSpace_DSTATE[2];
      rtb_Gain2[3] = (1.0)*rtDW.DiscreteStateSpace_DSTATE[3];
    }

    for (i = 0; i < 4; i++) {
      /* Gain: '<S23>/Gain2' incorporates:
       *  SignalConversion generated from: '<S14>/Matrix Multiply'
       *  Sum: '<S6>/Sum1'
       */
      rtb_Gain2_0 = rtb_TmpSignalConversionAtMatrix[i] - rtb_Gain2[i];
      rtb_Gain2[i] = rtb_Gain2_0;

      /* Product: '<S18>/x(t)e(t)' incorporates:
       *  Gain: '<S23>/Gain2'
       *  Inport: '<Root>/theta'
       *  Inport: '<Root>/x'
       *  Math: '<S18>/Transpose1'
       *  SignalConversion generated from: '<S14>/Matrix Multiply'
       */
      rtb_TmpSignalConversionAtMatr_4 = i << 2;
      rtb_TmpSignalConversionAtMatr_0[rtb_TmpSignalConversionAtMatr_4] = arg_x *
        rtb_Gain2_0;
      rtb_TmpSignalConversionAtMatr_0[rtb_TmpSignalConversionAtMatr_4 + 1] =
        rtb_TmpSignalConversionAtMatrix[1] * rtb_Gain2_0;
      rtb_TmpSignalConversionAtMatr_0[rtb_TmpSignalConversionAtMatr_4 + 2] =
        arg_theta * rtb_Gain2_0;
      rtb_TmpSignalConversionAtMatr_0[rtb_TmpSignalConversionAtMatr_4 + 3] =
        rtb_TmpSignalConversionAtMatrix[3] * rtb_Gain2_0;
    }

    for (i = 0; i < 4; i++) {
      /* Gain: '<S18>/Gain' */
      rtb_Gain2_0 = rtb_TmpSignalConversionAtMatr_0[i + 4];
      rtb_TmpSignalConversionAtMatr_1 = rtb_TmpSignalConversionAtMatr_0[i];
      rtb_TmpSignalConversionAtMatr_2 = rtb_TmpSignalConversionAtMatr_0[i + 8];
      rtb_TmpSignalConversionAtMatr_3 = rtb_TmpSignalConversionAtMatr_0[i + 12];

      /* Gain: '<S23>/Gain2' */
      tmp_0 = 0.0;
      for (rtb_TmpSignalConversionAtMatr_4 = 0; rtb_TmpSignalConversionAtMatr_4 <
           4; rtb_TmpSignalConversionAtMatr_4++) {
        /* Gain: '<S18>/Gain' */
        rtb_Gain_tmp_tmp = rtb_TmpSignalConversionAtMatr_4 << 2;
        rtb_Gain_tmp[i + rtb_Gain_tmp_tmp] = ((rtConstP.pooled2[rtb_Gain_tmp_tmp
          + 1] * rtb_Gain2_0 + rtConstP.pooled2[rtb_Gain_tmp_tmp] *
          rtb_TmpSignalConversionAtMatr_1) + rtConstP.pooled2[rtb_Gain_tmp_tmp +
          2] * rtb_TmpSignalConversionAtMatr_2) +
          rtConstP.pooled2[rtb_Gain_tmp_tmp + 3] *
          rtb_TmpSignalConversionAtMatr_3;

        /* Gain: '<S23>/Gain2' incorporates:
         *  Inport: '<Root>/r'
         *  Product: '<S23>/re(t)'
         */
        tmp_0 += rtConstP.pooled2[(i << 2) + rtb_TmpSignalConversionAtMatr_4] *
          (arg_r * rtb_Gain2[rtb_TmpSignalConversionAtMatr_4]);
      }

      /* Gain: '<S23>/Gain2' */
      tmp[i] = tmp_0;
    }

    /* Gain: '<S23>/Gain2' */
    rtb_Gain2[0] = tmp[0];
    rtb_Gain2[1] = tmp[1];
    rtb_Gain2[2] = tmp[2];
    rtb_Gain2[3] = tmp[3];

    /* Outport: '<Root>/out' incorporates:
     *  DiscreteIntegrator: '<S16>/DiscreteIntegrator'
     *  DiscreteIntegrator: '<S22>/DiscreteIntegrator'
     *  DiscreteIntegrator: '<S27>/DiscreteIntegrator'
     *  Inport: '<Root>/r'
     *  Inport: '<Root>/theta'
     *  Inport: '<Root>/x'
     *  Product: '<S14>/Matrix Multiply'
     *  Product: '<S18>/Product'
     *  Product: '<S23>/Matrix Multiply'
     *  SignalConversion generated from: '<S14>/Matrix Multiply'
     *  Sum: '<S6>/Sum'
     */
    rtY.out = ((((rtDW.DiscreteIntegrator_DSTATE[0] * arg_x +
                  rtDW.DiscreteIntegrator_DSTATE[1] *
                  rtb_TmpSignalConversionAtMatrix[1]) +
                 rtDW.DiscreteIntegrator_DSTATE[2] * arg_theta) +
                rtDW.DiscreteIntegrator_DSTATE[3] *
                rtb_TmpSignalConversionAtMatrix[3]) +
               rtDW.DiscreteIntegrator_DSTATE_m * arg_r) -
      (((rtDW.DiscreteIntegrator_DSTATE_i[0] * arg_x +
         rtDW.DiscreteIntegrator_DSTATE_i[1] * rtb_TmpSignalConversionAtMatrix[1])
        + rtDW.DiscreteIntegrator_DSTATE_i[2] * arg_theta) +
       rtDW.DiscreteIntegrator_DSTATE_i[3] * rtb_TmpSignalConversionAtMatrix[3]);

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
    rtDW.UD_DSTATE_b = arg_theta;

    /* Update for DiscreteStateSpace: '<S28>/Discrete State-Space' incorporates:
     *  Inport: '<Root>/r'
     */
    {
      real_T xnew[4];
      xnew[0] = (1.0)*rtDW.DiscreteStateSpace_DSTATE[0] + (0.043658020251613743)*
        rtDW.DiscreteStateSpace_DSTATE[1]
        + (-0.0003568603752980534)*rtDW.DiscreteStateSpace_DSTATE[2]
        + (-6.1099646001605579E-6)*rtDW.DiscreteStateSpace_DSTATE[3];
      xnew[0] += (2.4245949562796065E-5)*arg_r;
      xnew[1] = (0.75761274957713787)*rtDW.DiscreteStateSpace_DSTATE[1]
        + (-0.013484765924501799)*rtDW.DiscreteStateSpace_DSTATE[2]
        + (-0.00035686037529805345)*rtDW.DiscreteStateSpace_DSTATE[3];
      xnew[1] += (0.00092666789891798871)*arg_r;
      xnew[2] = (-0.031375333930495013)*rtDW.DiscreteStateSpace_DSTATE[1]
        + (0.93754612745599453)*rtDW.DiscreteStateSpace_DSTATE[2]
        + (0.048954013942377889)*rtDW.DiscreteStateSpace_DSTATE[3];
      xnew[2] += (0.00011995067694567053)*arg_r;
      xnew[3] = (-1.1855870338712609)*rtDW.DiscreteStateSpace_DSTATE[1]
        + (-2.468618213496145)*rtDW.DiscreteStateSpace_DSTATE[2]
        + (0.93754612745599453)*rtDW.DiscreteStateSpace_DSTATE[3];
      xnew[3] += (0.0045326041025063176)*arg_r;
      (void) memcpy(&rtDW.DiscreteStateSpace_DSTATE[0], xnew,
                    sizeof(real_T)*4);
    }

    for (i = 0; i < 4; i++) {
      /* Gain: '<S18>/Gain1' incorporates:
       *  Gain: '<S14>/Gain5'
       *  Gain: '<S18>/Gain'
       */
      rtb_Gain2_0 = ((rtb_Gain_tmp[i + 4] * 0.021240468214499927 +
                      rtb_Gain_tmp[i] * 0.0) + rtb_Gain_tmp[i + 8] * 0.0) +
        rtb_Gain_tmp[i + 12] * 0.10620234107249961;

      /* Update for DiscreteIntegrator: '<S22>/DiscreteIntegrator' incorporates:
       *  Gain: '<S18>/Gain1'
       *  Gain: '<S20>/gamma_Kx'
       */
      rtDW.DiscreteIntegrator_DSTATE[i] += rtb_Gain2_0 * -0.1 * 0.05;

      /* Update for DiscreteIntegrator: '<S16>/DiscreteIntegrator' */
      rtDW.DiscreteIntegrator_DSTATE_i[i] += rtb_Gain2_0 * 0.05;
    }

    /* Update for DiscreteIntegrator: '<S27>/DiscreteIntegrator' incorporates:
     *  Gain: '<S23>/Gain2'
     *  Gain: '<S23>/Gain3'
     *  Gain: '<S25>/gamma_Kr'
     */
    rtDW.DiscreteIntegrator_DSTATE_m += (((tmp[0] * 0.0 + tmp[1] *
      0.021240468214499927) + tmp[2] * 0.0) + tmp[3] * 0.10620234107249961) *
      -0.1 * 0.05;
  }

  rate_scheduler();

  /* Copy value for root outport '<Root>/out' since it is accessed globally */
  arg_out = rtY.out;
  return arg_out;
}

/* Model initialize function */
void ModelReferenceAdaptiveController_initialize(void)
{
  /* InitializeConditions for DiscreteIntegrator: '<S22>/DiscreteIntegrator' incorporates:
   *  Constant: '<S20>/Kx0'
   */
  rtDW.DiscreteIntegrator_DSTATE[0] = 3.1622776601686042;
  rtDW.DiscreteIntegrator_DSTATE[1] = 1.0556938824282325;
  rtDW.DiscreteIntegrator_DSTATE[2] = 380.07685131949637;
  rtDW.DiscreteIntegrator_DSTATE[3] = 88.564733598399272;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
