/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: state_feedback_controller_data.c
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

/* Constant parameters (default storage) */
const ConstP rtConstP = {
  /* Expression: Ad
   * Referenced by: '<S1>/Ad matrix'
   */
  { 1.0, 0.0, 0.0, 0.0, 0.0, 0.00099756872294798717, 0.99498832582368957,
    1.2149144749395092E-5, 0.025036120419265576, -0.14914508128747478,
    -1.5740705877922185E-7, -0.00031444651383469524, 1.0000252965587033,
    0.050575914141843231, 4.2828379317308883E-5, 5.8759212239161222E-9,
    1.168558922149904E-5, 0.00099905568955493853, 0.99812045634995272,
    -1.5933670470499045E-6, 1.136348838880356E-6, 0.0012474264535431475,
    -5.6769651265192752E-6, -0.0062267727525913488, -0.00016964736578179524 },

  /* Expression: Bd
   * Referenced by: '<S1>/Bd matrix'
   */
  { 0.0, 0.00038455121451569705, -8.8268188702926825E-7, -0.0019211387907013407,
    0.030811817148649768 },

  /* Expression: Cd
   * Referenced by: '<S1>/Cd matrix'
   */
  { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: Ld
   * Referenced by: '<S1>/Ld matrix'
   */
  { 0.005736234133980167, 0.00067218207367769622, -0.0027712761657203711,
    -0.28019053767273405, -19.823121890403829, -0.0062780238558761692,
    0.060457012780625874, 0.062867916179195071, 0.36487903269782151,
    -32.2811794607429 },

  /* Expression: K
   * Referenced by: '<Root>/State Feedback'
   */
  { -0.3162277660041275, -24.828358272519491, -100.48947518258879,
    -13.750142233596499, 0.055012329436106726 }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
