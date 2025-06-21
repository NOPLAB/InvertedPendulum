/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: StateFeedbackController.h
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

#ifndef StateFeedbackController_h_
#define StateFeedbackController_h_
#ifndef StateFeedbackController_COMMON_INCLUDES_
#define StateFeedbackController_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                            /* StateFeedbackController_COMMON_INCLUDES_ */

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T UD_DSTATE;                    /* '<S2>/UD' */
  real_T UD_DSTATE_p;                  /* '<S1>/UD' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<Root>/Discrete-Time Integrator' */
} DW;

/* Block signals and states (default storage) */
extern DW rtDW;

/* Model entry point functions */
extern void StateFeedbackController_initialize(void);

/* Customized model step function */
extern real_T StateFeedbackController_step(real_T arg_ref_x, real_T arg_x,
  real_T arg_theta, real_T arg_gain_k[4]);

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
 * '<Root>' : 'StateFeedbackController'
 * '<S1>'   : 'StateFeedbackController/Difference'
 * '<S2>'   : 'StateFeedbackController/Difference1'
 */
#endif                                 /* StateFeedbackController_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
