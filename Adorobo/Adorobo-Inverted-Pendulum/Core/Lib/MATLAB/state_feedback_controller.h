/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: state_feedback_controller.h
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

#ifndef state_feedback_controller_h_
#define state_feedback_controller_h_
#ifndef state_feedback_controller_COMMON_INCLUDES_
#define state_feedback_controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "math.h"
#endif                          /* state_feedback_controller_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay_DSTATE[5];          /* '<S1>/Unit Delay' */
  real_T Memory_PreviousInput;         /* '<Root>/Memory' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: Ad
   * Referenced by: '<S1>/Ad matrix'
   */
  real_T Admatrix_Gain[25];

  /* Expression: Bd
   * Referenced by: '<S1>/Bd matrix'
   */
  real_T Bdmatrix_Gain[5];

  /* Expression: Cd
   * Referenced by: '<S1>/Cd matrix'
   */
  real_T Cdmatrix_Gain[10];

  /* Expression: Ld
   * Referenced by: '<S1>/Ld matrix'
   */
  real_T Ldmatrix_Gain[10];

  /* Expression: K
   * Referenced by: '<Root>/State Feedback'
   */
  real_T StateFeedback_Gain[5];
} ConstP;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T u;                            /* '<Root>/u' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    SimTimeStep simTimeStep;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void state_feedback_controller_initialize(void);

/* Customized model step function */
extern real_T state_feedback_controller_step(real_T arg_y[2]);

/* Real-time Model object */
extern RT_MODEL *const rtM;

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
 * '<Root>' : 'state_feedback_controller'
 * '<S1>'   : 'state_feedback_controller/Luenberger Observer'
 */
#endif                                 /* state_feedback_controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
