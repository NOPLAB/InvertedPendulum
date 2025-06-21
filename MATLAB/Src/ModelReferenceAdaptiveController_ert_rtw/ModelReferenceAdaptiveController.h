/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ModelReferenceAdaptiveController.h
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

#ifndef ModelReferenceAdaptiveController_h_
#define ModelReferenceAdaptiveController_h_
#ifndef ModelReferenceAdaptiveController_COMMON_INCLUDES_
#define ModelReferenceAdaptiveController_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "math.h"
#endif                   /* ModelReferenceAdaptiveController_COMMON_INCLUDES_ */

#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteStateSpace_DSTATE[4]; /* '<S28>/Discrete State-Space' */
  real_T DiscreteIntegrator_DSTATE[4]; /* '<S22>/DiscreteIntegrator' */
  real_T DiscreteIntegrator_DSTATE_i[4];/* '<S16>/DiscreteIntegrator' */
  real_T UD_DSTATE;                    /* '<S2>/UD' */
  real_T UD_DSTATE_b;                  /* '<S1>/UD' */
  real_T DiscreteIntegrator_DSTATE_m;  /* '<S27>/DiscreteIntegrator' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: blkParam.P)
   * Referenced by:
   *   '<S18>/Gain'
   *   '<S23>/Gain2'
   *   '<S14>/Gain4'
   */
  real_T pooled2[16];
} ConstP;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T out;                          /* '<Root>/out' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void ModelReferenceAdaptiveController_initialize(void);

/* Customized model step function */
extern real_T ModelReferenceAdaptiveController_step(real_T arg_r, real_T arg_x,
  real_T arg_theta);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S3>/phi' : Unused code path elimination
 * Block '<S14>/Gamma_w' : Eliminated nontunable gain of 1
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
 * '<Root>' : 'ModelReferenceAdaptiveController'
 * '<S1>'   : 'ModelReferenceAdaptiveController/Difference'
 * '<S2>'   : 'ModelReferenceAdaptiveController/Difference1'
 * '<S3>'   : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control'
 * '<S4>'   : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem'
 * '<S5>'   : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller'
 * '<S6>'   : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct'
 * '<S7>'   : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Disturbance Model'
 * '<S8>'   : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Feedback Control'
 * '<S9>'   : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Feedforward Control'
 * '<S10>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/ReferencePlant'
 * '<S11>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Disturbance Model/CONV_SHL_MRAC'
 * '<S12>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Disturbance Model/Feature Vector'
 * '<S13>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Disturbance Model/CONV_SHL_MRAC/Conventional_MRAC'
 * '<S14>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Disturbance Model/CONV_SHL_MRAC/Conventional_MRAC/default'
 * '<S15>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Disturbance Model/CONV_SHL_MRAC/Conventional_MRAC/default/WIntegrator_default'
 * '<S16>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Disturbance Model/CONV_SHL_MRAC/Conventional_MRAC/default/WIntegrator_default/W_DiscreteIntegrator'
 * '<S17>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Disturbance Model/Feature Vector/State'
 * '<S18>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Feedback Control/Adaptive'
 * '<S19>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Feedback Control/Adaptive/Feedback Update'
 * '<S20>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Feedback Control/Adaptive/Feedback Update/default'
 * '<S21>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Feedback Control/Adaptive/Feedback Update/default/FBGainIntegrator_default'
 * '<S22>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Feedback Control/Adaptive/Feedback Update/default/FBGainIntegrator_default/FBDiscreteIntegrator'
 * '<S23>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Feedforward Control/Adaptive'
 * '<S24>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Feedforward Control/Adaptive/Feedforward Update'
 * '<S25>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Feedforward Control/Adaptive/Feedforward Update/default'
 * '<S26>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Feedforward Control/Adaptive/Feedforward Update/default/FFGainIntegrator_default'
 * '<S27>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/Feedforward Control/Adaptive/Feedforward Update/default/FFGainIntegrator_default/FFDiscreteIntegrator'
 * '<S28>'  : 'ModelReferenceAdaptiveController/Model Reference Adaptive Control/MRAC Subsystem/MRAC Controller/Direct/ReferencePlant/Discrete_RefModel'
 */
#endif                                 /* ModelReferenceAdaptiveController_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
