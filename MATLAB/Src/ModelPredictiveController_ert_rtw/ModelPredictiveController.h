/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: ModelPredictiveController.h
 *
 * Code generated for Simulink model 'ModelPredictiveController'.
 *
 * Model version                  : 1.41
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Mon Jun 16 15:38:04 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef ModelPredictiveController_h_
#define ModelPredictiveController_h_
#ifndef ModelPredictiveController_COMMON_INCLUDES_
#define ModelPredictiveController_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "math.h"
#endif                          /* ModelPredictiveController_COMMON_INCLUDES_ */

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
  real_T last_x_PreviousInput[7];      /* '<S2>/last_x' */
  real_T last_mv_DSTATE;               /* '<S2>/last_mv' */
  boolean_T Memory_PreviousInput[4];   /* '<S2>/Memory' */
} DW;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* Model entry point functions */
extern void ModelPredictiveController_initialize(void);

/* Customized model step function */
extern real_T ModelPredictiveController_step(real_T *arg_ref, real_T arg_x,
  real_T arg_dx, real_T arg_theta, real_T arg_dtheta);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S2>/Constant' : Unused code path elimination
 * Block '<S2>/Floor' : Unused code path elimination
 * Block '<S2>/Floor1' : Unused code path elimination
 * Block '<S3>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S4>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S5>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S6>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S7>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S8>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S9>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S10>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S11>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S12>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S13>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S14>/Matrix Dimension Check' : Unused code path elimination
 * Block '<S15>/Vector Dimension Check' : Unused code path elimination
 * Block '<S16>/Vector Dimension Check' : Unused code path elimination
 * Block '<S17>/Vector Dimension Check' : Unused code path elimination
 * Block '<S18>/Vector Dimension Check' : Unused code path elimination
 * Block '<S19>/Vector Dimension Check' : Unused code path elimination
 * Block '<S20>/Vector Dimension Check' : Unused code path elimination
 * Block '<S2>/Min' : Unused code path elimination
 * Block '<S2>/constant' : Unused code path elimination
 * Block '<S21>/Vector Dimension Check' : Unused code path elimination
 * Block '<S2>/umin_scale2' : Unused code path elimination
 * Block '<S2>/umin_scale3' : Unused code path elimination
 * Block '<S2>/umin_scale5' : Unused code path elimination
 * Block '<S2>/ym_zero' : Unused code path elimination
 * Block '<S1>/m_zero' : Unused code path elimination
 * Block '<S1>/p_zero' : Unused code path elimination
 * Block '<S2>/Reshape' : Reshape block reduction
 * Block '<S2>/Reshape1' : Reshape block reduction
 * Block '<S2>/Reshape2' : Reshape block reduction
 * Block '<S2>/Reshape3' : Reshape block reduction
 * Block '<S2>/Reshape4' : Reshape block reduction
 * Block '<S2>/Reshape5' : Reshape block reduction
 * Block '<S2>/ext.mv_scale' : Eliminated nontunable gain of 1
 * Block '<S2>/ext.mv_scale1' : Eliminated nontunable gain of 1
 * Block '<S2>/umin_scale1' : Eliminated nontunable gain of 1
 * Block '<S2>/umin_scale4' : Eliminated nontunable gain of 1
 * Block '<S2>/ymin_scale1' : Eliminated nontunable gain of 1
 * Block '<S2>/ymin_scale2' : Eliminated nontunable gain of 1
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
 * '<Root>' : 'ModelPredictiveController'
 * '<S1>'   : 'ModelPredictiveController/MPC Controller'
 * '<S2>'   : 'ModelPredictiveController/MPC Controller/MPC'
 * '<S3>'   : 'ModelPredictiveController/MPC Controller/MPC/MPC Matrix Signal Check'
 * '<S4>'   : 'ModelPredictiveController/MPC Controller/MPC/MPC Matrix Signal Check1'
 * '<S5>'   : 'ModelPredictiveController/MPC Controller/MPC/MPC Matrix Signal Check2'
 * '<S6>'   : 'ModelPredictiveController/MPC Controller/MPC/MPC Preview Signal Check'
 * '<S7>'   : 'ModelPredictiveController/MPC Controller/MPC/MPC Preview Signal Check1'
 * '<S8>'   : 'ModelPredictiveController/MPC Controller/MPC/MPC Preview Signal Check2'
 * '<S9>'   : 'ModelPredictiveController/MPC Controller/MPC/MPC Preview Signal Check3'
 * '<S10>'  : 'ModelPredictiveController/MPC Controller/MPC/MPC Preview Signal Check4'
 * '<S11>'  : 'ModelPredictiveController/MPC Controller/MPC/MPC Preview Signal Check5'
 * '<S12>'  : 'ModelPredictiveController/MPC Controller/MPC/MPC Preview Signal Check6'
 * '<S13>'  : 'ModelPredictiveController/MPC Controller/MPC/MPC Preview Signal Check7'
 * '<S14>'  : 'ModelPredictiveController/MPC Controller/MPC/MPC Preview Signal Check8'
 * '<S15>'  : 'ModelPredictiveController/MPC Controller/MPC/MPC Scalar Signal Check'
 * '<S16>'  : 'ModelPredictiveController/MPC Controller/MPC/MPC Scalar Signal Check1'
 * '<S17>'  : 'ModelPredictiveController/MPC Controller/MPC/MPC Scalar Signal Check2'
 * '<S18>'  : 'ModelPredictiveController/MPC Controller/MPC/MPC Vector Signal Check'
 * '<S19>'  : 'ModelPredictiveController/MPC Controller/MPC/MPC Vector Signal Check1'
 * '<S20>'  : 'ModelPredictiveController/MPC Controller/MPC/MPC Vector Signal Check6'
 * '<S21>'  : 'ModelPredictiveController/MPC Controller/MPC/moorx'
 * '<S22>'  : 'ModelPredictiveController/MPC Controller/MPC/optimizer'
 * '<S23>'  : 'ModelPredictiveController/MPC Controller/MPC/optimizer/optimizer'
 */
#endif                                 /* ModelPredictiveController_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
