//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: feedback_controller.h
//
// Code generated for Simulink model 'feedback_controller'.
//
// Model version                  : 1.350
// Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
// C/C++ source code generated on : Sun Jul  6 10:30:11 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef feedback_controller_h_
#define feedback_controller_h_
#include <cmath>
#include "rtwtypes.h"

// Class declaration for model feedback_controller
class feedback_controller final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW {
    real_T Probe[2];                   // '<S14>/Probe'
    real_T Probe_n[2];                 // '<S7>/Probe'
    real_T Integrator_DSTATE;          // '<S20>/Integrator'
    real_T UD_DSTATE;                  // '<S1>/UD'
    real_T Integrator_DSTATE_m;        // '<S13>/Integrator'
    real_T UD_DSTATE_e;                // '<S2>/UD'
    real_T Integrator_DSTATE_h;        // '<S55>/Integrator'
    real_T Integrator_DSTATE_hi;       // '<S107>/Integrator'
    int8_T Integrator_PrevResetState;  // '<S20>/Integrator'
    int8_T Integrator_PrevResetState_c;// '<S13>/Integrator'
    uint8_T Integrator_IC_LOADING;     // '<S20>/Integrator'
    uint8_T Integrator_IC_LOADING_b;   // '<S13>/Integrator'
  };

  // Copy Constructor
  feedback_controller(feedback_controller const&) = delete;

  // Assignment Operator
  feedback_controller& operator= (feedback_controller const&) & = delete;

  // Move Constructor
  feedback_controller(feedback_controller &&) = delete;

  // Move Assignment Operator
  feedback_controller& operator= (feedback_controller &&) = delete;

  // Block states get method
  const DW &getDWork() const
  {
    return rtDW;
  }

  // Block states set method
  void setDWork(const DW *pDW)
  {
    rtDW = *pDW;
  }

  // model initialize function
  void initialize();

  // model step function
  real_T step(real_T arg_x, real_T arg_theta, real_T arg_i);

  // Constructor
  feedback_controller();

  // Destructor
  ~feedback_controller();

  // private data and function members
 private:
  // Block states
  DW rtDW;
};

extern "C"
{
  extern real_T rtInf;
  extern real_T rtMinusInf;
  extern real_T rtNaN;
  extern real32_T rtInfF;
  extern real32_T rtMinusInfF;
  extern real32_T rtNaNF;
  static boolean_T rtIsInf(real_T value);
  static boolean_T rtIsInfF(real32_T value);
  static boolean_T rtIsNaN(real_T value);
  static boolean_T rtIsNaNF(real32_T value);
}                                      // extern "C"

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S13>/Saturation' : Eliminated Saturate block
//  Block '<S3>/K' : Eliminated nontunable gain of 1
//  Block '<S20>/Saturation' : Eliminated Saturate block
//  Block '<S4>/K' : Eliminated nontunable gain of 1


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'feedback_controller'
//  '<S1>'   : 'feedback_controller/Difference'
//  '<S2>'   : 'feedback_controller/Difference1'
//  '<S3>'   : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)'
//  '<S4>'   : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)1'
//  '<S5>'   : 'feedback_controller/PID Controller'
//  '<S6>'   : 'feedback_controller/PID Controller1'
//  '<S7>'   : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant'
//  '<S8>'   : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)/Initialization'
//  '<S9>'   : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)/Integrator (Discrete or Continuous)'
//  '<S10>'  : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant/Compare To Constant'
//  '<S11>'  : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)/Enable//disable time constant/Compare To Zero'
//  '<S12>'  : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)/Initialization/Init_u'
//  '<S13>'  : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)/Integrator (Discrete or Continuous)/Discrete'
//  '<S14>'  : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant'
//  '<S15>'  : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)1/Initialization'
//  '<S16>'  : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)1/Integrator (Discrete or Continuous)'
//  '<S17>'  : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant/Compare To Constant'
//  '<S18>'  : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)1/Enable//disable time constant/Compare To Zero'
//  '<S19>'  : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)1/Initialization/Init_u'
//  '<S20>'  : 'feedback_controller/Low-Pass Filter (Discrete or Continuous)1/Integrator (Discrete or Continuous)/Discrete'
//  '<S21>'  : 'feedback_controller/PID Controller/Anti-windup'
//  '<S22>'  : 'feedback_controller/PID Controller/D Gain'
//  '<S23>'  : 'feedback_controller/PID Controller/External Derivative'
//  '<S24>'  : 'feedback_controller/PID Controller/Filter'
//  '<S25>'  : 'feedback_controller/PID Controller/Filter ICs'
//  '<S26>'  : 'feedback_controller/PID Controller/I Gain'
//  '<S27>'  : 'feedback_controller/PID Controller/Ideal P Gain'
//  '<S28>'  : 'feedback_controller/PID Controller/Ideal P Gain Fdbk'
//  '<S29>'  : 'feedback_controller/PID Controller/Integrator'
//  '<S30>'  : 'feedback_controller/PID Controller/Integrator ICs'
//  '<S31>'  : 'feedback_controller/PID Controller/N Copy'
//  '<S32>'  : 'feedback_controller/PID Controller/N Gain'
//  '<S33>'  : 'feedback_controller/PID Controller/P Copy'
//  '<S34>'  : 'feedback_controller/PID Controller/Parallel P Gain'
//  '<S35>'  : 'feedback_controller/PID Controller/Reset Signal'
//  '<S36>'  : 'feedback_controller/PID Controller/Saturation'
//  '<S37>'  : 'feedback_controller/PID Controller/Saturation Fdbk'
//  '<S38>'  : 'feedback_controller/PID Controller/Sum'
//  '<S39>'  : 'feedback_controller/PID Controller/Sum Fdbk'
//  '<S40>'  : 'feedback_controller/PID Controller/Tracking Mode'
//  '<S41>'  : 'feedback_controller/PID Controller/Tracking Mode Sum'
//  '<S42>'  : 'feedback_controller/PID Controller/Tsamp - Integral'
//  '<S43>'  : 'feedback_controller/PID Controller/Tsamp - Ngain'
//  '<S44>'  : 'feedback_controller/PID Controller/postSat Signal'
//  '<S45>'  : 'feedback_controller/PID Controller/preInt Signal'
//  '<S46>'  : 'feedback_controller/PID Controller/preSat Signal'
//  '<S47>'  : 'feedback_controller/PID Controller/Anti-windup/Passthrough'
//  '<S48>'  : 'feedback_controller/PID Controller/D Gain/Disabled'
//  '<S49>'  : 'feedback_controller/PID Controller/External Derivative/Disabled'
//  '<S50>'  : 'feedback_controller/PID Controller/Filter/Disabled'
//  '<S51>'  : 'feedback_controller/PID Controller/Filter ICs/Disabled'
//  '<S52>'  : 'feedback_controller/PID Controller/I Gain/Internal Parameters'
//  '<S53>'  : 'feedback_controller/PID Controller/Ideal P Gain/Passthrough'
//  '<S54>'  : 'feedback_controller/PID Controller/Ideal P Gain Fdbk/Disabled'
//  '<S55>'  : 'feedback_controller/PID Controller/Integrator/Discrete'
//  '<S56>'  : 'feedback_controller/PID Controller/Integrator ICs/Internal IC'
//  '<S57>'  : 'feedback_controller/PID Controller/N Copy/Disabled wSignal Specification'
//  '<S58>'  : 'feedback_controller/PID Controller/N Gain/Disabled'
//  '<S59>'  : 'feedback_controller/PID Controller/P Copy/Disabled'
//  '<S60>'  : 'feedback_controller/PID Controller/Parallel P Gain/Internal Parameters'
//  '<S61>'  : 'feedback_controller/PID Controller/Reset Signal/Disabled'
//  '<S62>'  : 'feedback_controller/PID Controller/Saturation/Passthrough'
//  '<S63>'  : 'feedback_controller/PID Controller/Saturation Fdbk/Disabled'
//  '<S64>'  : 'feedback_controller/PID Controller/Sum/Sum_PI'
//  '<S65>'  : 'feedback_controller/PID Controller/Sum Fdbk/Disabled'
//  '<S66>'  : 'feedback_controller/PID Controller/Tracking Mode/Disabled'
//  '<S67>'  : 'feedback_controller/PID Controller/Tracking Mode Sum/Passthrough'
//  '<S68>'  : 'feedback_controller/PID Controller/Tsamp - Integral/TsSignalSpecification'
//  '<S69>'  : 'feedback_controller/PID Controller/Tsamp - Ngain/Passthrough'
//  '<S70>'  : 'feedback_controller/PID Controller/postSat Signal/Forward_Path'
//  '<S71>'  : 'feedback_controller/PID Controller/preInt Signal/Internal PreInt'
//  '<S72>'  : 'feedback_controller/PID Controller/preSat Signal/Forward_Path'
//  '<S73>'  : 'feedback_controller/PID Controller1/Anti-windup'
//  '<S74>'  : 'feedback_controller/PID Controller1/D Gain'
//  '<S75>'  : 'feedback_controller/PID Controller1/External Derivative'
//  '<S76>'  : 'feedback_controller/PID Controller1/Filter'
//  '<S77>'  : 'feedback_controller/PID Controller1/Filter ICs'
//  '<S78>'  : 'feedback_controller/PID Controller1/I Gain'
//  '<S79>'  : 'feedback_controller/PID Controller1/Ideal P Gain'
//  '<S80>'  : 'feedback_controller/PID Controller1/Ideal P Gain Fdbk'
//  '<S81>'  : 'feedback_controller/PID Controller1/Integrator'
//  '<S82>'  : 'feedback_controller/PID Controller1/Integrator ICs'
//  '<S83>'  : 'feedback_controller/PID Controller1/N Copy'
//  '<S84>'  : 'feedback_controller/PID Controller1/N Gain'
//  '<S85>'  : 'feedback_controller/PID Controller1/P Copy'
//  '<S86>'  : 'feedback_controller/PID Controller1/Parallel P Gain'
//  '<S87>'  : 'feedback_controller/PID Controller1/Reset Signal'
//  '<S88>'  : 'feedback_controller/PID Controller1/Saturation'
//  '<S89>'  : 'feedback_controller/PID Controller1/Saturation Fdbk'
//  '<S90>'  : 'feedback_controller/PID Controller1/Sum'
//  '<S91>'  : 'feedback_controller/PID Controller1/Sum Fdbk'
//  '<S92>'  : 'feedback_controller/PID Controller1/Tracking Mode'
//  '<S93>'  : 'feedback_controller/PID Controller1/Tracking Mode Sum'
//  '<S94>'  : 'feedback_controller/PID Controller1/Tsamp - Integral'
//  '<S95>'  : 'feedback_controller/PID Controller1/Tsamp - Ngain'
//  '<S96>'  : 'feedback_controller/PID Controller1/postSat Signal'
//  '<S97>'  : 'feedback_controller/PID Controller1/preInt Signal'
//  '<S98>'  : 'feedback_controller/PID Controller1/preSat Signal'
//  '<S99>'  : 'feedback_controller/PID Controller1/Anti-windup/Passthrough'
//  '<S100>' : 'feedback_controller/PID Controller1/D Gain/Disabled'
//  '<S101>' : 'feedback_controller/PID Controller1/External Derivative/Disabled'
//  '<S102>' : 'feedback_controller/PID Controller1/Filter/Disabled'
//  '<S103>' : 'feedback_controller/PID Controller1/Filter ICs/Disabled'
//  '<S104>' : 'feedback_controller/PID Controller1/I Gain/Internal Parameters'
//  '<S105>' : 'feedback_controller/PID Controller1/Ideal P Gain/Passthrough'
//  '<S106>' : 'feedback_controller/PID Controller1/Ideal P Gain Fdbk/Disabled'
//  '<S107>' : 'feedback_controller/PID Controller1/Integrator/Discrete'
//  '<S108>' : 'feedback_controller/PID Controller1/Integrator ICs/Internal IC'
//  '<S109>' : 'feedback_controller/PID Controller1/N Copy/Disabled wSignal Specification'
//  '<S110>' : 'feedback_controller/PID Controller1/N Gain/Disabled'
//  '<S111>' : 'feedback_controller/PID Controller1/P Copy/Disabled'
//  '<S112>' : 'feedback_controller/PID Controller1/Parallel P Gain/Internal Parameters'
//  '<S113>' : 'feedback_controller/PID Controller1/Reset Signal/Disabled'
//  '<S114>' : 'feedback_controller/PID Controller1/Saturation/Passthrough'
//  '<S115>' : 'feedback_controller/PID Controller1/Saturation Fdbk/Disabled'
//  '<S116>' : 'feedback_controller/PID Controller1/Sum/Sum_PI'
//  '<S117>' : 'feedback_controller/PID Controller1/Sum Fdbk/Disabled'
//  '<S118>' : 'feedback_controller/PID Controller1/Tracking Mode/Disabled'
//  '<S119>' : 'feedback_controller/PID Controller1/Tracking Mode Sum/Passthrough'
//  '<S120>' : 'feedback_controller/PID Controller1/Tsamp - Integral/TsSignalSpecification'
//  '<S121>' : 'feedback_controller/PID Controller1/Tsamp - Ngain/Passthrough'
//  '<S122>' : 'feedback_controller/PID Controller1/postSat Signal/Forward_Path'
//  '<S123>' : 'feedback_controller/PID Controller1/preInt Signal/Internal PreInt'
//  '<S124>' : 'feedback_controller/PID Controller1/preSat Signal/Forward_Path'

#endif                                 // feedback_controller_h_

//
// File trailer for generated code.
//
// [EOF]
//
