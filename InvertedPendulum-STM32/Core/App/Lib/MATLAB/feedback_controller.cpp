//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: feedback_controller.cpp
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
#include "feedback_controller.h"
#include "rtwtypes.h"
#include <cmath>
#include "limits"
#include "cmath"

extern "C"
{
  real_T rtNaN { -std::numeric_limits<real_T>::quiet_NaN() };

  real_T rtInf { std::numeric_limits<real_T>::infinity() };

  real_T rtMinusInf { -std::numeric_limits<real_T>::infinity() };

  real32_T rtNaNF { -std::numeric_limits<real32_T>::quiet_NaN() };

  real32_T rtInfF { std::numeric_limits<real32_T>::infinity() };

  real32_T rtMinusInfF { -std::numeric_limits<real32_T>::infinity() };
}

// Model step function
real_T feedback_controller::step(real_T arg_x, real_T arg_theta, real_T arg_i)
{
  real_T rtb_Integrator;
  real_T rtb_Integrator_d4;
  real_T rtb_Max;
  real_T rtb_Sum_n;
  boolean_T rtb_LogicalOperator;
  boolean_T rtb_LogicalOperator_f;

  // specified return value
  real_T arg_y;

  // Logic: '<S14>/Logical Operator' incorporates:
  //   Constant: '<S14>/Time constant'
  //   Constant: '<S18>/Constant'
  //   RelationalOperator: '<S18>/Compare'
  //   Sum: '<S14>/Sum1'

  rtb_LogicalOperator = (0.0015915494309189533 - rtDW.Probe[0] <= 0.0);

  // DiscreteIntegrator: '<S20>/Integrator' incorporates:
  //   Inport: '<Root>/x'

  if (rtDW.Integrator_IC_LOADING != 0) {
    rtDW.Integrator_DSTATE = arg_x;
  }

  if (rtb_LogicalOperator || (rtDW.Integrator_PrevResetState != 0)) {
    rtDW.Integrator_DSTATE = arg_x;
  }

  rtb_Integrator = rtDW.Integrator_DSTATE;

  // Sum: '<S1>/Diff' incorporates:
  //   DiscreteIntegrator: '<S20>/Integrator'
  //   UnitDelay: '<S1>/UD'
  //
  //  Block description for '<S1>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S1>/UD':
  //
  //   Store in Global RAM

  rtb_Sum_n = rtDW.Integrator_DSTATE - rtDW.UD_DSTATE;

  // Logic: '<S7>/Logical Operator' incorporates:
  //   Constant: '<S11>/Constant'
  //   Constant: '<S7>/Time constant'
  //   RelationalOperator: '<S11>/Compare'
  //   Sum: '<S7>/Sum1'

  rtb_LogicalOperator_f = (0.00015915494309189535 - rtDW.Probe_n[0] <= 0.0);

  // DiscreteIntegrator: '<S13>/Integrator' incorporates:
  //   Inport: '<Root>/theta'

  if (rtDW.Integrator_IC_LOADING_b != 0) {
    rtDW.Integrator_DSTATE_m = arg_theta;
  }

  if (rtb_LogicalOperator_f || (rtDW.Integrator_PrevResetState_c != 0)) {
    rtDW.Integrator_DSTATE_m = arg_theta;
  }

  rtb_Integrator_d4 = rtDW.Integrator_DSTATE_m;

  // Sum: '<Root>/Sum' incorporates:
  //   DiscreteIntegrator: '<S13>/Integrator'
  //   DiscreteIntegrator: '<S20>/Integrator'
  //   Gain: '<Root>/Gain'
  //   Gain: '<Root>/Gain1'
  //   SignalConversion generated from: '<Root>/Gain'
  //   Sum: '<S2>/Diff'
  //   UnitDelay: '<S2>/UD'
  //
  //  Block description for '<S2>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S2>/UD':
  //
  //   Store in Global RAM

  rtb_Sum_n = (((3.1623 * rtDW.Integrator_DSTATE + 7.4739 * rtb_Sum_n) + 40.598 *
                rtDW.Integrator_DSTATE_m) + (rtDW.Integrator_DSTATE_m -
    rtDW.UD_DSTATE_e) * 8.5943) * 0.44891548943449222 - rtb_Sum_n;

  // Sum: '<Root>/Sum1' incorporates:
  //   DiscreteIntegrator: '<S55>/Integrator'
  //   Gain: '<Root>/Gain2'
  //   Gain: '<S60>/Proportional Gain'
  //   Inport: '<Root>/i'
  //   Sum: '<S64>/Sum'

  rtb_Max = (5.0654 * rtb_Sum_n + rtDW.Integrator_DSTATE_h) *
    0.10299899133693659 - arg_i;

  // Outport: '<Root>/y' incorporates:
  //   DiscreteIntegrator: '<S107>/Integrator'
  //   Gain: '<S112>/Proportional Gain'
  //   Sum: '<S116>/Sum'

  arg_y = 3.06 * rtb_Max + rtDW.Integrator_DSTATE_hi;

  // Update for DiscreteIntegrator: '<S20>/Integrator' incorporates:
  //   Constant: '<S14>/Time constant'
  //   Inport: '<Root>/x'
  //   MinMax: '<S14>/Max'
  //   Product: '<S4>/1//T'
  //   Sum: '<S4>/Sum1'

  rtDW.Integrator_IC_LOADING = 0U;
  rtDW.Integrator_DSTATE += 1.0 / std::fmax(rtDW.Probe[0], 0.0015915494309189533)
    * (arg_x - rtDW.Integrator_DSTATE) * 0.002;
  rtDW.Integrator_PrevResetState = static_cast<int8_T>(rtb_LogicalOperator);

  // Update for UnitDelay: '<S1>/UD'
  //
  //  Block description for '<S1>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE = rtb_Integrator;

  // Update for DiscreteIntegrator: '<S13>/Integrator' incorporates:
  //   Constant: '<S7>/Time constant'
  //   Inport: '<Root>/theta'
  //   MinMax: '<S7>/Max'
  //   Product: '<S3>/1//T'
  //   Sum: '<S3>/Sum1'

  rtDW.Integrator_IC_LOADING_b = 0U;
  rtDW.Integrator_DSTATE_m += 1.0 / std::fmax(rtDW.Probe_n[0],
    0.00015915494309189535) * (arg_theta - rtDW.Integrator_DSTATE_m) * 0.002;
  rtDW.Integrator_PrevResetState_c = static_cast<int8_T>(rtb_LogicalOperator_f);

  // Update for UnitDelay: '<S2>/UD'
  //
  //  Block description for '<S2>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE_e = rtb_Integrator_d4;

  // Update for DiscreteIntegrator: '<S55>/Integrator' incorporates:
  //   Gain: '<S52>/Integral Gain'

  rtDW.Integrator_DSTATE_h += 22.2759 * rtb_Sum_n * 0.002;

  // Update for DiscreteIntegrator: '<S107>/Integrator' incorporates:
  //   Gain: '<S104>/Integral Gain'

  rtDW.Integrator_DSTATE_hi += 106380.0 * rtb_Max * 0.002;
  return arg_y;
}

// Model initialize function
void feedback_controller::initialize()
{
  // Start for Probe: '<S14>/Probe'
  rtDW.Probe[0] = 0.002;
  rtDW.Probe[1] = 0.0;

  // Start for Probe: '<S7>/Probe'
  rtDW.Probe_n[0] = 0.002;
  rtDW.Probe_n[1] = 0.0;

  // InitializeConditions for DiscreteIntegrator: '<S20>/Integrator'
  rtDW.Integrator_IC_LOADING = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S13>/Integrator'
  rtDW.Integrator_IC_LOADING_b = 1U;
}

// Constructor
feedback_controller::feedback_controller():
  rtDW()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
feedback_controller::~feedback_controller() = default;
extern "C"
{
  // Test if value is infinite
  static boolean_T rtIsInf(real_T value)
  {
    return std::isinf(value);
  }

  // Test if single-precision value is infinite
  static boolean_T rtIsInfF(real32_T value)
  {
    return std::isinf(value);
  }

  // Test if value is not a number
  static boolean_T rtIsNaN(real_T value)
  {
    return std::isnan(value);
  }

  // Test if single-precision value is not a number
  static boolean_T rtIsNaNF(real32_T value)
  {
    return std::isnan(value);
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
