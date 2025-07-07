#include "pid.hpp"

PID::PID(float kp, float ki, float kd, float dt, float min_output,
         float max_output)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      dt_(dt),
      min_output_(min_output),
      max_output_(max_output),
      integral_term_(0.0f),
      derivative_term_(0.0f),
      prev_error_(0.0f),
      first_call_(true) {}

void PID::setGains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PID::setOutputLimits(float min_output, float max_output) {
  min_output_ = min_output;
  max_output_ = max_output;
}

void PID::setDt(float dt) { dt_ = dt; }

void PID::reset() {
  integral_term_ = 0.0f;
  derivative_term_ = 0.0f;
  prev_error_ = 0.0f;
  first_call_ = true;
}

float PID::update(float setpoint, float measurement) {
  float error = setpoint - measurement;

  // Proportional term
  float proportional_term = kp_ * error;

  // Integral term
  integral_term_ += ki_ * error * dt_;

  // Derivative term
  if (first_call_) {
    derivative_term_ = 0.0f;
    first_call_ = false;
  } else {
    derivative_term_ = kd_ * (error - prev_error_) / dt_;
  }

  // Calculate PID output
  float output = proportional_term + integral_term_ + derivative_term_;

  // Apply output limits
  output = clamp(output, min_output_, max_output_);

  // Anti-windup: prevent integral windup by clamping integral term
  if (output > max_output_ && integral_term_ > 0.0f) {
    integral_term_ = max_output_ - proportional_term - derivative_term_;
    if (integral_term_ < 0.0f) integral_term_ = 0.0f;
  } else if (output < min_output_ && integral_term_ < 0.0f) {
    integral_term_ = min_output_ - proportional_term - derivative_term_;
    if (integral_term_ > 0.0f) integral_term_ = 0.0f;
  }

  prev_error_ = error;

  return output;
}

float PID::clamp(float value, float min_val, float max_val) {
  if (value > max_val) return max_val;
  if (value < min_val) return min_val;
  return value;
}