#include "lpf.hpp"

LowPassFilter::LowPassFilter(float gain, float time_constant, float dt)
    : gain_(gain),
      time_constant_(time_constant),
      dt_(dt),
      output_(0.0f),
      first_call_(true) {
  updateAlpha();
}

void LowPassFilter::setGain(float gain) { gain_ = gain; }

void LowPassFilter::setTimeConstant(float time_constant) {
  time_constant_ = time_constant;
  updateAlpha();
}

void LowPassFilter::setDt(float dt) {
  dt_ = dt;
  updateAlpha();
}

void LowPassFilter::reset() {
  output_ = 0.0f;
  first_call_ = true;
}

float LowPassFilter::update(float input) {
  if (first_call_) {
    output_ = gain_ * input;
    first_call_ = false;
  } else {
    output_ = alpha_ * gain_ * input + (1.0f - alpha_) * output_;
  }

  return output_;
}

void LowPassFilter::updateAlpha() {
  if (time_constant_ > 0.0f) {
    alpha_ = dt_ / (time_constant_ + dt_);
  } else {
    alpha_ = 1.0f;
  }
}