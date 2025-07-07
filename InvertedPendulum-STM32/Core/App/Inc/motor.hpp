#pragma once

#include "tim.h"

#define PRESCALER (3199 - 1) // 20kHz

class Motors {
public:
  Motors() {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  }

  void setSpeedLeft(const float v) {
    float s = clamp(v, -1.0f, 1.0f);

    if (s > 0) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, s * PRESCALER);
    } else if (s < 0) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -s * PRESCALER);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    } else {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    }
  }

  void setSpeedRight(const float v) {
    float s = clamp(v, -1.0f, 1.0f);

    if (s > 0) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, s * PRESCALER);
    } else if (s < 0) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -s * PRESCALER);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    }
  }

private:
  float clamp(float value, float min_val, float max_val) {
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
  }
};