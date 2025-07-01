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
    float s = v;
    s = s > 1.0 ? 1.0 : s;
    s = s < -1.0 ? -1.0 : s;

    if (s > 0) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, s * PRESCALER);
    } else if (s < 0) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, -s * PRESCALER);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
  }

  void setSpeedRight(const float v) {
    float s = v;
    s = s > 1.0 ? 1.0 : s;
    s = s < -1.0 ? -1.0 : s;

    if (s > 0) {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, s * PRESCALER);
    } else if (s < 0) {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -s * PRESCALER);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    } else {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
  }
};