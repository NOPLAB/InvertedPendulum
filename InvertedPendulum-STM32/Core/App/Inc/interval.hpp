#pragma once

#include "interrupt.hpp"
#include "tim.h"

class Interval : public ITimerInterruptHandler {
 private:
  void (*callback)() = nullptr;

 public:
  Interval(void (*callback)()) {
    this->callback = callback;
    HAL_TIM_Base_Start_IT(&htim6);
  }

 public:  // ITimerInterruptHandler
  TIM_HandleTypeDef *timerHandlerType() const override { return &htim6; }

  void handleTimerInterrupt() override {
    if (this->callback != nullptr) {
      this->callback();
    }
  }
};