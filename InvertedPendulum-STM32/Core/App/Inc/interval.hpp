#pragma once

#include "interrupt.hpp"
#include "tim.h"

class Interval : public ITimerInterruptHandler {
private:
  void (*callback)();

public:
  Interval(void (*callback)()) { this->callback = callback; }

public: // ITimerInterruptHandler
  TIM_HandleTypeDef *timerHandlerType() const override {
    return &htim6; // TIM6
  }

  void handleTimerInterrupt() override {
    if (this->callback) {
      this->callback();
    }
  }
};