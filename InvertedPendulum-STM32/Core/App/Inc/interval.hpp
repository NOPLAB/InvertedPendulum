#pragma once

#include "interrupt.hpp"
#include "tim.h"

class Interval : public ITimerInterruptHandler {
 private:
  void (*callback)() = nullptr;
  uint32_t base_frequency_hz;    // ベースタイマーの周波数 (10kHz)
  uint32_t target_frequency_hz;  // 目標実行周波数
  uint32_t counter;              // 内部カウンター
  uint32_t divider;              // 分周比

 public:
  Interval(void (*callback)(), uint32_t frequency_hz) {
    this->callback = callback;
    this->base_frequency_hz = 10000;  // 10kHz固定
    this->target_frequency_hz = frequency_hz;
    this->counter = 0;
    this->divider = this->base_frequency_hz / this->target_frequency_hz;

    // 分周比が0になることを防ぐ
    if (this->divider == 0) {
      this->divider = 1;
    }

    HAL_TIM_Base_Start_IT(&htim6);
  }

 public:  // ITimerInterruptHandler
  TIM_HandleTypeDef *timerHandlerType() const override { return &htim6; }

  void handleTimerInterrupt() override {
    this->counter++;

    // カウンターが分周比に達したら関数を実行
    if (this->counter >= this->divider) {
      this->counter = 0;
      if (this->callback != nullptr) {
        this->callback();
      }
    }
  }
};