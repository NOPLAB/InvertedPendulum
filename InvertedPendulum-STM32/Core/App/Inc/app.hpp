/*
 * app.hpp
 *
 *  Created on: Jun 30, 2025
 *      Author: nop
 */

#ifndef APP_INC_APP_HPP_
#define APP_INC_APP_HPP_

#include "adc.hpp"
#include "interrupt.hpp"
#include "interval.hpp"
#include "motor.hpp"

extern "C" {
#include "qei.h"
}

#define ADC_INTERRUPT_HANDLERS_NUM 2
#define TIMER_INTERRUPT_HANDLERS_NUM 1

class App {
public:
  bool initialized = false;

public:
  App() {}

  static App &getInstance() {
    static App instance;
    return instance;
  }

  static App *getInstanceRef() { return &getInstance(); }

  // アプリケーションを実行
  int run();

  // 初期化
  void initialize();

  // メインループ処理
  void loop();

  // 一定時間ごとに呼ばれる
  void interval();

public:
  InterruptHandler *getInterruptHandler() { return this->interruptHandler; }

private:
  InterruptHandler *interruptHandler = new InterruptHandler();
  IAdcInterruptHandler *adcInterruptHandlers[ADC_INTERRUPT_HANDLERS_NUM];
  ITimerInterruptHandler *timerInterruptHandlers[TIMER_INTERRUPT_HANDLERS_NUM];

  Interval *intervalCaller = nullptr;

  Adc1 *adc1 = nullptr;
  Adc2 *adc2 = nullptr;

  Motors *motors = nullptr;

private:
  int encoderLeftValue = 0;
  int encoderRightValue = 0;

  bool start_control = false;

  float zero_ad = 0.0f;

  float offset_current_l = 0.0;
  float offset_current_r = 0.0;

public:
  QEI_HandleTypeDef encoderLeft;
  QEI_HandleTypeDef encoderRight;
};

#endif /* APP_INC_APP_HPP_ */
