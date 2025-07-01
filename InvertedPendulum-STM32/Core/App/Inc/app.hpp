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

#define ADC_INTERRUPT_HANDLERS_NUM 2
#define TIMER_INTERRUPT_HANDLERS_NUM 1

class App {
public:
  App() {}

  static App &getInstance() {
    static App instance;
    return instance;
  }

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

  Interval *interval_caller = nullptr;

  Adc1 *adc1 = nullptr;
  Adc2 *adc2 = nullptr;

  Motors *motors = nullptr;
};

#endif /* APP_INC_APP_HPP_ */
