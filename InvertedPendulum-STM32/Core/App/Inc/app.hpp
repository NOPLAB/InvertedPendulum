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
  IAdcInterruptHandler *adcInterruptHandlers[1];
  ITimerInterruptHandler *timerInterruptHandlers[1];

  Interval *interval_caller = nullptr;
  Adc *mux = nullptr;
};

#endif /* APP_INC_APP_HPP_ */
