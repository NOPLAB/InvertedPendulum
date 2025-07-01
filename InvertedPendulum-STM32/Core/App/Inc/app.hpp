/*
 * app.hpp
 *
 *  Created on: Jun 30, 2025
 *      Author: nopla
 */

#ifndef APP_INC_APP_HPP_
#define APP_INC_APP_HPP_

#include "device.hpp"
#include "interrupt.hpp"
#include "mux.hpp"

class AppState {
public:
  InterruptHandler *interruptHandler;
  IAdcInterruptHandler *adcInterruptHandlers[1];

  IDevice *devices[1];

  Mux *mux;

public:
  AppState() {
    mux = new Mux();
    devices[0] = mux;
    adcInterruptHandlers[0] = mux;
    interruptHandler = new InterruptHandler(adcInterruptHandlers, 1);
  };
};

class App {
public:
  // アプリケーションが持つステート
  AppState *state = new AppState();

public:
  App() {}

  static App &getInstance() {
    static App instance;
    return instance;
  }

  // アプリケーションを実行
  int run();

private:
  // 初期化
  void initialize();

  // メインループ処理
  void loop();

  // 一定時間ごとに呼ばれる
  void interval();
};

#endif /* APP_INC_APP_HPP_ */
