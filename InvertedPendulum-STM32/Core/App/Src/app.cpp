/*
 * app.cpp
 *
 *  Created on: Jun 30, 2025
 *      Author: nop
 */

#include "app.hpp"
#include "mux.hpp"

int App::run() {
  this->initialize();

  while (true) {
    this->loop();
  }

  return 0;
}

void App::initialize() {
  this->mux = new Mux();
  this->interval_caller = new Interval([]() { App::getInstance().interval(); });

  this->adcInterruptHandlers[0] = this->mux;
  this->timerInterruptHandlers[0] = this->interval_caller;

  this->interruptHandler->registerAdc(this->adcInterruptHandlers, 1);
  this->interruptHandler->registerTimer(this->timerInterruptHandlers, 1);
}

void App::loop() {}

void App::interval() {
  MuxCorrectedValues values;
  App::getInstance().mux->getCorrectedValues(&values);
}
