/*
 * app.cpp
 *
 *  Created on: Jun 30, 2025
 *      Author: nop
 */

#include "app.hpp"
#include "main.h"
#include "stm32f3xx_hal_gpio.h"

int App::run() {
  this->initialize();

  while (true) {
    this->loop();
  }

  return 0;
}

void App::initialize() {
  this->mux = new Adc();
  this->interval_caller = new Interval([]() { App::getInstance().interval(); });

  this->adcInterruptHandlers[0] = this->mux;
  this->timerInterruptHandlers[0] = this->interval_caller;

  this->interruptHandler->registerAdc(this->adcInterruptHandlers, 1);
  this->interruptHandler->registerTimer(this->timerInterruptHandlers, 1);
}

void App::loop() {
  HAL_Delay(100); // 100msの遅延
}

void App::interval() {
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  AdcCorrectedValues values;
  this->mux->getCorrectedValues(&values);
  this->mux->scanAdcValues();
}
