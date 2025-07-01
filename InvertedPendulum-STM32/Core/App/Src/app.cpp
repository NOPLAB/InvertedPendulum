/*
 * app.cpp
 *
 *  Created on: Jun 30, 2025
 *      Author: nop
 */

#include "app.hpp"
#include "adc.hpp"
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
  this->interval_caller = new Interval([]() { App::getInstance().interval(); });
  this->adc1 = new Adc1();
  this->adc2 = new Adc2();
  this->motors = new Motors();

  this->adcInterruptHandlers[0] = this->adc1;
  this->adcInterruptHandlers[1] = this->adc2;
  this->timerInterruptHandlers[0] = this->interval_caller;

  this->interruptHandler->registerAdc(this->adcInterruptHandlers,
                                      ADC_INTERRUPT_HANDLERS_NUM);
  this->interruptHandler->registerTimer(this->timerInterruptHandlers,
                                        TIMER_INTERRUPT_HANDLERS_NUM);
}

void App::loop() {
  HAL_Delay(100); // 100msの遅延

  motors->setSpeedLeft(1.0);
  motors->setSpeedRight(1.0);
}

void App::interval() {
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

  // ADC1, 2の値取得

  Adc1CorrectedValues adc1_values;
  this->adc1->getCorrectedValues(&adc1_values);

  Adc2CorrectedValues adc2_values;
  this->adc2->getCorrectedValues(&adc2_values);

  // ADC1, 2の値のDMA読み取りを実施

  this->adc1->scanAdcValues();
  this->adc2->scanAdcValues();
}
