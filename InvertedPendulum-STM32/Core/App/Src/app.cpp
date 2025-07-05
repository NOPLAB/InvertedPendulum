/*
 * app.cpp
 *
 *  Created on: Jun 30, 2025
 *      Author: nop
 */

#include "app.hpp"
#include "adc.hpp"
#include "main.h"
#include "qei.h"
#include "stm32f3xx_hal_gpio.h"
#include <stdio.h>

#define AMPLIFICATION_FACTOR 150.0f
#define SHUNT_REGISTER 0.010f
float convertAdcToCurrent(float adcValue) {
  return adcValue * 3.3f / AMPLIFICATION_FACTOR / SHUNT_REGISTER;
}

int App::run() {
  this->initialize();

  while (true) {
    this->loop();
  }

  return 0;
}

void App::initialize() {
  this->intervalCaller = new Interval([]() { App::getInstance().interval(); });
  this->adc1 = new Adc1();
  this->adc2 = new Adc2();
  this->motors = new Motors();

  QEI_Init(&this->encoderLeft, QEI_L_A_GPIO_Port, QEI_L_A_Pin,
           QEI_L_B_GPIO_Port, QEI_L_B_Pin, 48, QEI_X4_ENCODING);
  QEI_Init(&this->encoderRight, QEI_R_A_GPIO_Port, QEI_R_A_Pin,
           QEI_R_B_GPIO_Port, QEI_R_B_Pin, 48, QEI_X4_ENCODING);

  this->adcInterruptHandlers[0] = this->adc1;
  this->adcInterruptHandlers[1] = this->adc2;
  this->timerInterruptHandlers[0] = this->intervalCaller;

  this->interruptHandler->registerAdc(this->adcInterruptHandlers,
                                      ADC_INTERRUPT_HANDLERS_NUM);
  this->interruptHandler->registerTimer(this->timerInterruptHandlers,
                                        TIMER_INTERRUPT_HANDLERS_NUM);

  this->initialized = true;
}

void App::loop() {
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

  if (!start_control &&
      HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_RESET) {
    zero_ad = adc1->getCorrectedValues()->p_1;

    offset_current_l = 0.0f;
    offset_current_r = 0.0f;
    for (int i = 0; i < 100; i++) {
      float current_l = adc2->getCorrectedValues()->currentL;
      float current_r = adc2->getCorrectedValues()->currentR;

      // 移動平均
      offset_current_l += current_l;
      offset_current_r += current_r;

      HAL_Delay(1);
    }
    offset_current_l /= 100.0f;
    offset_current_r /= 100.0f;

    // printf("offset current l: %f\n", offset_current_l);
    // printf("offset current r: %f\n", offset_current_r);

    start_control = true;
  }

  // printf("L %d, R %d\n", encoderLeftValue, encoderRightValue);
  // printf("L %f, R %f\n", adc2->getCorrectedValues()->currentL,

  HAL_Delay(100);
}

// ADC_TO_RAD = (333.3 * ((2.0*pi)/360.0)) / 4.85 * 3.3 / (2^12)
#define ADV_TO_RAD 0.00096633f

void App::interval() {
  adc1->scanAdcValues();
  adc2->scanAdcValues();

  if (!start_control)
    return;

  // ADC1, 2の値取得

  Adc1CorrectedValues *adc1_values;
  adc1_values = adc1->getCorrectedValues();

  Adc2CorrectedValues *adc2_values;
  adc2_values = adc2->getCorrectedValues();

  encoderLeftValue -= QEI_GetPulses(&encoderLeft);
  QEI_Reset(&encoderLeft);
  encoderRightValue += QEI_GetPulses(&encoderRight);
  QEI_Reset(&encoderRight);

  float theta = -(float)(adc1_values->p_1 - zero_ad) * ADV_TO_RAD;

  float real_current_l =
      convertAdcToCurrent(adc2_values->currentL - offset_current_l);
  float real_current_r =
      convertAdcToCurrent(adc2_values->currentR - offset_current_r);

  motors->setSpeedLeft(1.0);
  motors->setSpeedRight(1.0);

  // ADC1, 2の値のDMA読み取りを実施
}
