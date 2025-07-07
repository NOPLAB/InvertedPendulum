/*
 * app.cpp
 *
 *  Created on: Jun 30, 2025
 *      Author: nop
 */

#include "app.hpp"
#include "adc.hpp"
#include "feedback_controller.h"
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

    lpf_theta->reset();
    lpf_current->reset();

    current_observer_left->reset();
    current_observer_right->reset();

    pid_current->reset();
    pid_speed->reset();

    start_control = true;
  }

  // printf("L %d, R %d\n", encoderLeftValue, encoderRightValue);
  // printf("L %f, R %f\n", adc2->getCorrectedValues()->currentL,

  controller->initialize();

  HAL_Delay(100);
}

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

  int current_encoder_l = encoder_val_l - QEI_GetPulses(&encoderLeft);
  QEI_Reset(&encoderLeft);
  int current_encoder_r = encoder_val_r + QEI_GetPulses(&encoderRight);
  QEI_Reset(&encoderRight);

  encoder_val_l = current_encoder_l;
  encoder_val_r = current_encoder_r;

  // 車輪の位置
  float real_x_r = (float)encoder_val_r * PULSE_TO_POSITION;
  float real_x_l = (float)encoder_val_l * PULSE_TO_POSITION;

  float x = (real_x_r + real_x_l) / 2.0f;
  float dx = (x - prev_x) / DT;
  prev_x = x;

  // モーター軸の角速度を計算 [rad/s]（オブザーバー用）
  // エンコーダーはモーター軸に直接取り付けられているため、直接変換
  float motor_speed_left =
      (float)(current_encoder_l - prev_encoder_l) * PULSE_TO_RAD / DT;
  float motor_speed_right =
      (float)(current_encoder_r - prev_encoder_r) * PULSE_TO_RAD / DT;

  prev_encoder_l = current_encoder_l;
  prev_encoder_r = current_encoder_r;

  // printf(">x:%f\n", x);

  float theta = -(float)(adc1_values->p_1 - zero_ad) * ADV_TO_RAD;
  float theta_filtered = lpf_theta->update(theta);
  float dtheta = (theta_filtered - prev_theta) / DT;
  prev_theta = theta_filtered;

  // printf(">theta:%f\n", theta_filtered);

  // 電圧指令値を計算（PWM値から実際の電圧に変換）
  float vin = adc1_values->mux_value[4] * 3.3 * ADC_TO_VOLTAGE;

  // printf(">vin:%f\n", vin);

  float voltage_left = current_pwm_left * vin;
  float voltage_right = current_pwm_right * vin;

  // 測定電流値を取得
  float measured_current_l =
      convertAdcToCurrent(adc2_values->currentL - offset_current_l);
  float measured_current_r =
      convertAdcToCurrent(adc2_values->currentR - offset_current_r);

  // printf(">measured_l:%f\n", measured_current_l);
  // printf(">measured_r:%f\n", measured_current_r);

  // オブザーバーを使用して正しい電流値を取得
  float corrected_current_l = current_observer_left->getCorrectedCurrent(
      measured_current_l, voltage_left, motor_speed_left);
  float corrected_current_r = current_observer_right->getCorrectedCurrent(
      measured_current_r, voltage_right, motor_speed_right);

  // printf(">corrected_l:%f\n", corrected_current_l);
  // printf(">corrected_r:%f\n", corrected_current_r);

  float current = (corrected_current_l + corrected_current_r) / 2.0f;
  float current_filtered = lpf_current->update(current);

  // printf(">current_avg:%f\n", current);
  // printf(">c:%f\n", current_filtered);

  // [-3.1623, -8.4042, -58.4769, -11.7355]
  float state_feedback_u =
      x * -3.1623f + dx * -8.4042f + theta * -58.4769f + dtheta * -11.7355f;

  // 力から直接電流指令に変換
  float force_to_current =
      -state_feedback_u * (WHEEL_RADIUS / (GEAR_RATIO * Kt));

  // printf(">force_to_current:%f\n", force_to_current);

  float u = pid_current->update(force_to_current, current_filtered) / vin;

  // printf(">u:%f\n", u);

  motors->setSpeedLeft(u);
  motors->setSpeedRight(u);

  // PWM指令値を保存（次回のオブザーバーで使用）
  current_pwm_left = u;
  current_pwm_right = u;

  prev_u = u;

  // ADC1, 2の値のDMA読み取りを実施
}
