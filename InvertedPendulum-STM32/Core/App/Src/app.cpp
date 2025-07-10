/*
 * app.cpp
 *
 *  Created on: Jun 30, 2025
 *      Author: nop
 */

#include "app.hpp"

#include <stdio.h>
// #include <string.h>

#include "adc.hpp"
#include "main.h"
#include "qei.h"
#include "stm32f3xx_hal_gpio.h"
// #include "stm32f3xx_hal_uart.h"
// #include "usart.h"

// uint8_t uart_tx[10] = {0};
// uint8_t uart_rx[4] = {0};
// float uart_u = 0.0f;

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
  this->intervalCaller =
      new Interval([]() { App::getInstance().interval(); }, 10000);  // 10kHz
  // this->intervalCaller_1khz =
  // new Interval([]() { App::getInstance().interval_1khz(); }, 1000);  // 1kHz
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
  // this->timerInterruptHandlers[1] = this->intervalCaller_1khz;

  this->interruptHandler->registerAdc(this->adcInterruptHandlers,
                                      ADC_INTERRUPT_HANDLERS_NUM);
  this->interruptHandler->registerTimer(this->timerInterruptHandlers,
                                        TIMER_INTERRUPT_HANDLERS_NUM);

  // HAL_UART_Receive_DMA(&huart2, uart_rx, 4);

  this->initialized = true;
}

void App::loop() {
  // HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

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
    // pid_speed->reset();

    // [-2.0000, -5.6814, -22.4525, -2.7583]
    adaptive_controller->setInitialGains(-2.0000f, -5.6814f, -22.4525f,
                                         -2.7583f);
    adaptive_controller->setReferenceModel(10.0f, 0.7f);
    adaptive_controller->reset();

    start_control = true;
  }

  HAL_Delay(100);
}

void App::interval() {
  adc1->scanAdcValues();
  adc2->scanAdcValues();

  if (!start_control) return;

  Adc1CorrectedValues *adc1_values = adc1->getCorrectedValues();

  float vin = adc1_values->mux_value[4] * 3.3 * ADC_TO_VOLTAGE;

  float theta = -(float)(adc1_values->p_1 - zero_ad) * ADV_TO_RAD;
  float theta_filtered = lpf_theta->update(theta);
  float dtheta = (theta_filtered - prev_theta) / DT;
  prev_theta = theta_filtered;

  // ADC2の値取得
  Adc2CorrectedValues *adc2_values;
  adc2_values = adc2->getCorrectedValues();

  int now_encoder_l = encoder_val_l - QEI_GetPulses(&encoderLeft);
  QEI_Reset(&encoderLeft);
  int now_encoder_r = encoder_val_r + QEI_GetPulses(&encoderRight);
  QEI_Reset(&encoderRight);

  encoder_val_l = now_encoder_l;
  encoder_val_r = now_encoder_r;

  float real_x_r = (float)encoder_val_r * PULSE_TO_POSITION;
  float real_x_l = (float)encoder_val_l * PULSE_TO_POSITION;

  float x = (real_x_r + real_x_l) / 2.0f;
  float dx = (x - prev_x) / DT;
  prev_x = x;

  // モーター軸の角速度を計算 [rad/s]（オブザーバー用）
  // エンコーダーはモーター軸に直接取り付けられているため、直接変換
  float motor_speed_left =
      (float)(encoder_val_l - prev_encoder_l) * PULSE_TO_RAD / DT;
  float motor_speed_right =
      (float)(encoder_val_r - prev_encoder_r) * PULSE_TO_RAD / DT;

  float voltage_left = prev_pwm_left * vin;
  float voltage_right = prev_pwm_right * vin;

  // 測定電流値を取得
  float measured_current_l =
      convertAdcToCurrent(adc2_values->currentL - offset_current_l);
  float measured_current_r =
      convertAdcToCurrent(adc2_values->currentR - offset_current_r);

  // オブザーバーを使用して正しい電流値を取得
  float corrected_current_l = current_observer_left->getCorrectedCurrent(
      measured_current_l, voltage_left, motor_speed_left);
  float corrected_current_r = current_observer_right->getCorrectedCurrent(
      measured_current_r, voltage_right, motor_speed_right);

  float current = (corrected_current_l + corrected_current_r) / 2.0f;
  float current_filtered = lpf_current->update(current);

  float reference_theta = 0.0f;  // Target pendulum angle (upright position)
  float control_output = adaptive_controller->update(x, dx, theta_filtered,
                                                     dtheta, reference_theta);

  // TODO
  // uart_x = x;
  // uart_theta = theta_filtered;
  // control_output = uart_u;

  float force_to_current =
      control_output * (WHEEL_RADIUS / (GEAR_RATIO * Kt * 2.0f));
  float u = pid_current->update(force_to_current, current_filtered) / vin;

  u = u * vin > 12.0f ? 12.0f / vin : u;
  u = u * vin < -12.0f ? -12.0f / vin : u;

  if (vin < 6.0f) {
    // u = 0.0f;  // If voltage is too low, stop the motors
    // printf("Low voltage: %f V, stopping motors.\n", vin);
  }

  motors->setSpeedLeft(u);
  motors->setSpeedRight(u);

  // motors->setSpeedLeft(1.0f);
  // motors->setSpeedRight(1.0f);

  prev_pwm_left = u;
  prev_pwm_right = u;
  prev_u = u;

  // printf(">l:%d\n", encoder_val_l);
  // printf(">r:%d\n", encoder_val_r);
  // printf(">x:%f\n", x);
  // printf(">theta:%f\n", theta_filtered);
  // printf(">vin:%f\n", vin);
  // printf(">measured_l:%f\n", measured_current_l);
  // printf(">measured_r:%f\n", measured_current_r);
  // printf(">voltage_l:%f\n", voltage_left);
  // printf(">voltage_r:%f\n", voltage_right);
  // printf(">current_avg:%f\n", current);
  // printf(">current_filtered:%f\n", current_filtered);
  // printf(">u:%f\n", u);
}

/*
void App::interval_1khz() {
   if (!start_control) return;

   if (huart2.ErrorCode != HAL_UART_ERROR_NONE) {
     // UARTエラーが発生した場合、エラーコードをリセット
     // huart2.ErrorCode = HAL_UART_ERROR_NONE;
     // huart2.gState = HAL_UART_STATE_READY;
     // huart2.RxState = HAL_UART_STATE_READY;
     // HAL_UART_Receive_DMA(&huart2, uart_rx, 4);  // 再度受信を開始
   }

   if (huart2.gState == HAL_UART_STATE_READY) {
     float tx_data[2] = {uart_x, uart_theta};
     uart_tx[0] = 0x90;
     uart_tx[1] = 0x86;
     for (int i = 0; i < 8; i++) {
       uart_tx[i + 2] = ((uint8_t *)(&tx_data))[i];
     }
     HAL_UART_Transmit_DMA(&huart2, uart_tx, 10);
   }
 }

 void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
   if (huart->Instance == USART2) {
     huart2.gState = HAL_UART_STATE_READY;
   }
 }

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
   HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
   if (huart->Instance == USART2) {
     uart_u = *((float *)uart_rx);

     HAL_UART_Receive_DMA(&huart2, uart_rx, 4);
   }
 }
*/
