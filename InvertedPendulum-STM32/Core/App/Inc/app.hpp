/*
 * app.hpp
 *
 *  Created on: Jun 30, 2025
 *      Author: nop
 */

#ifndef APP_INC_APP_HPP_
#define APP_INC_APP_HPP_

#include "adc.hpp"
#include "feedback_controller.h"
#include "interrupt.hpp"
#include "interval.hpp"
#include "lpf.hpp"
#include "motor.hpp"
#include "motor_observer.hpp"
#include "pid.hpp"

extern "C" {
#include "qei.h"
}

#define ADC_INTERRUPT_HANDLERS_NUM 2
#define TIMER_INTERRUPT_HANDLERS_NUM 1

#define PI 3.14159265358979323846f
#define DT (1.0f / 10000.0f)
#define GEAR_RATIO 6.67f
#define WHEEL_RADIUS 0.0255f
#define PULSE_TO_RAD (2.0f * PI / (12.0f * 4.0f))
// ADC_TO_RAD = (333.3 * ((2.0*pi)/360.0)) / 5.0 * 3.3
#define ADV_TO_RAD 3.8393f
#define PULSE_TO_METER (2.0f * PI * WHEEL_RADIUS) / (12.0f * 4.0f * GEAR_RATIO)
#define ADC_TO_VOLTAGE ((2400.0f + 750.0f) / 750.0f)

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
  int encoder_val_l = 0;
  int encoder_val_r = 0;

  bool start_control = false;

  float zero_ad = 0.0f;

  float offset_current_l = 0.0;
  float offset_current_r = 0.0;

  float prev_theta = 0.0f;
  float prev_x = 0.0f;
  float prev_u = 0.0f;

  int prev_encoder_l = 0;
  int prev_encoder_r = 0;

  float current_pwm_left = 0.0f;
  float current_pwm_right = 0.0f;

  LowPassFilter *lpf_current = new LowPassFilter(1.0f, 1 / (2 * PI * 500), DT);

  PID *pid_current = new PID(0.928f, 10178.8f, 0.0f, DT, -12.0f, 12.0f);

  MotorCurrentObserver *current_observer_left =
      new MotorCurrentObserver(0.0186f, 0.0186f, 0.003f, 32.4f, DT);
  MotorCurrentObserver *current_observer_right =
      new MotorCurrentObserver(0.0186f, 0.0186f, 0.003f, 32.4f, DT);

  feedback_controller *controller = new feedback_controller();

public:
  QEI_HandleTypeDef encoderLeft;
  QEI_HandleTypeDef encoderRight;
};

#endif /* APP_INC_APP_HPP_ */
