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
#include "lpf.hpp"
#include "mit_adaptive_controller.hpp"
#include "motor.hpp"
#include "motor_observer.hpp"
#include "pid.hpp"

extern "C" {
#include "qei.h"
}

#define ADC_INTERRUPT_HANDLERS_NUM 2
#define TIMER_INTERRUPT_HANDLERS_NUM 1

#define PI 3.14159265358979323846f
#define DT (100.0f / 1000.0f / 1000.0f)  // 10kHzサンプリング周期 [s]
#define Kt 0.0186f                       // トルク定数 [Nm/A]
#define KE 0.0186f                       // 逆起電力定数 [V*s/rad]
#define LA 0.003f                        // 電機子インダクタンス [H]
#define RA 32.4f                         // 電機子抵抗 [Ω]
#define GEAR_RATIO 6.67f                 // ギア比
#define WHEEL_RADIUS 0.0255f             // 車輪半径 [m]
#define Bx 2.2276f                       // 粘性摩擦係数
#define PULSE_TO_RAD \
  (2.0f * PI / (12.0f * 4.0f))  // エンコーダのパルスからラジアンへの変換
// Potentiometer specifications: 333.3° electrical angle, 5V supply, 3.3V ADC
// ADV_TO_RAD = (333.3 * (2π/360)) / (3.3/5.0) = 5.818 / 0.66 = 8.815
#define POT_ELECTRICAL_ANGLE 333.3f  // ポテンショメータ電気的有効角 [度]
#define POT_SUPPLY_VOLTAGE 5.0f      // ポテンショメータ電源電圧 [V]
#define ADC_VREF_VOLTAGE 3.3f        // ADC基準電圧 [V]
#define VOLTAGE_DIVIDER_RATIO (ADC_VREF_VOLTAGE / POT_SUPPLY_VOLTAGE)  // 電圧分割比
#define ADV_TO_RAD ((POT_ELECTRICAL_ANGLE * (2.0f * PI / 360.0f)) / VOLTAGE_DIVIDER_RATIO)  // ADC値からラジアンへの変換
#define PULSE_TO_POSITION      \
  (2.0f * PI * WHEEL_RADIUS) / \
      (12.0f * 4.0f * GEAR_RATIO)  // エンコーダのパルスから位置[m]への変換
#define ADC_TO_VOLTAGE ((2400.0f + 750.0f) / 750.0f)  // ADC値から電圧への変換
#define SPEED_TO_CURRENT \
  (WHEEL_RADIUS / (2.0f * GEAR_RATIO * Kt))  // 速度から電流への変換

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

  // 1kHz制御周期で呼ばれる
  // void interval_1khz();

 public:
  InterruptHandler *getInterruptHandler() { return this->interruptHandler; }

 private:
  InterruptHandler *interruptHandler = new InterruptHandler();
  IAdcInterruptHandler *adcInterruptHandlers[ADC_INTERRUPT_HANDLERS_NUM];
  ITimerInterruptHandler *timerInterruptHandlers[TIMER_INTERRUPT_HANDLERS_NUM];

  Interval *intervalCaller = nullptr;
  // Interval *intervalCaller_1khz = nullptr;

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

  float prev_pwm_left = 0.0f;
  float prev_pwm_right = 0.0f;

  LowPassFilter *lpf_theta = new LowPassFilter(1.0f, 1 / (2 * PI * 500), DT);

  LowPassFilter *lpf_current = new LowPassFilter(1.0f, 1 / (2 * PI * 1000), DT);

  MotorCurrentObserver *current_observer_left =
      new MotorCurrentObserver(Kt, KE, LA, RA, DT);
  MotorCurrentObserver *current_observer_right =
      new MotorCurrentObserver(Kt, KE, LA, RA, DT);

  PID *pid_current = new PID(0.928f, 10178.8f, 0.0f, DT, -12.0f, 12.0f);
  // PID *pid_speed = new PID(5.0654f, 22.2759f, 0.0f, DT, -10.0f, 10.0f);

  MitAdaptiveController *adaptive_controller =
      new MitAdaptiveController(DT, 1.0f, 1.0f, 1.0f, 1.0f);

  float uart_x = 0.0f;
  float uart_theta = 0.0f;

 public:
  QEI_HandleTypeDef encoderLeft;
  QEI_HandleTypeDef encoderRight;
};

#endif /* APP_INC_APP_HPP_ */
