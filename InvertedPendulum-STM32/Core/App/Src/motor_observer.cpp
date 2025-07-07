#include "motor_observer.hpp"

#include <cmath>

MotorCurrentObserver::MotorCurrentObserver(float kt, float ke, float la,
                                           float ra, float dt)
    : kt_(kt),
      ke_(ke),
      la_(la),
      ra_(ra),
      dt_(dt),
      estimated_current_(0.0f),
      observer_gain_(1.0f) {}

void MotorCurrentObserver::setParameters(float kt, float ke, float la,
                                         float ra) {
  kt_ = kt;
  ke_ = ke;
  la_ = la;
  ra_ = ra;
}

void MotorCurrentObserver::setDt(float dt) { dt_ = dt; }

void MotorCurrentObserver::reset() { estimated_current_ = 0.0f; }

float MotorCurrentObserver::estimateCurrent(float voltage_command,
                                            float motor_speed,
                                            float measured_current) {
  // DCモーターの電気方程式: La * di/dt = Va - Ra * i - Ke * ω
  // 離散化: i[k+1] = i[k] + (dt/La) * (Va - Ra * i[k] - Ke * ω)

  // 逆起電力の計算
  float back_emf = ke_ * motor_speed;

  // 電流の微分項
  float current_derivative =
      (voltage_command - ra_ * estimated_current_ - back_emf) / la_;

  // 予測ステップ（オープンループ推定）
  float predicted_current = estimated_current_ + current_derivative * dt_;

  // 測定値を使用した修正ステップ（クローズドループ補正）
  float measurement_error = measured_current - predicted_current;

  // オブザーバーによる推定値の更新
  estimated_current_ =
      predicted_current + observer_gain_ * measurement_error * dt_;

  // 電流の飽和制限
  estimated_current_ = saturate(estimated_current_, -20.0f, 20.0f);

  return estimated_current_;
}

float MotorCurrentObserver::getCorrectedCurrent(float measured_current,
                                                float voltage_command,
                                                float motor_speed) {
  // オブザーバーで推定された電流値を取得
  float estimated =
      estimateCurrent(voltage_command, motor_speed, measured_current);

  // 電圧指令値が非常に小さい場合は電流もゼロに近いと仮定
  if (fabsf(voltage_command) < 0.1f) {
    return 0.0f;
  }

  // 符号推定の安定化：電圧指令値から主に符号を決定
  float voltage_based_sign = (voltage_command >= 0.0f) ? 1.0f : -1.0f;

  // オブザーバー推定値の符号も考慮するが、重み付けを軽くする
  float observer_sign = (estimated >= 0.0f) ? 1.0f : -1.0f;

  // 電圧指令値を80%、オブザーバー推定を20%で重み付け
  float final_sign = 0.8f * voltage_based_sign + 0.2f * observer_sign;
  final_sign = (final_sign >= 0.0f) ? 1.0f : -1.0f;

  // 測定値の絶対値に符号を適用
  float corrected_current = fabsf(measured_current) * final_sign;

  return corrected_current;
}

float MotorCurrentObserver::saturate(float value, float min_val,
                                     float max_val) {
  if (value > max_val) return max_val;
  if (value < min_val) return min_val;
  return value;
}