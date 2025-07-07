#ifndef APP_INC_MOTOR_OBSERVER_HPP_
#define APP_INC_MOTOR_OBSERVER_HPP_

class MotorCurrentObserver {
 public:
  MotorCurrentObserver(float kt, float ke, float la, float ra, float dt);

  void setParameters(float kt, float ke, float la, float ra);
  void setDt(float dt);
  void reset();

  float estimateCurrent(float voltage_command, float motor_speed,
                        float measured_current);
  float getCorrectedCurrent(float measured_current, float voltage_command,
                            float motor_speed);

  float getEstimatedCurrent() const { return estimated_current_; }
  float getObserverGain() const { return observer_gain_; }
  void setObserverGain(float gain) { observer_gain_ = gain; }

 private:
  float kt_;  // トルク定数 [Nm/A]
  float ke_;  // 逆起電力定数 [V*s/rad]
  float la_;  // 電機子インダクタンス [H]
  float ra_;  // 電機子抵抗 [Ω]
  float dt_;  // サンプリング周期 [s]

  float estimated_current_;  // 推定電流 [A]
  float observer_gain_;      // オブザーバーゲイン

  float saturate(float value, float min_val, float max_val);
};

#endif /* APP_INC_MOTOR_OBSERVER_HPP_ */