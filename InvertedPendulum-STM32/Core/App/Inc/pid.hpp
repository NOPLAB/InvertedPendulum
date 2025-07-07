#ifndef APP_INC_PID_HPP_
#define APP_INC_PID_HPP_

class PID {
 public:
  PID(float kp, float ki, float kd, float dt, float min_output = -1.0f,
      float max_output = 1.0f);

  void setGains(float kp, float ki, float kd);
  void setOutputLimits(float min_output, float max_output);
  void setDt(float dt);
  void reset();

  float update(float setpoint, float measurement);

  float getKp() const { return kp_; }
  float getKi() const { return ki_; }
  float getKd() const { return kd_; }
  float getIntegralTerm() const { return integral_term_; }
  float getDerivativeTerm() const { return derivative_term_; }

 private:
  float kp_, ki_, kd_;
  float dt_;
  float min_output_, max_output_;

  float integral_term_;
  float derivative_term_;
  float prev_error_;
  bool first_call_;

  float clamp(float value, float min_val, float max_val);
};

#endif /* APP_INC_PID_HPP_ */