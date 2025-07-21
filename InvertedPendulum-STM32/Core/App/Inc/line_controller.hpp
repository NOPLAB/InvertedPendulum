#ifndef LINE_CONTROLLER_HPP_
#define LINE_CONTROLLER_HPP_

#include <cstdint>

#include "constants.hpp"
#include "pid.hpp"

// Line sensor constants
const float LINE_THRESHOLD = 1000.0f;
const float SENSOR_POSITIONS[4] = {-0.03f, -0.01f, 0.01f,
                                   0.03f};  // Left to right positions in meters

class LineController {
 public:
  LineController();
  ~LineController() = default;

  // Line detection and position calculation
  bool detectLinePosition(float* position);
  float computePositionCommand();

  // Configuration
  void setPIDGains(float kp, float ki, float kd);
  void reset();

  // State queries
  bool isLineDetected() const { return line_detected_; }
  float getLinePosition() const { return line_position_; }
  float getLineVelocity() const { return line_velocity_; }

  // Debug information
  void getSensorValues(uint16_t* values);

 private:
  // Line tracking state
  bool line_detected_;
  float line_position_;
  float line_velocity_;
  float prev_line_position_;

  // PID controller for line following
  PID* line_pid_;

  // Filtering
  float position_filter_alpha_;

  // Helper functions
  float clamp(float value, float min_val, float max_val);
  float calculateAlpha(float cutoff_freq, float dt);
  uint16_t* getLineSensorValues();
};

#endif /* LINE_CONTROLLER_HPP_ */