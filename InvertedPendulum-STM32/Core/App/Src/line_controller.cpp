#include "line_controller.hpp"

#include <cmath>

#include "app.hpp"

LineController::LineController()
    : line_detected_(false),
      line_position_(0.0f),
      line_velocity_(0.0f),
      prev_line_position_(0.0f),
      position_filter_alpha_(
          calculateAlpha(20.0f, 0.0001f))  // Use DT value directly
{
  // Initialize PID controller for line following
  line_pid_ = new PID(80.0f, 0.5f, 1.2f, 0.0001f, -0.5f,
                      0.5f);  // kp, ki, kd, dt, min, max
}

bool LineController::detectLinePosition(float* position) {
  uint16_t* sensor_values = getLineSensorValues();

  float weighted_sum = 0.0f;
  float total_weight = 0.0f;
  bool line_detected = false;

  // Detect line using weighted average of sensor positions
  for (int i = 0; i < 4; i++) {
    if (sensor_values[i] > LINE_THRESHOLD) {
      line_detected = true;
      // Use line intensity as weight
      float weight = static_cast<float>(sensor_values[i] - LINE_THRESHOLD);
      weighted_sum += SENSOR_POSITIONS[i] * weight;
      total_weight += weight;
    }
  }

  // Calculate line position
  float raw_position = 0.0f;
  if (line_detected && total_weight > 0.0f) {
    raw_position = weighted_sum / total_weight;
  } else {
    line_detected_ = false;
    return false;
  }

  // Apply low-pass filter
  float filtered_position;
  if (line_detected_) {
    // Apply filter if line was previously detected
    filtered_position = line_position_ * (1.0f - position_filter_alpha_) +
                        raw_position * position_filter_alpha_;
  } else {
    // First detection, no filter
    filtered_position = raw_position;
  }

  // Calculate line velocity
  line_velocity_ = (filtered_position - prev_line_position_) /
                   0.0001f;  // Use DT value directly

  // Update state
  prev_line_position_ = line_position_;
  line_position_ = filtered_position;
  line_detected_ = line_detected;

  if (position) {
    *position = line_position_;
  }

  return true;
}

float LineController::computePositionCommand() {
  float position;
  if (!detectLinePosition(&position)) {
    return 0.0f;  // No line detected, return zero command
  }

  // Error from line center (target is 0.0)
  float error = -line_position_;  // Move towards the line

  // Compute PID output
  float position_command =
      line_pid_->update(0.0f, error);  // target=0, current=error

  // Limit position command
  return clamp(position_command, -0.5f, 0.5f);
}

void LineController::setPIDGains(float kp, float ki, float kd) {
  delete line_pid_;
  line_pid_ = new PID(kp, ki, kd, 0.0001f, -0.5f, 0.5f);
}

void LineController::reset() {
  line_detected_ = false;
  line_position_ = 0.0f;
  line_velocity_ = 0.0f;
  prev_line_position_ = 0.0f;
  if (line_pid_) {
    line_pid_->reset();
  }
}

void LineController::getSensorValues(uint16_t* values) {
  uint16_t* sensor_values = getLineSensorValues();
  for (int i = 0; i < 4; i++) {
    values[i] = sensor_values[i];
  }
}

float LineController::clamp(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

float LineController::calculateAlpha(float cutoff_freq, float dt) {
  float rc = 1.0f / (2.0f * 3.14159265359f * cutoff_freq);
  return dt / (rc + dt);
}

uint16_t* LineController::getLineSensorValues() {
  // Get line sensor values from ADC multiplexer (channels 0-3)
  static uint16_t sensor_values[4];
  Adc1CorrectedValues* adc_values =
      App::getInstance().getAdc1()->getCorrectedValues();

  // Assuming multiplexer channels 0-3 are line sensors
  sensor_values[0] = static_cast<uint16_t>(adc_values->mux_value[0]);
  sensor_values[1] = static_cast<uint16_t>(adc_values->mux_value[1]);
  sensor_values[2] = static_cast<uint16_t>(adc_values->mux_value[2]);
  sensor_values[3] = static_cast<uint16_t>(adc_values->mux_value[3]);

  return sensor_values;
}
