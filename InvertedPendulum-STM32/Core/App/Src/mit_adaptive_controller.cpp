/*
 * mit_adaptive_controller.cpp
 *
 *  Created on: July 8, 2025
 *      Author: nop
 */

#include "mit_adaptive_controller.hpp"

MitAdaptiveController::MitAdaptiveController(float dt, float gamma1,
                                             float gamma2, float gamma3,
                                             float gamma4)
    : dt(dt),
      prev_reference_output(0.0f),
      prev_reference_velocity(0.0f),
      prev_plant_output(0.0f),
      prev_plant_velocity(0.0f),
      adaptation_error(0.0f),
      reference_output(0.0f),
      plant_output(0.0f),
      error_integral(0.0f),
      prev_error(0.0f) {
  gamma[0] = gamma1;
  gamma[1] = gamma2;
  gamma[2] = gamma3;
  gamma[3] = gamma4;

  setReferenceModel(10.0f, 0.7f);

  for (int i = 0; i < 4; i++) {
    adaptive_gains[i] = 0.0f;
    initial_gains[i] = 0.0f;
  }
}

void MitAdaptiveController::setReferenceModel(float wn, float zeta) {
  reference_model_wn = wn;
  reference_model_zeta = zeta;

  reference_model_a1 = 2.0f * zeta * wn;
  reference_model_a2 = wn * wn;
}

void MitAdaptiveController::setInitialGains(float k1, float k2, float k3,
                                            float k4) {
  initial_gains[0] = k1;
  initial_gains[1] = k2;
  initial_gains[2] = k3;
  initial_gains[3] = k4;

  for (int i = 0; i < 4; i++) {
    adaptive_gains[i] = initial_gains[i];
  }
}

void MitAdaptiveController::reset() {
  prev_reference_output = 0.0f;
  prev_reference_velocity = 0.0f;
  prev_plant_output = 0.0f;
  prev_plant_velocity = 0.0f;
  adaptation_error = 0.0f;
  reference_output = 0.0f;
  plant_output = 0.0f;
  error_integral = 0.0f;
  prev_error = 0.0f;

  for (int i = 0; i < 4; i++) {
    adaptive_gains[i] = initial_gains[i];
  }
}

float MitAdaptiveController::update(float x, float dx, float theta,
                                    float dtheta, float reference) {
  updateReferenceModel(reference);

  plant_output = calculatePlantOutput(x, dx, theta, dtheta);

  adaptation_error = plant_output - reference_output;

  updateAdaptiveGains(x, dx, theta, dtheta);

  float control_input =
      -(adaptive_gains[0] * x + adaptive_gains[1] * dx +
        adaptive_gains[2] * theta + adaptive_gains[3] * dtheta);

  return control_input;
}

void MitAdaptiveController::updateReferenceModel(float reference) {
  float reference_velocity = (reference - prev_reference_output) / dt;

  // Reference acceleration calculation (for potential future use)
  // float reference_acceleration = reference_model_a2 * reference -
  //                               reference_model_a1 * reference_velocity;

  reference_output = prev_reference_output + reference_velocity * dt;

  prev_reference_output = reference_output;
  prev_reference_velocity = reference_velocity;
}

void MitAdaptiveController::updateAdaptiveGains(float x, float dx, float theta,
                                                float dtheta) {
  float plant_velocity = (plant_output - prev_plant_output) / dt;

  float sensitivity_derivatives[4];
  sensitivity_derivatives[0] = -x;
  sensitivity_derivatives[1] = -dx;
  sensitivity_derivatives[2] = -theta;
  sensitivity_derivatives[3] = -dtheta;

  for (int i = 0; i < 4; i++) {
    float adaptation_rate =
        gamma[i] * adaptation_error * sensitivity_derivatives[i];
    adaptive_gains[i] += adaptation_rate * dt;

    if (adaptive_gains[i] > initial_gains[i] * 2.0f) {
      adaptive_gains[i] = initial_gains[i] * 2.0f;
    }
    if (adaptive_gains[i] < initial_gains[i] * 0.5f) {
      adaptive_gains[i] = initial_gains[i] * 0.5f;
    }
  }

  prev_plant_output = plant_output;
  prev_plant_velocity = plant_velocity;
}

float MitAdaptiveController::calculatePlantOutput(float x, float dx,
                                                  float theta, float dtheta) {
  (void)x;       // Suppress unused parameter warning
  (void)dx;      // Suppress unused parameter warning
  (void)dtheta;  // Suppress unused parameter warning
  return theta;
}

void MitAdaptiveController::getAdaptiveGains(float* gains) {
  for (int i = 0; i < 4; i++) {
    gains[i] = adaptive_gains[i];
  }
}

void MitAdaptiveController::setAdaptationRates(float gamma1, float gamma2,
                                               float gamma3, float gamma4) {
  gamma[0] = gamma1;
  gamma[1] = gamma2;
  gamma[2] = gamma3;
  gamma[3] = gamma4;
}