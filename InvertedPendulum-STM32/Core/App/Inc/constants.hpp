#ifndef CONSTANTS_HPP_
#define CONSTANTS_HPP_

namespace InvertedPendulum {
namespace Constants {
// Mathematical constants
constexpr float PI = 3.14159265358979323846f;

// Control system constants
constexpr float DT = 1.0f / 10000.0f;  // 10kHz sampling rate
constexpr float CONTROL_FREQUENCY = 10000.0f;

// Motor constants
constexpr float Kt = 0.0186f;            // Torque constant [Nm/A]
constexpr float KE = 0.0186f;            // Back EMF constant [V*s/rad]
constexpr float LA = 0.003f;             // Armature inductance [H]
constexpr float RA = 32.4f;              // Armature resistance [Ω]
constexpr float GEAR_RATIO = 6.67f;      // Gear ratio
constexpr float WHEEL_RADIUS = 0.0255f;  // Wheel radius [m]
constexpr float Bx = 2.2276f;            // Friction coefficient

// Encoder constants
constexpr int ENCODER_PULSES_PER_REV = 12 * 4;  // 12 pulses * 4 (quadrature)
constexpr float PULSE_TO_RAD = (2.0f * PI) / ENCODER_PULSES_PER_REV;
constexpr float PULSE_TO_POSITION =
    (2.0f * PI * WHEEL_RADIUS) / (ENCODER_PULSES_PER_REV * GEAR_RATIO);

// ADC constants
constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;
constexpr float ADC_TO_VOLTAGE = (2400.0f + 750.0f) / 750.0f;
constexpr float ADV_TO_RAD = 3.8393f;

// Current measurement constants
constexpr float AMPLIFICATION_FACTOR = 150.0f;
constexpr float SHUNT_REGISTER = 0.010f;
constexpr float SPEED_TO_CURRENT = WHEEL_RADIUS / (2.0f * GEAR_RATIO * Kt);

// Control limits
constexpr float MAX_VOLTAGE = 12.0f;
constexpr float MIN_VOLTAGE = -12.0f;
constexpr float MAX_CURRENT = 10.0f;
constexpr float MIN_CURRENT = -10.0f;

// Filter constants
constexpr float THETA_FILTER_CUTOFF = 50.0f;      // Hz
constexpr float CURRENT_FILTER_CUTOFF = 1000.0f;  // Hz

// State feedback gains (LQR)
constexpr float K_POSITION = -3.1623f;
constexpr float K_VELOCITY = -8.4042f;
constexpr float K_ANGLE = -58.4769f;
constexpr float K_ANGULAR_VELOCITY = -11.7355f;
}  // namespace Constants
}  // namespace InvertedPendulum

#endif /* CONSTANTS_HPP_ */