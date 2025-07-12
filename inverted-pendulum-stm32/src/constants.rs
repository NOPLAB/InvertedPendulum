// Motor constants (from C++ implementation)
pub const MOTOR_KT: f32 = 0.0186; // Torque constant [Nm/A]
pub const MOTOR_KE: f32 = 0.0186; // Back EMF constant [V·s/rad]
pub const MOTOR_LA: f32 = 0.003; // Inductance [H]
pub const MOTOR_RA: f32 = 32.4; // Resistance [Ohm]

// Mechanical constants (from C++ implementation)
pub const WHEEL_RADIUS: f32 = 0.0255; // Wheel radius [m]
pub const GEAR_RATIO: f32 = 6.67; // Gear ratio
pub const BX: f32 = 2.2276; // Exact match to C++ Bx (friction coefficient)
pub const FRICTION_COEFFICIENT: f32 = 2.2276; // Friction coefficient (compatibility)

// ADC constants (from C++ implementation)
pub const ADC_RESOLUTION: u32 = 4096; // 12-bit ADC
pub const ADC_VREF: f32 = 3.3; // Reference voltage [V]
pub const ADC_TO_VOLTAGE: f32 = ADC_VREF / (ADC_RESOLUTION as f32);
pub const ADC_TO_VOLTAGE_C: f32 = (2400.0 + 750.0) / 750.0; // C++ implementation ADC_TO_VOLTAGE
pub const ADC_REFERENCE_VOLTAGE: f32 = 3.3; // Exact match to C++ ADC_REFERENCE_VOLTAGE
pub const ADV_TO_RAD: f32 = 3.8393; // Exact match to C++ ADV_TO_RAD
pub const ADC_TO_RAD: f32 = 3.8393;
pub const ADC_TO_DEG: f32 = 180.0 / core::f32::consts::PI * ADC_TO_RAD;

// Current sensing constants (from C++ implementation)
pub const AMPLIFICATION_FACTOR: f32 = 150.0; // Exact match to C++ AMPLIFICATION_FACTOR
pub const SHUNT_REGISTER: f32 = 0.010; // Exact match to C++ SHUNT_REGISTER [Ohm]
pub const CURRENT_AMPLIFICATION: f32 = 150.0; // Amplification factor (compatibility)
pub const CURRENT_SHUNT_RESISTOR: f32 = 0.010; // Shunt resistor [Ohm] (compatibility)
pub const CURRENT_SENSOR_SENSITIVITY: f32 = 0.1; // [V/A] (for compatibility)
pub const CURRENT_SENSOR_OFFSET: f32 = 1.65; // [V] (for compatibility)

// Encoder constants (from C++ implementation) - ALL IN RADIANS
pub const ENCODER_PULSES_PER_REV: i32 = 12 * 4; // Exact match to C++ (12 pulses * 4 quadrature)
pub const PULSE_TO_RAD: f32 = (2.0 * core::f32::consts::PI) / (12.0 * 4.0); // Exact match to C++
pub const ENCODER_RESOLUTION_RAD: f32 =
    (2.0 * core::f32::consts::PI) / (ENCODER_PULSES_PER_REV as f32); // Radians per pulse

// Control limits (from C++ implementation)
pub const MAX_CURRENT: f32 = 10.0; // Maximum current [A]
pub const MAX_VOLTAGE: f32 = 12.0; // Maximum voltage [V]
pub const MAX_SPEED: f32 = 10.0; // Maximum speed [rad/s]
pub const MAX_FORCE: f32 = 50.0; // Maximum control force [N]

// Filter constants (from C++ implementation)
pub const CURRENT_FILTER_CUTOFF_FREQ: f32 = 1000.0; // Exact match to C++ CURRENT_FILTER_CUTOFF [Hz]
pub const THETA_FILTER_CUTOFF_FREQ: f32 = 500.0; // Exact match to C++ THETA_FILTER_CUTOFF [Hz] (500Hz)
pub const DEFAULT_FILTER_CUTOFF_FREQ: f32 = 100.0; // Default cutoff frequency [Hz]
pub const DEFAULT_FILTER_GAIN: f32 = 1.0; // Default gain

// PID constants (from C++ implementation)
pub const CURRENT_PID_KP: f32 = 0.928;
pub const CURRENT_PID_KI: f32 = 10178.8;
pub const CURRENT_PID_KD: f32 = 0.0;
pub const CURRENT_PID_OUTPUT_MIN: f32 = -12.0;
pub const CURRENT_PID_OUTPUT_MAX: f32 = 12.0;

// MIT Adaptive Controller constants (from C++ implementation)
pub const MIT_GAMMA1: f32 = 1.0; // Adaptation gain 1
pub const MIT_GAMMA2: f32 = 1.0; // Adaptation gain 2
pub const MIT_GAMMA3: f32 = 1.0; // Adaptation gain 3
pub const MIT_REFERENCE_FREQ: f32 = 1.0; // Reference frequency [Hz]
pub const MIT_REFERENCE_DAMPING: f32 = 0.7; // Reference damping ratio
pub const MIT_MAX_GAIN: f32 = 100.0; // Maximum adaptive gain

// Physical pin mappings and channel assignments
pub const MULTIPLEXER_CHANNELS: usize = 8;

// Motor control frequency
pub const MOTOR_PWM_FREQUENCY: u32 = 100_000; // 100kHz PWM frequency

// Timing constants (from C++ implementation)
pub const DT: f32 = 1.0 / 1000.0;
pub const CONTROL_LOOP_FREQUENCY: u32 = 1000;
pub const ADC_SAMPLING_FREQUENCY: u32 = 1000;

// LQR State Feedback Gains (from C++ implementation)
pub const K_POSITION: f32 = -3.1623;
pub const K_VELOCITY: f32 = -8.4042;
pub const K_ANGLE: f32 = -58.4769;
pub const K_ANGULAR_VELOCITY: f32 = -11.7355;

pub const FORCE_TO_CURRENT: f32 = WHEEL_RADIUS / (GEAR_RATIO * MOTOR_KT * 2.0);

// Math utilities
pub fn calculate_alpha(cutoff_freq: f32, sample_time: f32) -> f32 {
    let tau = 1.0 / (2.0 * core::f32::consts::PI * cutoff_freq);
    sample_time / (tau + sample_time)
}

pub fn clamp_f32(value: f32, min_val: f32, max_val: f32) -> f32 {
    if value > max_val {
        max_val
    } else if value < min_val {
        min_val
    } else {
        value
    }
}

// Angle conversion functions
pub fn degrees_to_radians(degrees: f32) -> f32 {
    degrees * core::f32::consts::PI / 180.0
}

pub fn radians_to_degrees(radians: f32) -> f32 {
    radians * 180.0 / core::f32::consts::PI
}

// Convert ADC theta reading to radians (C++ style)
pub fn adc_theta_to_radians(adc_value: f32, zero_offset: f32) -> f32 {
    -(adc_value - zero_offset) * ADC_TO_RAD
}

pub fn adc_to_voltage(adc_value: u16) -> f32 {
    (adc_value as f32) * ADC_TO_VOLTAGE
}

pub fn voltage_to_current(voltage: f32) -> f32 {
    (voltage - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENSITIVITY
}

// C++ implementation current calculation (exact match)
// C++ version: convertAdcToCurrent(float adcValue) where adcValue is normalized (0-1.0)
pub fn convert_adc_to_current(adc_value: f32) -> f32 {
    adc_value * ADC_REFERENCE_VOLTAGE / AMPLIFICATION_FACTOR / SHUNT_REGISTER
}

// Convert raw ADC value to current - matches C++ flow
pub fn adc_to_current(adc_value: u16) -> f32 {
    // First normalize to 0-1.0 range like C++
    let normalized_adc = (adc_value as f32) / (ADC_RESOLUTION as f32);
    // Then apply C++ conversion
    convert_adc_to_current(normalized_adc)
}

// Convert encoder pulses to angle in radians
pub fn pulses_to_angle_radians(pulses: i32) -> f32 {
    (pulses as f32) * ENCODER_RESOLUTION_RAD
}

// Convert angle in radians to encoder pulses
pub fn angle_radians_to_pulses(angle_radians: f32) -> i32 {
    (angle_radians / ENCODER_RESOLUTION_RAD) as i32
}

// Deprecated: keeping for compatibility, but using radians internally
pub fn pulses_to_angle_degrees(pulses: i32) -> f32 {
    radians_to_degrees(pulses_to_angle_radians(pulses))
}

// C++ implementation position calculation (exact match)
pub const PULSE_TO_POSITION: f32 =
    (2.0 * core::f32::consts::PI * WHEEL_RADIUS) / ((ENCODER_PULSES_PER_REV as f32) * GEAR_RATIO);

pub fn pulses_to_position(pulses: i32) -> f32 {
    (pulses as f32) * PULSE_TO_POSITION
}
