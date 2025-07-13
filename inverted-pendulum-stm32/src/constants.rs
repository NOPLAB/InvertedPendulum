// Motor constants (from C++ implementation)
pub const MOTOR_KT: f32 = 0.0186; // Torque constant [Nm/A]
pub const MOTOR_KE: f32 = 0.0186; // Back EMF constant [V·s/rad]
pub const MOTOR_LA: f32 = 0.003; // Inductance [H]
pub const MOTOR_RA: f32 = 32.4; // Resistance [Ohm]

// Mechanical constants (from C++ implementation)
pub const WHEEL_RADIUS: f32 = 0.0255; // Wheel radius [m]
pub const GEAR_RATIO: f32 = 6.67; // Gear ratio

// ADC constants (from C++ implementation)
pub const ADC_RESOLUTION: u32 = 4096; // 12-bit ADC
pub const ADC_VREF: f32 = 3.3; // Reference voltage [V]
pub const ADC_TO_VOLTAGE: f32 = ADC_VREF / (ADC_RESOLUTION as f32);
pub const ADC_TO_VOLTAGE_C: f32 = (2400.0 + 750.0) / 750.0; // C++ implementation ADC_TO_VOLTAGE

pub const POT_ELECTRICAL_ANGLE: f32 = 333.3; // Electrical effective angle [degrees]
pub const POT_SUPPLY_VOLTAGE: f32 = 5.0; // Potentiometer supply voltage [V]
pub const VOLTAGE_DIVIDER_RATIO: f32 = ADC_VREF / POT_SUPPLY_VOLTAGE; // 3.3V/5V = 0.66
pub const ADC_TO_RAD: f32 =
    (POT_ELECTRICAL_ANGLE * (2.0 * core::f32::consts::PI / 360.0)) / VOLTAGE_DIVIDER_RATIO;

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

// Control limits (from C++ implementation)
pub const MAX_CURRENT: f32 = 10.0; // Maximum current [A]
pub const MAX_VOLTAGE: f32 = 12.0; // Maximum voltage [V]
pub const MAX_SPEED: f32 = 10.0; // Maximum speed [rad/s]
pub const MAX_FORCE: f32 = 10.0; // Maximum control force [N]

// Filter constants (from C++ implementation)
pub const CURRENT_FILTER_CUTOFF_FREQ: f32 = 500.0; // Exact match to C++ CURRENT_FILTER_CUTOFF [Hz]
pub const THETA_FILTER_CUTOFF_FREQ: f32 = 500.0; // Exact match to C++ THETA_FILTER_CUTOFF [Hz] (500Hz)

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
pub const MOTOR_PWM_FREQUENCY: u32 = 50_000; // 100kHz PWM frequency

// Timing constants (from C++ implementation)
pub const CONTROL_LOOP_FREQUENCY: u32 = 2000;
pub const DT: f32 = 1.0 / CONTROL_LOOP_FREQUENCY as f32; // Control loop time step [s]

// LQR State Feedback Gains
// [-7.0711, -7.9196, -30.1895, -3.7929]
pub const K_POSITION: f32 = -7.0711;
pub const K_VELOCITY: f32 = -7.9196;
pub const K_ANGLE: f32 = -30.1895;
pub const K_ANGULAR_VELOCITY: f32 = -3.7929;

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

pub fn adc_to_radians(ad_value: u16, zero_offset: u16) -> f32 {
    // C++ implementation: float theta = -(float)(adc1_values->p_1 - zero_ad) * ADV_TO_RAD;
    // where adc1_values->p_1 and zero_ad are normalized (0.0-1.0)
    let normalized_ad = (ad_value as f32) / (ADC_RESOLUTION as f32);
    let normalized_offset = (zero_offset as f32) / (ADC_RESOLUTION as f32);
    let radian_value = -(normalized_ad - normalized_offset) * ADC_TO_RAD;
    radian_value
}

pub fn adc_to_voltage(adc_value: u16) -> f32 {
    (adc_value as f32) * ADC_TO_VOLTAGE
}

pub fn convert_adc_to_current(adc_value: f32) -> f32 {
    adc_value * ADC_VREF / AMPLIFICATION_FACTOR / SHUNT_REGISTER
}

pub fn adc_to_current(adc_value: u16) -> f32 {
    // First normalize to 0-1.0 range like C++
    let normalized_adc = (adc_value as f32) / (ADC_RESOLUTION as f32);
    // Then apply C++ conversion
    convert_adc_to_current(normalized_adc)
}

pub fn adc_to_current_with_offset(adc_value: u16, offset: u16) -> f32 {
    // Apply offset compensation first, then convert
    let compensated_adc = if adc_value >= offset {
        adc_value - offset
    } else {
        0 // Clamp to zero if offset is larger
    };
    adc_to_current(compensated_adc)
}

// C++ implementation position calculation (exact match)
pub const PULSE_TO_POSITION: f32 =
    (2.0 * core::f32::consts::PI * WHEEL_RADIUS) / ((ENCODER_PULSES_PER_REV as f32) * GEAR_RATIO);

pub fn pulses_to_position(pulses: i32) -> f32 {
    (pulses as f32) * PULSE_TO_POSITION
}
