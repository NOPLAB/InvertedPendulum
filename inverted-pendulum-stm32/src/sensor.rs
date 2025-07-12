use crate::{constants, sensor::adc::AdcData};

pub mod adc;
pub mod qei;

/// Sensor data structure - ALL ANGLES IN RADIANS (simplified)
#[derive(Debug, Clone)]
pub struct SensorManager {
    sample_time: f32, // Sample time in seconds

    pub theta0: f32, // Pendulum angle 1 [radians]
    pub theta1: f32, // Pendulum angle 2 [radians]

    pub theta0_velocity: f32, // Angular velocity 1 [rad/s]
    pub theta1_velocity: f32, // Angular velocity 2 [rad/s]

    pub current_r: f32, // Right motor current [A] - raw from ADC
    pub current_l: f32, // Left motor current [A] - raw from ADC

    pub position_r: f32, // Right wheel position [m] - for control
    pub position_l: f32, // Left wheel position [m] - for control

    pub velocity_r: f32, // Right wheel velocity [m/s] - for control
    pub velocity_l: f32, // Left wheel velocity [m/s] - for control

    pub motor_speed_r: f32, // Right motor shaft speed [rad/s] - for observer
    pub motor_speed_l: f32, // Left motor shaft speed [rad/s] - for observer

    pub vin_voltage: f32, // Input voltage [V]

    prev_qei_r: i32,
    prev_qei_l: i32,
}

impl SensorManager {
    pub fn new(sample_time: f32) -> Self {
        Self {
            sample_time,
            theta0: 0.0,
            theta1: 0.0,
            theta0_velocity: 0.0,
            theta1_velocity: 0.0,
            current_r: 0.0,
            current_l: 0.0,
            position_r: 0.0,
            position_l: 0.0,
            velocity_r: 0.0,
            velocity_l: 0.0,
            motor_speed_r: 0.0,
            motor_speed_l: 0.0,
            vin_voltage: 0.0,
            prev_qei_r: 0,
            prev_qei_l: 0,
        }
    }

    pub fn update(&mut self, adc_data: &AdcData, qei_r: i32, qei_l: i32) {
        // Update sensor data from ADC readings
        let theta0 = adc_data.get_sensor_angles_radians().0;
        let theta1 = adc_data.get_sensor_angles_radians().1;

        self.theta0_velocity = (theta0 - self.theta0) / self.sample_time;
        self.theta1_velocity = (theta1 - self.theta1) / self.sample_time;

        self.theta0 = theta0;
        self.theta1 = theta1;

        let position_r = constants::pulses_to_position(qei_r);
        let position_l = -constants::pulses_to_position(qei_l);

        // Calculate wheel velocity in m/s (for control) - like C++ dx calculation
        self.velocity_r = constants::pulses_to_position(qei_r - self.prev_qei_r) / self.sample_time;
        self.velocity_l = -constants::pulses_to_position(qei_l - self.prev_qei_l) / self.sample_time;

        // Calculate motor shaft speed in rad/s (for observer) - like C++ motor_speed
        self.motor_speed_r = ((qei_r - self.prev_qei_r) as f32) * constants::PULSE_TO_RAD / self.sample_time;
        self.motor_speed_l = -((qei_l - self.prev_qei_l) as f32) * constants::PULSE_TO_RAD / self.sample_time;

        self.position_r = position_r;
        self.position_l = position_l;

        self.vin_voltage = adc_data.get_vin_voltage();

        // Store raw current measurements (no processing here)
        let motor_currents = adc_data.get_motor_currents();
        self.current_r = motor_currents.0;
        self.current_l = motor_currents.1;

        self.prev_qei_r = qei_r;
        self.prev_qei_l = qei_l;
    }
}
