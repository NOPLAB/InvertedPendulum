use crate::adaptive_line_controller::AdaptiveLineController;
use crate::constants::{self, *};
use crate::lpf::LowPassFilter;
use crate::mit_adaptive_controller::MitAdaptiveController;
use crate::motor_observer::MotorObserver;
use crate::pid::PidController;
use crate::sensor::SensorManager;

#[derive(Debug, Clone, Copy)]
pub enum ControllerError {
    None,
}

#[derive(Debug, Clone, Copy)]
pub struct ControllerOutputs {
    pub duty_l: f32, // Left motor duty cycle command [0.0 - 1.0]
    pub duty_r: f32, // Right motor duty cycle command [0.0 - 1.0]
}

/// Controller references structure
#[derive(Debug, Clone, Copy)]
pub struct ControlReferences {
    pub theta0_ref: f32,   // Pendulum angle 1 reference [rad]
    pub theta1_ref: f32,   // Pendulum angle 2 reference [rad]
    pub position_ref: f32, // Position reference [m]
    pub velocity_ref: f32, // Velocity reference [m/s]
}

impl ControlReferences {
    pub fn new() -> Self {
        Self {
            theta0_ref: 0.0,
            theta1_ref: 0.0,
            position_ref: 0.0,
            velocity_ref: 0.0,
        }
    }
}

/// High-level controller trait
/// All high-level controllers (LQR, Adaptive, MPC, etc.) implement this trait
pub trait HighLevelController {
    fn compute_control_force(
        &mut self,
        sensor_data: &SensorManager,
        references: &ControlReferences,
    ) -> Result<f32, ControllerError>;

    fn reset(&mut self);
    fn set_parameters(&mut self, params: &[f32]);
}

/// LQR State Feedback Controller
/// Outputs control force based on state feedback: u = -K*x
#[derive(Debug)]
pub struct LqrController {
    k_position: f32,
    k_velocity: f32,
    k_angle: f32,
    k_angular_velocity: f32,
    theta_filter: LowPassFilter,
    prev_filtered_theta: f32,
    theta_velocity: f32,
}

impl LqrController {
    pub fn new() -> Self {
        Self {
            k_position: K_POSITION,
            k_velocity: K_VELOCITY,
            k_angle: K_ANGLE,
            k_angular_velocity: K_ANGULAR_VELOCITY,
            theta_filter: LowPassFilter::new(DT, THETA_FILTER_CUTOFF_FREQ, 1.0),
            prev_filtered_theta: 0.0,
            theta_velocity: 0.0,
        }
    }

    pub fn with_gains(k_pos: f32, k_vel: f32, k_angle: f32, k_angular_vel: f32) -> Self {
        Self {
            k_position: k_pos,
            k_velocity: k_vel,
            k_angle: k_angle,
            k_angular_velocity: k_angular_vel,
            theta_filter: LowPassFilter::new(DT, THETA_FILTER_CUTOFF_FREQ, 1.0),
            prev_filtered_theta: 0.0,
            theta_velocity: 0.0,
        }
    }
}

impl HighLevelController for LqrController {
    fn compute_control_force(
        &mut self,
        sensor_data: &SensorManager,
        references: &ControlReferences,
    ) -> Result<f32, ControllerError> {
        // Calculate state variables
        let position = (sensor_data.position_r + sensor_data.position_l) / 2.0;
        let velocity = (sensor_data.velocity_r + sensor_data.velocity_l) / 2.0;
        let filtered_theta = self.theta_filter.update(sensor_data.theta0);

        // Calculate theta velocity from filtered theta derivative
        self.theta_velocity = (filtered_theta - self.prev_filtered_theta) / DT;
        self.prev_filtered_theta = filtered_theta;

        let angle = filtered_theta; // ADC側で符号処理済み
        let angular_velocity = self.theta_velocity;

        // Calculate state errors
        let position_error = position - references.position_ref;
        let velocity_error = velocity - references.velocity_ref;
        let angle_error = angle - references.theta0_ref;
        let angular_velocity_error = angular_velocity;

        // LQR state feedback: u = -K*x (force output)
        let control_force = -(self.k_position * position_error
            + self.k_velocity * velocity_error
            + self.k_angle * angle_error
            + self.k_angular_velocity * angular_velocity_error);

        // Apply force limits
        let limited_force = clamp_f32(control_force, -MAX_FORCE, MAX_FORCE);

        Ok(limited_force)
    }

    fn reset(&mut self) {
        self.theta_filter.reset();
        self.prev_filtered_theta = 0.0;
        self.theta_velocity = 0.0;
    }

    fn set_parameters(&mut self, params: &[f32]) {
        if params.len() >= 4 {
            self.k_position = params[0];
            self.k_velocity = params[1];
            self.k_angle = params[2];
            self.k_angular_velocity = params[3];
        }
    }
}

/// MIT Adaptive Controller Wrapper
/// Wraps the existing MIT adaptive controller to implement the trait
#[derive(Debug)]
pub struct AdaptiveController {
    controller: MitAdaptiveController,
    theta_filter: LowPassFilter,
    prev_filtered_theta: f32,
    theta_velocity: f32,
}

impl AdaptiveController {
    pub fn new(sample_time: f32) -> Self {
        let mut controller = MitAdaptiveController::new(sample_time);
        controller.set_initial_gains(K_POSITION, K_VELOCITY, K_ANGLE, K_ANGULAR_VELOCITY);
        controller.set_reference_model(10.0, 0.7);
        Self {
            controller,
            theta_filter: LowPassFilter::new(sample_time, THETA_FILTER_CUTOFF_FREQ, 1.0),
            prev_filtered_theta: 0.0,
            theta_velocity: 0.0,
        }
    }
}

impl HighLevelController for AdaptiveController {
    fn compute_control_force(
        &mut self,
        sensor_data: &SensorManager,
        references: &ControlReferences,
    ) -> Result<f32, ControllerError> {
        let x = (sensor_data.position_r + sensor_data.position_l) / 2.0;
        let dx = (sensor_data.velocity_r + sensor_data.velocity_l) / 2.0;
        let theta = self.theta_filter.update(sensor_data.theta0);

        // Calculate theta velocity from filtered theta derivative
        self.theta_velocity = (theta - self.prev_filtered_theta) / DT;
        self.prev_filtered_theta = theta;

        let dtheta = self.theta_velocity;

        /* info!(
            "position: {}, velocity: {}, angle: {}, angular_velocity: {}",
            x, dx, theta, dtheta
        ); */

        // MIT Adaptive control - outputs force directly
        let control_force = self
            .controller
            .update(x, dx, theta, dtheta, references.theta0_ref);

        // Apply force limits
        let limited_force = clamp_f32(control_force, -MAX_FORCE, MAX_FORCE);

        Ok(limited_force)
    }

    fn reset(&mut self) {
        self.controller.reset();
        self.theta_filter.reset();
        self.prev_filtered_theta = 0.0;
        self.theta_velocity = 0.0;
    }

    fn set_parameters(&mut self, params: &[f32]) {
        if params.len() >= 4 {
            self.controller
                .set_initial_gains(params[0], params[1], params[2], params[3]);
        }
    }
}

/// Current Controller
/// Low-level current regulation using PID control with motor observer and filtering
/// Uses custom PID implementation that exactly matches C++ code
pub struct CurrentController {
    pid_left: PidController,
    pid_right: PidController,
    observer_left: MotorObserver,
    observer_right: MotorObserver,
    current_filter_left: LowPassFilter,
    current_filter_right: LowPassFilter,
    prev_voltage_l: f32,
    prev_voltage_r: f32,
}

impl CurrentController {
    pub fn new() -> Self {
        // Create PID controllers with exact C++ parameters
        // C++: PID *pid_current = new PID(0.928f, 10178.8f, 0.0f, DT, -12.0f, 12.0f);
        let pid_left = PidController::new(
            CURRENT_PID_KP,
            CURRENT_PID_KI,
            CURRENT_PID_KD,
            DT,
            CURRENT_PID_OUTPUT_MIN,
            CURRENT_PID_OUTPUT_MAX,
        );
        let pid_right = PidController::new(
            CURRENT_PID_KP,
            CURRENT_PID_KI,
            CURRENT_PID_KD,
            DT,
            CURRENT_PID_OUTPUT_MIN,
            CURRENT_PID_OUTPUT_MAX,
        );

        // Create motor observers with C++ parameters (match MotorCurrentObserver)
        let observer_left = MotorObserver::new(DT);
        let observer_right = MotorObserver::new(DT);

        // Create current filters to match C++ implementation (1000Hz cutoff)
        let current_filter_left = LowPassFilter::new(DT, CURRENT_FILTER_CUTOFF_FREQ, 1.0);
        let current_filter_right = LowPassFilter::new(DT, CURRENT_FILTER_CUTOFF_FREQ, 1.0);

        Self {
            pid_left,
            pid_right,
            observer_left,
            observer_right,
            current_filter_left,
            current_filter_right,
            prev_voltage_l: 0.0,
            prev_voltage_r: 0.0,
        }
    }

    pub fn current_to_duty(
        &mut self,
        target_current_l: f32,
        target_current_r: f32,
        measured_current_l: f32,
        measured_current_r: f32,
        motor_speed_l: f32,
        motor_speed_r: f32,
        vin_voltage: f32,
    ) -> ControllerOutputs {
        // CORRECTED: Apply motor observer first, then filtering (match C++ implementation)
        // C++ order: measured_current -> observer -> corrected_current -> filter -> filtered_current
        let corrected_current_l = self.observer_left.get_corrected_current(
            measured_current_l,
            self.prev_voltage_l, // Use actual voltage command from PID
            motor_speed_l,
        );
        let corrected_current_r = self.observer_right.get_corrected_current(
            measured_current_r,
            self.prev_voltage_r, // Use actual voltage command from PID
            motor_speed_r,
        );

        // Apply current filtering after observer correction (match C++ implementation)
        let filtered_current_l = self.current_filter_left.update(corrected_current_l);
        let filtered_current_r = self.current_filter_right.update(corrected_current_r);

        // PID control for current regulation using filtered current
        // C++: float u = pid_current->update(force_to_current, current_filtered) / vin;
        // Note: C++ uses (setpoint, measurement) order
        let voltage_l = self.pid_left.update(target_current_l, filtered_current_l);
        let voltage_r = self.pid_right.update(target_current_r, filtered_current_r);

        // Apply voltage limits before duty cycle conversion (match C++ implementation)
        let limited_voltage_l = clamp_f32(voltage_l, -MAX_VOLTAGE, MAX_VOLTAGE);
        let limited_voltage_r = clamp_f32(voltage_r, -MAX_VOLTAGE, MAX_VOLTAGE);

        // Convert voltage to duty cycle (exact match to C++ implementation)
        let mut duty_l = limited_voltage_l / vin_voltage;
        let mut duty_r = limited_voltage_r / vin_voltage;

        // Apply duty cycle limits (match C++ voltage limiting logic)
        duty_l = if duty_l * vin_voltage > MAX_VOLTAGE {
            MAX_VOLTAGE / vin_voltage
        } else if duty_l * vin_voltage < -MAX_VOLTAGE {
            -MAX_VOLTAGE / vin_voltage
        } else {
            duty_l
        };

        duty_r = if duty_r * vin_voltage > MAX_VOLTAGE {
            MAX_VOLTAGE / vin_voltage
        } else if duty_r * vin_voltage < -MAX_VOLTAGE {
            -MAX_VOLTAGE / vin_voltage
        } else {
            duty_r
        };

        // Store previous voltages for observer correction
        self.prev_voltage_l = limited_voltage_l;
        self.prev_voltage_r = limited_voltage_r;

        ControllerOutputs { duty_l, duty_r }
    }

    pub fn reset(&mut self) {
        self.pid_left.reset();
        self.pid_right.reset();
        self.observer_left.reset();
        self.observer_right.reset();
        self.current_filter_left.reset();
        self.current_filter_right.reset();
    }
}

/// UART Force Controller
/// Directly applies force commands received via UART
#[derive(Debug)]
pub struct UartController {
    force_command: f32,
}

impl UartController {
    pub fn new() -> Self {
        Self { force_command: 0.0 }
    }

    pub fn set_force_command(&mut self, force: f32) {
        self.force_command = clamp_f32(force, -MAX_FORCE, MAX_FORCE);
    }
}

impl HighLevelController for UartController {
    fn compute_control_force(
        &mut self,
        _sensor_data: &SensorManager,
        _references: &ControlReferences,
    ) -> Result<f32, ControllerError> {
        Ok(self.force_command)
    }

    fn reset(&mut self) {
        self.force_command = 0.0;
    }

    fn set_parameters(&mut self, _params: &[f32]) {
        // UART controller doesn't use parameters
    }
}

/// Controller type enum for compile-time dispatch
pub enum ControllerType {
    Lqr(LqrController),
    Adaptive(AdaptiveController),
    Uart(UartController),
    AdaptiveLine(AdaptiveLineController),
}

/// Complete Controller System
/// Combines high-level controller, force-to-current converter, and current controller
pub struct ControllerSystem {
    controller_type: ControllerType,
    current_controller: CurrentController,
    references: ControlReferences,
}

impl ControllerSystem {
    pub fn new_lqr() -> Self {
        Self {
            controller_type: ControllerType::Lqr(LqrController::new()),
            current_controller: CurrentController::new(),
            references: ControlReferences::new(),
        }
    }

    pub fn new_adaptive(sample_time: f32) -> Self {
        let adaptive = AdaptiveController::new(sample_time);

        Self {
            controller_type: ControllerType::Adaptive(adaptive),
            current_controller: CurrentController::new(),
            references: ControlReferences::new(),
        }
    }

    pub fn new_uart() -> Self {
        Self {
            controller_type: ControllerType::Uart(UartController::new()),
            current_controller: CurrentController::new(),
            references: ControlReferences::new(),
        }
    }

    pub fn new_adaptive_line(sample_time: f32) -> Self {
        Self {
            controller_type: ControllerType::AdaptiveLine(AdaptiveLineController::new(sample_time)),
            current_controller: CurrentController::new(),
            references: ControlReferences::new(),
        }
    }

    pub fn set_references(&mut self, references: ControlReferences) {
        self.references = references;
    }

    /// Complete control computation: Force -> Current -> Voltage
    pub fn compute_control(
        &mut self,
        sensor_data: &SensorManager,
    ) -> Result<ControllerOutputs, ControllerError> {
        match &mut self.controller_type {
            ControllerType::AdaptiveLine(controller) => {
                // 倒立振子ライントレース用左右独立制御
                let (force_left, force_right) = controller.compute_control_outputs(sensor_data, &self.references)?;
                
                // 左右独立電流制御
                let control_outputs = self.current_controller.current_to_duty(
                    force_left * constants::FORCE_TO_CURRENT,
                    force_right * constants::FORCE_TO_CURRENT,
                    sensor_data.current_l,
                    sensor_data.current_r,
                    sensor_data.motor_speed_l,
                    sensor_data.motor_speed_r,
                    sensor_data.vin_voltage,
                );

                Ok(control_outputs)
            }
            _ => {
                // 他のコントローラ用従来実装（単一制御力）
                let control_force = match &mut self.controller_type {
                    ControllerType::Lqr(controller) => {
                        controller.compute_control_force(sensor_data, &self.references)?
                    }
                    ControllerType::Adaptive(controller) => {
                        controller.compute_control_force(sensor_data, &self.references)?
                    }
                    ControllerType::Uart(controller) => {
                        controller.compute_control_force(sensor_data, &self.references)?
                    }
                    ControllerType::AdaptiveLine(_) => unreachable!(), // 上で処理済み
                };

                // 従来の同一制御力での電流制御
                let control_outputs = self.current_controller.current_to_duty(
                    control_force * constants::FORCE_TO_CURRENT,
                    control_force * constants::FORCE_TO_CURRENT,
                    sensor_data.current_l,
                    sensor_data.current_r,
                    sensor_data.motor_speed_l,
                    sensor_data.motor_speed_r,
                    sensor_data.vin_voltage,
                );

                Ok(control_outputs)
            }
        }
    }

    pub fn set_uart_force_command(&mut self, force: f32) {
        if let ControllerType::Uart(controller) = &mut self.controller_type {
            controller.set_force_command(force);
        }
    }

    pub fn switch_to_lqr(&mut self) {
        self.controller_type = ControllerType::Lqr(LqrController::new());
        self.current_controller.reset();
    }

    pub fn switch_to_adaptive(&mut self, sample_time: f32) {
        self.controller_type = ControllerType::Adaptive(AdaptiveController::new(sample_time));
        self.current_controller.reset();
    }

    pub fn switch_to_uart(&mut self) {
        self.controller_type = ControllerType::Uart(UartController::new());
        self.current_controller.reset();
    }

    pub fn switch_to_adaptive_line(&mut self, sample_time: f32) {
        self.controller_type =
            ControllerType::AdaptiveLine(AdaptiveLineController::new(sample_time));
        self.current_controller.reset();
    }

    pub fn get_adaptive_line_position(&self) -> Option<f32> {
        if let ControllerType::AdaptiveLine(controller) = &self.controller_type {
            Some(controller.get_line_position())
        } else {
            None
        }
    }

    pub fn is_adaptive_line_detected(&self) -> Option<bool> {
        if let ControllerType::AdaptiveLine(controller) = &self.controller_type {
            Some(controller.is_line_detected())
        } else {
            None
        }
    }

    pub fn reset(&mut self) {
        match &mut self.controller_type {
            ControllerType::Lqr(controller) => controller.reset(),
            ControllerType::Adaptive(controller) => controller.reset(),
            ControllerType::Uart(controller) => controller.reset(),
            ControllerType::AdaptiveLine(controller) => controller.reset(),
        }
        self.current_controller.reset();
    }
}
