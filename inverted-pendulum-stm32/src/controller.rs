use crate::constants::{self, *};
use crate::fmt::info;
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

/// Control force output structure
/// Represents the output of high-level controllers (LQR, Adaptive, etc.)
#[derive(Debug, Clone, Copy)]
pub struct ControlForce {
    pub force: f32,    // Control force [N]
    pub torque_l: f32, // Left motor torque [Nm] - for individual motor control
    pub torque_r: f32, // Right motor torque [Nm] - for individual motor control
}

impl ControlForce {
    pub fn new() -> Self {
        Self {
            force: 0.0,
            torque_l: 0.0,
            torque_r: 0.0,
        }
    }

    pub fn from_force(force: f32) -> Self {
        Self {
            force,
            torque_l: 0.0,
            torque_r: 0.0,
        }
    }

    pub fn from_torques(torque_l: f32, torque_r: f32) -> Self {
        Self {
            force: 0.0,
            torque_l,
            torque_r,
        }
    }
}

/// Current command structure
/// Represents the output of force-to-current conversion
#[derive(Debug, Clone, Copy)]
pub struct CurrentCommand {
    pub current_l: f32, // Left motor current reference [A]
    pub current_r: f32, // Right motor current reference [A]
}

impl CurrentCommand {
    pub fn new() -> Self {
        Self {
            current_l: 0.0,
            current_r: 0.0,
        }
    }

    pub fn from_currents(current_l: f32, current_r: f32) -> Self {
        Self {
            current_l,
            current_r,
        }
    }
}

/// Controller references structure
#[derive(Debug, Clone, Copy)]
pub struct ControlReferences {
    pub theta1_ref: f32,   // Pendulum angle 1 reference [rad]
    pub theta2_ref: f32,   // Pendulum angle 2 reference [rad]
    pub position_ref: f32, // Position reference [m]
    pub velocity_ref: f32, // Velocity reference [m/s]
}

impl ControlReferences {
    pub fn new() -> Self {
        Self {
            theta1_ref: 0.0,
            theta2_ref: 0.0,
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
    ) -> Result<ControlForce, ControllerError>;

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
}

impl LqrController {
    pub fn new() -> Self {
        Self {
            k_position: K_POSITION,
            k_velocity: K_VELOCITY,
            k_angle: K_ANGLE,
            k_angular_velocity: K_ANGULAR_VELOCITY,
        }
    }

    pub fn with_gains(k_pos: f32, k_vel: f32, k_angle: f32, k_angular_vel: f32) -> Self {
        Self {
            k_position: k_pos,
            k_velocity: k_vel,
            k_angle: k_angle,
            k_angular_velocity: k_angular_vel,
        }
    }
}

impl HighLevelController for LqrController {
    fn compute_control_force(
        &mut self,
        sensor_data: &SensorManager,
        references: &ControlReferences,
    ) -> Result<ControlForce, ControllerError> {
        // Calculate state variables
        let position = (sensor_data.position_r + sensor_data.position_l) / 2.0;
        let velocity = (sensor_data.velocity_r + sensor_data.velocity_l) / 2.0;
        let angle = sensor_data.theta0;
        let angular_velocity = sensor_data.theta0_velocity;

        // Calculate state errors
        let position_error = position - references.position_ref;
        let velocity_error = velocity - references.velocity_ref;
        let angle_error = angle - references.theta1_ref;
        let angular_velocity_error = angular_velocity;

        // LQR state feedback: u = -K*x (force output)
        let control_force = -(self.k_position * position_error
            + self.k_velocity * velocity_error
            + self.k_angle * angle_error
            + self.k_angular_velocity * angular_velocity_error);

        // Apply force limits
        let limited_force = clamp_f32(control_force, -MAX_FORCE, MAX_FORCE);

        Ok(ControlForce::from_force(limited_force))
    }

    fn reset(&mut self) {
        // LQR controller is stateless, no reset needed
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
}

impl AdaptiveController {
    pub fn new(sample_time: f32) -> Self {
        let mut controller = MitAdaptiveController::new(sample_time);
        controller.set_initial_gains(-2.0000, -5.6814, -22.4525, -2.75834);
        controller.set_reference_model(10.0, 0.7);
        Self { controller }
    }
}

impl HighLevelController for AdaptiveController {
    fn compute_control_force(
        &mut self,
        sensor_data: &SensorManager,
        references: &ControlReferences,
    ) -> Result<ControlForce, ControllerError> {
        let x = (sensor_data.position_r + sensor_data.position_l) / 2.0;
        let dx = (sensor_data.velocity_r + sensor_data.velocity_l) / 2.0;
        let theta = sensor_data.theta0;
        let dtheta = sensor_data.theta0_velocity;

        // MIT Adaptive control - outputs force directly
        let control_force = self
            .controller
            .update(x, dx, theta, dtheta, references.theta1_ref);

        // Apply force limits
        let limited_force = clamp_f32(control_force, -MAX_FORCE, MAX_FORCE);

        Ok(ControlForce::from_force(limited_force))
    }

    fn reset(&mut self) {
        self.controller.reset();
    }

    fn set_parameters(&mut self, params: &[f32]) {
        if params.len() >= 4 {
            self.controller
                .set_initial_gains(params[0], params[1], params[2], params[3]);
        }
    }
}

/// Force-to-Current Converter
/// Converts control force to current commands using motor/mechanical parameters
pub struct ForceToCurrentConverter {
    wheel_radius: f32,
    gear_ratio: f32,
    motor_kt: f32,
    num_motors: f32,
}

impl ForceToCurrentConverter {
    pub fn new() -> Self {
        Self {
            wheel_radius: WHEEL_RADIUS,
            gear_ratio: GEAR_RATIO,
            motor_kt: MOTOR_KT,
            num_motors: 2.0, // Two motors (matches C++ implementation)
        }
    }

    /// Convert control force to current command
    /// Formula: I = F * (R / (G * Kt * N)) - exact match to C++ implementation
    /// C++: force_to_current = control_output * (WHEEL_RADIUS / (GEAR_RATIO * Kt * 2.0f))
    pub fn force_to_current(&self, control_force: ControlForce) -> CurrentCommand {
        if control_force.force != 0.0 {
            // Convert force to current (both motors get same current) - exact match to C++
            let current =
                control_force.force * (self.wheel_radius / (self.gear_ratio * self.motor_kt * 2.0));
            CurrentCommand::from_currents(current, current)
        } else {
            // Individual torque control
            let current_l = control_force.torque_l / self.motor_kt;
            let current_r = control_force.torque_r / self.motor_kt;
            CurrentCommand::from_currents(current_l, current_r)
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
        let current_filter_left = LowPassFilter::new(DT, CURRENT_FILTER_CUTOFF, 1.0);
        let current_filter_right = LowPassFilter::new(DT, CURRENT_FILTER_CUTOFF, 1.0);

        Self {
            pid_left,
            pid_right,
            observer_left,
            observer_right,
            current_filter_left,
            current_filter_right,
        }
    }

    pub fn current_to_duty(
        &mut self,
        current_cmd: CurrentCommand,
        measured_current_l: f32,
        measured_current_r: f32,
        motor_speed_l: f32,
        motor_speed_r: f32,
        vin_voltage: f32,
    ) -> ControllerOutputs {
        /* info!(
            "CurrentController: measured_current_l: {}, measured_current_r: {}, vin_voltage: {}",
            measured_current_l, measured_current_r, vin_voltage
        ); */

        // Apply current filtering first (match C++ implementation)
        let filtered_current_l = self.current_filter_left.update(measured_current_l);
        let filtered_current_r = self.current_filter_right.update(measured_current_r);

        // PID control for current regulation using filtered current
        // C++: float u = pid_current->update(force_to_current, current_filtered) / vin;
        // Note: C++ uses (setpoint, measurement) order
        let voltage_l = self
            .pid_left
            .update(current_cmd.current_l, filtered_current_l);
        let voltage_r = self
            .pid_right
            .update(current_cmd.current_r, filtered_current_r);

        // Apply motor observer for current correction with actual voltage command
        let corrected_current_l = self.observer_left.get_corrected_current(
            filtered_current_l,
            voltage_l, // Use actual voltage command from PID
            motor_speed_l,
        );
        let corrected_current_r = self.observer_right.get_corrected_current(
            filtered_current_r,
            voltage_r, // Use actual voltage command from PID
            motor_speed_r,
        );

        /* info!(
            "voltage_l: {}, voltage_r: {}, vin_voltage: {}",
            voltage_l, voltage_r, vin_voltage
        ); */

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

/// Controller type enum for compile-time dispatch
pub enum ControllerType {
    Lqr(LqrController),
    Adaptive(AdaptiveController),
}

/// Complete Controller System
/// Combines high-level controller, force-to-current converter, and current controller
pub struct ControllerSystem {
    controller_type: ControllerType,
    force_converter: ForceToCurrentConverter,
    current_controller: CurrentController,
    references: ControlReferences,
}

impl ControllerSystem {
    pub fn new_lqr() -> Self {
        Self {
            controller_type: ControllerType::Lqr(LqrController::new()),
            force_converter: ForceToCurrentConverter::new(),
            current_controller: CurrentController::new(),
            references: ControlReferences::new(),
        }
    }

    pub fn new_adaptive(sample_time: f32) -> Self {
        let adaptive = AdaptiveController::new(sample_time);

        Self {
            controller_type: ControllerType::Adaptive(adaptive),
            force_converter: ForceToCurrentConverter::new(),
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
        // Step 1: High-level controller computes force
        let control_force = match &mut self.controller_type {
            ControllerType::Lqr(controller) => {
                controller.compute_control_force(sensor_data, &self.references)?
            }
            ControllerType::Adaptive(controller) => {
                controller.compute_control_force(sensor_data, &self.references)?
            }
        };

        // Step 2: Convert force to current command
        let current_cmd = self.force_converter.force_to_current(control_force);

        let current_cmd = CurrentCommand::from_currents(0.1, 0.1);

        // Step 3: Current controller computes voltage
        let control_outputs = self.current_controller.current_to_duty(
            current_cmd,
            sensor_data.current_l,
            sensor_data.current_r,
            sensor_data.motor_speed_l, // motor speed left
            sensor_data.motor_speed_r, // motor speed right
            sensor_data.vin_voltage,
        );

        Ok(control_outputs)
    }

    pub fn reset(&mut self) {
        match &mut self.controller_type {
            ControllerType::Lqr(controller) => controller.reset(),
            ControllerType::Adaptive(controller) => controller.reset(),
        }
        self.current_controller.reset();
    }
}
