use crate::constants::*;
use crate::lpf::LowPassFilter;

/// Motor current observer for state estimation
/// This implements a Luenberger observer for estimating motor current
/// based on motor dynamics and measurements
#[derive(Debug, Clone)]
pub struct MotorObserver {
    // Motor parameters
    kt: f32, // Torque constant [Nm/A]
    ke: f32, // Back EMF constant [V/(rad/s)]
    la: f32, // Inductance [H]
    ra: f32, // Resistance [Ohm]

    // Observer gain
    observer_gain: f32, // Observer gain for correction

    // State variables
    estimated_current: f32,  // Estimated motor current [A]
    estimated_velocity: f32, // Estimated motor velocity [rad/s]

    // Previous values
    prev_voltage: f32,  // Previous applied voltage [V]
    prev_current: f32,  // Previous measured current [A]
    prev_velocity: f32, // Previous measured velocity [rad/s]

    // Measurement filters
    current_filter: LowPassFilter,  // Current measurement filter
    velocity_filter: LowPassFilter, // Velocity measurement filter
    voltage_filter: LowPassFilter,  // Voltage measurement filter

    // Observer parameters
    sample_time: f32, // Sample time [s]

    // Error tracking
    current_error: f32,      // Current estimation error
    velocity_error: f32,     // Velocity estimation error
    max_current_error: f32,  // Maximum current error
    max_velocity_error: f32, // Maximum velocity error

    // State flags
    initialized: bool, // Initialization flag

    // Statistics
    update_count: u32, // Number of updates

    // Saturation limits
    max_current: f32,  // Maximum current limit
    max_velocity: f32, // Maximum velocity limit
}

impl MotorObserver {
    /// Create a new motor observer with default parameters
    pub fn new(sample_time: f32) -> Self {
        Self {
            kt: MOTOR_KT,
            ke: MOTOR_KE,
            la: MOTOR_LA,
            ra: MOTOR_RA,
            observer_gain: 10.0,
            estimated_current: 0.0,
            estimated_velocity: 0.0,
            prev_voltage: 0.0,
            prev_current: 0.0,
            prev_velocity: 0.0,
            current_filter: LowPassFilter::new(sample_time, 100.0, 1.0),
            velocity_filter: LowPassFilter::new(sample_time, 50.0, 1.0),
            voltage_filter: LowPassFilter::new(sample_time, 200.0, 1.0),
            sample_time,
            current_error: 0.0,
            velocity_error: 0.0,
            max_current_error: 0.0,
            max_velocity_error: 0.0,
            initialized: false,
            update_count: 0,
            max_current: MAX_CURRENT,
            max_velocity: MAX_SPEED,
        }
    }

    /// Create a new motor observer with custom motor parameters
    pub fn new_with_params(
        kt: f32,
        ke: f32,
        la: f32,
        ra: f32,
        observer_gain: f32,
        sample_time: f32,
    ) -> Self {
        let mut observer = Self::new(sample_time);
        observer.set_motor_parameters(kt, ke, la, ra);
        observer.set_observer_gain(observer_gain);
        observer
    }

    /// Set motor parameters
    pub fn set_motor_parameters(&mut self, kt: f32, ke: f32, la: f32, ra: f32) {
        self.kt = kt;
        self.ke = ke;
        self.la = la;
        self.ra = ra;
    }

    /// Set observer gain
    pub fn set_observer_gain(&mut self, gain: f32) {
        self.observer_gain = gain;
    }

    /// Set sample time
    pub fn set_sample_time(&mut self, sample_time: f32) {
        self.sample_time = sample_time;
    }

    /// Set current and velocity limits
    pub fn set_limits(&mut self, max_current: f32, max_velocity: f32) {
        self.max_current = max_current;
        self.max_velocity = max_velocity;
    }

    /// Get corrected current (C++ style implementation)
    pub fn get_corrected_current(
        &mut self,
        measured_current: f32,
        voltage_command: f32,
        motor_speed: f32,
    ) -> f32 {
        // First estimate the current using the observer
        let estimated = self.estimate_current(voltage_command, motor_speed, measured_current);

        // Exact C++ implementation logic
        // If voltage command is very small, assume current is also near zero
        if voltage_command.abs() < 0.1 {
            return 0.0;
        }

        // Sign estimation stabilization: determine sign mainly from voltage command
        let voltage_based_sign = if voltage_command >= 0.0 { 1.0 } else { -1.0 };

        // Also consider observer estimation sign but with lighter weighting
        let observer_sign = if estimated >= 0.0 { 1.0 } else { -1.0 };

        // Weight voltage command 80%, observer estimation 20%
        let final_sign_raw = 0.8 * voltage_based_sign + 0.2 * observer_sign;
        let final_sign = if final_sign_raw >= 0.0 { 1.0 } else { -1.0 };

        // Apply sign to absolute value of measured current
        let corrected_current = measured_current.abs() * final_sign;

        corrected_current
    }

    /// Estimate current using motor dynamics (C++ style)
    fn estimate_current(
        &mut self,
        voltage_command: f32,
        motor_speed: f32,
        measured_current: f32,
    ) -> f32 {
        // DC motor electrical equation: La * di/dt = Va - Ra * i - Ke * ω
        // Discretized: i[k+1] = i[k] + (dt/La) * (Va - Ra * i[k] - Ke * ω)

        // Calculate back EMF
        let back_emf = self.ke * motor_speed;

        // Current derivative
        let current_derivative =
            (voltage_command - self.ra * self.estimated_current - back_emf) / self.la;

        // Prediction step (open-loop estimation)
        let predicted_current = self.estimated_current + current_derivative * self.sample_time;

        // Correction step using measurement (closed-loop correction)
        let measurement_error = measured_current - predicted_current;

        // Update estimated current using observer
        self.estimated_current =
            predicted_current + self.observer_gain * measurement_error * self.sample_time;

        // Current saturation limits (exact C++ values)
        self.estimated_current = clamp_f32(self.estimated_current, -20.0, 20.0);

        self.estimated_current
    }

    /// Update the observer with new measurements
    pub fn update(&mut self, voltage: f32, measured_current: f32, measured_velocity: f32) -> f32 {
        // Filter measurements
        let filtered_voltage = self.voltage_filter.update(voltage);
        let filtered_current = self.current_filter.update(measured_current);
        let filtered_velocity = self.velocity_filter.update(measured_velocity);

        // Initialize on first call
        if !self.initialized {
            self.estimated_current = filtered_current;
            self.estimated_velocity = filtered_velocity;
            self.prev_voltage = filtered_voltage;
            self.prev_current = filtered_current;
            self.prev_velocity = filtered_velocity;
            self.initialized = true;
            return 0.0;
        }

        // Motor dynamics model:
        // di/dt = (V - R*i - Ke*w) / L
        // dw/dt = (Kt*i - Tload) / J (velocity dynamics handled externally)

        // Predict current using motor model
        let back_emf = self.ke * self.estimated_velocity;
        let voltage_drop = self.ra * self.estimated_current;
        let di_dt = (filtered_voltage - voltage_drop - back_emf) / self.la;

        // Integrate to get predicted current
        let predicted_current = self.estimated_current + di_dt * self.sample_time;

        // Apply saturation
        let saturated_current = clamp_f32(predicted_current, -self.max_current, self.max_current);

        // Calculate current error
        self.current_error = filtered_current - saturated_current;

        // Observer correction
        let corrected_current = saturated_current + self.observer_gain * self.current_error;

        // Update estimated current
        self.estimated_current = clamp_f32(corrected_current, -self.max_current, self.max_current);

        // Update velocity estimate (simple first-order dynamics)
        // This is a simplified model - in practice, you'd use full mechanical dynamics
        let velocity_time_constant = 0.1; // [s]
        let velocity_alpha = self.sample_time / (velocity_time_constant + self.sample_time);
        self.estimated_velocity =
            velocity_alpha * filtered_velocity + (1.0 - velocity_alpha) * self.estimated_velocity;

        // Apply velocity saturation
        self.estimated_velocity = clamp_f32(
            self.estimated_velocity,
            -self.max_velocity,
            self.max_velocity,
        );

        // Calculate velocity error
        self.velocity_error = filtered_velocity - self.estimated_velocity;

        // Update error statistics
        let abs_current_error = self.current_error.abs();
        let abs_velocity_error = self.velocity_error.abs();

        if abs_current_error > self.max_current_error {
            self.max_current_error = abs_current_error;
        }

        if abs_velocity_error > self.max_velocity_error {
            self.max_velocity_error = abs_velocity_error;
        }

        // Store previous values
        self.prev_voltage = filtered_voltage;
        self.prev_current = filtered_current;
        self.prev_velocity = filtered_velocity;

        self.update_count += 1;

        // Return the estimated current
        self.estimated_current
    }

    /// Get estimated current
    pub fn get_estimated_current(&self) -> f32 {
        self.estimated_current
    }

    /// Get estimated velocity
    pub fn get_estimated_velocity(&self) -> f32 {
        self.estimated_velocity
    }

    /// Get current estimation error
    pub fn get_current_error(&self) -> f32 {
        self.current_error
    }

    /// Get velocity estimation error
    pub fn get_velocity_error(&self) -> f32 {
        self.velocity_error
    }

    /// Get maximum current error
    pub fn get_max_current_error(&self) -> f32 {
        self.max_current_error
    }

    /// Get maximum velocity error
    pub fn get_max_velocity_error(&self) -> f32 {
        self.max_velocity_error
    }

    /// Get update count
    pub fn get_update_count(&self) -> u32 {
        self.update_count
    }

    /// Check if observer is initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Reset the observer
    pub fn reset(&mut self) {
        self.estimated_current = 0.0;
        self.estimated_velocity = 0.0;
        self.prev_voltage = 0.0;
        self.prev_current = 0.0;
        self.prev_velocity = 0.0;
        self.current_error = 0.0;
        self.velocity_error = 0.0;
        self.max_current_error = 0.0;
        self.max_velocity_error = 0.0;
        self.initialized = false;
        self.update_count = 0;

        // Reset filters
        self.current_filter.reset();
        self.velocity_filter.reset();
        self.voltage_filter.reset();
    }

    /// Get motor parameters
    pub fn get_motor_parameters(&self) -> (f32, f32, f32, f32) {
        (self.kt, self.ke, self.la, self.ra)
    }

    /// Get observer gain
    pub fn get_observer_gain(&self) -> f32 {
        self.observer_gain
    }

    /// Calculate back EMF
    pub fn calculate_back_emf(&self, velocity: f32) -> f32 {
        self.ke * velocity
    }

    /// Calculate voltage drop
    pub fn calculate_voltage_drop(&self, current: f32) -> f32 {
        self.ra * current
    }

    /// Calculate torque from current
    pub fn calculate_torque(&self, current: f32) -> f32 {
        self.kt * current
    }

    /// Predict current for given voltage and velocity
    pub fn predict_current(&self, voltage: f32, velocity: f32) -> f32 {
        // Steady-state current: I = (V - Ke*w) / R
        let back_emf = self.calculate_back_emf(velocity);
        let steady_state_current = (voltage - back_emf) / self.ra;
        clamp_f32(steady_state_current, -self.max_current, self.max_current)
    }

    /// Get observer performance metrics
    pub fn get_performance_metrics(&self) -> ObserverMetrics {
        ObserverMetrics {
            max_current_error: self.max_current_error,
            max_velocity_error: self.max_velocity_error,
            current_error_rms: self.current_error.abs(), // Simplified RMS
            velocity_error_rms: self.velocity_error.abs(), // Simplified RMS
            update_count: self.update_count,
            is_initialized: self.initialized,
        }
    }

    /// Set filter parameters
    pub fn set_filter_parameters(
        &mut self,
        current_cutoff: f32,
        velocity_cutoff: f32,
        voltage_cutoff: f32,
    ) {
        self.current_filter.set_cutoff_frequency(current_cutoff);
        self.velocity_filter.set_cutoff_frequency(velocity_cutoff);
        self.voltage_filter.set_cutoff_frequency(voltage_cutoff);
    }
}

/// Observer performance metrics
#[derive(Debug, Clone, Copy)]
pub struct ObserverMetrics {
    pub max_current_error: f32,
    pub max_velocity_error: f32,
    pub current_error_rms: f32,
    pub velocity_error_rms: f32,
    pub update_count: u32,
    pub is_initialized: bool,
}

impl ObserverMetrics {
    pub fn new() -> Self {
        Self {
            max_current_error: 0.0,
            max_velocity_error: 0.0,
            current_error_rms: 0.0,
            velocity_error_rms: 0.0,
            update_count: 0,
            is_initialized: false,
        }
    }
}

impl Default for ObserverMetrics {
    fn default() -> Self {
        Self::new()
    }
}
