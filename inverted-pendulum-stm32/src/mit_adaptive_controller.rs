use crate::constants::*;
use crate::lpf::LowPassFilter;

/// MIT (Massachusetts Institute of Technology) Adaptive Controller
/// This implements a Model Reference Adaptive Control (MRAC) system
/// based on the MIT rule for parameter adaptation.
#[derive(Debug, Clone)]
pub struct MitAdaptiveController {
    // Reference model parameters
    ref_model_wn: f32,   // Natural frequency of reference model
    ref_model_zeta: f32, // Damping ratio of reference model
    ref_model_gain: f32, // Steady-state gain of reference model

    // Reference model state
    ref_model_x1: f32,     // First state variable
    ref_model_x2: f32,     // Second state variable
    ref_model_output: f32, // Reference model output

    // Plant model parameters (estimated)
    plant_b0: f32, // Plant numerator coefficient
    plant_b1: f32, // Plant numerator coefficient
    plant_a1: f32, // Plant denominator coefficient
    plant_a2: f32, // Plant denominator coefficient

    // Plant state
    plant_x1: f32,     // Plant first state
    plant_x2: f32,     // Plant second state
    plant_output: f32, // Plant output

    // Adaptive gains
    theta1: f32, // Adaptive gain 1
    theta2: f32, // Adaptive gain 2
    theta3: f32, // Adaptive gain 3

    // C++ style adaptive gains array (for 4-parameter system)
    adaptive_gains: [f32; 4], // Adaptive gains array [k1, k2, k3, k4]
    initial_gains: [f32; 4],  // Initial gains for reset and limiting

    // Adaptation parameters
    gamma1: f32, // Adaptation gain 1
    gamma2: f32, // Adaptation gain 2
    gamma3: f32, // Adaptation gain 3

    // Sensitivity derivatives
    phi1: f32, // Sensitivity function 1
    phi2: f32, // Sensitivity function 2
    phi3: f32, // Sensitivity function 3

    // Previous values for derivative calculation
    prev_phi1: f32,
    prev_phi2: f32,
    prev_phi3: f32,

    // Control parameters
    sample_time: f32, // Sample time
    max_gain: f32,    // Maximum allowable gain
    min_gain: f32,    // Minimum allowable gain

    // Filters for noise reduction
    error_filter: LowPassFilter, // Error signal filter
    phi_filter1: LowPassFilter,  // Phi1 filter
    phi_filter2: LowPassFilter,  // Phi2 filter
    phi_filter3: LowPassFilter,  // Phi3 filter

    // Control signals
    control_signal: f32, // Current control signal
    error_signal: f32,   // Current error signal

    // State flags
    initialized: bool,        // Initialization flag
    adaptation_enabled: bool, // Adaptation enable flag

    // Statistics
    max_error: f32,        // Maximum error encountered
    adaptation_count: u32, // Number of adaptation steps
}

impl MitAdaptiveController {
    /// Create a new MIT adaptive controller
    pub fn new(sample_time: f32) -> Self {
        let mut controller = Self {
            ref_model_wn: 2.0 * core::f32::consts::PI * MIT_REFERENCE_FREQ,
            ref_model_zeta: MIT_REFERENCE_DAMPING,
            ref_model_gain: 1.0,
            ref_model_x1: 0.0,
            ref_model_x2: 0.0,
            ref_model_output: 0.0,
            plant_b0: 1.0,
            plant_b1: 0.0,
            plant_a1: 0.0,
            plant_a2: 0.0,
            plant_x1: 0.0,
            plant_x2: 0.0,
            plant_output: 0.0,
            theta1: 0.0,
            theta2: 0.0,
            theta3: 0.0,
            adaptive_gains: [0.0; 4],
            initial_gains: [0.0; 4],
            gamma1: MIT_GAMMA1,
            gamma2: MIT_GAMMA2,
            gamma3: MIT_GAMMA3,
            phi1: 0.0,
            phi2: 0.0,
            phi3: 0.0,
            prev_phi1: 0.0,
            prev_phi2: 0.0,
            prev_phi3: 0.0,
            max_gain: MIT_MAX_GAIN,
            min_gain: -MIT_MAX_GAIN,
            sample_time,
            error_filter: LowPassFilter::new(sample_time, 50.0, 1.0),
            phi_filter1: LowPassFilter::new(sample_time, 50.0, 1.0),
            phi_filter2: LowPassFilter::new(sample_time, 50.0, 1.0),
            phi_filter3: LowPassFilter::new(sample_time, 50.0, 1.0),
            control_signal: 0.0,
            error_signal: 0.0,
            initialized: false,
            adaptation_enabled: true,
            max_error: 0.0,
            adaptation_count: 0,
        };

        controller.calculate_reference_model_coefficients();
        controller
    }

    /// Create a new MIT adaptive controller with custom parameters
    pub fn new_with_params(
        ref_freq: f32,
        ref_damping: f32,
        gamma1: f32,
        gamma2: f32,
        gamma3: f32,
        sample_time: f32,
    ) -> Self {
        let mut controller = Self::new(sample_time);
        controller.set_reference_model(ref_freq, ref_damping);
        controller.set_adaptation_gains(gamma1, gamma2, gamma3);
        controller
    }

    /// Calculate reference model coefficients
    fn calculate_reference_model_coefficients(&mut self) {
        // Second-order reference model: H(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
        // This is converted to discrete-time using bilinear transform

        let wn = self.ref_model_wn;
        let zeta = self.ref_model_zeta;
        let dt = self.sample_time;

        // Bilinear transform: s = 2/T * (z-1)/(z+1)
        let a = 2.0 / dt;
        let wn2 = wn * wn;
        let a2 = a * a;
        let two_zeta_wn = 2.0 * zeta * wn;

        // Discrete-time coefficients
        let denominator = a2 + two_zeta_wn * a + wn2;

        // Store coefficients for later use in update
        self.plant_b0 = wn2 / denominator;
        self.plant_b1 = 2.0 * wn2 / denominator;
        self.plant_a1 = (2.0 * wn2 - 2.0 * a2) / denominator;
        self.plant_a2 = (a2 - two_zeta_wn * a + wn2) / denominator;
    }

    /// Set reference model parameters
    pub fn set_reference_model(&mut self, frequency: f32, damping: f32) {
        self.ref_model_wn = 2.0 * core::f32::consts::PI * frequency;
        self.ref_model_zeta = damping;
        self.calculate_reference_model_coefficients();
    }

    /// Set adaptation gains
    pub fn set_adaptation_gains(&mut self, gamma1: f32, gamma2: f32, gamma3: f32) {
        self.gamma1 = gamma1;
        self.gamma2 = gamma2;
        self.gamma3 = gamma3;
    }

    /// Set gain limits
    pub fn set_gain_limits(&mut self, min_gain: f32, max_gain: f32) {
        self.min_gain = min_gain;
        self.max_gain = max_gain;
    }

    /// Enable or disable adaptation
    pub fn set_adaptation_enabled(&mut self, enabled: bool) {
        self.adaptation_enabled = enabled;
    }

    /// C++ style update method matching exact implementation
    pub fn update(&mut self, x: f32, dx: f32, theta: f32, dtheta: f32, reference: f32) -> f32 {
        self.update_reference_model_cpp(reference);

        let plant_output = self.calculate_plant_output(x, dx, theta, dtheta);

        let adaptation_error = plant_output - self.ref_model_output;
        self.error_signal = adaptation_error;

        self.update_adaptive_gains_cpp(x, dx, theta, dtheta, adaptation_error);

        // Control input calculation (exact C++ match)
        let control_input = -(self.adaptive_gains[0] * x
            + self.adaptive_gains[1] * dx
            + self.adaptive_gains[2] * theta
            + self.adaptive_gains[3] * dtheta);

        self.control_signal = control_input;
        control_input
    }

    /// Set initial gains (C++ style)
    pub fn set_initial_gains(&mut self, k1: f32, k2: f32, k3: f32, k4: f32) {
        self.initial_gains[0] = k1;
        self.initial_gains[1] = k2;
        self.initial_gains[2] = k3;
        self.initial_gains[3] = k4;

        // Copy to adaptive gains
        for i in 0..4 {
            self.adaptive_gains[i] = self.initial_gains[i];
        }

        // Also set individual theta values for compatibility
        self.theta1 = k1;
        self.theta2 = k2;
        self.theta3 = k3;
    }

    /// Update reference model (C++ style)
    fn update_reference_model_cpp(&mut self, reference: f32) {
        let reference_velocity = if self.initialized {
            (reference - self.ref_model_output) / self.sample_time
        } else {
            0.0
        };

        self.ref_model_output = if self.initialized {
            self.ref_model_output + reference_velocity * self.sample_time
        } else {
            reference
        };

        if !self.initialized {
            self.initialized = true;
        }
    }

    /// Update adaptive gains (C++ style)
    fn update_adaptive_gains_cpp(
        &mut self,
        x: f32,
        dx: f32,
        theta: f32,
        dtheta: f32,
        adaptation_error: f32,
    ) {
        if !self.adaptation_enabled {
            return;
        }

        // Sensitivity derivatives (exact C++ match)
        let sensitivity_derivatives = [-x, -dx, -theta, -dtheta];
        let gammas = [self.gamma1, self.gamma2, self.gamma3, self.gamma3]; // Note: gamma3 used for both

        for i in 0..4 {
            let adaptation_rate = gammas[i] * adaptation_error * sensitivity_derivatives[i];
            self.adaptive_gains[i] += adaptation_rate * self.sample_time;

            // Apply limits (exact C++ implementation)
            let max_limit = self.initial_gains[i] * 2.0;
            let min_limit = self.initial_gains[i] * 0.5;

            if self.adaptive_gains[i] > max_limit {
                self.adaptive_gains[i] = max_limit;
            }
            if self.adaptive_gains[i] < min_limit {
                self.adaptive_gains[i] = min_limit;
            }
        }

        // Update individual theta values for compatibility
        self.theta1 = self.adaptive_gains[0];
        self.theta2 = self.adaptive_gains[1];
        self.theta3 = self.adaptive_gains[2];
    }

    /// Calculate plant output (C++ style)
    fn calculate_plant_output(&self, x: f32, _dx: f32, _theta: f32, _dtheta: f32) -> f32 {
        // Return position (x) as plant output for position control
        // This allows the adaptive controller to properly control position
        x
    }

    /// Original update method (keeping for compatibility)
    pub fn update_original(&mut self, reference: f32, plant_output: f32) -> f32 {
        // Initialize on first call
        if !self.initialized {
            self.ref_model_x1 = reference;
            self.ref_model_x2 = reference;
            self.ref_model_output = reference;
            self.plant_x1 = plant_output;
            self.plant_x2 = plant_output;
            self.plant_output = plant_output;
            self.initialized = true;
        }

        // Update reference model
        self.update_reference_model(reference);

        // Calculate error signal
        let raw_error = self.ref_model_output - plant_output;
        self.error_signal = self.error_filter.update(raw_error);

        // Update maximum error
        let abs_error = raw_error.abs();
        if abs_error > self.max_error {
            self.max_error = abs_error;
        }

        // Calculate sensitivity derivatives (φ functions)
        self.calculate_sensitivity_derivatives(reference, plant_output);

        // Apply filters to sensitivity derivatives
        let filtered_phi1 = self.phi_filter1.update(self.phi1);
        let filtered_phi2 = self.phi_filter2.update(self.phi2);
        let filtered_phi3 = self.phi_filter3.update(self.phi3);

        // MIT adaptation rule: θ̇ = -γ * e * φ
        if self.adaptation_enabled {
            self.theta1 -= self.gamma1 * self.error_signal * filtered_phi1 * self.sample_time;
            self.theta2 -= self.gamma2 * self.error_signal * filtered_phi2 * self.sample_time;
            self.theta3 -= self.gamma3 * self.error_signal * filtered_phi3 * self.sample_time;

            // Apply gain limits
            self.theta1 = clamp_f32(self.theta1, self.min_gain, self.max_gain);
            self.theta2 = clamp_f32(self.theta2, self.min_gain, self.max_gain);
            self.theta3 = clamp_f32(self.theta3, self.min_gain, self.max_gain);

            self.adaptation_count += 1;
        }

        // Calculate control signal
        self.control_signal = self.theta1 * reference + self.theta2 * plant_output + self.theta3;

        // Update plant model state for next iteration
        self.plant_x2 = self.plant_x1;
        self.plant_x1 = plant_output;
        self.plant_output = plant_output;

        // Update previous sensitivity derivatives
        self.prev_phi1 = self.phi1;
        self.prev_phi2 = self.phi2;
        self.prev_phi3 = self.phi3;

        self.control_signal
    }

    /// Update reference model
    fn update_reference_model(&mut self, reference: f32) {
        // Second-order discrete-time difference equation
        // y[n] = b0*u[n] + b1*u[n-1] + b2*u[n-2] - a1*y[n-1] - a2*y[n-2]

        let b2 = self.plant_b0; // Same as b0 for second-order system
        let u_n = reference;
        let u_n1 = self.ref_model_x1;
        let u_n2 = self.ref_model_x2;
        let y_n1 = self.ref_model_output;
        let y_n2 = self.ref_model_x2; // Reuse x2 for y[n-2]

        // Calculate new reference model output
        let new_output = self.plant_b0 * u_n + self.plant_b1 * u_n1 + b2 * u_n2
            - self.plant_a1 * y_n1
            - self.plant_a2 * y_n2;

        // Update reference model state
        self.ref_model_x2 = self.ref_model_x1;
        self.ref_model_x1 = reference;
        self.ref_model_output = new_output;
    }

    /// Calculate sensitivity derivatives
    fn calculate_sensitivity_derivatives(&mut self, reference: f32, plant_output: f32) {
        // Sensitivity derivatives represent ∂y/∂θ for each parameter
        // These are typically calculated using auxiliary filters

        // For simplicity, we use approximate derivatives:
        // φ1 = ∂y/∂θ1 ≈ reference signal
        // φ2 = ∂y/∂θ2 ≈ plant output
        // φ3 = ∂y/∂θ3 ≈ constant (1.0)

        self.phi1 = reference;
        self.phi2 = plant_output;
        self.phi3 = 1.0;

        // Apply derivative filtering to reduce noise
        if self.initialized {
            let dt = self.sample_time;
            let dphi1_dt = (self.phi1 - self.prev_phi1) / dt;
            let dphi2_dt = (self.phi2 - self.prev_phi2) / dt;
            let dphi3_dt = (self.phi3 - self.prev_phi3) / dt;

            // Incorporate derivative information (optional enhancement)
            self.phi1 += 0.1 * dphi1_dt * dt;
            self.phi2 += 0.1 * dphi2_dt * dt;
            self.phi3 += 0.1 * dphi3_dt * dt;
        }
    }

    /// Reset the controller
    pub fn reset(&mut self) {
        self.ref_model_x1 = 0.0;
        self.ref_model_x2 = 0.0;
        self.ref_model_output = 0.0;
        self.plant_x1 = 0.0;
        self.plant_x2 = 0.0;
        self.plant_output = 0.0;
        self.theta1 = 0.0;
        self.theta2 = 0.0;
        self.theta3 = 0.0;
        self.phi1 = 0.0;
        self.phi2 = 0.0;
        self.phi3 = 0.0;
        self.prev_phi1 = 0.0;
        self.prev_phi2 = 0.0;
        self.prev_phi3 = 0.0;
        self.control_signal = 0.0;
        self.error_signal = 0.0;
        self.initialized = false;
        self.max_error = 0.0;
        self.adaptation_count = 0;

        // Reset adaptive gains to initial values (C++ style)
        for i in 0..4 {
            self.adaptive_gains[i] = self.initial_gains[i];
        }

        // Reset filters
        self.error_filter.reset();
        self.phi_filter1.reset();
        self.phi_filter2.reset();
        self.phi_filter3.reset();
    }

    /// Get current adaptive gains
    pub fn get_adaptive_gains(&self) -> (f32, f32, f32) {
        (self.theta1, self.theta2, self.theta3)
    }

    /// Get current error signal
    pub fn get_error(&self) -> f32 {
        self.error_signal
    }

    /// Get reference model output
    pub fn get_reference_output(&self) -> f32 {
        self.ref_model_output
    }

    /// Get control signal
    pub fn get_control_signal(&self) -> f32 {
        self.control_signal
    }

    /// Get maximum error
    pub fn get_max_error(&self) -> f32 {
        self.max_error
    }

    /// Get adaptation count
    pub fn get_adaptation_count(&self) -> u32 {
        self.adaptation_count
    }

    /// Check if adaptation is enabled
    pub fn is_adaptation_enabled(&self) -> bool {
        self.adaptation_enabled
    }

    /// Check if controller is initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Get sensitivity derivatives
    pub fn get_sensitivity_derivatives(&self) -> (f32, f32, f32) {
        (self.phi1, self.phi2, self.phi3)
    }

    /// Set sample time
    pub fn set_sample_time(&mut self, sample_time: f32) {
        self.sample_time = sample_time;
        self.calculate_reference_model_coefficients();
    }

    /// Get reference model parameters
    pub fn get_reference_model_params(&self) -> (f32, f32) {
        let frequency = self.ref_model_wn / (2.0 * core::f32::consts::PI);
        (frequency, self.ref_model_zeta)
    }

    /// Get adaptation parameters
    pub fn get_adaptation_params(&self) -> (f32, f32, f32) {
        (self.gamma1, self.gamma2, self.gamma3)
    }
}
