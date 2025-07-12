use crate::constants::*;

/// PID Controller - Exact port of C++ implementation
/// Matches the algorithm, anti-windup, and calculation formulas from InvertedPendulum-STM32
#[derive(Debug, Clone)]
pub struct PidController {
    // PID gains
    kp: f32, // Proportional gain
    ki: f32, // Integral gain
    kd: f32, // Derivative gain

    // Timing
    dt: f32, // Sample time [s]

    // Output limits
    min_output: f32, // Minimum output value
    max_output: f32, // Maximum output value

    // Internal state variables
    integral_term: f32,   // Integral accumulator
    derivative_term: f32, // Derivative term
    prev_error: f32,      // Previous error for derivative calculation
    first_call: bool,     // Flag for first call handling
}

impl PidController {
    /// Create a new PID controller with specified parameters
    /// Exact match to C++ constructor: PID(kp, ki, kd, dt, min_output, max_output)
    pub fn new(kp: f32, ki: f32, kd: f32, dt: f32, min_output: f32, max_output: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            dt,
            min_output,
            max_output,
            integral_term: 0.0,
            derivative_term: 0.0,
            prev_error: 0.0,
            first_call: true,
        }
    }

    /// Set PID gains - exact match to C++ setGains()
    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    /// Set output limits - exact match to C++ setOutputLimits()
    pub fn set_output_limits(&mut self, min_output: f32, max_output: f32) {
        self.min_output = min_output;
        self.max_output = max_output;
    }

    /// Set sample time - exact match to C++ setDt()
    pub fn set_dt(&mut self, dt: f32) {
        self.dt = dt;
    }

    /// Reset PID controller - exact match to C++ reset()
    pub fn reset(&mut self) {
        self.integral_term = 0.0;
        self.derivative_term = 0.0;
        self.prev_error = 0.0;
        self.first_call = true;
    }

    /// PID update function - EXACT match to C++ implementation
    /// C++ signature: float update(float setpoint, float measurement)
    pub fn update(&mut self, setpoint: f32, measurement: f32) -> f32 {
        // Calculate error - exact match to C++
        let error = setpoint - measurement;

        // Proportional term - exact match to C++
        let proportional_term = self.kp * error;

        // Integral term - exact match to C++
        self.integral_term += self.ki * error * self.dt;

        // Derivative term - exact match to C++
        if self.first_call {
            self.derivative_term = 0.0;
            self.first_call = false;
        } else {
            self.derivative_term = self.kd * (error - self.prev_error) / self.dt;
        }

        // Calculate PID output - exact match to C++
        let mut output = proportional_term + self.integral_term + self.derivative_term;

        // Apply output limits - exact match to C++
        output = self.clamp(output, self.min_output, self.max_output);

        // Anti-windup: prevent integral windup by clamping integral term
        // EXACT match to C++ anti-windup logic
        if output > self.max_output && self.integral_term > 0.0 {
            self.integral_term = self.max_output - proportional_term - self.derivative_term;
            if self.integral_term < 0.0 {
                self.integral_term = 0.0;
            }
        } else if output < self.min_output && self.integral_term < 0.0 {
            self.integral_term = self.min_output - proportional_term - self.derivative_term;
            if self.integral_term > 0.0 {
                self.integral_term = 0.0;
            }
        }

        // Store previous error for next derivative calculation
        self.prev_error = error;

        output
    }

    /// Reset only integral term (for discrete_pid compatibility)
    pub fn reset_integral(&mut self) {
        self.integral_term = 0.0;
    }

    /// Get proportional gain
    pub fn get_kp(&self) -> f32 {
        self.kp
    }

    /// Get integral gain
    pub fn get_ki(&self) -> f32 {
        self.ki
    }

    /// Get derivative gain
    pub fn get_kd(&self) -> f32 {
        self.kd
    }

    /// Get integral term value
    pub fn get_integral_term(&self) -> f32 {
        self.integral_term
    }

    /// Get derivative term value
    pub fn get_derivative_term(&self) -> f32 {
        self.derivative_term
    }

    /// Clamp function - exact match to C++ implementation
    fn clamp(&self, value: f32, min_val: f32, max_val: f32) -> f32 {
        if value > max_val {
            max_val
        } else if value < min_val {
            min_val
        } else {
            value
        }
    }
}

/// PID Controller builder for easy configuration
pub struct PidBuilder {
    kp: f32,
    ki: f32,
    kd: f32,
    dt: f32,
    min_output: f32,
    max_output: f32,
}

impl PidBuilder {
    /// Create new PID builder with default values
    pub fn new() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            dt: DT,
            min_output: -1.0,
            max_output: 1.0,
        }
    }

    /// Set proportional gain
    pub fn kp(mut self, kp: f32) -> Self {
        self.kp = kp;
        self
    }

    /// Set integral gain
    pub fn ki(mut self, ki: f32) -> Self {
        self.ki = ki;
        self
    }

    /// Set derivative gain
    pub fn kd(mut self, kd: f32) -> Self {
        self.kd = kd;
        self
    }

    /// Set sample time
    pub fn dt(mut self, dt: f32) -> Self {
        self.dt = dt;
        self
    }

    /// Set output limits
    pub fn output_limits(mut self, min_output: f32, max_output: f32) -> Self {
        self.min_output = min_output;
        self.max_output = max_output;
        self
    }

    /// Build the PID controller
    pub fn build(self) -> PidController {
        PidController::new(
            self.kp,
            self.ki,
            self.kd,
            self.dt,
            self.min_output,
            self.max_output,
        )
    }
}

impl Default for PidBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pid_creation() {
        let pid = PidController::new(1.0, 0.1, 0.05, 0.01, -10.0, 10.0);
        assert_eq!(pid.get_kp(), 1.0);
        assert_eq!(pid.get_ki(), 0.1);
        assert_eq!(pid.get_kd(), 0.05);
    }

    #[test]
    fn test_pid_builder() {
        let pid = PidBuilder::new()
            .kp(2.0)
            .ki(0.5)
            .kd(0.1)
            .dt(0.001)
            .output_limits(-5.0, 5.0)
            .build();

        assert_eq!(pid.get_kp(), 2.0);
        assert_eq!(pid.get_ki(), 0.5);
        assert_eq!(pid.get_kd(), 0.1);
    }

    #[test]
    fn test_pid_update_proportional_only() {
        let mut pid = PidController::new(1.0, 0.0, 0.0, 0.01, -10.0, 10.0);
        let output = pid.update(5.0, 2.0); // error = 3.0
        assert_eq!(output, 3.0); // Only proportional term
    }

    #[test]
    fn test_pid_update_with_integral() {
        let mut pid = PidController::new(1.0, 10.0, 0.0, 0.01, -10.0, 10.0);
        let output1 = pid.update(5.0, 2.0); // error = 3.0
        let output2 = pid.update(5.0, 2.0); // error = 3.0 again

        // Second output should be higher due to integral accumulation
        assert!(output2 > output1);
    }

    #[test]
    fn test_pid_output_limits() {
        let mut pid = PidController::new(10.0, 0.0, 0.0, 0.01, -5.0, 5.0);
        let output = pid.update(10.0, 0.0); // Large error, should be clamped
        assert_eq!(output, 5.0); // Should be clamped to max
    }

    #[test]
    fn test_pid_reset() {
        let mut pid = PidController::new(1.0, 1.0, 1.0, 0.01, -10.0, 10.0);

        // Accumulate some integral term
        pid.update(5.0, 2.0);
        pid.update(5.0, 2.0);

        // Integral term should be non-zero
        assert!(pid.get_integral_term() != 0.0);

        // Reset and check
        pid.reset();
        assert_eq!(pid.get_integral_term(), 0.0);
        assert_eq!(pid.get_derivative_term(), 0.0);
    }

    #[test]
    fn test_current_controller_pid() {
        let pid = PidController::new_current_controller();
        assert_eq!(pid.get_kp(), CURRENT_PID_KP);
        assert_eq!(pid.get_ki(), CURRENT_PID_KI);
        assert_eq!(pid.get_kd(), CURRENT_PID_KD);
    }
}
