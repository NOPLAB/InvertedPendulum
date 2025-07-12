use crate::constants::*;
use micromath::F32Ext;

/// Low-pass filter implementation
/// This implements a first-order low-pass filter with configurable cutoff frequency
#[derive(Debug, Clone, Copy)]
pub struct LowPassFilter {
    // Filter parameters
    alpha: f32,       // Filter coefficient (0 < alpha < 1)
    gain: f32,        // Filter gain
    cutoff_freq: f32, // Cutoff frequency in Hz
    sample_time: f32, // Sample time in seconds

    // Filter state
    output: f32,       // Current filter output
    initialized: bool, // Initialization flag
}

impl LowPassFilter {
    /// Create a new low-pass filter with specified cutoff frequency and gain
    pub fn new(sample_time: f32, cutoff_freq: f32, gain: f32) -> Self {
        let alpha = calculate_alpha(cutoff_freq, sample_time);

        Self {
            alpha,
            gain,
            cutoff_freq,
            sample_time,
            output: 0.0,
            initialized: false,
        }
    }

    /// Create a new low-pass filter with custom sample time
    pub fn new_with_sample_time(cutoff_freq: f32, gain: f32, sample_time: f32) -> Self {
        let alpha = calculate_alpha(cutoff_freq, sample_time);

        Self {
            alpha,
            gain,
            cutoff_freq,
            sample_time,
            output: 0.0,
            initialized: false,
        }
    }

    /// Process a single input sample through the filter
    pub fn update(&mut self, input: f32) -> f32 {
        if !self.initialized {
            // Initialize the filter with the first input value
            self.output = input * self.gain;
            self.initialized = true;
        } else {
            // Apply the low-pass filter equation:
            // y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
            self.output = self.alpha * input * self.gain + (1.0 - self.alpha) * self.output;
        }

        self.output
    }

    /// Reset the filter to initial state
    pub fn reset(&mut self) {
        self.output = 0.0;
        self.initialized = false;
    }

    /// Reset the filter with a specific initial value
    pub fn reset_with_value(&mut self, initial_value: f32) {
        self.output = initial_value * self.gain;
        self.initialized = true;
    }

    /// Get the current filter output
    pub fn get_output(&self) -> f32 {
        self.output
    }

    /// Check if the filter is initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Set the cutoff frequency (recalculates alpha)
    pub fn set_cutoff_frequency(&mut self, cutoff_freq: f32) {
        self.cutoff_freq = cutoff_freq;
        self.alpha = calculate_alpha(cutoff_freq, self.sample_time);
    }

    /// Set the filter gain
    pub fn set_gain(&mut self, gain: f32) {
        self.gain = gain;
    }

    /// Set the sample time (recalculates alpha)
    pub fn set_sample_time(&mut self, sample_time: f32) {
        self.sample_time = sample_time;
        self.alpha = calculate_alpha(self.cutoff_freq, sample_time);
    }

    /// Get the filter parameters
    pub fn get_parameters(&self) -> (f32, f32, f32) {
        (self.cutoff_freq, self.gain, self.sample_time)
    }

    /// Get the filter coefficient alpha
    pub fn get_alpha(&self) -> f32 {
        self.alpha
    }

    /// Get the filter time constant
    pub fn get_time_constant(&self) -> f32 {
        1.0 / (2.0 * core::f32::consts::PI * self.cutoff_freq)
    }

    /// Get the filter's 3dB bandwidth
    pub fn get_bandwidth(&self) -> f32 {
        self.cutoff_freq
    }

    /// Process multiple samples at once
    pub fn process_samples(&mut self, inputs: &[f32], outputs: &mut [f32]) {
        assert_eq!(inputs.len(), outputs.len());

        for (input, output) in inputs.iter().zip(outputs.iter_mut()) {
            *output = self.update(*input);
        }
    }

    /// Get the filter's frequency response at a given frequency
    pub fn frequency_response(&self, frequency: f32) -> f32 {
        let omega = 2.0 * core::f32::consts::PI * frequency;
        let tau = self.get_time_constant();
        let magnitude = 1.0 / (1.0 + (omega * tau).powi(2)).sqrt();
        magnitude * self.gain
    }

    /// Get the filter's phase response at a given frequency
    pub fn phase_response(&self, frequency: f32) -> f32 {
        let omega = 2.0 * core::f32::consts::PI * frequency;
        let tau = self.get_time_constant();
        -(omega * tau).atan()
    }
}

/// Butterworth low-pass filter (second-order)
#[derive(Debug, Clone, Copy)]
pub struct ButterworthFilter {
    // Filter coefficients
    a1: f32,
    a2: f32,
    b0: f32,
    b1: f32,
    b2: f32,

    // Filter state
    x1: f32, // Previous input
    x2: f32, // Previous previous input
    y1: f32, // Previous output
    y2: f32, // Previous previous output

    // Filter parameters
    cutoff_freq: f32,
    sample_time: f32,
    gain: f32,

    initialized: bool,
}

impl ButterworthFilter {
    /// Create a new second-order Butterworth filter
    pub fn new(sample_time: f32, cutoff_freq: f32, gain: f32) -> Self {
        let mut filter = Self {
            a1: 0.0,
            a2: 0.0,
            b0: 0.0,
            b1: 0.0,
            b2: 0.0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
            cutoff_freq,
            sample_time,
            gain,
            initialized: false,
        };

        filter.calculate_coefficients();
        filter
    }

    /// Calculate filter coefficients based on cutoff frequency
    fn calculate_coefficients(&mut self) {
        let wc = 2.0 * core::f32::consts::PI * self.cutoff_freq;
        let wc2 = wc * wc;
        let wc_sqrt2 = wc * 2.0_f32.sqrt();
        let dt = self.sample_time;
        let dt2 = dt * dt;

        // Bilinear transform coefficients
        let k = 2.0 / dt;
        let k2 = k * k;
        let k_sqrt2 = k * 2.0_f32.sqrt();

        let denominator = k2 + k_sqrt2 + wc2;

        // Calculate coefficients
        self.b0 = wc2 / denominator;
        self.b1 = 2.0 * wc2 / denominator;
        self.b2 = wc2 / denominator;

        self.a1 = (2.0 * wc2 - 2.0 * k2) / denominator;
        self.a2 = (k2 - k_sqrt2 + wc2) / denominator;

        // Apply gain
        self.b0 *= self.gain;
        self.b1 *= self.gain;
        self.b2 *= self.gain;
    }

    /// Update the filter with a new input sample
    pub fn update(&mut self, input: f32) -> f32 {
        if !self.initialized {
            // Initialize with steady-state values
            self.x1 = input;
            self.x2 = input;
            self.y1 = input * self.gain;
            self.y2 = input * self.gain;
            self.initialized = true;
            return input * self.gain;
        }

        // Apply the difference equation:
        // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
        let output = self.b0 * input + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2;

        // Update state variables
        self.x2 = self.x1;
        self.x1 = input;
        self.y2 = self.y1;
        self.y1 = output;

        output
    }

    /// Reset the filter
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
        self.initialized = false;
    }

    /// Reset with initial value
    pub fn reset_with_value(&mut self, initial_value: f32) {
        let initial_output = initial_value * self.gain;
        self.x1 = initial_value;
        self.x2 = initial_value;
        self.y1 = initial_output;
        self.y2 = initial_output;
        self.initialized = true;
    }

    /// Set cutoff frequency
    pub fn set_cutoff_frequency(&mut self, cutoff_freq: f32) {
        self.cutoff_freq = cutoff_freq;
        self.calculate_coefficients();
    }

    /// Set gain
    pub fn set_gain(&mut self, gain: f32) {
        self.gain = gain;
        self.calculate_coefficients();
    }

    /// Get current output
    pub fn get_output(&self) -> f32 {
        self.y1
    }

    /// Check if initialized
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }
}
