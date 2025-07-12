use crate::constants::*;
use crate::lpf::LowPassFilter;
use core::cell::RefCell;
use embassy_stm32::{adc::Adc, gpio::Output, peripherals};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::{Duration, Ticker};

/// ADC1 with multiplexer support (3 channels + 8 multiplexed channels)
pub struct Adc1Manager {
    adc: Adc<'static, peripherals::ADC1>,
    multiplexer_ch_a: Output<'static>,
    multiplexer_ch_b: Output<'static>,
    multiplexer_ch_c: Output<'static>,
    multiplexer_counter: usize,

    // Filters for each channel
    theta1_filter: LowPassFilter,
    theta2_filter: LowPassFilter,
    multiplexer_filters: [LowPassFilter; MULTIPLEXER_CHANNELS],

    // Calibration offsets
    theta1_offset: f32,
    theta2_offset: f32,
    multiplexer_offsets: [f32; MULTIPLEXER_CHANNELS],

    // Multiplexer data storage (like C++ muxAdcValues)
    multiplexer_values: [f32; MULTIPLEXER_CHANNELS],
    multiplexer_raw_values: [u16; MULTIPLEXER_CHANNELS],

    // Calibration status
    is_calibrated: bool,
    calibration_samples: u32,
    calibration_sums: [f32; MULTIPLEXER_CHANNELS + 2], // +2 for theta1 and theta2
}

impl Adc1Manager {
    pub fn new(
        adc: Adc<'static, peripherals::ADC1>,
        multiplexer_ch_a: Output<'static>,
        multiplexer_ch_b: Output<'static>,
        multiplexer_ch_c: Output<'static>,
        sample_time: f32,
    ) -> Self {
        Self {
            adc,
            multiplexer_ch_a,
            multiplexer_ch_b,
            multiplexer_ch_c,
            multiplexer_counter: 0,
            theta1_filter: LowPassFilter::new(sample_time, THETA_FILTER_CUTOFF_FREQ, 1.0),
            theta2_filter: LowPassFilter::new(sample_time, THETA_FILTER_CUTOFF_FREQ, 1.0),
            multiplexer_filters: [LowPassFilter::new(sample_time, DEFAULT_FILTER_CUTOFF_FREQ, 1.0);
                MULTIPLEXER_CHANNELS],
            theta1_offset: 0.0,
            theta2_offset: 0.0,
            multiplexer_offsets: [0.0; MULTIPLEXER_CHANNELS],
            multiplexer_values: [0.0; MULTIPLEXER_CHANNELS],
            multiplexer_raw_values: [0; MULTIPLEXER_CHANNELS],
            is_calibrated: false,
            calibration_samples: 0,
            calibration_sums: [0.0; MULTIPLEXER_CHANNELS + 2],
        }
    }

    pub async fn read_channels(
        &mut self,
        theta1_pin: &mut peripherals::PB0,
        theta2_pin: &mut peripherals::PB1,
        multiplexer_pin: &mut peripherals::PA0,
    ) -> Adc1Data {
        // Read direct channels
        let theta1_raw = self.adc.read(theta1_pin).await;
        let theta2_raw = self.adc.read(theta2_pin).await;

        // Read multiplexed channel
        let multiplexer_raw = self.adc.read(multiplexer_pin).await;

        // Convert to voltage
        let theta1_voltage = adc_to_voltage(theta1_raw);
        let theta2_voltage = adc_to_voltage(theta2_raw);
        let multiplexer_voltage = adc_to_voltage(multiplexer_raw);

        // Apply calibration offsets
        let theta1_calibrated = theta1_voltage - self.theta1_offset;
        let theta2_calibrated = theta2_voltage - self.theta2_offset;
        let multiplexer_calibrated =
            multiplexer_voltage - self.multiplexer_offsets[self.multiplexer_counter];

        // Apply filters
        let theta1_filtered = self.theta1_filter.update(theta1_calibrated);
        let theta2_filtered = self.theta2_filter.update(theta2_calibrated);
        let multiplexer_filtered =
            self.multiplexer_filters[self.multiplexer_counter].update(multiplexer_calibrated);

        // Store multiplexer data (C++ style - keep all channels)
        self.multiplexer_values[self.multiplexer_counter] = multiplexer_filtered;
        self.multiplexer_raw_values[self.multiplexer_counter] = multiplexer_raw;

        // Update multiplexer channel for next read
        self.update_multiplexer_channel();

        // Update calibration if in progress
        if !self.is_calibrated {
            self.update_calibration(theta1_voltage, theta2_voltage, multiplexer_voltage);
        }

        /*         info!(
                   "ADC1 Read: theta1: {} V, theta2: {} V, multiplexer: {} V, channel: {}",
                   theta1_voltage, theta2_voltage, multiplexer_voltage, self.multiplexer_counter
               );
        */
        Adc1Data {
            theta0: theta1_filtered,
            theta1: theta2_filtered,
            multiplexer: self.multiplexer_values, // Return all multiplexer values
        }
    }

    fn update_multiplexer_channel(&mut self) {
        // Move to next multiplexer channel
        self.multiplexer_counter = (self.multiplexer_counter + 1) % MULTIPLEXER_CHANNELS;

        // Set multiplexer control pins
        use embassy_stm32::gpio::Level;

        self.multiplexer_ch_a
            .set_level(if self.multiplexer_counter & 0b001 != 0 {
                Level::High
            } else {
                Level::Low
            });

        self.multiplexer_ch_b
            .set_level(if self.multiplexer_counter & 0b010 != 0 {
                Level::High
            } else {
                Level::Low
            });

        self.multiplexer_ch_c
            .set_level(if self.multiplexer_counter & 0b100 != 0 {
                Level::High
            } else {
                Level::Low
            });
    }

    fn update_calibration(&mut self, theta1: f32, theta2: f32, multiplexer: f32) {
        const CALIBRATION_SAMPLES: u32 = 100;

        if self.calibration_samples < CALIBRATION_SAMPLES {
            self.calibration_sums[0] += theta1;
            self.calibration_sums[1] += theta2;
            self.calibration_sums[2 + self.multiplexer_counter] += multiplexer;
            self.calibration_samples += 1;
        } else {
            // Finish calibration
            self.theta1_offset = self.calibration_sums[0] / (CALIBRATION_SAMPLES as f32);
            self.theta2_offset = self.calibration_sums[1] / (CALIBRATION_SAMPLES as f32);

            // Disable multiplexer channel offsets for now
            /* for i in 0..MULTIPLEXER_CHANNELS {
                self.multiplexer_offsets[i] = self.calibration_sums[2 + i]
                    / (CALIBRATION_SAMPLES as f32 / MULTIPLEXER_CHANNELS as f32);
            } */

            self.is_calibrated = true;
        }
    }

    pub fn start_calibration(&mut self) {
        self.is_calibrated = false;
        self.calibration_samples = 0;
        self.calibration_sums = [0.0; MULTIPLEXER_CHANNELS + 2];
    }

    pub fn is_calibrated(&self) -> bool {
        self.is_calibrated
    }

    pub fn get_calibration_progress(&self) -> f32 {
        const CALIBRATION_SAMPLES: u32 = 1000;
        (self.calibration_samples as f32) / (CALIBRATION_SAMPLES as f32)
    }

    pub fn set_filter_cutoff(&mut self, cutoff_freq: f32) {
        self.theta1_filter.set_cutoff_frequency(cutoff_freq);
        self.theta2_filter.set_cutoff_frequency(cutoff_freq);
        for filter in &mut self.multiplexer_filters {
            filter.set_cutoff_frequency(cutoff_freq);
        }
    }

    pub fn reset_filters(&mut self) {
        self.theta1_filter.reset();
        self.theta2_filter.reset();
        for filter in &mut self.multiplexer_filters {
            filter.reset();
        }
    }

    pub fn get_offsets(&self) -> (f32, f32, [f32; MULTIPLEXER_CHANNELS]) {
        (
            self.theta1_offset,
            self.theta2_offset,
            self.multiplexer_offsets,
        )
    }
}

/// ADC2 for current sensing (simplified - no filtering)
pub struct Adc2Manager {
    adc: Adc<'static, peripherals::ADC2>,

    // Calibration offsets
    current_r_offset: f32,
    current_l_offset: f32,

    // Calibration status
    is_calibrated: bool,
    calibration_samples: u32,
    calibration_sums: [f32; 2],
}

impl Adc2Manager {
    pub fn new(adc: Adc<'static, peripherals::ADC2>, sample_time: f32) -> Self {
        Self {
            adc,
            current_r_offset: 0.0, // Will be calibrated
            current_l_offset: 0.0, // Will be calibrated
            is_calibrated: false,
            calibration_samples: 0,
            calibration_sums: [0.0; 2],
        }
    }

    pub async fn read_currents(
        &mut self,
        current_r_pin: &mut peripherals::PA5,
        current_l_pin: &mut peripherals::PA7,
    ) -> Adc2Data {
        // Read current channels
        let current_r_raw = self.adc.read(current_r_pin).await;
        let current_l_raw = self.adc.read(current_l_pin).await;

        // Convert to current using C++ implementation (no filtering here)
        let current_r_calibrated = adc_to_current(current_r_raw);
        let current_l_calibrated = adc_to_current(current_l_raw);

        // Update calibration if in progress
        if !self.is_calibrated {
            let current_r_voltage = adc_to_voltage(current_r_raw);
            let current_l_voltage = adc_to_voltage(current_l_raw);
            self.update_calibration(current_r_calibrated, current_l_calibrated);
        }

        Adc2Data {
            current_r: current_r_calibrated - self.current_r_offset,
            current_l: current_l_calibrated - self.current_l_offset,
        }
    }

    fn update_calibration(&mut self, current_r: f32, current_l: f32) {
        const CALIBRATION_SAMPLES: u32 = 1000;

        if self.calibration_samples < CALIBRATION_SAMPLES {
            self.calibration_sums[0] += current_r;
            self.calibration_sums[1] += current_l;
            self.calibration_samples += 1;
        } else {
            // Finish calibration (assuming zero current during calibration)
            self.current_r_offset = self.calibration_sums[0] / (CALIBRATION_SAMPLES as f32);
            self.current_l_offset = self.calibration_sums[1] / (CALIBRATION_SAMPLES as f32);
            self.is_calibrated = true;
        }
    }

    pub fn start_calibration(&mut self) {
        self.is_calibrated = false;
        self.calibration_samples = 0;
        self.calibration_sums = [0.0; 2];
    }

    pub fn is_calibrated(&self) -> bool {
        self.is_calibrated
    }

    pub fn get_calibration_progress(&self) -> f32 {
        const CALIBRATION_SAMPLES: u32 = 1000;
        (self.calibration_samples as f32) / (CALIBRATION_SAMPLES as f32)
    }

    // Filtering removed - handled in CurrentController

    pub fn get_offsets(&self) -> (f32, f32) {
        (self.current_r_offset, self.current_l_offset)
    }
}

/// ADC1 data structure
#[derive(Debug, Clone, Copy)]
pub struct Adc1Data {
    pub theta0: f32,                              // Filtered theta1 [V]
    pub theta1: f32,                              // Filtered theta2 [V]
    pub multiplexer: [f32; MULTIPLEXER_CHANNELS], // Filtered multiplexer channels [V]
}

impl Adc1Data {
    pub fn new() -> Self {
        Self {
            theta0: 0.0,
            theta1: 0.0,
            multiplexer: [0.0; MULTIPLEXER_CHANNELS],
        }
    }

    /// Get theta1 in degrees
    pub fn get_theta0_degrees(&self) -> f32 {
        self.theta0 * ADC_TO_DEG
    }

    /// Get theta2 in degrees
    pub fn get_theta1_degrees(&self) -> f32 {
        self.theta1 * ADC_TO_DEG
    }

    /// Get theta1 in radians
    pub fn get_theta1_radians(&self) -> f32 {
        -self.theta0 * ADC_TO_RAD // C++版と同じく読み取り時に符号変更
    }

    /// Get theta2 in radians
    pub fn get_theta2_radians(&self) -> f32 {
        self.theta1 * ADC_TO_RAD
    }

    /// Get multiplexer channel value (C++ style interface)
    pub fn get_multiplexer_channel(&self, channel: usize) -> f32 {
        if channel < MULTIPLEXER_CHANNELS {
            self.multiplexer[channel]
        } else {
            0.0
        }
    }

    /// Get all multiplexer values as array (C++ style mux_value)
    pub fn get_mux_value(&self) -> &[f32; MULTIPLEXER_CHANNELS] {
        &self.multiplexer
    }

    /// Get multiplexer value at specific index (C++ style mux_value[i])
    pub fn get_mux_value_at(&self, index: usize) -> f32 {
        if index < MULTIPLEXER_CHANNELS {
            self.multiplexer[index]
        } else {
            0.0
        }
    }
}

impl Default for Adc1Data {
    fn default() -> Self {
        Self::new()
    }
}

/// ADC2 data structure
#[derive(Debug, Clone, Copy)]
pub struct Adc2Data {
    pub current_r: f32, // Filtered right motor current [A]
    pub current_l: f32, // Filtered left motor current [A]
}

impl Adc2Data {
    pub fn new() -> Self {
        Self {
            current_r: 0.0,
            current_l: 0.0,
        }
    }
}

impl Default for Adc2Data {
    fn default() -> Self {
        Self::new()
    }
}

/// Combined ADC data structure
#[derive(Debug, Clone, Copy)]
pub struct AdcData {
    pub adc1: Adc1Data,
    pub adc2: Adc2Data,
}

impl AdcData {
    pub fn new() -> Self {
        Self {
            adc1: Adc1Data::new(),
            adc2: Adc2Data::new(),
        }
    }

    /// Get sensor angles in degrees (for validation and display)
    pub fn get_sensor_angles_degrees(&self) -> (f32, f32) {
        (
            self.adc1.get_theta0_degrees(),
            self.adc1.get_theta1_degrees(),
        )
    }

    /// Get sensor angles in radians (for control algorithms)
    pub fn get_sensor_angles_radians(&self) -> (f32, f32) {
        (
            self.adc1.get_theta1_radians(),
            self.adc1.get_theta2_radians(),
        )
    }

    pub fn get_motor_currents(&self) -> (f32, f32) {
        (self.adc2.current_r, self.adc2.current_l)
    }

    pub fn get_vin_voltage(&self) -> f32 {
        // Assuming ADC1 channel 0 is the input voltage
        let vin = self.adc1.multiplexer[4];
        vin * ADC_TO_VOLTAGE_C
    }
}

impl Default for AdcData {
    fn default() -> Self {
        Self::new()
    }
}

/// Global ADC state for sharing between tasks
pub static ADC_DATA: Mutex<ThreadModeRawMutex, RefCell<AdcData>> =
    Mutex::new(RefCell::new(AdcData {
        adc1: Adc1Data {
            theta0: 0.0,
            theta1: 0.0,
            multiplexer: [0.0; MULTIPLEXER_CHANNELS],
        },
        adc2: Adc2Data {
            current_r: 0.0,
            current_l: 0.0,
        },
    }));

/// ADC task function
#[embassy_executor::task]
pub async fn adc_task(
    mut adc1_manager: Adc1Manager,
    mut adc2_manager: Adc2Manager,
    mut theta1_pin: peripherals::PB0,
    mut theta2_pin: peripherals::PB1,
    mut multiplexer_pin: peripherals::PA0,
    mut current_r_pin: peripherals::PA5,
    mut current_l_pin: peripherals::PA7,
    sample_time: Duration,
) -> ! {
    let mut ticker = Ticker::every(sample_time);

    loop {
        // Read ADC channels
        let adc1_data = adc1_manager
            .read_channels(&mut theta1_pin, &mut theta2_pin, &mut multiplexer_pin)
            .await;
        let adc2_data = adc2_manager
            .read_currents(&mut current_r_pin, &mut current_l_pin)
            .await;

        // Combine ADC data
        let combined_data = AdcData {
            adc1: adc1_data,
            adc2: adc2_data,
        };

        // Store in global state
        ADC_DATA.lock().await.replace(combined_data);

        // Wait for next tick
        ticker.next().await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_adc_data_creation() {
        let adc_data = AdcData::new();
        assert_eq!(adc_data.timestamp, 0);
        assert!(adc_data.is_data_valid());
    }

    #[test]
    fn test_adc1_data_angles() {
        let mut adc1_data = Adc1Data::new();
        adc1_data.theta0 = 1.65; // Mid-scale voltage
        adc1_data.theta1 = 0.825; // Quarter-scale voltage
        adc1_data.theta1_raw = 2047; // Mid-scale raw value
        adc1_data.theta2_raw = 1023; // Quarter-scale raw value

        let angle1_deg = adc1_data.get_theta0_degrees();
        let angle2_deg = adc1_data.get_theta1_degrees();
        let angle1_rad = adc1_data.get_theta1_radians();
        let angle2_rad = adc1_data.get_theta2_radians();

        // Check that angles are calculated
        assert!(angle1_deg.abs() < 180.0);
        assert!(angle2_deg.abs() < 180.0);
        assert!(angle1_rad.abs() < core::f32::consts::PI);
        assert!(angle2_rad.abs() < core::f32::consts::PI);
    }

    #[test]
    fn test_adc2_data_currents() {
        let mut adc2_data = Adc2Data::new();
        adc2_data.current_r = 2.0;
        adc2_data.current_l = 1.0;

        assert_eq!(adc2_data.get_total_current(), 3.0);
        assert_eq!(adc2_data.get_current_difference(), 1.0);
        assert!(!adc2_data.is_current_saturated());
    }

    #[test]
    fn test_current_saturation() {
        let mut adc2_data = Adc2Data::new();
        adc2_data.current_r = MAX_CURRENT + 1.0;

        assert!(adc2_data.is_current_saturated());
    }

    #[test]
    fn test_data_validity() {
        let mut adc_data = AdcData::new();
        adc_data.adc1.theta1_raw = 2000; // Valid raw value
        adc_data.adc1.theta2_raw = 2000;
        adc_data.adc1.multiplexer_raw = 2000;
        adc_data.adc2.current_r_raw = 2000;
        adc_data.adc2.current_l_raw = 2000;

        assert!(adc_data.is_data_valid());

        // Test invalid raw value
        adc_data.adc1.theta1_raw = 0; // Invalid (too low)
        assert!(!adc_data.is_data_valid());
    }

    #[test]
    fn test_sensor_status() {
        let mut adc_data = AdcData::new();
        adc_data.adc1.theta1_raw = 2000;
        adc_data.adc1.theta2_raw = 2000;
        adc_data.adc1.multiplexer_raw = 2000;
        adc_data.adc2.current_r_raw = 2000;
        adc_data.adc2.current_l_raw = 2000;
        adc_data.adc2.current_r = 1.0;
        adc_data.adc2.current_l = 1.0;

        let status = adc_data.get_sensor_status();
        assert!(status.is_all_valid());
        assert_eq!(status.get_error_flags(), 0);
    }

    #[test]
    fn test_mux_value_interface() {
        let mut adc1_data = Adc1Data::new();
        adc1_data.multiplexer[4] = 2.5; // Set channel 4 to 2.5V

        let mux_values = adc1_data.get_mux_value();
        assert_eq!(mux_values[4], 2.5);
        assert_eq!(adc1_data.get_mux_value_at(4), 2.5);
        assert_eq!(adc1_data.get_mux_value_at(99), 0.0); // Out of bounds
    }
}
