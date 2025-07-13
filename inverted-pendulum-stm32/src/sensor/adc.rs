use crate::constants::*;
use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};
use embassy_futures::join::join;
use embassy_stm32::{adc::Adc, gpio::Output, peripherals};

/// ADC1 with multiplexer support (3 channels + 8 multiplexed channels)
pub struct Adc1Manager {
    adc: Adc<'static, peripherals::ADC1>,
    multiplexer_ch_a: Output<'static>,
    multiplexer_ch_b: Output<'static>,
    multiplexer_ch_c: Output<'static>,
    multiplexer_counter: usize,

    multiplexer_raw_values: [u16; MULTIPLEXER_CHANNELS],
}

impl Adc1Manager {
    pub fn new(
        adc: Adc<'static, peripherals::ADC1>,
        multiplexer_ch_a: Output<'static>,
        multiplexer_ch_b: Output<'static>,
        multiplexer_ch_c: Output<'static>,
    ) -> Self {
        Self {
            adc,
            multiplexer_ch_a,
            multiplexer_ch_b,
            multiplexer_ch_c,
            multiplexer_counter: 0,
            multiplexer_raw_values: [0; MULTIPLEXER_CHANNELS],
        }
    }

    pub async fn read_channels(
        &mut self,
        theta1_pin: &mut peripherals::PB0,
        theta2_pin: &mut peripherals::PB1,
        multiplexer_pin: &mut peripherals::PA0,
    ) -> Adc1Data {
        // Read direct channels
        let theta0_raw = self.adc.read(theta1_pin).await;
        let theta1_raw = self.adc.read(theta2_pin).await;

        // Read multiplexed channel
        let multiplexer_raw = self.adc.read(multiplexer_pin).await;

        // Store multiplexer data (C++ style - keep all channels)
        self.multiplexer_raw_values[self.multiplexer_counter] = multiplexer_raw;

        // Update multiplexer channel for next read
        self.update_multiplexer_channel();

        Adc1Data {
            theta0_raw,
            theta1_raw,
            multiplexer_raw: self.multiplexer_raw_values,
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
}

/// ADC2 for current sensing - optimized for embedded
pub struct Adc2Manager {
    adc: Adc<'static, peripherals::ADC2>,
}

impl Adc2Manager {
    pub fn new(adc: Adc<'static, peripherals::ADC2>) -> Self {
        Self { adc }
    }

    /// Read raw current values efficiently - returns raw ADC values for atomic storage
    pub async fn read_raw_currents(
        &mut self,
        current_r_pin: &mut peripherals::PA5,
        current_l_pin: &mut peripherals::PA7,
    ) -> (u16, u16) {
        let current_r_raw = self.adc.read(current_r_pin).await;
        let current_l_raw = self.adc.read(current_l_pin).await;
        (current_r_raw, current_l_raw)
    }
}

/// ADC1 data structure
#[derive(Debug, Clone, Copy)]
pub struct Adc1Data {
    pub theta0_raw: u16,                              // Raw ADC value for theta1
    pub theta1_raw: u16,                              // Raw ADC value for theta2
    pub multiplexer_raw: [u16; MULTIPLEXER_CHANNELS], // Raw ADC values for multiplexer channels
}

impl Adc1Data {
    pub const fn new() -> Self {
        Self {
            theta0_raw: 0,
            theta1_raw: 0,
            multiplexer_raw: [0; MULTIPLEXER_CHANNELS],
        }
    }

    fn get_theta0_radians(&self, zero_offset: u16) -> f32 {
        adc_to_radians(self.theta0_raw, zero_offset)
    }

    fn get_theta1_radians(&self, zero_offset: u16) -> f32 {
        adc_to_radians(self.theta1_raw, zero_offset)
    }

    fn get_vin_voltage(&self) -> f32 {
        let vin = self.multiplexer_raw[4];
        adc_to_voltage(vin)
    }
}

impl Default for Adc1Data {
    fn default() -> Self {
        Self::new()
    }
}

/// Raw ADC data structure - optimized for atomic operations
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct AdcRawData {
    pub theta0_raw: u16,
    pub theta1_raw: u16,
    pub current_r_raw: u16,
    pub current_l_raw: u16,
}

impl AdcRawData {
    pub const fn new() -> Self {
        Self {
            theta0_raw: 0,
            theta1_raw: 0,
            current_r_raw: 0,
            current_l_raw: 0,
        }
    }

    // Pack into two u32 for atomic operations
    pub fn pack_high(&self) -> u32 {
        ((self.theta0_raw as u32) << 16) | (self.theta1_raw as u32)
    }

    pub fn pack_low(&self) -> u32 {
        ((self.current_r_raw as u32) << 16) | (self.current_l_raw as u32)
    }

    // Unpack from two u32
    pub fn unpack(high: u32, low: u32) -> Self {
        Self {
            theta0_raw: (high >> 16) as u16,
            theta1_raw: high as u16,
            current_r_raw: (low >> 16) as u16,
            current_l_raw: low as u16,
        }
    }
}

/// Global ADC raw data - atomic for lock-free access (split into two u32)
pub static ADC_RAW_DATA_HIGH: AtomicU32 = AtomicU32::new(0); // theta0_raw | theta1_raw  
pub static ADC_RAW_DATA_LOW: AtomicU32 = AtomicU32::new(0);  // current_r_raw | current_l_raw

/// Theta offsets - static for one-time calibration
static THETA0_OFFSET: AtomicU16 = AtomicU16::new(0);
static THETA1_OFFSET: AtomicU16 = AtomicU16::new(0);

/// Current offsets - static for one-time calibration (zero current offsets)
static CURRENT_R_OFFSET: AtomicU16 = AtomicU16::new(0);
static CURRENT_L_OFFSET: AtomicU16 = AtomicU16::new(0);

/// Calibrate theta offsets (call once when button is pressed)
pub fn calibrate_theta_offsets() {
    let high = ADC_RAW_DATA_HIGH.load(Ordering::Relaxed);
    let low = ADC_RAW_DATA_LOW.load(Ordering::Relaxed);
    let raw_data = AdcRawData::unpack(high, low);
    THETA0_OFFSET.store(raw_data.theta0_raw, Ordering::Relaxed);
    THETA1_OFFSET.store(raw_data.theta1_raw, Ordering::Relaxed);
}

/// Calibrate current offsets (call once at startup for zero current calibration)
pub fn calibrate_current_offsets() {
    let high = ADC_RAW_DATA_HIGH.load(Ordering::Relaxed);
    let low = ADC_RAW_DATA_LOW.load(Ordering::Relaxed);
    let raw_data = AdcRawData::unpack(high, low);
    CURRENT_R_OFFSET.store(raw_data.current_r_raw, Ordering::Relaxed);
    CURRENT_L_OFFSET.store(raw_data.current_l_raw, Ordering::Relaxed);
}

/// Get theta0 in radians (optimized for real-time)
pub fn get_theta0_radians() -> f32 {
    let high = ADC_RAW_DATA_HIGH.load(Ordering::Relaxed);
    let low = ADC_RAW_DATA_LOW.load(Ordering::Relaxed);
    let raw_data = AdcRawData::unpack(high, low);
    let offset = THETA0_OFFSET.load(Ordering::Relaxed);
    adc_to_radians(raw_data.theta0_raw, offset)
}

/// Get theta1 in radians (optimized for real-time)
pub fn get_theta1_radians() -> f32 {
    let high = ADC_RAW_DATA_HIGH.load(Ordering::Relaxed);
    let low = ADC_RAW_DATA_LOW.load(Ordering::Relaxed);
    let raw_data = AdcRawData::unpack(high, low);
    let offset = THETA1_OFFSET.load(Ordering::Relaxed);
    adc_to_radians(raw_data.theta1_raw, offset)
}

/// Get motor currents (optimized for real-time)
pub fn get_motor_currents() -> (f32, f32) {
    let high = ADC_RAW_DATA_HIGH.load(Ordering::Relaxed);
    let low = ADC_RAW_DATA_LOW.load(Ordering::Relaxed);
    let raw_data = AdcRawData::unpack(high, low);
    let r_offset = CURRENT_R_OFFSET.load(Ordering::Relaxed);
    let l_offset = CURRENT_L_OFFSET.load(Ordering::Relaxed);

    // Convert with offset compensation
    let current_r = adc_to_current_with_offset(raw_data.current_r_raw, r_offset);
    let current_l = adc_to_current_with_offset(raw_data.current_l_raw, l_offset);
    (current_r, current_l)
}

/// ADC task function - optimized for embedded
#[embassy_executor::task]
pub async fn adc_task(
    mut adc1_manager: Adc1Manager,
    mut adc2_manager: Adc2Manager,
    mut theta1_pin: peripherals::PB0,
    mut theta2_pin: peripherals::PB1,
    mut multiplexer_pin: peripherals::PA0,
    mut current_r_pin: peripherals::PA5,
    mut current_l_pin: peripherals::PA7,
) -> ! {
    loop {
        // Read ADC channels efficiently
        let (adc1_data, (current_r_raw, current_l_raw)) = join(
            adc1_manager.read_channels(&mut theta1_pin, &mut theta2_pin, &mut multiplexer_pin),
            adc2_manager.read_raw_currents(&mut current_r_pin, &mut current_l_pin),
        )
        .await;

        // Create raw data structure
        let raw_data = AdcRawData {
            theta0_raw: adc1_data.theta0_raw,
            theta1_raw: adc1_data.theta1_raw,
            current_r_raw,
            current_l_raw,
        };

        // Store atomically - two u32 writes
        ADC_RAW_DATA_HIGH.store(raw_data.pack_high(), Ordering::Relaxed);
        ADC_RAW_DATA_LOW.store(raw_data.pack_low(), Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_adc_raw_data_pack_unpack() {
        let raw_data = AdcRawData {
            theta0_raw: 0x1234,
            theta1_raw: 0x5678,
            current_r_raw: 0x9ABC,
            current_l_raw: 0xDEF0,
        };

        let high = raw_data.pack_high();
        let low = raw_data.pack_low();
        let unpacked = AdcRawData::unpack(high, low);

        assert_eq!(raw_data.theta0_raw, unpacked.theta0_raw);
        assert_eq!(raw_data.theta1_raw, unpacked.theta1_raw);
        assert_eq!(raw_data.current_r_raw, unpacked.current_r_raw);
        assert_eq!(raw_data.current_l_raw, unpacked.current_l_raw);
    }

    #[test]
    fn test_theta_conversion() {
        // Test with some example values
        let test_adc = 2048; // Mid-range ADC value
        let test_offset = 1800; // Some offset
        let result = crate::constants::adc_to_radians(test_adc, test_offset);

        // Should get a reasonable angle value
        assert!(result.abs() < 10.0); // Reasonable range for pendulum angles
    }
}
