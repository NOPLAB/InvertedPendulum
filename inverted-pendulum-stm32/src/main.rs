#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    rcc::AdcClockSource,
    time::Hertz,
};
use embassy_time::{Duration, Timer};
use fmt::info;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc;

        // 64Mhz
        config.rcc.sys = rcc::Sysclk::PLL1_P;

        // 64Mhz
        config.rcc.pll = Some(rcc::Pll {
            src: rcc::PllSource::HSI,
            mul: rcc::PllMul::MUL16,
            prediv: rcc::PllPreDiv::DIV2,
        });

        // 64Mhz
        config.rcc.adc = AdcClockSource::Pll(rcc::AdcPllPrescaler::DIV1);

        // 64Mhz
        config.rcc.ahb_pre = rcc::AHBPrescaler::DIV1;

        // 32Mhz
        config.rcc.apb1_pre = rcc::APBPrescaler::DIV2;

        // 64Mhz
        config.rcc.apb2_pre = rcc::APBPrescaler::DIV1;

        config.rcc.mux.tim1sw = rcc::mux::Timsw::PCLK2_TIM;
        config.rcc.mux.tim2sw = rcc::mux::Tim2sw::PCLK1_TIM;
    }

    let mut p = embassy_stm32::init(config);

    let mut led = Output::new(p.PB6, Level::Low, Speed::Low);

    loop {
        info!("Hello, World!");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
