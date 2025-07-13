#![no_std]
#![no_main]

mod constants;
mod controller;
mod fmt;
mod lpf;
mod mit_adaptive_controller;
mod motor;
mod motor_observer;
mod pid;
mod sensor;

use core::sync::atomic::{AtomicBool, AtomicI32, Ordering};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use crate::{
    controller::ControllerSystem,
    sensor::{
        adc::{adc_task, Adc1Manager, Adc2Manager, get_theta0_radians, get_theta1_radians, get_motor_currents},
        qei::{Qei, QeiEncoding},
        SensorManager,
    },
};

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use constants::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    adc::Adc,
    bind_interrupts,
    exti::ExtiInput,
    gpio::{Input, Level, Output, Pull, Speed},
    peripherals,
    rcc::AdcClockSource,
    time::Hertz,
    timer::simple_pwm::{PwmPin, SimplePwm},
};
use embassy_time::{Duration, Ticker, Timer};
use fmt::info;
use motor::Motors;

bind_interrupts!(struct Irqs {
    ADC1_2 => embassy_stm32::adc::InterruptHandler<peripherals::ADC1>,
    embassy_stm32::adc::InterruptHandler<peripherals::ADC2>;
});

static QEI_R_PULSES: AtomicI32 = AtomicI32::new(0);
static QEI_L_PULSES: AtomicI32 = AtomicI32::new(0);
static START_CONTROL: AtomicBool = AtomicBool::new(false);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
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

    let p = embassy_stm32::init(config);

    // LED
    let mut led = Output::new(p.PB6, Level::Low, Speed::Low);

    // Button
    let button = Input::new(p.PB5, Pull::Up);

    // ADC1
    let mut adc1 = Adc::new(p.ADC1, Irqs);
    adc1.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES601_5);

    let multiplexer_ch_a = Output::new(p.PB4, Level::Low, Speed::Low);
    let multiplexer_ch_b = Output::new(p.PB3, Level::Low, Speed::Low);
    let multiplexer_ch_c = Output::new(p.PA12, Level::Low, Speed::Low);

    // ADC2
    let mut adc2 = Adc::new(p.ADC2, Irqs);
    adc2.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES601_5);

    let adc1_manager = Adc1Manager::new(adc1, multiplexer_ch_a, multiplexer_ch_b, multiplexer_ch_c);
    let adc2_manager = Adc2Manager::new(adc2);

    // Spawn ADC task
    spawner
        .spawn(adc_task(
            adc1_manager,
            adc2_manager,
            p.PB0, // theta1
            p.PB1, // theta2
            p.PA0, // multiplexer
            p.PA5, // current_r
            p.PA7, // current_l
        ))
        .unwrap();

    // Encoder - Use ExtiInput for interrupt-based QEI
    let enc_r_a = ExtiInput::new(p.PA6, p.EXTI6, Pull::None);
    let enc_r_b = ExtiInput::new(p.PA4, p.EXTI4, Pull::None);
    let enc_l_a = ExtiInput::new(p.PA8, p.EXTI8, Pull::Up);
    let enc_l_b = ExtiInput::new(p.PA9, p.EXTI9, Pull::Up);

    let qei_r = Qei::new(enc_r_a, enc_r_b, 1000, QeiEncoding::X4Encoding);
    let qei_l = Qei::new(enc_l_a, enc_l_b, 1000, QeiEncoding::X4Encoding);

    // Spawn encoder tasks
    spawner.spawn(qei_encoder_r(qei_r)).unwrap();
    spawner.spawn(qei_encoder_l(qei_l)).unwrap();

    // Motor Setup
    let motor_r_1_2 = SimplePwm::new(
        p.TIM1,
        None,
        None,
        Some(PwmPin::new_ch3(
            p.PA10,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Some(PwmPin::new_ch4(
            p.PA11,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Hertz(constants::MOTOR_PWM_FREQUENCY),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let motor_l_1 = SimplePwm::new(
        p.TIM2,
        None,
        Some(PwmPin::new_ch2(
            p.PA1,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        None,
        None,
        Hertz(constants::MOTOR_PWM_FREQUENCY),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let motor_l_2 = SimplePwm::new(
        p.TIM3,
        None,
        None,
        None,
        Some(PwmPin::new_ch4(
            p.PB7,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        Hertz(constants::MOTOR_PWM_FREQUENCY),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );

    // Initialize Motor Controller
    let motors = Motors::new(motor_r_1_2, motor_l_1, motor_l_2);

    // Spawn control task
    spawner.spawn(control_task(motors)).unwrap();

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(100)).await;
        led.set_low();
        Timer::after(Duration::from_millis(100)).await;

        // Check button state
        if button.is_low() {
            sensor::adc::calibrate_theta_offsets();
            START_CONTROL.store(true, Ordering::Relaxed);
        }
    }
}

// ADC task is now handled by the adc module

#[embassy_executor::task]
async fn qei_encoder_r(mut qei: Qei<'static>) {
    loop {
        let encode_result = qei.run().await;
        qei.reset();

        let prev_value = QEI_R_PULSES.load(Ordering::Relaxed);
        QEI_R_PULSES.store(prev_value + encode_result.pulses, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn qei_encoder_l(mut qei: Qei<'static>) {
    loop {
        let encode_result = qei.run().await;
        qei.reset();

        let prev_value = QEI_L_PULSES.load(Ordering::Relaxed);
        QEI_L_PULSES.store(prev_value + encode_result.pulses, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn control_task(mut motors: Motors) {
    let mut ticker = Ticker::every(Duration::from_hz(CONTROL_LOOP_FREQUENCY as u64));

    let mut sensor_data = SensorManager::new(1.0 / CONTROL_LOOP_FREQUENCY as f32);
    let mut controller_system = ControllerSystem::new_adaptive(1.0 / CONTROL_LOOP_FREQUENCY as f32);
    controller_system.reset();

    loop {
        if START_CONTROL.load(Ordering::Relaxed) {
            // Get ADC data directly without mutex
            let theta0 = get_theta0_radians();
            let theta1 = get_theta1_radians();
            let (current_r, current_l) = get_motor_currents();

            sensor_data.update_direct(
                theta0,
                theta1,
                current_r,
                current_l,
                QEI_R_PULSES.load(Ordering::Relaxed),
                QEI_L_PULSES.load(Ordering::Relaxed),
            );

            let result = controller_system.compute_control(&sensor_data);

            if let Ok(output) = result {
                motors.set_duty_both(output.duty_l, output.duty_r);
            } else {
                info!("Control computation failed");
            }
        } else {
            motors.stop();
        }

        ticker.next().await;
    }
}
