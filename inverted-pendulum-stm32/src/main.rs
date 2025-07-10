#![no_std]
#![no_main]

mod fmt;
mod motor;
mod qei;

use core::cell::RefCell;

use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    adc::Adc,
    bind_interrupts,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
    peripherals,
    rcc::AdcClockSource,
    time::Hertz,
    timer::simple_pwm::{PwmPin, SimplePwm},
};
use embassy_time::{Duration, Ticker, Timer};
use fmt::info;
use motor::Motors;
use qei::{Qei, QeiEncoding, QeiState};

bind_interrupts!(struct Irqs {
    ADC1_2 => embassy_stm32::adc::InterruptHandler<peripherals::ADC1>,
    embassy_stm32::adc::InterruptHandler<peripherals::ADC2>;
});

#[derive(Debug)]
struct AdcState {
    theta1: u16,
    theta2: u16,
    multiplexer: [u16; 8],
    current_r: u16,
    current_l: u16,
}

static ADC_STATE: Mutex<ThreadModeRawMutex, RefCell<AdcState>> =
    embassy_sync::mutex::Mutex::new(RefCell::new(AdcState {
        theta1: 0,
        theta2: 0,
        multiplexer: [0; 8],
        current_r: 0,
        current_l: 0,
    }));

static QEI_R_STATE: Mutex<ThreadModeRawMutex, RefCell<QeiState>> =
    embassy_sync::mutex::Mutex::new(RefCell::new(QeiState {
        pulses: 0,
        revolutions: 0,
        curr_state: 0,
        prev_state: 0,
    }));

static QEI_L_STATE: Mutex<ThreadModeRawMutex, RefCell<QeiState>> =
    embassy_sync::mutex::Mutex::new(RefCell::new(QeiState {
        pulses: 0,
        revolutions: 0,
        curr_state: 0,
        prev_state: 0,
    }));

static MOTORS: Mutex<ThreadModeRawMutex, RefCell<Option<Motors>>> =
    embassy_sync::mutex::Mutex::new(RefCell::new(None));

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

    // ADC1
    let mut adc1 = Adc::new(p.ADC1, Irqs);
    adc1.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES601_5);

    let multiplexer_ch_a = Output::new(p.PB4, Level::Low, Speed::Low);
    let multiplexer_ch_b = Output::new(p.PB3, Level::Low, Speed::Low);
    let multiplexer_ch_c = Output::new(p.PA12, Level::Low, Speed::Low);

    // ADC2
    let mut adc2 = Adc::new(p.ADC2, Irqs);
    adc2.set_sample_time(embassy_stm32::adc::SampleTime::CYCLES601_5);

    spawner.must_spawn(ticker_adc(
        adc1,
        p.PA0,
        p.PB0,
        p.PB1,
        multiplexer_ch_a,
        multiplexer_ch_b,
        multiplexer_ch_c,
        adc2,
        p.PA5,
        p.PA7,
    ));

    // Encoder - Use ExtiInput for interrupt-based QEI
    let enc_r_a = ExtiInput::new(p.PA6, p.EXTI6, Pull::None);
    let enc_r_b = ExtiInput::new(p.PA4, p.EXTI4, Pull::None);
    let enc_l_a = ExtiInput::new(p.PA8, p.EXTI8, Pull::Up);
    let enc_l_b = ExtiInput::new(p.PA9, p.EXTI9, Pull::Up);

    // Create QEI instances
    let qei_r = Qei::new(enc_r_a, enc_r_b, 1000, QeiEncoding::X4Encoding); // 1000 pulses per revolution
    let qei_l = Qei::new(enc_l_a, enc_l_b, 1000, QeiEncoding::X4Encoding); // 1000 pulses per revolution

    spawner.must_spawn(qei_encoder_r(qei_r));
    spawner.must_spawn(qei_encoder_l(qei_l));

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
        Hertz(100_000),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let mut motor_l_1 = SimplePwm::new(
        p.TIM2,
        None,
        Some(PwmPin::new_ch2(
            p.PA1,
            embassy_stm32::gpio::OutputType::PushPull,
        )),
        None,
        None,
        Hertz(100_000),
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
        Hertz(100_000),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );

    motor_l_1.ch1().enable();
    motor_l_1.ch1().set_duty_cycle_percent(80);

    // Initialize Motor Controller
    let motors = Motors::new(motor_r_1_2, motor_l_1, motor_l_2);
    MOTORS.lock().await.replace(Some(motors));

    spawner.must_spawn(ticker_control());

    loop {
        led.set_high();
        Timer::after(Duration::from_millis(100)).await;
        led.set_low();
        Timer::after(Duration::from_millis(100)).await;

        let adc_state = ADC_STATE.lock().await;
        let adc_state = adc_state.borrow();

        let qei_r_state = QEI_R_STATE.lock().await;
        let qei_r = qei_r_state.borrow();

        let qei_l_state = QEI_L_STATE.lock().await;
        let qei_l = qei_l_state.borrow();

        // Calculate angles from encoder pulses (assuming 1000 pulses per revolution)
        // Convert to degrees * 10 for display (e.g., 1234 means 123.4 degrees)
        let angle_r_x10 = (qei_r.pulses * 3600) / 1000;
        let angle_l_x10 = (qei_l.pulses * 3600) / 1000;

        info!(
            "ADC: theta1={}, theta2={} | QEI R: {} pulses, {}°x10 | QEI L: {} pulses, {}°x10",
            adc_state.theta1,
            adc_state.theta2,
            qei_r.pulses,
            angle_r_x10,
            qei_l.pulses,
            angle_l_x10
        );
    }
}

const MULTIPLEXER_CHANNELS: usize = 8;

#[embassy_executor::task]
async fn ticker_adc(
    mut adc1: Adc<'static, peripherals::ADC1>,
    mut adc1_in1: peripherals::PA0,
    mut adc1_in11: peripherals::PB0,
    mut adc1_in12: peripherals::PB1,
    mut multiplexer_ch_a: Output<'static>,
    mut multiplexer_ch_b: Output<'static>,
    mut multiplexer_ch_c: Output<'static>,
    mut adc2: Adc<'static, peripherals::ADC2>,
    mut adc2_in2: peripherals::PA5,
    mut adc2_in4: peripherals::PA7,
) {
    let mut ticker = Ticker::every(Duration::from_hz(1_000));

    let mut multiplexer_counter = 0;

    loop {
        // ADC1

        let multiplexer_value = adc1.read(&mut adc1_in1).await;
        let theta1 = adc1.read(&mut adc1_in11).await;
        let theta2 = adc1.read(&mut adc1_in12).await;

        let mut multiplexer = ADC_STATE.lock().await.borrow().multiplexer;
        multiplexer[multiplexer_counter] = multiplexer_value;

        // Set multiplexer channels
        multiplexer_counter += 1;
        if multiplexer_counter % MULTIPLEXER_CHANNELS == 0 {
            multiplexer_counter = 0;
        }

        multiplexer_ch_a.set_level(if multiplexer_counter & 0b001 != 0 {
            Level::High
        } else {
            Level::Low
        });
        multiplexer_ch_b.set_level(if multiplexer_counter & 0b010 != 0 {
            Level::High
        } else {
            Level::Low
        });
        multiplexer_ch_c.set_level(if multiplexer_counter & 0b100 != 0 {
            Level::High
        } else {
            Level::Low
        });

        // ADC2

        let current_r = adc2.read(&mut adc2_in2).await;
        let current_l = adc2.read(&mut adc2_in4).await;

        ADC_STATE.lock().await.replace(AdcState {
            theta1,
            theta2,
            multiplexer,
            current_r,
            current_l,
        });

        // Wait for the next ticker event

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn qei_encoder_r(mut qei: Qei<'static>) {
    qei.run_with_global_state(&QEI_R_STATE).await;
}

#[embassy_executor::task]
async fn qei_encoder_l(mut qei: Qei<'static>) {
    qei.run_with_global_state(&QEI_L_STATE).await;
}

#[embassy_executor::task]
async fn ticker_control() {
    let mut ticker = Ticker::every(Duration::from_millis(100));
    let mut counter = 0u32;

    loop {
        // Example motor control logic
        if let Some(motors) = MOTORS.lock().await.borrow_mut().as_mut() {
            match (counter / 10) % 4 {
                0 => motors.set_speed_both(0.0, 0.0),   // Stop
                1 => motors.set_speed_both(1.0, 1.0),   // Forward
                2 => motors.set_speed_both(0.0, 0.0),   // Stop
                3 => motors.set_speed_both(-1.0, -1.0), // Backward
                _ => {}
            }
        }

        counter += 1;
        ticker.next().await;
    }
}
