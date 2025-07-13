#![no_std]
#![no_main]

mod adaptive_line_controller;
mod constants;
mod controller;
mod fmt;
mod line_controller;
mod lpf;
mod mit_adaptive_controller;
mod motor;
mod motor_observer;
mod pid;
mod sensor;

use core::sync::atomic::{AtomicI32, AtomicU32, AtomicU8, Ordering};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use crate::{
    controller::ControllerSystem,
    sensor::{
        adc::{
            adc_task, calibrate_theta_offsets, get_motor_currents, get_theta0_radians,
            get_theta1_radians, Adc1Manager, Adc2Manager,
        },
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
    gpio::{Level, Output, Pull, Speed},
    mode, peripherals,
    rcc::AdcClockSource,
    time::Hertz,
    timer::simple_pwm::{PwmPin, SimplePwm},
    usart::{self, Uart},
};
use embassy_time::{Duration, Ticker, WithTimeout};
use fmt::info;
use motor::Motors;

bind_interrupts!(struct Irqs {
    ADC1_2 => embassy_stm32::adc::InterruptHandler<peripherals::ADC1>,
    embassy_stm32::adc::InterruptHandler<peripherals::ADC2>;
    USART2 => embassy_stm32::usart::InterruptHandler<peripherals::USART2>;
});

static QEI_R_PULSES: AtomicI32 = AtomicI32::new(0);
static QEI_L_PULSES: AtomicI32 = AtomicI32::new(0);
static MODE: AtomicU8 = AtomicU8::new(0);

static UART: AtomicU32 = AtomicU32::new(0);
static THETA: AtomicU32 = AtomicU32::new(0);
static POSITION: AtomicU32 = AtomicU32::new(0);

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
    let mut button = ExtiInput::new(p.PB5, p.EXTI5, Pull::Up);

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

    let mut uart_config = usart::Config::default();
    uart_config.baudrate = 500000;
    let uart = Uart::new(
        p.USART2,
        p.PA15,
        p.PA2,
        Irqs,
        p.DMA1_CH7,
        p.DMA1_CH6,
        uart_config,
    )
    .unwrap();

    // Spawn control task
    spawner.spawn(control_task(motors)).unwrap();

    // Spawn UART task
    spawner.spawn(uart_task(uart)).unwrap();

    let mut button_pressed_count = 0;

    loop {
        // Wait for button press to start control
        button.wait_for_falling_edge().await;
        button_pressed_count += 1;

        // ADC
        calibrate_theta_offsets();

        match button_pressed_count {
            1 => {
                MODE.store(1, Ordering::Relaxed);
                led.set_high();
            }
            2 => {
                MODE.store(2, Ordering::Relaxed);
                led.set_high();
            }
            3 => {
                MODE.store(3, Ordering::Relaxed);
                led.set_high();
            }
            _ => {
                MODE.store(0, Ordering::Relaxed);
                led.set_low();
                button_pressed_count = 0; // Reset count after stopping control
            }
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
    let mut controller_system = ControllerSystem::new_lqr();
    controller_system.reset();
    let mut current_mode = 0u8;

    loop {
        let mode = MODE.load(Ordering::Relaxed);

        // Check if mode has changed and switch controller accordingly
        if mode != current_mode {
            match mode {
                1 => {
                    info!("Switching to LQR controller");
                    controller_system.switch_to_lqr();
                }
                2 => {
                    info!("Switching to Adaptive Line controller");
                    controller_system.switch_to_adaptive_line(1.0 / CONTROL_LOOP_FREQUENCY as f32);
                }
                3 => {
                    info!("Switching to UART controller");
                    controller_system.switch_to_uart();
                }
                _ => {
                    info!("Stopping control");
                }
            }
            current_mode = mode;
        }

        if mode != 0 {
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

            if mode == 1 {
                // LQR control
                let result = controller_system.compute_control(&sensor_data);
                if let Ok(output) = result {
                    motors.set_duty_both(output.duty_l, output.duty_r);
                } else {
                    info!("Control computation failed");
                }
            } else if mode == 2 {
                // New Adaptive Line Tracing control
                let result = controller_system.compute_control(&sensor_data);
                if let Ok(output) = result {
                    motors.set_duty_both(output.duty_l, output.duty_r);

                    // デバッグ情報の出力（必要に応じてコメントアウト）

                    /* if let Some(line_detected) = controller_system.is_adaptive_line_detected() {
                        if line_detected {
                            if let Some(line_pos) = controller_system.get_adaptive_line_position() {
                                info!("Line detected at position: {}", line_pos);
                            }
                        } else {
                            info!("No line detected");
                        }
                    } */
                } else {
                    info!("Adaptive Line control computation failed");
                }
            } else if mode == 3 {
                let position = (sensor_data.position_r + sensor_data.position_l) / 2.0;

                THETA.store(sensor_data.theta0.to_bits(), Ordering::Relaxed);
                POSITION.store(position.to_bits(), Ordering::Relaxed);

                let force = f32::from_bits(UART.load(Ordering::Relaxed) as u32);

                // Set force command to UART controller and compute control
                controller_system.set_uart_force_command(force);
                let result = controller_system.compute_control(&sensor_data);
                if let Ok(output) = result {
                    motors.set_duty_both(output.duty_l, output.duty_r);
                } else {
                    info!("UART control computation failed");
                }
            }
        } else {
            motors.stop();
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn uart_task(mut uart: Uart<'static, mode::Async>) {
    let mut buf = [0u8; 6];
    loop {
        if MODE.load(Ordering::Relaxed) != 3 {
            embassy_time::Timer::after(Duration::from_millis(200)).await;
            continue;
        }

        let theta = THETA.load(Ordering::Relaxed);
        let position = POSITION.load(Ordering::Relaxed);

        let data = [
            0x90,
            0x80,
            position as u8,
            (position >> 8) as u8,
            (position >> 16) as u8,
            (position >> 24) as u8,
            theta as u8,
            (theta >> 8) as u8,
            (theta >> 16) as u8,
            (theta >> 24) as u8,
        ];
        let _ = uart.write(&data).await;

        if uart
            .read(&mut buf)
            .with_timeout(Duration::from_millis(10))
            .await
            .is_ok()
        {
            if buf[0] == 0x90 && buf[1] == 0x80 {
                let force = f32::from_ne_bytes(buf[2..6].try_into().unwrap());

                UART.store(force.to_bits() as u32, Ordering::Relaxed);
            }
        }
    }
}
