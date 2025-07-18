use embassy_stm32::peripherals;
use embassy_stm32::timer::simple_pwm::SimplePwm;

pub struct Motors {
    right_motor: SimplePwm<'static, peripherals::TIM1>,
    left_motor_1: SimplePwm<'static, peripherals::TIM2>,
    left_motor_2: SimplePwm<'static, peripherals::TIM3>,
}

impl Motors {
    pub fn new(
        mut right_motor: SimplePwm<'static, peripherals::TIM1>,
        mut left_motor_1: SimplePwm<'static, peripherals::TIM2>,
        mut left_motor_2: SimplePwm<'static, peripherals::TIM3>,
    ) -> Self {
        // Enable all channels
        right_motor.ch3().enable();
        right_motor.ch4().enable();
        left_motor_1.ch2().enable();
        left_motor_2.ch4().enable();

        Self {
            right_motor,
            left_motor_1,
            left_motor_2,
        }
    }

    pub fn set_duty_left(&mut self, duty: f32) {
        let clamped_speed = self.clamp(duty, -1.0, 1.0);
        let duty_percent = (clamped_speed.abs() * 100.0) as u8;

        if clamped_speed > 0.0 {
            self.left_motor_2.ch4().set_duty_cycle_percent(0);
            self.left_motor_1.ch2().set_duty_cycle_percent(duty_percent);
        } else if clamped_speed < 0.0 {
            self.left_motor_2.ch4().set_duty_cycle_percent(duty_percent);
            self.left_motor_1.ch2().set_duty_cycle_percent(0);
        } else {
            self.left_motor_2.ch4().set_duty_cycle_percent(0);
            self.left_motor_1.ch2().set_duty_cycle_percent(0);
        }
    }

    pub fn set_duty_right(&mut self, duty: f32) {
        let clamped_speed = self.clamp(duty, -1.0, 1.0);
        let duty_percent = (clamped_speed.abs() * 100.0) as u8;

        if clamped_speed > 0.0 {
            self.right_motor.ch4().set_duty_cycle_percent(0);
            self.right_motor.ch3().set_duty_cycle_percent(duty_percent);
        } else if clamped_speed < 0.0 {
            self.right_motor.ch4().set_duty_cycle_percent(duty_percent);
            self.right_motor.ch3().set_duty_cycle_percent(0);
        } else {
            self.right_motor.ch4().set_duty_cycle_percent(0);
            self.right_motor.ch3().set_duty_cycle_percent(0);
        }
    }

    pub fn set_duty_both(&mut self, left_duty: f32, right_duty: f32) {
        self.set_duty_left(left_duty);
        self.set_duty_right(right_duty);
    }

    pub fn stop(&mut self) {
        self.set_duty_both(0.0, 0.0);
    }

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
