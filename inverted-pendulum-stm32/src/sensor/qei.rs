use embassy_futures::select;
use embassy_stm32::exti::ExtiInput;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum QeiEncoding {
    X2Encoding,
    X4Encoding,
}

#[derive(Debug, Clone, Copy)]
pub struct QeiResult {
    pub pulses: i32,
    pub revolutions: i32,
    pub curr_state: u8,
    pub prev_state: u8,
}

impl QeiResult {
    pub fn new() -> Self {
        Self {
            pulses: 0,
            revolutions: 0,
            curr_state: 0,
            prev_state: 0,
        }
    }
}

pub struct Qei<'a> {
    chan_a: ExtiInput<'a>,
    chan_b: ExtiInput<'a>,
    pulses_per_rev: i32,
    encoding: QeiEncoding,
    state: QeiResult,
}

impl<'a> Qei<'a> {
    pub fn new(
        chan_a: ExtiInput<'a>,
        chan_b: ExtiInput<'a>,
        pulses_per_rev: i32,
        encoding: QeiEncoding,
    ) -> Self {
        let mut qei = Self {
            chan_a,
            chan_b,
            pulses_per_rev,
            encoding,
            state: QeiResult::new(),
        };

        // Initialize current state
        qei.update_initial_state();
        qei
    }

    fn update_initial_state(&mut self) {
        let chan_a_val = if self.chan_a.is_high() { 1 } else { 0 };
        let chan_b_val = if self.chan_b.is_high() { 1 } else { 0 };
        let curr_state = (chan_a_val << 1) | chan_b_val;

        self.state.curr_state = curr_state;
        self.state.prev_state = curr_state;
    }

    pub async fn run(&mut self) -> QeiResult {
        // Wait for any edge on either channel
        select::select(
            self.chan_a.wait_for_any_edge(),
            self.chan_b.wait_for_any_edge(),
        )
        .await;

        // Process the encoder state change
        self.encode().await;

        self.state
    }

    pub fn reset(&mut self) {
        self.state.pulses = 0;
        self.state.revolutions = 0;
    }

    pub async fn get_state(&self) -> QeiResult {
        self.state
    }

    pub async fn encode(&mut self) {
        let chan_a_val = if self.chan_a.is_high() { 1 } else { 0 };
        let chan_b_val = if self.chan_b.is_high() { 1 } else { 0 };
        let curr_state = (chan_a_val << 1) | chan_b_val;

        self.state.curr_state = curr_state;

        match self.encoding {
            QeiEncoding::X2Encoding => {
                // X2 encoding: detect transitions on both edges of channel A
                if (self.state.prev_state == 0x3 && self.state.curr_state == 0x0)
                    || (self.state.prev_state == 0x0 && self.state.curr_state == 0x3)
                {
                    self.state.pulses += 1;
                } else if (self.state.prev_state == 0x2 && self.state.curr_state == 0x1)
                    || (self.state.prev_state == 0x1 && self.state.curr_state == 0x2)
                {
                    self.state.pulses -= 1;
                }
            }
            QeiEncoding::X4Encoding => {
                // X4 encoding: detect all transitions on both channels
                const QEI_INVALID: u8 = 0x3;
                const QEI_PREV_MASK: u8 = 0x1;
                const QEI_CURR_MASK: u8 = 0x2;

                if ((self.state.curr_state ^ self.state.prev_state) != QEI_INVALID)
                    && (self.state.curr_state != self.state.prev_state)
                {
                    let change = (self.state.prev_state & QEI_PREV_MASK)
                        ^ ((self.state.curr_state & QEI_CURR_MASK) >> 1);
                    if change == 0 {
                        self.state.pulses += 1;
                    } else {
                        self.state.pulses -= 1;
                    }
                }
            }
        }

        // Handle revolution counting
        if self.pulses_per_rev > 0 {
            if self.state.pulses >= self.pulses_per_rev {
                self.state.revolutions += 1;
                self.state.pulses -= self.pulses_per_rev;
            } else if self.state.pulses <= -self.pulses_per_rev {
                self.state.revolutions -= 1;
                self.state.pulses += self.pulses_per_rev;
            }
        }

        self.state.prev_state = self.state.curr_state;
    }

    pub fn encode_blocking(&mut self) {
        let chan_a_val = if self.chan_a.is_high() { 1 } else { 0 };
        let chan_b_val = if self.chan_b.is_high() { 1 } else { 0 };
        let curr_state = (chan_a_val << 1) | chan_b_val;

        self.state.curr_state = curr_state;

        match self.encoding {
            QeiEncoding::X2Encoding => {
                // X2 encoding: detect transitions on both edges of channel A
                if (self.state.prev_state == 0x3 && self.state.curr_state == 0x0)
                    || (self.state.prev_state == 0x0 && self.state.curr_state == 0x3)
                {
                    self.state.pulses += 1;
                } else if (self.state.prev_state == 0x2 && self.state.curr_state == 0x1)
                    || (self.state.prev_state == 0x1 && self.state.curr_state == 0x2)
                {
                    self.state.pulses -= 1;
                }
            }
            QeiEncoding::X4Encoding => {
                // X4 encoding: detect all transitions on both channels
                const QEI_INVALID: u8 = 0x3;
                const QEI_PREV_MASK: u8 = 0x1;
                const QEI_CURR_MASK: u8 = 0x2;

                if ((self.state.curr_state ^ self.state.prev_state) != QEI_INVALID)
                    && (self.state.curr_state != self.state.prev_state)
                {
                    let change = (self.state.prev_state & QEI_PREV_MASK)
                        ^ ((self.state.curr_state & QEI_CURR_MASK) >> 1);
                    if change == 0 {
                        self.state.pulses += 1;
                    } else {
                        self.state.pulses -= 1;
                    }
                }
            }
        }

        // Handle revolution counting
        if self.pulses_per_rev > 0 {
            if self.state.pulses >= self.pulses_per_rev {
                self.state.revolutions += 1;
                self.state.pulses -= self.pulses_per_rev;
            } else if self.state.pulses <= -self.pulses_per_rev {
                self.state.revolutions -= 1;
                self.state.pulses += self.pulses_per_rev;
            }
        }

        self.state.prev_state = self.state.curr_state;
    }

    pub async fn wait_for_any_edge(&mut self) {
        select::select(
            self.chan_a.wait_for_any_edge(),
            self.chan_b.wait_for_any_edge(),
        )
        .await;
    }
}
