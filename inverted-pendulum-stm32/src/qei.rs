use embassy_stm32::exti::ExtiInput;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_futures::select;
use core::cell::RefCell;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum QeiEncoding {
    X2Encoding,
    X4Encoding,
}

#[derive(Debug, Clone, Copy)]
pub struct QeiState {
    pub pulses: i32,
    pub revolutions: i32,
    pub curr_state: u8,
    pub prev_state: u8,
}

impl QeiState {
    fn new() -> Self {
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
    state: Mutex<ThreadModeRawMutex, RefCell<QeiState>>,
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
            state: Mutex::new(RefCell::new(QeiState::new())),
        };
        
        // Initialize current state
        qei.update_initial_state();
        qei
    }

    fn update_initial_state(&mut self) {
        let chan_a_val = if self.chan_a.is_high() { 1 } else { 0 };
        let chan_b_val = if self.chan_b.is_high() { 1 } else { 0 };
        let curr_state = (chan_a_val << 1) | chan_b_val;
        
        // We can't use async lock here, so we'll use a blocking approach
        critical_section::with(|_| {
            if let Ok(state_ref) = self.state.try_lock() {
                let mut state = state_ref.borrow_mut();
                state.curr_state = curr_state;
                state.prev_state = curr_state;
            }
        });
    }

    pub async fn run(&mut self) {
        loop {
            // Wait for any edge on either channel
            select::select(
                self.chan_a.wait_for_any_edge(),
                self.chan_b.wait_for_any_edge(),
            )
            .await;

            // Process the encoder state change
            self.encode().await;
        }
    }

    pub async fn run_with_global_state(&mut self, global_state: &Mutex<ThreadModeRawMutex, RefCell<QeiState>>) {
        loop {
            // Wait for any edge on either channel
            select::select(
                self.chan_a.wait_for_any_edge(),
                self.chan_b.wait_for_any_edge(),
            )
            .await;

            // Process the encoder state change
            self.encode().await;
            
            // Update global state
            let local_state = self.get_state().await;
            let global_state_ref = global_state.lock().await;
            *global_state_ref.borrow_mut() = local_state;
        }
    }

    pub async fn reset(&self) {
        let state_ref = self.state.lock().await;
        let mut state = state_ref.borrow_mut();
        state.pulses = 0;
        state.revolutions = 0;
    }

    pub async fn get_pulses(&self) -> i32 {
        let state_ref = self.state.lock().await;
        let pulses = state_ref.borrow().pulses;
        pulses
    }

    pub async fn get_revolutions(&self) -> i32 {
        let state_ref = self.state.lock().await;
        let revolutions = state_ref.borrow().revolutions;
        revolutions
    }

    pub async fn get_state(&self) -> QeiState {
        let state_ref = self.state.lock().await;
        let state = *state_ref.borrow();
        state
    }

    pub async fn encode(&self) {
        let chan_a_val = if self.chan_a.is_high() { 1 } else { 0 };
        let chan_b_val = if self.chan_b.is_high() { 1 } else { 0 };
        let curr_state = (chan_a_val << 1) | chan_b_val;

        let state_ref = self.state.lock().await;
        let mut state = state_ref.borrow_mut();
        state.curr_state = curr_state;

        match self.encoding {
            QeiEncoding::X2Encoding => {
                // X2 encoding: detect transitions on both edges of channel A
                if (state.prev_state == 0x3 && state.curr_state == 0x0)
                    || (state.prev_state == 0x0 && state.curr_state == 0x3)
                {
                    state.pulses += 1;
                } else if (state.prev_state == 0x2 && state.curr_state == 0x1)
                    || (state.prev_state == 0x1 && state.curr_state == 0x2)
                {
                    state.pulses -= 1;
                }
            }
            QeiEncoding::X4Encoding => {
                // X4 encoding: detect all transitions on both channels
                const QEI_INVALID: u8 = 0x3;
                const QEI_PREV_MASK: u8 = 0x1;
                const QEI_CURR_MASK: u8 = 0x2;

                if ((state.curr_state ^ state.prev_state) != QEI_INVALID)
                    && (state.curr_state != state.prev_state)
                {
                    let change = (state.prev_state & QEI_PREV_MASK)
                        ^ ((state.curr_state & QEI_CURR_MASK) >> 1);
                    if change == 0 {
                        state.pulses += 1;
                    } else {
                        state.pulses -= 1;
                    }
                }
            }
        }

        // Handle revolution counting
        if self.pulses_per_rev > 0 {
            if state.pulses >= self.pulses_per_rev {
                state.revolutions += 1;
                state.pulses -= self.pulses_per_rev;
            } else if state.pulses <= -self.pulses_per_rev {
                state.revolutions -= 1;
                state.pulses += self.pulses_per_rev;
            }
        }

        state.prev_state = state.curr_state;
    }

    pub fn encode_blocking(&self) {
        let chan_a_val = if self.chan_a.is_high() { 1 } else { 0 };
        let chan_b_val = if self.chan_b.is_high() { 1 } else { 0 };
        let curr_state = (chan_a_val << 1) | chan_b_val;

        critical_section::with(|_| {
            if let Ok(state_ref) = self.state.try_lock() {
                let mut state = state_ref.borrow_mut();
                state.curr_state = curr_state;

                match self.encoding {
                    QeiEncoding::X2Encoding => {
                        // X2 encoding: detect transitions on both edges of channel A
                        if (state.prev_state == 0x3 && state.curr_state == 0x0)
                            || (state.prev_state == 0x0 && state.curr_state == 0x3)
                        {
                            state.pulses += 1;
                        } else if (state.prev_state == 0x2 && state.curr_state == 0x1)
                            || (state.prev_state == 0x1 && state.curr_state == 0x2)
                        {
                            state.pulses -= 1;
                        }
                    }
                    QeiEncoding::X4Encoding => {
                        // X4 encoding: detect all transitions on both channels
                        const QEI_INVALID: u8 = 0x3;
                        const QEI_PREV_MASK: u8 = 0x1;
                        const QEI_CURR_MASK: u8 = 0x2;

                        if ((state.curr_state ^ state.prev_state) != QEI_INVALID)
                            && (state.curr_state != state.prev_state)
                        {
                            let change = (state.prev_state & QEI_PREV_MASK)
                                ^ ((state.curr_state & QEI_CURR_MASK) >> 1);
                            if change == 0 {
                                state.pulses += 1;
                            } else {
                                state.pulses -= 1;
                            }
                        }
                    }
                }

                // Handle revolution counting
                if self.pulses_per_rev > 0 {
                    if state.pulses >= self.pulses_per_rev {
                        state.revolutions += 1;
                        state.pulses -= self.pulses_per_rev;
                    } else if state.pulses <= -self.pulses_per_rev {
                        state.revolutions -= 1;
                        state.pulses += self.pulses_per_rev;
                    }
                }

                state.prev_state = state.curr_state;
            }
        });
    }
}