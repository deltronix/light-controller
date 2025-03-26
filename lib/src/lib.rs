#![no_std]
pub mod dsp {
    use num_traits::Float;
    pub struct EnvelopeDetector<const N_CHANNELS: usize> {
        attack: f32,
        release: f32,
        last_frame: [f32; N_CHANNELS],
    }

    pub struct TransientDetector<const N_CHANNELS: usize> {
        fast_attack: f32,
        fast_release: f32,
        slow_attack: f32,
        slow_release: f32,
        pub ma_input: [f32; N_CHANNELS],
        pub last_fast_frame: [f32; N_CHANNELS],
        pub last_slow_frame: [f32; N_CHANNELS],
        pub max_diff: [f32; N_CHANNELS],
        pub threshold_up: f32,
        pub threshold_down: f32,
        armed: [bool; N_CHANNELS],
        trig: [Option<bool>; N_CHANNELS],
    }

    impl<const N_CHANNELS: usize> TransientDetector<N_CHANNELS> {
        pub fn new(
            fast_attack: f32,
            fast_release: f32,
            slow_attack: f32,
            slow_release: f32,
            threshold_up: f32,
            threshold_down: f32,
        ) -> Self {
            Self {
                fast_attack: Self::calc_gain(fast_attack),
                fast_release: Self::calc_gain(fast_release),
                slow_attack: Self::calc_gain(slow_attack),
                slow_release: Self::calc_gain(slow_release),
                ma_input: [0.0; N_CHANNELS],
                last_fast_frame: [0.0; N_CHANNELS],
                last_slow_frame: [0.0; N_CHANNELS],
                max_diff: [0.0; N_CHANNELS],
                threshold_up,
                threshold_down,
                armed: [true; N_CHANNELS],
                trig: [None; N_CHANNELS],
            }
        }
        fn calc_gain(n_frames: f32) -> f32 {
            if n_frames == 0.0f32 {
                0.0f32
            } else {
                f32::powf(core::f32::consts::E, -1.0 / n_frames)
            }
        }

        pub fn process(&mut self, frame: &[f32; N_CHANNELS]) -> [Option<bool>; N_CHANNELS] {
            (0..N_CHANNELS).for_each(|ch| {
                let rectified = frame[ch].abs();
                let fast_diff = self.last_fast_frame[ch] - rectified;
                let slow_diff = self.last_slow_frame[ch] - rectified;
                if self.last_fast_frame[ch] < rectified {
                    self.last_fast_frame[ch] = rectified + (fast_diff * self.fast_attack)
                } else {
                    self.last_fast_frame[ch] = rectified + (fast_diff * self.fast_release)
                }
                if self.last_slow_frame[ch] < rectified {
                    self.last_slow_frame[ch] = rectified + (slow_diff * self.slow_attack)
                } else {
                    self.last_slow_frame[ch] = rectified + (slow_diff * self.slow_release)
                }

                let env_diff = self.last_fast_frame[ch] - self.last_slow_frame[ch];
                self.max_diff[ch] = env_diff;
                self.process_triggers();
            });
            self.trig
        }
        pub fn process_block(&mut self, block: &[[f32; N_CHANNELS]]) -> [Option<bool>; N_CHANNELS] {
            for frame in block {
                self.process(frame);
            }

            self.trig
        }
        fn process_triggers(&mut self) {
            (0..N_CHANNELS).for_each(|ch| {
                if self.armed[ch] && self.max_diff[ch] > self.threshold_up {
                    self.trig[ch] = Some(true);
                    self.armed[ch] = false;
                } else if !self.armed[ch] && self.max_diff[ch] < self.threshold_down {
                    self.trig[ch] = Some(false);
                    self.armed[ch] = true;
                } else {
                    self.trig[ch] = None;
                }
            });
        }
    }
}
