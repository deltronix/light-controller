#![no_std]

pub mod dsp {
    use dasp::ring_buffer::Fixed;
    pub struct TransientDetector<const N_CHANNELS: usize, const BLOCK_SIZE: usize> {
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

    impl<const N_CHANNELS: usize, const BLOCK_SIZE: usize> TransientDetector<N_CHANNELS, BLOCK_SIZE> {
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
        pub fn calc_gain(n_frames: f32) -> f32 {
            if n_frames == 0.0f32 {
                0.0f32
            } else {
                f32::powf(core::f32::consts::E, -1.0 / n_frames)
            }
        }

        pub fn process_block(&mut self, block: &[[f32; N_CHANNELS]]) -> [Option<bool>; N_CHANNELS] {
            self.max_diff = [0.0; N_CHANNELS];
            self.trig = [None; N_CHANNELS];
            for frame in block {
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
                    //self.max_diff[ch] = (self.max_diff[ch] + env_diff) * 0.5;
                    self.max_diff[ch] = env_diff;
                });
            }
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

            self.trig
        }
    }
    // let Detector {
    //             attack_gain,
    //             release_gain,
    //             ref mut detect,
    //             ref mut last_env_frame,
    //         } = *self;
    //
    //         let detected_frame = detect.detect(frame);
    //         let new_env_frame = last_env_frame.zip_map(detected_frame, |l, d| {
    //             let gain = if l < d { attack_gain } else { release_gain };
    //             let diff = l.add_amp(-d.to_signed_sample());
    //             d.add_amp(diff.mul_amp(gain.to_sample()).to_sample())
    //         });
    //         *last_env_frame = new_env_frame;
    //         new_env_frame
    //
    //     }
}

#[cfg(target_arch = "arm")]
mod setup {
    use embassy_stm32::Config;
    use embassy_stm32::Peripherals;

    fn clock_config(mut config: Config) -> Peripherals {
        use embassy_stm32::rcc::*;
        //let mut config = Config::default();

        //config.rcc.hsi = Some(HSIPrescaler::DIV2);
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,   // 64 MHz
            prediv: PllPreDiv::DIV4,  // 16 MHz
            mul: PllMul::MUL50,       // 800 MHz
            divp: Some(PllDiv::DIV2), // 400 MHz
            divq: None,
            divr: None,
        });
        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSI,   // 64 MHz
            prediv: PllPreDiv::DIV4,  // 16 MHz
            mul: PllMul::MUL50,       // 800 MHz
            divp: Some(PllDiv::DIV8), // 100MHz
            divq: None,
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 MHz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV4; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV4; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV4; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV4; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.mux.adcsel = mux::Adcsel::PLL2_P;

        embassy_stm32::init(config)
    }
}
