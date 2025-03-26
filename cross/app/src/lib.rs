#![no_std]

use embassy_stm32::dac::Dac;
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::TIM12;
use embassy_stm32::timer::low_level::Timer;
use embassy_stm32::{
    Peripherals,
    adc::{Adc, AdcChannel, AnyAdcChannel, SampleTime},
    dma::NoDma,
    gpio::{Level, Output, OutputType, Speed},
    pac::{self as pac},
    peripherals::{ADC2, DAC1, TIM6},
    spi::Spi,
    time::Hertz,
    timer::simple_pwm::{PwmPin, SimplePwm},
};
use p9813::P9813;
const SAMPLE_RATE: Hertz = Hertz(8_000);

const ADC2_DMA_REQ: u8 = 10;

const INPUT_CHANNELS: usize = 8;
const OUTPUT_CHANNELS: usize = 2;
const BLOCK_SIZE: usize = 32;
const DMA_BUFFER_SIZE: usize = BLOCK_SIZE * 2;

type DacDmaBuffer = [u32; DMA_BUFFER_SIZE];
type AdcDmaBuffer = [u16; INPUT_CHANNELS * DMA_BUFFER_SIZE];

pub struct StatusLeds {
    pub ld1: Output<'static>,
    pub ld2: Output<'static>,
    pub ld3: SimplePwm<'static, TIM12>,
}

pub struct Board {
    pub status_leds: StatusLeds,
    pub dac1: Dac<'static, DAC1>,
    pub adc2: Adc<'static, ADC2>,
    pub analog_clock: Timer<'static, TIM6>,
    pub p9813: P9813<Spi<'static, Blocking>>,
}

impl Board {
    pub fn init(p: Peripherals) -> Self {
        let status_leds = {
            let ld1 = Output::new(p.PB0, Level::High, Speed::Low);
            let ld2 = Output::new(p.PE1, Level::High, Speed::Low);
            //let mut ld3 = Output::new(p.PB14, Level::High, Speed::Low);
            let ld3 = {
                let pin = PwmPin::new_ch1(p.PB14, OutputType::PushPull);
                SimplePwm::new(
                    p.TIM12,
                    Some(pin),
                    None,
                    None,
                    None,
                    Hertz(1000),
                    embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
                )
            };
            StatusLeds { ld1, ld2, ld3 }
        };
        let dac1 = {
            let dac1_1 = p.PA4;
            let dac1_2 = p.PA5;

            use embassy_stm32::dac::*;
            use embassy_stm32::pac::DAC1;
            let mut dac = Dac::new(p.DAC1, NoDma, NoDma, dac1_1, dac1_2);

            dac.ch1().set_trigger(TriggerSel::Tim6);
            dac.ch2().set_trigger(TriggerSel::Tim6);
            dac.ch1().set_triggering(true);
            dac.ch2().set_triggering(true);
            dac.ch1()
                .set(embassy_stm32::dac::Value::Bit12Left(u16::MAX / 2));
            dac.ch2()
                .set(embassy_stm32::dac::Value::Bit12Left(u16::MAX / 2));
            DAC1.cr().modify(|r| r.set_dmaen(0, true));
            dac.ch1().enable();
            dac.ch2().enable();

            dac
        };
        let adc2 = {
            let _adc2_2: AnyAdcChannel<ADC2> = p.PF13.degrade_adc();
            let _adc2_3: AnyAdcChannel<ADC2> = p.PA6.degrade_adc();
            let _adc2_4: AnyAdcChannel<ADC2> = p.PC4.degrade_adc();
            let _adc2_5: AnyAdcChannel<ADC2> = p.PB1.degrade_adc();
            let _adc2_6: AnyAdcChannel<ADC2> = p.PF14.degrade_adc();
            let _adc2_7: AnyAdcChannel<ADC2> = p.PA7.degrade_adc();
            let _adc2_8: AnyAdcChannel<ADC2> = p.PC5.degrade_adc();
            let _adc2_11: AnyAdcChannel<ADC2> = p.PC1.degrade_adc();

            let mut adc = Adc::new(p.ADC2);
            adc.set_sample_time(SampleTime::CYCLES8_5);
            adc.set_resolution(embassy_stm32::adc::Resolution::BITS16);

            {
                use pac::adc::vals::*;

                pac::ADC2.pcsel().modify(|r| {
                    r.set_pcsel(2, Pcsel::PRESELECTED);
                    r.set_pcsel(3, Pcsel::PRESELECTED);
                    r.set_pcsel(4, Pcsel::PRESELECTED);
                    r.set_pcsel(5, Pcsel::PRESELECTED);
                    r.set_pcsel(6, Pcsel::PRESELECTED);
                    r.set_pcsel(7, Pcsel::PRESELECTED);
                    r.set_pcsel(8, Pcsel::PRESELECTED);
                    r.set_pcsel(11, Pcsel::PRESELECTED);
                });
                pac::ADC2.sqr1().modify(|sqr1| {
                    sqr1.set_l((INPUT_CHANNELS - 1) as u8);
                    sqr1.set_sq(0, 2);
                    sqr1.set_sq(1, 3);
                    sqr1.set_sq(2, 4);
                    sqr1.set_sq(3, 5);
                });
                pac::ADC2.sqr2().modify(|sqr2| {
                    sqr2.set_sq(0, 6);
                    sqr2.set_sq(1, 7);
                    sqr2.set_sq(2, 8);
                    sqr2.set_sq(3, 11);
                });

                pac::ADC2.cfgr().modify(|r| {
                    r.set_dmngt(Dmngt::DMA_CIRCULAR);
                    r.set_exten(Exten::RISING_EDGE);
                    r.set_extsel(0b01101); // tim6_trgo
                });
            }

            adc
        };
        let analog_clock = {
            use embassy_stm32::timer::low_level::Timer;
            use pac::timer::*;

            let tim6 = Timer::new(p.TIM6);
            tim6.set_frequency(SAMPLE_RATE);
            tim6.regs_basic()
                .cr2()
                .modify(|r| r.set_mms(vals::Mms::UPDATE));
            tim6
        };
        let p9813: P9813<Spi<'static, embassy_stm32::mode::Blocking>> = {
            use embassy_stm32::spi::Config;
            let sck = p.PG11;
            let mosi = p.PD7;
            let spi = Spi::new_blocking_txonly(p.SPI1, sck, mosi, Config::default());
            P9813::new(spi)
        };

        Board {
            status_leds,
            dac1,
            adc2,
            analog_clock,
            p9813,
        }
    }
}

pub mod clocks {
    use embassy_stm32::Config;
    use embassy_stm32::Peripherals;

    pub fn config(mut config: Config) -> Peripherals {
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
