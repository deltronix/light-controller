#![no_std]
#![no_main]
#![feature(slice_as_array)]

use {defmt_rtt as _, panic_probe as _};

use dasp::envelope::Detect;
use dasp::{Frame, Sample, Signal};
use defmt::{debug, info, todo};
use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_stm32::adc::SampleTime;
use embassy_stm32::dma::{NoDma, ReadableRingBuffer, TransferOptions, WritableRingBuffer};
use embassy_stm32::pac;
use embassy_stm32::peripherals::ADC2;
use embassy_stm32::time::Hertz;
use embassy_stm32::{
    adc::{Adc, AdcChannel, AnyAdcChannel},
    gpio::{Level, Output, Speed},
};

#[global_allocator]
static ALLOCATOR: emballoc::Allocator<2048> = emballoc::Allocator::new();

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::zerocopy_channel::{Channel, Receiver, Sender};
use embassy_time::{Duration, Instant, Timer};

use microdsp::sfnov::{HardKneeCompression, SpectralFluxNoveltyDetector};
use static_cell::StaticCell;
type DetectorType = SpectralFluxNoveltyDetector<HardKneeCompression>;

const SAMPLE_RATE: Hertz = Hertz(1000);

const ADC2_DMA_REQ: u8 = 10;

const INPUT_CHANNELS: usize = 4;
const OUTPUT_CHANNELS: usize = 2;
const BLOCK_SIZE: usize = 8;
const DMA_BUFFER_SIZE: usize = BLOCK_SIZE * 2;

type AdcDmaBuffer = [u16; INPUT_CHANNELS * DMA_BUFFER_SIZE];
type DacDmaBuffer = [u32; DMA_BUFFER_SIZE];
type InputFrame = [u16; INPUT_CHANNELS];
type OutputFrame = [u16; OUTPUT_CHANNELS];
type InputFrameChannel = Channel<'static, ThreadModeRawMutex, InputFrame>;
type OutputFrameChannel = Channel<'static, ThreadModeRawMutex, OutputFrame>;

#[derive(Clone, Copy)]
struct PeakDetector {
    threshold: f32,
    is_armed: bool,
}

impl PeakDetector {
    fn new(threshold: f32) -> Self {
        PeakDetector {
            threshold,
            is_armed: true,
        }
    }

    fn process(&mut self, input: f32) -> bool {
        if input > self.threshold && self.is_armed {
            self.is_armed = false;
            return true;
        } else if input < self.threshold {
            self.is_armed = true;
        };
        false
    }
}

//#[embassy_executor::task]
#[embassy_executor::task]
async fn read_adc(
    mut dma: ReadableRingBuffer<'static, u16>,
    mut send: Sender<'static, ThreadModeRawMutex, InputFrame>,
) {
    dma.start();
    loop {
        let buf = send.send().await;
        dma.read_exact(buf).await.expect("ADC DMA Error");
        send.send_done();
    }
}
#[embassy_executor::task]
async fn write_dac(
    mut dma: WritableRingBuffer<'static, u32>,
    mut recv: Receiver<'static, ThreadModeRawMutex, OutputFrame>,
) {
    dma.start();
    loop {
        if pac::DAC1.sr().read().dmaudr(0) {
            defmt::error!("DAC1 CH1 underrun!");
        }
        if pac::DAC1.sr().read().dmaudr(1) {
            defmt::error!("DAC1 CH1 underrun!");
        }
        let buf = recv.receive().await;
        debug!("dac dma got: {}", buf);
        let buf: u32 = buf[0] as u32 | (buf[1] as u32) << 16;
        dma.write_exact(&[buf]).await.expect("DAC DMA Error");
        recv.receive_done();
    }
}
#[embassy_executor::task]
async fn audio_process(
    mut adc: Receiver<'static, ThreadModeRawMutex, InputFrame>,
    mut dac: Sender<'static, ThreadModeRawMutex, OutputFrame>,
) {
    use dasp::envelope::Detector;
    use dasp::envelope::detect::Peak;
    use dasp::peak;
    use dasp::signal::Sine;

    let mut detector1: Detector<[f32; INPUT_CHANNELS], Peak> =
        Detector::peak_from_rectifier(peak::FullWave, 4.0, 4.0);
    let mut peaks: [PeakDetector; INPUT_CHANNELS] = [PeakDetector::new(0.5); INPUT_CHANNELS];

    let mut ring_buffer = dasp::ring_buffer::Bounded::from([[0.0f32; 4]; BLOCK_SIZE * 2]);

    let mut sine = dasp::signal::rate(SAMPLE_RATE.0 as f64).const_hz(1.0).saw();
    loop {
        //while !adc.is_empty()
        let frame = adc.receive().await;
        ring_buffer.push(frame.map(f32::from_sample));
        adc.receive_done();

        if ring_buffer.slices().0.len() == BLOCK_SIZE {
            let start = Instant::now();
            ring_buffer.slices().0.iter().for_each(|frame| {
                let env: [f32; INPUT_CHANNELS] = detector1.next(*frame);
                debug!("env: {}", env);
                env.iter().enumerate().for_each(|(ch, peak)| {
                    if peaks[ch].process(*peak) {
                        debug!("peak detected: {}", ch);
                    }
                })
            });
            let dur = Instant::now() - start;
            defmt::debug!("processing took: {}us", dur.as_micros());
        }
        //while !dac.is_full()
        let out_frame = dac.send().await;
        debug!("sending to dac: {}", out_frame);
        out_frame.chunks_mut(2).into_iter().for_each(|f| {
            let output = u16::from_sample(sine.next());
            f[0] = output;
            f[1] = output;
        });
        dac.send_done();
        //}
    }
}

mod process {
    use super::*;
    use dasp::{Signal, sample::ToSample};
    fn process(
        input: &impl Signal<Frame = [f32; INPUT_CHANNELS]>,
        outpu: &mut dyn Signal<Frame = [f32; OUTPUT_CHANNELS]>,
    ) {
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        //config.rcc.hsi = Some(HSIPrescaler::DIV2);
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: None,
            divr: None,
        });
        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV8), // 100mhz
            divq: None,
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV4; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV4; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV4; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV4; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.mux.adcsel = mux::Adcsel::PLL2_P;
    }

    let p = embassy_stm32::init(config);

    let mut led = Output::new(p.PB7, Level::High, Speed::Low);

    let mut dac = {
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

    static DAC1_DMA_BUFFER: StaticCell<DacDmaBuffer> = StaticCell::new();
    let dac1_dma_buf = DAC1_DMA_BUFFER.init_with(|| DacDmaBuffer::EQUILIBRIUM);

    let dac1_dma = unsafe {
        WritableRingBuffer::new(
            p.DMA1_CH1,
            67,
            pac::DAC1.dhr12ld().as_ptr() as *mut u32,
            dac1_dma_buf,
            TransferOptions::default(),
        )
    };

    let _adc2_2: AnyAdcChannel<ADC2> = p.PF13.degrade_adc();
    let _adc2_3: AnyAdcChannel<ADC2> = p.PA6.degrade_adc();
    let _adc2_4: AnyAdcChannel<ADC2> = p.PC4.degrade_adc();
    let _adc2_5: AnyAdcChannel<ADC2> = p.PB1.degrade_adc();

    let mut adc2 = Adc::new(p.ADC2);
    adc2.set_sample_time(SampleTime::CYCLES64_5);
    adc2.set_resolution(embassy_stm32::adc::Resolution::BITS16);

    {
        use pac::adc::vals::*;
        pac::ADC2.pcsel().modify(|r| {
            r.set_pcsel(2, Pcsel::PRESELECTED);
            r.set_pcsel(3, Pcsel::PRESELECTED);
            r.set_pcsel(4, Pcsel::PRESELECTED);
            r.set_pcsel(5, Pcsel::PRESELECTED);
        });
        pac::ADC2.sqr1().modify(|sqr1| {
            sqr1.set_l(3);
            sqr1.set_sq(0, 2);
            sqr1.set_sq(1, 3);
            sqr1.set_sq(2, 4);
            sqr1.set_sq(3, 5);
        });

        pac::ADC2.cfgr().modify(|r| {
            r.set_dmngt(Dmngt::DMA_CIRCULAR);
            r.set_exten(Exten::RISING_EDGE);
            r.set_extsel(0b01101); // tim6_trgo
        });
    }
    static ADC_DMA_BUFFER: StaticCell<AdcDmaBuffer> = StaticCell::new();
    let adc_dma_buf = ADC_DMA_BUFFER.init_with(|| {
        AdcDmaBuffer::from([<u16 as dasp::Sample>::EQUILIBRIUM; INPUT_CHANNELS * DMA_BUFFER_SIZE])
    });
    let adc2_dma = unsafe {
        ReadableRingBuffer::new(
            p.DMA1_CH0,
            ADC2_DMA_REQ,
            pac::ADC2.dr().as_ptr() as *mut u16,
            adc_dma_buf,
            TransferOptions::default(),
        )
    };
    // let tim3 = {
    //     use embassy_stm32::timer::low_level::Timer;
    //     use pac::timer::*;
    //
    //     let tim3 = Timer::new(p.TIM3);
    //     tim3.set_frequency(Hertz(2));
    //     tim3.regs_gp16()
    //         .cr2()
    //         .modify(|r| r.set_mms(vals::Mms::UPDATE));
    //
    //     tim3
    // };
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

    static INPUT_BUFFER: StaticCell<[InputFrame; BLOCK_SIZE]> = StaticCell::new();
    let input_buffer = INPUT_BUFFER.init([InputFrame::default(); BLOCK_SIZE]);
    static INPUT_CHANNEL: StaticCell<InputFrameChannel> = StaticCell::new();
    let input_channel = INPUT_CHANNEL.init_with(|| InputFrameChannel::new(input_buffer));
    let (input_sender, input_receiver) = input_channel.split();

    static OUTPUT_BUFFER: StaticCell<[OutputFrame; BLOCK_SIZE]> = StaticCell::new();
    let output_buffer = OUTPUT_BUFFER.init([OutputFrame::default(); BLOCK_SIZE]);
    static OUTPUT_CHANNEL: StaticCell<OutputFrameChannel> = StaticCell::new();
    let output_channel = OUTPUT_CHANNEL.init_with(|| OutputFrameChannel::new(output_buffer));
    let (output_sender, output_receiver) = output_channel.split();

    defmt::unwrap!(spawner.spawn(read_adc(adc2_dma, input_sender)));
    defmt::unwrap!(spawner.spawn(audio_process(input_receiver, output_sender)));
    defmt::unwrap!(spawner.spawn(write_dac(dac1_dma, output_receiver)));

    analog_clock.start();
    // tim3.start();
    pac::ADC2.cr().modify(|cr| cr.set_adstart(true));
    loop {
        //info!("Hello, World!");
        led.set_high();
        // let vref_int = adc2.blocking_read(&mut vrefint_channel) as f32 / u16::MAX as f32;
        // let adc = adc2.blocking_read(&mut adc2_3) as f32 / u16::MAX as f32;
        // info!("vref_int: {}, adc 2: {}", vref_int, adc);
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
