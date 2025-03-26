#![no_std]
#![no_main]
#![feature(slice_as_array)]
#![feature(array_chunks)]

use {defmt_rtt as _, panic_probe as _};

use dasp::{Frame, Sample, Signal};
use defmt::debug;
use embassy_executor::Spawner;

use embassy_stm32::Config;
use embassy_stm32::adc::SampleTime;
use embassy_stm32::dma::{NoDma, ReadableRingBuffer, TransferOptions, WritableRingBuffer};
use embassy_stm32::pac;
use embassy_stm32::peripherals::{ADC2, TIM12};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{
    adc::{Adc, AdcChannel, AnyAdcChannel},
    gpio::{Level, Output, Speed},
};

#[global_allocator]
static ALLOCATOR: emballoc::Allocator<2048> = emballoc::Allocator::new();

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::zerocopy_channel::{Channel, Receiver, Sender};
use embassy_time::{Duration, Instant, Timer};

use p9813::P9813;
use static_cell::StaticCell;

const SAMPLE_RATE: Hertz = Hertz(8_000);

const ADC2_DMA_REQ: u8 = 10;

const INPUT_CHANNELS: usize = 8;
const OUTPUT_CHANNELS: usize = 2;
const BLOCK_SIZE: usize = 32;
const DMA_BUFFER_SIZE: usize = BLOCK_SIZE * 2;

type AdcDmaBuffer = [u16; INPUT_CHANNELS * DMA_BUFFER_SIZE];
type DacDmaBuffer = [u32; DMA_BUFFER_SIZE];
type InputFrame = [u16; INPUT_CHANNELS];
type OutputFrame = [u16; OUTPUT_CHANNELS];
type InputFrameChannel = Channel<'static, ThreadModeRawMutex, InputFrame>;
type OutputFrameChannel = Channel<'static, ThreadModeRawMutex, OutputFrame>;

#[derive(Copy, Clone)]
enum Event {
    AnalogTrigger(usize),
}

const fn ms_to_frames(ms: f32) -> f32 {
    const SAMPLE_RATIO: f32 = (SAMPLE_RATE.0 as f32) / 1000.0;
    ms * SAMPLE_RATIO
}

#[embassy_executor::task]
async fn led_controller(
    mut ev: Receiver<'static, ThreadModeRawMutex, Event>,
    mut pwm: SimplePwm<'static, TIM12>,
) {
    let mut intensity: u16 = 0;
    let max_intensity: u16 = pwm.ch1().max_duty_cycle();
    let release: u16 = max_intensity / 10;

    pwm.ch1().enable();
    loop {
        if let Some(event) = ev.try_receive() {
            match event {
                Event::AnalogTrigger(channel) => {
                    if *channel == 1usize {
                        debug!("trig received, ch: {}", channel);
                        debug!("TRIG!");
                        intensity = max_intensity;
                    }
                }
            }
            ev.receive_done();
        } else {
            intensity = *intensity.checked_sub(release).get_or_insert(0);
        }
        pwm.ch1().set_duty_cycle(intensity);
        Timer::after_millis(10).await;
    }
}

#[embassy_executor::task]
async fn peak_detector(
    mut rb_in: ReadableRingBuffer<'static, u16>,
    ld: Output<'static>,
    mut ev: Sender<'static, ThreadModeRawMutex, Event>,
) {
    type SampleType = f32;
    use light_controller::dsp::TransientDetector;

    let mut input_buffer: [u16; INPUT_CHANNELS * BLOCK_SIZE] = [0; INPUT_CHANNELS * BLOCK_SIZE];
    let mut samples: [[SampleType; INPUT_CHANNELS]; BLOCK_SIZE];
    let mut triggers: [Option<bool>; INPUT_CHANNELS];

    let mut td: TransientDetector<INPUT_CHANNELS> = TransientDetector::new(
        ms_to_frames(2.0),
        ms_to_frames(80.0),
        ms_to_frames(20.0),
        ms_to_frames(80.0),
        0.02,
        0.01,
    );

    rb_in.clear();
    rb_in.start();
    loop {
        rb_in
            .read_exact(&mut input_buffer)
            .await
            .expect("ADC DMA Error");

        let start = Instant::now();
        // transmute [u16; N * M] to [[u16; N]; M]
        let frames = unsafe {
            core::mem::transmute::<
                [u16; INPUT_CHANNELS * BLOCK_SIZE],
                [[u16; INPUT_CHANNELS]; BLOCK_SIZE],
            >(input_buffer)
        };

        samples = frames.map(|f| f.to_float_frame());
        triggers = td.process_block(&samples);
        triggers.iter().enumerate().for_each(|(ch, t)| {
            if t.is_some_and(|t| t) {
                if let Some(event) = ev.try_send() {
                    *event = Event::AnalogTrigger(ch);
                    ev.send_done();
                }
            }
        });
        debug!("max diff: {}", td.max_diff[1]);
        // debug!(
        //     "processing time: {}us",
        //     (Instant::now() - start).as_micros()
        // );
    }
}

#[embassy_executor::task]
async fn signal_generator(mut rb_out: WritableRingBuffer<'static, u32>) {
    let mut saw = dasp::signal::rate(SAMPLE_RATE.0 as f64).const_hz(1.0).saw();
    let mut output_buffer: [u32; 2] = [0; 2];
    output_buffer.iter_mut().for_each(|frame| {
        let out = u16::from_sample(saw.next());
        *frame = out as u32 | (out as u32) << 16;
    });
    rb_out.start();
    loop {
        match rb_out.write_exact(&output_buffer).await {
            Ok(remaining) => {
                //debug!("dac written! remaining: {}", remaining);
            }
            Err(e) => {
                debug!("dac write error! {}", e);
            }
        }
        output_buffer.iter_mut().for_each(|frame| {
            let out = u16::from_sample(saw.next());
            *frame = out as u32 | (out as u32) << 16;
        });
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
        let buf: u32 = buf[0] as u32 | (buf[1] as u32) << 16;
        dma.write_exact(&[buf]).await.expect("DAC DMA Error");
        recv.receive_done();
    }
}

mod process {
    use super::*;
    use dasp::Signal;
    fn process(
        input: &impl Signal<Frame = [f32; INPUT_CHANNELS]>,
        outpu: &mut dyn Signal<Frame = [f32; OUTPUT_CHANNELS]>,
    ) {
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = Config::default();
    let p = light_controller::setup::clock_config(config);

    let mut ld1 = Output::new(p.PB0, Level::High, Speed::Low);
    let ld2 = Output::new(p.PE1, Level::High, Speed::Low);
    //let mut ld3 = Output::new(p.PB14, Level::High, Speed::Low);
    let ld3 = PwmPin::new_ch1(p.PB14, embassy_stm32::gpio::OutputType::PushPull);

    let pwm = SimplePwm::new(
        p.TIM12,
        Some(ld3),
        None,
        None,
        None,
        Hertz(1000),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );

    let p9813: P9813<Spi<'static, embassy_stm32::mode::Blocking>> = {
        use embassy_stm32::spi::Config;
        let sck = p.PG11;
        let mosi = p.PD7;
        let spi = Spi::new_blocking_txonly(p.SPI1, sck, mosi, Config::default());
        P9813::new(spi)
    };
    let dac = {
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
    let dac1_dma_buf = DAC1_DMA_BUFFER.init_with(|| DacDmaBuffer::from([0; DMA_BUFFER_SIZE]));

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
    let _adc2_6: AnyAdcChannel<ADC2> = p.PF14.degrade_adc();
    let _adc2_7: AnyAdcChannel<ADC2> = p.PA7.degrade_adc();
    let _adc2_8: AnyAdcChannel<ADC2> = p.PC5.degrade_adc();
    let _adc2_11: AnyAdcChannel<ADC2> = p.PC1.degrade_adc();

    let mut adc2 = Adc::new(p.ADC2);
    adc2.set_sample_time(SampleTime::CYCLES8_5);
    adc2.set_resolution(embassy_stm32::adc::Resolution::BITS16);

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

    static EVENT_BUFFER: StaticCell<[Event; BLOCK_SIZE]> = StaticCell::new();
    let event_buffer = EVENT_BUFFER.init([Event::AnalogTrigger(0); BLOCK_SIZE]);
    static EVENT_CHANNEL: StaticCell<Channel<ThreadModeRawMutex, Event>> = StaticCell::new();
    let event_channel = EVENT_CHANNEL.init_with(|| Channel::new(event_buffer));
    let (event_sender, event_receiver) = event_channel.split();
    defmt::unwrap!(spawner.spawn(peak_detector(adc2_dma, ld2, event_sender)));
    defmt::unwrap!(spawner.spawn(signal_generator(dac1_dma)));
    defmt::unwrap!(spawner.spawn(led_controller(event_receiver, pwm)));

    Timer::after_millis(200).await;
    pac::ADC2.cr().modify(|cr| cr.set_adstart(true));
    analog_clock.start();
    loop {
        ld1.set_high();
        Timer::after(Duration::from_millis(500)).await;
        ld1.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
