#![no_std]
#![no_main]
#![feature(slice_as_array)]
#![feature(array_chunks)]

use {defmt_rtt as _, panic_probe as _};

use board::{Board, StatusLeds};
use dasp::{Frame, Sample, Signal};
use defmt::debug;
use embassy_executor::Spawner;

use embassy_stm32::Config;
use embassy_stm32::Peripherals;
use embassy_stm32::dma::{ReadableRingBuffer, TransferOptions, WritableRingBuffer};
use embassy_stm32::gpio::Output;
use embassy_stm32::pac;
use embassy_stm32::peripherals::TIM12;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::SimplePwm;

#[global_allocator]
static ALLOCATOR: emballoc::Allocator<2048> = emballoc::Allocator::new();

use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::zerocopy_channel::{Channel, Receiver, Sender};
use embassy_time::{Duration, Instant, Timer};

use embedded_hal::spi;
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
type InputFrameChannel = Channel<'static, NoopRawMutex, InputFrame>;
type OutputFrameChannel = Channel<'static, NoopRawMutex, OutputFrame>;

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
    mut ev: Receiver<'static, NoopRawMutex, Event>,
    mut pwm: SimplePwm<'static, TIM12>,
    mut leds: P9813<Spi<'static, embassy_stm32::mode::Blocking>>,
) {
    let mut intensity: u16 = 0;
    let max_intensity: u16 = pwm.ch1().max_duty_cycle();
    let ratio = 255.0 / pwm.ch1().max_duty_cycle() as f32;
    let release: u16 = max_intensity / 30;

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
        let strip_int: u8 = (intensity as f32 * ratio) as u8;
        leds.set_colors([(strip_int, 0, strip_int)]).unwrap();
        Timer::after_millis(10).await;
    }
}

#[embassy_executor::task]
async fn peak_detector(
    mut rb_in: ReadableRingBuffer<'static, u16>,
    ld: Output<'static>,
    mut ev: Sender<'static, NoopRawMutex, Event>,
) {
    type SampleType = f32;
    use lib::dsp::TransientDetector;

    let mut input_buffer: [u16; INPUT_CHANNELS * BLOCK_SIZE] = [0; INPUT_CHANNELS * BLOCK_SIZE];
    let mut samples: [[SampleType; INPUT_CHANNELS]; BLOCK_SIZE];
    let mut triggers: [Option<bool>; INPUT_CHANNELS];
    let mut transient_detector: TransientDetector<INPUT_CHANNELS> = TransientDetector::new(
        ms_to_frames(2.0),
        ms_to_frames(80.0),
        ms_to_frames(20.0),
        ms_to_frames(80.0),
        0.05,
        0.04,
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
        for frame in samples {
            triggers = transient_detector.process(&frame);
            triggers.iter().enumerate().for_each(|(ch, t)| {
                if t.is_some_and(|t| t) {
                    if let Some(event) = ev.try_send() {
                        *event = Event::AnalogTrigger(ch);
                        ev.send_done();
                    }
                }
            });
        }
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
    mut send: Sender<'static, NoopRawMutex, InputFrame>,
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
    mut recv: Receiver<'static, NoopRawMutex, OutputFrame>,
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = board::clocks::config(Config::default());
    let Board {
        status_leds: StatusLeds { mut ld1, ld2, ld3 },
        dac1,
        dac1_dma,
        adc2,
        analog_clock,
        p9813,
    } = board::Board::init(p);

    static ADC_DMA_BUFFER: StaticCell<AdcDmaBuffer> = StaticCell::new();
    let adc_dma_buf =
        ADC_DMA_BUFFER.init_with(|| AdcDmaBuffer::from([0u16; INPUT_CHANNELS * DMA_BUFFER_SIZE]));
    let adc_dma = unsafe {
        ReadableRingBuffer::new(
            Peripherals::steal().DMA1_CH1,
            ADC2_DMA_REQ,
            pac::ADC2.dr().as_ptr() as *mut u16,
            adc_dma_buf,
            TransferOptions::default(),
        )
    };
    static EVENT_BUFFER: StaticCell<[Event; BLOCK_SIZE]> = StaticCell::new();
    let event_buffer = EVENT_BUFFER.init([Event::AnalogTrigger(0); BLOCK_SIZE]);
    static EVENT_CHANNEL: StaticCell<Channel<NoopRawMutex, Event>> = StaticCell::new();
    let event_channel = EVENT_CHANNEL.init_with(|| Channel::new(event_buffer));
    let (event_sender, event_receiver) = event_channel.split();

    //p9813.set_colors(&[(255u8, 0u8, 255u8)]).unwrap();
    defmt::unwrap!(spawner.spawn(peak_detector(adc_dma, ld2, event_sender)));
    defmt::unwrap!(spawner.spawn(signal_generator(dac1_dma)));
    defmt::unwrap!(spawner.spawn(led_controller(event_receiver, ld3, p9813)));

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
