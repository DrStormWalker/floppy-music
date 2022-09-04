#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

mod arc;
mod lock;

use core::{
    borrow, cmp,
    ffi::c_void,
    fmt::{self, Write},
    hash::{Hash, Hasher},
    marker::PhantomData,
    mem,
    ops::Deref,
    ptr,
    sync::atomic::{AtomicUsize, Ordering},
};

extern crate alloc;

use alloc::boxed::Box;
use alloc_cortex_m::CortexMHeap;
use arc::Arc;
use arrayvec::{ArrayString, ArrayVec};
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_time::fixed_point::FixedPoint;
use floppy_music_macros::midi_note_periods;
use heapless::LinearMap;
use lock::{SpinLock, TicketLock};
use lock_api::{GuardSend, RawMutex};
use midly::{
    live::{LiveEvent, SystemRealtime},
    stream::MidiStream,
    MidiMessage,
};
use rp_pico::{
    entry,
    hal::{
        self, clocks,
        gpio::{self, DynPin},
        multicore::{self, Stack},
        sio,
        usb::UsbBus,
        watchdog, Clock,
    },
    pac,
};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::SerialPort;

use panic_halt as _;

#[global_allocator]
static GLOBAL_ALLOC: CortexMHeap = CortexMHeap::empty();

#[alloc_error_handler]
fn oom(_: core::alloc::Layout) -> ! {
    loop {}
}

/// Array of time periods (in micro-seconds) corresponding to midi notes
///
/// Genertated using a proc macro to evaulate a constant expression see
/// floppy_music_macros/src/lib.rs for details
const NOTES: [u64; 128] = midi_note_periods!();

struct FloppyDrive {
    label: ArrayString<32>,
    step_pin: DynPin,
    dir_pin: DynPin,
    step_count: u16,
    pitch: u8,
    half_period: u64,
    last_half: u64,
}
impl FloppyDrive {
    pub fn new(label: &str, step_pin: DynPin, dir_pin: DynPin) -> Self {
        Self {
            label: ArrayString::from(label).unwrap(),
            step_pin,
            dir_pin,
            step_count: 0,
            pitch: 0,
            half_period: 0,
            last_half: 0,
        }
    }

    pub fn set_pitch(&mut self, pitch: u8) {
        self.pitch = pitch;
        self.half_period = NOTES[pitch as usize - 12];
    }

    pub fn get_half_period(&self) -> u64 {
        self.half_period
    }

    pub fn get_last_half(&self) -> u64 {
        self.last_half
    }

    pub fn get_pitch(&self) -> u8 {
        self.pitch
    }

    pub fn set_step_count(&mut self, step_count: u16) {
        self.step_count = step_count;
    }

    pub fn raw_step(&mut self) {
        self.step_pin.toggle();
    }

    pub fn change_direction(&mut self) {
        self.dir_pin.toggle();
    }

    pub fn step(&mut self) {
        self.step_pin.toggle();
        self.step_count += 1;

        if self.step_count >= 154 {
            self.dir_pin.toggle();
            self.step_count = 0;
        }
    }

    pub fn set_last_half(&mut self, last_half: u64) {
        self.last_half = last_half;
    }
}
impl fmt::Display for FloppyDrive {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.label)
    }
}
impl fmt::Debug for FloppyDrive {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self.label)
    }
}

fn core0_task() -> ! {
    loop {}
}

static mut CORE1_STACK: Stack<4096> = Stack::new();

type FloppyStack = Arc<SpinLock<ArrayVec<FloppyDrive, 10>>>;
type FloppyUsed = Arc<SpinLock<LinearMap<u8, FloppyDrive, 10>>>;
type StateContainer = Arc<SpinLock<State>>;

#[derive(Copy, Clone)]
enum State {
    Running,
    Paused,
}

fn core1_task(
    stack: FloppyStack,
    used: FloppyUsed,
    state: StateContainer,
    mut usb_dev: UsbDevice<UsbBus>,
    mut serial: SerialPort<UsbBus>,
) -> ! {
    midly::stack_buffer! {
        struct MidiBuffer([u8; 2048]);
    }
    let mut midi_stream = MidiStream::<MidiBuffer>::default();

    let mut pac = unsafe { pac::Peripherals::steal() };
    let core = unsafe { pac::CorePeripherals::steal() };

    serial.write(b"Starting USB poll");

    loop {
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {}
                Ok(0) => {}
                Ok(len) => midi_stream.feed(&buf[..len], |event| {
                    handle_midi_event(&stack, &used, &state, &mut serial, event)
                }),
            }
        }
    }
}

fn handle_midi_event(
    stack: &FloppyStack,
    used: &FloppyUsed,
    state: &StateContainer,
    serial: &mut SerialPort<UsbBus>,
    event: LiveEvent,
) {
    let mut buf = ArrayString::<128>::new();
    writeln!(&mut buf, "{:?}", event);

    serial.write(buf.as_bytes());

    match event {
        LiveEvent::Midi {
            channel,
            message: MidiMessage::NoteOn { key, vel },
        } => {
            let mut stack = stack.lock();
            if let Some(mut floppy) = stack.pop() {
                floppy.set_pitch(key.as_int());

                let mut used = used.lock();
                used.insert(key.as_int(), floppy);
            }
        }
        LiveEvent::Midi {
            channel,
            message: MidiMessage::NoteOff { key, vel },
        } => {
            if let Some(floppy) = used.lock().remove(&key.as_int()) {
                stack.lock().push(floppy);
            }
        }
        LiveEvent::Realtime(SystemRealtime::Reset) => {
            let mut used = used.lock();
            let mut stack = stack.lock();
            let mut state = state.lock();

            for pitch in used.keys().copied().collect::<ArrayVec<u8, 10>>() {
                if let Some(floppy) = used.remove(&pitch) {
                    stack.push(floppy);
                }
            }

            *state = State::Running;
        }
        LiveEvent::Realtime(SystemRealtime::Stop) => {
            let mut state = state.lock();

            *state = State::Paused;
        }
        LiveEvent::Realtime(SystemRealtime::Start | SystemRealtime::Continue) => {
            let mut state = state.lock();

            *state = State::Running;
        }
        _ => {}
    }
}

static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;

#[entry]
fn main() -> ! {
    let start = cortex_m_rt::heap_start() as usize;
    let size = 4096;
    unsafe { GLOBAL_ALLOC.init(start, size) }

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver
    let mut watchdog = watchdog::Watchdog::new(pac.WATCHDOG);

    // Configure clocks
    let clocks = clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    #[cfg(feature = "rp2040-e5")]
    {
        let mut sio = sio::Sio::new(pac.SIO);
        let pins = gpio::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
    }

    // Set up the USB Driver
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe { USB_BUS = Some(usb_bus) }

    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    let serial = SerialPort::new(&bus_ref);

    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        //.manufacturer("Sixth Form College Farnborough - Computer Science Dept.")
        //.product("SFCF Floppo-song")
        .serial_number("TEST1")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    // The single-cycle I/O block
    let mut sio = sio::Sio::new(pac.SIO);

    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();

    led_pin.toggle();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let mut stack = ArrayVec::<_, 10>::new();

    stack.push(FloppyDrive::new(
        "Floppy 1",
        pins.gpio3.into_push_pull_output().into(),
        pins.gpio2.into_push_pull_output().into(),
    ));

    stack.push(FloppyDrive::new(
        "Floppy 2",
        pins.gpio5.into_push_pull_output().into(),
        pins.gpio4.into_push_pull_output().into(),
    ));

    stack.push(FloppyDrive::new(
        "Floppy 3",
        pins.gpio7.into_push_pull_output().into(),
        pins.gpio6.into_push_pull_output().into(),
    ));

    for i in 0..160 {
        stack.iter_mut().for_each(|drive| {
            drive.raw_step();
        });

        delay.delay_us(500);
    }

    stack.iter_mut().for_each(|drive| {
        drive.change_direction();
    });

    for i in 0..2 {
        stack.iter_mut().for_each(|drive| {
            drive.raw_step();
        });
    }

    stack.iter_mut().for_each(|drive| {
        drive.set_step_count(0);
    });

    let channel0_stack = Arc::new(SpinLock::new(stack));
    let channel0_used = Arc::new(SpinLock::new(LinearMap::<_, _, 10>::new()));
    let state = Arc::new(SpinLock::new(State::Running));

    // Set up second core for execution
    let mut mc = multicore::Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    {
        let (stack, used, state) = (channel0_stack.clone(), channel0_used.clone(), state.clone());
        let _core1_task = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            core1_task(stack, used, state, usb_dev, serial);
        });
    }

    let mut last_toggle = 0;

    let mut count = 0u64;

    led_pin.toggle();

    loop {
        let state = {
            let state = state.lock();
            let copy = *state;
            drop(state);

            copy
        };

        if let State::Paused = state {
            delay.delay_us(100);
            continue;
        }

        let time_delay = {
            let mut channel0 = channel0_used.lock();

            channel0
                .values_mut()
                .map(|drive| {
                    if count >= drive.get_last_half() + drive.get_half_period() {
                        drive.step();
                        drive.set_last_half(count);
                    }

                    drive.get_half_period() + drive.get_last_half() - count
                })
                .min()
                .unwrap_or(100)
        };

        /*if count >= last_toggle + 1_000_000 {
            led_pin.toggle().unwrap();
            last_toggle = count;
        }*/

        delay.delay_us(time_delay as u32);
        count += time_delay;
    }
}
