//! # Pico USB Serial (with Interrupts) Example
//!
//! Creates a USB Serial device on a Pico board, with the USB driver running in
//! the USB interrupt.
//!
//! This will create a USB Serial device echoing anything it receives. Incoming
//! ASCII characters are converted to upercase, so you can tell it is working
//! and not just local-echo!
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use core::fmt::Write;
/*use core::sync::atomic::AtomicBool;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::AtomicU8;*/

use arrayvec::ArrayString;
use arrayvec::ArrayVec;
use embedded_hal::digital::v2::ToggleableOutputPin;
use floppy_music_macros::midi_note_periods;
use hashbrown::HashMap;
use heapless::Arc;
// The macro for our start-up function
use rp_pico::entry;

use rp_pico::hal::gpio::DynPin;
// The macro for marking our interrupt functions
use rp_pico::hal::pac::interrupt;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

use rp_pico::hal::usb::UsbBus;
use spin::Mutex;
use synctools::mcs::MCSLock;
use ufmt::derive::uDebug;

// USB Device support
use usb_device;

use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::UsbDevice;
use usb_device::device::UsbDeviceBuilder;
use usb_device::device::UsbVidPid;
// USB Communications Class Device support
use usbd_serial::SerialPort;

#[derive(uDebug, Debug)]
enum MidiMsg {
    NoteOn {
        channel: u8,
        pitch: u8,
        velocity: u8,
    },
    NoteOff {
        channel: u8,
        pitch: u8,
        velocity: u8,
    },
    Other,
}
impl MidiMsg {
    pub fn new(cmd: u8, buf: ArrayVec<u8, 2>) -> Self {
        assert_eq!(buf.len(), get_cmd_len(cmd));
        match cmd {
            // Note off
            0x80..=0x8F => Self::NoteOff {
                channel: cmd & 0xF,
                pitch: buf[0],
                velocity: buf[1],
            },
            // Note on
            0x90..=0x9F => Self::NoteOn {
                channel: cmd & 0xF,
                pitch: buf[0],
                velocity: buf[1],
            },

            _ => Self::Other,
        }
    }
}

struct FloppyDrive {
    step_pin: DynPin,
    dir_pin: DynPin,
    step_count: u16,
    pitch: u8,
    half_period: u32,
    last_half: u32,
}
impl FloppyDrive {
    pub fn new(step_pin: DynPin, dir_pin: DynPin) -> Self {
        Self {
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
        self.half_period = NOTES[pitch as usize];
    }

    pub fn get_half_period(&self) -> u32 {
        self.half_period
    }

    pub fn get_last_half(&self) -> u32 {
        self.last_half
    }

    pub fn step(&mut self) {
        self.step_pin.toggle();
        self.step_count += 1;

        if self.step_count >= 80 {
            self.dir_pin.toggle();
            self.step_count = 0;
        }
    }

    pub fn set_last_half(&mut self, last_half: u32) {
        self.last_half = last_half;
    }
}

/// Returns the length of the command (in bytes)
///
/// Values are from the 'Message Formats' table specifed
/// [here](https://www.cs.cmu.edu/~music/cmsip/readings/davids-midi-spec.htm)
const fn get_cmd_len(cmd: u8) -> usize {
    match cmd {
        0x80..=0x8F => 2, // Note off
        0x90..=0x9F => 2, // Note on
        0xA0..=0xAF => 2, // Key pressure
        0xB0..=0xBF => 2, // Controller change
        0xC0..=0xCF => 1, // Program change
        0xD0..=0xDF => 1, // Channel pressure
        0xE0..=0xEF => 2, // Pitch bend

        0xF0 => 0, // System exclusive (SysEx) (Unknown length)
        0xF2 => 2, // Song position
        0xF3 => 1, // Song select
        0xF5 => 1, // Unofficial bus select
        0xF6 => 0, // Tune request
        0xF7 => 0, // End of SysEx
        0xF8 => 0, // Timing tick
        0xFA => 0, // Start song
        0xFB => 0, // Continue song
        0xFC => 0, // Stop song
        0xFE => 0, // Active Sensing
        0xFF => 0, // System reset
        _ => panic!(),
        //_ => 0,
    }
}

/// Array of time periods (in micro-seconds) corresponding to midi notes
///
/// Genertated using a proc macro to evaulate a constant expression see
/// floppy_music_macros/src/lib.rs for details
const NOTES: [u32; 128] = midi_note_periods!();

const NO_CMD: u8 = 0;
const SYS_EX_CMD: u8 = 0xF0;
const SYS_EX_END: u8 = 0xF8;

static mut CMD: u8 = NO_CMD;
static mut CMD_LEN: usize = 0usize;
static mut CMD_BUF: Option<ArrayVec<u8, 2>> = None;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;

/// The USB Serial Device Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;

/*static mut PLAYING: AtomicBool = AtomicBool::new(false);
static mut NOTE: AtomicU8 = AtomicU8::new(0);
static mut PERIOD: AtomicU32 = AtomicU32::new(0u32);*/

static mut CHANNEL0_STACK: Arc<MCSLock<ArrayVec<FloppyDrive, 10>>> =
    Arc::new(MCSLock::new(ArrayVec::new()));
static mut CHANNEL0_USED: Arc<MCSLock<HashMap<u8, FloppyDrive>>> =
    Arc::new(MCSLock::new(HashMap::new()));

fn core1_task() -> ! {
    loop {}
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Static var initialised at runtime
    unsafe {
        CMD_BUF = Some(ArrayVec::new());
    }

    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
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

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB Communications Class Device driver
    let serial = SerialPort::new(bus_ref);
    unsafe {
        USB_SERIAL = Some(serial);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    // No more USB code after this point in main! We can do anything we want in
    // here since USB is handled in the interrupt - let's blink an LED!

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set the LED to be an output
    let mut step_pin0 = pins.gpio3.into_push_pull_output();
    let mut dir_pin0 = pins.gpio2.into_push_pull_output();

    let mut step_pin1 = pins.gpio5.into_push_pull_output();
    let mut dir_pin1 = pins.gpio4.into_push_pull_output();

    let mut step_pin2 = pins.gpio7.into_push_pull_output();
    let mut dir_pin2 = pins.gpio6.into_push_pull_output();

    let mut toggle_step = false;
    let mut toggle_dir = false;
    let mut i = 0;

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    // Blink the LED at 1 Hz
    loop {
        /*let period = unsafe {
            if !PLAYING.load(atomic::Ordering::Relaxed) {
                continue;
            }

            PERIOD.load(atomic::Ordering::Relaxed)
        };

        step_pin0.set_state(PinState::from(toggle_step));
        step_pin1.set_state(PinState::from(toggle_step));
        step_pin2.set_state(PinState::from(toggle_step));
        toggle_step = !toggle_step;

        i += 1;

        if i % 78 == 0 {
            dir_pin0.set_state(PinState::from(toggle_dir));
            dir_pin1.set_state(PinState::from(toggle_dir));
            dir_pin2.set_state(PinState::from(toggle_dir));
            toggle_dir = !toggle_dir;
        }

        delay.delay_us(period);*/

        let channel0 = unsafe { CHANNEL0_USED.lock() };
        let count = timer.get_counter_low();

        for drive in channel0.values() {
            if count >= drive.get_last_half() + drive.get_half_period() {
                drive.step();
                drive.set_last_half(count);
            }
        }
    }
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
///
/// We do all our USB work under interrupt, so the main thread can continue on
/// knowing nothing about USB.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Grab the global objects. This is OK as we only access them under interrupt.
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    let cmd = &mut CMD;
    let cmd_len = &mut CMD_LEN;
    let cmd_buf = CMD_BUF.as_mut().unwrap();

    // Poll the USB driver with all of our supported USB Classes
    if usb_dev.poll(&mut [serial]) {
        handle_usb_recv(usb_dev, serial, cmd, cmd_len, cmd_buf);
    }
}

fn handle_usb_recv(
    usb_dev: &mut UsbDevice<UsbBus>,
    serial: &mut SerialPort<UsbBus>,
    cmd: &mut u8,
    cmd_len: &mut usize,
    cmd_buf: &mut ArrayVec<u8, 2>,
) {
    let mut buf = [0u8; 64];
    match serial.read(&mut buf) {
        Err(_e) => {
            // Do nothing
        }
        Ok(0) => {
            // Do nothing
        }
        Ok(len) => buf.iter().take(len).for_each(|b| {
            match *cmd {
                // No command currently being captured
                NO_CMD => {
                    *cmd_len = get_cmd_len(*b);
                    *cmd = *b;
                }
                // System exclusive, ignore all bytes
                SYS_EX_CMD if *b == SYS_EX_END => *cmd = NO_CMD,
                SYS_EX_CMD => {}
                // Any other command body
                _ => {
                    cmd_buf.try_push(*b).unwrap();
                    if cmd_buf.len() >= *cmd_len {
                        handle_midi_msg(serial, MidiMsg::new(*cmd, cmd_buf.clone()));
                        *cmd = NO_CMD;
                        cmd_buf.clear();
                    }
                }
            }
        }),
    }
}

fn handle_midi_msg(serial: &mut SerialPort<UsbBus>, msg: MidiMsg) {
    let mut buf = ArrayString::<128>::new();
    writeln!(&mut buf, "{:?}", msg);
    serial.write(buf.as_bytes());

    match msg {
        MidiMsg::NoteOn {
            channel,
            pitch,
            velocity,
        } => unsafe {
            /*PERIOD.store(NOTES[pitch as usize], atomic::Ordering::Relaxed);
            PLAYING.store(true, atomic::Ordering::Relaxed);
            NOTE.store(pitch, atomic::Ordering::Relaxed);*/

            if let Some(mut floppy) = CHANNEL0_STACK.lock().pop() {
                floppy.set_pitch(pitch);

                CHANNEL0_USED.lock().insert(pitch, floppy);
            }
        },
        MidiMsg::NoteOff {
            channel,
            pitch,
            velocity,
        } => unsafe {
            /*if NOTE.load(atomic::Ordering::Relaxed) == pitch {
                PLAYING.store(false, atomic::Ordering::Relaxed);
            }*/
            if let Some(floppy) = CHANNEL0_USED.lock().remove(&pitch) {
                CHANNEL0_STACK.lock().push(floppy);
            }
        },
        _ => {}
    }
}

// End of file
