#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

use core::fmt::Write;

use alloc_cortex_m::CortexMHeap;
use arrayvec::ArrayString;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_time::fixed_point::FixedPoint;
use midly::{live::LiveEvent, stream::MidiStream, MidiMessage};
use rp_pico::{
    entry,
    hal::{
        self, clocks, gpio,
        multicore::{self, Stack},
        sio,
        usb::UsbBus,
        watchdog, Clock,
    },
    pac,
};
use thingbuf::mpsc;
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

fn core0_task() -> ! {
    loop {}
}

static mut CORE1_STACK: Stack<4096> = Stack::new();

fn core1_task(mut usb_dev: UsbDevice<UsbBus>, mut serial: SerialPort<UsbBus>) -> ! {
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
                Ok(len) => {
                    midi_stream.feed(&buf[..len], |event| handle_midi_event(&mut serial, event))
                }
            }
        }
    }
}

fn handle_midi_event(serial: &mut SerialPort<UsbBus>, event: LiveEvent) {
    let mut buf = ArrayString::<128>::new();
    writeln!(&mut buf, "{:?}", event);

    serial.write(buf.as_bytes());
}

static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;

#[entry]
fn main() -> ! {
    let start = cortex_m_rt::heap_start() as usize;
    let size = 1024;
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

    let (tx, rx) = mpsc::StaticChannel::<LiveEvent, 10>::new().split();

    // The single-cycle I/O block
    let mut sio = sio::Sio::new(pac.SIO);

    // Set up second core for execution
    let mut mc = multicore::Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _core1_task = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        core1_task(usb_dev, serial);
    });

    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio25.into_push_pull_output();

    let sys_freq = clocks.system_clock.freq().integer();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);

    loop {
        led_pin.toggle().unwrap();
        delay.delay_ms(1000);
    }
}
