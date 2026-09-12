#![no_std]
#![no_main]

use core::fmt::Write;
use core::hint::spin_loop;

use hal::pio::{PIOBuilder, PIOExt};
use hal::{entry, pac};
use heapless::String;
use panic_halt as _;
use rp235x_hal as hal;
use rp235x_hal::clocks::init_clocks_and_plls;
use usb_device::LangID;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

const CLOCK_DIVIDER: u16 = 60_000;
const TICKS_PER_RUN: u32 = 16;

/// Tell the RP2350 Boot ROM about this application.
#[unsafe(link_section = ".start_block")]
#[used]
static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

#[entry]
fn main() -> ! {
    let mut peripherals = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(peripherals.WATCHDOG);

    let clocks = init_clocks_and_plls(
        12_000_000,
        peripherals.XOSC,
        peripherals.CLOCKS,
        peripherals.PLL_SYS,
        peripherals.PLL_USB,
        &mut peripherals.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        peripherals.USB,
        peripherals.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut peripherals.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x1209, 0x2350))
        .strings(&[StringDescriptors::new(LangID::EN)
            .manufacturer("pioemu")
            .product("PIO tick prototype")
            .serial_number("0001")])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        .max_packet_size_0(64)
        .unwrap()
        .build();

    let (mut pio, sm0, sm1, _, _) = peripherals.PIO0.split(&mut peripherals.RESETS);

    // SM0 eventually stalls on PULL. Its JMP also has delay cycles, so its PC is
    // deliberately not a usable indication that a PIO clock tick occurred.
    let test_program = pio_proc::pio_asm!(
        ".origin 0",
        ".wrap_target",
        "set x, 3",
        "count:",
        "jmp x-- count [2]",
        "pull block",
        ".wrap",
    )
    .program;

    // These two shared instruction-memory slots are the clock marker. SM1's PC
    // changes on every clock tick, independently of SM0's delay or stall state.
    let marker_program = pio_proc::pio_asm!(".origin 30", ".wrap_target", "nop", "nop", ".wrap",).program;

    let test_program = pio.install(&test_program).unwrap();
    let marker_program = pio.install(&marker_program).unwrap();

    let (mut test_sm, _test_rx, _test_tx) = PIOBuilder::from_installed_program(test_program)
        .clock_divisor_fixed_point(CLOCK_DIVIDER, 0)
        .build(sm0);
    let (mut marker_sm, _marker_rx, _marker_tx) = PIOBuilder::from_installed_program(marker_program)
        .clock_divisor_fixed_point(CLOCK_DIVIDER, 0)
        .build(sm1);

    // Restart both divided clocks in one atomic CTRL write. Since both SMs are
    // subsequently enabled and disabled together, their divider phases remain aligned.
    test_sm.synchronize_with(&mut marker_sm);

    let mut cycle = 0u32;
    let mut receive_buffer = [0u8; 64];

    loop {
        if !usb.poll(&mut [&mut serial]) {
            continue;
        }

        let Ok(count) = serial.read(&mut receive_buffer) else {
            continue;
        };

        if !receive_buffer[..count].contains(&b'r') {
            continue;
        }

        send(
            &mut usb,
            &mut serial,
            b"cycle,test_pc,marker_pc,stalled,padout,padoe,irq,flevel,fdebug\r\n",
        );

        for _ in 0..TICKS_PER_RUN {
            let previous_marker_pc = marker_sm.instruction_address();

            // The grouped start and stop each compile to one atomic PIO CTRL write.
            // Interrupts remain disabled only while the slow PIO clock is running.
            (test_sm, marker_sm) = cortex_m::interrupt::free(|_| {
                let running = test_sm.with(marker_sm).start();
                let (test_running, marker_running) = running.free();

                while marker_running.instruction_address() == previous_marker_pc {
                    spin_loop();
                }

                test_running.with(marker_running).stop().free()
            });

            let registers = unsafe { &*pac::PIO0::ptr() };
            let mut line = String::<192>::new();
            writeln!(
                line,
                "{},{},{},{},{:08x},{:08x},{:02x},{:08x},{:08x}\r",
                cycle,
                test_sm.instruction_address(),
                marker_sm.instruction_address(),
                test_sm.stalled() as u8,
                registers.dbg_padout().read().bits(),
                registers.dbg_padoe().read().bits(),
                pio.get_irq_raw(),
                registers.flevel().read().bits(),
                registers.fdebug().read().bits(),
            )
            .unwrap();
            send(&mut usb, &mut serial, line.as_bytes());
            cycle += 1;
        }
    }
}

fn send(usb: &mut UsbDevice<'_, hal::usb::UsbBus>, serial: &mut SerialPort<'_, hal::usb::UsbBus>, mut bytes: &[u8]) {
    while !bytes.is_empty() {
        match serial.write(bytes) {
            Ok(0) | Err(usb_device::UsbError::WouldBlock) => {
                usb.poll(&mut [serial]);
            }
            Ok(count) => bytes = &bytes[count..],
            Err(_) => return,
        }
    }
}

/// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
static PICOTOOL_ENTRIES: [rp235x_hal::binary_info::EntryAddr; 5] = [
    rp235x_hal::binary_info::rp_cargo_bin_name!(),
    rp235x_hal::binary_info::rp_cargo_version!(),
    rp235x_hal::binary_info::rp_program_description!(c"PIO synchronized tick prototype"),
    rp235x_hal::binary_info::rp_cargo_homepage_url!(),
    rp235x_hal::binary_info::rp_program_build_attribute!(),
];
