#![no_std]
#![no_main]

use core::fmt::Write;
use core::hint::spin_loop;

use hal::{entry, pac};
use heapless::String;
use panic_halt as _;
use pioemu_protocol::{CSV_HEADER, Decoder, Request};
use rp235x_hal as hal;
use rp235x_hal::clocks::init_clocks_and_plls;
use usb_device::LangID;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

const CLOCK_DIVIDER: u16 = 1_000;
const MARKER_ORIGIN: usize = 30;

#[derive(Clone, Copy)]
struct SmConfig {
    clkdiv: u32,
    execctrl: u32,
    shiftctrl: u32,
    pinctrl: u32,
}

struct Snapshot {
    test_pc: u32,
    marker_pc: u32,
    padout: u32,
    padoe: u32,
    irq: u8,
    flevel: u32,
    fdebug: u32,
}

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

    peripherals.RESETS.reset().modify(|_, w| w.io_bank0().clear_bit());
    while peripherals.RESETS.reset_done().read().io_bank0().bit_is_clear() {
        spin_loop();
    }
    let registers = unsafe { &*pac::PIO0::ptr() };
    let mut decoder = Decoder::default();

    let mut receive_buffer = [0u8; 64];

    loop {
        let polled = usb.poll(&mut [&mut serial]);
        if serial.line_coding().data_rate() == 1200 {
            hal::reboot::reboot(
                hal::reboot::RebootKind::BootSel { picoboot_disabled: false, msd_disabled: false },
                hal::reboot::RebootArch::Normal,
            );
        }
        if !polled {
            continue;
        }

        let Ok(count) = serial.read(&mut receive_buffer) else {
            continue;
        };

        for byte in &receive_buffer[..count] {
            let Some(request) = decoder.push(*byte) else {
                continue;
            };
            let request = match request {
                Ok(request) => request,
                Err(error) => {
                    send(&mut usb, &mut serial, b"ERR ");
                    send(&mut usb, &mut serial, error.as_bytes());
                    send(&mut usb, &mut serial, b"\r\n");
                    continue;
                }
            };
            send(&mut usb, &mut serial, CSV_HEADER.as_bytes());
            send(&mut usb, &mut serial, b"\r\n");

            for (pin, gpio) in peripherals.IO_BANK0.gpio_iter().take(32).enumerate() {
                gpio.gpio_ctrl().write(|w| unsafe { w.inover().bits(2 | (request.gpio_in >> pin) as u8 & 1) });
            }
            for ticks_elapsed in 1..=request.ticks {
                if !serial.dtr() {
                    break;
                }
                reset_and_initialize(&peripherals.RESETS, registers, &request);

                let overran = (0..ticks_elapsed).any(|_| {
                    let previous_marker_pc = registers.sm(1).sm_addr().read().bits();
                    cortex_m::interrupt::free(|_| {
                        // Enable/disable both SMs in the same write. The divider is
                        // deliberately slow enough to stop before the next tick.
                        registers.ctrl().write(|w| unsafe { w.bits(0b11) });
                        while registers.sm(1).sm_addr().read().bits() == previous_marker_pc {
                            spin_loop();
                        }
                        registers.ctrl().write(|w| unsafe { w.bits(0) });
                    });
                    // Replays can be long; keep USB serviced between ticks.
                    usb.poll(&mut [&mut serial]);
                    registers.sm(1).sm_addr().read().bits() == previous_marker_pc
                });
                if overran {
                    send(&mut usb, &mut serial, b"ERR clock divider too fast to stop after one tick\r\n");
                    break;
                }

                // Save non-destructive state before the diagnostic instructions alter SM0.
                let snapshot = Snapshot {
                    test_pc: registers.sm(0).sm_addr().read().bits(),
                    marker_pc: registers.sm(1).sm_addr().read().bits(),
                    padout: registers.dbg_padout().read().bits(),
                    padoe: registers.dbg_padoe().read().bits(),
                    irq: registers.irq().read().bits() as u8,
                    flevel: registers.flevel().read().bits(),
                    fdebug: registers.fdebug().read().bits(),
                };

                let mut rx_words = [0u32; 4];
                let mut rx_count = 0usize;
                while registers.fstat().read().rxempty().bits() & 1 == 0 {
                    let word = registers.rxf(0).read().bits();
                    if rx_count < rx_words.len() {
                        rx_words[rx_count] = word;
                    }
                    rx_count += 1;
                }

                let Some(hidden) = extract_hidden_registers(registers) else {
                    send(&mut usb, &mut serial, b"ERR register extraction failed\r\n");
                    break;
                };

                let mut line = String::<320>::new();
                writeln!(
                    line,
                    "{},{},{},{:08x},{:08x},{:02x},{:08x},{:08x},{},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x}\r",
                    ticks_elapsed,
                    snapshot.test_pc,
                    snapshot.marker_pc,
                    snapshot.padout,
                    snapshot.padoe,
                    snapshot.irq,
                    snapshot.flevel,
                    snapshot.fdebug,
                    rx_count,
                    rx_words[0],
                    rx_words[1],
                    rx_words[2],
                    rx_words[3],
                    hidden[1],
                    hidden[2],
                    hidden[0],
                    hidden[3],
                )
                .unwrap();
                send(&mut usb, &mut serial, line.as_bytes());
                if ticks_elapsed == request.ticks {
                    send(&mut usb, &mut serial, b"DONE\r\n");
                }
            }
            for gpio in peripherals.IO_BANK0.gpio_iter() {
                gpio.gpio_ctrl().reset();
            }
        }
    }
}

fn reset_and_initialize(resets: &pac::RESETS, registers: &pac::pio0::RegisterBlock, request: &Request) {
    resets.reset().modify(|_, w| w.pio0().set_bit());
    resets.reset().modify(|_, w| w.pio0().clear_bit());
    while resets.reset_done().read().pio0().bit_is_clear() {
        spin_loop();
    }

    let mut code = [0u16; MARKER_ORIGIN];
    code[request.origin as usize..][..request.len].copy_from_slice(&request.code[..request.len]);
    for (slot, instruction) in code.iter().chain(&[0xa042; 2]).enumerate() {
        registers.instr_mem(slot).write(|w| unsafe { w.bits(*instruction as u32) });
    }

    let configs = [
        SmConfig {
            clkdiv: (CLOCK_DIVIDER as u32) << 16,
            execctrl: request.execctrl,
            shiftctrl: request.shiftctrl,
            pinctrl: request.pinctrl,
        },
        SmConfig { clkdiv: (CLOCK_DIVIDER as u32) << 16, execctrl: (30 << 7) | (31 << 12), shiftctrl: (1 << 18) | (1 << 19), pinctrl: 0 },
    ];
    for (index, config) in configs.iter().enumerate() {
        let sm = registers.sm(index);
        sm.sm_clkdiv().write(|w| unsafe { w.bits(config.clkdiv) });
        sm.sm_execctrl().write(|w| unsafe { w.bits(config.execctrl) });
        sm.sm_shiftctrl().write(|w| unsafe { w.bits(config.shiftctrl) });
        sm.sm_pinctrl().write(|w| unsafe { w.bits(config.pinctrl) });
    }

    // Clear pending instructions, stalls, delays and shift counters in both SMs.
    registers.ctrl().write(|w| unsafe { w.bits(0b11 << 4) });
    load_state(registers, request);
    inject(registers, jmp(request.origin));
    registers
        .sm(1)
        .sm_instr()
        .write(|w| unsafe { w.bits(jmp(MARKER_ORIGIN as u8) as u32) });
    for word in &request.tx[..request.tx_len] {
        registers.txf(0).write(|w| unsafe { w.bits(*word) });
    }

    // Restart both divided clocks in one write so future grouped starts are aligned.
    registers.ctrl().write(|w| unsafe { w.bits(0b11 << 8) });
}

fn load_state(registers: &pac::pio0::RegisterBlock, request: &Request) {
    let sm = registers.sm(0);
    sm.sm_clkdiv().write(|w| unsafe { w.bits(1 << 16) });
    sm.sm_shiftctrl().write(|w| unsafe { w.bits(request.shiftctrl & !(0b11 << 16 | 0b11 << 30)) });

    let (isr, isr_in) = match (request.isr_count, request.in_shift_right()) {
        (0, _) => (request.isr, 0),
        (32, _) => (0, request.isr),
        (count, false) => (request.isr >> count, request.isr & ((1 << count) - 1)),
        (count, true) => (request.isr << count, request.isr >> (32 - count)),
    };
    load_osr(registers, isr);
    execute(registers, 0xa0c7); // MOV ISR, OSR
    if request.isr_count > 0 {
        load_osr(registers, isr_in);
        execute(registers, 0x40e0 | (request.isr_count as u16 & 31)); // IN OSR, count
    }
    load_osr(registers, request.x);
    execute(registers, 0xa027); // MOV X, OSR
    load_osr(registers, request.y);
    execute(registers, 0xa047); // MOV Y, OSR
    let osr = match (request.osr_count, request.out_shift_right()) {
        (0 | 32, _) => request.osr,
        (count, true) => request.osr << count,
        (count, false) => request.osr >> count,
    };
    load_osr(registers, osr);
    if request.osr_count > 0 {
        execute(registers, 0x6060 | (request.osr_count as u16 & 31)); // OUT NULL, count
    }

    registers.irq_force().write(|w| unsafe { w.bits(request.irq as u32) });
    sm.sm_shiftctrl().write(|w| unsafe { w.bits(request.shiftctrl) });
    sm.sm_clkdiv().write(|w| unsafe { w.bits((CLOCK_DIVIDER as u32) << 16) });
}

fn load_osr(registers: &pac::pio0::RegisterBlock, word: u32) {
    registers.txf(0).write(|w| unsafe { w.bits(word) });
    execute(registers, 0x80a0); // PULL BLOCK
}

fn execute(registers: &pac::pio0::RegisterBlock, instruction: u16) {
    inject(registers, instruction);
    while registers.sm(0).sm_execctrl().read().exec_stalled().bit_is_set() {
        spin_loop();
    }
}

fn extract_hidden_registers(registers: &pac::pio0::RegisterBlock) -> Option<[u32; 4]> {
    registers
        .sm(0)
        .sm_shiftctrl()
        .modify(|r, w| unsafe { w.bits(r.bits() & !(0b11 << 16 | 0b11 << 30)) });

    // Diagnostic instructions must not apply mandatory side-set to pins.
    registers.sm(0).sm_pinctrl().modify(|_, w| unsafe { w.sideset_count().bits(0) });
    let isr = push_and_read(registers)?;
    inject(registers, 0xa0c1); // MOV ISR, X
    let x = push_and_read(registers)?;
    inject(registers, 0xa0c2); // MOV ISR, Y
    let y = push_and_read(registers)?;
    inject(registers, 0xa0c7); // MOV ISR, OSR
    let osr = push_and_read(registers)?;

    Some([isr, x, y, osr])
}

fn push_and_read(registers: &pac::pio0::RegisterBlock) -> Option<u32> {
    inject(registers, 0x8000); // PUSH NOBLOCK

    for _ in 0..4096 {
        if registers.fstat().read().rxempty().bits() & 1 == 0 {
            return Some(registers.rxf(0).read().bits());
        }
        spin_loop();
    }
    None
}

fn inject(registers: &pac::pio0::RegisterBlock, instruction: u16) {
    registers.sm(0).sm_instr().write(|w| unsafe { w.bits(instruction as u32) });
}

fn jmp(address: u8) -> u16 {
    address as u16
}

fn send(usb: &mut UsbDevice<'_, hal::usb::UsbBus>, serial: &mut SerialPort<'_, hal::usb::UsbBus>, mut bytes: &[u8]) {
    while !bytes.is_empty() {
        match serial.write(bytes) {
            Ok(count) if count > 0 => bytes = &bytes[count..],
            Ok(_) | Err(usb_device::UsbError::WouldBlock) if serial.dtr() => {
                usb.poll(&mut [serial]);
            }
            _ => return,
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
