#![no_std]
#![no_main]

use core::fmt::Write;
use core::hint::spin_loop;

use hal::{entry, pac};
use heapless::String;
use panic_halt as _;
use pioemu_protocol::{CSV_HEADER, Decoder, MAX_TICKS, Request, Sm};
use rp235x_hal as hal;
use rp235x_hal::clocks::init_clocks_and_plls;
use usb_device::LangID;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

const STR_R1_R0: u32 = 0x1000_f8c0;
const NOP_W: u32 = 0x8000_f3af;
const STR_R2_R0: u32 = 0x2000_f8c0;
const BX_LR: u32 = 0x4770;

static mut RUN: [u32; MAX_TICKS as usize + 2] = [0; MAX_TICKS as usize + 2];

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
    let mut seed = 0x2545_f491;

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
            'ticks: for ticks_elapsed in 1..=request.ticks {
                if !serial.dtr() {
                    break;
                }
                reset_and_initialize(&peripherals.RESETS, registers, &request, &mut seed);
                cortex_m::interrupt::free(|_| run(registers.ctrl().as_ptr(), request.mask() as u32 | 0xf << 8, ticks_elapsed));
                usb.poll(&mut [&mut serial]);

                let pcs: [u32; 4] = core::array::from_fn(|index| registers.sm(index).sm_addr().read().bits());
                let mut line = String::<512>::new();
                write!(
                    line,
                    "{},{:08x},{:08x},{:02x},{:08x},{:08x}",
                    ticks_elapsed,
                    registers.dbg_padout().read().bits(),
                    registers.dbg_padoe().read().bits(),
                    registers.irq().read().bits(),
                    registers.flevel().read().bits(),
                    registers.fdebug().read().bits(),
                )
                .unwrap();
                for (index, pc) in pcs.iter().enumerate() {
                    match request.sms[index].map(|_| read_sm(registers, index)) {
                        None => line.push_str(",,,,,,,,,,").unwrap(),
                        Some(None) => {
                            send(&mut usb, &mut serial, b"ERR register extraction failed\r\n");
                            break 'ticks;
                        }
                        Some(Some((rx_count, rx, [isr, x, y, osr]))) => write!(
                            line,
                            ",{},{},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x}",
                            pc, rx_count, rx[0], rx[1], rx[2], rx[3], x, y, isr, osr
                        )
                        .unwrap(),
                    }
                }
                line.push_str("\r\n").unwrap();
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

fn reset_and_initialize(resets: &pac::RESETS, registers: &pac::pio0::RegisterBlock, request: &Request, seed: &mut u32) {
    resets.reset().modify(|_, w| w.pio0().set_bit());
    resets.reset().modify(|_, w| w.pio0().clear_bit());
    while resets.reset_done().read().pio0().bit_is_clear() {
        spin_loop();
    }

    let mut junk = || {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 17;
        *seed ^= *seed << 5;
        *seed
    };
    for (index, sm) in request.sms.iter().enumerate() {
        registers.sm(index).sm_shiftctrl().write(|w| unsafe { w.bits(1 << 14) });
        let rx = sm.map_or([junk(), junk(), junk(), junk()], |sm| sm.rx);
        for (entry, word) in rx.iter().enumerate() {
            unsafe { putget_entry(registers, index, entry).write_volatile(*word) };
        }
        if sm.is_none() {
            for instruction in [0xe020 | junk() & 31, 0xe040 | junk() & 31, 0xa0c9, 0xa0f2] {
                registers.sm(index).sm_instr().write(|w| unsafe { w.bits(instruction) });
            }
        }
        let config = sm.map_or([junk(), junk(), junk(), junk()], |sm| [sm.clkdiv, sm.execctrl, sm.shiftctrl, sm.pinctrl]);
        registers.sm(index).sm_clkdiv().write(|w| unsafe { w.bits(config[0]) });
        registers.sm(index).sm_execctrl().write(|w| unsafe { w.bits(config[1]) });
        registers.sm(index).sm_shiftctrl().write(|w| unsafe { w.bits(config[2]) });
        registers.sm(index).sm_pinctrl().write(|w| unsafe { w.bits(config[3]) });
    }
    for (slot, instruction) in request.code.iter().enumerate() {
        registers.instr_mem(slot).write(|w| unsafe { w.bits(*instruction as u32) });
    }

    registers.ctrl().write(|w| unsafe { w.bits(0xf << 4) });
    for (index, sm) in request.sms.iter().enumerate() {
        if let Some(sm) = sm {
            load_state(registers, index, sm);
            inject(registers, index, sm.pc as u16);
            for word in &sm.tx[..sm.tx_len] {
                registers.txf(index).write(|w| unsafe { w.bits(*word) });
            }
        }
    }
    registers.irq_force().write(|w| unsafe { w.bits(request.irq as u32) });
}

fn load_state(registers: &pac::pio0::RegisterBlock, index: usize, request: &Sm) {
    let sm = registers.sm(index);
    sm.sm_clkdiv().write(|w| unsafe { w.bits(1 << 16) });
    sm.sm_shiftctrl().write(|w| unsafe { w.bits(request.shiftctrl & !(0b1111 << 14 | 0b11 << 30)) });

    let (isr, isr_in) = match (request.isr_count, request.in_shift_right()) {
        (0, _) => (request.isr, 0),
        (32, _) => (0, request.isr),
        (count, false) => (request.isr >> count, request.isr & ((1 << count) - 1)),
        (count, true) => (request.isr << count, request.isr >> (32 - count)),
    };
    load_osr(registers, index, isr);
    execute(registers, index, 0xa0c7); // MOV ISR, OSR
    if request.isr_count > 0 {
        load_osr(registers, index, isr_in);
        execute(registers, index, 0x40e0 | (request.isr_count as u16 & 31)); // IN OSR, count
    }
    load_osr(registers, index, request.x);
    execute(registers, index, 0xa027); // MOV X, OSR
    load_osr(registers, index, request.y);
    execute(registers, index, 0xa047); // MOV Y, OSR
    let osr = match (request.osr_count, request.out_shift_right()) {
        (0 | 32, _) => request.osr,
        (count, true) => request.osr << count,
        (count, false) => request.osr >> count,
    };
    load_osr(registers, index, osr);
    if request.osr_count > 0 {
        execute(registers, index, 0x6060 | (request.osr_count as u16 & 31)); // OUT NULL, count
    }

    sm.sm_shiftctrl().write(|w| unsafe { w.bits(request.shiftctrl) });
    sm.sm_clkdiv().write(|w| unsafe { w.bits(request.clkdiv) });
}

fn load_osr(registers: &pac::pio0::RegisterBlock, index: usize, word: u32) {
    registers.txf(index).write(|w| unsafe { w.bits(word) });
    execute(registers, index, 0x80a0); // PULL BLOCK
}

fn execute(registers: &pac::pio0::RegisterBlock, index: usize, instruction: u16) {
    inject(registers, index, instruction);
    while registers.sm(index).sm_execctrl().read().exec_stalled().bit_is_set() {
        spin_loop();
    }
}

fn read_sm(registers: &pac::pio0::RegisterBlock, index: usize) -> Option<(usize, [u32; 4], [u32; 4])> {
    let sm = registers.sm(index);
    let putget = sm.sm_shiftctrl().read().bits() >> 14 & 3;
    let mut rx = [0u32; 4];
    let mut rx_count = 0usize;
    if putget != 0 {
        sm.sm_shiftctrl().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 14) | 1 << 15) });
        for (entry, word) in rx.iter_mut().enumerate() {
            *word = unsafe { putget_entry(registers, index, entry).read_volatile() };
        }
    }
    while putget == 0 && registers.fstat().read().rxempty().bits() >> index & 1 == 0 {
        let word = registers.rxf(index).read().bits();
        if rx_count < rx.len() {
            rx[rx_count] = word;
        }
        rx_count += 1;
    }

    sm.sm_shiftctrl().modify(|r, w| unsafe { w.bits(r.bits() & !(0b1111 << 14 | 0b11 << 30)) });
    // Diagnostic instructions must not apply mandatory side-set to pins.
    sm.sm_pinctrl().modify(|_, w| unsafe { w.sideset_count().bits(0) });
    let isr = push_and_read(registers, index)?;
    inject(registers, index, 0xa0c1); // MOV ISR, X
    let x = push_and_read(registers, index)?;
    inject(registers, index, 0xa0c2); // MOV ISR, Y
    let y = push_and_read(registers, index)?;
    inject(registers, index, 0xa0c7); // MOV ISR, OSR
    let osr = push_and_read(registers, index)?;

    Some((rx_count, rx, [isr, x, y, osr]))
}

fn push_and_read(registers: &pac::pio0::RegisterBlock, index: usize) -> Option<u32> {
    inject(registers, index, 0x8000); // PUSH NOBLOCK

    for _ in 0..4096 {
        if registers.fstat().read().rxempty().bits() >> index & 1 == 0 {
            return Some(registers.rxf(index).read().bits());
        }
        spin_loop();
    }
    None
}

fn inject(registers: &pac::pio0::RegisterBlock, index: usize, instruction: u16) {
    registers.sm(index).sm_instr().write(|w| unsafe { w.bits(instruction as u32) });
}

fn putget_entry(registers: &pac::pio0::RegisterBlock, index: usize, entry: usize) -> *mut u32 {
    registers.rxf0_putget(0).as_ptr().wrapping_add(4 * index + entry)
}

fn run(ctrl: *mut u32, enable: u32, cycles: u32) {
    let code = unsafe { &mut *core::ptr::addr_of_mut!(RUN) };
    code[0] = STR_R1_R0;
    code[1..cycles as usize].fill(NOP_W);
    code[cycles as usize..][..2].copy_from_slice(&[STR_R2_R0, BX_LR]);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    let run: extern "C" fn(*mut u32, u32, u32) = unsafe { core::mem::transmute(code.as_ptr() as usize | 1) };
    run(ctrl, enable, 0);
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
