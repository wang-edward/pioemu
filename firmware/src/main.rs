#![no_std]
#![no_main]

use core::fmt::Write;
use core::hint::spin_loop;

use hal::pio::{PIOBuilder, PIOExt};
use hal::{entry, pac};
use heapless::String;
use panic_halt as _;
use pio::{InstructionOperands, JmpCondition, MovDestination, MovOperation, MovSource};
use rp235x_hal as hal;
use rp235x_hal::clocks::init_clocks_and_plls;
use usb_device::LangID;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

const CLOCK_DIVIDER: u16 = 60_000;
const TICKS_PER_RUN: u32 = 16;
const TEST_ORIGIN: usize = 0;
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
    stalled: bool,
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
    let test_code = test_program.code.clone();

    // These two shared instruction-memory slots are the clock marker. SM1's PC
    // changes on every clock tick, independently of SM0's delay or stall state.
    let marker_program = pio_proc::pio_asm!(".origin 30", ".wrap_target", "nop", "nop", ".wrap",).program;
    let marker_code = marker_program.code.clone();

    let test_program = pio.install(&test_program).unwrap();
    let marker_program = pio.install(&marker_program).unwrap();

    let (mut test_sm, _test_rx, _test_tx) = PIOBuilder::from_installed_program(test_program)
        .clock_divisor_fixed_point(CLOCK_DIVIDER, 0)
        .build(sm0);
    let (mut marker_sm, _marker_rx, _marker_tx) = PIOBuilder::from_installed_program(marker_program)
        .clock_divisor_fixed_point(CLOCK_DIVIDER, 0)
        .build(sm1);

    let registers = unsafe { &*pac::PIO0::ptr() };
    let configs = [sm_config(registers, 0), sm_config(registers, 1)];

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
            b"ticks_elapsed,test_pc,marker_pc,stalled,padout,padoe,irq,flevel,fdebug,rx_count,rx0,rx1,rx2,rx3,x,y,isr,osr\r\n",
        );

        for ticks_elapsed in 1..=TICKS_PER_RUN {
            reset_and_initialize(
                &peripherals.RESETS,
                registers,
                test_code.as_slice(),
                marker_code.as_slice(),
                configs,
            );

            for _ in 0..ticks_elapsed {
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
            }

            // Save non-destructive state before the diagnostic instructions alter SM0.
            let snapshot = Snapshot {
                test_pc: test_sm.instruction_address(),
                marker_pc: marker_sm.instruction_address(),
                stalled: test_sm.stalled(),
                padout: registers.dbg_padout().read().bits(),
                padoe: registers.dbg_padoe().read().bits(),
                irq: pio.get_irq_raw(),
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

            let hidden = extract_hidden_registers(registers).unwrap_or([u32::MAX; 4]);

            let mut line = String::<320>::new();
            writeln!(
                line,
                "{},{},{},{},{:08x},{:08x},{:02x},{:08x},{:08x},{},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x}\r",
                ticks_elapsed,
                snapshot.test_pc,
                snapshot.marker_pc,
                snapshot.stalled as u8,
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
        }
    }
}

fn sm_config(registers: &pac::pio0::RegisterBlock, index: usize) -> SmConfig {
    let sm = registers.sm(index);
    SmConfig {
        clkdiv: sm.sm_clkdiv().read().bits(),
        execctrl: sm.sm_execctrl().read().bits(),
        shiftctrl: sm.sm_shiftctrl().read().bits(),
        pinctrl: sm.sm_pinctrl().read().bits(),
    }
}

fn reset_and_initialize(
    resets: &pac::RESETS,
    registers: &pac::pio0::RegisterBlock,
    test_code: &[u16],
    marker_code: &[u16],
    configs: [SmConfig; 2],
) {
    resets.reset().modify(|_, w| w.pio0().set_bit());
    resets.reset().modify(|_, w| w.pio0().clear_bit());
    while resets.reset_done().read().pio0().bit_is_clear() {
        spin_loop();
    }

    for (offset, instruction) in test_code.iter().enumerate() {
        registers
            .instr_mem(TEST_ORIGIN + offset)
            .write(|w| unsafe { w.bits(*instruction as u32) });
    }
    for (offset, instruction) in marker_code.iter().enumerate() {
        registers
            .instr_mem(MARKER_ORIGIN + offset)
            .write(|w| unsafe { w.bits(*instruction as u32) });
    }

    for (index, config) in configs.iter().enumerate() {
        let sm = registers.sm(index);
        sm.sm_clkdiv().write(|w| unsafe { w.bits(config.clkdiv) });
        sm.sm_execctrl().write(|w| unsafe { w.bits(config.execctrl) });
        sm.sm_shiftctrl().write(|w| unsafe { w.bits(config.shiftctrl) });
        sm.sm_pinctrl().write(|w| unsafe { w.bits(config.pinctrl) });
    }

    // Clear pending instructions, stalls, delays and shift counters in both SMs.
    registers.ctrl().write(|w| unsafe { w.bits(0b11 << 4) });
    inject(registers, jmp(TEST_ORIGIN as u8));
    registers
        .sm(1)
        .sm_instr()
        .write(|w| unsafe { w.bits(jmp(MARKER_ORIGIN as u8) as u32) });

    // Restart both divided clocks in one write so future grouped starts are aligned.
    registers.ctrl().write(|w| unsafe { w.bits(0b11 << 8) });
}

fn extract_hidden_registers(registers: &pac::pio0::RegisterBlock) -> Option<[u32; 4]> {
    registers
        .sm(0)
        .sm_shiftctrl()
        .modify(|_, w| w.autopush().clear_bit().autopull().clear_bit());

    let isr = push_and_read(registers)?;
    inject(registers, mov_to_isr(MovSource::X));
    let x = push_and_read(registers)?;
    inject(registers, mov_to_isr(MovSource::Y));
    let y = push_and_read(registers)?;
    inject(registers, mov_to_isr(MovSource::OSR));
    let osr = push_and_read(registers)?;

    Some([isr, x, y, osr])
}

fn push_and_read(registers: &pac::pio0::RegisterBlock) -> Option<u32> {
    inject(registers, InstructionOperands::PUSH { if_full: false, block: false }.encode());

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
    InstructionOperands::JMP { condition: JmpCondition::Always, address }.encode()
}

fn mov_to_isr(source: MovSource) -> u16 {
    InstructionOperands::MOV { destination: MovDestination::ISR, op: MovOperation::None, source }.encode()
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
