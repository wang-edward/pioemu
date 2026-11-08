use std::error::Error;
use std::io::{BufRead, BufReader, Write};
use std::time::Duration;

use arbitrary_int::u5;
use pioemu::instr::Instr;
use pioemu::state::Block;
use pioemu_protocol::{CSV_HEADER, MAX_WORDS, Request};
use serialport::SerialPortType;

const TICKS: u32 = 16;
// Must match the firmware's clock marker: two NOPs (MOV Y, Y) in slots 30 and 31.
const MARKER_ORIGIN: u8 = 30;
const MARKER_NOP: u16 = 0xa042;
// EXEC_STALLED only covers injected instructions, and the emulator doesn't track FDEBUG's sticky flags.
const UNCOMPARED: [&str; 2] = ["stalled", "fdebug"];

fn main() -> Result<(), Box<dyn Error>> {
    // Edit the program here. Cargo assembles it on the PC at build time.
    let program = pio_proc::pio_asm!(
        ".origin 0",
        ".wrap_target",
        "set x, 3",
        "count:",
        "jmp x-- count [2]",
        "pull block",
        ".wrap",
    )
    .program;

    let request = make_request(&program, TICKS)?;
    let mut message = String::new();
    request.write(&mut message)?;

    let ports: Vec<_> = serialport::available_ports()?
        .into_iter()
        // macOS lists both /dev/cu.* and /dev/tty.* for each device. Use the callout port so one Pico is counted only once.
        .filter(|port| !cfg!(target_os = "macos") || port.port_name.starts_with("/dev/cu."))
        .filter(|port| {
            matches!(&port.port_type, SerialPortType::UsbPort(info)
            if info.vid == 0x1209 && info.pid == 0x2350)
        })
        .collect();
    let port = match ports.as_slice() {
        [port] => port,
        [] => return Err("No pioemu USB device found; connect a Pico 2 running the firmware".into()),
        _ => return Err("More than one pioemu device found; connect only one".into()),
    };
    let mut serial = serialport::new(&port.port_name, 115_200).timeout(Duration::from_secs(5)).open()?;
    serial.write_data_terminal_ready(true)?;
    serial.write_all(message.as_bytes())?;
    serial.flush()?;

    let mut reader = BufReader::new(serial);
    let mut line = String::new();
    let mut rows = 0;
    let mut hardware = Vec::new();
    let mut header_seen = false;
    loop {
        line.clear();
        if reader.read_line(&mut line)? == 0 {
            return Err("USB connection closed before DONE".into());
        }
        let response = line.trim_end();
        if let Some(error) = response.strip_prefix("ERR ") {
            return Err(error.to_owned().into());
        }
        if response == "DONE" {
            if !header_seen || rows != TICKS {
                return Err("Incomplete firmware trace".into());
            }
            break;
        }
        if !header_seen {
            if response != CSV_HEADER {
                return Err(format!("Unexpected firmware response: {response}").into());
            }
            header_seen = true;
        } else {
            rows += 1;
            if rows > TICKS
                || response.split(',').count() != 18
                || response.split(',').next().and_then(|tick| tick.parse::<u32>().ok()) != Some(rows)
            {
                return Err(format!("Invalid trace row: {response}").into());
            }
            hardware.push(response.to_owned());
        }
        println!("{response}");
    }
    compare(&hardware, &emulate(&request))
}

/// Runs the request on pioemu, configured like the firmware, and formats each tick as a firmware CSV row.
fn emulate(request: &Request) -> Vec<String> {
    let mut block = Block::new();
    for (slot, instr) in block.instr_mem.iter_mut().enumerate() {
        // The firmware resets PIO0 first, so unused slots hold 0 (JMP 0).
        let word = match slot.checked_sub(request.origin as usize) {
            Some(offset) if offset < request.len => request.code[offset],
            _ if slot >= MARKER_ORIGIN as usize => MARKER_NOP,
            _ => 0,
        };
        *instr = Some(Instr::decode(word, request.side_bits, request.side_optional));
    }

    let [test, marker, ..] = &mut block.sms;
    test.config.set_wrap(u5::new(request.wrap_target), u5::new(request.wrap_source));
    test.config
        .set_sideset(request.side_bits, request.side_optional, request.side_pindirs);
    test.state.pc = u5::new(request.origin);
    marker.config.set_wrap(u5::new(MARKER_ORIGIN), u5::new(MARKER_ORIGIN + 1));
    marker.state.pc = u5::new(MARKER_ORIGIN);
    test.enabled = true;
    marker.enabled = true;

    (1..=request.ticks)
        .map(|tick| {
            block.step();
            let flevel = block.sms.iter().enumerate().fold(0, |flevel, (i, sm)| {
                flevel | (sm.state.tx_fifo.len() as u32) << (8 * i) | (sm.state.rx_fifo.len() as u32) << (8 * i + 4)
            });
            let (test, marker) = (&block.sms[0].state, &block.sms[1].state);
            let mut rx = [0u32; 4];
            for (destination, word) in rx.iter_mut().zip(test.rx_fifo.iter()) {
                *destination = *word;
            }
            format!(
                "{},{},{},-,{:08x},{:08x},{:02x},{:08x},-,{},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x}",
                tick,
                test.pc.value(),
                marker.pc.value(),
                block.gpio_out,
                block.gpio_dir,
                block.irq_flags,
                flevel,
                test.rx_fifo.len(),
                rx[0],
                rx[1],
                rx[2],
                rx[3],
                test.x,
                test.y,
                test.isr,
                test.osr,
            )
        })
        .collect()
}

fn compare(hardware: &[String], emulator: &[String]) -> Result<(), Box<dyn Error>> {
    let columns: Vec<_> = CSV_HEADER.split(',').collect();
    for (hardware_row, emulator_row) in hardware.iter().zip(emulator) {
        let differences: Vec<_> = columns
            .iter()
            .zip(hardware_row.split(',').zip(emulator_row.split(',')))
            .filter(|(column, (hw, emu))| !UNCOMPARED.contains(column) && hw != emu)
            .collect();
        if !differences.is_empty() {
            eprintln!(
                "Emulator differs from hardware at tick {}:",
                hardware_row.split(',').next().unwrap_or("?")
            );
            for (column, (hw, emu)) in differences {
                eprintln!("  {column}: hardware {hw}, emulator {emu}");
            }
            return Err("Emulator mismatch".into());
        }
    }
    println!(
        "Emulator matches hardware for all {} ticks ({} not compared)",
        hardware.len(),
        UNCOMPARED.join(", ")
    );
    Ok(())
}

fn make_request(program: &pio::Program<32>, ticks: u32) -> Result<Request, &'static str> {
    let origin = program.origin.unwrap_or(0);
    if program.code.len() > MAX_WORDS || origin as usize + program.code.len() > MAX_WORDS {
        return Err("Program must fit in instruction slots 0..29");
    }
    let mut request = Request {
        ticks,
        origin,
        wrap_target: origin + program.wrap.target,
        wrap_source: origin + program.wrap.source,
        side_bits: program.side_set.bits(),
        side_optional: program.side_set.optional(),
        side_pindirs: program.side_set.pindirs(),
        len: program.code.len(),
        code: [0; MAX_WORDS],
    };
    for (destination, word) in request.code.iter_mut().zip(&program.code) {
        // The assembler emits relative JMP addresses even with .origin.
        *destination = if word >> 13 == 0 {
            let address = (word & 31) + origin as u16;
            if address >= MAX_WORDS as u16 {
                return Err("JMP targets reserved or out-of-range instruction slot");
            }
            (word & !31) | address
        } else {
            *word
        };
    }
    request.validate()?;
    Ok(request)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn assembled_program_round_trip() {
        let program = pio_proc::pio_asm!(
            ".origin 4",
            ".side_set 1 opt",
            ".wrap_target",
            "set x, 3 side 1",
            "loop:",
            "jmp x-- loop [2]",
            "pull block",
            ".wrap"
        )
        .program;
        let request = make_request(&program, 16).unwrap();
        assert_eq!(request.code[1] & 31, 5);
        assert_eq!((request.wrap_target, request.wrap_source), (4, 6));
        assert_eq!(request.side_bits, 2);
        assert!(request.side_optional);
        let mut wire = String::new();
        request.write(&mut wire).unwrap();
        assert_eq!(Request::parse(&wire).unwrap(), request);
    }
    #[test]
    fn emulated_trace() {
        let program = pio_proc::pio_asm!(
            ".origin 4",
            ".side_set 1 opt",
            ".wrap_target",
            "set x, 3 side 1",
            "loop:",
            "jmp x-- loop [2]",
            "pull block",
            ".wrap"
        )
        .program;
        let rows = emulate(&make_request(&program, 16).unwrap());
        let field = |tick: usize, column: &str| {
            let index = CSV_HEADER.split(',').position(|c| c == column).unwrap();
            rows[tick - 1].split(',').nth(index).unwrap().to_owned()
        };
        assert_eq!((field(1, "test_pc"), field(1, "x")), ("5".into(), "00000003".into()));
        assert_eq!((field(1, "marker_pc"), field(2, "marker_pc")), ("31".into(), "30".into()));
        // Three JMPs, each followed by 2 delay ticks, then the fall-through JMP at tick 11.
        assert_eq!((field(10, "test_pc"), field(10, "x")), ("5".into(), "00000000".into()));
        assert_eq!((field(11, "test_pc"), field(11, "x")), ("6".into(), "ffffffff".into()));
        assert_eq!(field(16, "test_pc"), "6");
    }
}
