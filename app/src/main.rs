use std::error::Error;
use std::io::{BufRead, BufReader, Write};
use std::time::Duration;

use pioemu_protocol::{CSV_HEADER, MAX_WORDS, Request};
use serialport::SerialPortType;

const TICKS: u32 = 16;

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
            return Ok(());
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
        }
        println!("{response}");
    }
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
}
