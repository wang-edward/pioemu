use std::error::Error;
use std::io::{BufRead, BufReader, Write};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use arbitrary_int::u5;
use pioemu::instr::Instr;
use pioemu::state::{Block, Config};
use pioemu_protocol::{CSV_HEADER, MAX_TX, MAX_WORDS, Request, osr_reachable};
use serialport::{SerialPort, SerialPortType};

const RANDOM_TICKS: u32 = 32;
const RANDOM_MAX_LEN: u64 = 8;
const MARKER_NOP: u16 = 0xa042;
const MARKER_EXECCTRL: u32 = (MAX_WORDS as u32) << 7 | (MAX_WORDS as u32 + 1) << 12;

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.iter().map(String::as_str).collect::<Vec<_>>().as_slice() {
        ["bootsel"] => {
            drop(open_port(1200)?);
            std::thread::sleep(Duration::from_secs(3));
            Ok(())
        }
        [count] => run_random(count.parse()?, SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos() as u64),
        [count, seed] => run_random(count.parse()?, seed.parse()?),
        _ => Err("usage: difftest <count> [seed] | difftest bootsel".into()),
    }
}

fn run_random(count: u64, seed: u64) -> Result<(), Box<dyn Error>> {
    let mut hardware = Hardware::open()?;
    let (mut failed, mut undefined) = (0, 0);
    for program_seed in seed..seed + count {
        let request = random_request(&mut Rng(program_seed));
        let rows = hardware.run(&request)?;
        let failure = match std::panic::catch_unwind(|| emulate(&request)) {
            Ok(emulator) => first_difference(&rows, &emulator),
            Err(payload) => match payload
                .downcast_ref::<String>()
                .map(String::as_str)
                .or(payload.downcast_ref::<&str>().copied())
            {
                Some(message) if message.starts_with("undefined") => {
                    undefined += 1;
                    None
                }
                _ => Some("Emulator panicked\n".to_owned()),
            },
        };
        if let Some(failure) = failure {
            failed += 1;
            println!("seed {program_seed}:\n{}{failure}", disassemble(&request));
        }
    }
    println!(
        "{} of {count} programs matched, {undefined} hit undefined behaviour",
        count - failed - undefined
    );
    if failed == 0 { Ok(()) } else { Err("Emulator mismatch".into()) }
}

struct Hardware {
    reader: BufReader<Box<dyn SerialPort>>,
}

fn open_port(baud_rate: u32) -> Result<Box<dyn SerialPort>, Box<dyn Error>> {
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
    Ok(serialport::new(&port.port_name, baud_rate).timeout(Duration::from_secs(5)).open()?)
}

impl Hardware {
    fn open() -> Result<Self, Box<dyn Error>> {
        let mut serial = open_port(115_200)?;
        serial.write_data_terminal_ready(true)?;
        Ok(Self { reader: BufReader::new(serial) })
    }

    fn run(&mut self, request: &Request) -> Result<Vec<String>, Box<dyn Error>> {
        let mut message = String::new();
        request.write(&mut message)?;
        let serial = self.reader.get_mut();
        serial.write_all(message.as_bytes())?;
        serial.flush()?;

        let mut line = String::new();
        let mut rows = Vec::new();
        let mut header_seen = false;
        loop {
            line.clear();
            if self.reader.read_line(&mut line)? == 0 {
                return Err("USB connection closed before DONE".into());
            }
            let response = line.trim_end();
            if let Some(error) = response.strip_prefix("ERR ") {
                return Err(error.to_owned().into());
            }
            if response == "DONE" {
                if !header_seen || rows.len() != request.ticks as usize {
                    return Err("Incomplete firmware trace".into());
                }
                return Ok(rows);
            }
            if !header_seen {
                if response != CSV_HEADER {
                    return Err(format!("Unexpected firmware response: {response}").into());
                }
                header_seen = true;
            } else {
                let tick = rows.len() as u32 + 1;
                if tick > request.ticks
                    || response.split(',').count() != CSV_HEADER.split(',').count()
                    || response.split(',').next().and_then(|tick| tick.parse::<u32>().ok()) != Some(tick)
                {
                    return Err(format!("Invalid trace row: {response}").into());
                }
                rows.push(response.to_owned());
            }
        }
    }
}

fn emulate(request: &Request) -> Vec<String> {
    let mut words = [0u16; 32];
    words[request.origin as usize..][..request.len].copy_from_slice(&request.code[..request.len]);
    words[MAX_WORDS..].fill(MARKER_NOP);
    let mut block = Block::new();
    for (instr, word) in block.instr_mem.iter_mut().zip(words) {
        *instr = Some(Instr::decode(word, request.side_bits(), request.side_optional()));
    }

    let [test, marker, ..] = &mut block.sms;
    test.config = Config::from_registers(request.execctrl, request.shiftctrl, request.pinctrl);
    test.state.pc = u5::new(request.origin);
    for word in &request.tx[..request.tx_len] {
        test.write_tx(*word);
    }
    test.state.x = request.x;
    test.state.y = request.y;
    test.state.isr = request.isr;
    test.state.isr_shift_count = request.isr_count;
    test.state.osr = request.osr;
    test.state.osr_shift_count = request.osr_count;
    block.irq_flags = request.irq;
    block.gpio_in = request.gpio_in;
    marker.config = Config::from_registers(MARKER_EXECCTRL, 0x000c_0000, 0);
    marker.state.pc = u5::new(MAX_WORDS as u8);
    test.enabled = true;
    marker.enabled = true;

    (1..=request.ticks)
        .map(|tick| {
            block.step();
            let flevel = block.sms.iter().enumerate().fold(0, |flevel, (i, sm)| {
                flevel | (sm.state.tx_fifo.len() as u32) << (8 * i) | (sm.state.rx_fifo.len() as u32) << (8 * i + 4)
            });
            let fdebug = block.sms.iter().enumerate().fold(0, |fdebug, (i, sm)| {
                fdebug
                    | (sm.state.rx_stalled as u32) << i
                    | (sm.state.tx_over as u32) << (16 + i)
                    | (sm.state.tx_stalled as u32) << (24 + i)
            });
            let (test, marker) = (&block.sms[0].state, &block.sms[1].state);
            let mut rx = test.rx_putget;
            for (destination, word) in rx.iter_mut().zip(test.rx_fifo.iter()) {
                *destination = *word;
            }
            format!(
                "{},{},{},{:08x},{:08x},{:02x},{:08x},{:08x},{},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x}",
                tick,
                test.pc.value(),
                marker.pc.value(),
                block.gpio_out,
                block.gpio_dir,
                block.irq_flags,
                flevel,
                fdebug,
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

fn first_difference(hardware: &[String], emulator: &[String]) -> Option<String> {
    hardware.iter().zip(emulator).find_map(|(hardware_row, emulator_row)| {
        let columns: String = CSV_HEADER
            .split(',')
            .zip(hardware_row.split(',').zip(emulator_row.split(',')))
            .filter(|(_, (hw, emu))| hw != emu)
            .map(|(column, (hw, emu))| format!("  {column}: hardware {hw}, emulator {emu}\n"))
            .collect();
        let tick = hardware_row.split(',').next().unwrap_or("?");
        (!columns.is_empty()).then(|| format!("Emulator differs from hardware at tick {tick}:\n{columns}"))
    })
}

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    fn below(&mut self, n: u64) -> u16 {
        (self.next() % n) as u16
    }
}

fn random_word(rng: &mut Rng, origin: u8, len: usize) -> u16 {
    let (opcode, a, b) = match rng.below(9) {
        0 => (0, rng.below(8), origin as u16 + rng.below(len as u64)),
        1 => match rng.below(4) {
            3 => (1, rng.below(2) << 2 | 3, rng.below(4)),
            source => (1, rng.below(2) << 2 | source, rng.below(32)),
        },
        2 => (2, rng.below(8), rng.below(32)),
        3 => (3, rng.below(8), rng.below(32)),
        4 => match rng.below(3) {
            0 => (4, rng.below(8), 0x10 | rng.below(2) * (8 | rng.below(8))),
            _ => (4, rng.below(8), rng.below(16)),
        },
        5 => (5, rng.below(8), rng.below(32)),
        6 => (6, rng.below(4), rng.below(32)),
        _ => (7, rng.below(8), rng.below(32)),
    };
    let delay = if rng.below(2) == 0 { rng.below(32) } else { 0 };
    opcode << 13 | delay << 8 | a << 5 | b
}

fn random_value(rng: &mut Rng, origin: u8, len: usize) -> u32 {
    match rng.below(5) {
        0 => 0,
        1 => u32::MAX,
        2 => 1 << rng.below(32),
        3 => random_word(rng, origin, len) as u32,
        _ => rng.next() as u32,
    }
}

fn random_request(rng: &mut Rng) -> Request {
    let len = 1 + rng.below(RANDOM_MAX_LEN) as usize;
    let origin = rng.below((MAX_WORDS - len + 1) as u64) as u8;
    let wrap_top = origin as u32 + rng.below(len as u64) as u32;
    let wrap_bottom = origin as u32 + rng.below(wrap_top as u64 - origin as u64 + 1) as u32;
    let mut field = |values: u64, lsb: u32| (rng.below(values) as u32) << lsb;
    let side_count = field(6, 0);
    let side_en = if side_count > 0 { field(2, 30) } else { 0 };
    let execctrl = field(32, 0) | field(4, 5) | field(128, 17) | field(32, 24) | wrap_bottom << 7 | wrap_top << 12 | field(2, 29) | side_en;
    let putget = if field(2, 0) == 0 { field(4, 14) } else { 0 };
    let autopush = if putget == 0 { field(2, 16) } else { 0 };
    let shiftctrl = field(32, 0) | putget | autopush | field(8, 17) | field(32, 20) | field(32, 25) | field(4, 30);
    let pinctrl = field(32, 0) | field(32, 5) | field(32, 10) | field(32, 15) | field(33, 20) | field(6, 26) | side_count << 29;
    let mut request = Request {
        ticks: RANDOM_TICKS,
        execctrl,
        shiftctrl,
        pinctrl,
        origin,
        len,
        code: [0; MAX_WORDS],
        tx_len: rng.below(MAX_TX as u64 + 1) as usize,
        tx: [0; MAX_TX],
        x: random_value(rng, origin, len),
        y: random_value(rng, origin, len),
        isr: random_value(rng, origin, len),
        isr_count: rng.below(33) as u8,
        osr: random_value(rng, origin, len),
        osr_count: rng.below(33) as u8,
        irq: rng.below(256) as u8,
        gpio_in: rng.next() as u32,
    };
    request.osr = osr_reachable(request.osr, request.osr_count, request.out_shift_right());
    for word in &mut request.code[..len] {
        *word = random_word(rng, origin, len);
    }
    for word in &mut request.tx[..request.tx_len] {
        *word = random_value(rng, origin, len);
    }
    request
}

fn disassemble(request: &Request) -> String {
    let mut text = format!(
        "  execctrl {:08x}, shiftctrl {:08x}, pinctrl {:08x}, tx {:08x?}\n  x {:08x}, y {:08x}, isr {:08x} ({}), osr {:08x} ({}), irq {:02x}, gpio_in {:08x}\n",
        request.execctrl,
        request.shiftctrl,
        request.pinctrl,
        &request.tx[..request.tx_len],
        request.x,
        request.y,
        request.isr,
        request.isr_count,
        request.osr,
        request.osr_count,
        request.irq,
        request.gpio_in
    );
    for (slot, word) in (request.origin as usize..).zip(&request.code[..request.len]) {
        text += &format!(
            "  {slot:2}: {word:04x}  {}\n",
            Instr::decode(*word, request.side_bits(), request.side_optional())
        );
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn emulated_trace() {
        let mut request = Request {
            ticks: 16,
            execctrl: 1 << 30 | 6 << 12 | 4 << 7,
            shiftctrl: 0x000c_0000,
            pinctrl: 2 << 29 | 5 << 26,
            origin: 4,
            len: 3,
            code: [0; MAX_WORDS],
            tx_len: 0,
            tx: [0; MAX_TX],
            x: 0,
            y: 0,
            isr: 0,
            isr_count: 0,
            osr: 0,
            osr_count: 32,
            irq: 0,
            gpio_in: 0,
        };
        request.code[..3].copy_from_slice(&[0xf823, 0x0245, 0x80a0]);
        let rows = emulate(&request);
        let field = |tick: usize, column: &str| {
            let index = CSV_HEADER.split(',').position(|c| c == column).unwrap();
            rows[tick - 1].split(',').nth(index).unwrap().to_owned()
        };
        assert_eq!((field(1, "test_pc"), field(1, "x")), ("5".into(), "00000003".into()));
        assert_eq!((field(1, "marker_pc"), field(2, "marker_pc")), ("31".into(), "30".into()));
        assert_eq!((field(10, "test_pc"), field(10, "x")), ("5".into(), "00000000".into()));
        assert_eq!((field(11, "test_pc"), field(11, "x")), ("6".into(), "ffffffff".into()));
        assert_eq!(field(16, "test_pc"), "6");
    }
    #[test]
    fn random_requests_are_valid() {
        for seed in 0..10_000 {
            let request = random_request(&mut Rng(seed));
            request.validate().unwrap();
            disassemble(&request);
        }
    }
}
