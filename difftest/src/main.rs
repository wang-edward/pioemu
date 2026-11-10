use std::error::Error;
use std::io::{BufRead, BufReader, Write};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use arbitrary_int::u5;
use pioemu::instr::Instr;
use pioemu::state::{Block, Config};
use pioemu_protocol::{CSV_HEADER, MAX_TX, Request, Sm, osr_reachable};
use serialport::{SerialPort, SerialPortType};

mod cases;

const RANDOM_TICKS: u32 = 32;
const RANDOM_MAX_LEN: u64 = 8;
const RECORD_RUNS: u64 = 50;

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.iter().map(String::as_str).collect::<Vec<_>>().as_slice() {
        ["bootsel"] => {
            drop(open_port(1200)?);
            std::thread::sleep(Duration::from_secs(3));
            Ok(())
        }
        ["sweep", state] => run(sweep(state.parse()?)),
        ["record-cases"] => {
            let mut hardware = Hardware::open()?;
            let mut nondeterministic = Vec::new();
            for (name, request) in cases::cases().into_iter().filter_map(|(name, _, _, test)| match test {
                cases::Test::Pass(request) => Some((name, request)),
                cases::Test::Untestable(reason) => {
                    println!("{name}: not testable, {reason}");
                    None
                }
            }) {
                let trace = hardware.run(&request)?;
                for seed in 1..RECORD_RUNS {
                    let (mut code, mut rng) = (request.code, Rng(seed));
                    let len = request
                        .sms
                        .iter()
                        .flatten()
                        .map(|sm| sm.execctrl as usize >> 12 & 31)
                        .max()
                        .unwrap_or(0)
                        + 1;
                    for word in &mut code[len..] {
                        *word = random_word(&mut rng, 0, 32);
                    }
                    if hardware.run(&Request { code, ..request })? != trace {
                        nondeterministic.push(name);
                        break;
                    }
                }
                let mut fixture = String::new();
                request.write(&mut fixture)?;
                fixture += &format!("{CSV_HEADER}\n{}\n", trace.join("\n"));
                std::fs::write(format!("{}/cases/{name}.csv", env!("CARGO_MANIFEST_DIR")), fixture)?;
            }
            match nondeterministic.as_slice() {
                [] => Ok(()),
                names => Err(format!("traces differ between runs: {}", names.join(", ")).into()),
            }
        }
        [count] | [count, _] => {
            let seed = match args.get(1) {
                Some(seed) => seed.parse()?,
                None => SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos() as u64,
            };
            run((seed..seed + count.parse::<u64>()?)
                .map(|seed| (format!("seed {seed}"), random_request(&mut Rng(seed))))
                .collect())
        }
        _ => Err(format!(
            "usage: difftest <count> [seed] | difftest sweep <0..{}> | difftest record-cases | difftest bootsel",
            SWEEP_STATES.len() - 1
        )
        .into()),
    }
}

fn run(requests: Vec<(String, Request)>) -> Result<(), Box<dyn Error>> {
    let mut hardware = Hardware::open()?;
    let mut failed = Vec::new();
    let start = Instant::now();
    std::panic::set_hook(Box::new(|_| {}));
    for (index, (label, request)) in requests.iter().enumerate() {
        let failure = match std::panic::catch_unwind(|| emulate(request, false)) {
            Ok(emulator) => first_difference(&hardware.run(request)?, &emulator),
            Err(payload) => Some(format!("Emulator panicked: {}\n", panic_message(&*payload))),
        };
        if let Some(failure) = failure {
            failed.push(label.as_str());
            println!("{label}:\n{}{failure}", disassemble(request));
        }
        eprint!(
            "\r{} of {} programs, {:.0}/s ",
            index + 1,
            requests.len(),
            (index + 1) as f64 / start.elapsed().as_secs_f64()
        );
    }
    println!("\n{} of {} programs matched", requests.len() - failed.len(), requests.len());
    if failed.is_empty() {
        Ok(())
    } else {
        println!("failed: {}", failed.join(", "));
        Err("Emulator mismatch".into())
    }
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

fn panic_message(payload: &(dyn std::any::Any + Send)) -> &str {
    payload
        .downcast_ref::<String>()
        .map(String::as_str)
        .or(payload.downcast_ref::<&str>().copied())
        .unwrap_or("?")
}

fn emulate(request: &Request, strict: bool) -> Vec<String> {
    let mut block = Block::new(strict);
    block.instr_mem = request.code;
    block.irq_flags = request.irq as u32;
    block.gpio_in = request.gpio_in;
    for (i, request) in request.sms.into_iter().enumerate() {
        if let Some(request) = request {
            block.configure(i, request.clkdiv, request.execctrl, request.shiftctrl, request.pinctrl);
            let sm = &mut block.sms[i];
            sm.state.pc = u5::new(request.pc);
            for word in &request.tx[..request.tx_len] {
                sm.write_tx(*word);
            }
            sm.state.x = request.x;
            sm.state.y = request.y;
            sm.state.isr = request.isr;
            sm.state.isr_shift_count = request.isr_count;
            sm.state.osr = request.osr;
            sm.state.osr_shift_count = request.osr_count;
            sm.state.rx_putget = request.rx;
            sm.enabled = true;
        }
    }

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
            let [padout, padoe] = block.pads();
            let mut row = format!(
                "{tick},{padout:08x},{padoe:08x},{:02x},{flevel:08x},{fdebug:08x}",
                block.irq_flags & 0xff
            );
            for (sm, request) in block.sms.iter().zip(&request.sms) {
                row += &request.map_or(",,,,,,,,,,".into(), |request| {
                    let state = &sm.state;
                    let mut rx = if request.shiftctrl >> 14 & 3 == 0 {
                        [0; 4]
                    } else {
                        state.rx_putget
                    };
                    for (destination, word) in rx.iter_mut().zip(state.rx_fifo.iter()) {
                        *destination = *word;
                    }
                    format!(
                        ",{},{},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x},{:08x}",
                        state.pc.value(),
                        state.rx_fifo.len(),
                        rx[0],
                        rx[1],
                        rx[2],
                        rx[3],
                        state.x,
                        state.y,
                        state.isr,
                        state.osr,
                    )
                });
            }
            row
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
            2 => (1, rng.below(2) << 2 | 2, irq_index(rng)),
            source => (1, rng.below(2) << 2 | source, rng.below(32)),
        },
        2 => (2, rng.below(8), rng.below(32)),
        3 => (3, rng.below(8), rng.below(32)),
        4 => match rng.below(3) {
            0 => (4, rng.below(8), 0x10 | rng.below(16)),
            _ => (4, rng.below(8), rng.below(16)),
        },
        5 => (5, rng.below(8), rng.below(32)),
        6 => (6, rng.below(8), irq_index(rng)),
        _ => (7, rng.below(8), rng.below(32)),
    };
    let delay = if rng.below(2) == 0 { rng.below(32) } else { 0 };
    opcode << 13 | delay << 8 | a << 5 | b
}

fn irq_index(rng: &mut Rng) -> u16 {
    match rng.below(2) {
        0 => rng.below(32),
        _ => rng.below(2) << 4 | rng.below(4),
    }
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
    let (mask, pins, mut code) = (1 + rng.below(15), [4, 32][rng.below(2) as usize], [0; 32]);
    let sms: [Option<Sm>; 4] = std::array::from_fn(|index| (mask >> index & 1 == 1).then(|| random_sm(rng, pins, &mut code)));
    let tx_joined = sms.iter().flatten().any(|sm| sm.shiftctrl & (1 << 30 | 3 << 14) == 1 << 30);
    for word in code.iter_mut().filter(|_| tx_joined) {
        let exec = matches!((*word >> 13, *word >> 5 & 7), (3, 7) | (5, 4));
        if exec || *word >> 13 == 4 && *word & 0x90 == 0x90 {
            *word ^= if exec { 0x80 } else { 0x10 };
        }
    }
    Request { ticks: RANDOM_TICKS, code, irq: rng.below(256) as u8, gpio_in: rng.next() as u32, sms }
}

fn random_sm(rng: &mut Rng, pins: u64, code: &mut [u16; 32]) -> Sm {
    let len = 1 + rng.below(RANDOM_MAX_LEN) as usize;
    let origin = rng.below((32 - len + 1) as u64) as u8;
    let wrap_top = origin as u32 + rng.below(len as u64) as u32;
    let wrap_bottom = origin as u32 + rng.below(wrap_top as u64 - origin as u64 + 1) as u32;
    let mut field = |values: u64, lsb: u32| (rng.below(values) as u32) << lsb;
    let clkdiv = match field(3, 0) {
        0 => 1 << 16,
        _ => (1 + field(3, 0)) << 16 | field(2, 0) * field(256, 8),
    };
    let side_count = if field(16, 0) == 0 { 6 | field(2, 0) } else { field(6, 0) };
    let side_en = if side_count > 0 { field(2, 30) } else { 0 };
    let execctrl = field(32, 0) | field(4, 5) | field(128, 17) | field(32, 24) | wrap_bottom << 7 | wrap_top << 12 | field(2, 29) | side_en;
    let putget = if field(2, 0) == 0 { field(4, 14) } else { 0 };
    let shiftctrl = field(32, 0) | putget | field(16, 16) | field(32, 20) | field(32, 25) | field(4, 30);
    let set_count = if field(16, 0) == 0 { 6 | field(2, 0) } else { field(6, 0) };
    let pinctrl = field(pins, 0) | field(pins, 5) | field(pins, 10) | field(32, 15) | field(64, 20) | set_count << 26 | side_count << 29;
    let mut sm = Sm {
        clkdiv,
        execctrl,
        shiftctrl,
        pinctrl,
        pc: origin,
        tx_len: rng.below(MAX_TX as u64 + 1) as usize,
        tx: [0; MAX_TX],
        x: random_value(rng, origin, len),
        y: random_value(rng, origin, len),
        isr: random_value(rng, origin, len),
        isr_count: rng.below(33) as u8,
        osr: random_value(rng, origin, len),
        osr_count: rng.below(33) as u8,
        rx: std::array::from_fn(|_| rng.next() as u32),
    };
    sm.osr = osr_reachable(sm.osr, sm.osr_count, sm.out_shift_right());
    for word in &mut code[origin as usize..][..len] {
        *word = random_word(rng, origin, len);
    }
    for word in &mut sm.tx[..sm.tx_len] {
        *word = random_value(rng, origin, len);
    }
    sm
}

const SWEEP_STATES: [(u8, u32, Sm); 5] = [
    (
        0,
        0,
        Sm {
            clkdiv: 1 << 16,
            execctrl: 0,
            shiftctrl: 0,
            pinctrl: 0,
            pc: 0,
            tx_len: 0,
            tx: [0; MAX_TX],
            x: 0,
            y: 0,
            isr: 0,
            isr_count: 0,
            osr: 0,
            osr_count: 0,
            rx: [0; 4],
        },
    ),
    (
        0xff,
        0xaaaa_aaaa,
        Sm {
            clkdiv: 1 << 16,
            execctrl: 5 << 24 | 9 << 12 | 9 << 7 | 2,
            shiftctrl: 20 << 25 | 12 << 20 | 1 << 19 | 1 << 18,
            pinctrl: 2 << 29 | 5 << 26 | 8 << 20 | 3 << 15 | 12 << 10 | 8 << 5 | 4,
            pc: 9,
            tx_len: 4,
            tx: [0x0123_4567, 0x89ab_cdef, 0xfedc_ba98, 0x7654_3210],
            x: 0xffff_ffff,
            y: 0x5555_5555,
            isr: 0xa5a5_a5a5,
            isr_count: 16,
            osr: 0x0000_c3c3,
            osr_count: 16,
            rx: [0; 4],
        },
    ),
    (
        0x5a,
        0x0f0f_0f0f,
        Sm {
            clkdiv: 1 << 16,
            execctrl: 1 << 30 | 1 << 29 | 7 << 24 | 6 << 19 | 1 << 18 | 1 << 17 | 20 << 12 | 20 << 7 | 1 << 5 | 1,
            shiftctrl: 24 << 25 | 8 << 20 | 1 << 17 | 1 << 16,
            pinctrl: 3 << 29 | 3 << 26 | 4 << 20 | 16 << 15 | 31 << 10 | 30,
            pc: 20,
            tx_len: 1,
            tx: [0xdead_beef, 0, 0, 0],
            x: 0x8000_0001,
            y: 1,
            isr: 0x1234_5678,
            isr_count: 7,
            osr: 0xdea0_0000,
            osr_count: 20,
            rx: [0; 4],
        },
    ),
    (
        0x0f,
        0xffff_ffff,
        Sm {
            clkdiv: 1 << 16,
            execctrl: 29 << 12 | 29 << 7 | 2 << 5 | 3,
            shiftctrl: 1 << 18 | 1 << 15 | 1 << 14,
            pinctrl: 5 << 29 | 5 << 26 | 32 << 20 | 2 << 10 | 26 << 5,
            pc: 29,
            tx_len: 2,
            tx: [0xffff_0000, 0x0000_ffff, 0, 0],
            x: 1,
            y: 0xffff_fffe,
            isr: 0xcafe_f00d,
            isr_count: 32,
            osr: 0x1357_9bdf,
            osr_count: 0,
            rx: [0x1111_1111, 0x2222_2222, 0x4444_4444, 0x8888_8888],
        },
    ),
    (
        0x80,
        0x5555_5555,
        Sm {
            clkdiv: 1 << 16,
            execctrl: 1 << 30 | 31 << 24 | 15 << 12 | 15 << 7 | 4,
            shiftctrl: 1 << 30 | 1 << 19 | 1 << 18 | 1 << 17 | 1 << 16,
            pinctrl: 1 << 29 | 1 << 26 | 1 << 20 | 1 << 15 | 1 << 10 | 1 << 5 | 1,
            pc: 15,
            tx_len: 3,
            tx: [0x8000_0000, 0xffff_ffff, 1, 0],
            x: 0x7fff_ffff,
            y: 0x8000_0000,
            isr: 0x7fff_fffe,
            isr_count: 31,
            osr: 0,
            osr_count: 32,
            rx: [0; 4],
        },
    ),
];

fn sweep(state: usize) -> Vec<(String, Request)> {
    let (irq, gpio_in, sm) = SWEEP_STATES[state];
    (0..=u16::MAX)
        .map(|word| {
            let mut code = [0; 32];
            code[sm.pc as usize] = word;
            (
                format!("word {word:04x}"),
                Request { ticks: (word >> 8 & 31) as u32 + 2, code, irq, gpio_in, sms: [Some(sm), None, None, None] },
            )
        })
        .collect()
}

fn disassemble(request: &Request) -> String {
    let mut text = format!("  irq {:02x}, gpio_in {:08x}\n", request.irq, request.gpio_in);
    for (index, sm) in request.sms.iter().enumerate().filter_map(|(index, sm)| Some((index, sm.as_ref()?))) {
        text += &format!(
            "  sm{index}: clkdiv {:08x}, execctrl {:08x}, shiftctrl {:08x}, pinctrl {:08x}, pc {}, tx {:08x?}\n    x {:08x}, y {:08x}, isr {:08x} ({}), osr {:08x} ({}), rx {:08x?}\n",
            sm.clkdiv,
            sm.execctrl,
            sm.shiftctrl,
            sm.pinctrl,
            sm.pc,
            &sm.tx[..sm.tx_len],
            sm.x,
            sm.y,
            sm.isr,
            sm.isr_count,
            sm.osr,
            sm.osr_count,
            sm.rx
        );
    }
    let configs: Vec<Config> = request
        .sms
        .iter()
        .flatten()
        .map(|sm| Config::from_registers(sm.clkdiv, sm.execctrl, sm.shiftctrl, sm.pinctrl))
        .collect();
    for (slot, word) in request.code.iter().enumerate() {
        let decoded = configs.iter().map(|config| Instr::decode(*word, config, false).to_string());
        text += &format!("  {slot:2}: {word:04x}  {}\n", decoded.collect::<Vec<_>>().join(" | "));
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn emulated_trace() {
        let sm = Sm {
            clkdiv: 1 << 16,
            execctrl: 1 << 30 | 6 << 12 | 4 << 7,
            shiftctrl: 0x000c_0000,
            pinctrl: 2 << 29 | 5 << 26,
            pc: 4,
            tx_len: 0,
            tx: [0; MAX_TX],
            x: 0,
            y: 0,
            isr: 0,
            isr_count: 0,
            osr: 0,
            osr_count: 32,
            rx: [0; 4],
        };
        let mut request =
            Request { ticks: 16, code: [0; 32], irq: 0, gpio_in: 0, sms: [Some(sm), None, Some(Sm { clkdiv: 0x0002_8000, ..sm }), None] };
        request.code[4..7].copy_from_slice(&[0xf823, 0x0245, 0x80a0]);
        let rows = emulate(&request, true);
        let field = |tick: usize, column: &str| {
            let index = CSV_HEADER.split(',').position(|c| c == column).unwrap();
            rows[tick - 1].split(',').nth(index).unwrap().to_owned()
        };
        assert_eq!(
            (field(1, "sm0_pc"), field(1, "sm0_x"), field(1, "sm1_pc")),
            ("5".into(), "00000003".into(), "".into())
        );
        assert_eq!((field(10, "sm0_pc"), field(10, "sm0_x")), ("5".into(), "00000000".into()));
        assert_eq!((field(11, "sm0_pc"), field(11, "sm0_x")), ("6".into(), "ffffffff".into()));
        assert_eq!(field(16, "sm0_pc"), "6");
        let sm2_x: Vec<String> = (1..=16).map(|tick| field(tick, "sm2_x")).collect();
        assert_eq!(sm2_x.iter().map(|x| &x[7..]).collect::<String>(), "3322222221111111");
    }
    #[test]
    fn cases_match_fixtures() {
        let failures: Vec<String> = cases::cases()
            .into_iter()
            .filter_map(|(name, _, _, test)| match test {
                cases::Test::Pass(request) => {
                    let fixture = std::fs::read_to_string(format!("{}/cases/{name}.csv", env!("CARGO_MANIFEST_DIR"))).unwrap();
                    let mut wire = String::new();
                    request.write(&mut wire).unwrap();
                    let lines: Vec<String> = fixture.lines().map(str::to_owned).collect();
                    assert_eq!(lines[..2], [wire.trim_end(), CSV_HEADER], "{name}: re-run record-cases");
                    first_difference(&lines[2..], &emulate(&request, false)).map(|difference| format!("{name}: {difference}"))
                }
                cases::Test::Untestable(_) => None,
            })
            .collect();
        assert!(failures.is_empty(), "{}", failures.join("\n"));
    }
    #[test]
    fn strict_cases_panic_with_their_name() {
        for (name, _, kind, test) in cases::cases() {
            if let cases::Test::Pass(request) = test {
                let panic = std::panic::catch_unwind(|| emulate(&request, true)).err();
                let expected = match kind {
                    cases::Kind::Contradicts(_) => None,
                    cases::Kind::Undefined(_) | cases::Kind::Undocumented => Some(format!("undefined: {name}")),
                };
                assert_eq!(panic.as_deref().map(panic_message), expected.as_deref(), "{name}");
            }
        }
    }
    #[test]
    fn undefined_names_are_cases() {
        let names: Vec<&str> = cases::cases().into_iter().map(|(name, ..)| name).collect();
        for source in [include_str!("../../pioemu/src/state.rs"), include_str!("../../pioemu/src/instr.rs")] {
            for call in source.split("assert_strict(").skip(1) {
                let name = call.split('"').nth(1).unwrap();
                assert!(names.contains(&name), "undefined: {name} is not in the case table");
            }
        }
    }
    #[test]
    fn quotes_are_in_datasheet() {
        let datasheet = include_str!("../../datasheet/rp2350.txt")
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" ");
        for (name, _, kind, _) in cases::cases() {
            if let cases::Kind::Undefined(quote) | cases::Kind::Contradicts(quote) = kind {
                let quote = quote.split_whitespace().collect::<Vec<_>>().join(" ");
                assert!(datasheet.contains(&quote), "{name}: {quote}");
            }
        }
    }
    #[test]
    fn sections_are_in_datasheet() {
        let datasheet = include_str!("../../datasheet/rp2350.txt");
        for (name, section, ..) in cases::cases() {
            assert!(
                datasheet.lines().any(|line| line.starts_with(&format!("{section}. "))),
                "{name}: {section}"
            );
        }
    }
    #[test]
    fn random_requests_are_valid() {
        for seed in 0..10_000 {
            let request = random_request(&mut Rng(seed));
            request.validate().unwrap();
            disassemble(&request);
        }
    }
    #[test]
    fn sweep_requests_are_valid() {
        for state in 0..SWEEP_STATES.len() {
            for (_, request) in sweep(state) {
                let mut wire = String::new();
                request.write(&mut wire).unwrap();
                assert_eq!(Request::parse(wire.trim_end()).as_ref(), Ok(&request));
                emulate(&request, false);
            }
        }
    }
}
