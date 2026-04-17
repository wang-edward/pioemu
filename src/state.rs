use crate::pio::{Condition, Instr, Instruction, set};
use arbitrary_int::u5;
use std::collections::VecDeque;
use std::fmt;

#[derive(Clone, Copy, Debug)]
struct Range<const MIN: u8, const MAX: u8>(u8);

impl<const MIN: u8, const MAX: u8> Range<MIN, MAX> {
    fn new(val: u8) -> Self {
        assert!(val >= MIN && val <= MAX, "value {val} not in [{MIN}, {MAX}]");
        Self(val)
    }

    fn get(self) -> u8 {
        self.0
    }
}
type PinRange = Range<0, 31>;

const FIFO_DEPTH: usize = 4;
#[derive(Debug)]
struct Fifo {
    data: VecDeque<u32>,
    depth: usize, // 4 normally, 8 if joined
}

impl Fifo {
    fn new(depth: usize) -> Self {
        Self { data: VecDeque::with_capacity(depth), depth }
    }
    fn is_full(&self) -> bool {
        self.data.len() >= self.depth
    }
    fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
    fn push(&mut self, val: u32) -> bool {
        if self.is_full() {
            return false;
        }
        self.data.push_back(val);
        true
    }
    fn pop(&mut self) -> Option<u32> {
        self.data.pop_front()
    }
}

impl fmt::Display for Fifo {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        if self.data.is_empty() {
            write!(f, "[empty]")
        } else {
            write!(f, "[")?;
            for (i, x) in self.data.iter().enumerate() {
                if i > 0 {
                    write!(f, ", ")?;
                }
                write!(f, "{:08x}", x)?;
            }
            write!(f, "] ({}/{})", self.data.len(), self.depth)
        }
    }
}

#[derive(Debug)]
pub struct Block {
    pub instr_mem: [Option<Instr>; 32],
    pub state_machines: [StateMachine; 4],
    pub gpio_out: u32,
    pub gpio_dir: u32,
    pub gpio_in: u32,
    cycle: u64,
}

impl fmt::Display for Block {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(f, "=== PIO Block (cycle {}) ===", self.cycle)?;
        writeln!(
            f,
            "gpio: out={:08x} dir={:08x} in={:08x}",
            self.gpio_out, self.gpio_dir, self.gpio_in
        )?;

        // // print loaded program
        // writeln!(f, "program:")?;
        // for (i, slot) in self.instr_mem.iter().enumerate() {
        //     if let Some(instr) = slot {
        //         writeln!(f, "  {:02}: {}", i, instr)?;
        //     }
        // }

        // print state machines
        for (i, sm) in self.state_machines.iter().enumerate() {
            writeln!(f, "sm{}: {}", i, sm)?;
        }
        Ok(())
    }
}

impl Block {
    pub fn new() -> Self {
        Self {
            instr_mem: std::array::from_fn(|_| None),
            state_machines: std::array::from_fn(|_| StateMachine { state: State::new(), config: Config::new(), enabled: false }),
            gpio_out: 0,
            gpio_dir: 0,
            gpio_in: 0,
            cycle: 0,
        }
    }
    pub fn step(&mut self) {
        let Block { state_machines, instr_mem, gpio_out, gpio_dir, gpio_in, cycle } = self;
        for (i, sm) in state_machines.iter_mut().enumerate() {
            if !sm.enabled {
                continue;
            }
            let pc = sm.state.pc.value() as usize;
            let instr = instr_mem[pc].expect("no instruction at PC");

            // if sm.state.delay_counter > 0 {
            //     println!(
            //         "[{:>5}] sm{} @{:02}  {:<26} (delay {}) | x={:08x} y={:08x}",
            //         cycle, i, pc, instr, sm.state.delay_counter, sm.state.x, sm.state.y,
            //     );
            // } else {
            //     println!(
            //         "[{:>5}] sm{} @{:02}  {:<26}           | x={:08x} y={:08x}",
            //         cycle, i, pc, instr, sm.state.x, sm.state.y,
            //     );
            // }

            sm.execute(&instr, gpio_out, gpio_dir, *gpio_in);
        }
        *cycle += 1;
    }
}

#[derive(Debug)]
pub struct StateMachine {
    pub state: State,
    pub config: Config,
    pub enabled: bool,
}

impl fmt::Display for StateMachine {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        if self.enabled {
            write!(f, "[on]  {}", self.state)
        } else {
            write!(f, "[off]")
        }
    }
}

fn to_mask(val: u8) -> u32 {
    if val >= 32 { u32::MAX } else { (1u32 << val) - 1 }
}

impl StateMachine {
    fn execute(&mut self, instr: &Instr, gpio_out: &mut u32, gpio_dir: &mut u32, gpio_in: u32) {
        if self.state.delay_counter > 0 {
            self.state.delay_counter -= 1;
            return;
        }
        let mut advance_pc = true;

        match instr.instruction {
            Instruction::Jmp { condition, address } => {
                let jump = match condition {
                    Condition::Always => true,
                    Condition::XZero => self.state.x == 0,
                    Condition::XDec => {
                        let result = self.state.x == 0;
                        self.state.x -= 1;
                        result
                    }
                    Condition::YZero => self.state.y == 0,
                    Condition::YDec => {
                        let result = self.state.y == 0;
                        self.state.y -= 1;
                        result
                    }
                    Condition::XNeqY => self.state.x != self.state.y,
                    Condition::Pin => (gpio_in >> self.config.jmp_pin.value()) & 1 == 1,
                    Condition::OsrNotEmpty => self.state.osr_shift_count < self.config.pull_thresh.get(), // TODO
                };
                if jump {
                    self.state.pc = address;
                    advance_pc = false;
                }
            }
            Instruction::Set { destn, data } => match destn {
                set::Destn::Pins => {
                    let (base, cnt) = (self.config.set_base.get() as u32, self.config.set_count.get());
                    let data = data.value() as u32;
                    let mask = if cnt == 32 { u32::MAX } else { to_mask(cnt) << base };
                    *gpio_out = (*gpio_out & !mask) | ((data << base) & mask);
                }
                set::Destn::X => {
                    self.state.x = data.value() as u32;
                }
                set::Destn::Y => {
                    self.state.y = data.value() as u32;
                }
                set::Destn::PinDirs => {
                    let (base, cnt) = (self.config.set_base.get() as u32, self.config.set_count.get());
                    let data = data.value() as u32;
                    let mask = if cnt == 32 { u32::MAX } else { to_mask(cnt) << base };
                    *gpio_dir = (*gpio_dir & !mask) | ((data << base) & mask);
                }
                _ => panic!(),
            },
            _ => panic!(),
        }

        self.state.delay_counter = instr.delay.value();
        if advance_pc {
            if self.state.pc == self.config.wrap_top {
                self.state.pc = self.config.wrap_bottom;
            } else {
                self.state.pc = u5::new(self.state.pc.value() + 1);
            }
        }
    }
}

#[derive(Debug)]
struct State {
    pc: u5,
    // no clock divider, don't think about timing rn
    x: u32,
    y: u32,
    isr: u32,
    osr: u32,
    isr_shift_count: u8,
    osr_shift_count: u8,
    tx_fifo: Fifo,
    rx_fifo: Fifo,
    delay_counter: u8,
    stalled: bool,
}

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(
            f,
            "pc={:02} x={:08x} y={:08x} osr={:08x}({}) isr={:08x}({})",
            self.pc.value(),
            self.x,
            self.y,
            self.osr,
            self.osr_shift_count,
            self.isr,
            self.isr_shift_count,
        )?;
        write!(f, "    tx: {}  rx: {}", self.tx_fifo, self.rx_fifo)
    }
}

impl State {
    fn new() -> Self {
        Self {
            pc: u5::new(0),
            x: 0,
            y: 0,
            osr: 0,
            isr: 0,
            osr_shift_count: 32, // empty at reset
            isr_shift_count: 0,
            tx_fifo: Fifo::new(FIFO_DEPTH),
            rx_fifo: Fifo::new(FIFO_DEPTH),
            delay_counter: 0,
            stalled: false,
        }
    }
}

#[derive(Debug)]
enum ShiftDir {
    Left,
    Right,
}

#[derive(Debug)]
enum StatusSel {
    TxLevel,
    RxLevel,
}

#[derive(Debug)]
struct Config {
    // pinctrl
    out_base: PinRange,
    out_count: Range<0, 32>,
    set_base: PinRange,
    set_count: Range<0, 5>,
    in_base: PinRange,
    sideset_base: PinRange,
    sideset_count: Range<0, 5>,

    // execctrl
    sideset_en: bool,
    side_pindir: bool,
    jmp_pin: u5,
    // out_en_sel: u5,
    wrap_top: u5,
    wrap_bottom: u5,
    status_sel: StatusSel,
    status_n: Range<0, 15>,

    // shiftctrl
    pull_thresh: Range<0, 31>,
    push_thresh: Range<0, 31>,
    out_shiftdir: ShiftDir,
    in_shiftdir: ShiftDir,
    autopull: bool,
    autopush: bool,
    fjoin_rx: bool,
    fjoin_tx: bool,
    // TODO clkdiv?
    // clkdiv_int: u16,
    // clkdiv_frac: u8,
}

impl fmt::Display for ShiftDir {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            ShiftDir::Left => write!(f, "L"),
            ShiftDir::Right => write!(f, "R"),
        }
    }
}

impl fmt::Display for Config {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        writeln!(
            f,
            "  wrap={}..{} jmp_pin={}",
            self.wrap_bottom.value(),
            self.wrap_top.value(),
            self.jmp_pin.value(),
        )?;
        writeln!(
            f,
            "  pins: out={}+{} set={}+{} in={} sideset={}+{}",
            self.out_base.get(),
            self.out_count.get(),
            self.set_base.get(),
            self.set_count.get(),
            self.in_base.get(),
            self.sideset_base.get(),
            self.sideset_count.get(),
        )?;
        write!(
            f,
            "  shift: out={} in={} autopull={} autopush={} pull_thresh={} push_thresh={}",
            self.out_shiftdir,
            self.in_shiftdir,
            if self.autopull { "on" } else { "off" },
            if self.autopush { "on" } else { "off" },
            self.pull_thresh.get(),
            self.push_thresh.get(),
        )
    }
}

impl Config {
    // TODO pull_thresh override 32 = 0 and fifo_depth if fjoin
    fn new() -> Self {
        Self {
            out_base: PinRange::new(0),
            out_count: Range::new(0),
            set_base: PinRange::new(0),
            set_count: Range::new(5),
            in_base: PinRange::new(0),
            sideset_base: PinRange::new(0),
            sideset_count: Range::new(0),
            sideset_en: false,
            side_pindir: false,
            jmp_pin: u5::new(0),
            wrap_top: u5::new(0x1f),
            wrap_bottom: u5::new(0),
            status_sel: StatusSel::TxLevel,
            status_n: Range::new(0),
            pull_thresh: Range::new(0),
            push_thresh: Range::new(0),
            out_shiftdir: ShiftDir::Right,
            in_shiftdir: ShiftDir::Right,
            autopull: false,
            autopush: false,
            fjoin_rx: false,
            fjoin_tx: false,
        }
    }
}
