use crate::instr::{Condition, Instr, Instruction, set, shift, wait};
use arbitrary_int::u5;
use std::cmp;
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
pub struct Fifo {
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
    pub sms: [StateMachine; 4],
    pub gpio_out: u32,
    pub gpio_dir: u32,
    pub gpio_in: u32,
    irq_flags: u8,
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

        // print state machines
        for (i, sm) in self.sms.iter().enumerate() {
            writeln!(f, "sm{}: {}", i, sm)?;
        }
        Ok(())
    }
}

impl Block {
    pub fn new() -> Self {
        Self {
            instr_mem: std::array::from_fn(|_| None),
            sms: std::array::from_fn(|_| StateMachine { state: State::new(), config: Config::new(), enabled: false }),
            gpio_out: 0,
            gpio_dir: 0,
            gpio_in: 0,
            irq_flags: 0,
            cycle: 0,
        }
    }
    pub fn step(&mut self) {
        let Block { sms, instr_mem, gpio_out, gpio_dir, gpio_in, irq_flags, cycle } = self;
        for (i, sm) in sms.iter_mut().enumerate() {
            if !sm.enabled {
                continue;
            }
            let pc = sm.state.pc.value() as usize;
            let instr = instr_mem[pc].expect("no instruction at PC");

            sm.execute(&instr, gpio_out, gpio_dir, *gpio_in, irq_flags, i as u8);
        }
        *cycle += 1;
    }
    pub fn print_instr_mem(&self) {
        println!("program:");
        for (i, slot) in self.instr_mem.iter().enumerate() {
            if let Some(instr) = slot {
                println!("  {:02}: {}", i, instr);
            }
        }
    }
    pub fn print(&self) {
        println!("{}", self);
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

pub fn to_mask(val: u8) -> u32 {
    assert!(val <= 32);
    if val == 32 { u32::MAX } else { (1u32 << val) - 1 }
}

pub fn wrap_shiftr(x: u32, shift: u8) -> u32 {
    let lift = (x & to_mask(shift)) << (32 - shift);
    return (x >> shift) | lift;
}

fn calc_irq_index(index: u8, sm_id: u8) -> u8 {
    if index & 0x10 != 0 {
        (index & 0x04) | ((index + sm_id) & 0x03)
    } else {
        index & 0x07
    }
}

impl StateMachine {
    fn execute(&mut self, instr: &Instr, gpio_out: &mut u32, gpio_dir: &mut u32, gpio_in: u32, irq_flags: &mut u8, sm_id: u8) {
        if self.state.delay_counter > 0 && !self.state.stalled {
            self.state.delay_counter -= 1;
            return;
        }

        let mut advance_pc = true;
        self.state.stalled = false;

        match instr.instruction {
            Instruction::Jmp { condition, address } => {
                let jump = match condition {
                    Condition::Always => true,
                    Condition::XZero => self.state.x == 0,
                    Condition::XDec => {
                        let result = self.state.x != 0;
                        self.state.x = self.state.x.wrapping_sub(1);
                        result
                    }
                    Condition::YZero => self.state.y == 0,
                    Condition::YDec => {
                        let result = self.state.y != 0;
                        self.state.y = self.state.y.wrapping_sub(1);
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
            Instruction::Wait { polarity, source, index } => {
                let (polarity, index) = (polarity.value() as u32, index.value() as u32);
                let irq_index = calc_irq_index(index as u8, sm_id);
                let cond_met = match source {
                    wait::Source::Gpio => (gpio_in >> index) & 1,
                    wait::Source::Pin => (wrap_shiftr(gpio_in, self.config.in_base.get()) >> index) & 1,
                    wait::Source::Irq => {
                        assert!(index <= 7);
                        (*irq_flags as u32 >> irq_index) & 1
                    }
                    _ => panic!(),
                } == polarity; // convert to bool / negate
                if cond_met {
                    if matches!(source, wait::Source::Irq) && polarity == 1 {
                        *irq_flags &= !(1 << irq_index); // If Polarity is 1, the selected IRQ flag is cleared by the state machine upon the wait condition being met.
                    }
                } else {
                    self.state.stalled = true;
                    return;
                }
            }
            Instruction::In { source, bit_count } => {
                let bit_count = if bit_count.value() == 0 { 32 } else { bit_count.value() };
                let data = match source {
                    shift::Source::Pins => wrap_shiftr(gpio_in, self.config.in_base.get()),
                    shift::Source::X => self.state.x,
                    shift::Source::Y => self.state.y,
                    shift::Source::Null => 0,
                    shift::Source::Isr => self.state.isr,
                    shift::Source::Osr => self.state.osr,
                    _ => panic!(),
                } & to_mask(bit_count);
                match self.config.in_shiftdir {
                    ShiftDir::Left => self.state.isr = (self.state.isr << bit_count) | data,
                    ShiftDir::Right => self.state.isr = (self.state.isr >> bit_count) | wrap_shiftr(data, bit_count),
                }
                self.state.isr_shift_count = self.state.isr_shift_count.saturating_add(bit_count);
                // TODO handle autopush: If automatic push is enabled, IN will also push the ISR contents to the RX FIFO if the push threshold is reached (SHIFTCTRL_PUSH_THRESH). IN still executes in one cycle, whether an automatic push takes place or not. The state machine will stall if the RX FIFO is full when an automatic push occurs. An automatic push clears the ISR contents to all-zeroes, and clears the input shift count. See Section 3.5.4 }
                if self.config.autopush && self.state.isr_shift_count >= self.config.calc_push_thresh() {
                    if self.state.rx_fifo.is_full() {
                        self.state.stalled = true;
                        return;
                    }
                    self.state.rx_fifo.push(self.state.isr);
                    self.state.isr = 0;
                    self.state.isr_shift_count = 0;
                }
            }
            Instruction::Out { destn, bit_count } => {
                let bit_count = if bit_count.value() == 0 { 32 } else { bit_count.value() };
                if self.config.autopull && self.state.osr_shift_count >= self.config.pull_thresh.get() {
                    if self.state.tx_fifo.is_empty() {
                        self.state.stalled = true;
                        return;
                    }
                    // todo ? here?
                    self.state.osr = self.state.tx_fifo.pop().expect("tx fifo empty when it shouldn't be");
                    self.state.osr_shift_count = 0;
                }

                let data = match self.config.out_shiftdir {
                    ShiftDir::Left => {
                        let ans = (self.state.osr >> (32 - bit_count)) & to_mask(bit_count);
                        self.state.osr = self.state.osr << bit_count;
                        ans
                    }
                    ShiftDir::Right => {
                        let ans = self.state.osr & to_mask(bit_count);
                        self.state.osr = self.state.osr >> bit_count;
                        ans
                    }
                };
                match destn {
                    shift::Destn::Pins => {
                        let (out_count, out_base) = (self.config.out_count.get(), self.config.out_base.get());
                        let mask = to_mask(out_count) << out_base;
                        *gpio_out = (*gpio_out & !mask) | ((data << out_base) & mask);
                    }
                    shift::Destn::X => self.state.x = data,
                    shift::Destn::Y => self.state.y = data,
                    shift::Destn::Null => (),
                    shift::Destn::PinDirs => {
                        let (out_count, out_base) = (self.config.out_count.get(), self.config.out_base.get());
                        let mask = to_mask(out_count) << out_base;
                        *gpio_dir = (*gpio_dir & !mask) | ((data << out_base) & mask);
                    }
                    shift::Destn::Pc => {
                        self.state.pc = u5::new(data as u8);
                        advance_pc = false;
                    }
                    shift::Destn::Isr => {
                        self.state.isr = data;
                        self.state.isr_shift_count = bit_count;
                    }
                    shift::Destn::Exec => panic!(),
                    // todo EXEC destn. i think we need to have bit decoding for this to work
                }

                self.state.osr_shift_count = cmp::min(32, self.state.osr_shift_count + bit_count);
                if self.config.autopull && self.state.osr_shift_count >= self.config.calc_pull_thresh() {
                    if !self.state.tx_fifo.is_empty() {
                        self.state.osr = self.state.tx_fifo.pop().expect("tx fifo empty when it shouldn't be");
                        self.state.osr_shift_count = 0;
                    }
                }
            }
            Instruction::Push { if_full, block } => {
                let (if_full, block) = (if_full.value() == 1, block.value() == 1);
                let should_push = !if_full | (self.state.isr_shift_count >= self.config.calc_push_thresh());
                if should_push {
                    if self.state.rx_fifo.is_full() {
                        if block {
                            self.state.stalled = true;
                            return;
                        }
                    } else {
                        self.state.rx_fifo.push(self.state.isr);
                    }
                    self.state.isr = 0;
                    self.state.isr_shift_count = 0;
                }
            }
            Instruction::Pull { if_empty, block } => {
                // TODO autopull noop and stuff
                // TODO program checker (don't use mov dst, osr when autopull on and more)
                let (if_empty, block) = (if_empty.value() == 1, block.value() == 1);
                let should_pull = !if_empty | (self.state.osr_shift_count >= self.config.calc_pull_thresh());
                if should_pull {
                    if self.state.tx_fifo.is_empty() {
                        if block {
                            self.state.stalled = true;
                            return;
                        } else {
                            self.state.osr = self.state.x;
                        }
                    } else {
                        self.state.osr = self.state.tx_fifo.pop().expect("tx fifo empty when it shouldn't be");
                    }
                    self.state.osr_shift_count = 0;
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
        // TODO side set
        // should be something like switch self.config.sideset_count == 5 vs 0
        // also side_en controls whether to use MSB as enable
        // If an instruction stalls, the side-set still takes effect immediately.
        // - so maybe put this at the top
        // and takes priority over OUT writing to the same pin
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
pub struct State {
    pub pc: u5,
    // no clock divider, don't think about timing rn
    pub x: u32,
    pub y: u32,
    pub isr: u32,
    pub osr: u32,
    pub isr_shift_count: u8,
    pub osr_shift_count: u8,
    pub tx_fifo: Fifo,
    pub rx_fifo: Fifo,
    pub delay_counter: u8,
    pub stalled: bool,
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
pub struct Config {
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
    fn calc_pull_thresh(&self) -> u8 {
        if self.pull_thresh.get() == 0 { 32 } else { self.pull_thresh.get() }
    }
    fn calc_push_thresh(&self) -> u8 {
        if self.push_thresh.get() == 0 { 32 } else { self.push_thresh.get() }
    }
}
