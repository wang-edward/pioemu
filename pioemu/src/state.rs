use crate::instr::{Condition, Instr, Instruction, mov, set, shift, wait};
use arbitrary_int::u5;
use std::cmp;
use std::collections::VecDeque;

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
    pub fn push(&mut self, val: u32) -> bool {
        if self.is_full() {
            return false;
        }
        self.data.push_back(val);
        true
    }
    fn pop(&mut self) -> Option<u32> {
        self.data.pop_front()
    }
    pub fn len(&self) -> usize {
        self.data.len()
    }
    pub fn iter(&self) -> impl Iterator<Item = &u32> {
        self.data.iter()
    }
}

#[derive(Debug)]
pub struct Block {
    pub instr_mem: [Option<Instr>; 32],
    pub sms: [StateMachine; 4],
    pub gpio_out: u32,
    pub gpio_dir: u32,
    pub gpio_in: u32,
    pub irq_flags: u8,
    cycle: u64,
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
}

#[derive(Debug)]
pub struct StateMachine {
    pub state: State,
    pub config: Config,
    pub enabled: bool,
}

pub fn to_mask(val: u8) -> u32 {
    assert!(val <= 32);
    if val == 32 { u32::MAX } else { (1u32 << val) - 1 }
}

fn write_pins(pins: &mut u32, data: u32, base: u8, count: u8) {
    let mask = to_mask(count).rotate_left(base as u32);
    *pins = (*pins & !mask) | (data.rotate_left(base as u32) & mask);
}

pub fn wrap_shiftr(x: u32, shift: u8) -> u32 {
    x.rotate_right(shift as u32)
}

pub fn reverse(x: u32) -> u32 {
    // TODO gotta be a builtin way
    let mut x = x;
    let mut ans: u32 = 0;
    for _ in 0..32 {
        ans = ans << 1 | (x & 0x1);
        x = x >> 1;
    }
    ans
}

pub fn bit_at(x: u8, index: u8) -> bool {
    ((x >> index) & 0x1) == 1
}

pub fn calc_irq_index(index: u8, sm_id: u8) -> u8 {
    if index & 0x10 != 0 {
        (index & 0x04) | ((index + sm_id) & 0x03)
    } else {
        index & 0x07
    }
}

pub fn sat_shl(x: u32, n: u8) -> u32 {
    assert!(n <= 32);
    ((x as u64) << n) as u32
}

pub fn sat_shr(x: u32, n: u8) -> u32 {
    assert!(n <= 32);
    ((x as u64) >> n) as u32
}

impl StateMachine {
    // TODO function for gpio in / out mapping
    fn execute(&mut self, instr: &Instr, gpio_out: &mut u32, gpio_dir: &mut u32, gpio_in: u32, irq_flags: &mut u8, sm_id: u8) {
        let threshold = self.config.calc_pull_thresh();
        let was_empty = self.state.osr_shift_count >= threshold;
        match (self.state.irq_wait, self.state.delay_counter > 0 && !self.state.stalled) {
            (Some(irq_index), _) => {
                if !bit_at(*irq_flags, irq_index) {
                    self.state.irq_wait = None;
                    self.state.stalled = false;
                }
            }
            (None, true) => self.state.delay_counter -= 1,
            (None, false) => {
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
                            Condition::OsrNotEmpty => self.state.osr_shift_count < self.config.calc_pull_thresh(), // TODO
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
                        let isr = match self.config.in_shiftdir {
                            ShiftDir::Left => sat_shl(self.state.isr, bit_count) | data,
                            ShiftDir::Right => sat_shr(self.state.isr, bit_count) | wrap_shiftr(data, bit_count),
                        };
                        let isr_shift_count = cmp::min(63, self.state.isr_shift_count + bit_count);
                        if self.config.autopush && isr_shift_count >= self.config.calc_push_thresh() {
                            if self.state.rx_fifo.push(isr) {
                                self.state.isr = 0;
                                self.state.isr_shift_count = 0;
                            } else {
                                self.state.stalled = true;
                            }
                        } else {
                            self.state.isr = isr;
                            self.state.isr_shift_count = isr_shift_count;
                        }
                    }
                    Instruction::Out { destn, bit_count } => {
                        let bit_count = if bit_count.value() == 0 { 32 } else { bit_count.value() };
                        if self.config.autopull && self.state.osr_shift_count >= self.config.calc_pull_thresh() {
                            self.state.stalled = true;
                        } else {
                            let data = match self.config.out_shiftdir {
                                ShiftDir::Left => {
                                    let ans = sat_shr(self.state.osr, 32 - bit_count) & to_mask(bit_count);
                                    self.state.osr = sat_shl(self.state.osr, bit_count);
                                    ans
                                }
                                ShiftDir::Right => {
                                    let ans = self.state.osr & to_mask(bit_count);
                                    self.state.osr = sat_shr(self.state.osr, bit_count);
                                    ans
                                }
                            };
                            match destn {
                                shift::Destn::Pins => {
                                    write_pins(gpio_out, data, self.config.out_base.get(), self.config.out_count.get());
                                }
                                shift::Destn::X => self.state.x = data,
                                shift::Destn::Y => self.state.y = data,
                                shift::Destn::Null => (),
                                shift::Destn::PinDirs => {
                                    write_pins(gpio_dir, data, self.config.out_base.get(), self.config.out_count.get());
                                }
                                shift::Destn::Pc => {
                                    self.state.pc = u5::new(data as u8);
                                    advance_pc = false;
                                }
                                shift::Destn::Isr => {
                                    self.state.isr = data;
                                    self.state.isr_shift_count = bit_count;
                                }
                                shift::Destn::Exec => panic!(), // todo EXEC destn. i think we need to have bit decoding for this to work
                            }

                            self.state.osr_shift_count = cmp::min(32, self.state.osr_shift_count + bit_count);
                        }
                    }
                    Instruction::Push { if_full, block } => {
                        let (if_full, block) = (if_full.value() == 1, block.value() == 1);
                        let should_push = !if_full || (self.state.isr_shift_count >= self.config.calc_push_thresh());
                        if should_push && block && self.state.rx_fifo.is_full() {
                            self.state.stalled = true;
                        } else if should_push {
                            self.state.rx_fifo.push(self.state.isr);
                            self.state.isr = 0;
                            self.state.isr_shift_count = 0;
                        }
                    }
                    Instruction::Pull { if_empty, block } => {
                        // TODO program checker (don't use mov dst, osr when autopull on and more)
                        let (if_empty, block) = (if_empty.value() == 1, block.value() == 1);
                        let should_pull = !(self.config.autopull && self.state.osr_shift_count == 0)
                            && (!if_empty || self.state.osr_shift_count >= self.config.calc_pull_thresh());
                        if should_pull {
                            match (self.state.tx_fifo.pop(), block) {
                                (Some(word), _) => {
                                    self.state.osr = word;
                                    self.state.osr_shift_count = 0;
                                }
                                (None, true) => self.state.stalled = true,
                                (None, false) => {
                                    self.state.osr = self.state.x;
                                    self.state.osr_shift_count = 0;
                                }
                            }
                        }
                    }
                    Instruction::Mov { destn, op, source } => {
                        let data = match source {
                            mov::Source::Pins => wrap_shiftr(gpio_in, self.config.in_base.get()),
                            mov::Source::X => self.state.x,
                            mov::Source::Y => self.state.y,
                            mov::Source::Null => 0,
                            mov::Source::Status => {
                                let level = match self.config.status_sel {
                                    StatusSel::TxLevel => self.state.tx_fifo.len(),
                                    StatusSel::RxLevel => self.state.rx_fifo.len(),
                                };
                                if level < self.config.status_n.get() as usize {
                                    0xffff_ffff
                                } else {
                                    0x0
                                }
                            }
                            mov::Source::Isr => self.state.isr,
                            mov::Source::Osr => self.state.osr,
                            _ => panic!(),
                        };
                        let data = match op {
                            mov::Op::None => data,
                            mov::Op::Invert => !data,
                            mov::Op::BitReverse => reverse(data),
                            _ => panic!(),
                        };
                        match destn {
                            mov::Destn::Pins => {
                                write_pins(gpio_out, data, self.config.out_base.get(), self.config.out_count.get());
                            }
                            mov::Destn::X => self.state.x = data,
                            mov::Destn::Y => self.state.y = data,
                            mov::Destn::Exec => panic!(), // TODO exec
                            mov::Destn::Pc => self.state.pc = u5::new(data as u8),
                            mov::Destn::Isr => {
                                self.state.isr = data;
                                self.state.isr_shift_count = 0;
                            }
                            mov::Destn::Osr => {
                                self.state.osr = data;
                                self.state.osr_shift_count = 0;
                            }
                            _ => panic!(),
                        }
                    }
                    Instruction::Irq { clear, wait, index } => {
                        let irq_index = calc_irq_index(index.value(), sm_id);
                        if clear.value() == 1 {
                            *irq_flags &= !(1 << irq_index);
                        } else {
                            *irq_flags |= 1 << irq_index;
                            if wait.value() == 1 {
                                self.state.irq_wait = Some(irq_index);
                            }
                        }
                    }
                    Instruction::Set { destn, data } => match destn {
                        set::Destn::Pins => {
                            write_pins(
                                gpio_out,
                                data.value() as u32,
                                self.config.set_base.get(),
                                self.config.set_count.get(),
                            );
                        }
                        set::Destn::X => {
                            self.state.x = data.value() as u32;
                        }
                        set::Destn::Y => {
                            self.state.y = data.value() as u32;
                        }
                        set::Destn::PinDirs => {
                            write_pins(
                                gpio_dir,
                                data.value() as u32,
                                self.config.set_base.get(),
                                self.config.set_count.get(),
                            );
                        }
                        _ => panic!(),
                    },
                }

                if let Some(value) = instr.side_set {
                    let pins = if self.config.side_pindir { gpio_dir } else { gpio_out };
                    let count = self.config.sideset_count.get() - self.config.sideset_en as u8;
                    write_pins(pins, value.value() as u32, self.config.sideset_base.get(), count);
                }
                if !self.state.stalled {
                    self.state.delay_counter = instr.delay.value();
                    if advance_pc {
                        if self.state.pc == self.config.wrap_top {
                            self.state.pc = self.config.wrap_bottom;
                        } else {
                            self.state.pc = u5::new((self.state.pc.value() + 1) % 32);
                        }
                    }
                }
            }
        }
        let count = self.state.osr_shift_count;
        if self.config.autopull && (was_empty && count >= threshold || count == threshold) && !self.state.tx_fifo.is_empty() {
            self.state.osr = self.state.tx_fifo.pop().expect("tx fifo empty when it shouldn't be");
            self.state.osr_shift_count = 0;
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
    irq_wait: Option<u8>,
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
            irq_wait: None,
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

impl Config {
    // TODO pull_thresh override 32 = 0 and fifo_depth if fjoin
    fn new() -> Self {
        Self::from_registers(0x0001_f000, 0x000c_0000, 0x1400_0000)
    }
    pub fn from_registers(execctrl: u32, shiftctrl: u32, pinctrl: u32) -> Self {
        let field = |register: u32, lsb: u32, bits: u32| ((register >> lsb) & ((1 << bits) - 1)) as u8;
        let shift_dir = |bit| {
            if field(shiftctrl, bit, 1) == 1 {
                ShiftDir::Right
            } else {
                ShiftDir::Left
            }
        };
        Self {
            out_base: Range::new(field(pinctrl, 0, 5)),
            out_count: Range::new(field(pinctrl, 20, 6)),
            set_base: Range::new(field(pinctrl, 5, 5)),
            set_count: Range::new(field(pinctrl, 26, 3)),
            in_base: Range::new(field(pinctrl, 15, 5)),
            sideset_base: Range::new(field(pinctrl, 10, 5)),
            sideset_count: Range::new(field(pinctrl, 29, 3)),
            sideset_en: field(execctrl, 30, 1) == 1,
            side_pindir: field(execctrl, 29, 1) == 1,
            jmp_pin: u5::new(field(execctrl, 24, 5)),
            wrap_top: u5::new(field(execctrl, 12, 5)),
            wrap_bottom: u5::new(field(execctrl, 7, 5)),
            status_sel: match field(execctrl, 5, 2) {
                0 => StatusSel::TxLevel,
                1 => StatusSel::RxLevel,
                _ => panic!(),
            },
            status_n: Range::new(field(execctrl, 0, 4)),
            pull_thresh: Range::new(field(shiftctrl, 25, 5)),
            push_thresh: Range::new(field(shiftctrl, 20, 5)),
            out_shiftdir: shift_dir(19),
            in_shiftdir: shift_dir(18),
            autopull: field(shiftctrl, 17, 1) == 1,
            autopush: field(shiftctrl, 16, 1) == 1,
            fjoin_rx: field(shiftctrl, 31, 1) == 1,
            fjoin_tx: field(shiftctrl, 30, 1) == 1,
        }
    }
    fn calc_pull_thresh(&self) -> u8 {
        if self.pull_thresh.get() == 0 { 32 } else { self.pull_thresh.get() }
    }
    fn calc_push_thresh(&self) -> u8 {
        if self.push_thresh.get() == 0 { 32 } else { self.push_thresh.get() }
    }
}
