use crate::assert_strict;
use crate::instr::{Condition, Instr, Instruction, mov, set, shift, wait};
use arbitrary_int::u5;
use std::cmp;
use std::collections::VecDeque;

#[derive(Clone, Copy, Debug)]
pub struct Range<const MIN: u8, const MAX: u8>(u8);

impl<const MIN: u8, const MAX: u8> Range<MIN, MAX> {
    pub fn new(val: u8) -> Self {
        assert!(val >= MIN && val <= MAX, "value {val} not in [{MIN}, {MAX}]");
        Self(val)
    }

    pub fn get(self) -> u8 {
        self.0
    }
}
type PinRange = Range<0, 31>;

#[derive(Debug)]
pub struct Block {
    pub instr_mem: [u16; 32],
    pub sms: [StateMachine; 4],
    pads: [u32; 2],
    pub gpio_in: u32,
    pub irq_flags: u32,
    cycle: u64,
    strict: bool,
}

impl Block {
    pub fn new(strict: bool) -> Self {
        Self {
            instr_mem: [0; 32],
            sms: std::array::from_fn(|_| StateMachine { state: State::new(), config: Config::new(), enabled: false }),
            pads: [0; 2],
            gpio_in: 0,
            irq_flags: 0,
            cycle: 0,
            strict,
        }
    }
    pub fn configure(&mut self, sm: usize, clkdiv: u32, execctrl: u32, shiftctrl: u32, pinctrl: u32) {
        let field = |register: u32, lsb: u32, bits: u32| (register >> lsb) & ((1 << bits) - 1);
        assert_strict(
            self.strict,
            field(shiftctrl, 16, 1) == 0 || field(shiftctrl, 15, 1) == 0,
            "autopush_rx_put",
        );
        assert_strict(
            self.strict,
            field(shiftctrl, 16, 1) == 0 || field(shiftctrl, 14, 1) == 0,
            "autopush_rx_get",
        );
        assert_strict(
            self.strict,
            field(shiftctrl, 14, 2) == 0 || field(shiftctrl, 30, 2) == 0,
            "fjoin_with_rx_putget",
        );
        assert_strict(self.strict, field(pinctrl, 20, 6) <= 32, "out_count_above_32");
        assert_strict(self.strict, field(pinctrl, 26, 3) <= 5, "set_count_above_5");
        assert_strict(self.strict, field(pinctrl, 29, 3) <= 5, "sideset_count_above_5");
        assert_strict(
            self.strict,
            field(execctrl, 30, 1) == 0 || field(pinctrl, 29, 3) > 0,
            "sideset_en_count_0",
        );
        assert_strict(
            self.strict,
            field(clkdiv, 16, 16) > 0 || field(clkdiv, 8, 8) == 0,
            "clkdiv_int_0_frac_nonzero",
        );
        self.sms[sm].config = Config::from_registers(clkdiv, execctrl, shiftctrl, pinctrl);
    }
    pub fn step(&mut self) {
        let Block { sms, instr_mem, pads, gpio_in, irq_flags, cycle, strict } = self;
        let (mut irq_set, mut irq_clear) = (0, 0);
        for (i, sm) in sms.iter_mut().enumerate() {
            let waiting = sm.state.irq_wait.is_some();
            sm.state.irq_wait = sm.state.irq_wait.filter(|&index| !sm.enabled || bit_at(*irq_flags, index));
            let ticks = |cycle: u64| match cycle {
                0 => 0,
                _ => (cycle * 256 + sm.config.clkdiv % 256).div_ceil(sm.config.clkdiv),
            };
            let word = sm.state.exec.unwrap_or(instr_mem[sm.state.pc.value() as usize]);
            let (set, clear, wrote_pins, side_write) = match sm.enabled && !waiting && ticks(*cycle + 1) > ticks(*cycle) {
                true => sm.execute(word, *gpio_in, *irq_flags, i as u8, *strict),
                false => (0, 0, false, None),
            };
            let pin_write = sm.state.last_pin_write.filter(|_| wrote_pins || sm.enabled && sm.config.out_sticky);
            for write in [pin_write, side_write].into_iter().flatten() {
                write_pins(pads, write);
            }
            (irq_set, irq_clear) = (irq_set | set, irq_clear | clear);
        }
        *irq_flags = *irq_flags & !irq_clear & 0xff | irq_set;
        *cycle += 1;
    }
    pub fn pads(&self) -> [u32; 2] {
        let mut pads = self.pads;
        for write in self
            .sms
            .iter()
            .filter(|sm| sm.enabled && sm.config.out_sticky)
            .filter_map(|sm| sm.state.last_pin_write)
        {
            write_pins(&mut pads, write);
        }
        pads
    }
}

#[derive(Debug)]
pub struct StateMachine {
    pub state: State,
    config: Config,
    pub enabled: bool,
}

pub fn to_mask(val: u8) -> u32 {
    assert!(val <= 32);
    if val == 32 { u32::MAX } else { (1u32 << val) - 1 }
}

fn write_pins(pads: &mut [u32; 2], (dirs, data, base, count): (bool, u32, u8, u8)) {
    let pins = &mut pads[dirs as usize];
    let mask = to_mask(count).rotate_left(base as u32);
    *pins = (*pins & !mask) | (data.rotate_left(base as u32) & mask);
}

pub fn bit_at(x: u32, index: u8) -> bool {
    ((x >> index) & 0x1) == 1
}

pub fn calc_irq_index(index: u8, sm_id: u8) -> u8 {
    match index >> 3 & 3 {
        2 => (index & 4) | ((index + sm_id) & 3),
        mode => [0, 8, 0, 16][mode as usize] | index & 7,
    }
}

impl StateMachine {
    pub fn write_tx(&mut self, word: u32) {
        if self.state.tx_fifo.len() < self.config.tx_depth {
            self.state.tx_fifo.push_back(word);
        } else {
            self.state.tx_over = true;
        }
    }

    fn execute(
        &mut self,
        word: u16,
        gpio_in: u32,
        irq_flags: u32,
        sm_id: u8,
        strict: bool,
    ) -> (u32, u32, bool, Option<(bool, u32, u8, u8)>) {
        let (mut irq_set, mut irq_clear, mut wrote_pins) = (0, 0, false);
        let threshold = self.config.pull_thresh.get();
        let was_empty = self.state.osr_shift_count >= threshold;
        let side_write = match self.state.delay_counter > 0 && !self.state.stalled {
            true => {
                self.state.delay_counter -= 1;
                None
            }
            false => {
                let instr = Instr::decode(word, &self.config, strict);
                let mut advance_pc = true;
                let exec = self.state.exec.take();
                self.state.stalled = false;
                let status = match self.config.status {
                    Status::TxLevel(n) => self.state.tx_fifo.len() < (n & 15) as usize,
                    Status::RxLevel(n) => self.state.rx_fifo.len() < (n & 15) as usize,
                    Status::Irq(n) => bit_at(irq_flags, n),
                    Status::Reserved => false,
                };
                let status = if status { u32::MAX } else { 0 };
                let pins = gpio_in.rotate_right(self.config.in_base.get() as u32) & to_mask(self.config.in_count.get());
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
                            Condition::OsrNotEmpty => self.state.osr_shift_count < self.config.pull_thresh.get(),
                        };
                        if jump {
                            self.state.pc = address;
                            advance_pc = false;
                        }
                    }
                    Instruction::Wait { polarity, source, index } => {
                        let (polarity, index) = (polarity.value() as u32, index.value() as u32);
                        let irq_index = calc_irq_index(index as u8, sm_id);
                        assert_strict(strict, !matches!(source, wait::Source::Irq) || irq_index < 8, "irq_prev_next");
                        assert_strict(
                            strict,
                            !matches!(source, wait::Source::JmpPin) || index < 4,
                            "wait_jmppin_index_above_3",
                        );
                        let cond_met = match source {
                            wait::Source::Gpio => (gpio_in >> index) & 1,
                            wait::Source::Pin => (pins >> index) & 1,
                            wait::Source::Irq => irq_flags >> irq_index & 1,
                            wait::Source::JmpPin => (gpio_in >> ((self.config.jmp_pin.value() as u32 + (index & 3)) % 32)) & 1,
                        } == polarity;
                        if cond_met {
                            if let (wait::Source::Irq, 1) = (source, polarity) {
                                irq_clear |= 1 << irq_index;
                            }
                        } else {
                            self.state.stalled = true;
                        }
                    }
                    Instruction::In { source, bit_count } => {
                        let bit_count = bit_count.get();
                        let data = match source {
                            shift::Source::Pins => pins,
                            shift::Source::X => self.state.x,
                            shift::Source::Y => self.state.y,
                            shift::Source::Null => 0,
                            shift::Source::Reserved1 | shift::Source::Reserved2 => {
                                assert_strict(strict, false, "in_reserved_sources");
                                status
                            }
                            shift::Source::Isr => self.state.isr,
                            shift::Source::Osr => self.state.osr,
                        } & to_mask(bit_count);
                        let isr = match self.config.in_shiftdir {
                            ShiftDir::Left => self.state.isr.checked_shl(bit_count as u32).unwrap_or(0) | data,
                            ShiftDir::Right => {
                                self.state.isr.checked_shr(bit_count as u32).unwrap_or(0) | data.rotate_right(bit_count as u32)
                            }
                        };
                        let isr_shift_count = cmp::min(63, self.state.isr_shift_count + bit_count);
                        if self.config.autopush && isr_shift_count >= self.config.push_thresh.get() {
                            match (
                                self.config.rx_put,
                                self.config.rx_get,
                                self.state.rx_fifo.len() < self.config.rx_depth,
                            ) {
                                (true, _, _) => {
                                    self.state.rx_putget[if bit_count & 8 == 0 {
                                        self.state.y as usize & 3
                                    } else {
                                        bit_count as usize & 3
                                    }] = isr
                                }
                                (false, true, _) => (),
                                (false, false, true) => self.state.rx_fifo.push_back(isr),
                                (false, false, false) => {
                                    self.state.stalled = true;
                                    self.state.rx_stalled = true;
                                }
                            }
                            if !self.state.stalled {
                                self.state.isr = 0;
                                self.state.isr_shift_count = 0;
                            }
                        } else {
                            self.state.isr = isr;
                            self.state.isr_shift_count = isr_shift_count;
                        }
                    }
                    Instruction::Out { destn, bit_count } => {
                        let bit_count = bit_count.get();
                        if self.config.autopull && self.state.osr_shift_count >= self.config.pull_thresh.get() {
                            self.state.stalled = true;
                            self.state.tx_stalled |= self.state.tx_fifo.is_empty();
                        } else {
                            let data = match self.config.out_shiftdir {
                                ShiftDir::Left => {
                                    let ans = self.state.osr.checked_shr(32 - bit_count as u32).unwrap_or(0) & to_mask(bit_count);
                                    self.state.osr = self.state.osr.checked_shl(bit_count as u32).unwrap_or(0);
                                    ans
                                }
                                ShiftDir::Right => {
                                    let ans = self.state.osr & to_mask(bit_count);
                                    self.state.osr = self.state.osr.checked_shr(bit_count as u32).unwrap_or(0);
                                    ans
                                }
                            };
                            let enabled = self.config.out_enable_bit.is_none_or(|bit| data >> bit & 1 == 1);
                            match destn {
                                shift::Destn::Pins | shift::Destn::PinDirs => {
                                    let dirs = matches!(destn, shift::Destn::PinDirs);
                                    let write = (dirs, data, self.config.out_base.get(), self.config.out_count.get() * enabled as u8);
                                    self.state.last_pin_write = Some(write);
                                    wrote_pins = true;
                                }
                                shift::Destn::X => self.state.x = data,
                                shift::Destn::Y => self.state.y = data,
                                shift::Destn::Null => (),
                                shift::Destn::Pc => {
                                    self.state.pc = u5::new(data as u8 & 31);
                                    advance_pc = false;
                                }
                                shift::Destn::Isr => {
                                    self.state.isr = data;
                                    self.state.isr_shift_count = bit_count;
                                }
                                shift::Destn::Exec => self.state.exec = Some(data as u16),
                            }

                            self.state.osr_shift_count = cmp::min(32, self.state.osr_shift_count + bit_count);
                        }
                    }
                    Instruction::Push { if_full, block, index } | Instruction::MovToRx { if_full, block, index } => {
                        let put = matches!(instr.instruction, Instruction::MovToRx { .. });
                        match (put, self.config.rx_put, self.config.rx_get) {
                            (false, true, _) => assert_strict(strict, false, "push_rx_put"),
                            (false, false, true) => assert_strict(strict, false, "push_rx_get"),
                            (true, false, true) => assert_strict(strict, false, "mov_rxfifo_isr_get_only"),
                            (true, false, false) => assert_strict(strict, false, "mov_rxfifo_isr_no_join"),
                            _ => (),
                        }
                        let (if_full, block) = (if_full.value() == 1, block.value() == 1);
                        let should_push = put || !if_full || self.state.isr_shift_count >= self.config.push_thresh.get();
                        if should_push {
                            let pushed = match (
                                self.config.rx_put,
                                self.config.rx_get,
                                self.state.rx_fifo.len() < self.config.rx_depth,
                            ) {
                                (true, _, _) => {
                                    self.state.rx_putget[index.map_or(self.state.y as usize & 3, |i| i.value() as usize)] = self.state.isr;
                                    true
                                }
                                (false, true, _) => true,
                                (false, false, true) => {
                                    self.state.rx_fifo.push_back(self.state.isr);
                                    true
                                }
                                (false, false, false) => false,
                            };
                            self.state.rx_stalled |= !pushed;
                            match (pushed || !block, put) {
                                (true, false) => {
                                    self.state.isr = 0;
                                    self.state.isr_shift_count = 0;
                                }
                                (true, true) => (),
                                (false, _) => self.state.stalled = true,
                            }
                        }
                    }
                    Instruction::Pull { if_empty, block } => {
                        let (if_empty, block) = (if_empty.value() == 1, block.value() == 1);
                        let should_pull = !(self.config.autopull && self.state.osr_shift_count == 0)
                            && (!if_empty || self.state.osr_shift_count >= self.config.pull_thresh.get());
                        if should_pull {
                            match (self.state.tx_fifo.pop_front(), block) {
                                (Some(word), _) => {
                                    self.state.osr = word;
                                    self.state.osr_shift_count = 0;
                                }
                                (None, true) => {
                                    self.state.stalled = true;
                                    self.state.tx_stalled = true;
                                }
                                (None, false) => {
                                    self.state.tx_stalled = true;
                                    self.state.osr = self.state.x;
                                    self.state.osr_shift_count = 0;
                                }
                            }
                        }
                    }
                    Instruction::MovFromRx { if_empty, block, index } => {
                        let data = match (self.config.rx_get, self.config.rx_put, self.config.rx_depth) {
                            (true, _, _) => self.state.rx_putget[index.map_or(self.state.y as usize & 3, |i| i.value() as usize)],
                            (false, true, _) => {
                                assert_strict(strict, false, "mov_osr_rxfifo_put_only");
                                self.state.rx_putget[2]
                            }
                            (false, false, 0) => {
                                assert_strict(strict, false, "mov_osr_rxfifo_tx_joined");
                                self.state.tx_fifo.front().copied().unwrap_or(self.state.rx_putget[0])
                            }
                            (false, false, _) => {
                                assert_strict(strict, false, "mov_osr_rxfifo_no_join");
                                self.state.rx_fifo.front().copied().unwrap_or(self.state.rx_putget[0])
                            }
                        };
                        let pull = !(self.config.autopull && self.state.osr_shift_count == 0)
                            && (if_empty.value() == 0 || self.state.osr_shift_count >= self.config.pull_thresh.get());
                        if pull && self.state.tx_fifo.pop_front().is_none() {
                            self.state.tx_stalled = true;
                            self.state.stalled = block.value() == 1;
                        }
                        if !self.state.stalled {
                            self.state.osr = data;
                            self.state.osr_shift_count = 0;
                        }
                    }
                    Instruction::Mov { destn, op, source } => {
                        let data = match source {
                            mov::Source::Pins => pins,
                            mov::Source::X => self.state.x,
                            mov::Source::Y => self.state.y,
                            mov::Source::Null => 0,
                            mov::Source::Reserved => {
                                assert_strict(strict, false, "mov_reserved_source");
                                status
                            }
                            mov::Source::Status => {
                                match self.config.status {
                                    Status::TxLevel(n) => {
                                        assert_strict(strict, n as usize <= self.config.tx_depth, "status_level_above_depth")
                                    }
                                    Status::RxLevel(n) => {
                                        assert_strict(strict, n as usize <= self.config.rx_depth, "status_level_above_depth")
                                    }
                                    Status::Irq(0..8) => (),
                                    Status::Irq(8..24) => assert_strict(strict, false, "irq_prev_next"),
                                    Status::Irq(_) => assert_strict(strict, false, "status_irq_index_above_23"),
                                    Status::Reserved => assert_strict(strict, false, "status_sel_3"),
                                }
                                status
                            }
                            mov::Source::Isr => self.state.isr,
                            mov::Source::Osr => {
                                assert_strict(strict, !self.config.autopull, "mov_osr_autopull");
                                self.state.osr
                            }
                        };
                        let data = match op {
                            mov::Op::None => data,
                            mov::Op::Invert => !data,
                            mov::Op::BitReverse => data.reverse_bits(),
                            mov::Op::Reserved => {
                                assert_strict(strict, false, "mov_reserved_op");
                                !data.reverse_bits()
                            }
                        };
                        match destn {
                            mov::Destn::Pins | mov::Destn::PinDirs => {
                                let enabled = self.config.out_enable_bit.is_none_or(|bit| data >> bit & 1 == 1);
                                let dirs = matches!(destn, mov::Destn::PinDirs);
                                let write = (dirs, data, self.config.out_base.get(), self.config.out_count.get() * enabled as u8);
                                self.state.last_pin_write = Some(write);
                                wrote_pins = true;
                            }
                            mov::Destn::X => self.state.x = data,
                            mov::Destn::Y => self.state.y = data,
                            mov::Destn::Exec => self.state.exec = Some(data as u16),
                            mov::Destn::Pc => {
                                self.state.pc = u5::new(data as u8 & 31);
                                advance_pc = false;
                            }
                            mov::Destn::Isr => {
                                self.state.isr = data;
                                self.state.isr_shift_count = 0;
                            }
                            mov::Destn::Osr => {
                                self.state.osr = data;
                                self.state.osr_shift_count = 0;
                            }
                        }
                    }
                    Instruction::Irq { clear, wait, index } => {
                        let i = calc_irq_index(index.value(), sm_id);
                        assert_strict(strict, i < 8, "irq_prev_next");
                        match (clear.value(), wait.value()) {
                            (1, _) => irq_clear |= 1 << i,
                            (_, wait) => {
                                irq_set |= 1 << i;
                                if wait == 1 {
                                    self.state.irq_wait = Some(i);
                                }
                            }
                        }
                    }
                    Instruction::Set { destn, data } => match destn {
                        set::Destn::Pins | set::Destn::PinDirs => {
                            let dirs = matches!(destn, set::Destn::PinDirs);
                            let data = self.state.last_pin_write.map_or(0, |write| write.1) & !31 | data.value() as u32;
                            let write = (dirs, data, self.config.set_base.get(), self.config.set_count.get());
                            self.state.last_pin_write = Some(write);
                            wrote_pins = true;
                        }
                        set::Destn::X => {
                            self.state.x = data.value() as u32;
                        }
                        set::Destn::Y => {
                            self.state.y = data.value() as u32;
                        }
                        set::Destn::Reserved1 | set::Destn::Reserved2 | set::Destn::Reserved3 | set::Destn::Reserved4 => {
                            assert_strict(strict, false, "set_reserved_destinations")
                        }
                    },
                }

                let side_count = self.config.sideset_count.get().saturating_sub(self.config.sideset_en as u8);
                let side_write = instr.side_set.map(|value| {
                    (
                        self.config.side_pindir,
                        value.value() as u32,
                        self.config.sideset_base.get(),
                        side_count,
                    )
                });
                if self.state.stalled {
                    self.state.exec = exec;
                } else {
                    self.state.delay_counter = instr.delay.value();
                    if advance_pc && exec.is_none() {
                        if self.state.pc == self.config.wrap_top {
                            self.state.pc = self.config.wrap_bottom;
                        } else {
                            self.state.pc = u5::new((self.state.pc.value() + 1) % 32);
                        }
                    }
                }
                side_write
            }
        };
        let count = self.state.osr_shift_count;
        if self.config.autopull && (was_empty && count >= threshold || count == threshold) && !self.state.tx_fifo.is_empty() {
            self.state.osr = self.state.tx_fifo.pop_front().expect("tx fifo empty when it shouldn't be");
            self.state.osr_shift_count = 0;
        }
        (irq_set, irq_clear, wrote_pins, side_write)
    }
}

#[derive(Debug)]
pub struct State {
    pub pc: u5,
    pub x: u32,
    pub y: u32,
    pub isr: u32,
    pub osr: u32,
    pub isr_shift_count: u8,
    pub osr_shift_count: u8,
    pub tx_fifo: VecDeque<u32>,
    pub rx_fifo: VecDeque<u32>,
    pub rx_putget: [u32; 4],
    pub delay_counter: u8,
    pub stalled: bool,
    pub tx_stalled: bool,
    pub rx_stalled: bool,
    pub tx_over: bool,
    last_pin_write: Option<(bool, u32, u8, u8)>,
    irq_wait: Option<u8>,
    exec: Option<u16>,
}

impl State {
    fn new() -> Self {
        Self {
            pc: u5::new(0),
            x: 0,
            y: 0,
            osr: 0,
            isr: 0,
            osr_shift_count: 32,
            isr_shift_count: 0,
            tx_fifo: VecDeque::new(),
            rx_fifo: VecDeque::new(),
            rx_putget: [0; 4],
            delay_counter: 0,
            stalled: false,
            tx_stalled: false,
            rx_stalled: false,
            tx_over: false,
            last_pin_write: None,
            irq_wait: None,
            exec: None,
        }
    }
}

#[derive(Debug)]
enum ShiftDir {
    Left,
    Right,
}

#[derive(Debug)]
enum Status {
    TxLevel(u8),
    RxLevel(u8),
    Irq(u8),
    Reserved,
}

#[derive(Debug)]
pub struct Config {
    out_base: PinRange,
    out_count: Range<0, 32>,
    set_base: PinRange,
    set_count: Range<0, 7>,
    in_base: PinRange,
    in_count: Range<1, 32>,
    sideset_base: PinRange,
    pub(crate) sideset_count: Range<0, 5>,
    pub(crate) sideset_en: bool,
    side_pindir: bool,
    jmp_pin: u5,
    out_enable_bit: Option<u8>,
    out_sticky: bool,
    wrap_top: u5,
    wrap_bottom: u5,
    status: Status,
    pull_thresh: Range<1, 32>,
    push_thresh: Range<1, 32>,
    out_shiftdir: ShiftDir,
    in_shiftdir: ShiftDir,
    autopull: bool,
    autopush: bool,
    rx_put: bool,
    rx_get: bool,
    tx_depth: usize,
    rx_depth: usize,
    clkdiv: u64,
}

impl Config {
    fn new() -> Self {
        Self::from_registers(0x0001_0000, 0x0001_f000, 0x000c_0000, 0x1400_0000)
    }
    pub fn from_registers(clkdiv: u32, execctrl: u32, shiftctrl: u32, pinctrl: u32) -> Self {
        let field = |register: u32, lsb: u32, bits: u32| ((register >> lsb) & ((1 << bits) - 1)) as u8;
        let join = if field(shiftctrl, 14, 2) == 0 {
            field(shiftctrl, 30, 2) as usize
        } else {
            0
        };
        let count = |lsb| match field(shiftctrl, lsb, 5) {
            0 => 32,
            n => n,
        };
        let shift_dir = |bit| {
            if field(shiftctrl, bit, 1) == 1 {
                ShiftDir::Right
            } else {
                ShiftDir::Left
            }
        };
        Self {
            out_base: Range::new(field(pinctrl, 0, 5)),
            out_count: Range::new(cmp::min(32, field(pinctrl, 20, 6))),
            set_base: Range::new(field(pinctrl, 5, 5)),
            set_count: Range::new(field(pinctrl, 26, 3)),
            in_base: Range::new(field(pinctrl, 15, 5)),
            in_count: Range::new(count(0)),
            sideset_base: Range::new((field(pinctrl, 10, 5) + field(pinctrl, 29, 3).saturating_sub(5)) % 32),
            sideset_count: Range::new(cmp::min(5, field(pinctrl, 29, 3))),
            sideset_en: field(execctrl, 30, 1) == 1,
            side_pindir: field(execctrl, 29, 1) == 1,
            jmp_pin: u5::new(field(execctrl, 24, 5)),
            out_enable_bit: (field(execctrl, 18, 1) == 1).then(|| field(execctrl, 19, 5)),
            out_sticky: field(execctrl, 17, 1) == 1,
            wrap_top: u5::new(field(execctrl, 12, 5)),
            wrap_bottom: u5::new(field(execctrl, 7, 5)),
            status: match field(execctrl, 5, 2) {
                0 => Status::TxLevel(field(execctrl, 0, 5)),
                1 => Status::RxLevel(field(execctrl, 0, 5)),
                2 => Status::Irq(field(execctrl, 0, 5)),
                _ => Status::Reserved,
            },
            pull_thresh: Range::new(count(25)),
            push_thresh: Range::new(count(20)),
            out_shiftdir: shift_dir(19),
            in_shiftdir: shift_dir(18),
            autopull: field(shiftctrl, 17, 1) == 1,
            autopush: field(shiftctrl, 16, 1) == 1,
            rx_put: field(shiftctrl, 15, 1) == 1,
            rx_get: field(shiftctrl, 14, 1) == 1,
            tx_depth: [4, 8, 0, 0][join],
            rx_depth: [4, 0, 8, 0][join],
            clkdiv: if clkdiv >> 16 == 0 {
                1 << 24 | clkdiv as u64 >> 8
            } else {
                clkdiv as u64 >> 8
            },
        }
    }
}
