use crate::instr::{Condition, Instr, Instruction, mov, set, shift, wait};
use arbitrary_int::{u1, u5};
use std::cmp;
use std::collections::VecDeque;
use std::fmt;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StepEvent {
    pub rx_pushed: Option<u32>,
    pub tx_popped: Option<u32>,
    pub irq_changed: Option<u8>,
}

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
    fn len(&self) -> usize {
        self.data.len()
    }
    pub fn peek_front(&self) -> Option<u32> {
        self.data.front().copied()
    }
    pub fn peek_back(&self) -> Option<u32> {
        self.data.back().copied()
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
    pub fn step(&mut self) -> StepEvent {
        let Block { sms, instr_mem, gpio_out, gpio_dir, gpio_in, irq_flags, cycle } = self;
        // only for SM0
        let mut event = StepEvent { rx_pushed: None, tx_popped: None, irq_changed: None };
        for (i, sm) in sms.iter_mut().enumerate() {
            if !sm.enabled {
                continue;
            }
            let pc = sm.state.pc.value() as usize;
            let instr = instr_mem[pc].expect("no instruction at PC");

            let ev = sm.execute(&instr, gpio_out, gpio_dir, *gpio_in, irq_flags, i as u8);
            if ev.rx_pushed.is_some() {
                event.rx_pushed = ev.rx_pushed;
            }
            if ev.tx_popped.is_some() {
                event.tx_popped = ev.tx_popped;
            }
            if ev.irq_changed.is_some() {
                event.irq_changed = ev.irq_changed;
            }
        }
        *cycle += 1;
        event
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

#[derive(Clone, Debug, Default)]
pub struct Sm0Setup {
    pub instructions: Vec<u16>, // assembled PIO words, <= 32
    pub origin: u8,
    pub wrap_top: u8,
    pub wrap_bottom: u8,
    pub in_base: u8,
    pub out_base: u8,
    pub out_count: u8,
    pub set_base: u8,
    pub set_count: u8,
    pub jmp_pin: u8,
    pub pull_thresh: u8,
    pub push_thresh: u8,
    pub out_shiftdir_right: bool,
    pub in_shiftdir_right: bool,
    pub autopull: bool,
    pub autopush: bool,
    pub fjoin_tx: bool,
    pub fjoin_rx: bool,
    pub x_init: u32,
    pub y_init: u32,
    pub tx_seed: Vec<u32>,
}

/// Decode an assembled PIO word into the emulator's own `Instr`. Decoding uses
/// the `pio` crate's decoder and translates it to emulator's Instr
pub fn decode_to_instr(word: u16) -> Option<Instr> {
    use pio::{
        InSource as PIn, Instruction as PInstr, InstructionOperands as PO, JmpCondition as PJ, MovDestination as PMD, MovOperation as PMO,
        MovSource as PMS, OutDestination as POut, SetDestination as PSet, SideSet, WaitSource as PW,
    };

    // no side-set
    let decoded = PInstr::decode(word, SideSet::new(false, 0, false))?;
    // 0x00 = 32
    let bc = |bit_count: u8| u5::new(bit_count & 0x1f);

    let instruction = match decoded.operands {
        PO::JMP { condition, address } => Instruction::Jmp {
            condition: match condition {
                PJ::Always => Condition::Always,
                PJ::XIsZero => Condition::XZero,
                PJ::XDecNonZero => Condition::XDec,
                PJ::YIsZero => Condition::YZero,
                PJ::YDecNonZero => Condition::YDec,
                PJ::XNotEqualY => Condition::XNeqY,
                PJ::PinHigh => Condition::Pin,
                PJ::OutputShiftRegisterNotEmpty => Condition::OsrNotEmpty,
            },
            address: u5::new(address & 0x1f),
        },
        PO::WAIT { polarity, source, index, .. } => Instruction::Wait {
            polarity: u1::new(polarity & 1),
            source: match source {
                PW::GPIO => wait::Source::Gpio,
                PW::PIN => wait::Source::Pin,
                PW::IRQ => wait::Source::Irq,
                PW::JMPPIN => wait::Source::Reserved,
            },
            index: u5::new(index & 0x1f),
        },
        PO::IN { source, bit_count } => Instruction::In {
            source: match source {
                PIn::PINS => shift::Source::Pins,
                PIn::X => shift::Source::X,
                PIn::Y => shift::Source::Y,
                PIn::NULL => shift::Source::Null,
                PIn::ISR => shift::Source::Isr,
                PIn::OSR => shift::Source::Osr,
            },
            bit_count: bc(bit_count),
        },
        PO::OUT { destination, bit_count } => Instruction::Out {
            destn: match destination {
                POut::PINS => shift::Destn::Pins,
                POut::X => shift::Destn::X,
                POut::Y => shift::Destn::Y,
                POut::NULL => shift::Destn::Null,
                POut::PINDIRS => shift::Destn::PinDirs,
                POut::PC => shift::Destn::Pc,
                POut::ISR => shift::Destn::Isr,
                POut::EXEC => shift::Destn::Exec,
            },
            bit_count: bc(bit_count),
        },
        PO::PUSH { if_full, block } => Instruction::Push { if_full: u1::new(if_full as u8), block: u1::new(block as u8) },
        PO::PULL { if_empty, block } => Instruction::Pull { if_empty: u1::new(if_empty as u8), block: u1::new(block as u8) },
        PO::MOV { destination, op, source } => Instruction::Mov {
            destn: match destination {
                PMD::PINS => mov::Destn::Pins,
                PMD::X => mov::Destn::X,
                PMD::Y => mov::Destn::Y,
                PMD::PINDIRS => mov::Destn::Reserved, // emulator models no MOV→PINDIRS
                PMD::EXEC => mov::Destn::Exec,
                PMD::PC => mov::Destn::Pc,
                PMD::ISR => mov::Destn::Isr,
                PMD::OSR => mov::Destn::Osr,
            },
            op: match op {
                PMO::None => mov::Op::None,
                PMO::Invert => mov::Op::Invert,
                PMO::BitReverse => mov::Op::BitReverse,
            },
            source: match source {
                PMS::PINS => mov::Source::Pins,
                PMS::X => mov::Source::X,
                PMS::Y => mov::Source::Y,
                PMS::NULL => mov::Source::Null,
                PMS::STATUS => mov::Source::Status,
                PMS::ISR => mov::Source::Isr,
                PMS::OSR => mov::Source::Osr,
            },
        },
        PO::IRQ { clear, wait, index, .. } => {
            Instruction::Irq { clear: u1::new(clear as u8), wait: u1::new(wait as u8), index: u5::new(index & 0x1f) }
        }
        PO::SET { destination, data } => Instruction::Set {
            destn: match destination {
                PSet::PINS => set::Destn::Pins,
                PSet::X => set::Destn::X,
                PSet::Y => set::Destn::Y,
                PSet::PINDIRS => set::Destn::PinDirs,
            },
            data: u5::new(data & 0x1f),
        },
        _ => return None,
    };

    Some(Instr { instruction, delay: u5::new(decoded.delay & 0x1f), side_set: None })
}

impl Block {
    pub fn irq_flags(&self) -> u8 {
        self.irq_flags
    }

    /// SM0's FLEVEL packing: TX0 in bits[3:0], RX0 in bits[7:4] (RP2040 layout;
    /// the other SMs are idle in v1 so their nibbles read 0).
    ///
    /// seems fine to me (???)
    pub fn fifo_levels_sm0(&self) -> u32 {
        let tx0 = self.sms[0].state.tx_fifo.len() as u32;
        let rx0 = self.sms[0].state.rx_fifo.len() as u32;
        (tx0 & 0xf) | ((rx0 & 0xf) << 4)
    }

    /// Configure SM0 from a `Sm0Setup`
    pub fn configure_sm0(&mut self, prog: &Sm0Setup) {
        // Full reset 
        self.instr_mem = std::array::from_fn(|_| None);
        self.gpio_out = 0;
        self.gpio_dir = 0;
        self.gpio_in = 0;
        self.irq_flags = 0;
        self.cycle = 0;
        for sm in &mut self.sms {
            sm.enabled = false;
            sm.state = State::new();
            sm.config = Config::new();
        }

        // Yeah idk about this part tbh

        // Decode + load the program at [origin, origin+len), as the firmware
        // loads instruction memory. Undecodable / unmodelled words stay `None`;
        // the run loop stops if the PC reaches one.
        for (i, &word) in prog.instructions.iter().enumerate() {
            let addr = prog.origin as usize + i;
            if addr < 32 {
                self.instr_mem[addr] = decode_to_instr(word);
            }
        }

        let sm = &mut self.sms[0];
        sm.enabled = true;
        sm.state.pc = u5::new(prog.origin & 0x1f);

        let cfg = &mut sm.config;
        cfg.out_base = PinRange::new(prog.out_base & 0x1f);
        cfg.out_count = Range::new(prog.out_count.min(32));
        cfg.set_base = PinRange::new(prog.set_base & 0x1f);
        cfg.set_count = Range::new(prog.set_count.min(5));
        cfg.in_base = PinRange::new(prog.in_base & 0x1f);
        cfg.sideset_base = PinRange::new(0);
        cfg.sideset_count = Range::new(0);
        cfg.sideset_en = false;
        cfg.side_pindir = false;
        cfg.jmp_pin = u5::new(prog.jmp_pin & 0x1f);
        cfg.wrap_top = u5::new(prog.wrap_top & 0x1f);
        cfg.wrap_bottom = u5::new(prog.wrap_bottom & 0x1f);
        // Threshold of 32 is encoded as the 5-bit value 0 (calc_*_thresh maps
        // it back), matching the firmware's `& 0x1f`.
        cfg.pull_thresh = Range::new(prog.pull_thresh & 0x1f);
        cfg.push_thresh = Range::new(prog.push_thresh & 0x1f);
        cfg.out_shiftdir = if prog.out_shiftdir_right { ShiftDir::Right } else { ShiftDir::Left };
        cfg.in_shiftdir = if prog.in_shiftdir_right { ShiftDir::Right } else { ShiftDir::Left };
        cfg.autopull = prog.autopull;
        cfg.autopush = prog.autopush;
        cfg.fjoin_tx = prog.fjoin_tx;
        cfg.fjoin_rx = prog.fjoin_rx;

        // FIFO depths follow the join setting: a join steals the other FIFO's
        // four entries (RP2040 §3.5.4). v1 generation keeps both joins off.
        let (tx_depth, rx_depth) = match (prog.fjoin_tx, prog.fjoin_rx) {
            (true, false) => (FIFO_DEPTH * 2, 0),
            (false, true) => (0, FIFO_DEPTH * 2),
            _ => (FIFO_DEPTH, FIFO_DEPTH),
        };
        sm.state.tx_fifo = Fifo::new(tx_depth);
        sm.state.rx_fifo = Fifo::new(rx_depth);

        // Preload X/Y and seed TX, exactly as the firmware does — and only when
        // a TX FIFO exists (an RX join leaves X/Y at their reset value 0 and
        // nothing to seed). After the firmware's PULL+OUT preload the OSR is
        // empty (osr_shift_count = 32), which is `State::new`'s default.
        if !prog.fjoin_rx {
            sm.state.x = prog.x_init;
            sm.state.y = prog.y_init;
            for &word in prog.tx_seed.iter() {
                sm.state.tx_fifo.push(word);
            }
        }
    }

    /// Force one instruction into SM0 without PC/instr-mem bookkeeping 
    fn force_sm0(&mut self, instruction: Instruction) {
        let instr = Instr { instruction, delay: u5::new(0), side_set: None };
        let Block { sms, gpio_out, gpio_dir, gpio_in, irq_flags, .. } = self;
        let _ = sms[0].execute(&instr, gpio_out, gpio_dir, *gpio_in, irq_flags, 0);
    }

    // I don't know about these parts

    /// Drain SM0's RX FIFO (the program's own pushed words), capped at 8 like
    /// the firmware's `drain_rx`. Must run *before* [`Block::readout_suffix_sm0`].
    pub fn drain_rx0(&mut self) -> Vec<u32> {
        let mut v = Vec::new();
        while v.len() < 8 {
            match self.sms[0].state.rx_fifo.pop() {
                Some(w) => v.push(w),
                None => break,
            }
        }
        v
    }

    /// The identical readout suffix to `firmware::runner::readout_suffix`: with
    /// autopush/autopull disabled, expose the hidden registers through the RX
    /// FIFO via forced instructions and compare the *drained words* (never the
    /// internal x/y fields). Yields `[ISR, X, Y, OSR]`.
    pub fn readout_suffix_sm0(&mut self) -> Vec<u32> {
        self.sms[0].config.autopush = false;
        self.sms[0].config.autopull = false;

        let push = Instruction::Push { if_full: u1::new(0), block: u1::new(0) };
        let mut v = Vec::new();
        let pop_into = |this: &mut Block, v: &mut Vec<u32>| {
            if let Some(w) = this.sms[0].state.rx_fifo.pop() {
                v.push(w);
            }
        };

        self.force_sm0(push); // FIFO <- ISR
        pop_into(self, &mut v);
        self.force_sm0(Instruction::In { source: shift::Source::X, bit_count: u5::new(0) }); // ISR <- X
        self.force_sm0(push); // FIFO <- X
        pop_into(self, &mut v);
        self.force_sm0(Instruction::In { source: shift::Source::Y, bit_count: u5::new(0) }); // ISR <- Y
        self.force_sm0(push); // FIFO <- Y
        pop_into(self, &mut v);
        self.force_sm0(Instruction::Mov { destn: mov::Destn::Isr, op: mov::Op::None, source: mov::Source::Osr }); // ISR <- OSR (copy, does not drain OSR)
        self.force_sm0(push); // FIFO <- OSR
        pop_into(self, &mut v);

        v
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
    // Capture-before / diff-after wrapper around the instruction body. Snapshot
    // the FIFO/IRQ-shaped state up front, run the (unchanged) execute body, then
    // diff to recover what landed this cycle. This catches every push/pop/irq
    // site — PUSH, PULL, autopush in IN, autopull in OUT, the trailing autopull
    // after non-OUT instructions, MOV/wait-on-IRQ clearing — without threading
    // event fields through each one, and it works regardless of which early
    // return the body took.
    fn execute(&mut self, instr: &Instr, gpio_out: &mut u32, gpio_dir: &mut u32, gpio_in: u32, irq_flags: &mut u8, sm_id: u8) -> StepEvent {
        let irq_before = *irq_flags;
        let rx_before_len = self.state.rx_fifo.len();
        let tx_before_len = self.state.tx_fifo.len();
        let tx_front_before = self.state.tx_fifo.peek_front();

        self.execute_inner(instr, gpio_out, gpio_dir, gpio_in, irq_flags, sm_id);

        let irq_changed = (*irq_flags != irq_before).then_some(*irq_flags);
        let rx_pushed = (self.state.rx_fifo.len() > rx_before_len).then(|| self.state.rx_fifo.peek_back().expect("just pushed"));
        let tx_popped = (self.state.tx_fifo.len() < tx_before_len).then(|| tx_front_before.expect("tx shrank, so it had a front"));
        StepEvent { rx_pushed, tx_popped, irq_changed }
    }

    // TODO function for gpio in / out mapping
    fn execute_inner(&mut self, instr: &Instr, gpio_out: &mut u32, gpio_dir: &mut u32, gpio_in: u32, irq_flags: &mut u8, sm_id: u8) {
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
                    ShiftDir::Left => self.state.isr = sat_shl(self.state.isr, bit_count) | data,
                    ShiftDir::Right => self.state.isr = sat_shr(self.state.isr, bit_count) | wrap_shiftr(data, bit_count),
                }
                self.state.isr_shift_count = cmp::min(63, self.state.isr_shift_count + bit_count);
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
                if self.config.autopull && self.state.osr_shift_count >= self.config.calc_pull_thresh() {
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
                    shift::Destn::Exec => panic!(), // todo EXEC destn. i think we need to have bit decoding for this to work
                }

                self.state.osr_shift_count = cmp::min(63, self.state.osr_shift_count + bit_count);
                if self.config.autopull && self.state.osr_shift_count >= self.config.calc_pull_thresh() {
                    if !self.state.tx_fifo.is_empty() {
                        self.state.osr = self.state.tx_fifo.pop().expect("tx fifo empty when it shouldn't be");
                        self.state.osr_shift_count = 0;
                    }
                }
            }
            Instruction::Push { if_full, block } => {
                let (if_full, block) = (if_full.value() == 1, block.value() == 1);
                let should_push = !if_full || (self.state.isr_shift_count >= self.config.calc_push_thresh());
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
                        let (out_count, out_base) = (self.config.out_count.get(), self.config.out_base.get());
                        let mask = to_mask(out_count) << out_base;
                        *gpio_out = (*gpio_out & !mask) | ((data << out_base) & mask);
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
                let (clear, wait, index) = (clear.value() == 1, wait.value() == 1, index.value());
                let irq_index = calc_irq_index(index as u8, sm_id);

                // irq_stalled implies this instr is being run for the 2nd time
                // so it's "polling" whether the flag has been cleared by an external source
                // - and not doing the normal instr behaviour
                if self.state.irq_stalled {
                    if bit_at(*irq_flags, irq_index) {
                        self.state.stalled = true;
                        return;
                    } else {
                        self.state.irq_stalled = false;
                    }
                // normal set / clear behaviour
                } else {
                    if clear {
                        *irq_flags &= !(1 << irq_index);
                    } else {
                        // set the flag
                        *irq_flags |= 1 << irq_index;
                        if wait {
                            self.state.irq_stalled = true;
                            self.state.stalled = true;
                            return;
                        }
                    }
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
        }

        // autopull
        if !matches!(instr.instruction, Instruction::Out { .. }) {
            if matches!(instr.instruction, Instruction::Mov { .. } | Instruction::Pull { .. }) {
                self.state.osr_shift_count = 0;
            }
            if self.state.osr_shift_count >= self.config.calc_pull_thresh() {
                if !self.state.tx_fifo.is_empty() {
                    self.state.osr = self.state.tx_fifo.pop().expect("tx fifo empty when it shouldn't be");
                    self.state.osr_shift_count = 0;
                }
            }
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
    irq_stalled: bool,
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
            irq_stalled: false,
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
