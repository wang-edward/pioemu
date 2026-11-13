use crate::assert_strict;
use crate::state::{Config, Range};
use arbitrary_int::{u1, u2, u5};
use std::fmt;

#[derive(Clone, Copy, Debug)]
#[rustfmt::skip]
pub enum Condition { Always, XZero, XDec, YZero, YDec, XNeqY, Pin, OsrNotEmpty }

#[rustfmt::skip]
pub mod wait {
    #[derive(Clone, Copy, Debug)]
    pub enum Source { Gpio, Pin, Irq, JmpPin }
}

#[rustfmt::skip]
pub mod shift {
    #[derive(Clone, Copy, Debug)]
    pub enum Source { Pins, X, Y, Null, Reserved1, Reserved2, Isr, Osr }
    #[derive(Clone, Copy, Debug)]
    pub enum Destn { Pins, X, Y, Null, PinDirs, Pc, Isr, Exec }
}

#[rustfmt::skip]
pub mod mov {
    #[derive(Clone, Copy, Debug)]
    pub enum Destn { Pins, X, Y, PinDirs, Exec, Pc, Isr, Osr }
    #[derive(Clone, Copy, Debug)]
    pub enum Op { None, Invert, BitReverse, Reserved }
    #[derive(Clone, Copy, Debug)]
    pub enum Source { Pins, X, Y, Null, Reserved, Status, Isr, Osr }
}

#[rustfmt::skip]
pub mod set {
    #[derive(Clone, Copy, Debug)]
    pub enum Destn { Pins, X, Y, Reserved1, PinDirs, Reserved2, Reserved3, Reserved4 }
}

#[derive(Clone, Copy, Debug)]
pub enum Instruction {
    Jmp {
        condition: Condition,
        address: u5,
    },
    Wait {
        polarity: u1,
        source: wait::Source,
        index: u5,
    },
    In {
        source: shift::Source,
        bit_count: Range<1, 32>,
    },
    Out {
        destn: shift::Destn,
        bit_count: Range<1, 32>,
    },
    Push {
        if_full: u1,
        block: u1,
        index: Option<u2>,
    },
    Pull {
        if_empty: u1,
        block: u1,
    },
    MovToRx {
        if_full: u1,
        block: u1,
        index: Option<u2>,
    },
    MovFromRx {
        if_empty: u1,
        block: u1,
        index: Option<u2>,
    },
    Mov {
        destn: mov::Destn,
        op: mov::Op,
        source: mov::Source,
    },
    Irq {
        clear: u1,
        wait: u1,
        index: u5,
    },
    Set {
        destn: set::Destn,
        data: u5,
    },
}

#[derive(Clone, Copy, Debug)]
pub struct Instr {
    pub instruction: Instruction,
    pub delay: u5,
    pub side_set: Option<u5>,
}

impl Instr {
    pub fn decode(word: u16, config: &Config, strict: bool) -> Self {
        let (a, b) = (((word >> 5) & 7) as usize, (word & 31) as u8);
        let bit_count = Range::new(if b == 0 { 32 } else { b });
        let bit = |i: usize| u1::new(((a >> i) & 1) as u8);
        let instruction = match word >> 13 {
            0 => {
                use Condition::*;
                Instruction::Jmp { condition: [Always, XZero, XDec, YZero, YDec, XNeqY, Pin, OsrNotEmpty][a], address: u5::new(b) }
            }
            1 => {
                use wait::Source::*;
                Instruction::Wait { polarity: bit(2), source: [Gpio, Pin, Irq, JmpPin][a & 3], index: u5::new(b) }
            }
            2 => {
                use shift::Source::*;
                Instruction::In { source: [Pins, X, Y, Null, Reserved1, Reserved2, Isr, Osr][a], bit_count }
            }
            3 => {
                use shift::Destn::*;
                Instruction::Out { destn: [Pins, X, Y, Null, PinDirs, Pc, Isr, Exec][a], bit_count }
            }
            4 => {
                let index = (b >> 3 & 1 == 1).then(|| u2::new(b & 3));
                match (b >> 4, a & 4, b & 0x0f) {
                    (0, 0, _) => {
                        assert_strict(strict, b == 0, "push_low_bits");
                        Instruction::Push { if_full: bit(1), block: bit(0), index }
                    }
                    (0, _, _) => {
                        assert_strict(strict, b == 0, "pull_low_bits");
                        Instruction::Pull { if_empty: bit(1), block: bit(0) }
                    }
                    (_, 0, low) => {
                        assert_strict(strict, !(1..8).contains(&low), "put_y_index_nonzero");
                        assert_strict(strict, a == 0, "put_bits_6_5");
                        Instruction::MovToRx { if_full: bit(1), block: bit(0), index }
                    }
                    (_, _, low) => {
                        assert_strict(strict, !(1..8).contains(&low), "get_y_index_nonzero");
                        assert_strict(strict, a == 4, "get_bits_6_5");
                        Instruction::MovFromRx { if_empty: bit(1), block: bit(0), index }
                    }
                }
            }
            5 => {
                use mov::{Destn, Op, Source};
                Instruction::Mov {
                    destn: [
                        Destn::Pins,
                        Destn::X,
                        Destn::Y,
                        Destn::PinDirs,
                        Destn::Exec,
                        Destn::Pc,
                        Destn::Isr,
                        Destn::Osr,
                    ][a],
                    op: [Op::None, Op::Invert, Op::BitReverse, Op::Reserved][(b >> 3) as usize],
                    source: [
                        Source::Pins,
                        Source::X,
                        Source::Y,
                        Source::Null,
                        Source::Reserved,
                        Source::Status,
                        Source::Isr,
                        Source::Osr,
                    ][(b & 7) as usize],
                }
            }
            6 => {
                assert_strict(strict, a < 4, "irq_bit_7");
                Instruction::Irq { clear: bit(1), wait: bit(0), index: u5::new(b) }
            }
            _ => {
                use set::Destn::*;
                Instruction::Set { destn: [Pins, X, Y, Reserved1, PinDirs, Reserved2, Reserved3, Reserved4][a], data: u5::new(b) }
            }
        };

        let side_bits = config.sideset_count.get();
        let field = ((word >> 8) & 31) as u8;
        let delay_bits = 5 - side_bits;
        let side = field >> delay_bits;
        let side_set = match (side_bits, config.sideset_en) {
            (0, _) => None,
            (_, true) if side >> (side_bits - 1) == 0 => None,
            (_, true) => Some(side & !(1 << (side_bits - 1))),
            _ => Some(side),
        };
        Self { instruction, delay: u5::new(field & ((1 << delay_bits) - 1)), side_set: side_set.map(u5::new) }
    }
}

impl fmt::Display for Instruction {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Instruction::Jmp { condition, address } => {
                let cond = ["", "!x, ", "x--, ", "!y, ", "y--, ", "x!=y, ", "pin, ", "!osre, "][*condition as usize];
                write!(f, "jmp {}{}", cond, address.value())
            }
            Instruction::Wait { polarity, source, index } => {
                let src = ["gpio", "pin", "irq", "jmppin"][*source as usize];
                write!(f, "wait {} {} {}", polarity.value(), src, index.value())
            }
            Instruction::In { source, bit_count } => {
                let src = ["pins", "x", "y", "null", "???", "???", "isr", "osr"][*source as usize];
                write!(f, "in {}, {}", src, bit_count.get())
            }
            Instruction::Out { destn, bit_count } => {
                let dst = ["pins", "x", "y", "null", "pindirs", "pc", "isr", "exec"][*destn as usize];
                write!(f, "out {}, {}", dst, bit_count.get())
            }
            Instruction::Push { if_full, block, .. } => {
                let iffull = if if_full.value() == 1 { " iffull" } else { "" };
                let blk = if block.value() == 0 { " noblock" } else { "" };
                write!(f, "push{}{}", iffull, blk)
            }
            Instruction::Pull { if_empty, block } => {
                let ifempty = if if_empty.value() == 1 { " ifempty" } else { "" };
                let blk = if block.value() == 0 { " noblock" } else { "" };
                write!(f, "pull{}{}", ifempty, blk)
            }
            Instruction::MovToRx { index, .. } => match index {
                Some(index) => write!(f, "mov rxfifo[{}], isr", index.value()),
                None => write!(f, "mov rxfifo[y], isr"),
            },
            Instruction::MovFromRx { index, .. } => match index {
                Some(index) => write!(f, "mov osr, rxfifo[{}]", index.value()),
                None => write!(f, "mov osr, rxfifo[y]"),
            },
            Instruction::Mov { destn, op, source } => {
                let dst = ["pins", "x", "y", "pindirs", "exec", "pc", "isr", "osr"][*destn as usize];
                let op = ["", "!", "::", "???"][*op as usize];
                let src = ["pins", "x", "y", "null", "???", "status", "isr", "osr"][*source as usize];
                write!(f, "mov {}, {}{}", dst, op, src)
            }
            Instruction::Irq { clear, wait, index } => {
                if clear.value() == 1 {
                    write!(f, "irq clear {}", index.value())
                } else if wait.value() == 1 {
                    write!(f, "irq wait {}", index.value())
                } else {
                    write!(f, "irq {}", index.value())
                }
            }
            Instruction::Set { destn, data } => {
                let dst = ["pins", "x", "y", "???", "pindirs", "???", "???", "???"][*destn as usize];
                write!(f, "set {}, {}", dst, data.value())
            }
        }
    }
}

impl fmt::Display for Instr {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.instruction)?;
        if let Some(ss) = self.side_set {
            write!(f, " side {}", ss.value())?;
        }
        if self.delay.value() > 0 {
            write!(f, " [{}]", self.delay.value())?;
        }
        Ok(())
    }
}
