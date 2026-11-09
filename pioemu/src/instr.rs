use arbitrary_int::{u1, u5};
use std::fmt;

#[derive(Clone, Copy, Debug)]
#[rustfmt::skip]
pub enum Condition { Always, XZero, XDec, YZero, YDec, XNeqY, Pin, OsrNotEmpty }

#[rustfmt::skip]
pub mod wait {
    #[derive(Clone, Copy, Debug)]
    pub enum Source { Gpio, Pin, Irq, Reserved }
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
        address: u5, // TODO use bool and range for instructions instead of u5
    },
    Wait {
        polarity: u1,
        source: wait::Source,
        index: u5,
    },
    In {
        source: shift::Source,
        bit_count: u5, // 0x00 = 32, not 0
    },
    Out {
        destn: shift::Destn,
        bit_count: u5, // 0x00 = 32, not 0
    },
    Push {
        if_full: u1,
        block: u1,
    },
    Pull {
        if_empty: u1,
        block: u1,
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
    // TODO something about delay and side_set sharing bits
}

impl Instr {
    pub fn decode(word: u16, side_bits: u8, side_optional: bool) -> Self {
        let (a, b) = (((word >> 5) & 7) as usize, (word & 31) as u8);
        let bit = |i: usize| u1::new(((a >> i) & 1) as u8);
        let instruction = match word >> 13 {
            0 => {
                use Condition::*;
                Instruction::Jmp { condition: [Always, XZero, XDec, YZero, YDec, XNeqY, Pin, OsrNotEmpty][a], address: u5::new(b) }
            }
            1 => {
                use wait::Source::*;
                Instruction::Wait { polarity: bit(2), source: [Gpio, Pin, Irq, Reserved][a & 3], index: u5::new(b) }
            }
            2 => {
                use shift::Source::*;
                Instruction::In { source: [Pins, X, Y, Null, Reserved1, Reserved2, Isr, Osr][a], bit_count: u5::new(b) }
            }
            3 => {
                use shift::Destn::*;
                Instruction::Out { destn: [Pins, X, Y, Null, PinDirs, Pc, Isr, Exec][a], bit_count: u5::new(b) }
            }
            4 => {
                assert!(b == 0, "unsupported PUSH/PULL encoding {word:04x}");
                if a & 4 == 0 {
                    Instruction::Push { if_full: bit(1), block: bit(0) }
                } else {
                    Instruction::Pull { if_empty: bit(1), block: bit(0) }
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
            6 => Instruction::Irq { clear: bit(1), wait: bit(0), index: u5::new(b) },
            _ => {
                use set::Destn::*;
                Instruction::Set { destn: [Pins, X, Y, Reserved1, PinDirs, Reserved2, Reserved3, Reserved4][a], data: u5::new(b) }
            }
        };

        let field = ((word >> 8) & 31) as u8;
        let delay_bits = 5 - side_bits;
        let side = field >> delay_bits;
        let side_set = match (side_bits, side_optional) {
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
            Instruction::Jmp { condition, address } => match condition {
                Condition::Always => write!(f, "jmp {}", address.value()),
                Condition::XZero => write!(f, "jmp !x, {}", address.value()),
                Condition::XDec => write!(f, "jmp x--, {}", address.value()),
                Condition::YZero => write!(f, "jmp !y, {}", address.value()),
                Condition::YDec => write!(f, "jmp y--, {}", address.value()),
                Condition::XNeqY => write!(f, "jmp x!=y, {}", address.value()),
                Condition::Pin => write!(f, "jmp pin, {}", address.value()),
                Condition::OsrNotEmpty => write!(f, "jmp !osre, {}", address.value()),
            },
            Instruction::Wait { polarity, source, index } => {
                let src = match source {
                    wait::Source::Gpio => "gpio",
                    wait::Source::Pin => "pin",
                    wait::Source::Irq => "irq",
                    wait::Source::Reserved => "???",
                };
                write!(f, "wait {} {} {}", polarity.value(), src, index.value())
            }
            Instruction::In { source, bit_count } => {
                let src = match source {
                    shift::Source::Pins => "pins",
                    shift::Source::X => "x",
                    shift::Source::Y => "y",
                    shift::Source::Null => "null",
                    shift::Source::Isr => "isr",
                    shift::Source::Osr => "osr",
                    _ => "???",
                };
                let n = if bit_count.value() == 0 { 32 } else { bit_count.value() as u32 };
                write!(f, "in {}, {}", src, n)
            }
            Instruction::Out { destn, bit_count } => {
                let dst = match destn {
                    shift::Destn::Pins => "pins",
                    shift::Destn::X => "x",
                    shift::Destn::Y => "y",
                    shift::Destn::Null => "null",
                    shift::Destn::PinDirs => "pindirs",
                    shift::Destn::Pc => "pc",
                    shift::Destn::Isr => "isr",
                    shift::Destn::Exec => "exec",
                };
                let n = if bit_count.value() == 0 { 32 } else { bit_count.value() as u32 };
                write!(f, "out {}, {}", dst, n)
            }
            Instruction::Push { if_full, block } => {
                let iffull = if if_full.value() == 1 { " iffull" } else { "" };
                let blk = if block.value() == 0 { " noblock" } else { "" };
                write!(f, "push{}{}", iffull, blk)
            }
            Instruction::Pull { if_empty, block } => {
                let ifempty = if if_empty.value() == 1 { " ifempty" } else { "" };
                let blk = if block.value() == 0 { " noblock" } else { "" };
                write!(f, "pull{}{}", ifempty, blk)
            }
            Instruction::Mov { destn, op, source } => {
                let dst = match destn {
                    mov::Destn::Pins => "pins",
                    mov::Destn::X => "x",
                    mov::Destn::Y => "y",
                    mov::Destn::PinDirs => "pindirs",
                    mov::Destn::Exec => "exec",
                    mov::Destn::Pc => "pc",
                    mov::Destn::Isr => "isr",
                    mov::Destn::Osr => "osr",
                };
                let src = match source {
                    mov::Source::Pins => "pins",
                    mov::Source::X => "x",
                    mov::Source::Y => "y",
                    mov::Source::Null => "null",
                    mov::Source::Status => "status",
                    mov::Source::Isr => "isr",
                    mov::Source::Osr => "osr",
                    _ => "???",
                };
                let op_str = match op {
                    mov::Op::None => "",
                    mov::Op::Invert => "!",
                    mov::Op::BitReverse => "::",
                    _ => "???",
                };
                write!(f, "mov {}, {}{}", dst, op_str, src)
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
                let dst = match destn {
                    set::Destn::Pins => "pins",
                    set::Destn::X => "x",
                    set::Destn::Y => "y",
                    set::Destn::PinDirs => "pindirs",
                    _ => "???",
                };
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
