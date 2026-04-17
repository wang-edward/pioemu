use arbitrary_int::{u1, u5};

#[rustfmt::skip]
enum Condition { Always, XZero, XDec, YZero, YDec, XNeqY, Pin, OsrNotEmpty, }

#[rustfmt::skip]
mod wait {
    pub enum Source { Gpio, Pin, Irq, Reserved }
}

#[rustfmt::skip]
mod shift {
    pub enum Source { Pins, X, Y, Null, Reserved1, Reserved2, Isr, Osr }
    pub enum Destn { Pins, X, Y, Null, PinDirs, Pc, Isr, Exec }
}

#[rustfmt::skip]
mod mov {
    pub enum Destn { Pins, X, Y, Reserved, Exec, Pc, Isr, Osr }
    pub enum Op { None, Invert, BitReverse, Reserved }
    pub enum Source { Pins, X, Y, Null, Reserved, Status, Isr, Osr }
}

#[rustfmt::skip]
mod set {
    pub enum Destn { Pins, X, Y, Reserved1, PinDirs, Reserved2, Reserved3, Reserved4 }
}

enum Instruction {
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

pub struct Instr {
    instruction: Instruction,
    delay: u5,
}
