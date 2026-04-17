use arbitrary_int::{u1, u5};

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
    pub enum Destn { Pins, X, Y, Reserved, Exec, Pc, Isr, Osr }
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
