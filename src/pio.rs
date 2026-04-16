use arbitrary_int::{u1, u5};

enum Condition {
    Always,
    XZero,
    XDec,
    YZero,
    YDec,
    XNeqY,
    Pin,
    OsrNotEmpty,
}

enum WaitSource {
    Gpio,
    Pin,
    Irq,
    Reserved,
}

enum ShiftSource {
    Pins,
    X,
    Y,
    Null,
    Reserved1,
    Reserved2,
    Isr,
    Osr,
}

enum ShiftDestn {
    Pins,
    X,
    Y,
    Null,
    PinDirs,
    Pc,
    Isr,
    Exec,
}

enum MovDestn {
    Pins,
    X,
    Y,
    Reserved,
    Exec,
    Pc,
    Isr,
    Osr,
}

enum MovOp {
    None,
    Invert,
    BitReverse,
    Reserved,
}

enum MovSource {
    Pins,
    X,
    Y,
    Null,
    Reserved,
    Status,
    Isr,
    Osr,
}

enum Instruction {
    Jmp {
        condition: Condition,
    },
    Wait {
        polarity: u1,
        source: WaitSource,
        index: u5,
    },
    In {
        source: ShiftSource,
        bit_count: u5, // 0x00 = 32, not 0
    },
    Out {
        destn: ShiftDestn,
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
    Mov {},
}

struct Instr {
    instruction: Instruction,
    delay: u5,
}
