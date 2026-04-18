use arbitrary_int::{u1, u5};
use pioemu::instr::{self, Condition::*, Instruction::*, set::Destn as SetDestn, wait::Source as WaitSource};
use pioemu::state;

macro_rules! pio {
    // === set with delay ===
    (set $destn:tt, $val:literal [$delay:literal]) => {{
        let mut i = pio!(set $destn, $val);
        i.delay = u5::new($delay);
        i
    }};
    // === jmp with delay ===
    (jmp $addr:literal [$delay:literal]) => {{
        let mut i = pio!(jmp $addr);
        i.delay = u5::new($delay);
        i
    }};
    (jmp !x, $addr:literal [$delay:literal]) => {{ let mut i = pio!(jmp !x, $addr); i.delay = u5::new($delay); i }};
    (jmp !y, $addr:literal [$delay:literal]) => {{ let mut i = pio!(jmp !y, $addr); i.delay = u5::new($delay); i }};
    (jmp x--, $addr:literal [$delay:literal]) => {{ let mut i = pio!(jmp x--, $addr); i.delay = u5::new($delay); i }};
    (jmp y--, $addr:literal [$delay:literal]) => {{ let mut i = pio!(jmp y--, $addr); i.delay = u5::new($delay); i }};
    (jmp x!=y, $addr:literal [$delay:literal]) => {{ let mut i = pio!(jmp x!=y, $addr); i.delay = u5::new($delay); i }};
    (jmp pin, $addr:literal [$delay:literal]) => {{ let mut i = pio!(jmp pin, $addr); i.delay = u5::new($delay); i }};
    (jmp !osre, $addr:literal [$delay:literal]) => {{ let mut i = pio!(jmp !osre, $addr); i.delay = u5::new($delay); i }};
    // === wait with delay ===
    (wait $pol:literal, $src:tt $idx:literal [$delay:literal]) => {{
        let mut i = pio!(wait $pol, $src $idx);
        i.delay = u5::new($delay);
        i
    }};

    // === set ===
    (set x, $val:literal) => {
        instr::Instr { instruction: Set { destn: SetDestn::X, data: u5::new($val) }, delay: u5::new(0), side_set: None }
    };
    (set y, $val:literal) => {
        instr::Instr { instruction: Set { destn: SetDestn::Y, data: u5::new($val) }, delay: u5::new(0), side_set: None }
    };
    (set pins, $val:literal) => {
        instr::Instr { instruction: Set { destn: SetDestn::Pins, data: u5::new($val) }, delay: u5::new(0), side_set: None }
    };
    (set pindirs, $val:literal) => {
        instr::Instr { instruction: Set { destn: SetDestn::PinDirs, data: u5::new($val) }, delay: u5::new(0), side_set: None }
    };

    // === jmp ===
    (jmp $addr:literal) => {
        instr::Instr { instruction: Jmp { condition: Always, address: u5::new($addr) }, delay: u5::new(0), side_set: None }
    };
    (jmp !x, $addr:literal) => {
        instr::Instr { instruction: Jmp { condition: XZero, address: u5::new($addr) }, delay: u5::new(0), side_set: None }
    };
    (jmp !y, $addr:literal) => {
        instr::Instr { instruction: Jmp { condition: YZero, address: u5::new($addr) }, delay: u5::new(0), side_set: None }
    };
    (jmp x--, $addr:literal) => {
        instr::Instr { instruction: Jmp { condition: XDec, address: u5::new($addr) }, delay: u5::new(0), side_set: None }
    };
    (jmp y--, $addr:literal) => {
        instr::Instr { instruction: Jmp { condition: YDec, address: u5::new($addr) }, delay: u5::new(0), side_set: None }
    };
    (jmp x!=y, $addr:literal) => {
        instr::Instr { instruction: Jmp { condition: XNeqY, address: u5::new($addr) }, delay: u5::new(0), side_set: None }
    };
    (jmp pin, $addr:literal) => {
        instr::Instr { instruction: Jmp { condition: Pin, address: u5::new($addr) }, delay: u5::new(0), side_set: None }
    };
    (jmp !osre, $addr:literal) => {
        instr::Instr { instruction: Jmp { condition: OsrNotEmpty, address: u5::new($addr) }, delay: u5::new(0), side_set: None }
    };

    // === wait ===
    (wait $pol:literal, gpio $idx:literal) => {
        instr::Instr { instruction: Wait { polarity: u1::new($pol), source: WaitSource::Gpio, index: u5::new($idx) }, delay: u5::new(0), side_set: None }
    };
    (wait $pol:literal, pin $idx:literal) => {
        instr::Instr { instruction: Wait { polarity: u1::new($pol), source: WaitSource::Pin, index: u5::new($idx) }, delay: u5::new(0), side_set: None }
    };
    (wait $pol:literal, irq $idx:literal) => {
        instr::Instr { instruction: Wait { polarity: u1::new($pol), source: WaitSource::Irq, index: u5::new($idx) }, delay: u5::new(0), side_set: None }
    };
}

fn setup(program: &[instr::Instr], en: [bool; 4]) -> state::Block {
    let mut block = state::Block::new();
    for (i, instr) in program.iter().enumerate() {
        block.instr_mem[i] = Some(*instr);
    }
    for (i, sm) in block.sms.iter_mut().enumerate() {
        sm.enabled = en[i];
    }
    block
}

fn sm0(block: &state::Block) -> &state::State {
    &block.sms[0].state
}

#[test]
fn squarewave() {
    let program = [pio!(set x, 1 [1]), pio!(set x, 0), pio!(jmp 0)];
    let mut block = setup(&program, [true, true, true, true]);
    let mut xs: Vec<[u32; 4]> = Vec::new();
    let expected = vec![
        [1, 1, 1, 1],
        [1, 1, 1, 1],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [1, 1, 1, 1],
        [1, 1, 1, 1],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
    ];
    for _ in 0..8 {
        block.step();
        let x: [u32; 4] = std::array::from_fn(|i| block.sms[i].state.x);
        xs.push(x);
    }
    assert_eq!(xs, expected);
}

#[test]
fn countdown() {
    let program = [pio!(set x, 3), pio!(jmp x--, 1), pio!(set y, 31)];
    let mut block = setup(&program, [true, false, false, false]);
    for _ in 0..5 {
        block.step();
    }
    assert_eq!(sm0(&block).x, u32::MAX);
    assert_eq!(sm0(&block).pc.value(), 2);
    block.step();
    assert_eq!(sm0(&block).y, 31);
}

#[test]
fn wait_gpio() {
    let program = [pio!(wait 1, gpio 0), pio!(set x, 7)];
    let mut block = setup(&program, [true, false, false, false]);
    block.gpio_in = 0;
    for _ in 0..5 {
        block.step();
    }
    assert_eq!(sm0(&block).pc.value(), 0);
    assert_eq!(sm0(&block).x, 0);

    block.gpio_in = 1;
    block.step();
    assert_eq!(sm0(&block).pc.value(), 1);
    block.step();
    assert_eq!(sm0(&block).x, 7);
}

#[test]
fn wait_zero() {
    let program = [pio!(wait 0, gpio 3), pio!(set x, 5)];
    let mut block = setup(&program, [true, false, false, false]);
    block.gpio_in = 0b1000;
    for _ in 0..3 {
        block.step();
    }
    assert_eq!(sm0(&block).pc.value(), 0);

    block.gpio_in = 0;
    block.step();
    assert_eq!(sm0(&block).pc.value(), 1);
}

#[test]
fn delay_cycles() {
    let program = [pio!(set x, 1 [3]), pio!(set x, 2)];
    let mut block = setup(&program, [true, false, false, false]);
    block.step();
    assert_eq!(sm0(&block).x, 1);
    for _ in 0..3 {
        block.step();
        assert_eq!(sm0(&block).x, 1);
    }
    block.step();
    assert_eq!(sm0(&block).x, 2);
}

#[test]
fn jmp_x_eq_y() {
    let program = [pio!(set x, 5), pio!(set y, 5), pio!(jmp x!=y, 0), pio!(set x, 31)];
    let mut block = setup(&program, [true, false, false, false]);
    block.step(); // set x, 5
    block.step(); // set y, 5
    block.step(); // jmp x!=y — equal, no jump
    block.step(); // set x, 31
    assert_eq!(sm0(&block).x, 31);
}
