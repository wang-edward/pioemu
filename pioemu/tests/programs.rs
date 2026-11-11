use pioemu::state;

fn setup(program: &[u16], en: [bool; 4]) -> state::Block {
    let mut block = state::Block::new(true);
    block.instr_mem[..program.len()].copy_from_slice(program);
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
    let program = [0xe121, 0xe020, 0x0000];
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
    let program = [0xe023, 0x0041, 0xe05f];
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
    let program = [0x2080, 0xe027];
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
    let program = [0x2003, 0xe025];
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
    let program = [0xe321, 0xe022];
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
    let program = [0xe025, 0xe045, 0x00a0, 0xe03f];
    let mut block = setup(&program, [true, false, false, false]);
    block.step(); // set x, 5
    block.step(); // set y, 5
    block.step(); // jmp x!=y — equal, no jump
    block.step(); // set x, 31
    assert_eq!(sm0(&block).x, 31);
}

#[test]
fn fractional_clock_divider() {
    let mut block = setup(&[0x0040], [true, false, false, false]);
    block.configure(0, 0x0002_8000, 0, 0x000c_0000, 0x1400_0000);
    let ticks: Vec<u32> = (1..=13)
        .filter(|_| {
            let x = block.sms[0].state.x;
            block.step();
            block.sms[0].state.x != x
        })
        .collect();
    assert_eq!(ticks, [1, 3, 5, 8, 10, 13]);
}

#[test]
fn delay_does_not_decode_next_instruction() {
    let mut block = setup(&[0xe721, 0x800f], [true, false, false, false]);
    for _ in 0..8 {
        block.step();
    }
    assert_eq!(sm0(&block).pc.value(), 1);
}
