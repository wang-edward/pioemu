use arbitrary_int::u5;
use pioemu::pio;
use pioemu::state;

fn setup(program: &[pio::Instr], en: [bool; 4]) -> state::Block {
    let mut block = state::Block::new();
    for (i, instr) in program.iter().enumerate() {
        block.instr_mem[i] = Some(*instr);
    }
    for (i, sm) in block.state_machines.iter_mut().enumerate() {
        sm.enabled = en[i];
    }
    block
}

#[test]
fn squarewave() {
    let n = 8;
    let program = [
        pio::Instr {
            instruction: pio::Instruction::Set { destn: pio::set::Destn::X, data: u5::new(1) },
            delay: u5::new(1),
            side_set: None,
        },
        pio::Instr {
            instruction: pio::Instruction::Set { destn: pio::set::Destn::X, data: u5::new(0) },
            delay: u5::new(0),
            side_set: None,
        },
        pio::Instr {
            instruction: pio::Instruction::Jmp { condition: pio::Condition::Always, address: u5::new(0) },
            delay: u5::new(0),
            side_set: None,
        },
    ];

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

    for _ in 0..n {
        block.step();
        let x: [u32; 4] = std::array::from_fn(|i| block.state_machines[i].state.x);
        xs.push(x);
    }

    assert_eq!(xs, expected);
}
