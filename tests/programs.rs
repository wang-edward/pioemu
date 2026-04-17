use arbitrary_int::u5;
use pioemu::pio;
use pioemu::state;

#[test]
fn squarewave() {
    let mut block = state::Block::new();
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

    for (i, instr) in program.iter().enumerate() {
        block.instr_mem[i] = Some(*instr);
    }

    for sm in &mut block.state_machines {
        sm.enabled = true;
    }

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
