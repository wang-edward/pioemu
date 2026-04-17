mod pio;
mod state;
use arbitrary_int::u5;

fn main() {
    println!("hi");

    let mut block = state::Block::new();
    block.instr_mem[0] = Some(pio::Instr {
        instruction: pio::Instruction::Set { destn: pio::set::Destn::X, data: u5::new(1) },
        delay: u5::new(1),
        side_set: None,
    });
    block.instr_mem[1] = Some(pio::Instr {
        instruction: pio::Instruction::Set { destn: pio::set::Destn::X, data: u5::new(0) },
        delay: u5::new(0),
        side_set: None,
    });
    block.instr_mem[2] = Some(pio::Instr {
        instruction: pio::Instruction::Jmp { condition: pio::Condition::Always, address: u5::new(0) },
        delay: u5::new(0),
        side_set: None,
    });

    for sm in &mut block.state_machines {
        sm.enabled = true;
    }

    for _ in 0..8 {
        block.step();
        println!("{}", block);
    }
}
