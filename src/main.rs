use arbitrary_int::u5;
use pioemu::instr;
use pioemu::state;

fn main() {
    println!("hi");

    let mut block = state::Block::new();
    block.instr_mem[0] = Some(instr::Instr {
        instruction: instr::Instruction::Set { destn: instr::set::Destn::X, data: u5::new(1) },
        delay: u5::new(1),
        side_set: None,
    });
    block.instr_mem[1] = Some(instr::Instr {
        instruction: instr::Instruction::Set { destn: instr::set::Destn::X, data: u5::new(0) },
        delay: u5::new(0),
        side_set: None,
    });
    block.instr_mem[2] = Some(instr::Instr {
        instruction: instr::Instruction::Jmp { condition: instr::Condition::Always, address: u5::new(0) },
        delay: u5::new(0),
        side_set: None,
    });

    for sm in &mut block.sms {
        sm.enabled = true;
    }

    for _ in 0..8 {
        block.step();
        println!("{}", block);
    }
}
