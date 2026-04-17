mod pio;
mod state;
use arbitrary_int::u5;

fn main() {
    println!("hi");

    let mut block = state::Block::new();
    block.instr_mem[0] = Some(pio::Instr {
        instruction: pio::Instruction::Jmp {
            condition: pio::Condition::Always, // capital A, named field
            address: u5::new(0),               // u5, not bare int
        },
        delay: u5::new(0),
        side_set: None,
    });

    println!("{:?}", block);
}
