use pioemu_protocol::{Request, Sm};

#[allow(dead_code)]
pub enum Kind {
    Undefined(&'static str),
    Contradicts(&'static str),
    Undocumented,
}
use Kind::*;

pub enum Test {
    Pass(Request),
    Untestable(&'static str),
}
use Test::*;

const BASE: Sm = Sm {
    clkdiv: 1 << 16,
    execctrl: 0,
    shiftctrl: 0,
    pinctrl: 0,
    pc: 0,
    tx_len: 2,
    tx: [0x1111_1111, 0x2222_2222, 0, 0],
    x: 0x0000_00f0,
    y: 0xaaaa_5556,
    isr: 0x0123_4567,
    isr_count: 0,
    osr: 0x89ab_cdef,
    osr_count: 0,
    rx: [0xa0a0_a0a0, 0xb1b1_b1b1, 0xc2c2_c2c2, 0xd3d3_d3d3],
};

fn program(words: &[u16], sm: Sm) -> Request {
    let mut code = [0; 32];
    code[..words.len()].copy_from_slice(words);
    let execctrl = sm.execctrl | (words.len() as u32 - 1) << 12;
    Request { ticks: 8, code, irq: 0, gpio_in: 0x0f0f_00ff, sms: [Some(Sm { execctrl, ..sm }), None, None, None] }
}

pub fn cases() -> Vec<(&'static str, &'static str, Kind, Test)> {
    const PUSH: &str = "The operation of the PUSH instruction is undefined when SM0_SHIFTCTRL.FJOIN_RX_PUT or F JOIN_RX_GET is set";
    const AUTOPUSH: &str = "Autopush must not be enabled when SHIFTCTRL_FJOIN_RX_PUT or SHIFTCTRL_FJOIN_RX_GET is set. Its operation in this state is undefined.";
    const PUT: &str = "Requires the SHIFTCTRL_FJOIN_RX_PUT configuration field to be set, otherwise its operation is undefined.";
    const GET: &str = "Requires the SHIFTCTRL_FJOIN_RX_GET configuration field to be set, otherwise its operation is undefined.";
    const INDEX: &str = "When IdxI is clear, all non-zero values of Index are reserved encodings, and their operation is undefined.";
    #[rustfmt::skip]
    let cases = vec![
        ("push_rx_put", "11.4.6.2", Undefined(PUSH), Pass(program(&[0x4028, 0x8000], Sm { shiftctrl: 1 << 15, ..BASE }))),
        ("push_rx_get", "11.4.6.2", Undefined(PUSH), Pass(program(&[0x4028, 0x8000], Sm { shiftctrl: 1 << 14, ..BASE }))),
        ("autopush_rx_put", "11.5.4.1", Undefined(AUTOPUSH), Pass(program(&[0x4028], Sm { shiftctrl: 8 << 20 | 1 << 16 | 1 << 15, ..BASE }))),
        ("autopush_rx_get", "11.5.4.1", Undefined(AUTOPUSH), Pass(program(&[0x4028], Sm { shiftctrl: 8 << 20 | 1 << 16 | 1 << 14, ..BASE }))),
        ("mov_rxfifo_isr_no_join", "11.4.8.2", Undefined(PUT), Pass(program(&[0x8018, 0x8010], BASE))),
        ("mov_rxfifo_isr_get_only", "11.4.8.2", Undefined(PUT), Pass(program(&[0x8018, 0x8010], Sm { shiftctrl: 1 << 14, ..BASE }))),
        ("mov_osr_rxfifo_no_join", "11.4.9.2", Undefined(GET), Pass(program(&[0x809b, 0x8090], BASE))),
        ("mov_osr_rxfifo_put_only", "11.4.9.2", Undefined(GET), Pass(program(&[0x809b, 0x8090], Sm { shiftctrl: 1 << 15, ..BASE }))),
        ("mov_osr_rxfifo_put_only_bus", "11.4.9.2", Undefined(GET), Untestable("with only FJOIN_RX_PUT set it reads the RX entry picked by the address of whatever system bus access reaches the PIO on that cycle; the firmware keeps the bus idle while the state machines run, which picks entry 2")),
        ("mov_osr_rxfifo_tx_joined", "11.4.9.2", Undefined(GET), Pass(Request { ticks: 2, ..program(&[0x8098, 0x8098], Sm { shiftctrl: 1 << 30, ..BASE }) })),
        ("mov_osr_rxfifo_tx_joined_empty", "11.4.9.2", Undefined(GET), Untestable("with FJOIN_TX set and the TX FIFO empty it reads the storage entry at the TX FIFO pointer, which the firmware moves while it loads the registers, so random programs leave it out")),
        ("put_y_index_nonzero", "11.4.8.2", Undefined(INDEX), Pass(program(&[0x8011], Sm { shiftctrl: 1 << 15, ..BASE }))),
        ("get_y_index_nonzero", "11.4.9.2", Undefined(INDEX), Pass(program(&[0x8091], Sm { shiftctrl: 1 << 14, ..BASE }))),
        ("status_level_above_depth", "11.7", Undefined("If STATUS_SEL is TXLEVEL or RXLEVEL, then values of STATUS_N greater than the current FIFO depth are reserved, and have undefined behaviour."), Pass(program(&[0xa025], Sm { execctrl: 12, shiftctrl: 1 << 30, ..BASE }))),
        ("status_sel_3", "11.7", Undocumented, Pass(program(&[0xa025], Sm { execctrl: 3 << 5 | 15, ..BASE }))),
        ("status_irq_index_above_23", "11.7", Undocumented, Pass(Request { irq: 0xff, ..program(&[0xa025], Sm { execctrl: 2 << 5 | 25, ..BASE }) })),
        ("wait_jmppin_index_above_3", "11.4.3.2", Undefined("plus an Index in the range 0-3, all modulo 32. Other values of Index are reserved."), Pass(program(&[0x20e5, 0xe021], Sm { execctrl: 4 << 24, ..BASE }))),
        ("in_reserved_sources", "11.4.4.2", Undefined("◦ 100: Reserved ◦ 101: Reserved ◦ 110: ISR"), Pass(program(&[0x4090, 0x40b0], Sm { execctrl: 3, ..BASE }))),
        ("mov_reserved_source", "11.4.10.2", Undefined("◦ 100: Reserved ◦ 101: STATUS"), Pass(program(&[0xa024], Sm { execctrl: 3, ..BASE }))),
        ("mov_reserved_op", "11.4.10.2", Undefined("◦ 10: Bit-reverse ◦ 11: Reserved"), Pass(program(&[0xa03a], BASE))),
        ("set_reserved_destinations", "11.4.12.2", Undefined("◦ 011: Reserved ◦ 100: PINDIRS ◦ 101: Reserved ◦ 110: Reserved ◦ 111: Reserved"), Pass(program(&[0xe07f, 0xe0bf, 0xe0df, 0xe0ff], Sm { pinctrl: 5 << 26, ..BASE }))),
        ("push_low_bits", "11.4.6.1", Undefined("PUSH 1 0 0 Delay/side-set 0 IfF Blk 0 0 0 0 0"), Pass(program(&[0x800f], BASE))),
        ("pull_low_bits", "11.4.7.1", Undefined("PULL 1 0 0 Delay/side-set 1 IfE Blk 0 0 0 0 0"), Pass(program(&[0x80af], BASE))),
        ("put_bits_6_5", "11.4.8.1", Undefined("MOV 1 0 0 Delay/side-set 0 0 0 1 IdxI Index"), Pass(program(&[0x8078], Sm { shiftctrl: 1 << 15, ..BASE }))),
        ("get_bits_6_5", "11.4.9.1", Undefined("MOV 1 0 0 Delay/side-set 1 0 0 1 IdxI Index"), Pass(program(&[0x80f8, 0x80f8, 0x80f8], Sm { shiftctrl: 1 << 14, tx_len: 1, osr: 0, osr_count: 32, ..BASE }))),
        ("irq_bit_7", "11.4.11.1", Undefined("IRQ 1 1 0 Delay/side-set 0 Clr Wait IdxMode Index"), Pass(Request { irq: 0x04, ..program(&[0xc081, 0xc0c2, 0xc0a3], BASE) })),
        ("irq_prev_next", "11.4.11.2", Undocumented, Pass(Request { irq: 0x0f, ..program(&[0xc008, 0xa025, 0xc038, 0x20c8], Sm { execctrl: 2 << 5 | 8, ..BASE }) })),
        ("mov_osr_autopull", "11.5.4.2", Undefined("Note that a MOV from the OSR is undefined whilst autopull is enabled"), Pass(program(&[0x6028, 0xa047], Sm { shiftctrl: 16 << 25 | 1 << 17, ..BASE }))),
        ("set_count_above_5", "11.7", Undefined("SET_COUNT: The number of pins asserted by a SET. In the range 0 to 5 inclusive."), Pass(program(&[0xa001, 0xe080], Sm { pinctrl: 7 << 26, ..BASE }))),
        ("sideset_count_above_5", "11.7", Undefined("Minimum of 0 (all delay bits, no side-set) and maximum of 5 (all side-set, no delay)."), Pass(program(&[0xe01f, 0xbf42], Sm { pinctrl: 7 << 29 | 5 << 26, ..BASE }))),
        ("sideset_en_count_0", "11.7", Undocumented, Pass(Request { ticks: 48, ..program(&[0xa00b, 0xa06b, 0xf121, 0xe122, 0xf023, 0xe024], Sm { execctrl: 1 << 30, pinctrl: 32 << 20 | 4 << 10, ..BASE }) })),
        ("clkdiv_int_0_frac_nonzero", "11.7", Undefined("If INT is 0, FRAC must also be 0."), Untestable("one tick takes at least 65536 system clocks, beyond what the firmware's nop routine timing can reach")),
        ("out_count_above_32", "11.7", Undefined("OUT_COUNT: The number of pins asserted by an OUT PINS, OUT PINDIRS or MOV PINS instruction. In the range 0 to 32 inclusive."), Pass(program(&[0x6000, 0xa06b], Sm { pinctrl: 40 << 20 | 4, ..BASE }))),
        ("fjoin_with_rx_putget", "11.7", Undocumented, Pass(program(&[0x80a0, 0x80a0], Sm { shiftctrl: 1 << 31 | 1 << 15, ..BASE }))),
        ("out_exec_delay", "11.4.5.2", Contradicts("Delay cycles on the initial OUT are ignored, but the executee may insert delay cycles as normal."), Pass(program(&[0x63e0], Sm { osr: 0xe021, ..BASE }))),
        ("mov_exec_delay", "11.4.10.2", Contradicts("Delay cycles on MOV EXEC are ignored, but the executee may insert delay cycles as normal."), Pass(program(&[0xa382], Sm { y: 0xe040, ..BASE }))),
        ("autopull_overshoot", "11.5.4.2", Contradicts("osr count = saturate(osr count + out count) if osr count >= threshold: if tx fifo not empty: osr = pull()"), Pass(program(&[0x6025, 0x6025, 0x6025], Sm { shiftctrl: 8 << 25 | 1 << 17, ..BASE }))),
        ("out_sticky_side_set", "11.5.6", Contradicts("If a side-set overlaps with an OUT/SET performed by that state machine on the same cycle, the side-set takes precedence in the overlapping region."), Pass(program(&[0xf000, 0xb042], Sm { execctrl: 1 << 17, pinctrl: 1 << 29 | 1 << 26, ..BASE }))),
        ("out_sticky_beats_higher_sm", "11.5.6.1", Contradicts("For each GPIO, PIO collates the writes from all four state machines, and applies the write from the highest-numbered state machine."), Pass(Request { sms: [Some(Sm { execctrl: 1 << 17, pinctrl: 32 << 20, ..BASE }), Some(Sm { execctrl: 1 << 12 | 1 << 7, pinctrl: 5 << 26, pc: 1, ..BASE }), None, None], ..program(&[0xa06b, 0xe080], BASE) })),
        ("out_sticky_release", "11.5.6.1", Contradicts("If no state machine asserts a write to a GPIO’s level or direction, the value does not change."), Pass(program(&[0xe09f, 0xb042, 0xe000], Sm { execctrl: 1 << 30 | 1 << 29 | 1 << 17, pinctrl: 2 << 29 | 5 << 26, ..BASE }))),
        ("pull_noblock_empty", "11.7", Contradicts("TXSTALL: State machine has stalled on empty TX FIFO during a blocking PULL, or an OUT with autopull enabled."), Pass(program(&[0x8080], Sm { tx_len: 0, ..BASE }))),
        ("metastable_inputs", "11.5.6.3", Undefined("Sampling a metastable input can lead to unpredictable state machine behaviour."), Untestable("the firmware forces every input with INOVER, so no input is ever metastable")),
        ("stalling_instr_with_exec", "11.5.7", Undefined("If EXEC instructions are used, instructions written to INSTR must not stall."), Untestable("a request has no way to write SMx_INSTR during the run")),
        ("wait_irq_with_interrupts", "11.4.3.2", Undefined("WAIT 1 IRQ x should not be used with IRQ flags presented to the interrupt controller"), Untestable("the firmware never routes IRQ flags to the interrupt controller")),
        ("rxf_read_empty", "11.7", Undefined("The data returned to the system on a read from an empty FIFO is undefined."), Untestable("the firmware reads RXF0 only while the RX FIFO is not empty")),
        ("rx_storage_survives_reset", "11.7", Undocumented, Untestable("RESETS does not clear the RX FIFO storage, so the firmware loads it with the request's RX words after every reset")),
        ("putget_read_get_only", "11.7", Contradicts("Direct read/write access to entry 0 of SM0's RX FIFO, if SHIFTCTRL_F JOIN_RX_PUT xor SHIFTCTRL_F JOIN_RX_GET is set."), Untestable("in GET-only mode all four RXF0_PUTGET registers read the same stored word, so the firmware switches to PUT-only to read the storage; the emulator does not model system reads")),
    ];
    cases
}
