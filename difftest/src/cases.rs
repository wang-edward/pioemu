use pioemu_protocol::{MAX_WORDS, Request};

pub enum Expect {
    Pass,
    Xfail,
}
use Expect::*;

const BASE: Request = Request {
    ticks: 8,
    execctrl: 0,
    shiftctrl: 0,
    pinctrl: 0,
    origin: 0,
    len: 0,
    code: [0; MAX_WORDS],
    tx_len: 2,
    tx: [0x1111_1111, 0x2222_2222, 0, 0],
    x: 0x0000_00f0,
    y: 0xaaaa_5556,
    isr: 0x0123_4567,
    isr_count: 0,
    osr: 0x89ab_cdef,
    osr_count: 0,
    irq: 0,
    gpio_in: 0x0f0f_00ff,
    rx: [0xa0a0_a0a0, 0xb1b1_b1b1, 0xc2c2_c2c2, 0xd3d3_d3d3],
};

pub fn cases() -> Vec<(&'static str, Expect, Request)> {
    #[rustfmt::skip]
    let table: [(&str, Expect, &[u16], Request); 28] = [
        ("push_rx_put", Xfail, &[0x4028, 0x8000], Request { shiftctrl: 1 << 15, ..BASE }),
        ("push_rx_get", Xfail, &[0x4028, 0x8000], Request { shiftctrl: 1 << 14, ..BASE }),
        ("autopush_rx_put", Xfail, &[0x4028], Request { shiftctrl: 8 << 20 | 1 << 16 | 1 << 15, ..BASE }),
        ("mov_rxfifo_isr_no_join", Xfail, &[0x8018, 0x8010], BASE),
        ("mov_rxfifo_isr_get_only", Xfail, &[0x8018, 0x8010], Request { shiftctrl: 1 << 14, ..BASE }),
        ("mov_osr_rxfifo_no_join", Xfail, &[0x809b, 0x8090], BASE),
        ("mov_osr_rxfifo_put_only", Xfail, &[0x809b, 0x8090], Request { shiftctrl: 1 << 15, ..BASE }),
        ("put_y_index_nonzero", Xfail, &[0x8011], Request { shiftctrl: 1 << 15, ..BASE }),
        ("get_y_index_nonzero", Xfail, &[0x8091], Request { shiftctrl: 1 << 14, ..BASE }),
        ("status_level_above_depth", Pass, &[0xa025], Request { execctrl: 12, shiftctrl: 1 << 30, ..BASE }),
        ("status_sel_3", Pass, &[0xa025], Request { execctrl: 3 << 5 | 15, ..BASE }),
        ("wait_jmppin_index_above_3", Pass, &[0x20e5, 0xe021], Request { execctrl: 4 << 24, ..BASE }),
        ("in_reserved_sources", Pass, &[0x4090, 0x40b0], Request { execctrl: 3, ..BASE }),
        ("mov_reserved_source", Pass, &[0xa024], Request { execctrl: 3, ..BASE }),
        ("mov_reserved_op", Pass, &[0xa03a], BASE),
        ("set_reserved_destinations", Pass, &[0xe07f, 0xe0bf, 0xe0df, 0xe0ff], Request { pinctrl: 5 << 26, ..BASE }),
        ("push_pull_low_bits", Pass, &[0x800f, 0x80af], BASE),
        ("put_bits_6_5", Pass, &[0x8078], Request { shiftctrl: 1 << 15, ..BASE }),
        ("get_bits_6_5", Pass, &[0x80f8, 0x80f8, 0x80f8], Request { shiftctrl: 1 << 14, tx_len: 1, osr: 0, osr_count: 32, ..BASE }),
        ("irq_prev_next", Pass, &[0xc008, 0xc018, 0xa025, 0x20c8], Request { execctrl: 2 << 5 | 8, irq: 0x0f, ..BASE }),
        ("mov_osr_autopull", Pass, &[0x6028, 0xa047], Request { shiftctrl: 16 << 25 | 1 << 17, ..BASE }),
        ("irq_bit_7", Pass, &[0xc081, 0xc0c2, 0xc0a3], Request { irq: 0x04, ..BASE }),
        ("set_count_above_5", Xfail, &[0xa00b, 0xe01f], Request { pinctrl: 7 << 26 | 32 << 20, ..BASE }),
        ("sideset_count_above_5", Xfail, &[0xbfe1, 0xa0e2], Request { pinctrl: 7 << 29, ..BASE }),
        ("exec_delay", Pass, &[0x63e0, 0xa382], Request { ticks: 12, y: 0xe042, osr: 0xe021, ..BASE }),
        ("out_sticky_side_set", Pass, &[0xf000, 0xb042], Request { execctrl: 1 << 17, pinctrl: 1 << 29 | 1 << 26, ..BASE }),
        ("pull_noblock_empty", Pass, &[0x8080], Request { tx_len: 0, ..BASE }),
        ("autopull_overshoot", Pass, &[0x6025, 0x6025, 0x6025], Request { shiftctrl: 8 << 25 | 1 << 17, ..BASE }),
    ];
    table
        .into_iter()
        .map(|(name, expect, program, request)| {
            let mut code = [0; MAX_WORDS];
            code[..program.len()].copy_from_slice(program);
            let execctrl = request.execctrl | (program.len() as u32 - 1) << 12;
            (name, expect, Request { len: program.len(), code, execctrl, ..request })
        })
        .collect()
}
