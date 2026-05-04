use pioemu::state::{calc_irq_index, reverse, to_mask, wrap_shiftr};

#[test]
fn test_to_mask() {
    assert_eq!(to_mask(5), 0x0000_001f);
    assert_eq!(to_mask(15), 0x0000_7fff);
    assert_eq!(to_mask(31), 0x7fff_ffff);
    assert_eq!(to_mask(32), 0xffff_ffff);
}

#[test]
fn test_wrap_shiftr() {
    assert_eq!(wrap_shiftr(0x0000_000f, 4), 0xf000_0000);
    assert_eq!(wrap_shiftr(0x0000_00ff, 4), 0xf000_000f);
}

#[test]
fn test_reverse() {
    assert_eq!(reverse(0x0000_010e), 0x7080_0000);
}

#[test]
fn test_invert() {
    // is ! logical not or bitwise?
    assert_eq!(!0xffff_0000 as u32, 0x0000_ffff as u32);
}

#[test]
fn test_irq_index() {
    assert_eq!(calc_irq_index(0x11, 2), 3);
    assert_eq!(calc_irq_index(0x13, 2), 1);
}
