use pioemu::state::{bit_at, calc_irq_index, to_mask};

#[test]
fn test_to_mask() {
    assert_eq!(to_mask(5), 0x0000_001f);
    assert_eq!(to_mask(15), 0x0000_7fff);
    assert_eq!(to_mask(31), 0x7fff_ffff);
    assert_eq!(to_mask(32), 0xffff_ffff);
}

#[test]
fn test_irq_index() {
    assert_eq!(calc_irq_index(0x11, 2), 3);
    assert_eq!(calc_irq_index(0x13, 2), 1);
}

#[test]
fn test_bit_at() {
    assert_eq!(bit_at(0x11, 2), false);
    assert_eq!(bit_at(0x11, 0), true);
    assert_eq!(bit_at(0x11, 4), true);
}
