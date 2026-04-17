use pioemu::state::{to_mask, wrap_shiftr};

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
