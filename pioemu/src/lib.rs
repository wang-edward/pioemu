pub mod instr;
pub mod state;

pub fn assert_strict(strict: bool, defined: bool, case: &str) {
    assert!(!strict || defined, "undefined: {case}");
}
