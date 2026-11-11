#![no_std]

pub const MAX_TICKS: u32 = 1000;
pub const MAX_REQUEST: usize = 1024;
pub const MAX_TX: usize = 4;
pub const CSV_HEADER: &str = "ticks_elapsed,padout,padoe,irq,flevel,fdebug,sm0_pc,sm0_rx_count,sm0_rx0,sm0_rx1,sm0_rx2,sm0_rx3,sm0_x,sm0_y,sm0_isr,sm0_osr,sm1_pc,sm1_rx_count,sm1_rx0,sm1_rx1,sm1_rx2,sm1_rx3,sm1_x,sm1_y,sm1_isr,sm1_osr,sm2_pc,sm2_rx_count,sm2_rx0,sm2_rx1,sm2_rx2,sm2_rx3,sm2_x,sm2_y,sm2_isr,sm2_osr,sm3_pc,sm3_rx_count,sm3_rx0,sm3_rx1,sm3_rx2,sm3_rx3,sm3_x,sm3_y,sm3_isr,sm3_osr";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Sm {
    pub clkdiv: u32,
    pub execctrl: u32,
    pub shiftctrl: u32,
    pub pinctrl: u32,
    pub pc: u8,
    pub tx_len: usize,
    pub tx: [u32; MAX_TX],
    pub x: u32,
    pub y: u32,
    pub isr: u32,
    pub isr_count: u8,
    pub osr: u32,
    pub osr_count: u8,
    pub rx: [u32; 4],
}

#[derive(Debug, PartialEq, Eq)]
pub struct Request {
    pub ticks: u32,
    pub code: [u16; 32],
    pub irq: u8,
    pub gpio_in: u32,
    pub sms: [Option<Sm>; 4],
}

impl Sm {
    pub fn side_bits(&self) -> u8 {
        (self.pinctrl >> 29) as u8
    }

    pub fn side_optional(&self) -> bool {
        self.execctrl >> 30 & 1 == 1
    }

    pub fn in_shift_right(&self) -> bool {
        self.shiftctrl >> 18 & 1 == 1
    }

    pub fn out_shift_right(&self) -> bool {
        self.shiftctrl >> 19 & 1 == 1
    }

    fn validate(&self) -> Result<(), &'static str> {
        if self.clkdiv & 0xff != 0 {
            return Err("CLKDIV bits 7:0 are reserved");
        }
        if self.pc > 31 {
            return Err("PC must be 0..31");
        }
        if self.side_optional() && self.side_bits() == 0 {
            return Err("invalid side-set");
        }
        if self.tx_len > MAX_TX {
            return Err("at most 4 TX words");
        }
        if self.isr_count > 32 || self.osr_count > 32 {
            return Err("shift counts must be 0..32");
        }
        if self.osr != osr_reachable(self.osr, self.osr_count, self.out_shift_right()) {
            return Err("OSR bits already shifted out must be zero");
        }
        Ok(())
    }
}

impl Request {
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.ticks == 0 || self.ticks > MAX_TICKS {
            return Err("ticks must be 1..1000");
        }
        if self.mask() == 0 {
            return Err("at least one state machine");
        }
        self.sms.iter().flatten().try_for_each(Sm::validate)
    }

    pub fn parse(line: &str) -> Result<Self, &'static str> {
        let mut fields = line.split_ascii_whitespace();
        if fields.next() != Some("RUN3") {
            return Err("expected RUN3");
        }
        let mut number = |radix| u32::from_str_radix(fields.next().ok_or("missing field")?, radix).map_err(|_| "invalid number");
        let ticks = number(10)?;
        let mask = number(16)?;
        let irq = u8::try_from(number(16)?).map_err(|_| "field out of range")?;
        let gpio_in = number(16)?;
        let mut code = [0; 32];
        for word in &mut code {
            *word = u16::try_from(number(16)?).map_err(|_| "invalid instruction")?;
        }
        let mut sms = [None; 4];
        for sm in sms
            .iter_mut()
            .enumerate()
            .filter_map(|(index, sm)| (mask >> index & 1 == 1).then_some(sm))
        {
            let (clkdiv, execctrl, shiftctrl, pinctrl) = (number(16)?, number(16)?, number(16)?, number(16)?);
            let pc = u8::try_from(number(10)?).map_err(|_| "field out of range")?;
            let tx_len = number(10)? as usize;
            if tx_len > MAX_TX {
                return Err("invalid TX length");
            }
            let mut tx = [0; MAX_TX];
            for word in &mut tx[..tx_len] {
                *word = number(16)?;
            }
            let (x, y, isr) = (number(16)?, number(16)?, number(16)?);
            let isr_count = u8::try_from(number(10)?).map_err(|_| "field out of range")?;
            let osr = number(16)?;
            let osr_count = u8::try_from(number(10)?).map_err(|_| "field out of range")?;
            let mut rx = [0; 4];
            for word in &mut rx {
                *word = number(16)?;
            }
            *sm = Some(Sm { clkdiv, execctrl, shiftctrl, pinctrl, pc, tx_len, tx, x, y, isr, isr_count, osr, osr_count, rx });
        }
        if mask > 15 || fields.next().is_some() {
            return Err("extra fields");
        }
        let request = Self { ticks, code, irq, gpio_in, sms };
        request.validate()?;
        Ok(request)
    }

    pub fn mask(&self) -> u8 {
        self.sms
            .iter()
            .enumerate()
            .fold(0, |mask, (index, sm)| mask | (sm.is_some() as u8) << index)
    }

    pub fn write(&self, output: &mut impl core::fmt::Write) -> core::fmt::Result {
        write!(
            output,
            "RUN3 {} {:x} {:02x} {:08x}",
            self.ticks,
            self.mask(),
            self.irq,
            self.gpio_in
        )?;
        for word in &self.code {
            write!(output, " {word:04x}")?;
        }
        for sm in self.sms.iter().flatten() {
            write!(
                output,
                " {:08x} {:08x} {:08x} {:08x} {} {}",
                sm.clkdiv, sm.execctrl, sm.shiftctrl, sm.pinctrl, sm.pc, sm.tx_len
            )?;
            for word in &sm.tx[..sm.tx_len] {
                write!(output, " {word:08x}")?;
            }
            write!(
                output,
                " {:08x} {:08x} {:08x} {} {:08x} {}",
                sm.x, sm.y, sm.isr, sm.isr_count, sm.osr, sm.osr_count
            )?;
            for word in &sm.rx {
                write!(output, " {word:08x}")?;
            }
        }
        writeln!(output)
    }
}

pub fn osr_reachable(osr: u32, count: u8, shift_right: bool) -> u32 {
    match (count, shift_right) {
        (32, _) => 0,
        (count, true) => osr & u32::MAX >> count,
        (count, false) => osr & u32::MAX << count,
    }
}

/// Accumulates arbitrary USB packets; oversized lines are discarded through LF.
pub struct Decoder {
    bytes: [u8; MAX_REQUEST],
    len: usize,
    overflow: bool,
}
impl Default for Decoder {
    fn default() -> Self {
        Self { bytes: [0; MAX_REQUEST], len: 0, overflow: false }
    }
}
impl Decoder {
    pub fn push(&mut self, byte: u8) -> Option<Result<Request, &'static str>> {
        if byte == b'\n' {
            let result = if self.overflow {
                Err("request too long")
            } else {
                core::str::from_utf8(&self.bytes[..self.len])
                    .map_err(|_| "invalid UTF-8")
                    .and_then(Request::parse)
            };
            self.len = 0;
            self.overflow = false;
            Some(result)
        } else {
            if self.len == MAX_REQUEST {
                self.overflow = true;
            } else {
                self.bytes[self.len] = byte;
                self.len += 1;
            }
            None
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::string::String;
    const LINE: &str = "RUN3 16 5 ff 89abcdef e023 0241 80a0 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 001f 00010000 00002000 000c0000 14000000 0 2 12345678 9abcdef0 00000001 00000002 00000003 4 00000005 6 0000000a 0000000b 0000000c 0000000d 00028000 40006200 000f0000 40000000 31 0 deadbeef 00000000 ffffffff 32 00000000 32 00000001 00000002 00000003 00000004\n";
    #[test]
    fn framing_and_recovery() {
        let mut decoder = Decoder::default();
        for _ in 0..MAX_REQUEST + 1 {
            assert!(decoder.push(b'x').is_none());
        }
        assert!(decoder.push(b'\n').unwrap().is_err());
        for _ in 0..2 {
            for byte in &LINE.as_bytes()[..LINE.len() - 1] {
                assert!(decoder.push(*byte).is_none());
            }
            assert!(decoder.push(b'\r').is_none());
            let request = decoder.push(b'\n').unwrap().unwrap();
            let sm = request.sms[0].unwrap();
            assert_eq!((request.ticks, request.irq, request.gpio_in), (16, 0xff, 0x89ab_cdef));
            assert_eq!((&request.code[..3], request.code[31]), (&[0xe023, 0x0241, 0x80a0][..], 0x001f));
            assert_eq!(&sm.tx[..sm.tx_len], &[0x1234_5678, 0x9abc_def0]);
            assert_eq!((sm.isr, sm.isr_count, sm.osr_count), (3, 4, 6));
            assert_eq!((request.sms[1], request.sms[2].unwrap().pc, request.sms[3]), (None, 31, None));
        }
    }
    #[test]
    fn round_trip() {
        let mut wire = String::new();
        Request::parse(LINE).unwrap().write(&mut wire).unwrap();
        assert_eq!(wire, LINE);
    }
    #[test]
    fn invalid_requests() {
        let line = LINE.trim_end();
        for (from, to) in [
            ("RUN3 16", "RUN3 0"),
            ("RUN3 16", "RUN3 1001"),
            ("RUN3 16 5", "RUN3 16 15"),
            ("RUN3 16 5", "RUN3 16 15 0"),
            ("RUN3 16 5", "RUN3 16 25"),
            ("00010000 00002000", "00010001 00002000"),
            ("40006200 000f0000 40000000 31", "40006200 000f0000 40000000 32"),
            ("40006200 000f0000 40000000", "40006200 000f0000 00000000"),
            ("0 2 12345678", "0 5 12345678"),
            ("00000003 4", "00000003 33"),
            ("00000005 6", "00000005 31"),
            ("00000004", "00000004 extra"),
            ("RUN3", "RUN2"),
        ] {
            let invalid = line.replacen(from, to, 1);
            assert!(Request::parse(&invalid).is_err(), "{invalid}");
        }
        assert!(Request { sms: [None; 4], ..Request::parse(LINE).unwrap() }.validate().is_err());
    }
}
