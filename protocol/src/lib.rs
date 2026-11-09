#![no_std]

pub const MAX_WORDS: usize = 30;
pub const MAX_TICKS: u32 = 1000;
pub const MAX_REQUEST: usize = 512;
pub const MAX_TX: usize = 4;
pub const CSV_HEADER: &str = "ticks_elapsed,test_pc,marker_pc,padout,padoe,irq,flevel,fdebug,rx_count,rx0,rx1,rx2,rx3,x,y,isr,osr";

/// Wire addresses (including encoded JMPs) are absolute.
#[derive(Debug, PartialEq, Eq)]
pub struct Request {
    pub ticks: u32,
    pub execctrl: u32,
    pub shiftctrl: u32,
    pub pinctrl: u32,
    pub origin: u8,
    pub len: usize,
    pub code: [u16; MAX_WORDS],
    pub tx_len: usize,
    pub tx: [u32; MAX_TX],
    pub x: u32,
    pub y: u32,
    pub isr: u32,
    pub isr_count: u8,
    pub osr: u32,
    pub osr_count: u8,
    pub irq: u8,
}

impl Request {
    pub fn wrap_bottom(&self) -> u8 {
        (self.execctrl >> 7) as u8 & 31
    }

    pub fn wrap_top(&self) -> u8 {
        (self.execctrl >> 12) as u8 & 31
    }

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

    pub fn validate(&self) -> Result<(), &'static str> {
        if self.ticks == 0 || self.ticks > MAX_TICKS {
            return Err("ticks must be 1..1000");
        }
        if self.len == 0 || self.len > MAX_WORDS || self.origin as usize + self.len > MAX_WORDS {
            return Err("program must fit in instruction slots 0..29");
        }
        let program = self.origin..self.origin + self.len as u8;
        if !program.contains(&self.wrap_bottom()) || !program.contains(&self.wrap_top()) {
            return Err("wrap outside program");
        }
        if self.side_bits() > 5 || (self.side_optional() && self.side_bits() == 0) {
            return Err("invalid side-set");
        }
        // JMPs may target unused slots, but never the clock marker.
        if self.code[..self.len].iter().any(|w| w >> 13 == 0 && w & 31 >= 30) {
            return Err("JMP targets reserved marker slots");
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

    pub fn parse(line: &str) -> Result<Self, &'static str> {
        let mut fields = line.split_ascii_whitespace();
        if fields.next() != Some("RUN2") {
            return Err("expected RUN2");
        }
        let mut number = |radix| u32::from_str_radix(fields.next().ok_or("missing field")?, radix).map_err(|_| "invalid number");
        let ticks = number(10)?;
        let (execctrl, shiftctrl, pinctrl) = (number(16)?, number(16)?, number(16)?);
        let origin = u8::try_from(number(10)?).map_err(|_| "field out of range")?;
        let len = number(10)? as usize;
        if len > MAX_WORDS {
            return Err("invalid length");
        }
        let mut code = [0; MAX_WORDS];
        for word in &mut code[..len] {
            *word = u16::try_from(number(16)?).map_err(|_| "invalid instruction")?;
        }
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
        let irq = u8::try_from(number(16)?).map_err(|_| "field out of range")?;
        if fields.next().is_some() {
            return Err("extra fields");
        }
        let request =
            Self { ticks, execctrl, shiftctrl, pinctrl, origin, len, code, tx_len, tx, x, y, isr, isr_count, osr, osr_count, irq };
        request.validate()?;
        Ok(request)
    }

    pub fn write(&self, output: &mut impl core::fmt::Write) -> core::fmt::Result {
        write!(
            output,
            "RUN2 {} {:08x} {:08x} {:08x} {} {}",
            self.ticks, self.execctrl, self.shiftctrl, self.pinctrl, self.origin, self.len
        )?;
        for word in &self.code[..self.len] {
            write!(output, " {word:04x}")?;
        }
        write!(output, " {}", self.tx_len)?;
        for word in &self.tx[..self.tx_len] {
            write!(output, " {word:08x}")?;
        }
        writeln!(
            output,
            " {:08x} {:08x} {:08x} {} {:08x} {} {:02x}",
            self.x, self.y, self.isr, self.isr_count, self.osr, self.osr_count, self.irq
        )
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
    #[test]
    fn framing_and_recovery() {
        let line =
            b"RUN2 16 00002000 000c0000 14000000 0 3 e023 0241 80a0 2 12345678 9abcdef0 00000001 00000002 00000003 4 00000005 6 ff\r\n";
        let mut decoder = Decoder::default();
        for _ in 0..300 {
            assert!(decoder.push(b'x').is_none());
        }
        assert!(decoder.push(b'\n').unwrap().is_err());
        for _ in 0..2 {
            for byte in &line[..line.len() - 1] {
                assert!(decoder.push(*byte).is_none());
            }
            let request = decoder.push(b'\n').unwrap().unwrap();
            assert_eq!(request.ticks, 16);
            assert_eq!((request.wrap_bottom(), request.wrap_top()), (0, 2));
            assert_eq!(&request.code[..3], &[0xe023, 0x0241, 0x80a0]);
            assert_eq!(&request.tx[..request.tx_len], &[0x1234_5678, 0x9abc_def0]);
            assert_eq!((request.isr, request.isr_count, request.osr_count, request.irq), (3, 4, 6, 0xff));
        }
    }
    #[test]
    fn round_trip() {
        let line = "RUN2 7 40006200 000f0000 40000000 4 3 f823 0245 80a0 1 deadbeef 00000000 ffffffff 80000000 32 00000000 32 00\n";
        let mut wire = String::new();
        Request::parse(line).unwrap().write(&mut wire).unwrap();
        assert_eq!(wire, line);
    }
    #[test]
    fn invalid_requests() {
        for line in [
            "RUN2 0 00000000 000c0000 14000000 0 1 e023 0 0 0 0 0 0 32 0",
            "RUN2 1001 00000000 000c0000 14000000 0 1 e023 0 0 0 0 0 0 32 0",
            "RUN2 1 0001e000 000c0000 14000000 0 1 e023 0 0 0 0 0 0 32 0",
            "RUN2 1 00000000 000c0000 c0000000 0 1 e023 0 0 0 0 0 0 32 0",
            "RUN2 1 40000000 000c0000 14000000 0 1 e023 0 0 0 0 0 0 32 0",
            "RUN2 1 00000000 000c0000 14000000 0 1 001e 0 0 0 0 0 0 32 0",
            "RUN2 1 00000000 000c0000 14000000 0 2 e023 0 0 0 0 0 0 32 0",
            "RUN2 1 00000000 000c0000 14000000 0 1 e023 5 1 2 3 4 5 0 0 0 0 0 32 0",
            "RUN2 1 00000000 000c0000 14000000 0 1 e023 0 0 0 0 0 0 32 0 extra",
            "RUN2 1 00000000 000c0000 14000000 0 1 e023 0 0 0 0 33 0 32 0",
            "RUN2 1 00000000 000c0000 14000000 0 1 e023 0 0 0 0 0 80000000 1 0",
            "RUN2 1 00000000 00040000 14000000 0 1 e023 0 0 0 0 0 00000001 1 0",
            "RUN1 1 0 0 0 0 0 1 e023",
        ] {
            assert!(Request::parse(line).is_err(), "{line}");
        }
    }
}
