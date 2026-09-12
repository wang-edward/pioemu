#![no_std]

pub const MAX_WORDS: usize = 30;
pub const MAX_TICKS: u32 = 1000;
pub const MAX_REQUEST: usize = 256;
pub const CSV_HEADER: &str = "ticks_elapsed,test_pc,marker_pc,stalled,padout,padoe,irq,flevel,fdebug,rx_count,rx0,rx1,rx2,rx3,x,y,isr,osr";

/// Wire addresses (including encoded JMPs) are absolute, relocated by the host.
#[derive(Debug, PartialEq, Eq)]
pub struct Request {
    pub ticks: u32,
    pub origin: u8,
    pub wrap_target: u8,
    pub wrap_source: u8,
    pub side_bits: u8,
    pub side_optional: bool,
    pub side_pindirs: bool,
    pub len: usize,
    pub code: [u16; MAX_WORDS],
}

impl Request {
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.ticks == 0 || self.ticks > MAX_TICKS {
            return Err("ticks must be 1..1000");
        }
        if self.len == 0 || self.len > MAX_WORDS || self.origin as usize + self.len > MAX_WORDS {
            return Err("program must fit in instruction slots 0..29");
        }
        let end = self.origin as usize + self.len;
        if self.wrap_target < self.origin
            || self.wrap_target as usize >= end
            || self.wrap_source < self.origin
            || self.wrap_source as usize >= end
        {
            return Err("wrap outside program");
        }
        if self.side_bits > 5 || (self.side_optional && self.side_bits == 0) {
            return Err("invalid side-set");
        }
        // JMPs may target unused slots, but never the clock marker.
        if self.code[..self.len].iter().any(|w| w >> 13 == 0 && w & 31 >= 30) {
            return Err("JMP targets reserved marker slots");
        }
        Ok(())
    }

    pub fn parse(line: &str) -> Result<Self, &'static str> {
        let mut fields = line.split_ascii_whitespace();
        if fields.next() != Some("RUN1") {
            return Err("expected RUN1");
        }
        let mut number = || fields.next().ok_or("missing field")?.parse::<u32>().map_err(|_| "invalid number");
        let ticks = number()?;
        let mut byte = || u8::try_from(number()?).map_err(|_| "field out of range");
        let origin = byte()?;
        let wrap_target = byte()?;
        let wrap_source = byte()?;
        let side_bits = byte()?;
        let flags = byte()?;
        let len = byte()? as usize;
        if flags > 3 || len > MAX_WORDS {
            return Err("invalid flags or length");
        }
        let mut request = Self {
            ticks,
            origin,
            wrap_target,
            wrap_source,
            side_bits,
            side_optional: flags & 1 != 0,
            side_pindirs: flags & 2 != 0,
            len,
            code: [0; MAX_WORDS],
        };
        for word in &mut request.code[..len] {
            *word = u16::from_str_radix(fields.next().ok_or("missing instruction")?, 16).map_err(|_| "invalid instruction")?;
        }
        if fields.next().is_some() {
            return Err("extra fields");
        }
        request.validate()?;
        Ok(request)
    }

    pub fn write(&self, output: &mut impl core::fmt::Write) -> core::fmt::Result {
        write!(
            output,
            "RUN1 {} {} {} {} {} {} {}",
            self.ticks,
            self.origin,
            self.wrap_target,
            self.wrap_source,
            self.side_bits,
            self.side_optional as u8 | ((self.side_pindirs as u8) << 1),
            self.len
        )?;
        for word in &self.code[..self.len] {
            write!(output, " {word:04x}")?;
        }
        writeln!(output)
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
    use super::*;
    #[test]
    fn framing_and_recovery() {
        let line = b"RUN1 16 0 0 2 0 0 3 e023 0241 80a0\r\n";
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
            assert_eq!(&request.code[..3], &[0xe023, 0x0241, 0x80a0]);
        }
    }
    #[test]
    fn invalid_requests() {
        for line in [
            "RUN1 0 0 0 0 0 0 1 e023",
            "RUN1 1001 0 0 0 0 0 1 e023",
            "RUN1 1 29 29 30 0 0 2 e023 e023",
            "RUN1 1 0 0 0 6 0 1 e023",
            "RUN1 1 0 0 0 0 0 1 001e",
            "RUN1 1 0 0 0 0 0 2 e023",
            "RUN1 1 0 0 0 0 0 1 e023 extra",
        ] {
            assert!(Request::parse(line).is_err(), "{line}");
        }
    }
}
