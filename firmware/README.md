## Prerequisites
Install the embedded Rust target:
```sh
rustup target add thumbv8m.main-none-eabihf
```

## Build
Run this from the `firmware` directory so Cargo loads its embedded target and runner configuration:
```sh
cargo build
```

Flash over the Pico 2 USB bootloader:
```sh
cargo run --release
```

The firmware enumerates as a USB CDC serial device named `PIO tick prototype`.
After flashing, run the PC example from the repository root:

```sh
cargo run -p difftest -- <count> [seed]
```

difftest generates `count` random programs, sends each one's instruction words and
configuration to the firmware, which returns one CSV snapshot after each tick, and
compares the trace with the emulator. It prints the seed of each program that differs,
so `cargo run -p difftest -- 1 <seed>` reruns it. difftest discovers the USB device by
VID/PID. Connect only one pioemu device at a time.

SM0 runs the uploaded program. SM1 runs two fixed NOPs in slots 30 and 31 as a
clock marker. Both use a divider of 1,000 and start/stop together, so the marker
PC measures a tick even during SM0 delay cycles or stalls. Uploaded programs must
fit in slots 0–29. Tick counts are limited to 1–1000.

Each CSV row is reconstructed independently: row N resets PIO0, reloads the
program, executes N ticks, captures readable state, then destructively extracts
X, Y, ISR, and OSR through SM0's RX FIFO. Consequently, collecting N rows executes
N(N+1)/2 ticks. Existing RX words appear in `rx_count` and `rx0` through `rx3`.
SM0's EXECCTRL, SHIFTCTRL and PINCTRL come from the request, and the request's TX
words are loaded into SM0's TX FIFO before the first tick. No external pin stimulus
is supplied, physical GPIO routing is not configured, and replays assume repeatable
external inputs.

The shared `protocol` crate defines the newline-delimited request:

```text
RUN2 ticks execctrl shiftctrl pinctrl origin word_count hex_words... tx_count hex_tx_words...\n
```

Counts, ticks and the origin are decimal; registers and words are hexadecimal.
Wrap and side-set come from EXECCTRL and PINCTRL. Addresses, including JMP targets,
are absolute. The firmware accepts requests split across USB packets. It responds
with the CSV header, the requested rows, and `DONE`, or an `ERR message` line, and
stops sending once the host closes the port. Wait for completion before sending
another request.
