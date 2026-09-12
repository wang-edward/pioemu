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
cargo run -p pioemu-app
```

Edit the inline PIO program and `TICKS` in `app/src/main.rs`. Cargo assembles the
program on the PC; the firmware receives instruction words and configuration,
then returns one CSV snapshot after each tick. The app discovers the USB device
by VID/PID and prints the trace. Connect only one pioemu device at a time.

SM0 runs the uploaded program. SM1 runs two fixed NOPs in slots 30 and 31 as a
clock marker. Both use a divider of 60,000 and start/stop together, so the marker
PC measures a tick even during SM0 delay cycles or stalls. Uploaded programs must
fit in slots 0–29. Tick counts are limited to 1–1000.

Each CSV row is reconstructed independently: row N resets PIO0, reloads the
program, executes N ticks, captures readable state, then destructively extracts
X, Y, ISR, and OSR through SM0's RX FIFO. Consequently, collecting N rows executes
N(N+1)/2 ticks. Existing RX words appear in `rx_count` and `rx0` through `rx3`.
`stalled` is the hardware EXEC_STALLED bit, not a general indication of a blocked
program instruction. No TX FIFO data or external pin stimulus is supplied;
blocking PULL/WAIT instructions may stay blocked. Shift directions are right,
autopush/autopull are disabled, pin bases are zero, SET count is five, and OUT
count is zero, matching the original prototype defaults. Physical GPIO routing
is not configured. Replays assume repeatable external inputs.

The shared `protocol` crate defines the newline-delimited request:

```text
RUN1 ticks origin wrap_target wrap_source side_bits side_flags word_count hex_words...\n
```

Metadata is decimal; instruction words are hexadecimal. Side flags use bit 0 for
optional side-set and bit 1 for pin directions; `side_bits` includes the optional
enable bit. Addresses are absolute; the app relocates JMPs and wrap addresses.
The firmware accepts requests split across USB packets. It responds with the CSV
header, the requested rows, and `DONE`, or an `ERR message` line. Wait for completion
before sending another request. The old single-character `r` command is replaced.
