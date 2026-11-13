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

To reflash a running firmware without holding BOOTSEL, reboot it into the bootloader first
(opening the port at 1200 baud triggers the ROM reboot):
```sh
cargo run -q -p difftest -- bootsel && cd firmware && cargo run --release
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

The request lists which of SM0–SM3 run; the others stay disabled and hold junk.
One CTRL write enables the listed state machines and restarts all four clock
dividers, and a second CTRL write disables them exactly N system clocks later. Both
writes come from a routine the firmware writes to SRAM for each row: `str`, N−1
`nop.w`, `str`. The Cortex-M33 issues two 16-bit NOPs per cycle, so the routine uses
32-bit ones. Nothing else touches the bus while the state machines run. Tick counts
are system clocks, limited to 1–1000.

Each CSV row is reconstructed independently: row N resets PIO0, reloads everything,
runs N system clocks, captures readable state, then destructively extracts X, Y, ISR
and OSR of each running state machine through its RX FIFO. Consequently, collecting
N rows runs N(N+1)/2 system clocks. Existing RX words appear in `smN_rx_count` and
`smN_rx0` through `smN_rx3`. With FJOIN_RX_PUT or FJOIN_RX_GET set, the RX FIFO is not
drained; the words are read from RXFn_PUTGET0..3 instead (clearing FJOIN_RX_GET first
when both are set). Columns of state machines that do not run are empty.
Each running state machine's CLKDIV, EXECCTRL, SHIFTCTRL and PINCTRL come from the
request, and its TX words are loaded into its TX FIFO before the first tick. Before
that, its X, Y, ISR, OSR and both shift counts, and the IRQ flags, are set from the
request with injected instructions at full speed (autopush/autopull off, then
SHIFTCTRL is restored). An OSR shift count can only follow shifting, so bits it has
shifted out must be zero.
The request's `gpio_in` word forces every GPIO input seen by PIO through IO_BANK0
GPIOn_CTRL.INOVER for the whole request, so no external stimulus is needed. Pin outputs
are not routed to the pads.

The shared `protocol` crate defines the newline-delimited request:

```text
RUN3 ticks sm_mask irq gpio_in 32_hex_words [clkdiv execctrl shiftctrl pinctrl pc tx_count hex_tx_words... x y isr isr_count osr osr_count rx0 rx1 rx2 rx3]...\n
```

The bracketed group repeats for each state machine in `sm_mask`, lowest first.
Counts, ticks and PCs are decimal; the mask, registers, words, IRQ flags and `gpio_in`
are hexadecimal. Wrap and side-set come from EXECCTRL and PINCTRL. The firmware accepts requests split across USB packets. It responds
with the CSV header, the requested rows, and `DONE`, or an `ERR message` line, and
stops sending once the host closes the port. Wait for completion before sending
another request.
