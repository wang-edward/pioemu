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
Open that serial port at any baud rate and send `r`. It returns 16 CSV snapshots:

```sh
screen /dev/cu.usbmodem* 115200
```

SM0 runs a small program containing delay cycles and a blocking `PULL`. SM1 runs
two NOPs from instruction-memory slots 30 and 31. Both use a divider of 60,000
and are enabled and disabled together for each row, so the marker PC indicates
one PIO clock tick even when SM0's PC does not change.
