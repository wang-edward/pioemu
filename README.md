# pioemu

`pioemu/` contains the emulator; `firmware/` runs PIO on a Pico 2.

After [building and flashing the firmware](firmware/README.md), run:

```sh
cargo run -p pioemu-app
```

Edit the PIO program and tick count in `app/src/main.rs`. The PC assembles the
program, sends it over USB, and prints the firmware's step-by-step CSV trace.

Emulator todos:
- side set
- mov exec, out exec
- random testing on real pio
