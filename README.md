# pioemu

This is an emulator for the PIO peripheral of the RP2040 and [RP2350](https://www.raspberrypi.com/products/rp2350/) microcontrollers.

I tested the emulator against real hardware.

## Testing
The emulator is tested as follows:
1. Generate a random PIO program in `difftest`
2. Send it over USB to `firmware`, run it on the real PIO and send back the results
3. Run the same program on `pioemu`
4. Compare the results

## Q: How do we sample the PIO?
It isn't obvious to know when to sample the PIO. The PC does not always change on a tick. For example, the SM (state machine) could be stalled, and with clock dividers an SM only ticks on some system clocks.
The firmware stops the PIO after an exact number of system clocks instead.
One write to CTRL starts the SMs and restarts their clock dividers, and a second write stops them N system clocks later.
The CPU runs both writes from a routine it writes to RAM, with N-1 `nop.w` instructions between them, so the gap is exact.

## How do we read internal registers `(x, y, isr, osr)`?
We can't read these registers directly from the CPU.
Instead, we have to extract them by pushing them into the RX fifo and popping it with the CPU.[^1]
But we can't do this on intermediate ticks, since it changes program execution.

Here's a workaround. If we want to run a program for N system clocks, we'll instead run it for:
- 1 system clock and finish
- Then read the internal registers by pushing to RX.
- Reset the state machines.
- Then run from the start for 2 system clocks and do the same thing.

This takes longer but makes the testing much more rigorous.
We compare internal registers and FIFO contents after every tick which cataches differences that wouldn’t show up if we're only looking at the PC or GPIO outputs.

## Specifics on reading internal registers
- ISR: (push its value into the RX FIFO, then have the CPU read it. This clears ISR, so capture it first.)
- X: (Copy X into ISR, push ISR into the RX FIFO, then read it from the CPU.)
- Y: (Copy Y into ISR, push ISR into the RX FIFO, then read it from the CPU.)
- OSR: (Copy OSR into ISR, push ISR into the RX FIFO, then read it from the CPU.)

Only the ISR and the RX FIFO are changed with this approach.
X, Y, and the OSR stay the same.
