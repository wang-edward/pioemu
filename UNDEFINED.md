# Undefined and off-datasheet PIO behaviour

Sources: RP2350 datasheet chapter 11 (PIO); page numbers are the printed page numbers.
"panics" means the emulator panics with a message starting `undefined`, which difftest
counts separately. "Hardware" is what difftest observed on a Pico 2.

## Called undefined, reserved or "must not" by the datasheet

| Behaviour | Datasheet | Emulator | Hardware |
|---|---|---|---|
| PUSH while FJOIN_RX_PUT or FJOIN_RX_GET is set: "operation of the PUSH instruction is undefined" | 11.4.6.2, p.807 | panics | not compared |
| Autopush while FJOIN_RX_PUT or FJOIN_RX_GET is set: "must not be enabled ... undefined" | 11.5.4.1, p.820 | panics (difftest never generates it) | not compared |
| `mov rxfifo[], isr` without FJOIN_RX_PUT: "otherwise its operation is undefined" | 11.4.8.2, p.809 | panics | not compared |
| `mov osr, rxfifo[]` without FJOIN_RX_GET: "otherwise its operation is undefined" | 11.4.9.2, p.810 | panics | not compared |
| PUT/GET with IdxI clear and a nonzero Index: "reserved encodings ... undefined" | 11.4.8.2 p.809, 11.4.9.2 p.810 | panics | not compared |
| STATUS_N above the FIFO depth for TXLEVEL/RXLEVEL: "reserved, and have undefined behaviour" | 11.7 SM0_EXECCTRL, p.857 | compares the level with N as usual, so all-ones | matches the emulator, including joined (depth 8) FIFOs |
| STATUS_SEL = 3 (no value listed) | 11.7 SM0_EXECCTRL, p.857 | STATUS reads 0 | reads 0 |
| WAIT JMPPIN with Index 4–31: "Other values of Index are reserved" | 11.4.3.2, p.804 | uses Index & 3 | uses Index & 3 |
| IN sources 100 and 101: "Reserved" | 11.4.4.2, p.805 | read the STATUS value | read the same value as `mov x, status` |
| MOV source 100: "Reserved" | 11.4.10.2, p.811 | reads the STATUS value | reads the same value as `mov x, status` |
| MOV operation 11: "Reserved" | 11.4.10.2, p.811 | invert plus bit-reverse | invert plus bit-reverse |
| SET destinations 011, 101, 110, 111: "Reserved" | 11.4.12.2, p.813 | no-op | no-op |
| PUSH/PULL with nonzero bits 3:0 (encoding shows 0) | 11.4.6.1 p.807, 11.4.7.1 p.808 | ignored: plain PUSH/PULL | plain PUSH/PULL |
| PUT with nonzero bits 6:5 (IfFull/Block, encoding shows 0) | 11.4.8.1, p.809 | ignored | ignored: ISR kept, no stall |
| GET with nonzero bits 6:5 (IfEmpty/Block, encoding shows 0) | 11.4.9.1, p.810 | modelled as observed | follows the PULL path: if a PULL with those bits would pull, it pops and discards a TX word, or on an empty TX FIFO sets TXSTALL and stalls when Block is set; unless stalled, OSR is loaded from the RX storage entry and the shift count is cleared. With bits 6:5 = 0 it still pops a TX word if one is there |
| IRQ / WAIT IRQ / STATUS IRQ with PREV or NEXT index modes | 11.4.3.2 p.804, 11.4.11.2 p.812 | ignored: IRQ is a no-op, WAIT and STATUS read 0 | the same, because PIO1/PIO2 are held in reset |
| MOV from OSR while autopull is on: "undefined ... race against system DMA" | 11.5.4.2, p.821 | reads the current OSR; refill happens at the end of the cycle | matches (TX is preloaded before the run, so there is no race) |
| Metastable inputs when the input synchroniser is bypassed: "unpredictable" | 11.5.6.3, p.824 | not modelled (inputs are static) | not tested |
| Instructions written to SMx_INSTR "must not stall" when EXEC is used | 11.5.7, p.825 | SMx_INSTR writes are not modelled | not tested |
| `WAIT 1 IRQ` "should not be used" with IRQ flags routed to the interrupt controller | 11.4.3.2, p.805 | not modelled | not tested |
| Reading an empty RX FIFO from the system: "data returned ... is undefined" | 11.7 RXF0–3, p.853 | not modelled | not tested |
| IRQ instruction bit 7 (encoding shows 0) | 11.4.11.1, p.812 | ignored | ignored, with and without Clear and Wait |
| SET_COUNT or SIDESET_COUNT above 5 ("in the range 0 to 5") | 11.7 SM0_PINCTRL, p.859 | panics | not compared |

## Defined by the datasheet, but the hardware differs or goes further

| Behaviour | Datasheet | Hardware (emulator models this) |
|---|---|---|
| OUT EXEC / MOV EXEC delay | "Delay cycles on the initial OUT are ignored" (11.4.5.2, p.807); same for MOV EXEC (11.4.10.2, p.811) | the delay of the OUT/MOV EXEC itself **is** applied |
| Autopull refill after an OUT | after an OUT, refill whenever `osr count >= threshold` (RP2040 3.5.4.2, p.339; RP2350 11.5.4.2, p.821) | the refill happens on the same cycle only when the count lands exactly on the threshold (or the OSR was already at or above the threshold before the cycle). If the OUT overshoots the threshold, the refill happens on the next cycle |
| OUT_STICKY vs side-set | "the side-set takes precedence in the overlapping region" (11.5.6, p.823) | with OUT_STICKY set, the OUT/SET pin write beats side-set |
| PULL NOBLOCK on an empty TX FIFO | TXSTALL is set only by a blocking PULL or an autopull OUT (11.7 FDEBUG, p.852) | also sets TXSTALL |
| RX FIFO storage across a PIO reset | not stated | RESETS does not clear the RX storage: after a reset, RXF0_PUTGET0..3 show stale words, so the firmware zeroes them |
| Setting FJOIN_RX_PUT/GET | "will clear the FJOIN_TX and FJOIN_RX bits" (11.7 SM0_SHIFTCTRL, p.857) | confirmed: the TX FIFO stays 4 deep |
