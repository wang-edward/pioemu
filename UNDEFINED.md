# Undefined and off-datasheet PIO behaviour

Sources: RP2350 datasheet chapter 11 (PIO); page numbers are the printed page numbers.
"panics" means the emulator panics with a message starting `undefined`, which difftest
counts separately. "Hardware" is what difftest observed on a Pico 2 (RP2350 A2, QFN60).

Each row names its regression case in `difftest/src/cases.rs`. The hardware trace of every case is
checked in as `difftest/cases/<case>.csv`; `cargo run -q -p difftest -- record-cases` rerecords them on
the board, and `cargo test -p difftest` compares the emulator against them without a board. An `xfail`
case is expected to differ from its fixture and fails the test once it matches.

## Called undefined, reserved or "must not" by the datasheet

| Behaviour | Datasheet | Emulator | Hardware | Case |
|---|---|---|---|---|
| PUSH while FJOIN_RX_PUT or FJOIN_RX_GET is set: "operation of the PUSH instruction is undefined" | 11.4.6.2, p.807 | panics | with FJOIN_RX_PUT it writes ISR to RX storage entry Y & 3 and clears ISR; with FJOIN_RX_GET it only clears ISR. No FIFO entry, no stall | `push_rx_put`, `push_rx_get` (xfail) |
| Autopush while FJOIN_RX_PUT or FJOIN_RX_GET is set: "must not be enabled ... undefined" | 11.5.4.1, p.820 | panics (difftest never generates it) | with FJOIN_RX_PUT the autopush writes ISR to RX storage entry 0 (not Y) and clears ISR | `autopush_rx_put` (xfail) |
| `mov rxfifo[], isr` without FJOIN_RX_PUT: "otherwise its operation is undefined" | 11.4.8.2, p.809 | panics | without any join it acts as a PUSH NOBLOCK that keeps ISR: the RX FIFO fills with ISR, then RXSTALL is set and nothing stalls. GET-only: no visible effect | `mov_rxfifo_isr_no_join`, `mov_rxfifo_isr_get_only` (xfail) |
| `mov osr, rxfifo[]` without FJOIN_RX_GET: "otherwise its operation is undefined" | 11.4.9.2, p.810 | panics | pops a TX word like the GET path (TXSTALL once empty, no stall), but OSR is loaded from a fixed storage entry whatever the index: entry 0 (the RX FIFO head) without a join, entry 1 with PUT-only | `mov_osr_rxfifo_no_join`, `mov_osr_rxfifo_put_only` (xfail) |
| PUT/GET with IdxI clear and a nonzero Index: "reserved encodings ... undefined" | 11.4.8.2 p.809, 11.4.9.2 p.810 | panics | the Index field is ignored: entry Y & 3 is used, as for Index 0 | `put_y_index_nonzero`, `get_y_index_nonzero` (xfail) |
| STATUS_N above the FIFO depth for TXLEVEL/RXLEVEL: "reserved, and have undefined behaviour" | 11.7 SM0_EXECCTRL, p.857 | compares the level with N as usual, so all-ones | matches the emulator, including joined (depth 8) FIFOs | `status_level_above_depth` |
| STATUS_SEL = 3 (no value listed) | 11.7 SM0_EXECCTRL, p.857 | STATUS reads 0 | reads 0 | `status_sel_3` |
| WAIT JMPPIN with Index 4–31: "Other values of Index are reserved" | 11.4.3.2, p.804 | uses Index & 3 | uses Index & 3 | `wait_jmppin_index_above_3` |
| IN sources 100 and 101: "Reserved" | 11.4.4.2, p.805 | read the STATUS value | read the same value as `mov x, status` | `in_reserved_sources` |
| MOV source 100: "Reserved" | 11.4.10.2, p.811 | reads the STATUS value | reads the same value as `mov x, status` | `mov_reserved_source` |
| MOV operation 11: "Reserved" | 11.4.10.2, p.811 | invert plus bit-reverse | invert plus bit-reverse | `mov_reserved_op` |
| SET destinations 011, 101, 110, 111: "Reserved" | 11.4.12.2, p.813 | no-op | no-op | `set_reserved_destinations` |
| PUSH/PULL with nonzero bits 3:0 (encoding shows 0) | 11.4.6.1 p.807, 11.4.7.1 p.808 | ignored: plain PUSH/PULL | plain PUSH/PULL | `push_pull_low_bits` |
| PUT with nonzero bits 6:5 (IfFull/Block, encoding shows 0) | 11.4.8.1, p.809 | ignored | ignored: ISR kept, no stall | `put_bits_6_5` |
| GET with nonzero bits 6:5 (IfEmpty/Block, encoding shows 0) | 11.4.9.1, p.810 | modelled as observed | follows the PULL path: if a PULL with those bits would pull, it pops and discards a TX word, or on an empty TX FIFO sets TXSTALL and stalls when Block is set; unless stalled, OSR is loaded from the RX storage entry and the shift count is cleared. With bits 6:5 = 0 it still pops a TX word if one is there | `get_bits_6_5` |
| IRQ / WAIT IRQ / STATUS IRQ with PREV or NEXT index modes | 11.4.3.2 p.804, 11.4.11.2 p.812 | ignored: IRQ is a no-op, WAIT and STATUS read 0 | the same, because PIO1/PIO2 are held in reset | `irq_prev_next` |
| MOV from OSR while autopull is on: "undefined ... race against system DMA" | 11.5.4.2, p.821 | reads the current OSR; refill happens at the end of the cycle | matches (TX is preloaded before the run, so there is no race) | `mov_osr_autopull` |
| Metastable inputs when the input synchroniser is bypassed: "unpredictable" | 11.5.6.3, p.824 | not modelled (inputs are static) | not tested | none: the firmware forces every input with INOVER, so nothing is ever metastable |
| Instructions written to SMx_INSTR "must not stall" when EXEC is used | 11.5.7, p.825 | SMx_INSTR writes are not modelled | not tested | none: a request has no way to write SMx_INSTR during the run |
| `WAIT 1 IRQ` "should not be used" with IRQ flags routed to the interrupt controller | 11.4.3.2, p.805 | not modelled | not tested | none: the firmware never routes IRQ flags to the interrupt controller |
| Reading an empty RX FIFO from the system: "data returned ... is undefined" | 11.7 RXF0–3, p.853 | not modelled | not tested | none: the firmware reads RXF0 only while the FIFO is not empty |
| IRQ instruction bit 7 (encoding shows 0) | 11.4.11.1, p.812 | ignored | ignored, with and without Clear and Wait | `irq_bit_7` |
| SET_COUNT or SIDESET_COUNT above 5 ("in the range 0 to 5") | 11.7 SM0_PINCTRL, p.859 | panics | SET_COUNT 7 writes only 5 pins, like SET_COUNT 5. SIDESET_COUNT 7 takes all 5 delay/side-set bits as side-set (no delay), shifted up by 2 with zeros below, and drives 7 pins | `set_count_above_5`, `sideset_count_above_5` (xfail) |

## Defined by the datasheet, but the hardware differs or goes further

| Behaviour | Datasheet | Hardware (emulator models this) | Case |
|---|---|---|---|
| OUT EXEC / MOV EXEC delay | "Delay cycles on the initial OUT are ignored" (11.4.5.2, p.807); same for MOV EXEC (11.4.10.2, p.811) | the delay of the OUT/MOV EXEC itself **is** applied | `exec_delay` |
| Autopull refill after an OUT | after an OUT, refill whenever `osr count >= threshold` (RP2040 3.5.4.2, p.339; RP2350 11.5.4.2, p.821) | the refill happens on the same cycle only when the count lands exactly on the threshold (or the OSR was already at or above the threshold before the cycle). If the OUT overshoots the threshold, the refill happens on the next cycle | `autopull_overshoot` |
| OUT_STICKY vs side-set | "the side-set takes precedence in the overlapping region" (11.5.6, p.823) | with OUT_STICKY set, the OUT/SET pin write beats side-set | `out_sticky_side_set` |
| PULL NOBLOCK on an empty TX FIFO | TXSTALL is set only by a blocking PULL or an autopull OUT (11.7 FDEBUG, p.852) | also sets TXSTALL | `pull_noblock_empty` |
| RX FIFO storage across a PIO reset | not stated | RESETS does not clear the RX storage: after a reset, RXF0_PUTGET0..3 show stale words, so the firmware loads them with the request's RX words | none: a harness property; every PUT/GET case relies on it |
| Reading RXF0_PUTGET0..3 in GET-only mode | "Direct read/write access ... if SHIFTCTRL_FJOIN_RX_PUT xor SHIFTCTRL_FJOIN_RX_GET is set" (11.7 RXF0_PUTGET0, p.865) | all four registers return the same stored word, since the state machine owns the single read port; the firmware switches to PUT-only to read the storage | none: a harness property; every GET case relies on it |
| Setting FJOIN_RX_PUT/GET | "will clear the FJOIN_TX and FJOIN_RX bits" (11.7 SM0_SHIFTCTRL, p.857) | confirmed: the TX FIFO stays 4 deep | none: a request carries at most 4 TX words, so a 4-deep and an 8-deep TX FIFO look the same |
