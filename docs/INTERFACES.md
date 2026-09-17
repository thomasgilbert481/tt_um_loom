# Loom module interfaces (M1)

Every port of every hand-written module in `src/`, what it means, and the
cycle in which it is valid. Written by the RTL implementer as part of M1
(`docs/ARCHITECTURE.md` section 14 requires this file to match the RTL).

Cycle names follow `docs/SEMANTICS.md` section 2: a slot of thread `t` starts
in cycle `k` with `k mod 4 == t` and has stages F (cycle k), D (k+1),
X (k+2, the slot's "X cycle" `x`), W (k+3). Every effect of the slot is
registered at edge k+4 = x+2.

Common conventions: one clock `clk`, synchronous active-low `rst_n`, no
latches, no `initial` blocks. Unless stated otherwise an output is a register
and therefore holds a stable value for the whole cycle.

---

## tt_um_loom

Tiny Tapeout wrapper: pad mapping only.

| Port | Dir | Meaning / validity |
|---|---|---|
| `ui_in[7:0]` | in | `[4]` HOST_CS_n, `[5]` HOST_SCK, `[6]` HOST_MOSI, `[3:0]` IN0..IN3 (pin index 8..11), `[7]` IN4 (index 12) |
| `uo_out[7:0]` | out | `[7]` HOST_MISO, `[6]` HOST_IRQ, `[5:0]` OUT0..OUT5 (pin index 16..21). All registers |
| `uio_in[7:0]` | in | BIDIR0..7 pad inputs (pin index 0..7) |
| `uio_out[7:0]` | out | BIDIR outputs, register (`PIN_OUT[7:0]`) |
| `uio_oe[7:0]` | out | BIDIR output enables, register (`PIN_OE[7:0]`) |
| `ena` | in | ignored, listed in `_unused` |
| `clk`, `rst_n` | in | single clock domain, synchronous reset |

Parameter `IMEM_WORDS` (default 256) is passed down to `loom_top`. The retire
record is instantiated but unconnected here and collected in `_unused`.

---

## loom_top

Instantiates everything; all parameters live here.

Parameters: `IMEM_WORDS` (16 bits, default 256), `ID_VALUE` (0x4C4D),
`VERSION` (0x0001). Derived: `IMEM_AW = $clog2(IMEM_WORDS)`, `CAPS_VAL`.

Ports: the eight Tiny Tapeout pad signals above (minus `ena`), plus the
retire record.

| Port | Dir | Meaning / validity |
|---|---|---|
| `tr_valid` | out | a valid slot is in W this cycle |
| `tr_thread[1:0]` | out | which thread's slot |
| `tr_pc[9:0]` | out | the PC the slot fetched |
| `tr_ir[15:0]` | out | the instruction word |
| `tr_done` | out | 1 if the instruction completed, 0 if it stalled |
| `tr_we`, `tr_rd[2:0]`, `tr_val[15:0]` | out | register write this slot performs, if any |
| `tr_flags[2:0]` | out | `{T, C, Z}` after this slot |
| `tr_next_pc[9:0]` | out | PC after this slot |

All `tr_*` are valid during the W cycle of a slot, that is one cycle before
the slot's effects are registered. They are for simulation and debug only.

Instruction-memory arbitration lives here: the host wins the port whenever
`h_imem_req` is high, which `loom_host_ctl` only does while `core_busy` is
low, so a core fetch is never lost.

---

## loom_sync

Two flip-flop synchroniser, `WIDTH` bits, reset value `RESET_VAL`.

| Port | Dir | Meaning / validity |
|---|---|---|
| `d[WIDTH-1:0]` | in | asynchronous pad value |
| `q[WIDTH-1:0]` | out | `d` as sampled at edge `x-1`, valid during cycle `x` |

---

## loom_spi_host

SPI mode 0 slave, bit layer. SCK is sampled by `clk`, never used as a clock;
the protocol requires SCK period >= 8 core clocks.

| Port | Dir | Meaning / validity |
|---|---|---|
| `cs_n_pad`, `sck_pad`, `mosi_pad` | in | raw pads, synchronised inside |
| `miso` | out | `tx_shift[7]` while CS is active, 0 when CS_n is high. Changes only on a detected falling SCK edge |
| `cs_active` | out | synchronised CS_n low |
| `byte_done` | out | one-clock pulse, two clocks after the eighth rising SCK edge of a byte |
| `rx_byte[7:0]` | out | the received byte, valid with `byte_done` and held until the next one |
| `tx_byte[7:0]` | in | byte to send next; must be stable from one clock after `byte_done` until the following falling SCK edge |

---

## loom_host_ctl

Command layer: CMD, 16-bit ADDR, 16-bit words, auto-increment, one dummy byte
on reads. Parameters `IMEM_AW`, `ID_VALUE`, `VERSION`, `CAPS`.

Every control output is a **one-clock pulse** asserted in the cycle after the
`byte_done` of the word that caused it, so it commits at a clock edge like any
other register write.

| Port | Dir | Meaning / validity |
|---|---|---|
| `cs_active`, `byte_done`, `rx_byte`, `tx_byte` | in/out | the byte layer above |
| `h_imem_req` | out | level: the host owns the IMEM port this cycle |
| `h_imem_we` | out | one-clock pulse with `h_imem_addr`/`h_imem_wdata` valid |
| `h_imem_addr[IMEM_AW-1:0]` | out | read or write address, valid while `h_imem_req` |
| `h_imem_wdata[15:0]` | out | write data, valid with `h_imem_we` |
| `h_imem_rdata[15:0]` | in | read data, one cycle after the request |
| `h_run_we`, `h_run[3:0]` | out | pulse: write CTRL RUN |
| `h_reset[3:0]` | out | pulse: CTRL RESET, bit per thread |
| `h_step_we`, `h_step[3:0]` | out | pulse: set STEP_REQ bits |
| `h_rpc_we`, `h_rpc_sel[1:0]`, `h_rpc[9:0]` | out | pulse: write RESET_PC[sel] |
| `h_sfset_we`, `h_sfset[7:0]` | out | pulse: OR into SFLAGS |
| `h_sfclr_we`, `h_sfclr[7:0]` | out | pulse: clear SFLAGS bits |
| `h_badop_clr_we`, `h_badop_clr[15:0]` | out | pulse: write 1 clears BADOP bits |
| `h_badop_set15` | out | pulse: a refused IMEM access (read or write) |
| `h_swirq_clr_we`, `h_swirq_clr[3:0]` | out | pulse: clear SWIRQ bits |
| `h_dbg_req`, `h_dbg_wr` | out | level: debug request, held until `h_dbg_ack` |
| `h_dbg_thread[1:0]`, `h_dbg_reg[7:0]`, `h_dbg_wdata[15:0]` | out | valid while `h_dbg_req` |
| `h_dbg_ack` | in | one-clock pulse; on a read `h_dbg_rdata` is valid in the same cycle |
| `h_pout_we`/`h_pout[15:0]`, `h_poe_we`/`h_poe[7:0]`, `h_od_we`/`h_od[7:0]` | out | pulse: host writes to PIN_OUT, PIN_OE, OD_MASK |
| `run`, `halted`, `badop`, `sflags`, `swirq`, `resetpc_all` | in | status, read back through CTRL |
| `core_busy` | in | high while RUN, STEP_REQ or any valid slot is in flight; IMEM access is refused unless it is low |
| `pin_out_reg`, `pin_oe_reg`, `od_mask_reg`, `pin_in_reg` | in | pin registers for CTRL reads |
| `irq` | out | combinational: `\|(IRQ_STAT & IRQ_EN) \| \|SWIRQ`, drives uo_out[6] |

### Address map as built at M1

SPACE 0 (CTRL): 0x00 ID (R), 0x01 VERSION (R), 0x02 RUN (RW), 0x03 HALTED (R),
0x04 RESET (W), 0x08..0x0B RESET_PC[0..3] (RW), 0x10 IRQ_EN (RW),
0x11 IRQ_STAT (R) = `{SFLAGS[7:0], INQ_NOT_FULL[3:0], OUTQ_NOT_EMPTY[3:0]}`
(the two FIFO fields read 0 until M2), 0x12 IRQ_STAT2 (R) = `{12'b0, HALTED}`,
0x13 SFLAGS (RW, a write sets bits), 0x14 SFLAGS_CLR (W), 0x15 OD_MASK (RW),
0x16 PIN_OUT (RW), 0x17 PIN_OE (RW), 0x18 PIN_IN (R), 0x19 CAPS (R),
0x1A BADOP (RW, write 1 to clear), **0x1B SWIRQ (R, write 1 to clear)**.

`CAPS` = `{log2(IMEM_WORDS)[3:0], 5'b0, BOOTROM, DMEM, BIT_ENGINE, FIFO,
FIFO_DEPTH_LOG2[2:0]}`. The M1 256-word build reads `0x8000`.

SPACE 1 (IMEM): address 0..IMEM_WORDS-1. Reads and writes are accepted only
while `core_busy` is low; otherwise the write is dropped, the read returns 0
and BADOP[15] is set.

SPACE 4 (DEBUG): ADDR = `{thread[9:8], reg[7:0]}`.

| reg | Name | Access |
|---|---|---|
| 0x00..0x07 | r0..r7 | RW only while the thread is halted; otherwise reads 0, writes dropped |
| 0x08 | PC | RW (a write also clears WAIT_ACTIVE) |
| 0x09 | FLAGS `{T,C,Z}` in bits 2:0 | RW |
| 0x0A | TD | RW |
| 0x0B | NOW | R |
| 0x0C..0x0E | SR, CNT, CRC | read 0, writes ignored (M2) |
| 0x0F | RS0 | RW |
| 0x10..0x1F | CSR 0x00..0x0F of that thread | RW where the CSR is built |
| 0x20 | STEPS | RW |
| **0x21** | `{4'b0, DEPTH[1:0], RS1[9:0]}` | RW |
| **0x22** | WAIT_ACTIVE in bit 0 | RW |
| **0x23** | DT | RW |

The three addresses in bold are the ones `docs/HOST_PROTOCOL.md` leaves to the
implementation; 0x21 follows the note in that document.

SPACE 5 (STEP): a write to ADDR = t sets STEP_REQ[t] (ignored if RUN[t]).
SPACES 2 and 3 (DMEM, FIFO) read 0 and ignore writes at M1.

---

## loom_imem

Single-port instruction memory, FLOPS backend, no reset on the array.

| Port | Dir | Meaning / validity |
|---|---|---|
| `en` | in | enable: gates both the read and the write |
| `we` | in | write enable |
| `addr[AW-1:0]` | in | address, sampled at the edge that ends the cycle |
| `wdata[15:0]` | in | write data |
| `rdata[15:0]` | out | contents of `addr` as presented in the previous cycle: the address driven in F is returned in D |

Parameters `WORDS` (16 bits) and `AW`. An SRAM macro backend at M2 keeps these
ports and this one-cycle read latency.

---

## loom_core

The barrel pipeline. Parameters `IMEM_AW`, `IMEM_WORDS`.

| Port | Dir | Meaning / validity |
|---|---|---|
| `imem_addr[IMEM_AW-1:0]` | out | combinational `PC[ph]`, valid during F |
| `imem_en` | out | combinational slot validity, valid during F |
| `imem_rdata[15:0]` | in | the instruction word, consumed in D |
| `pin_in_vec[31:0]` | in | the whole pin index space as an instruction sees it in X |
| `pin_in_reg`, `pin_out_reg`, `pin_oe_reg`, `od_mask_reg` | in | register views for CSR reads and for the open-drain rule, as visible in X |
| `cw_valid` | out | a valid slot is in W: its pin writes commit at the end of this cycle |
| `cw_out_mask[13:0]`, `cw_out_data[13:0]` | out | bit-masked write to PIN_OUT (bits 7:0 BIDIR, 13:8 OUT0..5), open-drain rule already applied. Valid during W |
| `cw_oe_mask[7:0]`, `cw_oe_data[7:0]` | out | bit-masked write to PIN_OE, valid during W |
| `cw_od_we`, `cw_od[7:0]` | out | write to OD_MASK, valid during W |
| `h_*` (run control, debug) | in | see loom_host_ctl; every pulse takes effect at the next edge |
| `h_dbg_ack` | out | one-clock pulse; `h_dbg_rdata` is valid in the same cycle. A read or write of r0..r7 waits for a bubble slot, which arrives within 4 cycles |
| `h_dbg_rdata[15:0]` | out | debug read data, valid with `h_dbg_ack` |
| `run[3:0]`, `halted[3:0]`, `badop[15:0]`, `sflags[7:0]`, `swirq[3:0]` | out | register values, valid every cycle |
| `resetpc_all[39:0]` | out | RESET_PC[t] in bits `[10t+9:10t]` |
| `core_busy` | out | `\|RUN \| \|STEP_REQ \| valid slot in D, X or W` |
| `tr_*` | out | the retire record, valid during W |

Internals worth knowing: `ph` is the free-running 2-bit phase counter; the D
stage decodes only far enough to address the register file, and a second
`loom_decode` instance in X produces every strobe the execute logic uses. The
wait-condition mux has a spare input reserved for `WAITB` at M2.

---

## loom_regfile

4 threads x 8 registers x 16 bits.

| Port | Dir | Meaning / validity |
|---|---|---|
| `ra_thread[1:0]`, `ra_addr[2:0]`, `rd_a[15:0]` | in/out | read port A, combinational within the cycle. Driven from D, and borrowed by the host for debug reads of r0..r7 in cycles where the D stage is a bubble |
| `rb_thread[1:0]`, `rb_addr[2:0]`, `rd_b[15:0]` | in/out | read port B, combinational |
| `we`, `w_thread[1:0]`, `w_addr[2:0]`, `wdata[15:0]` | in | write port, takes effect at the next edge. Driven from W, and borrowed by the host for debug writes when no slot in W writes a register |

Synchronous reset clears all 32 registers to 0, as SEMANTICS section 5
requires. A read in the same cycle as a write returns the old value.

---

## loom_alu

Purely combinational; used in X and registered into W at the edge that ends X.

| Port | Dir | Meaning |
|---|---|---|
| `sel_alu`, `sel_alui`, `sel_unary` | in | which operand convention and funct table applies |
| `funct[2:0]` | in | `f_funct` from the decoder |
| `a[15:0]`, `b[15:0]` | in | operands chosen by loom_core (SEMANTICS 6.1) |
| `y[15:0]` | out | result; the caller decides whether it is written back |
| `c_out` | out | the C flag this operation would set |
| `z_out` | out | `y == 0` |

---

## loom_timer

The four per-thread timebases. All outputs are register values, so they are
exactly what an instruction sees in its X cycle. Per-thread fields are
flattened: thread `t` occupies `[16t+15:16t]` of the 16-bit vectors and
`[8t+7:8t]` of the 8-bit ones.

| Port | Dir | Meaning / validity |
|---|---|---|
| `cm_valid`, `cm_thread[1:0]` | in | a valid slot of that thread commits at this edge; clears TICK_SEEN |
| `cm_td_we`/`cm_td`, `cm_dt_we`/`cm_dt` | in | TD and DT writes from the W stage |
| `cm_tint_we`/`cm_tint`, `cm_tfrac_we`/`cm_tfrac` | in | TICK_INT and TICK_FRAC writes; either also clears ACC and suppresses the tick at that edge |
| `h_reset[3:0]` | in | CTRL RESET: `TD <= NOW` for that thread |
| `h_we`, `h_thread[1:0]`, `h_td_we`, `h_dt_we`, `h_tint_we`, `h_tfrac_we`, `h_wdata[15:0]` | in | host debug writes; a TICK_INT/TICK_FRAC write clears ACC exactly as CSRW does |
| `now_all[63:0]`, `td_all[63:0]`, `dt_all[63:0]` | out | NOW, TD, DT per thread |
| `tick_int_all[63:0]`, `tick_frac_all[31:0]` | out | the period CSRs as written (0 is stored, and treated as 1 by the divider) |
| `tick_seen_all[3:0]` | out | sticky tick flag, cleared at each valid slot commit. Only WAITB (M2) reads it |

Core writes win over host writes on the same register in the same cycle.

---

## loom_pins

The pin index space and the three global pin registers.

| Port | Dir | Meaning / validity |
|---|---|---|
| `pad_in[12:0]` | in | `{IN4, IN3..IN0, BIDIR7..BIDIR0}` raw pads |
| `cw_valid`, `cw_out_mask/data`, `cw_oe_mask/data`, `cw_od_we`, `cw_od` | in | the core's W-stage write port; the write lands at the end of this cycle, which is edge `x+2` of the slot |
| `h_out_we`/`h_out`, `h_oe_we`/`h_oe`, `h_od_we`/`h_od` | in | host writes; the core wins on any bit both touch |
| `pin_in_vec[31:0]` | out | combinational: `[12:0]` synchronised pads, `[15:13]` 0, `[21:16]` the PIN_OUT bits of OUT0..5, `[31:22]` 0. This is the value an instruction sees in cycle `x` |
| `pin_in_reg[15:0]` | out | the PIN_IN CSR view, `{3'b0, pad_sync}` |
| `pin_out_reg[15:0]` | out | `{2'b0, OUT5..OUT0, BIDIR7..0}` |
| `pin_oe_reg[7:0]`, `od_mask_reg[7:0]` | out | PIN_OE and OD_MASK |
| `uio_out[7:0]`, `uio_oe[7:0]`, `out_pins[5:0]` | out | pad registers |

---

## loom_decode (generated)

`src/loom_decode.v` is generated from `isa/isa.yaml` by `tools/loomisa`.
Purely combinational from `ir`. Two instances are used inside `loom_core`:
one in D (only `f_rd`, `f_ra`, `f_rb` and `grp_alu` are consumed, to address
the register file) and one in X (everything else). No hand-written RTL in this
project compares instruction bits.
