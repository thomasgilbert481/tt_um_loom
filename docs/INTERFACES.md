# Loom module interfaces (M3 slice B)

Every port of every hand-written module in `src/`, what it means, and the
cycle in which it is valid. Written by the RTL implementer as part of M1,
updated with the M2 RTL, again with M3 slice A (the bit-engine encoders,
stuffing and DIFF of `docs/SEMANTICS.md` 6.9.1) and again with M3 slice B
(the data memory of 6.11); `docs/ARCHITECTURE.md` section 14 requires this
file to match the RTL.

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

Parameters `IMEM_IMPL` (default `"MACRO"`) and `IMEM_WORDS` (default 512) are
passed down to `loom_top` (D-020). The retire record is instantiated but
unconnected here and collected in `_unused`.

---

## loom_top

Instantiates everything; all parameters live here.

Parameters: `IMEM_IMPL` (`"MACRO"`, the default, or `"FLOPS"`; see
loom_imem), `IMEM_WORDS` (16 bits, default 512; the macro needs 512),
`FIFO_DEPTH` (a power of two from 2 to 8, default 4), `ID_VALUE` (0x4C4D),
`VERSION` (0x0004: 4 from M3 slice B on, SEMANTICS 6.11; 3 was slice A).
loom_top also arbitrates the single instruction-memory port between the host
(which may only ask while `core_busy` is low) and the core, whose own write
is the `ST` of 6.11.
Derived: `IMEM_AW = $clog2(IMEM_WORDS)`, `CAPS_VAL` (SEMANTICS 5: FIFOs with
`log2(FIFO_DEPTH)` in bits 2:0, and one bit per M2 feature as it is built).

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
| `byte_done` | out | one-clock pulse registered at edge E+2, where E is the edge at which the first synchroniser flop takes SCK high for the byte's eighth bit |
| `rx_byte[7:0]` | out | the received byte, valid with `byte_done` and held until the next one |
| `tx_byte[7:0]` | in | byte to send next; must be stable from one clock after `byte_done` until the following falling SCK edge |

---

## loom_host_ctl

Command layer: CMD, 16-bit ADDR, 16-bit words, auto-increment (except in the
FIFO space), one dummy byte on reads. Parameters `IMEM_AW`, `ID_VALUE`,
`VERSION`, `CAPS`.

**Host write commit rule** (`docs/spec-questions/rtl-m2.md` 7). Let E be the
first edge at which the first synchroniser flop takes SCK high for the last
bit of a word. `byte_done` arrives at E+2 and every effect of the word is
registered here at E+3: a one-clock pulse on the outputs below, or a write
strobe for IRQ_EN and IRQ_EN2, which live in this module. So the target
register loads at edge E+4 and the effect is visible from cycle E+4. The same
edge applies to the pop of a FIFO read word and to BADOP[14]. A DEBUG write
of r0..r7 may wait up to three more edges for the register-file write port.

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
| `h_inq_push[3:0]`, `h_fifo_wdata[15:0]` | out | pulse: push the word into INQ[t]; loom_core drops it and sets BADOP[14] if INQ[t] is full in that cycle |
| `h_outq_pop[3:0]` | out | pulse: pop OUTQ[t], at the end of a read word whose peek found an entry |
| `h_badop_set14` | out | pulse: a read word of the FIFO space found OUTQ empty when it was loaded |
| `fifo_stat[47:0]` | in | per thread, 12 bits each: `{OUTQ_COUNT[3:0], INQ_COUNT[3:0], OUTQ_EMPTY, OUTQ_FULL, INQ_EMPTY, INQ_FULL}` |
| `outq_head[63:0]`, `outq_next[63:0]` | in | per thread: the OUTQ head entry and the entry after it (for a word loaded at the edge where the previous word is popped) |
| `h_dbg_req`, `h_dbg_wr` | out | level: debug request, held until `h_dbg_ack` |
| `h_dbg_thread[1:0]`, `h_dbg_reg[7:0]`, `h_dbg_wdata[15:0]` | out | valid while `h_dbg_req` |
| `h_dbg_ack` | in | one-clock pulse; on a read `h_dbg_rdata` is valid in the same cycle |
| `h_pout_we`/`h_pout[15:0]`, `h_poe_we`/`h_poe[7:0]`, `h_od_we`/`h_od[7:0]` | out | pulse: host writes to PIN_OUT, PIN_OE, OD_MASK |
| `run`, `halted`, `badop`, `sflags`, `swirq`, `resetpc_all` | in | status, read back through CTRL |
| `core_busy` | in | high while RUN, STEP_REQ or any valid slot is in flight; IMEM access is refused unless it is low |
| `pin_out_reg`, `pin_oe_reg`, `od_mask_reg`, `pin_in_reg` | in | pin registers for CTRL reads |
| `irq` | out | HOST_IRQ (`uo_out[6]`), a register loaded at every edge with `\|(IRQ_STAT & IRQ_EN) \| \|(IRQ_STAT2 & IRQ_EN2) \| \|SWIRQ` from the values visible in the cycle before (SEMANTICS 6.8) |

### Address map as built

SPACE 0 (CTRL): 0x00 ID (R), 0x01 VERSION (R), 0x02 RUN (RW), 0x03 HALTED (R),
0x04 RESET (W), 0x08..0x0B RESET_PC[0..3] (RW), 0x10 IRQ_EN (RW),
0x11 IRQ_STAT (R) = `{SFLAGS[7:0], INQ_NOT_FULL[3:0], OUTQ_NOT_EMPTY[3:0]}`,
0x12 IRQ_STAT2 (R) = `{12'b0, HALTED}`, 0x13 SFLAGS (RW, a write sets bits),
0x14 SFLAGS_CLR (W), 0x15 OD_MASK (RW), 0x16 PIN_OUT (RW), 0x17 PIN_OE (RW),
0x18 PIN_IN (R), 0x19 CAPS (R), 0x1A BADOP (RW, write 1 to clear; bits 3:0
per thread, 14 host FIFO error, 15 host access error), 0x1B SWIRQ (R, write 1
to clear), 0x1C IRQ_EN2 (RW, bits 3:0; bits 15:4 read 0).

`CAPS` = `{log2(IMEM_WORDS)[3:0], 3'b0, BE_AUTO, SETP_D, BOOTROM, DMEM,
BIT_ENGINE, FIFO, FIFO_DEPTH_LOG2[2:0]}` (SEMANTICS 5); a feature bit is set
once the feature is built.

SPACE 1 (IMEM): address 0..IMEM_WORDS-1. Reads and writes are accepted only
while `core_busy` is low; otherwise the write is dropped, the read returns 0
and BADOP[15] is set.

SPACE 3 (FIFO): 0x0000+t, a write pushes INQ[t] and a read pops OUTQ[t]
(peek when the word is loaded, pop at the end of the word, SEMANTICS 6.7);
0x0100+t reads the status word `{4'b0, fifo_stat[t]}`. The address never
increments in this space; other addresses read 0 and ignore writes.

SPACE 4 (DEBUG): ADDR = `{thread[9:8], reg[7:0]}`.

| reg | Name | Access |
|---|---|---|
| 0x00..0x07 | r0..r7 | RW only while the thread is halted; otherwise reads 0, writes dropped |
| 0x08 | PC | RW (a write also clears WAIT_ACTIVE) |
| 0x09 | FLAGS `{T,C,Z}` in bits 2:0 | RW |
| 0x0A | TD | RW |
| 0x0B | NOW | R |
| 0x0C..0x0E | SR, CNT, CRC (the same registers as CSR 0x0D..0x0F at 0x1D..0x1F) | RW |
| 0x0F | RS0 | RW |
| 0x10..0x1F | CSR 0x00..0x0F of that thread, the bit-engine CSRs included (BE_CFG keeps bits 1, 4:3, 6:5, 7, 9, 10; a write to it also clears the encoder state) | RW where the CSR is writable |
| 0x20 | STEPS | RW |
| **0x21** | `{4'b0, DEPTH[1:0], RS1[9:0]}` | RW |
| **0x22** | WAIT_ACTIVE in bit 0 | RW |
| **0x23** | DT | RW |
| 0x24 | TICK_SEEN in bit 0 | RW |
| 0x25 | `{9'b0, LAT_VALID, LAT_VAL, LAT_PIN[4:0]}` (6.10) | RW |
| 0x26 | `{INQ_CNT, OUTQ_CNT}` as `{byte, byte}` | R |
| 0x27 | `{8'b0, FIRST, HALF, PEND, RVAL, RUN[2:0], LVL}`, the encoder state (6.9.1) | RW |
| 0x28 | `{11'b0, MEM_PEND, MEM_LD, MEM_RD[2:0]}`, the data access in progress (6.11) | RW |

The three addresses in bold are the ones `docs/HOST_PROTOCOL.md` leaves to the
implementation; 0x21 follows the note in that document. Debug writes other
than r0..r7 take effect only while the thread is not running.

SPACE 5 (STEP): a write to ADDR = t sets STEP_REQ[t] (ignored if RUN[t]).
SPACE 2 (DMEM) reads 0 and ignores writes, in slice B as before: the data
memory of 6.11 *is* the instruction memory, so a data image is loaded and
dumped through SPACE 1 (`docs/spec-questions/rtl-m3b.md` question 6).

---

## loom_imem

Single-port instruction memory, no reset on the array. Parameter `IMPL`
selects the backend: `"MACRO"` (default, D-020) is one
`RM_IHPSG13_1P_512x16_c2_bm_bist` SRAM macro through `loom_imem_macro` and
needs `WORDS` 512, `AW` 9; `"FLOPS"` is a `WORDS` x 16 flip-flop array.
Both keep the ports and the one-cycle read latency below.

| Port | Dir | Meaning / validity |
|---|---|---|
| `en` | in | enable: gates both the read and the write; a cycle without it leaves `rdata` unchanged (MACRO: drives the macro's A_MEN) |
| `we` | in | write enable, only with `en` |
| `addr[AW-1:0]` | in | address, sampled at the edge that ends the cycle |
| `wdata[15:0]` | in | write data |
| `rdata[15:0]` | out | contents of `addr` as presented in the previous cycle: the address driven in F is returned in D, whether that address was a `PC` or the data address of an `LD` (6.11). After a write cycle it is not defined by this contract (FLOPS: the old contents; MACRO: unchanged); the completion slot of an `ST` is the only reader and SEMANTICS 8 leaves its `tr_ir` uncompared |

Parameters `IMPL`, `WORDS` (16 bits) and `AW`. The flattened path of the macro
instance, which `src/config.json` names, is `u_loom.u_imem.g_macro.u_macro.sram`.

---

## loom_imem_macro

The SRAM macro behind a single-port read/write interface (brought over
unchanged from the `sram-smoke` test). Every enable of the macro is active
high; the BIST port is tied off, `A_DLY` is tied 1, the bit mask all ones.

| Port | Dir | Meaning / validity |
|---|---|---|
| `rst_n` | in | drives A_MEN: low means no access and A_DOUT holds. loom_imem feeds it `en` |
| `addr[8:0]` | in | read address, sampled at the edge that ends the cycle |
| `rdata[15:0]` | out | A_DOUT, the macro's registered output: the word read at the previous edge |
| `we` | in | write this cycle (A_WEN); no read is performed in that cycle (A_REN = ~we) |
| `waddr[8:0]`, `wdata[15:0]` | in | write address and data, valid with `we` |

---

## loom_core

The barrel pipeline. Parameters `IMEM_AW`, `IMEM_WORDS`, `FIFO_DEPTH`.

| Port | Dir | Meaning / validity |
|---|---|---|
| `imem_addr[IMEM_AW-1:0]` | out | combinational, valid during F: `PC[ph]`, or the held data address when this F cycle is the access of an `LD`/`ST` (6.11) |
| `imem_en` | out | combinational slot validity, valid during F |
| `imem_we` | out | combinational, valid during F: this F cycle is the access of an `ST`. Never high without `imem_en` |
| `imem_wdata[15:0]` | out | the `r[rd]` an `ST` writes, held in the shared register since the first slot's commit edge; valid with `imem_we` |
| `imem_rdata[15:0]` | in | consumed in D: the instruction word, or the data word of an `LD` |
| `pin_in_vec[31:0]` | in | the whole pin index space as an instruction sees it in X |
| `pin_in_reg`, `pin_out_reg`, `pin_oe_reg`, `od_mask_reg` | in | register views for CSR reads and for the open-drain rule, as visible in X |
| `cw_valid` | out | a valid slot is in W: its pin writes commit at the end of this cycle |
| `cw_out_mask[13:0]`, `cw_out_data[13:0]` | out | bit-masked write to PIN_OUT (bits 7:0 BIDIR, 13:8 OUT0..5), open-drain rule already applied. Valid during W |
| `cw_oe_mask[7:0]`, `cw_oe_data[7:0]` | out | bit-masked write to PIN_OE, valid during W |
| `cw_od_we`, `cw_od[7:0]` | out | write to OD_MASK, valid during W |
| `h_*` (run control, debug) | in | see loom_host_ctl; every pulse takes effect at the next edge |
| `h_inq_push[3:0]`, `h_fifo_wdata[15:0]`, `h_outq_pop[3:0]`, `h_badop_set14` | in | the host FIFO port of loom_host_ctl; each pulse takes effect at the next edge. A push into a full INQ is dropped and sets BADOP[14] |
| `fifo_stat[47:0]`, `outq_head[63:0]`, `outq_next[63:0]` | out | FIFO status words and OUTQ entries for the host, register values valid every cycle |
| `h_dbg_ack` | out | one-clock pulse; `h_dbg_rdata` is valid in the same cycle. A read or write of r0..r7 waits for a bubble slot, which arrives within 4 cycles |
| `h_dbg_rdata[15:0]` | out | debug read data, valid with `h_dbg_ack` |
| `run[3:0]`, `halted[3:0]`, `badop[15:0]`, `sflags[7:0]`, `swirq[3:0]` | out | register values, valid every cycle |
| `resetpc_all[39:0]` | out | RESET_PC[t] in bits `[10t+9:10t]` |
| `core_busy` | out | `\|RUN \| \|STEP_REQ \| valid slot in D, X or W` |
| `tr_*` | out | the retire record, valid during W |

Internals worth knowing: `ph` is the free-running 2-bit phase counter; the D
stage decodes only far enough to address the register file, and a second
`loom_decode` instance in X produces every strobe the execute logic uses. The
W stage never decodes a thread number (D-019): each consumer group of
per-thread state (PC and flags; the other per-thread core state; the register
file; the timers; the FIFOs; the bit engine) has its own self-rotating one-hot
ring, reset to `4'b0010`, qualified by `w_valid`. PUSH and POP decide in X and
stall like waits; WAITB tests bit engine idle (0, always true in manual mode),
OUTQ not full (1), INQ not empty (2) and TICK_SEEN (3). SHO, SHI, LDSR, CRCI
and CSRW compute the new SR, CNT and CRC in X, and the W stage commits them
into loom_be; SHO's pin write goes through the same single-pin path as SETP,
so the open-drain rule and the writable-index check are shared. M3 slice A
(6.9.1) adds one copy of the encoder, the stuffer and the decoder beside
them, selected by the same `xsel`, and an eighth commit strobe for the
per-thread encoder state; `DIFF`'s second pin write joins the 32-bit mask
the group write already builds, so a non-writable `out + 1` is dropped like
any other.

M3 slice B (6.11, D-027) adds the data memory. Per thread it holds
`MEM_PEND`, `MEM_LD` and `MEM_RD[2:0]` of SEMANTICS 5 and the access itself,
`{address[IMEM_AW-1:0], store word[15:0], write}`, all committed on the PC
ring. The first slot of an `LD`/`ST` adds `ra + imm5` in X and rides the
existing wait machinery, so it holds `PC` and raises `WAIT_ACTIVE` exactly
as a stalling wait does; its commit edge loads that thread's held access
from the W stage (the store word travels in `w_rval`, which a memory slot
never uses for a register write). The F stage then takes it:
`mem_fetch = valid_f & MEM_PEND[ph]` selects the held address, `imem_we`
and `imem_wdata` over `PC`, and no instruction is fetched for that slot. The
completion slot is that same slot: it has `MEM_PEND` set in X and decodes
nothing at all, because `xins` is low and the new `dec_ok` (which replaces
`~bad_op` everywhere) suppresses every decode-driven effect. What is left is
the `LD` write of `r[MEM_RD]` with the received word, `MEM_PEND` and
`WAIT_ACTIVE` cleared and `PC <= next`.

For a running thread that valid F cycle is the one right after the first
slot's W edge, so the access lands exactly where 6.11 puts it. For a thread
stepped one slot at a time it is the next `STEP`, which is what makes
"stepping n times is observably identical to running n slots" (SEMANTICS 7)
true of an access; this is why the access is held per thread rather than in
one register shared by the four, which another thread's `LD`/`ST` could
overwrite in between (the architect's ruling of 2026-09-22, superseding the
single register D-027 costed; `docs/spec-questions/rtl-m3b.md` question 1).
`MEM_PEND` is the valid bit of the held access, so there is no second one,
and `CTRL.RESET` or a debug `PC` write clears `MEM_PEND` only, abandoning
the access while `MEM_LD` and `MEM_RD` keep their values. A host write of
debug 0x28 sets the three section-5 fields and leaves the held address and
store word alone. Two ports were added to loom_core: `imem_we` and
`imem_wdata`.

---

## loom_regfile

4 threads x 8 registers x 16 bits.

| Port | Dir | Meaning / validity |
|---|---|---|
| `ra_thread[1:0]`, `ra_addr[2:0]`, `rd_a[15:0]` | in/out | read port A, combinational within the cycle. Driven from D, and borrowed by the host for debug reads of r0..r7 in cycles where the D stage is a bubble |
| `rb_thread[1:0]`, `rb_addr[2:0]`, `rd_b[15:0]` | in/out | read port B, combinational |
| `we_oh[3:0]`, `w_addr[2:0]`, `wdata[15:0]` | in | write port, takes effect at the next edge; `we_oh` is one-hot (or zero) over threads, from loom_core's W-stage ring (D-019). Driven from W, and borrowed by the host for debug writes when no slot in W writes a register |

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
| `cm_sel[3:0]` | in | one-hot: a valid slot of thread t commits at this edge (from loom_core's ring, D-019); clears TICK_SEEN |
| `cm_td_we`/`cm_td`, `cm_dt_we`/`cm_dt` | in | TD and DT writes from the W stage |
| `cm_tint_we`/`cm_tint`, `cm_tfrac_we`/`cm_tfrac` | in | TICK_INT and TICK_FRAC writes; either also clears ACC and suppresses the tick at that edge |
| `h_reset[3:0]` | in | CTRL RESET: `TD <= NOW` for that thread |
| `h_we`, `h_thread[1:0]`, `h_td_we`, `h_dt_we`, `h_tint_we`, `h_tfrac_we`, `h_tseen_we`, `h_wdata[15:0]` | in | host debug writes; a TICK_INT/TICK_FRAC write clears ACC exactly as CSRW does; `h_tseen_we` writes TICK_SEEN (debug 0x24) |
| `now_all[63:0]`, `td_all[63:0]`, `dt_all[63:0]` | out | NOW, TD, DT per thread |
| `tick_int_all[63:0]`, `tick_frac_all[31:0]` | out | the period CSRs as written (0 is stored, and treated as 1 by the divider) |
| `tick_seen_all[3:0]` | out | sticky tick flag, cleared at each valid slot commit. Only WAITB (M2) reads it |

Core writes win over host writes on the same register in the same cycle.

---

## loom_fifo

One 16-bit FIFO of `DEPTH` entries (a power of two, at least 2; `AW =
log2(DEPTH)`). loom_core has eight: INQ[t] (host pushes, thread pops) and
OUTQ[t] (thread pushes, host pops).

| Port | Dir | Meaning / validity |
|---|---|---|
| `clr` | in | CTRL.RESET of the owning thread: empties the FIFO at this edge, wins over push and pop |
| `push`, `wdata[15:0]` | in | append `wdata` at this edge; the caller never pushes into a full FIFO |
| `pop` | in | remove the head at this edge; the caller never pops an empty FIFO |
| `count[AW:0]` | out | occupancy, a register; push and pop in the same cycle give `count + push - pop` |
| `head[15:0]`, `next[15:0]` | out | the oldest entry (valid when count > 0) and the one after it (count > 1) |

Entries are not reset (SEMANTICS 5); the count and pointers are.

---

## loom_be

The per-thread bit-engine state, manual mode (SEMANTICS 6.9 and 6.9.1): SR
(16), CNT (5), CRC (16), BE_CFG (only DIR bit 1, ENC bits 4:3, STUFF bits
6:5, INV bit 7, CRC_EN bit 9 and DIFF bit 10 are stored, and the reserved
value 3 of ENC and of STUFF is stored as 0; the other bits read 0 and ignore
writes), BE_PINS (10), BE_RELOAD (5), CRC_POLY (16), CRC_INIT (16), and the
8-bit encoder state `{FIRST, HALF, PEND, RVAL, RUN[2:0], LVL}`, all reset to
0. No arithmetic: loom_core's X stage computes the shifts, the CRC step and
the encoder, stuffer and decoder; this module stores what the W stage
commits.

| Port | Dir | Meaning / validity |
|---|---|---|
| `cm_sel[3:0]` | in | one-hot: a valid slot of thread t commits at this edge (from loom_core's ring) |
| `cm_sr_we`/`cm_sr`, `cm_cnt_we`/`cm_cnt`, `cm_crc_we`/`cm_crc` | in | SR, CNT, CRC written by SHO, SHI, LDSR, CRCI or CSRW |
| `cm_cfg_we`, `cm_pins_we`, `cm_reload_we`, `cm_poly_we`, `cm_init_we`, `cm_csr[15:0]` | in | CSRW of a configuration CSR, with its 16-bit value. `cm_cfg_we` also clears the encoder state (6.9.1) |
| `cm_enc_we`, `cm_enc[7:0]` | in | the encoder state after an SHO or SHI, from the X stage |
| `h_sel[3:0]`, `h_*_we`, `h_wdata[15:0]` | in | host debug writes (one-hot thread, only while it is not running); the commit wins if both ever hit one register. `h_cfg_we` (debug 0x14) clears the encoder state, `h_enc_we` is debug 0x27 |
| `h_reset[3:0]` | in | CTRL.RESET of a thread: clears its encoder state, last in the priority chain |
| `sr_all`, `cnt_all`, `crc_all`, `cfg_all`, `pins_all`, `reload_all`, `poly_all`, `init_all`, `enc_all` | out | register values per thread, what an instruction in X or a debug read sees. `cfg_all` is `{DIFF, STUFF[1:0], ENC[1:0], CRC_EN, INV, DIR}` and `enc_all` is `{FIRST, HALF, PEND, RVAL, RUN[2:0], LVL}`, eight bits each |

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
