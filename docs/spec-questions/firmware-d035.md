# Spec questions from the D-035 firmware

The programs written after the hardware was complete (D-032) for protocols
Loom was not designed for (D-035). Each agent raised its questions in a
scratch file; the director decided each one at integration. Each item says
the question, the decision and where the code records it. Nothing here
needed an RTL or a golden-model change.

## NEC infrared (`ir_nec_tx.loom`, `ir_nec_rx.loom`), integrated 2026-10-12

### 1. Host-set ticks, and a 1.824 MHz board for the RTL tests

The RTL simulates about 6,400 clocks a second, so a frame and its repeat at
50 MHz (6 M clocks) would take about 15 minutes per scenario. Both programs
leave the tick to the host and the RTL default set runs them on a board
clocked at 1.824 MHz (transmitter TICK 16, receiver TICK 64 + 32/256): the
same waveform in microseconds in 27 times fewer clocks. The 50 MHz bodies
run on the golden model.

**Decision:** accepted. Host-set ticks are the M2 convention (the clock is
the board's), and `jtag_master` and `swd_master` do the same. The program
headers and `firmware/README.md` give the 50 MHz settings.

### 2. The NEC unit as 64 ticks, 0.19 per cent short

At 38 kHz the NEC unit is 64.125 thirds of a carrier period. The program
takes 64, so every time is an exact tick count and the carrier is 38 kHz
within 3 ppm.

**Decision:** accepted. NEC receivers allow 10 per cent and more, and the
receiver's windows (+-25 per cent plus 4 ticks) take it many times over. An
exact carrier is worth more than an exact unit. Recorded in the
`ir_nec_tx.loom` header (Envelope).

### 3. Repeat words outside a chain are dropped silently

**Decision:** accepted as is. A repeat code with no frame before it is
wrong on the wire, and the host that pushed it knows it released the key;
reporting it would give the transmitter its only OUTQ traffic and the host
one more thing to drain. Recorded in the `ir_nec_tx.loom` header (Repeats).

### 4. The receiver does not check that a repeat code follows a frame

**Decision:** accepted as is. The host has the last frame and the time; a
check in firmware costs about eight words and would make the receiver
reject repeats after a frame it missed, which a host may want to see.
Recorded in the `ir_nec_rx.loom` header.

### 5. The receiver blocks on a full OUTQ

A report is never split; a host must pop within about 40 ms, or the next
leader is missed. An overrun flag would need a non-blocking push of two
words, which a thread cannot make atomic with the FIFO status it has.

**Decision:** accepted. 40 ms is long for any host on the SPI port (a pop
is about 200 clocks). Recorded in the `ir_nec_rx.loom` header.

### 6. TSOP figures

The agent took the receiver module's figures from the older TSOP18238
sheet (delay up to 15/f0, output pulse up to +6/f0) and could not open the
TSOP382 sheet. The director checked them on 2026-10-12 against Vishay's
TSOP382.., TSOP384.. datasheet (document 82491, rev. 2.1, 27 May 2025,
figure 1 and its notes): 7/f0 < td < 13/f0, tpi - 5/f0 < tpo < tpi + 5/f0,
bursts of 10 cycles or more.

**Decision:** the `ir_nec_rx.loom` header and the model's docstring quote
the TSOP382 figures with that attribution. The receiver's 4-tick margin
(140.6 us) covers the +-132 us of pulse error. The test at the module's
limits keeps its two cases: the TSOP382 short limits (td 7, -5/f0) and the
TSOP18238 long ones (td 15, +6/f0), which lie beyond the TSOP382 sheet, so
the long TSOP382 limits are covered with margin. `firmware/README.md`
quotes no TSOP figure.

### 7. `test_rekey_from_a_git_revision_runs` fails in agent worktrees

The worktree's `.git` file holds a Windows path that git in WSL cannot
follow. **Decision:** expected there; it passes in main (checked by
`scripts/check_all.sh quick` at integration).
