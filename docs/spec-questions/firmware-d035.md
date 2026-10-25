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

## 1-Wire (`onewire_master.loom`, `onewire_device.loom`), integrated 2026-10-25

### 1. The master's read sample at E = 8, not AN126's 9

AN126 samples 15 us after the fall, exactly where the DS18B20's tRDV
guarantee ends. Loom sees the line 2 to 5 clocks after the deadline (the
instruction after `WAITD`, then the synchroniser), so E = 9 would be up to
0.1 us late against a part that holds its 0 for exactly the guaranteed
time, and the model DS18B20 does.

**Decision:** E = 8 accepted (14.04 to 14.10 us after the fall). A master
must sample before tRDV ends; AN126's value is a recommendation, and the
datasheet's window is the requirement. The program is described as AN126
timing with this one stated departure (its header and `firmware/README.md`).

### 2. A bad ROM CRC ends the search after that ID

**Decision:** accepted. The ID is reported with the residue the CRC unit
computed, then the END marker; a host that wants the rest of the bus
searches again. A search that went on over bad bits could branch without
end. Recorded in the master's header.

### 3. Write-0 low of 60 us, the DS18B20's tLOW0 minimum

**Decision:** kept. Every edge is deadline-latched, so the low is 60.00 us
to the clock and never less; AN126 uses the same value.

### 4. The device calls a 240 us low a reset

**Decision:** accepted. It lies between the longest slot (120 us) and the
shortest reset (480 us) with margin both ways. Recorded in the device's
header.

### 5. WRITE SCRATCHPAD keeps the whole configuration byte

**Decision:** accepted and documented as a difference from a DS18B20 (which
keeps bits 6:5): the device's scratchpad belongs to the host.

### 6. Both programs use thread 2's quarter for their slot subroutines

The master is 157 words and the device 149 plus 8 of data; neither fits
128. **Decision:** accepted. They load and run together on threads 0 and 1,
thread 3's quarter stays free, and thread 2 must not be started with either
loaded. The headers and `firmware/README.md` say so.

### 7. The device's report is a blocking PUSH

**Decision:** accepted, as for `ir_nec_rx`: the host has four words of room
(two WRITE SCRATCHPADs), and a pop on the SPI port takes about 200 clocks.

### 8. Not done: overdrive, strong pull-up, ALARM SEARCH, CONVERT T, COPY, RECALL

**Decision:** out of scope for D-035 and listed under "Weaker than" in each
section.

### 9. RTL speed and the time-scaled build

**Decision:** accepted. The RTL default set is the four real-rate scenarios
(resets, presence pulses, both READ ROMs); the searches and scratchpads run
on the model, also on a build with `US = 10`, which the deadline checker
proves at that tick.
