# SPDX-License-Identifier: Apache-2.0
"""Data memory: ``LD`` and ``ST`` on the instruction memory
(docs/SEMANTICS.md 6.11, M3 slice B, D-027).

`LD rd, ra, imm5` reads and `ST rd, ra, imm5` writes the word at
``a = (ra + imm5) mod 2**16``, taken modulo ``IMEM_WORDS``. Each is two
slots of its own thread and no cycle of any other's:

* first slot: computes ``a``, keeps ``PC``, sets ``MEM_PEND``, ``MEM_LD``,
  ``MEM_RD`` and ``WAIT_ACTIVE``; the access itself rides the thread's very
  next F cycle, so the write lands one clock later;
* completion slot: decodes nothing, writes ``r[MEM_RD]`` for ``LD``, clears
  ``MEM_PEND`` and ``WAIT_ACTIVE`` and takes ``PC <= next``. ``STEPS``
  counts both.

Everything here is driven through the pins, and every expected value is
computed from that text, never from the golden model (VERIFICATION.md
METH-1). Timing is read on the pads with ``PadMonitor``, as in
``test_setpd``: an ordinary ``SETP OUT1, 1`` marker with X cycle ``x_m``
shows in entry ``m = x_m + 2``, and the thread's later slots have X cycles
``x_m + 4``, ``x_m + 8``, ... So an instruction two slots after the marker
writes its pin in entry ``m + 8``, and one more slot later in ``m + 12``.

Memory map used below (every address is under 256, so the same programs run
on the 512-word macro build and on the 256-word FLOPS build):

    0x00..0x1F  thread 0 programs        0x40..0x4F  thread 1 programs
    0x50..0x5F  threads 2, 3 programs    0x60..0x7F  data

The data area is always seeded before it is used. It has to be: the word an
``ST``'s completion slot receives is unspecified (6.11, section 8), and on
the FLOPS backend that word is the *old* contents of the address written,
which is X for an address the host never loaded -- and ``test/tb.v`` stops
the run on an X in a valid slot's D stage.
"""

import os

import cocotb
from cocotb.triggers import ClockCycles

from spi_host import (
    LoomHost, PadMonitor, asm, run_snippet, SP_CTRL, SP_IMEM,
    CTRL_CAPS, CTRL_VERSION, CTRL_PIN_OUT, CAPS_DMEM, CSR_FLAGS, CSR_TD,
    DBG_PC, DBG_STEPS, DBG_WAIT_ACTIVE, DBG_FLAGS, DBG_MEM,
)

#: VERSION from M3 slice B on (SEMANTICS 6.11; slice A was 3).
VERSION_SLICE_B = 0x0004

OUT0, OUT1, OUT2 = 16, 17, 18

PROG0, PROG1, PROG23, DATA = 0x00, 0x40, 0x50, 0x60
DATA_WORDS = 16

#: Debug 0x28 fields (docs/HOST_PROTOCOL.md SPACE 4, SEMANTICS 6.11):
#: {11'b0, MEM_PEND, MEM_LD, MEM_RD[2:0]}.
MEM_PEND, MEM_LD = 1 << 4, 1 << 3


def data_word(i):
    """A distinctive, non-X word for data slot ``i``."""
    return (0xD000 | (i << 4) | i) ^ 0x0A05


async def seed_data(host, count=DATA_WORDS):
    """Fill the data area and read it back."""
    words = [data_word(i) for i in range(count)]
    await host.write(SP_IMEM, DATA, words)
    got = await host.read(SP_IMEM, DATA, count)
    assert got == words, f"data area readback {got}"
    return words


def rises(mon, bit, start):
    return [c for c, v in mon.changes(mon.uo, bit, start) if v]


async def fresh(dut):
    """A started host on a build that reports the data memory."""
    host = LoomHost(dut)
    await host.start()
    caps = await host.read1(SP_CTRL, CTRL_CAPS)
    assert caps & CAPS_DMEM, \
        f"CAPS {caps:#06x}: the build must report the data memory (CAPS[5])"
    return host


async def arm(host, prog, thread=0, base=PROG0, regs=None):
    """Load a program and point a halted thread at it, without running it.

    STEPS is not one of the registers CTRL.RESET clears (SEMANTICS 7), so it
    is zeroed here: the stepping tests below count slots from zero.
    """
    await host.load_program(prog, verify=False)
    await host.set_reset_pc(thread, base)
    await host.reset_thread(thread)
    await host.write_debug(thread, DBG_STEPS, 0)
    for n, v in (regs or {}).items():
        await host.write_reg(thread, n, v)


# ------------------------------------------------------------------ build
@cocotb.test()
async def test_mem_caps_and_version(dut):
    """CAPS[5] reads 1 from slice B on and VERSION has advanced (6.11)."""
    host = LoomHost(dut)
    await host.start()
    caps = await host.read1(SP_CTRL, CTRL_CAPS)
    assert caps & CAPS_DMEM, f"CAPS {caps:#06x} must set bit 5 (data memory)"
    version = await host.read1(SP_CTRL, CTRL_VERSION)
    assert version == VERSION_SLICE_B, \
        f"VERSION {version:#06x}, expected {VERSION_SLICE_B:#06x}"


# -------------------------------------------------------------- LD and ST
@cocotb.test()
async def test_ld_reads_what_the_host_wrote(dut):
    """`LD` returns the word the host loaded at that address, and sets no
    BADOP: LD/ST are built, so section 9 no longer applies to them."""
    host = await fresh(dut)
    words = await seed_data(host)
    await host.clear_badop()
    for off in (0, 1, 5, 15):
        await run_snippet(host, [asm("LD", rd=1, ra=2, imm=off)],
                          regs={1: 0, 2: DATA})
        got = await host.read_reg(0, 1)
        assert got == words[off], \
            f"LD r1, r2, {off} gave {got:#06x}, IMEM[{DATA + off:#04x}] " \
            f"is {words[off]:#06x}"
    assert await host.badop() == 0, "LD is built: no BADOP (SEMANTICS 9)"


@cocotb.test()
async def test_a_loaded_setd_word_never_moves_td(dut):
    """6.11: the completion slot decodes nothing, so a loaded word that
    happens to encode `SETD` leaves TD where it was. The X stage still works
    out a `SETD` target from that word; only the decode gate on the TD write
    keeps it out. The mutant that writes TD on every commit
    (loom_timer_L0146C023_stuck_79d2) passed every other check."""
    host = await fresh(dut)
    await seed_data(host)
    setd = asm("SETD", imm=200)
    await host.write(SP_IMEM, DATA, [setd])
    await run_snippet(host, [asm("CSRR", rd=3, csr=CSR_TD),
                             asm("LD", rd=1, ra=2, imm=0),
                             asm("CSRR", rd=4, csr=CSR_TD)],
                      regs={1: 0, 2: DATA, 3: 0, 4: 0})
    got = await host.read_reg(0, 1)
    assert got == setd, f"LD gave {got:#06x}, the word is {setd:#06x}"
    before = await host.read_reg(0, 3)
    after = await host.read_reg(0, 4)
    assert after == before,         f"TD {before:#06x} before the LD and {after:#06x} after it: the "         f"completion slot decoded the SETD it loaded"


@cocotb.test()
async def test_st_then_ld_same_word(dut):
    """`ST` writes the word `r[rd]` holds, the host reads it back, and an
    `LD` of the same address in the next instruction sees it."""
    host = await fresh(dut)
    await seed_data(host)
    await host.clear_badop()
    for off, value in ((3, 0xBEEF), (0, 0x0001), (12, 0xFFFF), (7, 0x0000)):
        await run_snippet(host, [
            asm("ST", rd=1, ra=2, imm=off),
            asm("LD", rd=4, ra=2, imm=off),
        ], regs={1: value, 2: DATA, 4: 0x5555})
        assert await host.read_reg(0, 4) == value, \
            f"LD after ST of {value:#06x} at {DATA + off:#04x}"
        back = await host.read1(SP_IMEM, DATA + off)
        assert back == value, \
            f"host read of {DATA + off:#04x} is {back:#06x}, ST wrote {value:#06x}"
        # ST writes no register and leaves r1 alone.
        assert await host.read_reg(0, 1) == value
    assert await host.badop() == 0, "ST is built: no BADOP"


@cocotb.test()
async def test_mem_address_arithmetic(dut):
    """`a = (ra + imm5) mod 2**16`, taken modulo IMEM_WORDS; imm5 is
    zero-extended, so the offset is 0..31 and never negative."""
    host = await fresh(dut)
    words = await seed_data(host)
    imem_words = 1 << ((await host.read1(SP_CTRL, CTRL_CAPS)) >> 12)

    # ra + imm5 with the same address reached three ways.
    for base, off in ((DATA, 9), (DATA + 9, 0), (DATA + 4, 5)):
        await run_snippet(host, [asm("LD", rd=1, ra=2, imm=off)],
                          regs={1: 0, 2: base})
        assert await host.read_reg(0, 1) == words[9], \
            f"LD r1, {base:#x}, {off}"

    # The largest imm5 the encoding has: base + 31 = DATA + 15.
    await run_snippet(host, [asm("LD", rd=1, ra=2, imm=31)],
                      regs={1: 0, 2: DATA + 15 - 31})
    assert await host.read_reg(0, 1) == words[15]

    # Modulo IMEM_WORDS: 0x260 and 0x460 are 0x60 in both the 512-word and
    # the 256-word build.
    for alias in (0x260, 0x460):
        assert alias % imem_words == DATA, "the test's alias must fold to DATA"
        await run_snippet(host, [asm("LD", rd=1, ra=2, imm=2)],
                          regs={1: 0, 2: alias})
        assert await host.read_reg(0, 1) == words[2], \
            f"LD through the alias {alias:#x}"

    # Wrap at 2**16: 0xFFFF + 1 is address 0, which holds a program word --
    # the data memory is the instruction memory. The program is loaded at
    # PROG0 = 0, so IMEM[0] is its own first instruction.
    first = asm("LD", rd=1, ra=2, imm=1)
    await run_snippet(host, [first], regs={1: 0, 2: 0xFFFF})
    got = await host.read_reg(0, 1)
    assert got == first, \
        f"(0xFFFF + 1) mod 2**16 must be address 0: got {got:#06x}, " \
        f"IMEM[0] is {first:#06x}"


@cocotb.test()
async def test_mem_state_and_flags(dut):
    """A completed access leaves MEM_PEND and WAIT_ACTIVE clear, advances PC
    by one and counts two slots in STEPS; flags are untouched."""
    host = await fresh(dut)
    await seed_data(host)
    # Set all three flags, then run an LD and an ST: 6.11 leaves flags alone.
    await run_snippet(host, [
        asm("CSRW", csr=CSR_FLAGS, ra=3),   # {T, C, Z} = 0b111
        asm("LD", rd=1, ra=2, imm=6),
        asm("ST", rd=1, ra=2, imm=7),
    ], regs={1: 0, 2: DATA, 3: 0x7})
    assert await host.read_debug(0, DBG_FLAGS) == 0x7, \
        "LD/ST must leave the flags alone (6.11)"
    # 6.11 has the completion slot clear MEM_PEND and says nothing about
    # MEM_LD and MEM_RD, so those hold what the last access set, exactly
    # as LAT_PIN and LAT_VAL do when LAT_VALID clears (HOST_PROTOCOL
    # 0x25). rtl-m3b.md question 2.
    assert await host.read_debug(0, DBG_MEM) == 1, \
        "MEM_PEND clear; MEM_LD and MEM_RD left by the ST of r1"
    assert await host.read_debug(0, DBG_WAIT_ACTIVE) == 0
    # 4 instructions: CSRW (1) + LD (2) + ST (2) + HALT (1) = 6 slots.
    assert await host.read_debug(0, DBG_PC) == 4
    assert await host.read_debug(0, DBG_STEPS) == 6, \
        "STEPS counts both slots of an LD and of an ST (6.11)"


# ------------------------------------------------------------ slot timing
async def _marked_run(host, mon, middle, extra=None, run_mask=None):
    """Run `SETP OUT1,1; <middle>; SETP OUT0,1; SETP OUT1,0; HALT` on thread
    0 and return the entries of OUT0's rise relative to the marker."""
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    prog = {PROG0 + 0: asm("SETP", pin=OUT1, val=1),
            PROG0 + 1: middle,
            PROG0 + 2: asm("SETP", pin=OUT0, val=1),
            PROG0 + 3: asm("SETP", pin=OUT1, val=0),
            PROG0 + 4: asm("HALT")}
    prog.update(extra or {})
    await host.load_program(prog, verify=False)
    await host.set_reset_pc(0, PROG0)
    await host.reset_thread(0)
    await host.write_reg(0, 1, 0)
    await host.write_reg(0, 2, DATA)
    start = mon.now
    await host.run(run_mask if run_mask is not None else 0b0001)
    await host.wait_halted(0b0001)
    m = rises(mon, 1, start)[0]
    return [c - m for c in rises(mon, 0, start)]


@cocotb.test()
async def test_mem_takes_exactly_two_slots(dut):
    """On the pads: the instruction after an `LD` or an `ST` commits two
    slots after the memory instruction's own first slot, where after a `NOP`
    it commits one slot after. Both are measured against a `SETP` marker."""
    host = await fresh(dut)
    await seed_data(host)
    mon = PadMonitor(dut).start()

    got = await _marked_run(host, mon, asm("NOP"))
    assert got == [8], f"after a NOP the next SETP writes at m+8, got m+{got}"

    for name in ("LD", "ST"):
        got = await _marked_run(host, mon,
                                asm(name, rd=1, ra=2, imm=0))
        assert got == [12], \
            f"after a {name} the next SETP writes at m+12, got m+{got}"

    # "never late": the pair is the same length whatever the other threads
    # do. Threads 1..3 spin in their own quarters, writing no pin.
    spin = {PROG1: asm("JMP", abs=PROG1),
            PROG23: asm("JMP", abs=PROG23),
            PROG23 + 1: asm("JMP", abs=PROG23 + 1)}
    for t, pc in ((1, PROG1), (2, PROG23), (3, PROG23 + 1)):
        await host.set_reset_pc(t, pc)
    await host.load_program(spin, verify=False)
    for t in (1, 2, 3):
        await host.reset_thread(t)
    got = await _marked_run(host, mon, asm("LD", rd=1, ra=2, imm=0),
                            extra=spin, run_mask=0b1111)
    assert got == [12], \
        f"an LD with three other threads running still costs two slots: m+{got}"
    await host.halt()
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_mem_does_not_move_another_thread(dut):
    """A stream of `LD`/`ST` in thread 0 does not move thread 1's slots by a
    single clock: no cycle of another thread's is used (6.11)."""
    host = await fresh(dut)
    await seed_data(host)
    mon = PadMonitor(dut).start()

    # Thread 1: a three-slot loop, so OUT2 rises every 12 clocks.
    blink = {PROG1 + 0: asm("SETP", pin=OUT2, val=1),
             PROG1 + 1: asm("SETP", pin=OUT2, val=0),
             PROG1 + 2: asm("JMP", abs=PROG1)}
    # Thread 0: a five-slot loop with an LD and an ST in it.
    hammer = {PROG0 + 0: asm("LD", rd=1, ra=2, imm=0),
              PROG0 + 1: asm("ST", rd=1, ra=2, imm=1),
              PROG0 + 2: asm("JMP", abs=PROG0)}
    await host.load_program({**blink, **hammer}, verify=False)
    await host.set_reset_pc(0, PROG0)
    await host.set_reset_pc(1, PROG1)

    gaps = {}
    for label, mask in (("alone", 0b0010), ("with LD/ST", 0b0011)):
        await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
        await host.reset_thread(0)
        await host.reset_thread(1)
        await host.write_reg(0, 1, 0)
        await host.write_reg(0, 2, DATA)
        start = mon.now
        await host.run(mask)
        await ClockCycles(dut.clk, 400)
        await host.halt()
        await ClockCycles(dut.clk, 20)
        edges = rises(mon, 2, start)
        assert len(edges) >= 8, f"{label}: only {len(edges)} OUT2 rises"
        gaps[label] = sorted(set(b - a for a, b in zip(edges, edges[1:])))
        assert gaps[label] == [12], \
            f"{label}: thread 1's period is {gaps[label]}, expected [12]"
    assert gaps["alone"] == gaps["with LD/ST"]
    mon.stop()
    await ClockCycles(dut.clk, 2)


# --------------------------------------------------------------- stepping
@cocotb.test()
async def test_mem_single_step_through_an_access(dut):
    """Stepping twice runs the two slots one at a time, and debug 0x28 shows
    MEM_PEND, MEM_LD and MEM_RD in between (6.11, SEMANTICS 7)."""
    host = await fresh(dut)
    words = await seed_data(host)

    for name, want_ld in (("LD", True), ("ST", False)):
        await arm(host, {PROG0: asm(name, rd=3, ra=2, imm=4),
                         PROG0 + 1: asm("HALT")},
                  regs={2: DATA, 3: 0x1234 if name == "ST" else 0})
        assert await host.read_debug(0, DBG_MEM) & MEM_PEND == 0, \
            "armed, nothing pending"

        await host.step(0)
        await ClockCycles(dut.clk, 20)
        mem = await host.read_debug(0, DBG_MEM)
        want = MEM_PEND | (MEM_LD if want_ld else 0) | 3      # MEM_RD = r3
        assert mem == want, \
            f"{name}: debug 0x28 is {mem:#06x} between the slots, expected {want:#06x}"
        assert await host.read_debug(0, DBG_WAIT_ACTIVE) == 1
        assert await host.read_debug(0, DBG_PC) == PROG0, "PC is held"
        assert await host.read_debug(0, DBG_STEPS) == 1
        if want_ld:
            assert await host.read_reg(0, 3) == 0, "nothing is written yet"

        await host.step(0)
        await ClockCycles(dut.clk, 20)
        # MEM_PEND clears; MEM_LD and MEM_RD hold (rtl-m3b.md question 2).
        assert await host.read_debug(0, DBG_MEM) == want & ~MEM_PEND, \
            f"{name}: the completion slot clears MEM_PEND and only that"
        assert await host.read_debug(0, DBG_WAIT_ACTIVE) == 0
        assert await host.read_debug(0, DBG_PC) == PROG0 + 1
        assert await host.read_debug(0, DBG_STEPS) == 2, \
            "stepping twice is the same two slots as running them"
        if want_ld:
            assert await host.read_reg(0, 3) == words[4], \
                "the stepped LD wrote the word at its address"
        else:
            assert await host.read1(SP_IMEM, DATA + 4) == 0x1234
            words[4] = 0x1234


@cocotb.test()
async def test_mem_step_survives_another_threads_accesses(dut):
    """A thread stepped through the first slot of an `LD` keeps its access
    while another thread runs a stream of `LD`/`ST`, and the second `STEP`
    still returns the right word.

    This is the case that decides where the access is held: one register
    shared by the four threads would be overwritten by thread 1's stream
    between thread 0's two steps (architect's ruling of 2026-09-22,
    `docs/spec-questions/rtl-m3b.md` question 1).
    """
    host = await fresh(dut)
    words = await seed_data(host)
    # Thread 0: one LD, stepped. Thread 1: an LD/ST loop of its own, on
    # different words, free running in between.
    await host.load_program({
        PROG0 + 0: asm("LD", rd=3, ra=2, imm=4),
        PROG0 + 1: asm("HALT"),
        PROG1 + 0: asm("LD", rd=1, ra=4, imm=10),
        PROG1 + 1: asm("ST", rd=1, ra=4, imm=11),
        PROG1 + 2: asm("JMP", abs=PROG1),
    }, verify=False)
    await host.set_reset_pc(0, PROG0)
    await host.set_reset_pc(1, PROG1)
    await host.reset_thread(0)
    await host.reset_thread(1)
    await host.write_debug(0, DBG_STEPS, 0)
    await host.write_reg(0, 2, DATA)
    await host.write_reg(0, 3, 0)
    await host.write_reg(1, 4, DATA)

    await host.step(0)
    await ClockCycles(dut.clk, 20)
    want = MEM_PEND | MEM_LD | 3
    assert await host.read_debug(0, DBG_MEM) == want, "thread 0's access is armed"

    # Thread 1 hammers the memory with its own accesses.
    await host.run(0b0010)
    await ClockCycles(dut.clk, 400)
    await host.halt()
    await ClockCycles(dut.clk, 20)
    assert await host.read_debug(1, DBG_STEPS) > 20, "thread 1 really ran"
    assert await host.read_debug(0, DBG_MEM) == want, \
        "thread 0's pending access must survive thread 1's stream"
    assert await host.read_debug(0, DBG_STEPS) == 1, "thread 0 ran one slot"

    await host.step(0)
    await ClockCycles(dut.clk, 20)
    assert await host.read_debug(0, DBG_MEM) == want & ~MEM_PEND
    assert await host.read_reg(0, 3) == words[4], \
        "the second step must load the word thread 0 addressed"
    assert await host.read_debug(0, DBG_PC) == PROG0 + 1
    assert await host.read_debug(0, DBG_STEPS) == 2


@cocotb.test()
async def test_mem_reset_clears_a_pending_access(dut):
    """`CTRL.RESET` clears MEM_PEND, so a thread never completes an access it
    did not start (6.11, SEMANTICS 7). The thread is stepped once first, so
    it is halted when the reset arrives, as section 7 requires."""
    host = await fresh(dut)
    words = await seed_data(host)
    await arm(host, {PROG0: asm("LD", rd=3, ra=2, imm=2),
                     PROG0 + 1: asm("HALT")},
              regs={2: DATA, 3: 0xA5A5})
    await host.step(0)
    await ClockCycles(dut.clk, 20)
    assert await host.read_debug(0, DBG_MEM) == MEM_PEND | MEM_LD | 3

    await host.reset_thread(0)
    await ClockCycles(dut.clk, 20)
    assert await host.read_debug(0, DBG_MEM) & MEM_PEND == 0, \
        "CTRL.RESET clears MEM_PEND"
    assert await host.read_debug(0, DBG_WAIT_ACTIVE) == 0
    assert await host.read_debug(0, DBG_PC) == PROG0
    assert await host.read_reg(0, 3) == 0xA5A5, "the abandoned LD wrote nothing"

    # The thread starts again from the top and the access runs whole.
    await host.run(0b0001)
    await host.wait_halted(0b0001)
    assert await host.read_reg(0, 3) == words[2]
    assert await host.read_debug(0, DBG_MEM) & MEM_PEND == 0


@cocotb.test()
async def test_mem_debug_pc_write_clears_a_pending_access(dut):
    """A debug write of PC clears MEM_PEND and WAIT_ACTIVE (SEMANTICS 7)."""
    host = await fresh(dut)
    await seed_data(host)
    await arm(host, {PROG0: asm("LD", rd=3, ra=2, imm=1),
                     PROG0 + 1: asm("HALT"),
                     PROG0 + 8: asm("LDI", rd=5, imm=0x77),
                     PROG0 + 9: asm("HALT")},
              regs={2: DATA, 3: 0xA5A5, 5: 0})
    await host.step(0)
    await ClockCycles(dut.clk, 20)
    assert await host.read_debug(0, DBG_MEM) & MEM_PEND

    await host.write_debug(0, DBG_PC, PROG0 + 8)
    await ClockCycles(dut.clk, 20)
    assert await host.read_debug(0, DBG_MEM) & MEM_PEND == 0
    assert await host.read_debug(0, DBG_WAIT_ACTIVE) == 0
    assert await host.read_debug(0, DBG_PC) == PROG0 + 8

    await host.run(0b0001)
    await host.wait_halted(0b0001)
    assert await host.read_reg(0, 5) == 0x77, "the thread ran on from the new PC"
    assert await host.read_reg(0, 3) == 0xA5A5, "the abandoned LD wrote nothing"


@cocotb.test()
async def test_mem_debug_0x28_is_writable(dut):
    """Debug 0x28 is a plain register while the thread is halted."""
    host = await fresh(dut)
    await seed_data(host)
    await arm(host, {PROG0: asm("HALT")})
    for value in (MEM_PEND | MEM_LD | 5, MEM_LD | 2, 0):
        await host.write_debug(0, DBG_MEM, value)
        got = await host.read_debug(0, DBG_MEM)
        assert got == value, f"0x28 wrote {value:#06x}, read {got:#06x}"
    # Bits above 4 read 0.
    await host.write_debug(0, DBG_MEM, 0xFFFF)
    assert await host.read_debug(0, DBG_MEM) == 0x1F
    await host.write_debug(0, DBG_MEM, 0)


# ------------------------------------------------------- self-modifying
@cocotb.test()
async def test_st_writes_an_instruction_the_thread_then_fetches(dut):
    """A word written by `ST` is an instruction word like any other: a thread
    that writes its own next instruction sees the new word (6.11). The same
    program is run twice, storing a different instruction each time."""
    host = await fresh(dut)
    old_word = asm("LDI", rd=3, imm=0x11)
    new_word = asm("LDI", rd=3, imm=0x5A)
    prog = {PROG0 + 0: asm("ST", rd=1, ra=2, imm=1),   # target is PROG0 + 1
            PROG0 + 1: old_word,                       # overwritten at run time
            PROG0 + 2: asm("HALT")}
    for word, want in ((old_word, 0x11), (new_word, 0x5A)):
        await host.load_program(prog, verify=False)    # restore the original
        await host.set_reset_pc(0, PROG0)
        await host.reset_thread(0)
        await host.write_reg(0, 1, word)
        await host.write_reg(0, 2, PROG0)
        await host.write_reg(0, 3, 0)
        await host.run(0b0001)
        await host.wait_halted(0b0001)
        assert await host.read_reg(0, 3) == want,             f"the instruction after the ST must be {word:#06x}"
        assert await host.read1(SP_IMEM, PROG0 + 1) == word


# ------------------------------------------------------------ FLOPS build
class _FlopsDut:
    """``tb``'s second instance, the flip-flop instruction memory (D-020),
    under the names ``spi_host`` uses. Same mapping as ``test_flops.py``'s
    ``FlopsDut``; it is repeated here so this module does not pull in the
    co-simulation harness at import time."""

    def __init__(self, tb):
        self.clk = tb.clk_flops
        self.rst_n = tb.rst_n_flops
        self.ena = tb.ena_flops
        self.ui_in = tb.ui_in_flops
        self.uio_drv = tb.uio_drv_flops
        self.uo_out = tb.uo_out_flops
        self.uio_out = tb.uio_out_flops
        self.uio_oe = tb.uio_oe_flops
        self.user_project = tb.user_project_flops
        self._log = tb._log


#: The gate-level netlist is the MACRO build and has no FLOPS instance.
ABSENT = os.environ.get("GATES", "").lower() == "yes"


@cocotb.test(skip=ABSENT)
async def test_mem_on_the_flops_backend(dut):
    """The 256-word flip-flop memory gives the same answers as the macro."""
    try:
        dut.user_project_flops.u_loom.u_imem.g_flops.mem      # noqa: B018
    except AttributeError:
        raise AssertionError("tb.user_project_flops with the flop array "
                             "u_imem.g_flops.mem not found (test/tb.v builds "
                             "it unless GL_TEST)") from None
    fd = _FlopsDut(dut)
    host = await fresh(fd)
    words = await seed_data(host)

    # LD of a host-written word, ST then LD, and the address arithmetic.
    await run_snippet(host, [asm("LD", rd=1, ra=2, imm=5)],
                      regs={1: 0, 2: DATA})
    assert await host.read_reg(0, 1) == words[5]

    await run_snippet(host, [
        asm("ST", rd=1, ra=2, imm=8),
        asm("LD", rd=4, ra=2, imm=8),
    ], regs={1: 0xC0DE, 2: DATA, 4: 0})
    assert await host.read_reg(0, 4) == 0xC0DE
    assert await host.read1(SP_IMEM, DATA + 8) == 0xC0DE

    # 0x260 folds to DATA in a 256-word memory as well.
    await run_snippet(host, [asm("LD", rd=1, ra=2, imm=3)],
                      regs={1: 0, 2: 0x260})
    assert await host.read_reg(0, 1) == words[3]

    # Two slots, stepped, with debug 0x28 in between.
    await arm(host, {PROG0: asm("LD", rd=3, ra=2, imm=6),
                     PROG0 + 1: asm("HALT")},
              regs={2: DATA, 3: 0})
    await host.step(0)
    await ClockCycles(fd.clk, 20)
    assert await host.read_debug(0, DBG_MEM) == MEM_PEND | MEM_LD | 3
    await host.step(0)
    await ClockCycles(fd.clk, 20)
    assert await host.read_debug(0, DBG_MEM) == MEM_LD | 3
    assert await host.read_reg(0, 3) == words[6]
    assert await host.read_debug(0, DBG_STEPS) == 2
