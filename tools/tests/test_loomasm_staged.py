"""The late-staged-edge check of the deadline checker (W-124).

``SETP pin, v, D`` stages a write that lands at the thread's next deadline
(SEMANTICS 6.10). It lands on the tick itself (rule 1, or rule 2 at the tick)
only if the ``WAITD`` that sets that deadline commits by then. Its first issue
has X cycle ``x`` and commits at edge ``x + 2`` (SEMANTICS 2), and a pair the
checker calls feasible with slack ``s`` lets ``x`` be as late as
``3 - s`` clocks after the tick (the slot-grid grace). So the edge can land
``5 - s`` clocks late (rule 2 at ``x + 2``), and is exact only from ``s = 5``.
The brief's figure of 3 came from one 74HC595 run whose thread phase put
the anchor 1 clock after its tick, not 3; the model sweep below shows all
four phases.

The check is off by default and a warning even under ``--strict``, because
nineteen shipped programs trip it (REPORT of W-124, tools/loomasm/README.md).
"""

import pytest

from tools.loomasm import assemble_text
from tools.loomasm.__main__ import main
from tools.loomasm.deadline import staged_exact_slack
from tools.loomisa import load
from tools.loomsim import Machine

ISA = load()


def toggle(period, nops, thread=0):
    """A staged square wave: the pair into each WAITD passes one SETP ... D.

    Into the first WAITD (rising edge): JMP, SETP, ``nops`` NOPs, WAITD =
    ``nops + 3`` slots; into the second (falling edge): ``nops + 2`` slots.
    """
    pad = ["        NOP"] * nops
    return "\n".join([
        "        .imem 512", "        .thread %d" % thread,
        "        .tick %d" % period,
        "        SETD 2",
        "loop:   SETP OUT0, 1, D"] + pad + [
        "        WAITD 1",
        "        SETP OUT0, 0, D"] + pad + [
        "        WAITD 1",
        "        JMP loop", ""])


def asm(text, **kwargs):
    return assemble_text(text, "t.loom", isa=ISA, **kwargs)


def staged_warnings(program):
    return [d for d in program.warnings if "staged edge" in d.message]


def warnings_into(program, pair):
    tag = "-> 0x%03X WAITD" % pair.dst_addr
    return [d for d in staged_warnings(program) if tag in d.message]


def rising_pair(program, thread=0):
    """The WAITD -> first WAITD pair (the one through the JMP)."""
    pairs = [p for p in program.deadlines[thread].pairs
             if p.src_name == "WAITD" and p.dst_addr < p.src_addr]
    assert len(pairs) == 1
    return pairs[0]


def test_the_exact_slack_is_five_clocks_with_four_clock_slots():
    assert staged_exact_slack() == staged_exact_slack(4) == 5


@pytest.mark.parametrize("slack", [0, 1, 2, 3, 4])
def test_a_staged_pair_with_slack_under_five_warns(slack):
    # nops = 1: the rising pair is 4 slots = 16 clocks, the falling one 12.
    program = asm(toggle(16 + slack, 1), staged_check=True)
    pair = rising_pair(program)
    assert pair.staged and pair.slack == pair.staged_slack == slack
    assert pair.staged_late == 5 - slack
    texts = [d.message for d in warnings_into(program, pair)]
    assert len(texts) == 1
    assert ("up to %d clock" % (5 - slack)) in texts[0]
    # the falling pair has slack 4 + s, so it warns (1 clock) only at s = 0
    assert len(staged_warnings(program)) == (2 if slack == 0 else 1)
    assert "add slack or write the pin unstaged" in texts[0]
    assert all(d.severity == "warning" for d in staged_warnings(program))


@pytest.mark.parametrize("slack", [5, 6, 40])
def test_a_staged_pair_with_slack_five_or_more_is_clean(slack):
    program = asm(toggle(16 + slack, 1), staged_check=True)
    assert rising_pair(program).staged_slack == slack
    assert rising_pair(program).staged_late == 0
    assert staged_warnings(program) == []


def test_the_check_is_off_by_default():
    program = asm(toggle(16, 1))
    assert rising_pair(program).staged_late == 5   # analysed all the same
    assert staged_warnings(program) == []
    assert program.warnings == []


def test_an_unstaged_pair_never_warns():
    text = toggle(16, 1).replace(", D", "")
    program = asm(text, staged_check=True)
    assert not rising_pair(program).staged
    assert staged_warnings(program) == []


def test_only_paths_through_the_staged_setp_count():
    """The SETP ... D sits on the short branch: the long branch (no staged
    write) sets the pair's slack, the short one the staged slack."""
    text = "\n".join([
        "        .thread 0", "        .tick 24",
        "        SETD 4",
        "top:    WAITD 1",
        "        BZ long",
        "        SETP OUT0, 1, D",
        "        JMP done",
        "long:   NOP", "        NOP", "        NOP", "        NOP",
        "done:   WAITD 1",
        "        JMP top", ""])
    program = asm(text, staged_check=True)
    pair = [p for p in program.deadlines[0].pairs
            if p.src_name == "WAITD" and p.dst_addr > p.src_addr][0]
    assert pair.slots == 6 and pair.slack == 0          # BZ, 4 NOP, WAITD
    assert pair.staged_slots == 4 and pair.staged_slack == 8  # BZ, SETP, JMP, WAITD
    assert pair.staged_late == 0
    assert staged_warnings(program) == []


def test_a_missed_deadline_is_an_error_and_not_also_a_staged_warning():
    program = asm(toggle(12, 1), staged_check=True)
    pair = rising_pair(program)
    assert pair.feasible is False and pair.staged_late is None
    assert warnings_into(program, pair) == []
    assert any(d.severity == "error" for d in program.diagnostics)


def test_a_setd_anchor_is_checked_the_same_way():
    # SETD 1 right after a WAITD 1: phase <= 7, so the limit is
    # 2 x 8 - 7 + 3 = 12 clocks and two slots (SETP D, WAITD) leave 4.
    text = "\n".join([
        "        .thread 0", "        .tick 8",
        "        SETD 4", "        WAITD 1", "        SETD 1",
        "        SETP OUT0, 1, D", "        WAITD 1", "        HALT", ""])
    program = asm(text, staged_check=True)
    pair = [p for p in program.deadlines[0].pairs if p.src_addr == 2][0]
    assert pair.src_name == "SETD" and pair.staged_slack == 4
    assert pair.staged_late == 1
    assert len(staged_warnings(program)) == 1


def test_the_listing_marks_the_pair_when_the_check_is_on():
    on = asm(toggle(16, 1), staged_check=True).listing_text()
    off = asm(toggle(16, 1)).listing_text()
    assert "(staged edge up to 5 late)" in on
    assert "2 with a late staged edge" in on     # falling pair: slack 4
    assert "staged" not in off


# --------------------------------------------------------------- command line
def test_the_command_line_flag_warns_and_strict_still_passes(tmp_path, capsys):
    source = tmp_path / "p.loom"
    source.write_text(toggle(16, 1), encoding="utf-8")
    out = tmp_path / "p.json"
    assert main([str(source), "-o", str(out), "--strict"]) == 0
    assert "staged edge" not in capsys.readouterr().err
    assert main([str(source), "-o", str(out), "--strict", "--staged-edges"]) == 0
    err = capsys.readouterr().err
    assert "warning: staged edge may land up to 5 clocks late" in err
    assert out.exists()


# ------------------------------------------- the bound, on the golden model
def run_model(text, thread, period, cycles):
    """Pads ``uo_out`` and the cycles at which ``NOW`` ticked."""
    program = asm(text)
    machine = Machine(program.to_image(), features=("SETPD",), isa=ISA,
                      imem_words=512)
    machine.host_write_debug(thread, "TICK_INT", period)
    machine.host_set_run(1 << thread)
    th = machine.threads[thread]
    pads, ticks, now = [], [], th.now
    for _ in range(cycles):
        if th.now != now:
            ticks.append(machine.cycle)
            now = th.now
        pads.append(machine.uo_out)
        machine.step_cycle()
    assert machine.badop == 0
    return pads, ticks


def lateness(pads, ticks, bit, value=None, skip=2):
    """Clocks from the last tick to each edge of ``uo_out[bit]``."""
    level = [(p >> bit) & 1 for p in pads]
    edges = [c for c in range(1, len(level)) if level[c] != level[c - 1]
             and (value is None or level[c] == value)]
    return [e - max(t for t in ticks if t <= e) for e in edges[skip:]]


@pytest.mark.parametrize("slack", [0, 1, 2, 3, 4, 5, 6])
def test_the_model_lands_the_edge_up_to_five_minus_slack_late(slack):
    """Rising edges of ``toggle``, in all four threads (four phases of the
    slot grid against the ticks): never later than 5 - s, and some thread
    reaches it, so 5 (not 3) is the bound SEMANTICS 2 and 6.10 give."""
    period = 16 + slack
    worst = 0
    for thread in range(4):
        text = toggle(period, 1, thread)
        assert rising_pair(asm(text), thread).staged_slack == slack
        late = lateness(*run_model(text, thread, period, period * 40), 0, 1)
        assert len(late) > 10
        assert max(late) <= max(0, 5 - slack)
        worst = max(worst, max(late))
    assert worst == max(0, 5 - slack)


# ------------------------------------------- the 74HC595 case of 2026-11-19
DATA = 0xA5


def hc595(period, thread=0):
    """A 74HC595 shift driver as the porting test wrote it: SER staged so it
    moves exactly on a tick, SRCLK an ordinary SETP one tick later. The
    decision about the next bit runs between the WAITD after SRCLK rises and
    the WAITD the staged SER lands on: 7 slots, 28 clocks, so slack
    ``period - 28``."""
    return "\n".join([
        "        .imem 512", "        .thread %d" % thread,
        "        .tick %d" % period,
        "        .pins SER = OUT0, SRCLK = OUT1",
        "        SETD 4",
        "        LDI r1, 0", "        LDIH r1, 0x%02X" % DATA,
        "        LDI r2, 8",
        "bit:    SHLI r1, 1",
        "        BC one",
        "        SETP SER, 0, D",
        "        JMP go",
        "one:    SETP SER, 1, D",
        "        NOP",
        "go:     WAITD 1",              # SER lands on this deadline
        "        SETP SRCLK, 1",
        "        WAITD 1",
        "        SETP SRCLK, 0",
        "        DJNZ r2, bit",
        "        HALT", ""])


def ser_pair(program, thread):
    return [p for p in program.deadlines[thread].pairs
            if p.src_name == "WAITD" and p.dst_addr < p.src_addr][0]


def shifted_in(pads):
    """SER as a 74HC595 samples it, at each rising edge of SRCLK."""
    srclk = [(p >> 1) & 1 for p in pads]
    rises = [c for c in range(1, len(pads)) if srclk[c] and not srclk[c - 1]]
    value = 0
    for c in rises:
        value = (value << 1) | (pads[c - 1] & 1)
    return value, len(rises)


@pytest.mark.parametrize("period,warns", [(28, True), (33, False)])
def test_the_74hc595_driver(period, warns):
    """Slack 0 passes the deadline check and shifts the right byte, but the
    SER edge lands late in some phase; slack 5 is exact in every phase."""
    worst = 0
    for thread in range(4):
        text = hc595(period, thread)
        program = asm(text, staged_check=True)
        assert all(d.severity != "error" for d in program.diagnostics)
        pair = ser_pair(program, thread)
        assert pair.slack == pair.staged_slack == period - 28
        assert bool(warnings_into(program, pair)) is warns
        pads, ticks = run_model(text, thread, period, period * 24)
        assert shifted_in(pads) == (DATA, 8)          # the bits decode fine
        late = lateness(pads, ticks, 0, skip=0)
        assert late and max(late) <= max(0, 5 - (period - 28))
        worst = max(worst, max(late))
    assert (worst > 0) is warns
