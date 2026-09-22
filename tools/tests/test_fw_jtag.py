"""L3-JTAG-MASTER on the model and the RTL: TAP reset and an IDCODE read.

``firmware/jtag_master.loom`` is loaded and driven entirely through
``tools.loomhost.Loom``, with ``tools.protomodels.jtag.JtagTap`` - the whole
sixteen-state IEEE 1149.1 controller - on the four pins. Every body takes a
``backend`` (``tools/tests/fw_backend.py``), so the same bodies run here on
the golden model and under cocotb on the RTL (``test/test_fw.py``).

The TAP is the reference: it samples TMS and TDI on the rising edge of TCK
and moves TDO on the falling edge, so a master that got the phase wrong
would read shifted or stale bits rather than the IDCODE. Modelled rate:
TICK_INT 32, one tick per half period, so TCK is 781 kHz at 50 MHz. One
operation is 44 TCK periods: five Test-Logic-Reset clocks, four to reach
Shift-DR, 32 shifts and three to get back to Run-Test/Idle.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.jtag import JtagTap

ISA = load()
FIRMWARE = REPO / "firmware"

TICK = 32                                   # clocks per half TCK period
IDCODE = 0x1BA00477                         # an ARM DAP: version 1, part 0xBA00
READ = 0x0000                               # the one command word


def setup(backend, idcode=IDCODE):
    bench = backend.bench()
    tap = bench.add(JtagTap("OUT0", "OUT1", "OUT2", "IN0", idcode=idcode))
    program = assemble_file(FIRMWARE / "jtag_master.loom", isa=ISA, strict=True)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program)
    loom.write_csr(0, "TICK_INT", TICK)
    loom.run(0)
    return bench, tap, loom


def word_pair(words):
    """The two result words back into one 32-bit IDCODE."""
    return words[0] | (words[1] << 16)


def test_jtag_resets_the_tap_and_reads_the_idcode(backend):
    bench, tap, loom = setup(backend)
    loom.push(0, [READ])
    assert word_pair(loom.pop(0, 2)) == IDCODE
    # the master really walked the state machine, it did not guess
    assert tap.reset_count >= 1
    assert tap.reached("Shift-DR") and tap.reached("Update-DR")
    assert not tap.reached("Shift-IR")       # no IR scan is needed after reset
    assert tap.visited[-1] == "Run-Test/Idle"
    # 32 shifts for the data plus the one the exit clock carries
    assert tap.shifts == 33
    assert loom.badop() == 0


@pytest.mark.parametrize("idcode", [0x4BA00477, 0x00000001])
def test_jtag_reads_any_idcode_bit_pattern(backend, idcode):
    """Bit 0 of a real IDCODE is 1; every other bit is the part's business."""
    bench, tap, loom = setup(backend, idcode=idcode)
    loom.push(0, [READ])
    assert word_pair(loom.pop(0, 2)) == idcode


def test_jtag_a_second_command_resets_the_tap_again(backend):
    """TCK simply stops between operations: the master owns the clock."""
    bench, tap, loom = setup(backend)
    loom.push(0, [READ])
    assert word_pair(loom.pop(0, 2)) == IDCODE
    loom.push(0, [READ])
    assert word_pair(loom.pop(0, 2)) == IDCODE
    assert tap.reset_count >= 2
    assert loom.badop() == 0


def test_jtag_master_assembles_strict_with_no_diagnostic():
    """L2-DEADLINE for jtag_master.loom at its declared tick."""
    program = assemble_file(FIRMWARE / "jtag_master.loom", isa=ISA, strict=True)
    assert program.diagnostics == []
    report = program.deadlines[0]
    assert report.period == TICK
    assert report.pairs and not report.infeasible and not report.unbounded
    assert report.worst_slack >= 0
    assert list(program.threads) == [0]
    assert max(program.words) < 512 // 4


def test_jtag_tap_model_follows_the_standard_state_machine():
    """Five TMS-high clocks reach Test-Logic-Reset from anywhere."""
    from tools.protomodels.jtag import NEXT, TLR
    for start in NEXT:
        state = start
        for _ in range(5):
            state = NEXT[state][1]
        assert state == TLR, start
