"""The M2 firmware programs assemble cleanly and their schedules are proved.

Check ID L2-DEADLINE for the M2 programs: each must assemble with --strict,
with no diagnostic at all (no infeasible pair, no unbounded warning), for the
tick period its ``.tick`` declares, and use only instructions the golden
model builds with ``features={"FIFO"}`` (docs/SEMANTICS.md 6.7).
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load

ISA = load()
FIRMWARE = REPO / "firmware"

#: program -> (declared tick in clocks, words) as documented in firmware/README.md
PROGRAMS = {
    "uart_tx_fifo": (24, 18),
    "uart_rx": (8, 47),
    "spi_master": (28, 72),
    "i2c_master": (5, 105),
}
M2_BUILT = {"PUSH", "POP", "WAITB"}


@pytest.fixture(scope="module", params=sorted(PROGRAMS))
def program(request):
    return request.param, assemble_file(FIRMWARE / (request.param + ".loom"),
                                        isa=ISA, strict=True)


def test_assembles_with_no_diagnostic(program):
    name, prog = program
    assert prog.diagnostics == []
    assert list(prog.threads) == [0]
    assert len(prog.words) == PROGRAMS[name][1]


def test_every_deadline_pair_is_proved(program):
    name, prog = program
    report = prog.deadlines[0]
    assert report.period == PROGRAMS[name][0]
    assert report.pairs and not report.infeasible and not report.unbounded
    assert report.worst_slack >= 0


def test_only_instructions_the_m2_model_builds(program):
    name, prog = program
    for addr, word in prog.words.items():
        decoded = ISA.decode(word)
        assert decoded is not None, hex(addr)
        instr, ops = decoded
        assert instr.cls not in ("be",) and instr.optional is None, instr.name
        if instr.name == "WAITB":
            assert ops["cond"] != 0                   # BE_IDLE needs the bit engine
        if instr.name == "SETP":
            assert ops.get("lat", 0) == 0             # no SETP ... D: not built yet
        if instr.cls == "fifo" or instr.name == "WAITB":
            assert instr.name in M2_BUILT


def test_programs_fit_thread_0_of_a_512_word_build(program):
    """D-020 makes the memory 512 words: thread 0 owns 0..127."""
    name, prog = program
    assert max(prog.words) < 512 // 4
