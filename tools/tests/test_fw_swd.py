"""L3-SWD-MASTER on the model and the RTL: connect, then read DPIDR.

``firmware/swd_master.loom`` is loaded and driven through
``tools.loomhost.Loom``, with ``tools.protomodels.swd.SwdDp`` on SWCLK
(OUT0) and SWDIO (BIDIR0, with a bench pull-up for the turnaround cycles).
Every body takes a ``backend`` (``tools/tests/fw_backend.py``), so the same
bodies run here on the golden model and under cocotb on the RTL
(``test/test_fw.py``).

The DP is the reference: it counts the 50-cycle line reset, only leaves JTAG
mode on the exact 0xE79E select sequence, checks the parity, stop and park
bits of the packet request, and drives SWDIO only in its own half of the
packet. Modelled rate: TICK_INT 32, one tick per half period, so SWCLK is
781 kHz at 50 MHz. One connect is 130 cycles and one packet 54, so a
scenario is about 12000 core clocks.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.swd import (ACK_FAULT, ACK_OK, ACK_WAIT, SWITCH_SEQUENCE,
                                   SwdDp)

ISA = load()
FIRMWARE = REPO / "firmware"

TICK = 32                                   # clocks per half SWCLK period
DPIDR = 0x2BA01477                          # an ARM DP: designer ARM, part 0xBA01
READ = 0x0000                               # the one command word
PARITY_ERR = 0x8                            # status bit 3
SWDIO_PULLUP = 0b1                          # BIDIR0


def setup(backend, **dp):
    bench = backend.bench(pullups=SWDIO_PULLUP)
    dp.setdefault("dpidr", DPIDR)
    port = bench.add(SwdDp("OUT0", "BIDIR0", **dp))
    program = assemble_file(FIRMWARE / "swd_master.loom", isa=ISA, strict=True)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program)
    loom.write_csr(0, "TICK_INT", TICK)
    loom.run(0)
    return bench, port, loom


def read_dpidr(loom):
    """``(value, status)`` from the three result words of one operation."""
    low, high, status = loom.pop(0, 3)
    return low | (high << 16), status


def test_swd_connects_and_reads_the_dpidr(backend):
    bench, port, loom = setup(backend)
    loom.push(0, [READ])
    assert read_dpidr(loom) == (DPIDR, ACK_OK)
    # the connect really happened, in the right order
    assert port.switches == 1 and port.mode == "swd"
    assert port.line_resets >= 2
    # one packet: a DP read of register 0x00, with a valid request byte
    assert port.packets == [(0, 1, 0x00, True)]
    assert port.bad_packets == 0
    assert bench.contention_free()
    assert loom.badop() == 0


class Swclk(Model):
    """Every SWCLK edge, by cycle, with the level it went to."""

    def __init__(self):
        self.pad, self.prev, self.edges = pad_of("OUT0"), None, []

    def observe(self, lines):
        now = lines.get(self.pad)
        if self.prev is not None and now != self.prev:
            self.edges.append((lines.cycle, now))
        self.prev = now


def test_swd_every_swclk_high_phase_is_one_tick(backend):
    """Each high phase of SWCLK is exactly one tick (32 clocks, a multiple of
    the slot), through the connect sequence and a whole read; a low phase is
    at least one tick, and longer where the program pauses the clock with
    SETD 1. The first version paused with SETD 0: a pause that fell late in a
    tick made the next rising edge late and the high phase after it short
    (tools finding T-1, docs/VERIFICATION.md)."""
    bench, port, loom = setup(backend)
    swclk = bench.add(Swclk())
    loom.push(0, [READ])
    assert read_dpidr(loom) == (DPIDR, ACK_OK)
    edges = swclk.edges
    assert len(edges) > 2 * 100                          # the connect and a packet
    highs = [b - a for (a, la), (b, _) in zip(edges, edges[1:]) if la == 1]
    lows = [b - a for (a, la), (b, _) in zip(edges, edges[1:]) if la == 0]
    assert set(highs) == {TICK}, sorted(set(highs))
    assert min(lows) == TICK


def test_swd_a_second_read_needs_no_second_connect(backend):
    """The connect runs once; each host word is one packet."""
    bench, port, loom = setup(backend)
    loom.push(0, [READ, READ])
    assert read_dpidr(loom) == (DPIDR, ACK_OK)
    assert read_dpidr(loom) == (DPIDR, ACK_OK)
    assert port.switches == 1 and len(port.packets) == 2
    assert bench.contention_free()


@pytest.mark.parametrize("ack", [ACK_WAIT, ACK_FAULT])
def test_swd_a_refused_packet_has_no_data_phase(backend, ack):
    """WAIT and FAULT: the status carries the ACK and the data words are zero."""
    bench, port, loom = setup(backend, ack=ack)
    loom.push(0, [READ])
    assert read_dpidr(loom) == (0, ack)
    assert bench.contention_free()
    assert loom.badop() == 0


def test_swd_a_wrong_parity_bit_is_reported(backend):
    """The data still arrives; bit 3 of the status says not to trust it."""
    bench, port, loom = setup(backend, parity_error=True)
    loom.push(0, [READ])
    assert read_dpidr(loom) == (DPIDR, ACK_OK | PARITY_ERR)


@pytest.mark.parametrize("dpidr", [0x00000000, 0xFFFFFFFF, 0x12345678])
def test_swd_reads_any_dpidr_and_gets_its_parity_right(backend, dpidr):
    """Even parity over 32 bits: 0xFFFFFFFF is even, 0x12345678 has 13 ones."""
    bench, port, loom = setup(backend, dpidr=dpidr)
    loom.push(0, [READ])
    assert read_dpidr(loom) == (dpidr, ACK_OK)


def test_swd_master_assembles_strict_with_no_diagnostic():
    """L2-DEADLINE for swd_master.loom at its declared tick."""
    program = assemble_file(FIRMWARE / "swd_master.loom", isa=ISA, strict=True)
    assert program.diagnostics == []
    report = program.deadlines[0]
    assert report.period == 32                  # the .tick it is proved for
    assert report.pairs and not report.infeasible and not report.unbounded
    assert report.worst_slack >= 0
    assert list(program.threads) == [0]
    assert max(program.words) < 512 // 4


def test_swd_switch_sequence_is_the_one_in_the_specification():
    """0xE79E least significant bit first is 0111 1001 1110 0111 on the wire."""
    wire = "".join(str((SWITCH_SEQUENCE >> i) & 1) for i in range(16))
    assert wire == "0111100111100111"
