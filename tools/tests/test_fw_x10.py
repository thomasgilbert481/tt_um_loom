"""L3 for the X10 programs, on the model and the RTL.

``firmware/x10_tx.loom`` (thread 0; ZC = IN0, TX = OUT0) sends X10 frames
at the zero crossings; ``firmware/x10_rx.loom`` (thread 1; ZC = IN1,
RX = IN2) receives them from a TW523-style envelope. The references are
``tools.protomodels.x10``: the code tables, the frame layout, a decoder,
the mains, a line sender and an envelope detector, written from the X10
technical note ([PLC] in that module).

Time scale. A real 60 Hz half-cycle is 416,667 clocks at 50 MHz and one
command (two frames and the silence) is 50 half-cycles, 21 M clocks: far
past the RTL budget. The RTL scenarios therefore run a scaled line: a
half-cycle of 1000 clocks (60 Hz) or 1200 (50 Hz, model only), a carrier
half period (the TX tick) of 20 clocks and 4 carrier cycles a burst. What
is checked there comes from the frame rules of [PLC] and from SEMANTICS
(the staged edges land on ticks, so every carrier half period is exactly
one tick, and a burst's first rise lands 3 to 4 ticks plus the
synchroniser and slot grid after the ZC edge). The [PLC] windows in time
(200 us, 1 ms -50/+100 us, 120 kHz, three phases) are checked at the real
scale, on the model only, on the start code of one command.

The PHY-GL-SDF candidate is ``test_x10_tx_sends_each_command_as_a_pair``.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import x10 as X
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
TX_SRC = FIRMWARE / "x10_tx.loom"
RX_SRC = FIRMWARE / "x10_rx.loom"
CLOCK = 50e6

HALF_60, HALF_50 = 1000, 1200       # scaled half-cycles, clocks
P = 20                              # scaled carrier half period, TX tick
CYCLES = 4                          # scaled carrier cycles a burst
RX_TICK, RX_S = 20, 1               # RX tick, offset: sample 40 to 68 clocks in
NO_GAP = 0x8000
A1 = X.command("A", X.unit(1))
A_ON = X.command("A", X.function("ON"))
M_DIM = X.command("M", X.function("DIM"))
OK, BAD, DIFFER, LONE = 0, 1, 2, 3


def bench_for(backend):
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD"), imem_words=512)
    return backend.bench()


def report(status, value):
    return status << 12 | value


def start_tx(backend, half=HALF_60, tick=P, frac=0, cycles=CYCLES,
             bursts=1, gap=1, extra=(), rx_s=None):
    bench = bench_for(backend)
    mains = bench.add(X.Mains(["IN0", "IN1"], half))
    log = bench.add(X.EdgeLog("OUT0"))
    for m in extra:
        bench.add(m)
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(TX_SRC, isa=ISA, strict=True))
    loom.write_csr(0, "TICK_INT", tick)
    if frac:
        loom.write_csr(0, "TICK_FRAC", frac)
    if rx_s is None:
        loom.run(0)
    else:                               # x10_rx on thread 1 as well
        loom.load(assemble_file(RX_SRC, isa=ISA, strict=True))
        loom.write_csr(1, "TICK_INT", RX_TICK)
        loom.run([0, 1])
        loom.push(1, [rx_s])
    loom.push(0, [cycles, bursts, gap])
    return bench, mains, log, loom


def wait_halves(bench, mains, n, half):
    """Run until ``n`` more ZC edges have passed."""
    target = len(mains.edges) + n
    assert bench.run_until(lambda: len(mains.edges) >= target,
                           int((n + 2) * half), every=200)


def line_halves(mains, log, half, tick=P):
    """The half-cycles as the line carries them, leading silence cut."""
    bursts = X.bursts_of(log.edges, 2 * tick)
    halves = X.halves_of(mains.edges, bursts, half // 2)
    while halves and not halves[0]:
        halves.pop(0)
    return halves, bursts


def test_x10_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: both programs clean under --strict, every pair proved
    (TX at its .tick of 20 clocks, RX at 8), nothing unbounded, no
    ``.bounded``; 54 and 65 words, in threads 0 and 1, so they fit
    together."""
    tx = assemble_file(TX_SRC, isa=ISA, strict=True)
    rx = assemble_file(RX_SRC, isa=ISA, strict=True)
    for prog, thread, period, words in ((tx, 0, 20, 54), (rx, 1, 8, 65)):
        assert prog.diagnostics == [] and list(prog.threads) == [thread]
        rep = prog.deadlines[thread]
        assert rep.period == period and rep.pairs and not rep.infeasible
        assert not rep.unbounded and rep.worst_slack >= 0
        assert len(prog.words) == words
        assert all(thread * 128 <= a < thread * 128 + 128 for a in prog.words)


def test_x10_tx_sends_each_command_as_a_pair(backend):
    """Address A1 then A ON: on the line, each frame twice back to back
    (start code 1110, nine bits in true/complement form) and at least six
    clear half-cycles after each pair; each burst exactly CYCLES carrier
    cycles of one tick high, one tick low; each first rise 3 to 4 ticks
    plus at most 8 clocks after its ZC edge; the commands acknowledged."""
    bench, mains, log, loom = start_tx(backend)
    loom.push(0, [A1, A_ON])
    wait_halves(bench, mains, 104, HALF_60)
    assert loom.pop(0, 2) == [A1, A_ON]
    halves, bursts = line_halves(mains, log, HALF_60)
    assert halves[:100] == X.pair_halves(A1) + X.pair_halves(A_ON)
    assert not any(halves[100:])
    assert X.decode_halves(halves) == [("ok", A1)] * 2 + [("ok", A_ON)] * 2
    for first, last, rises in bursts:
        assert rises == CYCLES and last - first == (2 * CYCLES - 1) * P
        edge = max(e for e in mains.edges if e <= first)
        assert 3 * P <= first - edge <= 4 * P + 8
    pulses = [c for c, _ in log.edges]
    for b_first, b_last, _ in bursts:
        inside = [c for c in pulses if b_first <= c <= b_last]
        assert all(y - x == P for x, y in zip(inside, inside[1:]))


# --------------------------------------------------------------- receiver
def start_rx(backend, halves, half=HALF_60, tick=RX_TICK, s=RX_S,
             delay=10, length=None):
    """x10_rx on thread 1 behind a line sender on IN1/IN2 whose envelope
    is 1 ms of the scaled half-cycle (``half`` is 8.33 ms at 60 Hz),
    starting ``delay`` clocks after each ZC edge."""
    bench = bench_for(backend)
    mains = bench.add(X.Mains(["IN0", "IN1"], half))
    if length is None:
        length = int(half * 1e-3 / (1 / 120.0))
    sender = bench.add(X.Sender("IN1", "IN2", delay, length))
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(RX_SRC, isa=ISA, strict=True))
    loom.write_csr(1, "TICK_INT", tick)
    loom.run(1)
    loom.push(1, [s])
    wait_halves(bench, mains, 2, half)
    sender.halves += [0, 0] + list(halves)
    return bench, mains, sender, loom


def rx_reports(backend, halves, n, **kw):
    bench, mains, sender, loom = start_rx(backend, halves, **kw)
    half = kw.get("half", HALF_60)
    wait_halves(bench, mains, len(sender.halves) + 4, half)
    return loom.pop(1, n), loom.pop_available(1)


def test_x10_rx_receives_pairs_from_the_line(backend):
    """Two pairs from a reference sender (A1, then A ON, the six-half-cycle
    silence after each): one status-0 report per pair, the value as sent,
    and nothing else."""
    first, rest = rx_reports(backend, X.pair_halves(A1) + X.pair_halves(A_ON), 2)
    assert first == [report(OK, A1), report(OK, A_ON)] and rest == []


def test_x10_rx_rejects_a_corrupted_complement(backend):
    """The 4th bit of the first copy followed by itself, not its
    complement: that copy is reported bad (status 1, the three bits before
    it), the repeat alone as a lone frame (status 3): no status 0."""
    halves = X.pair_halves(A_ON)
    k = 3
    halves[4 + 2 * k + 1] = halves[4 + 2 * k]
    assert X.decode_halves(halves)[0] == ("bad", k)
    first, rest = rx_reports(backend, halves, 2)
    assert first == [report(BAD, A_ON >> (9 - k)), report(LONE, A_ON)]
    assert rest == []


@model_only("error paths beyond the RTL budget; the complement case runs there")
def test_x10_rx_flags_a_repeat_that_differs_and_a_lone_frame(backend):
    """A pair whose copies differ (A1 then A2): status 2 with the repeat's
    value; then one frame with silence after it: status 3."""
    a2 = X.command("A", X.unit(2))
    halves = X.frame_halves(A1) + X.frame_halves(a2) + [0] * 6
    halves += X.frame_halves(M_DIM) + [0] * 6
    first, rest = rx_reports(backend, halves, 2)
    assert first == [report(DIFFER, a2), report(LONE, M_DIM)] and rest == []


@model_only("50 Hz: the 60 Hz cases run on the RTL")
def test_x10_rx_at_50_hz(backend):
    """The same pairs on a 50 Hz line (a half-cycle 1.2 times as long,
    and not a multiple of the tick, so the sample lands at another phase);
    the 1 ms envelope is shorter relative to the half-cycle."""
    first, rest = rx_reports(backend, X.pair_halves(M_DIM) + X.pair_halves(A1), 2,
                             half=HALF_50 + 10)
    assert first == [report(OK, M_DIM), report(OK, A1)] and rest == []


# -------------------------------------------------------------- together
def together(backend, half, cmds, rx_s=3):
    """x10_tx on thread 0 and x10_rx on thread 1, one line: the mains on
    IN0 and IN1, TX (OUT0) through a TW523-style envelope detector to RX
    (IN2). The envelope holds 2 ticks past the last carrier edge; RX
    samples 80 to 108 clocks after the edge, inside the burst (60 to 248)."""
    env = X.Envelope("OUT0", "IN2", hold=2 * P)
    bench, mains, log, loom = start_tx(backend, half=half, extra=[env],
                                       rx_s=rx_s)
    loom.push(0, list(cmds))
    wait_halves(bench, mains, 50 * len(cmds) + 4, half)
    assert loom.pop(0, len(cmds)) == list(cmds)
    return loom.pop(1, len(cmds)), loom.pop_available(1)


def test_x10_tx_and_rx_together(backend):
    """A1 then A ON from thread 0 to thread 1: each pair reported once,
    status 0, the value sent."""
    got, rest = together(backend, HALF_60, [A1, A_ON])
    assert got == [report(OK, A1), report(OK, A_ON)] and rest == []


@model_only("50 Hz: the 60 Hz case runs on the RTL")
def test_x10_tx_and_rx_together_at_50_hz(backend):
    """The same on a 50 Hz line whose half-cycle (1210 clocks) is not a
    multiple of either tick, so each burst starts at another tick phase."""
    got, rest = together(backend, HALF_50 + 10, [M_DIM, A1])
    assert got == [report(OK, M_DIM), report(OK, A1)] and rest == []


@model_only("a run of dims: three pairs, 66 half-cycles")
def test_x10_tx_dim_run_has_no_gaps(backend):
    """Bright and dim go out "continuously ... with no gaps" ([PLC]): with
    bit 15 set the pairs follow each other with no clear half-cycle, and
    the last one (bit 15 clear) is followed by the six-half-cycle gap."""
    bench, mains, log, loom = start_tx(backend)
    loom.push(0, [M_DIM | NO_GAP, M_DIM | NO_GAP, M_DIM])
    wait_halves(bench, mains, 3 * 44 + 10, HALF_60)
    assert loom.pop(0, 3) == [M_DIM | NO_GAP, M_DIM | NO_GAP, M_DIM]
    halves, _ = line_halves(mains, log, HALF_60)
    assert halves[:138] == X.frame_halves(M_DIM) * 6 + [0] * 6
    assert not any(halves[138:])


@model_only("real time: 1.7 M clocks at 50 MHz on a 60 or 50 Hz line")
@pytest.mark.parametrize("mains_hz, gap", [(60, 427), (50, 560)])
def test_x10_tx_real_scale_three_phase(backend, mains_hz, gap):
    """At 50 MHz with the carrier tick (208 + 85/256 clocks, 120 kHz) and
    three bursts a half-cycle, on a real 60 or 50 Hz line: the start code
    of one command, every burst checked against [PLC]'s windows (within
    200 us of its phase's zero crossing, 1 ms -50/+100 us of 120 kHz) by
    ``x10.check_bursts``, three bursts in each of the first three
    half-cycles and none in the fourth."""
    half = CLOCK / mains_hz / 2
    bench, mains, log, loom = start_tx(backend, half=half, tick=208, frac=85,
                                       cycles=120, bursts=3, gap=gap)
    loom.push(0, [A1])
    edges0 = len(mains.edges)
    assert bench.run_until(lambda: len(X.bursts_of(log.edges, 500)) >= 9,
                           int(6 * half), every=2000)
    wait_halves(bench, mains, 2, half)  # the fourth half-cycle, complete
    bursts = X.bursts_of(log.edges, 500)
    assert len(bursts) == 9
    assert X.check_bursts(mains.edges, bursts, CLOCK, mains_hz, phases=3) == []
    first_edge = max(e for e in mains.edges if e <= bursts[0][0])
    i = mains.edges.index(first_edge)
    for k in range(4):
        lo, hi = mains.edges[i + k], mains.edges[i + k + 1]
        assert sum(lo <= b[0] < hi for b in bursts) == (3 if k < 3 else 0)
    assert i >= edges0
