"""L3 for the IEC 61158-2 MBP programs, on the model and the RTL.

``firmware/mbp_tx.loom`` (thread 0; TX = OUT0, TXE = OUT1) sends a PhPDU
from host words; ``firmware/mbp_rx.loom`` (thread 1; RX = IN0) receives
PhPDUs and reports their octets on OUTQ[1]. The reference is
``tools.protomodels.mbp``: the symbol and half-bit encoder, the FCS, a pin
source with rate offset and jitter, the MAU echo, an edge recorder, an
independent decoder and the timing checker.

Expected values come from the sources in the model's docstring (preamble,
SD and ED symbols, 32 us bits, the FCS, +-0.2 % and the jitter figures)
and from the program headers' host-word formats, never from what the model
or the RTL did.

Boards. A bit is four ticks. At 50 MHz the tick is 400 clocks and a bit
1600, so the RTL runs on a 12.5 MHz board (tick 100, the fastest the
programs are proved for): the same waveform in microseconds, 400 clocks a
bit. Every body is named ``test_mbp_*``: test/test_fw.py collects all
modules into one namespace.

The PHY-GL-SDF candidate is ``test_mbp_tx_and_rx_together`` on the
12.5 MHz board: one three-octet frame through the MAU echo, about 30,000
clocks after the load.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import mbp as M
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
TX = assemble_file(FIRMWARE / "mbp_tx.loom", isa=ISA, strict=True)
RX = assemble_file(FIRMWARE / "mbp_rx.loom", isa=ISA, strict=True)
GOOD, BAD_FCS, CODING, OVERRUN = 0x8000, 0x4000, 0x2000, 0x1000  # RX header
SENT, UNDERRUN = 0, 1                                             # TX header


class Board:
    """A clock and the tick (a quarter bit) the program headers give for it."""

    def __init__(self, name, clock_hz, tick):
        self.name, self.clock_hz, self.tick = name, float(clock_hz), tick

    def cycles(self, bits):
        return int(round(bits * M.BIT_S * self.clock_hz))

    def __repr__(self):
        return self.name


B50 = Board("50MHz", 50_000_000, 400)
BRTL = Board("12_5MHz", 12_500_000, 100)


def mbp_bench(backend):
    """FIFOs and the bit engine with its encoders; the RTL has them all."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC"), imem_words=512)
    return backend.bench()


def start(backend, bench, board, tx=False, rx=False):
    """Load the chosen programs, give them the board's tick, run them."""
    loom = backend.loom(bench, isa=ISA)
    words, threads = {}, []
    if tx:
        words.update(TX.words)
        threads.append(0)
    if rx:
        words.update(RX.words)
        threads.append(1)
    loom.load(words)
    for t in threads:
        loom.write_csr(t, "TICK_INT", board.tick)
    loom.run(threads)
    bench.step(200)
    return loom


def exchange(bench, loom, board, bits, tx_words=(), drain=(0, 1)):
    """Run ``bits`` bit times as the host: keep INQ[0] fed with
    ``tx_words`` and drain the OUTQs of ``drain`` every two bits (the
    headers: an octet is due or arrives every eight). Returns ``{thread:
    words}``."""
    pending = list(tx_words)
    out = {t: [] for t in drain}
    left = board.cycles(bits)
    chunk = board.cycles(2)
    while left > 0:
        if pending:
            status = loom.fifo_status(0)
            free = status["depth"] - status["inq"]
            if free > 0:
                loom.push(0, pending[:free])
                pending = pending[free:]
        for t in drain:
            out[t] += loom.pop_available(t)
        n = min(chunk, left)
        bench.step(n)
        left -= n
    for t in drain:
        out[t] += loom.pop_available(t)
    assert not pending, "the transmitter did not take every host word"
    return out


def report(n, flags=0):
    """The RX report word (its header)."""
    return GOOD | flags | n


def frame_bits(n, preamble=1):
    """Bits of a PhPDU with ``n`` DLPDU octets before the FCS."""
    return 8 * preamble + 8 + 8 * (n + 2) + 8


def tx_header(n, preamble=1):
    return ((preamble & 15) << 8) | n


def check_tx(rec, board, data, preamble):
    """The chip's PhPDU on TX against the sources (module docstring)."""
    txe = rec.of("OUT1")
    assert [v for _, v in txe] == [1, 0], txe
    rise, fall = txe[0][0], txe[1][0]
    tx = rec.of("OUT0")
    assert tx and rise < tx[0][0] and tx[-1] == (fall, 0), (rise, fall, tx[-3:])
    frame = M.decode(tx, board.clock_hz, end=fall)
    assert frame.ended and not frame.errors, frame.errors
    assert frame.good, [hex(o) for o in frame.octets]
    assert frame.data == list(data)
    assert frame.preamble_bits == 8 * preamble
    timing = M.check_timing(tx, board.clock_hz)
    # every edge on its half-bit grid (TICK_INT a multiple of the slot,
    # SEMANTICS 2), so far inside +-1.0 us and +-0.2 %
    assert timing["max_dev_s"] * board.clock_hz < 0.5
    assert timing["max_dev_s"] <= M.TX_JITTER_S
    assert abs(timing["rate_err"]) <= M.RATE_TOL
    assert timing["runs"] <= {1, 2, 3}
    return frame


def tx_run(backend, board, data, preamble=1, rx=False):
    bench = mbp_bench(backend)
    rec = bench.add(M.Recorder())
    if rx:
        bench.add(M.Mau())
    loom = start(backend, bench, board, tx=True, rx=rx)
    words = [tx_header(len(data), preamble)] + list(data)
    out = exchange(bench, loom, board, frame_bits(len(data), preamble) + 8,
                   words, drain=(0, 1) if rx else (0,))
    return rec, out


# ------------------------------------------------------------- assembly
def test_mbp_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: every pair proved at 100 clocks a tick, nothing
    unbounded, one ``.bounded`` each (the guarded POP and PUSH). The
    transmitter fills thread 0's quarter, the receiver thread 1's, so the
    two load together."""
    for prog, thread in ((TX, 0), (RX, 1)):
        assert prog.diagnostics == [] and list(prog.threads) == [thread]
        report_ = prog.deadlines[thread]
        assert report_.period == 100
        assert report_.pairs and not report_.infeasible and not report_.unbounded
        assert report_.worst_slack >= 0
        addrs = sorted(prog.words)
        assert addrs[0] == 128 * thread and addrs[-1] < 128 * (thread + 1)
    assert not set(TX.words) & set(RX.words)


# ---------------------------------------------------------- transmitter
def test_mbp_tx_to_model(backend):
    """Three octets, one preamble octet, 12.5 MHz board: the reference
    decoder finds the PhPDU, the FCS checks, every edge is on the grid."""
    rec, out = tx_run(backend, BRTL, [0x01, 0x7E, 0xC3])
    check_tx(rec, BRTL, [0x01, 0x7E, 0xC3], 1)
    assert out[0] == [SENT]


@model_only("31.25 kbit/s at 50 MHz is 1600 clocks a bit: about 0.2 M clocks a frame")
@pytest.mark.parametrize("preamble, data", [(1, [0x00]), (2, [0xFF, 0x00, 0x55, 0xAA]),
                                            (8, list(range(0x30, 0x40)))])
def test_mbp_tx_to_model_50mhz(backend, preamble, data):
    """The real rate, preamble lengths 1, 2 and 8 octets [P], runs of 0s,
    1s and alternating bits."""
    rec, out = tx_run(backend, B50, data, preamble)
    check_tx(rec, B50, data, preamble)
    assert out[0] == [SENT]


@model_only("an error path; the RTL runs the good frames")
def test_mbp_tx_underrun(backend):
    """The header announces six octets and the host sends two: the frame is
    cut off (TXE falls at once) and OUTQ[0] says underrun."""
    bench = mbp_bench(backend)
    rec = bench.add(M.Recorder())
    loom = start(backend, bench, BRTL, tx=True)
    out = exchange(bench, loom, BRTL, frame_bits(6) + 8,
                   [tx_header(6), 0x11, 0x22], drain=(0,))
    assert out[0] == [UNDERRUN]
    txe = rec.of("OUT1")
    assert [v for _, v in txe] == [1, 0]
    # cut off in the third octet: before the FCS would have started
    length = (txe[1][0] - txe[0][0]) / BRTL.cycles(1)
    assert 8 + 8 + 16 < length < 8 + 8 + 24 + 2, length
    frame = M.decode(rec.of("OUT0"), BRTL.clock_hz, end=txe[1][0])
    assert not frame.good


# -------------------------------------------------------------- receiver
def rx_run(backend, board, frames, rate=M.BIT_RATE, jitter_s=0.0, gap_bits=4):
    """The receiver alone; ``frames`` are half-bit level lists sent one
    after the other with ``gap_bits`` of idle line between, at ``rate``,
    each level change moved by up to ``jitter_s``. Returns OUTQ[1]."""
    bench = mbp_bench(backend)
    loom = start(backend, bench, board, rx=True)
    at = bench.cycle + board.cycles(gap_bits)
    total = gap_bits
    for levels in frames:
        jit = M.jitter_pattern(len(levels) + 1, jitter_s, seed=len(levels)) \
            if jitter_s else None
        src = bench.add(M.Source("IN0", levels, at, board.clock_hz, rate,
                                 jitter=jit))
        bits = len(levels) / 2.0
        at = src.done_cycle + board.cycles(gap_bits)
        total += bits * M.BIT_RATE / rate + gap_bits
    return exchange(bench, loom, board, total + 4, drain=(1,))[1]


def levels_of(data, preamble=1, **kw):
    return M.halfbits(M.frame_symbols(data, preamble, **kw))


def test_mbp_model_to_rx(backend):
    """A good frame from the reference source: the octets, the FCS octets,
    then the clean report 0x8000 | n."""
    data = [0x10, 0x5A, 0xE7]
    got = rx_run(backend, BRTL, [levels_of(data)])
    assert got == M.with_fcs(data) + [report(len(data) + 2)]


def test_mbp_rx_bad_fcs(backend):
    """One FCS bit flipped: the octets as received, then bit 14."""
    data = [0x42, 0x24]
    bad = [M.fcs(data)[0], M.fcs(data)[1] ^ 0x01]
    got = rx_run(backend, BRTL, [levels_of(data, fcs_octets=bad)])
    assert got == data + bad + [report(4, BAD_FCS)]


def test_mbp_rx_corrupted_end_delimiter(backend):
    """The ED's second N- sent as a 1: an octet with violations that is not
    the ED is a coding error, and the frame ends there."""
    data = [0x99]
    ed = ("1", "N+", "N-", "N+", "1", "1", "0", "1")
    got = rx_run(backend, BRTL, [levels_of(data, ed=ed)])
    assert got == M.with_fcs(data) + [report(3, CODING)]


def test_mbp_rx_corrupted_start_delimiter_then_good_frame(backend):
    """The SD's first N+ sent as a 1: no frame is reported (no SD is found,
    the receiver loses lock at the idle line and hunts again), and the good
    frame that follows is received whole."""
    sd = ("1", "1", "N-", "1", "0", "N-", "N+", "0")
    first, second = [0xAB, 0xCD], [0x3C]
    got = rx_run(backend, BRTL, [levels_of(first, sd=sd), levels_of(second)],
                 gap_bits=6)
    assert got == M.with_fcs(second) + [report(3)]


@pytest.mark.parametrize("offset", [+0.002, -0.002])
def test_mbp_rx_sender_off_rate(backend, offset):
    """A sender 0.2 % fast or slow (the tolerance, unchecked) with +-1.0 us
    of jitter on every level change (the transmit jitter, unchecked)."""
    data = [0xF0, 0x0F, 0x81, 0x7E]
    got = rx_run(backend, BRTL, [levels_of(data)], rate=M.BIT_RATE * (1 + offset),
                 jitter_s=M.TX_JITTER_S)
    assert got == M.with_fcs(data) + [report(6)]


@model_only("31.25 kbit/s at 50 MHz: about 0.9 M clocks a frame")
@pytest.mark.parametrize("offset", [+0.002, -0.002])
def test_mbp_rx_long_frame_off_rate_50mhz(backend, offset):
    """64 octets at the real rate, 0.2 % off with +-1.0 us jitter: 0.2 % of
    560 bits is more than a bit, so this holds only because the grid
    restarts on every mid-bit edge."""
    data = [(37 * i + 11) & 0xFF for i in range(64)]
    got = rx_run(backend, B50, [levels_of(data, 2)], rate=M.BIT_RATE * (1 + offset),
                 jitter_s=M.TX_JITTER_S)
    assert got == M.with_fcs(data) + [report(66)]


@model_only("the receiver's jitter margin; the RTL runs the transmit jitter")
def test_mbp_rx_tolerates_receive_jitter(backend):
    """+-3.2 us on every level change (10 % of a bit, the receive figure,
    unchecked), a preamble of eight octets [P]."""
    data = [0x00, 0xFF, 0xA5]
    got = rx_run(backend, BRTL, [levels_of(data, 8)], jitter_s=M.RX_JITTER_S)
    assert got == M.with_fcs(data) + [report(5)]


@model_only("an error path of the host interface")
def test_mbp_rx_overrun(backend):
    """The host does not drain OUTQ[1]: four octets fit, the fifth finds it
    full, and the report (bit 12, four octets) follows them."""
    data = list(range(1, 9))
    bench = mbp_bench(backend)
    loom = start(backend, bench, BRTL, rx=True)
    bench.add(M.Source("IN0", levels_of(data), bench.cycle + 1000, BRTL.clock_hz))
    bench.step(BRTL.cycles(frame_bits(len(data)) + 4))
    assert loom.pop(1, 5) == data[:4] + [report(4, OVERRUN)]


# ------------------------------------------------------------- together
def test_mbp_tx_and_rx_together(backend):
    """Thread 0 sends, the MAU echoes TX onto IN0 while TXE is high, thread
    1 receives: the frame on the pins checks against the reference and
    the receiver reports it whole. The PHY-GL-SDF candidate."""
    data = [0x5D, 0x00, 0xFE]
    rec, out = tx_run(backend, BRTL, data, rx=True)
    check_tx(rec, BRTL, data, 1)
    assert out[0] == [SENT]
    assert out[1] == M.with_fcs(data) + [report(5)]
