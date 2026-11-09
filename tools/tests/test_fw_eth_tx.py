"""L3-ETH: 10BASE-T frames and link pulses from ``firmware/eth_tx.loom``,
on the model and the RTL.

The program (thread 0) sends frames from its buffer through the Ethernet
serializer (``docs/SEMANTICS.md`` 6.12, D-036) on TD+/TD- (OUT0/OUT1) and
normal link pulses between them. The pads are sampled once a clock and
decoded by the reference receiver ``tools/protomodels/ethernet.py``, which
knows nothing of Loom; the frames' bytes and FCS are checked against
``zlib.crc32`` and the clause 3 layout, never against the model.

Expected numbers: a half bit is N = 2 clocks (10 Mbit/s at the 40 MHz board
clock the program is written for); the interframe gap is at least 96 bit
times (384 clocks, clause 4.4.2 [S]); TP_IDL is 250 to 450 ns (the receiver's
window, [S, unchecked]); an NLP is TD+ high with TD- low, 100 ns nominal and
at most 200 ns, 16 ms +- 8 ms after the last frame's end or the last pulse
(``tools/protomodels/tenbaset_link.py``, [S]). The program's own contract,
from its header: pulses exactly 4 clocks, P * 160 clocks rise to rise, the
first P * 160 clocks after a frame's release.

Scaling on the RTL (Icarus, about 6,400 clocks a second): the 16 ms period
is the host's P in 4 us units, so the RTL runs P = 10 (40 us, 1,600 clocks)
and the window scales with it (P * 4 us +- 50 %). The real P = 4000
(640,000 clocks) runs on the model only. Frames are not scaled.
"""

import zlib

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import ethernet as rx
from tools.protomodels import tenbaset_link as link
from tools.protomodels.bench import Model
from tools.tests.fw_backend import model_only

ISA = load()
IMEM = 512
CLOCK_HZ = 40e6
HALF = 2                          # N: clocks a half bit at 40 MHz
BIT = 2 * HALF
UNIT = 160                        # the program's period unit, 4 us
IFG = 96 * BIT                    # 9.6 us
NLP_WIDTH = 4                     # 100 ns, the program's contract
GOOD, UNDERRUN, REFUSED = 0x8004, 0x800C, 0x8020


def program():
    prog = assemble_file(REPO / "firmware" / "eth_tx.loom", isa=ISA, strict=True)
    assert prog.diagnostics == [], prog.diagnostics
    return prog


TX = program()
BUF = TX.symbols["BUF"]
MAXB = TX.symbols["MAXB"]


def words_of(data):
    """Two bytes a word, the first byte low (SEMANTICS 6.12.2)."""
    data = bytes(data) + (b"\0" if len(data) % 2 else b"")
    return [data[i] | data[i + 1] << 8 for i in range(0, len(data), 2)]


def arp_request():
    """A broadcast ARP request (RFC 826): 42 bytes before the pad."""
    sha, spa, tpa = bytes.fromhex("020000000001"), bytes([192, 168, 1, 2]), bytes([192, 168, 1, 1])
    return (b"\xff" * 6 + sha + b"\x08\x06"
            + bytes.fromhex("0001080006040001") + sha + spa + b"\0" * 6 + tpa)


def sample_frame(length, seed):
    return bytes(bytearray.fromhex("021122334455") + bytes.fromhex("020000000001")
                 + (length - 14).to_bytes(2, "big")
                 + bytes((seed + 7 * i) & 0xFF for i in range(length - 14)))


class Trace(Model):
    """TD+ and TD- (``uo_out[0]``, ``uo_out[1]``) once a clock."""

    def __init__(self):
        self.first = None
        self.tdp = bytearray()
        self.tdn = bytearray()
        self.run = 0                  # length of the driven run in progress
        self.done = {"nlp": 0, "frame": 0}

    def observe(self, lines):
        if self.first is None:
            self.first = lines.cycle
        p, n = lines.uo & 1, (lines.uo >> 1) & 1
        self.tdp.append(p)
        self.tdn.append(n)
        if p != n:
            self.run += 1
        elif self.run:
            self.done["nlp" if self.run <= 16 else "frame"] += 1
            self.run = 0

    def segments(self):
        """``(start, end, kind)`` of every driven run: an NLP or a frame."""
        out, lv, i, n = [], rx.line_levels(self.tdp, self.tdn), 0, len(self.tdp)
        while i < n:
            if lv[i] == 0:
                i += 1
                continue
            j = i
            while j < n and lv[j] != 0:
                j += 1
            kind = "nlp" if j - i <= 16 and all(v > 0 for v in lv[i:j]) else "frame"
            out.append((i, j, kind))
            i = j
        return out

    def frames(self):
        return [rx.receive_frame(self.tdp, self.tdn, HALF, CLOCK_HZ, start=a)
                for a, b, k in self.segments() if k == "frame" and b < len(self.tdp)]

    def nlps(self):
        return [(a, b) for a, b, k in self.segments() if k == "nlp"]


# ------------------------------------------------------------------ setup
def boot(backend, frame=None, period=0):
    """Load the program (and ``frame`` into the buffer), run thread 0 and set
    the NLP period (0: none)."""
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "SETPD", "DMEM", "ETX"), imem_words=IMEM)
    else:
        bench = backend.bench()
    trace = bench.add(Trace())
    loom = backend.loom(bench, isa=ISA)
    image = dict(TX.words)
    if frame is not None:
        image.update({BUF + i: w for i, w in enumerate(words_of(frame))})
    loom.load(image)
    loom.run([0])
    loom.push(0, [0x4000 | period])
    return bench, trace, loom


def wait_for(bench, trace, kind, n, budget):
    ok = bench.run_until(lambda: trace.done[kind] >= n, budget, every=64)
    assert ok, "%d of %d %s after %d clocks" % (trace.done[kind], n, kind, budget)


def frame_clocks(length):
    """Preamble and SFD, the padded frame, the FCS, TP_IDL (6.12.6)."""
    return (64 + 8 * (max(length, 60) + 4)) * BIT + 6 * HALF


def check_frame(f, data):
    """A good frame carrying ``data`` padded to 60 bytes, with its FCS."""
    padded = bytes(data) + b"\0" * max(0, 60 - len(data))
    assert f.ok, f.errors
    assert f.preamble_bits == 64
    assert f.payload == padded
    assert f.fcs == zlib.crc32(padded)
    assert rx.TP_IDL_MIN_NS <= f.tp_idl_ns <= rx.TP_IDL_MAX_NS
    d = f.decoded
    assert {b - a for a, b in zip(d.middles, d.middles[1:])} == {BIT}
    assert d.middles[-1] - d.start == (len(d.bits) - 1) * BIT + HALF


def last_bit_end(f):
    return f.decoded.middles[-1] + HALF


def check_nlp(trace, a, b):
    assert b - a == NLP_WIDTH and b - a <= link.PULSE_MAX_NS * CLOCK_HZ / 1e9
    assert not any(trace.tdn[a:b]) and all(trace.tdp[a:b])


# --------------------------------------------------------------- frames
def test_eth_tx_arp_request_minimum_frame(backend):
    """An ARP request (42 bytes) padded to 60: the receiver finds 64 bits of
    preamble and SFD, the 60 bytes, a good FCS (``zlib.crc32``), bit cells of
    exactly 4 clocks and TP_IDL inside 250..450 ns; the result is 0x8004."""
    arp = arp_request()
    bench, trace, loom = boot(backend, arp)
    loom.push(0, [0x8000 | len(arp)])
    wait_for(bench, trace, "frame", 1, frame_clocks(60) + 3000)
    frames = trace.frames()
    assert len(frames) == 1
    check_frame(frames[0], arp)
    assert loom.pop(0) == [GOOD]
    assert loom.badop() == 0


@pytest.mark.parametrize("length", [301, 128])
def test_eth_tx_larger_frame_from_host_words(backend, length):
    """A frame loaded through INQ (the load command) and sent: an odd length
    ends with ETX_LAST1, an even one with ETX_LAST."""
    data = sample_frame(length, length)
    bench, trace, loom = boot(backend)
    words = words_of(data)
    loom.push(0, [len(words)] + words + [0x8000 | length])
    wait_for(bench, trace, "frame", 1, frame_clocks(length) + 40 * len(words) + 20000)
    frames = trace.frames()
    assert len(frames) == 1
    check_frame(frames[0], data)
    assert loom.pop(0) == [GOOD]


def test_eth_tx_two_frames_and_the_gap(backend):
    """Two sends back to back: both good, the second starting at least 96 bit
    times (384 clocks) after the first's last FCS bit and after its TP_IDL."""
    arp = arp_request()
    bench, trace, loom = boot(backend, arp + bytes(22))   # words 30, 31 loaded too
    loom.push(0, [0x8000 | len(arp), 0x8000 | 64])
    wait_for(bench, trace, "frame", 2, 2 * frame_clocks(64) + 6000)
    f1, f2 = trace.frames()
    check_frame(f1, arp)
    check_frame(f2, arp + b"\0" * 22)
    assert f2.decoded.start - last_bit_end(f1) >= IFG
    assert f2.decoded.start - f1.decoded.idle_at >= IFG
    assert f2.decoded.start - f1.decoded.idle_at <= 3 * UNIT + 200   # at most a unit late
    assert loom.pop(0, 2) == [GOOD, GOOD]


# ------------------------------------------------------------ link pulses
def check_period(starts, period):
    for a, b in zip(starts, starts[1:]):
        assert b - a == period * UNIT                       # the program's contract
        assert 0.5 * period * UNIT <= b - a <= 1.5 * period * UNIT   # 16 +- 8 ms, scaled


def test_eth_tx_nlps_between_frames(backend):
    """P = 10 (40 us, the 16 ms scaled): NLPs exactly 1,600 clocks apart,
    each 4 clocks of TD+ high and TD- low; a frame restarts the timer, so the
    first pulse after it rises 1,600 clocks after its TP_IDL ends."""
    p = 10
    arp = arp_request()
    bench, trace, loom = boot(backend, arp, period=p)
    wait_for(bench, trace, "nlp", 3, 4 * p * UNIT + 4000)
    loom.push(0, [0x8000 | len(arp)])
    wait_for(bench, trace, "frame", 1, frame_clocks(60) + p * UNIT + 4000)
    n0 = trace.done["nlp"]
    wait_for(bench, trace, "nlp", n0 + 3, 4 * p * UNIT)
    (f,) = trace.frames()
    check_frame(f, arp)
    pulses = trace.nlps()
    for a, b in pulses:
        check_nlp(trace, a, b)
    before = [a for a, _ in pulses if a < f.decoded.start]
    after = [a for a, _ in pulses if a > f.decoded.idle_at]
    assert len(before) >= 3 and len(after) >= 3
    check_period(before, p)
    check_period(after, p)
    assert after[0] - f.decoded.idle_at == p * UNIT
    assert f.decoded.start - before[-1] < p * UNIT               # none was due
    assert loom.pop(0) == [GOOD]


@model_only("16 ms a pulse: 640,000 clocks each, about 100 s of Icarus each")
def test_eth_tx_nlp_16ms_after_a_frame(backend):
    """The reset period, P = 4000: the first NLP 16 ms (640,000 clocks) after
    the frame's end, the next 16 ms later, inside 16 +- 8 ms."""
    arp = arp_request()
    bench, trace, loom = boot(backend, arp, period=4000)
    loom.push(0, [0x8000 | len(arp)])
    wait_for(bench, trace, "frame", 1, frame_clocks(60) + 4000)
    wait_for(bench, trace, "nlp", 2, 2 * 640_000 + 4000)
    (f,) = trace.frames()
    starts = [a for a, _ in trace.nlps()]
    assert starts[0] - f.decoded.idle_at == 640_000
    check_period(starts, 4000)
    for a, b in trace.nlps():
        check_nlp(trace, a, b)


# --------------------------------------------------------------- underrun
def test_eth_tx_underrun_reported_and_frame_rejected(backend):
    """The feeder halted mid-frame (a slow feed, as the serializer sees it):
    the serializer ends the frame with TP_IDL and no FCS, the receiver rejects
    it, the program reports 0x800C (UNDER, FDONE) and the next frame is good."""
    data = sample_frame(300, 3)
    bench, trace, loom = boot(backend, data)
    loom.push(0, [0x8000 | len(data)])
    ok = bench.run_until(lambda: trace.run > 1200, 20000, every=16)
    assert ok
    loom.halt([0])
    bench.step(400)                                  # 6.25 words: > one window
    loom.run([0])
    wait_for(bench, trace, "frame", 1, 4000)
    assert loom.pop(0) == [UNDERRUN]
    arp = arp_request()
    words = words_of(arp)
    loom.push(0, [len(words)] + words + [0x8000 | len(arp)])
    wait_for(bench, trace, "frame", 2, frame_clocks(60) + 30000)
    cut, good = trace.frames()
    assert not cut.ok and not cut.fcs_ok
    assert cut.sfd_found and len(cut.decoded.bits) % 16 == 0      # whole words, then the tail
    assert len(cut.data) < len(data)
    assert rx.TP_IDL_MIN_NS <= cut.tp_idl_ns <= rx.TP_IDL_MAX_NS
    check_frame(good, arp)
    assert loom.pop(0) == [GOOD]


@model_only("a 772-octet frame is 25 k clocks; the length checks are control flow")
def test_eth_tx_largest_frame_and_refused_lengths(backend):
    """768 bytes (the whole buffer) is sent and good; L = 0 and L = 769 are
    refused with 0x8020 and send nothing."""
    data = sample_frame(MAXB, 9)
    bench, trace, loom = boot(backend, data)
    loom.push(0, [0x8000, 0x8000 | (MAXB + 1), 0x8000 | MAXB])
    wait_for(bench, trace, "frame", 1, frame_clocks(MAXB) + 6000)
    (f,) = trace.frames()
    check_frame(f, data)
    assert loom.pop(0, 3) == [REFUSED, REFUSED, GOOD]
