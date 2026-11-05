"""L3-ARINC429: the ARINC 429 transmitter and receiver, on the model and the RTL.

``firmware/arinc429_tx.loom`` (thread 0, a line driver's inputs on OUT0 and
OUT1) and ``firmware/arinc429_rx.loom`` (thread 1, a line receiver's outputs
on IN0 and IN1) are assembled, loaded and driven through
``tools.loomhost.Loom``. ``tools.protomodels.arinc429.ArincLine`` decodes
every word the transmitter sends and holds it to the transmit windows
(rate, pulse width, HI/NULL ratio, gap, the driver's slew);
``ArincPlayer`` plays words into the receiver at any rate, with bad
parity, short gaps, short words and both lines high; ``Wire`` joins OUT0
and OUT1 to IN0 and IN1 for the two programs together. Every body that
takes a ``backend`` runs here on the golden model and under cocotb on the
RTL (``test/test_fw.py``).

The expected numbers come from the sources the model cites (the Holt
HI-8282A datasheet's ARINC 429 timing table and receiver rules, and the
word layout of the Alta Data Technologies tutorial) and from
``docs/SEMANTICS.md``, never from what the model or the RTL happened to
do: the transmitter's tick is half a bit and SEMANTICS 6.10 lands each
deadline-latched edge on the exact clock edge of its tick, so at 50 MHz
and 100 kbit/s every pulse is exactly 250 clocks, every bit 500 and every
word 36 bits (4 of gap) apart. The receiver must take every bit rate
Holt's 8-to-12-sample rule takes (the spec's +-1 % and 12 to 14.5 kbit/s
inside it) and refuse rates outside it.

High speed runs at 50 MHz (18 k clocks a word); low speed runs on a 10 MHz
board on the RTL (29 k clocks a word) and at 50 MHz on the model only.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import arinc429 as A
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
TX, RX = 0, 1                                  # threads


class Board:
    """A board clock and the ticks the program headers give for it."""

    def __init__(self, mhz):
        self.mhz = mhz

    def tick(self, per_second):
        """``(TICK_INT, TICK_FRAC)`` for ``per_second`` ticks (SEMANTICS 4)."""
        p = int(round(self.mhz * 1e6 * 256 / per_second))
        return p >> 8, p & 0xFF

    def clocks(self, us):
        return int(round(us * self.mhz))

    def __repr__(self):
        return "%dMHz" % self.mhz


B50, B10 = Board(50), Board(10)
HS, LS = 100_000.0, 12_500.0                   # nominal rates, bit/s

_PROGRAMS = {}


def program(name):
    if name not in _PROGRAMS:
        _PROGRAMS[name] = assemble_file(FIRMWARE / (name + ".loom"), isa=ISA,
                                        strict=True)
    return _PROGRAMS[name]


def make_bench(backend):
    """FIFOs, the deadline latch, the bit engine, data memory and the
    512-word memory; the RTL has them all."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM)
    return backend.bench()


def boot(backend, bench, board, *, tx_rate=None, rx_rate=None, gap=None,
         raw=False, labels=None):
    """Load the programs asked for, set their ticks (TX: half a bit, RX: a
    twentieth of a nominal bit), patch the TX configuration word and the RX
    label table, start them."""
    loom = backend.loom(bench, isa=ISA)
    threads = []
    if tx_rate:
        prog = program("arinc429_tx")
        words = dict(prog.words)
        if gap is not None or raw:
            words[prog.symbols["cfg"]] = (gap if gap is not None else 4) | (0x8000 if raw else 0)
        loom.load(words)
        ti, tf = board.tick(2 * tx_rate)
        loom.write_csr(TX, "TICK_INT", ti)
        loom.write_csr(TX, "TICK_FRAC", tf)
        threads.append(TX)
    if rx_rate:
        prog = program("arinc429_rx")
        words = dict(prog.words)
        if labels is not None:
            base = prog.symbols["ltab"]
            for k in range(16):
                words[base + k] = sum(1 << j for j in range(16) if 16 * k + j in labels)
        loom.load(words)
        ti, tf = board.tick(20 * rx_rate)
        loom.write_csr(RX, "TICK_INT", ti)
        loom.write_csr(RX, "TICK_FRAC", tf)
        threads.append(RX)
    loom.run(threads)
    return loom


# Three words: label 0o205 with every field set, 0o310 with a full data
# field, 0o001 with nothing else (its parity bit is the only other 1).
W1 = A.word(0o205, 0x12345, sdi=1, ssm=3)
W2 = A.word(0o310, 0x7FFFF, sdi=2, ssm=0)
W3 = A.word(0o001)
WORDS = [W1, W2, W3]


def odd(v):
    return A.with_parity(v)


# ------------------------------------------------------- the transmitter
def tx_setup(backend, board, rate, **kw):
    bench = make_bench(backend)
    line = bench.add(A.ArincLine("OUT0", "OUT1", clock_mhz=board.mhz))
    return bench, line, boot(backend, bench, board, tx_rate=rate, **kw)


def word_us(rate, gap=4):
    return (32 + gap) * 1e6 / rate


def send(bench, line, loom, board, rate, values, gap=4, junk_parity=True, total=None):
    """Push ``values`` (bit 32 set wrongly when ``junk_parity``, to show
    the program ignores it) as INQ room allows; step until all are on the
    line (``total`` words in all, if more were pushed before) and the gap
    after the last has passed."""
    words = []
    for v in values:
        words += A.host_words(odd(v) ^ 0x80000000 if junk_parity else v)
    budget = board.clocks(word_us(rate, gap) * (len(values) + 2))
    pushed = 0
    while pushed < len(words):
        room = 4 - loom.fifo_status(TX)["inq"]
        if room >= 2:
            loom.push(TX, words[pushed:pushed + 2])
            pushed += 2
        else:
            bench.step(board.clocks(1e6 / rate))
    total = len(values) if total is None else total
    assert bench.run_until(lambda: len(line.words) >= total, budget, every=256), line.report()
    bench.step(board.clocks(word_us(rate, gap) - 32e6 / rate))
    return line.words


def test_arinc_tx_high_speed_exact(backend):
    """100 kbit/s at 50 MHz: the three words with odd parity, label MSB
    first; every pulse exactly 250 clocks (5 us), every bit 500 (10 us),
    the words 36 bits apart (a gap of exactly 4); inside every transmit
    window with a 1.5 us driver slew; the two lines never both high."""
    board = B50
    bench, line, loom = tx_setup(backend, board, HS)
    assert send(bench, line, loom, board, HS, WORDS) == [odd(v) for v in WORDS]
    assert line.check("high", slew_us=1.5) == [], line.report()
    assert {p.fall - p.rise for p in line.pulses} == {250}
    groups = line.word_pulses()
    assert {q.rise - p.rise for w in groups for p, q in zip(w, w[1:])} == {500}
    assert [b[0].rise - a[0].rise for a, b in zip(groups, groups[1:])] == [18000, 18000]
    assert loom.badop() == 0


def test_arinc_tx_low_speed(backend):
    """12.5 kbit/s on a 10 MHz board (TICK 400): pulses exactly 40 us,
    bits 80 us, inside the low-speed windows with a 10 us slew."""
    board = B10
    bench, line, loom = tx_setup(backend, board, LS)
    assert send(bench, line, loom, board, LS, WORDS[:2]) == [odd(v) for v in WORDS[:2]]
    assert line.check("low", slew_us=10.0) == [], line.report()
    assert {p.fall - p.rise for p in line.pulses} == {400}
    assert loom.badop() == 0


@model_only("50 MHz low speed with fractional ticks: 144 k clocks a word; the 10 MHz run covers the code")
@pytest.mark.parametrize("rate", [12_000.0, 14_500.0])
def test_arinc_tx_low_speed_edges_at_50mhz(backend, rate):
    """12 and 14.5 kbit/s, the ends of the low-speed range, at 50 MHz
    (TICK 2083 + 85/256 and 1724 + 35/256): inside every window, and every
    pulse within one clock of half the ideal bit."""
    board = B50
    bench, line, loom = tx_setup(backend, board, rate)
    assert send(bench, line, loom, board, rate, [W1]) == [odd(W1)]
    assert line.check("low", slew_us=10.0) == [], line.report()
    ideal = board.mhz * 1e6 / (2 * rate)
    assert all(abs((p.fall - p.rise) - ideal) <= 1 for p in line.pulses)


@model_only("the configuration word; the RTL runs the default")
def test_arinc_tx_gap_and_raw_parity(backend):
    """G = 6: words 38 bits apart. RAW: bit 32 goes out as the host gave
    it, so a word with even parity reaches the line unchanged."""
    board = B50
    bench, line, loom = tx_setup(backend, board, HS, gap=6, raw=True)
    bad = odd(W1) ^ 0x80000000
    assert send(bench, line, loom, board, HS, [bad, odd(W2)], gap=6, junk_parity=False) == [bad, odd(W2)]
    assert not A.parity_ok(line.words[0])
    groups = line.word_pulses()
    assert groups[1][0].rise - groups[0][0].rise == 38 * 500
    assert line.check("high") == []


@model_only("a late host; the RTL keeps INQ fed")
def test_arinc_tx_waits_for_a_late_high_half(backend):
    """The low half alone, then the high half 2 ms later: the line stays
    NULL meanwhile, the word goes out whole and the gap before it is only
    longer (every window still met)."""
    board = B50
    bench, line, loom = tx_setup(backend, board, HS)
    lo, hi = A.host_words(odd(W2))
    loom.push(TX, [lo])
    bench.step(board.clocks(2000))
    assert line.pulses == []
    loom.push(TX, [hi])
    assert send(bench, line, loom, board, HS, [W3], total=2) == [odd(W2), odd(W3)]
    assert line.check("high") == []


# ---------------------------------------------------------- the receiver
def rx_setup(backend, board, nominal, labels=None):
    bench = make_bench(backend)
    player = bench.add(A.ArincPlayer("IN0", "IN1", clock_mhz=board.mhz))
    return bench, player, boot(backend, bench, board, rx_rate=nominal, labels=labels)


def collect(bench, loom, board, us, chunk_us):
    """Step ``us`` of board time, emptying OUTQ every ``chunk_us`` (a word
    time or less, so a report always has room)."""
    out, left = [], board.clocks(us)
    while left > 0:
        step = min(left, board.clocks(chunk_us))
        bench.step(step)
        left -= step
        out += loom.pop_available(RX)
    return out


def play(bench, player, loom, board, batches):
    """Queue ``batches`` of ``(values, player keywords)`` after 6 bit times
    of NULL, run them through, return the parsed reports."""
    total, chunk = 0.0, None
    for k, (values, kw) in enumerate(batches):
        kw = dict(kw)
        bit_us = kw.setdefault("bit_us", 10.0)
        if k == 0:
            kw["lead_bits"] = 6
            total += 6 * bit_us
        player.queue_words(values, **kw)
        total += len(values) * (32 + kw.get("gap_bits", 4)) * bit_us
        chunk = min(chunk or 1e9, 30 * bit_us)
    words = collect(bench, loom, board, total + 200, chunk)
    return A.parse_reports(words)


def good(v):
    return A.Report(A.S_MARK | A.S_WORD, odd(v))


def test_arinc_rx_high_speed(backend):
    """100 kbit/s, nominal timing, back-to-back words with the minimum
    gap: each reported with status 0x8080, label in octal order, parity
    good."""
    board = B50
    bench, player, loom = rx_setup(backend, board, HS)
    assert play(bench, player, loom, board, [(WORDS, {})]) == [good(v) for v in WORDS]
    assert loom.badop() == 0


@pytest.mark.parametrize("rate", [99_000.0, 101_000.0])
def test_arinc_rx_high_speed_spec_edges(backend, rate):
    """The +-1 % of the high-speed rate (HI-8282A page 3)."""
    board = B50
    bench, player, loom = rx_setup(backend, board, HS)
    got = play(bench, player, loom, board, [([W1, W2], dict(bit_us=1e6 / rate))])
    assert got == [good(W1), good(W2)]


def test_arinc_rx_low_speed_range(backend):
    """The low-speed range at its two ends, 12 and 14.5 kbit/s, into a
    receiver set for 12.5 kbit/s, on a 10 MHz board."""
    board = B10
    bench, player, loom = rx_setup(backend, board, LS)
    got = play(bench, player, loom, board, [([W1], dict(bit_us=1e6 / 12_000)),
                                           ([W2], dict(bit_us=1e6 / 14_500))])
    assert got == [good(W1), good(W2)]


@model_only("Holt's acceptance edges; the RTL runs the spec's")
@pytest.mark.parametrize("board, nominal, rate", [
    (B50, HS, 84_000.0), (B50, HS, 124_000.0),
    (B10, LS, 10_400.0), (B10, LS, 15_800.0), (B50, LS, 12_000.0)])
def test_arinc_rx_holt_acceptance_range(backend, board, nominal, rate):
    """Holt's 8 to 12 samples a bit accepts 83 to 125 kbit/s and 10.4 to
    15.6 kbit/s; the program's header guarantees 83.6 to 124.4 at 50 MHz
    and 10.3 to 15.9 on a 10 MHz board (its stamping error): just inside
    each end is received, and low speed at 50 MHz too."""
    bench, player, loom = rx_setup(backend, board, nominal)
    got = play(bench, player, loom, board, [([W1, W3], dict(bit_us=1e6 / rate))])
    assert got == [good(W1), good(W3)]


@model_only("rates outside the window; the RTL runs the error scenario")
@pytest.mark.parametrize("rate", [72_000.0, 160_000.0])
def test_arinc_rx_refuses_rates_outside_the_window(backend, rate):
    """Under 73.9 and over 154.8 kbit/s, which the header says are always
    refused (a bit 27.8 or 12.5 ticks after the one before): E_RATE with one bit
    received, the rest of the word ignored, no word reported."""
    board = B50
    bench, player, loom = rx_setup(backend, board, HS)
    got = play(bench, player, loom, board, [([W1], dict(bit_us=1e6 / rate))])
    assert got == [A.Report(A.S_MARK | 1 << 8 | A.E_RATE)]


def err(e, bits=0):
    return A.Report(A.S_MARK | bits << 8 | e)


def test_arinc_rx_errors(backend):
    """Bad parity is reported with the word, flagged E_PAR; a word after a
    2-bit gap is dropped (E_GAP); both lines high in bit 6 gives E_BOTH
    after 5 bits; a 20-bit word E_SHORT; each time the next good word is
    received whole."""
    board = B50
    bench, player, loom = rx_setup(backend, board, HS)
    got = play(bench, player, loom, board, [
        ([W1], dict(parity="even", gap_bits=2)),
        ([W2, W3], dict()),
        ([W1], dict(both_at=5)),
        ([W2], dict(nbits=20)),
        ([W3], dict())])
    bad = A.Report(A.S_MARK | A.S_WORD | A.E_PAR, A.with_parity(W1, odd=False))
    assert got == [bad, err(A.E_GAP), good(W3), err(A.E_BOTH, 5),
                   err(A.E_SHORT, 20), good(W3)]
    assert loom.badop() == 0


@model_only("a pulse with no NULL and a bit too late; the RTL runs the other errors")
def test_arinc_rx_pulse_and_rate_errors(backend):
    """A word with no NULL at all, each pulse the whole 10 us bit: label
    0o205 sends a HI and then five LOs, which are one 50 us LO pulse, so
    E_NULL after 1 bit (bit 1's HI ends as bit 2's LO starts, which the
    receiver sees as an instant of NULL). A word whose 10th bit is 4 us
    late (28 ticks): E_RATE after 9 bits. Then a good word."""
    board = B50
    bench, player, loom = rx_setup(backend, board, HS)
    player.queue_words([W1], hi_us=10.0, lead_bits=6)
    bits = A.wire_bits(odd(W2))
    seg = []
    for k, b in enumerate(bits):
        seg += [(5.0, b, 1 - b), (5.0 + (4.0 if k == 8 else 0.0), 0, 0)]
    player.queue(seg + [(40.0, 0, 0)])
    player.queue_words([W3])
    got = A.parse_reports(collect(bench, loom, board, 3 * 360 + 200, 300))
    assert got == [err(A.E_NULL, 1), err(A.E_RATE, 9), good(W3)]


@model_only("the label table; the RTL runs the default table")
def test_arinc_rx_label_filter(backend):
    """A table that wants labels 0o205 and 0o001 only: the 0o310 word is
    dropped with no report."""
    board = B50
    bench, player, loom = rx_setup(backend, board, HS, labels={0o205, 0o001})
    assert play(bench, player, loom, board, [(WORDS, {})]) == [good(W1), good(W3)]


# ------------------------------------------------- the two together
def test_arinc_tx_and_rx_on_two_threads(backend):
    """Transmitter (thread 0) on OUT0/OUT1, wired by the bench to the
    receiver (thread 1) on IN0/IN1, both at 100 kbit/s: the three words
    on the line meet every transmit window, and the receiver reports
    each, parity good."""
    board = B50
    bench = make_bench(backend)
    bench.add(A.Wire("OUT0", "IN0"))
    bench.add(A.Wire("OUT1", "IN1"))
    line = bench.add(A.ArincLine("OUT0", "OUT1", clock_mhz=board.mhz))
    loom = boot(backend, bench, board, tx_rate=HS, rx_rate=HS)
    bench.step(board.clocks(80))                # 8 bits of NULL first
    loom.push(TX, A.host_words(W1) + A.host_words(W2))
    reports = collect(bench, loom, board, 2 * 360, 300)
    loom.push(TX, A.host_words(W3))
    reports += collect(bench, loom, board, 360 + 100, 300)
    assert line.words == [odd(v) for v in WORDS]
    assert line.check("high", slew_us=1.5) == [], line.report()
    assert A.parse_reports(reports) == [good(v) for v in WORDS]
    assert loom.badop() == 0


# ------------------------------------------------------- the programs
@pytest.mark.parametrize("name, thread, words, pairs, bounded", [
    ("arinc429_tx", 0, 67, 16, ["POP", "POP"]),
    ("arinc429_rx", 1, 121, 0, []),
])
def test_arinc_assembles_strict_with_no_diagnostic(name, thread, words, pairs, bounded):
    """L2-DEADLINE: every half-bit pair of the transmitter proved at the
    declared 32-clock tick, the declarations the two guarded POPs only;
    the receiver has no WAITD (it follows the line)."""
    prog = program(name)
    assert prog.diagnostics == []
    assert list(prog.threads) == [thread]
    report = prog.deadlines[thread]
    assert not report.infeasible and not report.unbounded
    assert len(report.pairs) == pairs
    assert [d.name for d in report.declarations] == bounded
    assert len(prog.words) == words
    if pairs:
        assert report.period == 32 and report.worst_slack >= 0


def test_arinc_programs_fit_together():
    """D-020: the transmitter in thread 0's quarter, the receiver and its
    label table in thread 1's; no shared word, every thread startable."""
    tx, rx = program("arinc429_tx"), program("arinc429_rx")
    assert max(tx.words) < 128
    assert min(rx.words) == 128 and max(rx.words) < 256
    assert rx.symbols["ltab"] + 15 == max(rx.words)
    assert not set(tx.words) & set(rx.words)
    assert not tx.unstartable and not rx.unstartable


# ------------------------------------------------------ the model itself
def test_arinc_model_word_layout():
    """The Alta tutorial's example: label 0o012 goes out 0, 0, 0, 0, 1, 0,
    1, 0 (its MSB first). SDI in bits 9-10, data 11-29, SSM 30-31, odd
    parity in 32; the host words and the wire order round-trip."""
    assert A.wire_bits(A.word(0o012))[:8] == [0, 0, 0, 0, 1, 0, 1, 0]
    v = A.word(0o377, (1 << 19) - 1, sdi=3, ssm=3)
    assert A.arinc_bits_value(v) == 0x7FFFFFFF
    bits = A.wire_bits(A.word(0, 1, sdi=2, ssm=1))
    assert bits[8:10] == [0, 1] and bits[10] == 1 and bits[29:31] == [1, 0]
    for w in WORDS:
        p = odd(w)
        assert A.parity_ok(p) and not A.parity_ok(p ^ 1 << 31)
        assert A.from_wire(A.wire_bits(p)) == p
        assert A.from_host(*A.host_words(p)) == p
    assert odd(W3) >> 31 == 0 and A.ones(odd(W3)) == 1


def feed(model, player, mhz=50.0):
    """Clock ``player``'s two lines (``ui[0]``, ``ui[1]``) through ``model``."""
    from tools.protomodels.bench import Lines
    lines, c = Lines(), 0
    while not player.idle:
        lines.cycle, lines.ui = c, player.a | player.b << 1
        player.observe(lines)
        model.observe(lines)
        c += 1


def checked(speed="high", slew=None, **kw):
    p = A.ArincPlayer(("ui", 0), ("ui", 1))
    p.queue_words([W1, W2], **kw)
    line = A.ArincLine(("ui", 0), ("ui", 1))
    feed(line, p)
    return line, {v.kind for v in line.check(speed, slew_us=slew)}


def test_arinc_model_line_checks():
    """The nominal stream passes with a 1.5 us slew and decodes; each
    injected fault is caught: 3-bit gap, 102 kbit/s, a 6 us pulse, both
    lines high, a 2.5 us slew, a 31-bit word, and low-speed timing
    against the high-speed windows."""
    line, kinds = checked(slew=1.5)
    assert kinds == set() and line.words == [odd(W1), odd(W2)]
    assert checked(gap_bits=3)[1] == {"gap"}
    assert checked(bit_us=1e6 / 102_000)[1] == {"rate"}
    assert checked(hi_us=6.0)[1] == {"width", "ratio"}
    assert "both" in checked(both_at=3)[1]
    assert checked(slew=2.5)[1] == {"slew"}
    assert "count" in checked(nbits=31)[1]
    assert "rate" in checked(bit_us=80.0)[1]
    assert checked("low", bit_us=80.0)[1] == set()
