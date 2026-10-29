"""L3 for the Wiegand programs, on the model and the RTL.

``firmware/wiegand_reader.loom`` (D0 = BIDIR0, D1 = BIDIR1, open drain) is
the card reader; ``firmware/wiegand_panel.loom`` (D0 = IN0, D1 = IN1) is the
door controller. The references are ``tools.protomodels.wiegand``:
``WiegandPanel`` decodes the reader's pads and checks every pulse against
SIA AC-01's windows (pulse 20 to 100 us, interval 200 us to 20 ms, the
other line high throughout), and ``WiegandReader`` feeds the panel frames
with chosen widths, intervals, jitter and faults. Expected values come from
the format definitions and the windows in the model's docstring and from
the program headers' stated thresholds, never from what the model or the
RTL did.

Two boards, as the NEC infrared tests have. Both programs take their tick
from the host. At 50 MHz the reader's tick is 250 clocks (5 us), so the
typical 50 us pulse every 1 ms is PW 10, IV 200, and a 26-bit frame is
26 ms, 1.3 M clocks: minutes on the RTL. So the RTL runs the same programs
on a board clocked at 5 MHz, a slow test clock: the reader's tick is then
100 clocks (20 us, the shortest tick its schedule is proved for), the
panel's 25 (5 us), and the frames use SIA's shortest interval, 200 us
(IV 10), with a 40 us pulse (PW 2), so a 26-bit frame is 26,000 clocks. The
panel's inter-frame timeout on that board is set by the host to 100 ticks
(500 us), long enough for a reader known to use 200 us intervals; SIA's
widest interval needs the default 25 ms, which the 50 MHz bodies use. The
50 MHz bodies and the 20 ms intervals are ``model_only``.

The PHY-GL-SDF candidate is
``test_reader_sends_a_26_bit_card_with_exact_pulses``: 26 latched falls and
26 latched releases on two open-drain pads, every edge to the clock, in
about 30,000 clocks.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import wiegand as W
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
READER, PANEL = FIRMWARE / "wiegand_reader.loom", FIRMWARE / "wiegand_panel.loom"

# reader commands (the header of wiegand_reader.loom)
K_FRAME, K_CARD26, K_WIDTH, K_PERIOD, K_GAP = (k << 13 for k in range(5))
# panel status bits (the header of wiegand_panel.loom)
P26, PERR, GLITCH, BOTH, LONG, OVER = (1 << b for b in (8, 9, 10, 11, 12, 13))
STARTUP = 400                   # clocks for a program's set-up after RUN


class Board:
    """A clock and the host settings the program headers give for it."""

    def __init__(self, name, clock_hz, reader_tick, panel_tick, pw, iv, end):
        self.name, self.clock_hz = name, float(clock_hz)
        self.reader_tick, self.panel_tick = reader_tick, panel_tick
        self.pw, self.iv = pw, iv               # reader ticks
        self.end = end                          # panel ticks, None: the default

    def clocks(self, us):
        return W.us_to_clocks(us, self.clock_hz)

    @property
    def end_clocks(self):
        return (5000 if self.end is None else self.end) * self.panel_tick

    def __repr__(self):
        return self.name


B5 = Board("5MHz", 5_000_000, 100, 25, 2, 10, 100)
B50 = Board("50MHz", 50_000_000, 250, 250, 10, 200, None)


def program(path, thread=0):
    """A program from ``firmware/`` with ``--strict``, moved to ``thread``."""
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, path.name, isa=ISA, imem_words=IMEM, strict=True)


def wbench(backend, pullups=0):
    """The FIFOs, the deadline latch and the 512-word memory; the RTL has
    them all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD"), imem_words=IMEM,
                             pullups=pullups)
    return backend.bench(pullups=pullups)


def frame_words(status):
    """How many bit words follow a panel status word."""
    n = status & 0xFF if not status & OVER else 48
    return (min(n, 48) + 15) // 16


# ------------------------------------------------------------- assembly
def test_wiegand_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: the reader's every pair is proved at its .tick of 100
    clocks, nothing unbounded and no ``.bounded``; the panel has no deadline
    pair by design (timeouts only). Each fits thread 0's quarter, and with
    the panel moved to thread 1 the two images do not overlap."""
    reader = program(READER)
    assert reader.diagnostics == [] and list(reader.threads) == [0]
    report = reader.deadlines[0]
    assert report.period == 100
    assert report.pairs and not report.infeasible and not report.unbounded
    assert report.worst_slack >= 0
    assert len(reader.words) == 107 and max(reader.words) < 0x80
    panel = program(PANEL)
    assert panel.diagnostics == [] and panel.deadlines[0].pairs == []
    assert len(panel.words) == 109 and max(panel.words) < 0x80
    moved = program(PANEL, thread=1)
    assert min(moved.words) == 0x80 and max(moved.words) < 0x100
    assert not set(reader.words) & set(moved.words)


# --------------------------------------------------------------- reader
def reader_setup(backend, board, pw=None, iv=None):
    """The reader on thread 0, a model panel on its pads (pull-ups on both)."""
    bench = wbench(backend, pullups=0b11)
    mon = bench.add(W.WiegandPanel("BIDIR0", "BIDIR1", clock_hz=board.clock_hz))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(READER))
    loom.write_csr(0, "TICK_INT", board.reader_tick)
    loom.run(0)
    pw = board.pw if pw is None else pw
    iv = board.iv if iv is None else iv
    loom.push(0, [K_WIDTH | pw, K_PERIOD | iv])
    bench.step(STARTUP)
    return bench, mon, loom, pw, iv


def reader_send(bench, mon, board, iv, n, budget_extra=2000):
    """Run until the reader's ``n`` pulses are out and the frame has ended."""
    tick = board.reader_tick
    limit = (n + 2) * iv * tick + budget_extra
    target = len(mon.pulses) + n
    assert bench.run_until(lambda: len(mon.pulses) >= target, limit, every=64)
    bench.step(iv * tick)
    mon.flush()


def check_exact(mon, board, pw, iv, n):
    """Every pulse ``pw`` ticks wide and every fall ``iv`` ticks after the
    one before, to the clock; inside SIA's windows; never both lines."""
    tick = board.reader_tick
    pulses = mon.frames[-1]["pulses"]
    assert len(pulses) == n
    assert all(rise - fall == pw * tick for _, fall, rise in pulses)
    falls = [fall for _, fall, _ in pulses]
    assert all(b - a == iv * tick for a, b in zip(falls, falls[1:]))
    assert mon.violations == []


def test_reader_sends_a_26_bit_card_with_exact_pulses(backend):
    """CARD26: the program computes both parities. The model panel decodes
    facility 0x12, card 0x3456 with good parities; 26 pulses of exactly
    40 us every 200 us at 5 MHz (SIA's shortest interval). The PHY-GL-SDF
    candidate."""
    board = B5
    bench, mon, loom, pw, iv = reader_setup(backend, board)
    loom.push(0, [K_CARD26, 0x12, 0x3456])
    reader_send(bench, mon, board, iv, 26)
    frame = mon.frames[-1]
    assert frame["n"] == 26 and frame["value"] == W.h10301(0x12, 0x3456)
    assert W.check_parities(frame["value"], 26)
    check_exact(mon, board, pw, iv, 26)
    assert bench.contention_free() and loom.badop() == 0


@pytest.mark.parametrize("fc, card", [(0, 0), (0xFF, 0xFFFF), (0xA5, 0x0F0F)])
def test_reader_computes_both_parities(backend, fc, card):
    """Cards whose parities are each 0 and 1, at both ends of the fields."""
    board = B5
    bench, mon, loom, pw, iv = reader_setup(backend, board)
    loom.push(0, [K_CARD26, fc, card])
    reader_send(bench, mon, board, iv, 26)
    assert mon.frames[-1]["value"] == W.h10301(fc, card)
    assert W.decode26(mon.frames[-1]["value"]) == (fc, card)
    assert mon.violations == []


@pytest.mark.parametrize("n", [34, 37])
def test_reader_sends_long_frames(backend, n):
    """FRAME: 34 bits (a raw pattern) and 37 bits (H10304, the parities
    computed by the model and sent as data), MSB first, every pulse exact."""
    board = B5
    bench, mon, loom, pw, iv = reader_setup(backend, board)
    value = W.h10304(54321, 432109) if n == 37 else 0x2_DEAD_BEEF & ((1 << n) - 1)
    loom.push(0, [K_FRAME | n] + W.to_words(value, n))
    reader_send(bench, mon, board, iv, n)
    assert mon.frames[-1]["n"] == n and mon.frames[-1]["value"] == value
    if n == 37:
        assert W.check_parities(value, 37)
    check_exact(mon, board, pw, iv, n)


def test_reader_sends_a_bad_parity_card_raw_and_a_gap(backend):
    """A 26-bit frame with its odd parity flipped goes out as FRAME data,
    unchanged (the reader does not check what it is given); GAP holds the
    lines idle; a 1-bit frame and a 48-bit frame, the extremes of N, follow;
    N = 0 and N = 49 are dropped and send nothing."""
    board = B5
    bench, mon, loom, pw, iv = reader_setup(backend, board)
    bad = W.h10301(0x12, 0x3456) ^ 1
    loom.push(0, [K_FRAME | 26] + W.to_words(bad, 26) + [K_GAP | 50])
    reader_send(bench, mon, board, iv, 26)
    assert mon.frames[-1]["value"] == bad and not W.check_parities(bad, 26)
    loom.push(0, [K_FRAME | 0, K_FRAME | 49, K_FRAME | 1, 1])
    reader_send(bench, mon, board, iv, 1)
    assert (mon.frames[-1]["n"], mon.frames[-1]["value"]) == (1, 1)
    big = 0xA5A5_0F0F_C3C3
    loom.push(0, [K_FRAME | 48] + W.to_words(big, 48))
    reader_send(bench, mon, board, iv, 48)
    assert (mon.frames[-1]["n"], mon.frames[-1]["value"]) == (48, big)
    assert len(mon.frames) == 3 and mon.violations == []


@pytest.mark.parametrize("pw", [1, 5])
def test_reader_at_the_pulse_width_limits(backend, pw):
    """PW 1 and 5 ticks at 5 MHz: SIA's 20 us and 100 us, exactly."""
    board = B5
    bench, mon, loom, pw, iv = reader_setup(backend, board, pw=pw)
    loom.push(0, [K_FRAME | 8, 0xA5])
    reader_send(bench, mon, board, iv, 8)
    assert mon.frames[-1]["value"] == 0xA5
    check_exact(mon, board, pw, iv, 8)


@model_only("a 20 ms interval is 100,000 clocks a bit even at 5 MHz")
def test_reader_at_the_widest_interval(backend):
    """IV 1000 ticks at 5 MHz: SIA's 20 ms, exactly, with a 100 us pulse."""
    board = B5
    bench, mon, loom, pw, iv = reader_setup(backend, board, pw=5, iv=1000)
    loom.push(0, [K_FRAME | 3, 0b101])
    reader_send(bench, mon, board, iv, 3)
    assert mon.frames[-1]["value"] == 0b101
    check_exact(mon, board, pw, iv, 3)


@model_only("a 26-bit frame at 50 MHz and 1 ms intervals is 1.3 M clocks")
def test_reader_at_50_mhz_typical_timing(backend):
    """The suggested set-up: tick 250 (5 us), PW 10, IV 200: the typical
    50 us pulse every 1 ms, to the clock."""
    board = B50
    bench, mon, loom, pw, iv = reader_setup(backend, board)
    loom.push(0, [K_CARD26, 200, 51234])
    reader_send(bench, mon, board, iv, 26)
    assert mon.frames[-1]["value"] == W.h10301(200, 51234)
    check_exact(mon, board, pw, iv, 26)
    assert pw * board.reader_tick == board.clocks(50)
    assert iv * board.reader_tick == board.clocks(1000)


# ---------------------------------------------------------------- panel
def panel_setup(backend, board, thread=0, **reader_kwargs):
    """The panel on ``thread``, a model reader on IN0 and IN1."""
    bench = wbench(backend)
    kwargs = dict(pw=board.clocks(W.TYPICAL_PW_US),
                  pi=board.clocks(W.PI_MIN_US) if board is B5 else None,
                  gap=board.end_clocks + board.clocks(1000))
    kwargs.update(reader_kwargs)
    rdr = bench.add(W.WiegandReader("IN0", "IN1", clock_hz=board.clock_hz, **kwargs))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(PANEL, thread))
    if board.end is not None:
        loom.push(thread, [board.end])
    loom.write_csr(thread, "TICK_INT", board.panel_tick)
    loom.run(thread)
    bench.step(STARTUP)
    return bench, rdr, loom


def panel_frame(bench, rdr, loom, board, thread=0):
    """Run until the model reader is done and the panel's timeout has
    passed; return ``(status, value)`` of the one frame reported."""
    limit = rdr.idle_at() - bench.cycle + board.end_clocks + 4000
    assert bench.run_until(lambda: not rdr.busy, limit, every=64)
    bench.step(board.end_clocks + 2000)
    status = loom.pop(thread, 1)[0]
    words = loom.pop(thread, frame_words(status)) if status & 0xFF or status & OVER else []
    assert loom.pop_available(thread) == []
    return status, W.from_words(words)


@pytest.mark.parametrize("flip", [None, "even", "odd"])
def test_panel_checks_26_bit_parity(backend, flip):
    """A good card: N = 26, P26 and no PERR, the frame back as sent. Either
    parity bit flipped: PERR."""
    board = B5
    bench, rdr, loom = panel_setup(backend, board)
    value = W.h10301(0x12, 0x3456)
    if flip == "even":
        value ^= 1 << 25
    elif flip == "odd":
        value ^= 1
    rdr.send(value, 26)
    status, got = panel_frame(bench, rdr, loom, board)
    assert status & 0xFF == 26 and status & P26 and got == value
    assert bool(status & PERR) == (flip is not None)
    assert status & (GLITCH | BOTH | LONG | OVER) == 0
    assert loom.badop() == 0


@pytest.mark.parametrize("n", [1, 34, 37, 48])
def test_panel_counts_frames_of_any_length(backend, n):
    """N other than 26: no parity check, the bits back right-aligned."""
    board = B5
    bench, rdr, loom = panel_setup(backend, board)
    value = (W.h10304(4660, 300000) if n == 37 else
             0x9_1234_5678_9ABC & ((1 << n) - 1)) | 1
    rdr.send(value, n)
    status, got = panel_frame(bench, rdr, loom, board)
    assert status == n and got == value


@pytest.mark.parametrize("pw_us", [W.PW_MIN_US, W.PW_MAX_US])
def test_panel_at_the_pulse_width_limits(backend, pw_us):
    """SIA's narrowest (20 us) and widest (100 us) pulse at its shortest
    interval (200 us), with +-2 us of jitter on every fall."""
    board = B5
    bench, rdr, loom = panel_setup(backend, board, pw=board.clocks(pw_us),
                                   jitter=board.clocks(2))
    value = W.h10301(0x5A, 0xC33C)
    rdr.send(value, 26)
    status, got = panel_frame(bench, rdr, loom, board)
    assert status == 26 | P26 and got == value


@model_only("26 bits at SIA's widest interval, 20 ms, is 26 M clocks at 50 MHz; "
            "three bits are enough and still 3 M")
def test_panel_at_the_widest_interval_and_50_mhz(backend):
    """50 MHz, tick 250, the default 25 ms timeout: three bits 20 ms apart
    (SIA's widest interval) are one frame, not three."""
    board = B50
    bench, rdr, loom = panel_setup(backend, board, pi=board.clocks(W.PI_MAX_US))
    rdr.send(0b101, 3)
    status, got = panel_frame(bench, rdr, loom, board)
    assert status == 3 and got == 0b101


def test_panel_ignores_glitches_and_reports_them(backend):
    """A 6 us low pulse on D0 and one on D1 between bits: each is seen by the
    poll (longer than its 24 clocks) and rejected (shorter than G = 10 us),
    so the frame is intact and GLITCH is set. A 20 us pulse is a bit."""
    board = B5
    bench, rdr, loom = panel_setup(backend, board, pw=board.clocks(W.PW_MIN_US))
    value = W.h10301(0x01, 0x8001)
    width = board.clocks(6)
    rdr.send(value, 26, glitches=[(3, 0, width), (11, 1, width)])
    status, got = panel_frame(bench, rdr, loom, board)
    assert status == 26 | P26 | GLITCH and got == value


def test_panel_reports_both_lines_low(backend):
    """Bit 5 pulses both lines at once: BOTH, that pulse not counted, the
    other 25 bits kept in order (so N = 25 and no parity check)."""
    board = B5
    bench, rdr, loom = panel_setup(backend, board)
    value = W.h10301(0x77, 0x1234)
    rdr.send(value, 26, both=[5])
    status, got = panel_frame(bench, rdr, loom, board)
    bits = W.bit_list(value, 26)
    expect = W.from_bits(bits[:5] + bits[6:])
    assert status == 25 | BOTH and got == expect


def test_panel_reports_a_missing_bit_as_a_short_frame(backend):
    """A bit left out (its slot empty): the panel sees 25 bits, so the frame
    is not a 26-bit card and the host can tell from N."""
    board = B5
    bench, rdr, loom = panel_setup(backend, board)
    value = W.h10301(0x12, 0x3456)
    rdr.send(value, 26, drop=[20])
    status, got = panel_frame(bench, rdr, loom, board)
    bits = W.bit_list(value, 26)
    assert status == 25 and got == W.from_bits(bits[:20] + bits[21:])


def test_panel_reports_a_stuck_line_and_recovers(backend):
    """A pulse of 300 ticks (more than the 255-tick limit): LONG, the pulse
    not counted (N = 0, no bit words); the next frame, after the line is
    released, is clean."""
    board = B5
    bench, rdr, loom = panel_setup(backend, board)
    rdr.send(0b1, 1, pw=300 * board.panel_tick)
    status, got = panel_frame(bench, rdr, loom, board)
    assert status == LONG
    rdr.send(0b0110, 4)
    status, got = panel_frame(bench, rdr, loom, board)
    assert (status, got) == (4, 0b0110)


def test_panel_keeps_the_last_48_bits_of_a_longer_frame(backend):
    """50 bits: N = 50, OVER, and the last 48 bits."""
    board = B5
    bench, rdr, loom = panel_setup(backend, board)
    value = (0b11 << 48) | 0x8421_1248_FEDC
    rdr.send(value, 50)
    status, got = panel_frame(bench, rdr, loom, board)
    assert status == 50 | OVER and got == value & ((1 << 48) - 1)


# ------------------------------------------------------------- together
def test_reader_and_panel_together(backend):
    """The reader on thread 0 (BIDIR0, BIDIR1, open drain, pulled up) and
    the panel on thread 1 (IN0, IN1), the bench wiring BIDIR0 to IN0 and
    BIDIR1 to IN1 as two board wires: a 26-bit card and a 37-bit frame go
    across, each pulse exact on the reader's pads, each frame reported by
    the panel with the right status."""
    board = B5
    bench = wbench(backend, pullups=0b11)
    mon = bench.add(W.WiegandPanel("BIDIR0", "BIDIR1", clock_hz=board.clock_hz))
    bench.add(W.Wire("BIDIR0", "IN0"))
    bench.add(W.Wire("BIDIR1", "IN1"))
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(READER).words)
    panel = program(PANEL, thread=1).words
    assert not set(words) & set(panel)
    words.update(panel)
    loom.load(words)
    loom.write_csr(0, "TICK_INT", board.reader_tick)
    loom.write_csr(1, "TICK_INT", board.panel_tick)
    loom.push(1, [board.end])
    loom.run([0, 1])
    loom.push(0, [K_WIDTH | board.pw, K_PERIOD | board.iv])
    bench.step(STARTUP)
    loom.push(0, [K_CARD26, 0x9C, 0x0421, K_GAP | 50])
    reader_send(bench, mon, board, board.iv, 26)
    check_exact(mon, board, board.pw, board.iv, 26)
    bench.step(board.end_clocks + 2000)
    assert loom.pop(1, 3) == [26 | P26] + W.to_words(W.h10301(0x9C, 0x0421), 26)
    value = W.h10304(1, 2)
    loom.push(0, [K_FRAME | 37] + W.to_words(value, 37))
    reader_send(bench, mon, board, board.iv, 37)
    check_exact(mon, board, board.pw, board.iv, 37)
    bench.step(board.end_clocks + 2000)
    assert loom.pop(1, 4) == [37] + W.to_words(value, 37)
    assert loom.pop_available(1) == []
    assert bench.contention_free() and loom.badop() == 0


# ------------------------------------------------- the reference model alone
def test_model_formats():
    """H10301 and H10304 from their definitions: the fields come back, both
    parities check, and flipping any single bit breaks a parity."""
    v = W.h10301(0x12, 0x3456)
    assert v == (0x123456 << 1) and W.decode26(v) == (0x12, 0x3456)
    for n, frame in ((26, v), (37, W.h10304(54321, 432109))):
        assert W.check_parities(frame, n)
        for k in range(n):
            assert not W.check_parities(frame ^ (1 << k), n), (n, k)
    assert W.to_words(v, 26) == [0x24, 0x68AC] and W.from_words([0x24, 0x68AC]) == v


def test_model_reader_and_panel_agree_and_check_windows():
    """The model reader on two plain pins, the model panel watching them:
    a legal frame decodes with no violation; a 10 us pulse, a 150 us
    interval and both lines low are each reported."""
    from tools.protomodels.bench import Bench
    bench = Bench(features=("FIFO",))
    rdr = bench.add(W.WiegandReader("IN0", "IN1", clock_hz=5e6, gap=3000))
    mon = bench.add(W.WiegandPanel("IN0", "IN1", clock_hz=5e6))
    rdr.send(0b1011, 4)
    bench.step(rdr.idle_at() + 10)
    mon.flush()
    assert mon.frames[-1]["value"] == 0b1011 and mon.violations == []
    rdr.send(0b10, 2, pw=50)
    rdr.send(0b10, 2, pi=750)
    rdr.send(0b10, 2, both=[0])
    bench.step(rdr.idle_at() - bench.cycle + 10)
    mon.flush()
    kinds = [kind for _, kind, _ in mon.violations]
    assert kinds.count("width") == 2 and "interval" in kinds and "both" in kinds
