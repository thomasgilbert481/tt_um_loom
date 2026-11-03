"""L3-BRIDGE-WIEG on the model and the RTL: firmware/bridge_wiegand_uart.loom.

Two threads of one program, no host in the loop: thread 0 is a Wiegand
panel on IN0 (D0) and IN1 (D1), fed by ``tools.protomodels.wiegand
.WiegandReader``; it queues each card in data memory, and thread 1 sends it
as one ASCII line, 8N1, on OUT0, decoded by ``tools.protomodels.uart
.UartRx``. The host loads the image, sets the two ticks and starts both
threads, and does nothing else.

Expected values come from the specifications and the program header, never
from what the model or the RTL did: the card formats (H10301 from
``tools.protomodels.wiegand``; the 34-bit H10306 layout below, written from
its published description, because the model has 26 and 37 bits only), the
line format, the queue depth and the overflow rule of the header, and the
UART bit time (one bit is ``clock / baud``; every edge is checked against
that grid from the frame's start edge).

Boards, as test_fw_wiegand.py has them. At 50 MHz a 26-bit card at SIA
AC-01's typical 1 ms interval is 1.3 M clocks, so the RTL runs the program
on a board clocked at 5 MHz: the panel's tick is 25 clocks (5 us), the
reader uses SIA's shortest interval, 200 us, with a 40 us pulse, the
frame end is patched in the image to 100 ticks (500 us), and the UART runs
at 115200 baud, 43.40 clocks a bit (TICK_INT 43, TICK_FRAC 103). The
50 MHz board (434 + 8/256 a bit, the default 25 ms frame end) and the
queue's overflow run on the model only.

The PHY-GL-SDF candidate is ``test_bridge_sends_26_and_34_bit_cards_as_lines``:
three cards in, three lines out, every UART edge to the clock.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import wiegand as W
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.uart import UartRx
from tools.tests.fw_backend import model_only

ISA = load()
SOURCE = REPO / "firmware" / "bridge_wiegand_uart.loom"
IMEM = 512
PROGRAM = assemble_file(SOURCE, isa=ISA, imem_words=IMEM, strict=True)
SYM = PROGRAM.symbols
SHARED, QUEUE, Q = SYM["SHARED"], SYM["QUEUE"], 8     # the header's layout
WR, RD, END = SHARED, SHARED + 1, SHARED + 2


class Board:
    """A clock and the host settings the program header gives for it."""

    def __init__(self, name, clock_hz, panel_tick, uart_tick, end):
        self.name, self.clock_hz = name, float(clock_hz)
        self.panel_tick = panel_tick
        self.uart_tick = uart_tick              # (TICK_INT, TICK_FRAC)
        self.end = end                          # panel ticks, None: the image's 5000

    def clocks(self, us):
        return W.us_to_clocks(us, self.clock_hz)

    def bit(self, baud=115200):
        return self.clock_hz / baud

    @property
    def end_clocks(self):
        return (5000 if self.end is None else self.end) * self.panel_tick

    def __repr__(self):
        return self.name


B5 = Board("5MHz", 5_000_000, 25, (43, 103), 100)
B50 = Board("50MHz", 50_000_000, 250, (434, 8), None)


# ------------------------------------------------------------ the formats
def h10306(facility, card):
    """The 34-bit frame (HID H10306 as published): bit 1 even parity over
    bits 2 to 17, a 16-bit facility code, a 16-bit card number, bit 34 odd
    parity over bits 18 to 33 (bits numbered from 1 in transmission order)."""
    bits = [0] + W.bit_list((facility << 16) | card, 32) + [0]
    bits[0] = W.parity(W.from_bits(bits[1:17]))
    bits[-1] = 1 - W.parity(W.from_bits(bits[17:33]))
    return W.from_bits(bits)


def parity_ok(value, n):
    """Both parities of a 26-bit (H10301) or 34-bit (H10306) frame."""
    if n == 26:
        return W.check_parities(value, 26)
    bits = W.bit_list(value, 34)
    return (W.parity(W.from_bits(bits[0:17])) == 0
            and W.parity(W.from_bits(bits[17:34])) == 1)


def line(value, n, *, lost=False, line_error=False):
    """The line the header specifies for an ``n``-bit frame."""
    shown = min(n, 48)
    digits = (shown + 3) // 4
    hexpart = "%0*X" % (digits, value & ((1 << shown) - 1))
    if line_error:
        status = "LE"
    elif n > 48:
        status = "OV"
    elif n in (26, 34):
        status = "OK" if parity_ok(value, n) else "PE"
    else:
        status = "--"
    return "%s%02d %s %s\r\n" % ("!" if lost else "W", min(n, 63), hexpart, status)


def record(value, n, *, lost=False):
    """A queue record as the header lays it out (for preloaded images):
    the status, then the last 48 bits as three words, most significant
    first. Only cards with good parities, so P alone of the flags."""
    status = min(n, 63) | (1 << 11 if n in (26, 34) else 0) | (1 << 12 if lost else 0)
    v = value & ((1 << 48) - 1)
    return [status, v >> 32, (v >> 16) & 0xFFFF, v & 0xFFFF]


def preload(cards):
    """Image words that put ``cards`` (``(value, n)``) in the queue as if
    thread 0 had published them: slots 0, 1, ... and WR = their count."""
    words = {WR: len(cards)}
    for k, (value, n) in enumerate(cards):
        for i, w in enumerate(record(value, n)):
            words[QUEUE + 4 * k + i] = w
    return words


# ------------------------------------------------------------- the bench
class Edges(Model):
    """Every change of one pad, as (cycle, level)."""

    def __init__(self, pin):
        self.pad, self.prev, self.log = pad_of(pin), None, []

    def observe(self, lines):
        level = lines.get(self.pad)
        if self.prev is not None and level != self.prev:
            self.log.append((lines.cycle, level))
        self.prev = level


class Rig:
    """The bridge on a bench: a model reader on IN0/IN1, the UART decoder
    and an edge log on OUT0 (TX), an edge log on OUT1 (READ)."""

    def __init__(self, backend, board, *, image=None, uart_tick=None, baud=115200,
                 threads=(0, 1), **reader_kwargs):
        if backend.name == "model":
            bench = backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=IMEM)
        else:
            bench = backend.bench()             # RtlBench has them all
        kwargs = dict(pw=board.clocks(W.TYPICAL_PW_US),
                      pi=board.clocks(W.PI_MIN_US) if board is B5 else None,
                      gap=board.end_clocks + board.clocks(1000))
        kwargs.update(reader_kwargs)
        self.board, self.bench = board, bench
        self.reader = bench.add(W.WiegandReader("IN0", "IN1", clock_hz=board.clock_hz, **kwargs))
        self.uart = bench.add(UartRx("OUT0", board.bit(baud)))
        self.tx, self.read = bench.add(Edges("OUT0")), bench.add(Edges("OUT1"))
        words = dict(PROGRAM.words)
        if board.end is not None:
            words[END] = board.end
        words.update(image or {})
        self.loom = loom = backend.loom(bench, isa=ISA)
        loom.load(words)
        tick = uart_tick or board.uart_tick
        loom.write_csr(0, "TICK_INT", board.panel_tick)
        loom.write_csr(1, "TICK_INT", tick[0])
        loom.write_csr(1, "TICK_FRAC", tick[1])
        loom.run(list(threads), exclusive=True)
        bench.step(400)                         # both threads past their set-up

    def text(self):
        return self.uart.data.decode("ascii")

    def lines(self):
        return [s + "\n" for s in self.text().split("\n")[:-1]]

    def until_lines(self, count, limit):
        """Step until ``count`` whole lines have been received."""
        assert self.bench.run_until(lambda: self.text().count("\n") >= count,
                                    limit, every=500), self.text()
        self.bench.step(int(2 * self.board.bit()))      # past the last stop bit
        return self.lines()

    def card_clocks(self, n):
        """A generous bound on one frame's time on the wire plus its end."""
        pi = self.reader.pi
        return n * pi + self.board.end_clocks + self.reader.gap + 4000

    def line_clocks(self, chars, baud=115200):
        return int(chars * 10 * self.board.bit(baud)) + 2000


def assert_uart_timing(uart, bit):
    """Every frame 8N1 with a good stop bit, and every edge of it within one
    clock of the nominal grid ``k * bit`` from its start edge (the program
    lands each bit on its tick with SETP ... D); characters of one line
    follow each other with no idle time, so successive start edges of a
    line are ten bits apart, to the clock."""
    assert uart.errors == [], uart.errors
    for frame in uart.frames:
        for k, off in uart.edge_offsets(frame):
            assert abs(off) <= 1.0, (frame.start, k, off)
    starts = [f.start for f in uart.frames]
    for a, b in zip(starts, starts[1:]):
        if b - a < 11 * bit:                    # the same line
            assert abs(b - a - 10 * bit) <= 1.0, (a, b)


# ------------------------------------------------------------- the tests
GOOD26 = W.h10301(0x12, 0x3456)
GOOD34 = h10306(0x1234, 0xBEEF)
BAD26 = GOOD26 ^ 1                              # bit 26, the odd parity, flipped


def test_bridge_sends_26_and_34_bit_cards_as_lines(backend):
    """A 26-bit card, a 34-bit card and a 26-bit card with a wrong odd
    parity: three lines, in order, ``OK``, ``OK``, ``PE``, the raw frames in
    hex; every UART edge on the 115200 baud grid; READ high once per frame,
    from the frame's first fall to its line being queued; afterwards both
    counts say three records published and taken."""
    rig = Rig(backend, B5)
    for value, n in ((GOOD26, 26), (GOOD34, 34), (BAD26, 26)):
        rig.reader.send(value, n)
    got = rig.until_lines(3, 3 * rig.card_clocks(34) + rig.line_clocks(18))
    assert got == [line(GOOD26, 26), line(GOOD34, 34), line(BAD26, 26)], got
    assert got[0] == "W26 02468AC OK\r\n"         # parities 0 and 0 around 0x123456
    assert_uart_timing(rig.uart, B5.bit())
    rises = [c for c, v in rig.read.log if v == 1]
    falls = [c for c, v in rig.read.log if v == 0]
    assert len(rises) == len(falls) == 3, rig.read.log
    for frame, up, down in zip(rig.reader.frames_sent, rises, falls):
        first, last = frame["pulses"][0][1], frame["pulses"][-1][2]
        assert 0 < up - first <= 60, (first, up)                # the poll latency
        assert down - last >= B5.end_clocks, (last, down)         # after the frame end
        assert down - last <= B5.end_clocks + B5.panel_tick + 400
    starts = [f.start for f in rig.uart.frames]
    assert all(s > falls[0] for s in starts)    # the first line follows the first record
    loom = rig.loom
    assert loom.badop() == 0
    loom.halt([0, 1])
    assert loom.read_imem(WR, 2) == [3, 3]


def test_bridge_threads_do_not_disturb_each_other(backend):
    """Three runs with the same host transactions. A: one card, an empty
    queue (the same image words as B, with WR = 0, so the four records are
    there but not published). B: the same card, with four 34-bit records
    preloaded, so thread
    1 is sending lines the whole time thread 0 reads the card and queues
    it. C: the four records and no card. READ (thread 0's timing) has the
    same edges, to the clock, in A and B; TX (thread 1's) has the same
    edges in B and C for the four preloaded lines; B's fifth line is the
    card's (SEMANTICS 2 and 6.11: neither thread waits for the other)."""
    pre = [(h10306(0x0100 + k, 0x2000 + k), 34) for k in range(4)]
    a = Rig(backend, B5, image={**preload(pre), WR: 0})
    a.reader.send(GOOD26, 26)
    a.until_lines(1, a.card_clocks(26) + a.line_clocks(16))
    b = Rig(backend, B5, image=preload(pre))
    b.reader.send(GOOD26, 26)
    got = b.until_lines(5, b.card_clocks(26) + 5 * b.line_clocks(18))
    c = Rig(backend, B5, image=preload(pre))
    c.until_lines(4, 4 * c.line_clocks(18))

    assert got == [line(v, n) for v, n in pre] + [line(GOOD26, 26)], got
    assert len(a.read.log) == 2 and a.read.log == b.read.log, (a.read.log, b.read.log)
    pre_end = c.uart.frames[-1].start + int(10 * B5.bit())
    assert b.read.log[1][0] < pre_end, "the card was queued while the UART was busy"
    assert b.reader.frames_sent[0]["pulses"][0][1] < c.uart.frames[0].start + 100
    quiet = [e for e in c.tx.log if e[0] <= pre_end]
    assert [e for e in b.tx.log if e[0] <= pre_end] == quiet and len(quiet) > 100
    assert c.lines() == got[:4] and c.read.log == []
    for rig in (a, b, c):
        assert_uart_timing(rig.uart, B5.bit())
        assert rig.loom.badop() == 0


@model_only("six frames, 200 k clocks; the RTL scenarios run the same parser")
def test_bridge_reports_other_lengths_glitches_and_line_errors(backend):
    """An 8-bit frame and a 37-bit H10304 card (no parity defined for the
    bridge: ``--``); 50 bits (``OV``, the last 48 in hex); a 26-bit card with
    both lines low for bit 6 (that pulse is not a bit: 25 bits, ``LE``); a
    frame of one glitch shorter than the 2-tick threshold (no line at all);
    a 26-bit card with bit 4 missing (25 bits, ``--``)."""
    rig = Rig(backend, B5)
    big = 0x2_ABCD_1234_5678 | 1 << 49
    both = W.bit_list(GOOD26, 26)
    del both[5]
    missing = W.bit_list(GOOD26, 26)
    del missing[3]
    rig.reader.send(0xA5, 8)
    rig.reader.send(W.h10304(0x1234, 0x5678A), 37)
    rig.reader.send(big, 50)
    rig.reader.send(GOOD26, 26, both=[5])
    rig.reader.send(1, 1, drop=[0], glitches=[(0, 0, B5.panel_tick)])
    rig.reader.send(GOOD26, 26, drop=[3])
    want = [line(0xA5, 8), line(W.h10304(0x1234, 0x5678A), 37), line(big, 50),
            line(W.from_bits(both), 25, line_error=True), line(W.from_bits(missing), 25)]
    got = rig.until_lines(5, 200000 + 6 * rig.card_clocks(0))
    assert got == want, got
    assert got[0] == "W08 A5 --\r\n" and got[2] == "W50 ABCD12345678 OV\r\n"
    assert len(rig.read.log) == 12                  # the glitch frame raised READ too
    rig.bench.step(rig.card_clocks(0))
    assert rig.lines() == want
    assert_uart_timing(rig.uart, B5.bit())


BAUD_9600 = (520, 213)                          # 5 MHz / 9600 = 520.83 clocks


@model_only("six cards and six lines at 9600 baud, 600 k clocks")
def test_bridge_queues_cards_that_arrive_faster_than_the_uart_drains(backend):
    """At 9600 baud a line takes 16.7 ms (26 bits) or 18.8 ms (34 bits) and
    the reader sends a card every 7 ms or so: the cards pile up in the
    queue (at least three waiting before the fourth line starts) and all
    six come out, in order, none marked lost."""
    rig = Rig(backend, B5, uart_tick=BAUD_9600, baud=9600)
    cards = [(W.h10301(k, 1000 + k), 26) if k % 2 else (h10306(k, 2000 + k), 34)
             for k in range(1, 7)]
    for value, n in cards:
        rig.reader.send(value, n)
    got = rig.until_lines(6, 6 * rig.line_clocks(18, 9600) + rig.card_clocks(34))
    assert got == [line(v, n) for v, n in cards], got
    falls = [c for c, v in rig.read.log if v == 0]
    first = [sum(len(s) for s in got[:k]) for k in range(6)]
    starts = [rig.uart.frames[i].start for i in first]
    assert len(falls) == 6 and falls[-1] < starts[3], (falls, starts)
    assert_uart_timing(rig.uart, B5.bit(9600))
    rig.loom.halt([0, 1])
    assert rig.loom.read_imem(WR, 2) == [6, 6]


@model_only("thirteen cards, 550 k clocks")
def test_bridge_drops_cards_past_the_queue_depth_and_says_so(backend):
    """Thread 1 is held (not started), so nothing drains: of eleven cards
    the first Q = 8 are queued and the last three dropped (the header's
    rule: the newest is dropped, the order kept). Once thread 1 runs, the
    eight come out in order; the next card's line starts with ``!``, the
    one after with ``W`` again."""
    rig = Rig(backend, B5, threads=(0,))
    cards = [W.h10301(0x40 + k, 0x1000 + k) for k in range(13)]
    for value in cards[:11]:
        rig.reader.send(value, 26)
    assert rig.bench.run_until(lambda: not rig.reader.busy, 11 * rig.card_clocks(26), every=1000)
    rig.bench.step(B5.end_clocks + 2000)
    assert rig.tx.log == [] and len(rig.read.log) == 22
    rig.loom.run(1)
    got = rig.until_lines(8, 8 * rig.line_clocks(16))
    rig.reader.send(cards[11], 26)
    rig.reader.send(cards[12], 26)
    got = rig.until_lines(10, 2 * rig.card_clocks(26) + 2 * rig.line_clocks(16))
    assert got == [line(v, 26) for v in cards[:8]] + [
        line(cards[11], 26, lost=True), line(cards[12], 26)], got
    assert got[8].startswith("!26 ")
    assert_uart_timing(rig.uart, B5.bit())
    rig.loom.halt([0, 1])
    assert rig.loom.read_imem(WR, 2) == [10, 10]


B50_FAST = Board("50MHz", 50_000_000, 250, (434, 8), 100)


@model_only("115200 baud at 50 MHz is 434 clocks a bit; the RTL runs the 5 MHz board")
def test_bridge_at_50_mhz(backend):
    """The suggested settings at 50 MHz: the panel's tick 250 clocks (5 us),
    the UART's 434 + 8/256 (115200.4 baud), here with SIA's shortest
    interval (200 us) and a frame end of 100 ticks for speed: one 34-bit
    card, its line, every edge on the 115200 baud grid."""
    rig = Rig(backend, B50_FAST, pi=B50_FAST.clocks(W.PI_MIN_US))
    rig.reader.send(GOOD34, 34)
    got = rig.until_lines(1, rig.card_clocks(34) + rig.line_clocks(18))
    assert got == [line(GOOD34, 34)] and got[0].endswith(" OK\r\n"), got
    assert_uart_timing(rig.uart, B50_FAST.bit())
    assert B50_FAST.uart_tick[0] + B50_FAST.uart_tick[1] / 256 == pytest.approx(
        B50_FAST.bit(), abs=1 / 256)              # the nearest 1/256 of a clock


def test_bridge_assembles_strict_with_every_pair_proved():
    """--strict and no diagnostic; thread 1's every deadline pair proved at
    its .tick of 40 clocks; thread 0 has no pair by design (timeouts only,
    as wiegand_panel.loom); each thread's code in its own quarter, the data
    in thread 2's (the header's layout), thread 3's quarter free."""
    assert PROGRAM.diagnostics == [] and sorted(PROGRAM.threads) == [0, 1]
    report = PROGRAM.deadlines[1]
    assert report.period == 40
    assert report.pairs and not report.infeasible and not report.unbounded
    assert report.worst_slack >= 0
    assert PROGRAM.deadlines[0].pairs == []
    words = sorted(PROGRAM.words)
    code0 = [a for a in words if a < 128]
    code1 = [a for a in words if 128 <= a < 256]
    data = [a for a in words if a >= 256]
    assert (len(code0), len(code1)) == (122, 128)
    assert data == [WR, RD, END] and PROGRAM.words[END] == 5000
    assert (SHARED, QUEUE, SYM["LINE"]) == (256, 264, 296)
    assert SYM["LINE"] + 21 <= 384             # the longest line, below thread 3
