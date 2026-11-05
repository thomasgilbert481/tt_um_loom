"""L3-BRIDGE-DSHOT on the model and the RTL: firmware/bridge_dshot_uart.loom.

Two threads of one program, no host in the loop once it runs: thread 0
drives bidirectional DShot600 on BIDIR0 to ``tools.protomodels.dshot
.DshotEsc``, which checks every frame against its windows and answers with
GCR eRPM telemetry; thread 0 decodes each reply and queues it in data
memory, and thread 2 prints one ASCII line per reply, 8N1, on OUT0, decoded
by ``tools.protomodels.uart.UartRx``. The host loads the image, sets the two
ticks, starts threads 0 and 2 and pushes the interval and the throttle.

Expected values come from the specifications and the program header, never
from what the model or the RTL did: the DShot figures from the sources
``tools.protomodels.dshot`` cites (20 ticks a bit, 7 and 14 low, the reply
GCR-coded at 5/4 of the rate, its checksum inverted), the line format, the
queue depth and the drop rule of the header, and the UART bit time (one
bit is one tick of thread 2; every edge is checked against that grid from
its frame's start edge).

Boards: 48 MHz (DShot600's tick exactly 4 clocks, 1 Mbaud exactly 48) on
the RTL, so every DShot and UART edge is exact to the clock; 50 MHz with
115200 baud, where the 8 kHz stream overruns the UART, on the model.

The PHY-GL-SDF candidate is ``test_bridge_prints_one_line_per_reply``: five
frames at 8 kHz, five replies, five lines, every edge to the clock.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import dshot as D
from tools.protomodels.uart import UartRx
from tools.tests.fw_backend import model_only
from tools.tests.test_fw_dshot import (B600, B600_48, check_frames, check_starts,
                                       interval, word)

ISA = load()
SOURCE = REPO / "firmware" / "bridge_dshot_uart.loom"
IMEM = 512
PROGRAM = assemble_file(SOURCE, isa=ISA, imem_words=IMEM, strict=True)
SYM = PROGRAM.symbols
SHARED, QUEUE, Q = SYM["SHARED"], SYM["QUEUE"], 8       # the header's layout
WR, RD, LOST = SHARED, SHARED + 1, SHARED + 2
RATE_I = 75                                             # 8 kHz at DShot600
NO_REPLY, BAD_GCR, BAD_SUM = 0x8001, 0x8002, 0x8003

#: Thread 2's tick for each board: 1 Mbaud (exact), 115200 baud.
UART_1M = {B600_48: (48, 0), B600: (50, 0)}
UART_115200 = (434, 8)                                  # at 50 MHz


def line(word_):
    """The header's line for a queued word."""
    if word_ & 0x8000:
        return "! %s\r\n" % {NO_REPLY: "NRP", BAD_GCR: "GCR", BAD_SUM: "CRC"}[word_]
    return "E %03X\r\n" % (word_ & 0xFFF)


def dline(count):
    return "D %04X\r\n" % (count & 0xFFFF)


# ------------------------------------------------------------- the bench
class Rig:
    """The bridge on a bench: the ESC model on BIDIR0 (with the board's
    pull-up), the UART decoder on OUT0."""

    def __init__(self, backend, board, *, uart_tick=None, image=None, **esc):
        if backend.name == "model":
            bench = backend.bench(features=("FIFO", "SETPD", "BE", "BEENC", "DMEM"),
                                  imem_words=IMEM, pullups=0b1)
        else:
            bench = backend.bench(pullups=0b1)          # RtlBench has them all
        self.board, self.bench = board, bench
        self.tick = uart_tick or UART_1M[board]
        self.bit = self.tick[0] + self.tick[1] / 256.0  # clocks a UART bit
        self.esc = bench.add(D.DshotEsc("BIDIR0", clock_mhz=board.clock_mhz,
                                        rate=board.rate, **esc))
        self.uart = bench.add(UartRx("OUT0", self.bit))
        words = dict(PROGRAM.words)
        words.update(image or {})
        self.loom = loom = backend.loom(bench, isa=ISA)
        loom.load(words)
        loom.write_csr(0, "TICK_INT", board.tick[0])
        loom.write_csr(0, "TICK_FRAC", board.tick[1])
        loom.write_csr(2, "TICK_INT", self.tick[0])
        loom.write_csr(2, "TICK_FRAC", self.tick[1])
        loom.run([0, 2], exclusive=True)
        bench.step(200)                         # both threads past their set-up

    def stream(self, value, bits=RATE_I):
        """The two host words: the interval, then the throttle."""
        self.loom.push(0, [interval(bits), word(value)])

    def text(self):
        return self.uart.data.decode("ascii")

    def lines(self):
        return [s + "\n" for s in self.text().split("\n")[:-1]]

    def until_lines(self, count, limit):
        assert self.bench.run_until(lambda: self.text().count("\n") >= count,
                                    limit, every=256), \
            (self.text(), self.esc.rx.report())
        return self.lines()[:count]

    def frame_clocks(self, bits=RATE_I):
        return int(bits * self.board.bit)

    def line_clocks(self, chars=8):
        return int(chars * 10 * self.bit)


def assert_uart_timing(uart, bit):
    """Every frame 8N1 with a good stop bit and every edge of it within one
    clock of the grid ``k * bit`` from its start edge (exactly on it when
    the tick is a whole number of clocks: SEMANTICS 4 and 6.10); the
    characters of a line back to back, start edges ten bits apart."""
    exact = bit == int(bit)
    assert uart.errors == [], uart.errors
    for frame in uart.frames:
        for k, off in uart.edge_offsets(frame):
            assert (off == 0) if exact else abs(off) <= 1.0, (frame.start, k, off)
    starts = [f.start for f in uart.frames]
    for a, b in zip(starts, starts[1:]):
        if b - a < 11 * bit:                    # the same line
            assert abs(b - a - 10 * bit) <= (0 if exact else 1.0), (a, b)


def check_dshot(rig, bits=None):
    """The FC's inverted frames inside every window and on the tick grid,
    the line never driven both ways, and with ``bits`` the frame starts
    exactly that many bit periods apart."""
    check_frames(rig.esc.rx, rig.board)
    assert all(f.bidir for f in rig.esc.frames)
    assert rig.bench.contention_free(), rig.bench.contentions[:4]
    if bits:
        check_starts(rig.esc.frames, bits, rig.board)


# ------------------------------------------------------------- the tests
#: Stopped (0xFFF), eRPM periods with exponents 0 .. 3 (dshot.erpm_value,
#: BF-TLM's eeem mmmm mmmm), an EDT temperature frame (type 0x2, 50).
TLM = [0x0FFF, D.erpm_value(300), D.erpm_value(1234), D.erpm_value(3000), 0x0232]


def test_bridge_prints_one_line_per_reply(backend):
    """The 8 kHz stream (I = 75) of one throttle: five frames, five
    replies, five lines in order, the value of each in three hex digits;
    the frames exactly 75 bits apart while the UART prints; every DShot and
    UART edge exact to the clock at 48 MHz; nothing dropped."""
    rig = Rig(backend, B600_48, values=TLM)
    rig.stream(1000)
    got = rig.until_lines(5, 6 * rig.frame_clocks() + rig.line_clocks())
    assert got == [line(v) for v in TLM], got
    assert got[0] == "E FFF\r\n" and got[4] == "E 232\r\n"
    assert [r.word for r in rig.esc.replies[:5]] == [D.telemetry_word(v) for v in TLM]
    assert [f.frame for f in rig.esc.frames[:5]] == \
        [D.encode_frame(1000, False, bidir=True)] * 5
    check_dshot(rig, RATE_I)
    assert_uart_timing(rig.uart, rig.bit)
    # each line starts after its reply was decoded, before the next reply
    ends = [r.start + 21 * rig.esc.tbit for r in rig.esc.replies]
    starts = [f.start for f in rig.uart.frames][::7]
    for k, s in enumerate(starts[:4]):
        assert ends[k] < s < ends[k + 1], (k, ends[k], s)
    loom = rig.loom
    assert loom.badop() == 0
    loom.halt([0, 2])
    wr, rd, lost = loom.read_imem(WR, 3)
    assert lost == 0 and wr - rd <= 1, (wr, rd, lost)


def test_bridge_reports_missing_and_bad_replies(backend):
    """A good reply, a bad checksum, a code GCR never uses, no reply, a
    good reply: ``E``, ``! CRC``, ``! GCR``, ``! NRP``, ``E``, in order,
    and the stream keeps its grid across all of them."""
    values = [0x0123, 0x0456, 0x0789, 0x0ABC, 0x0DEF]
    rig = Rig(backend, B600_48, values=values,
              faults=["", "crc", "gcr", "none", ""])
    rig.stream(1500)
    got = rig.until_lines(5, 6 * rig.frame_clocks() + rig.line_clocks())
    assert got == ["E 123\r\n", "! CRC\r\n", "! GCR\r\n", "! NRP\r\n", "E DEF\r\n"], got
    check_dshot(rig, RATE_I)
    assert_uart_timing(rig.uart, rig.bit)
    assert rig.loom.badop() == 0


def preload(words_, counts=None):
    """Image words that put ``words_`` in the queue as if thread 0 had
    published them, each after ``counts[k]`` drops (0 by default): slots
    0, 1, ... and WR = their count."""
    image = {WR: len(words_)}
    for k, w in enumerate(words_):
        image[QUEUE + 2 * k] = w
        image[QUEUE + 2 * k + 1] = counts[k] if counts else 0
    return image


def walk(records, values):
    """The header's rule on (word, drops before it) records: the n values
    before each record's word were dropped. Returns how many values the
    records account for."""
    k = 0
    for w, n in records:
        k += n
        assert w == values[k], (w, n, k, values[k])
        k += 1
    return k


def account(lines_, values):
    """The same rule on the UART: every reply printed once, in order, or
    counted, and each D line of n exactly where its n replies are missing.
    Returns the printed values, the D counts and the values accounted for."""
    records, counts, n = [], [], 0
    for s in lines_:
        if s.startswith("D "):
            assert n == 0 and s == dline(int(s[2:6], 16)), s
            n = int(s[2:6], 16)
            counts.append(n)
            continue
        word_ = {"! NRP\r\n": NO_REPLY, "! GCR\r\n": BAD_GCR,
                 "! CRC\r\n": BAD_SUM}.get(s, None)
        word_ = int(s[2:5], 16) if word_ is None else word_
        assert s == line(word_), s
        records.append((word_, n))
        n = 0
    seen = walk(records, values)
    return [w for w, _ in records], counts, seen


def test_bridge_overrun_drops_and_counts(backend):
    """The overrun rule on the RTL, made short by preloading a full queue
    (Q = 8 records, WR = 8; the second record says 3 replies were dropped
    before it) and printing at 240 kbaud (200 clocks a bit, 14,000 clocks a
    line, 2.3 frames of the 8 kHz stream). The UART gives the first record,
    the D line, the second; meanwhile the first reply takes the slot freed
    by the first record and the next ones find the queue full and are
    dropped and counted. The frame stream never slips (no back-pressure),
    and every reply is accounted for: the records thread 0 published, with
    the drops before each, and LOST, the drops not yet told."""
    pre = [0x0100 + k for k in range(Q)]
    values = [0x0200 + k for k in range(16)]
    rig = Rig(backend, B600_48, uart_tick=(200, 0), values=values,
              image=preload(pre, [0, 3] + [0] * (Q - 2)))
    rig.stream(1200)
    got = rig.until_lines(3, 3 * rig.line_clocks() + 2000)
    assert got == [line(pre[0]), dline(3), line(pre[1])], got
    loom = rig.loom
    loom.push(0, [interval(0)])                 # stop the stream, so the
    rig.bench.step(2 * rig.frame_clocks())      #   last reply is decoded
    loom.halt([0, 2])
    wr, rd, lost = loom.read_imem(WR, 3)
    queue = loom.read_imem(QUEUE, 2 * Q)
    replies = len(rig.esc.replies)
    records = [(queue[2 * (j % Q)], queue[2 * (j % Q) + 1]) for j in range(Q, wr)]
    assert replies >= 6 and wr - rd == Q and records, (replies, wr, rd)
    assert records[0] == (values[0], 0)         # the slot the first record freed
    assert walk(records, values) + lost == replies, (records, lost, replies)
    assert lost + sum(n for _, n in records) >= replies - 3, (records, lost)
    check_dshot(rig, RATE_I)
    assert_uart_timing(rig.uart, rig.bit)
    assert loom.badop() == 0


def drain(rig, limit):
    """Stop the stream (an interval word of 0) and let the UART empty the
    queue and print the last D line."""
    rig.loom.push(0, [interval(0)])
    rig.bench.step(2 * rig.frame_clocks())
    n = len(rig.esc.replies)
    last = rig.lines()

    def quiet():
        nonlocal last
        now = rig.lines()
        done = now == last and rig.uart.frames and \
            rig.bench.cycle - rig.uart.frames[-1].start > 3 * rig.line_clocks()
        last = now
        return done
    assert rig.bench.run_until(quiet, limit, every=4096), rig.text()
    assert len(rig.esc.replies) == n                    # the stream has stopped
    return rig.lines()




@model_only("30 frames and the drain at 115200 baud, 1.2 M clocks; the RTL runs the overrun with a preloaded queue")
def test_bridge_overrun_at_115200_baud(backend):
    """The 8 kHz stream at 50 MHz into a 115200 baud UART (608 us a line,
    4.9 frames): one reply on the wire and Q queued, then the queue is full
    and replies are dropped and counted, each D line right where its gap
    is; the frames keep their grid; once the stream stops and the UART
    drains, every reply has been printed once in order or counted (LOST
    holds the drops after the last record), and both counts agree."""
    values = [0x0300 + k for k in range(64)]
    rig = Rig(backend, B600, uart_tick=UART_115200, values=values)
    rig.stream(800)
    rig.bench.run_until(lambda: len(rig.esc.replies) >= 30,
                        31 * rig.frame_clocks() + 20000, every=1024)
    got = drain(rig, 20 * rig.line_clocks())
    replies = len(rig.esc.replies)
    printed, counts, seen = account(got, values[:replies])
    assert printed[:Q + 1] == values[:Q + 1]    # one on the wire, Q queued
    assert len(counts) >= 2 and all(counts), counts
    assert len(printed) < replies / 2           # the overrun is real
    check_dshot(rig)
    check_starts(rig.esc.frames[:replies - 1], RATE_I, rig.board)
    assert_uart_timing(rig.uart, rig.bit)
    loom = rig.loom
    loom.halt([0, 2])
    wr, rd, lost = loom.read_imem(WR, 3)
    assert wr == rd == len(printed) and seen + lost == replies, (wr, rd, lost, seen)


@model_only("the shortest-interval stream; the RTL runs the 8 kHz one")
def test_bridge_keeps_the_grid_at_i_60(backend):
    """The header's claim: with the queue in the reply path the frame grid
    does not slip at I = 60 (measured threshold; the reply 30 us after the
    frame), and at
    1 Mbaud every reply is still printed (70 us a line, 100 us a frame)."""
    values = [0x0400 + k for k in range(8)]
    rig = Rig(backend, B600_48, values=values)
    rig.stream(600, bits=60)
    got = rig.until_lines(8, 9 * rig.frame_clocks(60) + rig.line_clocks())
    assert got == [line(v) for v in values], got
    check_dshot(rig, 60)
    assert_uart_timing(rig.uart, rig.bit)


@model_only("one more 1 Mbaud stream at 50 MHz: the fractional DShot tick")
def test_bridge_at_50mhz(backend):
    """DShot600 at 50 MHz (4 + 43/256 clocks a tick, every edge within a
    clock of the grid) and 1 Mbaud exactly (50 clocks)."""
    rig = Rig(backend, B600, values=TLM)
    rig.stream(1000)
    got = rig.until_lines(5, 6 * rig.frame_clocks() + rig.line_clocks())
    assert got == [line(v) for v in TLM], got
    check_dshot(rig, RATE_I)
    assert_uart_timing(rig.uart, rig.bit)


# ------------------------------------------------------------ the program
def test_assembles_strict_with_no_diagnostic():
    """L2-DEADLINE: every pair of both threads proved at the declared ticks
    (4 clocks for DShot, 40 for the UART), nothing unbounded; the one
    declaration is dshot_bidir.loom's guarded INQ POP; the word map of the
    header: thread 0 to 0x0AC, thread 1 unstartable, the shared words,
    thread 2 in its own quarter, thread 3's quarter empty."""
    assert PROGRAM.diagnostics == []
    assert sorted(PROGRAM.threads) == [0, 2]
    r0, r2 = PROGRAM.deadlines[0], PROGRAM.deadlines[2]
    assert (r0.period, r2.period) == (4, 40)
    for r in (r0, r2):
        assert not r.infeasible and not r.unbounded
    assert (len(r0.pairs), len(r2.pairs)) == (17, 8)
    assert [d.name for d in r0.declarations] == ["POP"]
    assert r2.declarations == []
    addrs = sorted(PROGRAM.words)
    assert len(addrs) == 287
    t0 = [a for a in addrs if a < SHARED]
    assert max(t0) == 0x0AC and len(t0) == 173
    assert [a for a in addrs if SHARED <= a < 0x100] == [WR, RD, LOST]
    t2 = [a for a in addrs if 0x100 <= a]
    assert min(t2) == 0x100 and max(t2) == 0x16E and len(t2) == 111
    assert 1 in PROGRAM.unstartable and 3 not in PROGRAM.unstartable
    assert QUEUE == SHARED + 8 and SYM["LINE"] == QUEUE + 2 * Q == 0xF8
