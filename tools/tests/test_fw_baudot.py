"""L3 for the ITA2 (Baudot) teleprinter programs, on the model and the RTL.

``firmware/baudot_tx.loom`` (TX = OUT0) sends five-unit codes from the
host as 7.5-unit start-stop characters; ``firmware/baudot_rx.loom`` (RX =
IN0) samples characters at mid-element and gives the host the codes, with
a framing-error bit; ``firmware/bridge_uart_baudot.loom`` takes ASCII on a
UART (thread 0, IN0), inserts the shifts and sends ITA2 (thread 1, OUT0),
with a flow-control output CTS_N on OUT1. The references are
``tools.protomodels.baudot`` (ITU-T S.1 for the code and the shift rules,
CCITT R.103 for the 7.5-unit character and its 1.5-unit stop element) and
``tools.protomodels.uart`` for the 8N1 side.

Expected values come from S.1 (Table 1, 4.4), from R.103 (sections 1, 2.2,
3.3) and from the program headers' stated tick and drop rules, never from
what the model or the RTL did. The transmitters' edges are latched
(SEMANTICS 6.10), so every element must be exactly 32 ticks and every
back-to-back stop element exactly 48, to the clock.

Scale. At 45.45 bauds a unit is 22 ms, 1.1 M clocks at 50 MHz, a character
8.25 M clocks: beyond either backend. The programs take their rate from
the host (32 ticks a unit), so the scenarios run the teleprinter side at
TICK_INT 16, a unit of 512 clocks (97.7 kbaud), and the bridge's UART at
TICK_INT 32, 256 clocks a bit. The program logic is the same at any rate;
only the tick changes. The 45.45, 50 and 75 baud tick settings are checked
by arithmetic (``test_baudot_tick_arithmetic``). RTL budget, stated: the
six scenarios without ``model_only`` total 488 k clocks (counted on the
model: tx_to_model 87 k, rx_from_model 98 k, rx_framing_error 28 k, the
two rx_off_rate 51 k and 52 k, bridge_mixed 172 k), inside the 770 k the
batch allows; the rest run on the model only, with the reason on each.

Every body here is named ``test_baudot_*``: test/test_fw.py collects the
bodies of all modules into one namespace.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import baudot as B
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.uart import UartTx
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512

TICK = 16                       # teleprinter side: clocks a tick in the scenarios
TPB = 32                        # ticks a unit (program headers)
UNIT = TICK * TPB               # 512 clocks
CHAR = int(7.5 * UNIT)          # a character with its 1.5-unit stop
UART_TICK = 32                  # the bridge's UART: 8 ticks a bit (header)
UART_BIT = 8 * UART_TICK        # 256 clocks
FE, OVR = 0x100, 0x200          # baudot_rx result bits (header)


def program(name):
    return assemble_file(FIRMWARE / (name + ".loom"), isa=ISA, imem_words=IMEM, strict=True)


def baudot_bench(backend):
    """The FIFOs, the deadline latch, the bit engine and LD/ST with the
    512-word memory on the model; the RTL has them all."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "DMEM"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, prog, ticks):
    """Load ``prog`` and run its threads, ``ticks`` = {thread: TICK_INT}."""
    loom = backend.loom(bench, isa=ISA)
    loom.load(dict(prog.words))
    for thread, tint in ticks.items():
        loom.write_csr(thread, "TICK_INT", tint)
        loom.write_csr(thread, "TICK_FRAC", 0)
    loom.run(sorted(ticks), exclusive=True)
    return loom


class Edges(Model):
    """Every change of a pad after it first reads 1, as (cycle, level)."""

    def __init__(self, pin="OUT0"):
        self.pad, self.prev, self.log = pad_of(pin), None, []

    def observe(self, lines):
        level = lines.get(self.pad)
        if self.prev is None:
            if level == 1:
                self.prev = 1
            return
        if level != self.prev:
            self.log.append((lines.cycle, level))
        self.prev = level


def feed_run(bench, loom, words, done, budget, chunk=1024):
    """Keep INQ[0] topped up from ``words`` while running until ``done``
    holds with every word sent, or ``budget`` clocks pass."""
    words, end = list(words), bench.cycle + budget
    while True:
        if words:
            free = (lambda st: st["depth"] - st["inq"])(loom.fifo_status(0))
            if free > 0:
                loom.push(0, words[:free])
                words = words[free:]
        if not words and done():
            return True
        if bench.cycle >= end:
            return False
        bench.run_until(done if not words else (lambda: False),
                        max(1, min(chunk, end - bench.cycle)), every=64)


def line_done(bench, edges, n):
    """A predicate: ``n`` characters decoded from the line and the line
    back at Z for 2 units."""
    def done():
        log = edges.log
        return (log and log[-1][1] == 1 and bench.cycle - log[-1][0] > 2 * UNIT
                and len(B.decode_edges(log, UNIT)) >= n)
    return done


# ------------------------------------------------------------ arithmetic
def test_baudot_tick_arithmetic():
    """The headers' tick settings give 1/32 unit at 50 MHz: 45.45 bauds
    (22 ms, the amateur rate, secondary source) and 50 bauds (R.103 1)
    exactly, 75 bauds within 1 ppm; a character with its 1.5-unit stop is
    7.5 units (R.103 1): 165 ms at 45.45 bauds, 150 ms at 50."""
    f = 50e6
    for unit_s, tint, tfrac in ((0.022, 34375, 0), (0.020, 31250, 0), (1 / 75, 20833, 85)):
        tick = tint + tfrac / 256
        assert tint < 1 << 16
        assert abs(tick * TPB / f - unit_s) / unit_s < 1e-6
    assert abs(7.5 * 0.022 - 0.165) < 1e-12 and abs(7.5 * 0.020 - 0.150) < 1e-12


# ------------------------------------------------------------ transmitter
def tx_setup(backend):
    bench = baudot_bench(backend)
    edges = bench.add(Edges("OUT0"))
    loom = start(backend, bench, program("baudot_tx"), {0: TICK})
    return bench, loom, edges


def test_baudot_tx_to_model(backend):
    """A mixed letters and figures line, codes queued ahead: every element
    exactly one unit and every stop element exactly 1.5 units, to the
    clock (R.103 1, 2.2; latched edges), and the reference decoder gives
    the text back (S.1 Table 1 and 4.4)."""
    text = "RYRY 73 DE LOOM."
    codes, dropped = B.encode(text)
    assert not dropped
    bench, loom, edges = tx_setup(backend)
    assert feed_run(bench, loom, codes, line_done(bench, edges, len(codes)),
                    (len(codes) + 4) * CHAR)
    assert B.decode_edges(edges.log, UNIT) == codes
    assert B.decode(B.decode_edges(edges.log, UNIT)) == text
    assert B.check_timing(edges.log, UNIT, tol=0, back_to_back=True) == []
    # Every edge on the unit grid from the first start edge, exactly.
    t0 = edges.log[0][0]
    assert all((c - t0) % (UNIT // 2) == 0 for c, _ in edges.log)


@model_only("the idle-gap paths repeat what test_baudot_tx_to_model proves "
            "about the element timing; run on the model to save RTL time")
def test_baudot_tx_gaps(backend):
    """Codes that arrive one at a time with idle between them: each stop
    element is at least 1.5 units (never shorter, R.103 3.3 asks 1.25) and
    each start edge lands on a tick, so the elements stay exact."""
    codes = [B.LTRS, 0x01, 0x10, B.FIGS, 0x01, 0x10, B.LTRS, 0x1C]
    bench, loom, edges = tx_setup(backend)
    for i, code in enumerate(codes):
        loom.push(0, [code])
        bench.run_until(line_done(bench, edges, i + 1), 3 * CHAR, every=64)
        bench.step(UNIT // 3 * (i + 1) + 5 * i)  # idle, every phase of a tick
    assert B.decode_edges(edges.log, UNIT) == codes
    assert B.check_timing(edges.log, UNIT, tol=0, back_to_back=False) == []
    starts = [c for c, v in edges.log if v == 0]
    assert all((s - starts[0]) % TICK == 0 for s in starts)


# --------------------------------------------------- the reference model
def test_baudot_model_code_and_checker():
    """The reference model against S.1 itself and against broken lines, so
    the firmware checks above cannot pass vacuously: Table 1 spot values,
    the 4.4 shift rules, and check_timing catching a 1-unit stop element
    and an element off the grid."""
    assert (B.LETTER[0x01], B.LETTER[0x10], B.FIGURE[0x01]) == ("E", "T", "3")
    assert (B.LTRS, B.FIGS, B.SPACE, B.CR, B.LF) == (0x1F, 0x1B, 0x04, 0x08, 0x02)
    assert len(B.LETTER) == 26 and len(B.FIGURE) == 23       # F, G, H undefined
    codes, dropped = B.encode("ab1 2c!")
    assert dropped == ["!"]
    assert codes == [B.LTRS, 0x03, 0x19, B.FIGS, 0x17, B.SPACE, B.FIGS, 0x13,
                     B.LTRS, 0x0E]
    assert B.decode(codes) == "AB1 2C" and B.decode(codes, usos=True) == "AB1 2C"
    # E (start, Z, A A A A), stop, then LTRS (start, Z Z Z Z Z).
    line = lambda stop: [(0, 0), (UNIT, 1), (2 * UNIT, 0), (6 * UNIT, 1),
                         (int((6 + stop) * UNIT), 0), (int((7 + stop) * UNIT), 1)]
    assert B.check_timing(line(1.5), UNIT) == []
    assert B.check_timing(line(1.0), UNIT, back_to_back=False) != []
    assert B.check_timing(line(1.75), UNIT, back_to_back=True) != []
    assert B.check_timing([(0, 0), (UNIT + 9, 1)], UNIT) != []
    assert B.decode_edges(line(1.5), UNIT) == [0x01, B.LTRS]


# --------------------------------------------------------------- receiver
def rx_setup(backend, unit=UNIT, stop_units=B.STOP_UNITS):
    bench = baudot_bench(backend)
    line = bench.add(B.BaudotTx("IN0", unit, stop_units=stop_units))
    loom = start(backend, bench, program("baudot_rx"), {0: TICK})
    bench.step(4 * UNIT)                        # the receiver armed, line idle
    return bench, loom, line


def rx_collect(bench, loom, line, n, budget):
    """Run until the sender is idle, popping OUTQ[0]; ``n`` words expected."""
    got, end = [], bench.cycle + budget
    while bench.cycle < end and (line.busy or len(got) < n):
        bench.step(UNIT)
        got += loom.pop_available(0)
    bench.step(2 * UNIT)
    return got + loom.pop_available(0)


def test_baudot_rx_from_model(backend):
    """A mixed letters and figures line from the reference encoder, back to
    back with 1.5-unit stops: every code arrives, no error bit, and the
    reference decoder gives the text (S.1 4.4)."""
    text = "CQ CQ DE LOOM 1/2-3"
    codes, _ = B.encode(text)
    bench, loom, line = rx_setup(backend)
    line.send(codes)
    got = rx_collect(bench, loom, line, len(codes), (len(codes) + 4) * CHAR)
    assert got == codes
    assert B.decode(got) == text


def test_baudot_rx_framing_error(backend):
    """A stop element held at start polarity is reported with bit 8 and the
    code; the next good character is received normally."""
    bench, loom, line = rx_setup(backend)
    line.send_frame(0x0A, stop=0)                # R, broken stop element
    line.send([B.LTRS, 0x10])
    got = rx_collect(bench, loom, line, 3, 8 * CHAR)
    assert got == [FE | 0x0A, B.LTRS, 0x10]


@pytest.mark.parametrize("error", [+0.02, -0.02])
def test_baudot_rx_off_rate(backend, error):
    """A sender 2 % fast (and 2 % slow) back to back: every code arrives
    (the stop sample moves 0.13 unit, header)."""
    codes, _ = B.encode("RYRY 73")
    bench, loom, line = rx_setup(backend, unit=UNIT / (1 + error))
    line.send(codes)
    got = rx_collect(bench, loom, line, len(codes), (len(codes) + 4) * CHAR)
    assert got == codes


@model_only("stop-length tolerance: the same sampling path as "
            "test_baudot_rx_from_model, run on the model to save RTL time")
@pytest.mark.parametrize("stop_units", [1.0, 1.25, 2.0])
def test_baudot_rx_stop_lengths(backend, stop_units):
    """Senders with 1, 1.25 (R.103 3.3's minimum) and 2-unit stops, back to
    back: all received (the stop is sampled half a unit in)."""
    codes, _ = B.encode("THE QUICK BROWN FOX 0123")
    bench, loom, line = rx_setup(backend, stop_units=stop_units)
    line.send(codes)
    got = rx_collect(bench, loom, line, len(codes), (len(codes) + 4) * CHAR)
    assert got == codes


# ----------------------------------------------------------------- bridge
class CtsUartTx(UartTx):
    """An 8N1 sender that, with ``honour``, starts a frame only while CTS_N
    (OUT1) reads 0 at the start, as a USB UART with hardware flow control
    does (it finishes a frame already started)."""

    def __init__(self, *args, honour=True, **kwargs):
        super().__init__(*args, **kwargs)
        self.honour, self.pending, self.cts_pad, self.cts = honour, [], pad_of("OUT1"), 0
        self.cts_high = 0                       # cycles seen with CTS_N = 1

    def observe(self, lines):
        self.cts = lines.get(self.cts_pad)
        self.cts_high += self.cts

    def queue(self, data):
        self.pending += list(data)

    def drive(self, drive, cycle):
        if self.pending and not self._changes and cycle >= self._free_at \
                and (self.cts == 0 or not self.honour):
            self.now = cycle
            self.send_frame(self.pending.pop(0))
        super().drive(drive, cycle)


def bridge_setup(backend, honour=True, uart_bit=UART_BIT, uart_tick=UART_TICK, tick=TICK):
    bench = baudot_bench(backend)
    uart = bench.add(CtsUartTx("IN0", uart_bit, honour=honour))
    line = bench.add(B.BaudotRx("OUT0", tick * TPB))
    edges = bench.add(Edges("OUT0"))
    loom = start(backend, bench, program("bridge_uart_baudot"), {0: uart_tick, 1: tick})
    bench.step(64)
    return bench, loom, uart, line, edges


def bridge_run(bench, uart, line, n, budget):
    """Run until ``n`` characters have come out and the line is idle."""
    end = bench.cycle + budget
    while bench.cycle < end:
        bench.step(UNIT)
        if not uart.pending and not uart.busy and len(line.frames) >= n:
            break
    bench.step(2 * UNIT)


def test_baudot_bridge_mixed(backend):
    """Mixed case letters, figures and punctuation, CR LF, bytes sent back
    to back at the full UART rate: the line carries exactly the reference
    encoder's codes (S.1 4.4: a shift first, LTRS and FIGS where the case
    changes, the repeated shift after a space in figure case), decodes to
    the text in upper case, and every element and stop element is exact
    to the clock (R.103 1, 2.2)."""
    text = "Ry 73, de Loom 1/2.\r\n"
    codes, dropped = B.encode(text)
    assert not dropped
    bench, loom, uart, line, edges = bridge_setup(backend)
    uart.queue(text.encode())
    bridge_run(bench, uart, line, len(codes), (len(codes) + 6) * CHAR)
    assert [f.framing_error for f in line.frames] == [False] * len(codes)
    assert line.codes == codes
    assert B.decode(line.codes) == text.upper()
    assert B.check_timing(edges.log, UNIT, tol=0, back_to_back=True) == []
    assert uart.cts_high == 0                   # never near full


@model_only("the translation table's drop rules: no new timing path beyond "
            "test_baudot_bridge_mixed, run on the model to save RTL time")
def test_baudot_bridge_drops(backend):
    """Bytes with no ITA2 code are dropped, not replaced (header): ! @ { ~
    DEL, bytes over 0x7F, ENQ (it would be WRU, S.1 4.1) and NUL (all-space,
    S.1 4.7.2); BEL is the audible signal, figure J (S.1 4.3). A UART frame
    with a low stop bit is dropped too, and the shift state is unchanged by
    every dropped byte."""
    data = b"a!b@c{~\x7f\x80\xff\x05\x00 \x07x"
    want, _ = B.encode("abc \x07x")
    bench, loom, uart, line, edges = bridge_setup(backend)
    uart.queue(data[:6])
    bench.run_until(lambda: not uart.pending and not uart.busy, 40 * UART_BIT * 10)
    uart.send_frame(ord("Q"), stop=0)          # framing error: dropped
    uart.queue(data[6:])
    bridge_run(bench, uart, line, len(want), (len(want) + 6) * CHAR)
    assert line.codes == want
    assert B.decode(line.codes) == "ABC \x07X"
    assert B.check_timing(edges.log, UNIT, tol=0, back_to_back=False) == []


@model_only("0.6 to 1.1 M clocks of teleprinter time, beyond the RTL budget; "
            "the queue and CTS_N logic is not rate-dependent")
@pytest.mark.parametrize("honour", [True, False])
def test_baudot_bridge_flood(backend, honour):
    """Characters sent as fast as the UART allows. A sender that
    honours CTS_N (OUT1, header) loses nothing: CTS_N rises before the
    queue is full and falls again as it drains, and every character comes
    out in order. A sender that ignores it overruns the 63-entry queue: the
    bridge then drops whole characters (never a shift without its
    character), so what comes out is still the reference encoding of a
    subsequence of the text, in order, and it decodes cleanly."""
    # Honoured: 120 characters, the teleprinter 1.5 times slower than the
    # UART. Ignored: 80 characters (about 90 codes) with the teleprinter
    # four times slower (TICK_INT 64), so the backlog passes 63 entries.
    text = ("THE QUICK BROWN FOX 0123456789 JUMPS OVER THE LAZY DOG. " * 3)
    text = text[:120] if honour else text[:80]
    tick = TICK if honour else 4 * TICK
    unit, char = tick * TPB, int(7.5 * tick * TPB)
    want, _ = B.encode(text)
    bench, loom, uart, line, edges = bridge_setup(backend, honour=honour, tick=tick)
    uart.queue(text.encode())
    quiet = lambda: (not uart.pending and not uart.busy and edges.log
                     and bench.cycle - edges.log[-1][0] > 8 * unit)   # > a character
    bench.run_until(quiet, len(text) * 12 * UART_BIT + (len(want) + 8) * char, every=256)
    got = line.codes
    assert B.check_timing(edges.log, unit, tol=0, back_to_back=False) == []
    if honour:
        assert got == want
        assert uart.cts_high > 0                # the flow control did act
        return
    out = B.decode(got)
    assert len(out) < len(text)
    it = iter(text)
    assert all(ch in it for ch in out), "not a subsequence of the text"
    assert 63 <= len(got) < len(want)           # the queue was full, then drained
    # Whole characters: every letter or figure code is printed in its case,
    # so decoding gives back exactly the characters of a subsequence.
    assert all(c in B.LETTER or c in B.FIGURE or c in B.EITHER or c in (B.LTRS, B.FIGS)
               for c in got)
