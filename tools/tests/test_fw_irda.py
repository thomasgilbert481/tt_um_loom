"""L3-IRDA on the model and the RTL: firmware/irda_sir_tx.loom and irda_sir_rx.loom.

The transmitter (thread 0, TX = OUT0, high = LED on) sends the bytes the
host feeds it as SIR pulses; ``tools.protomodels.irda.decode`` turns the
recorded pulses back into bytes and ``violations`` checks every pulse's
width and place against IrPHY 1.4's tables (through Vishay note 82513).
The receiver (thread 0, RX = IN0, low = a pulse, as a transceiver's RXD)
takes pulse streams from ``SirSource`` at the rate and width extremes the
tables allow and past them. Together, the transmitter on thread 0 drives
the receiver on thread 1 through an inverting bench link (the optics).

Expected values come from IrPHY through the reference model (rates,
widths, tolerances, the character format) and from the program headers
(the command and result formats, the pulse's place in its cell, where the
headers fix it), never from what the model or the RTL did.

Clock: 50 MHz. 115.2 kbit/s is 434 clocks a bit, a character 4340: the
RTL runs those. A 9.6 kbit/s character is 52 k clocks, so the 9.6 kbit/s
scenarios are ``model_only``; they take the code paths the faster ones do,
with longer loops (N = 12).

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import irda
from tools.protomodels.bench import Model, pad_of
from tools.tests.fw_backend import model_only

ISA = load()
TX_SRC = REPO / "firmware" / "irda_sir_tx.loom"
RX_SRC = REPO / "firmware" / "irda_sir_rx.loom"
IMEM = 512
CLOCK = 50e6
SLOT = 4                            # the slot grid (SEMANTICS 2)

# Ticks (program headers): 1/16 of a bit, (TICK_INT, TICK_FRAC).
T115 = (27, 33)                     # 115.2 kbit/s: 27.129 clocks
T9600 = (325, 133)                  # 9.6 kbit/s: 325.52 clocks


def tick_clocks(tick):
    return tick[0] + tick[1] / 256


CONFIG = 0x8000                     # TX command: configuration (header)
FIXED = 1 << 4                      # TX config: fixed 3-tick pulses
FERR, OVERRUN = 1 << 8, 1 << 9      # RX result flags (header)


def sir_bench(backend, features=("FIFO",)):
    """The FIFOs (and for the bridge LD/ST) and the 512-word memory on the
    model; the RTL has all of them and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=features, imem_words=IMEM)
    return backend.bench()


def program(path, thread=0, pins=None):
    """A program for the 512-word memory, moved to ``thread`` and to other
    pins by changing only its ``.thread`` and ``.pins`` lines."""
    text = path.read_text(encoding="utf-8")
    if thread:
        assert text.count(".thread 0") == 1
        text = text.replace(".thread 0", ".thread %d" % thread)
    if pins:
        old, new = pins
        assert text.count(old) == 1, old
        text = text.replace(old, new)
    prog = assemble_text(text, path.name, isa=ISA, imem_words=IMEM, strict=True)
    assert prog.diagnostics == []
    return prog


def boot(backend, bench, programs):
    """Load ``[(program, thread, tick)]`` as one image, set each thread's
    tick, run them all."""
    words = {}
    for prog, _, _ in programs:
        assert not set(words) & set(prog.words), "the programs overlap"
        words.update(prog.words)
    loom = backend.loom(bench, isa=ISA)
    loom.load(words)
    for _, thread, (ti, tf) in programs:
        loom.write_csr(thread, "TICK_INT", ti)
        loom.write_csr(thread, "TICK_FRAC", tf)
    loom.run([thread for _, thread, _ in programs])
    return loom


class Optics(Model):
    """The infrared link between two transceivers: the receiver's RXD is
    low while the transmitter's TXD is high, one clock late (a registered
    model, like every bench model)."""

    def __init__(self, src, dst):
        self.src, self.dst, self.level = pad_of(src), pad_of(dst), 0

    def drive(self, drive, cycle):
        drive.set(self.dst, 0 if self.level else 1)

    def observe(self, lines):
        self.level = lines.get(self.src)


# ------------------------------------------------------------- transmitter
def transmitter(backend, tick):
    bench = sir_bench(backend)
    rec = bench.add(irda.SirRecorder("OUT0"))
    loom = boot(backend, bench, [(program(TX_SRC), 0, tick)])
    bench.step(64)
    return bench, loom, rec


def check_tx(rec, data, rate, n, pulse_ticks, tick):
    """The pulses decode to ``data``, IrPHY allows every one of them, and
    they sit where the header puts them: cells of 16 * N ticks, each pulse
    ``pulse_ticks`` long, every edge 0 to 3 clocks after its tick (so
    positions and widths are exact to a slot)."""
    bit = 16 * n * tick_clocks(tick)
    assert not rec.high_at_end
    frames = irda.decode(rec.pulses, bit)
    assert [f.byte for f in frames] == list(data)
    assert not any(f.ferr for f in frames)
    assert irda.violations(rec.pulses, rate, CLOCK) == []
    width = pulse_ticks * tick_clocks(tick)
    for f in frames:
        for start, w in f.pulses:
            assert abs(w - width) <= SLOT, (start, w, width)
            k = round((start - f.start) / bit)
            assert abs(start - f.start - k * bit) <= SLOT, (start, k)
    return frames


def tx_3_16(backend, rate, n, tick):
    bench, loom, rec = transmitter(backend, tick)
    data = [0x55, 0x00, 0xA3]
    loom.push(0, ([CONFIG | n] if n != 1 else []) + data)
    bench.step(int((len(data) + 1) * 10 * 16 * n * tick_clocks(tick)))
    check_tx(rec, data, rate, n, 3 * n, tick)


def test_tx_3_16_pulses_at_115200(backend):
    """3/16 pulses at 115.2 kbit/s (a 1/16-bit tick, N = 1): every byte
    decoded, every pulse 3/16 of a bit (1.63 us) and in place."""
    tx_3_16(backend, 115200, 1, T115)


@model_only("9.6 kbit/s: 52 k clocks a character")
@pytest.mark.parametrize("n, tick", [(1, T9600), (12, T115)])
def test_tx_3_16_pulses_at_9600(backend, n, tick):
    """The same at 9.6 kbit/s, both ways the header gives: a 1/16-bit tick,
    and N = 12 on the 115.2 kbit/s tick (19.53 us pulses)."""
    tx_3_16(backend, 9600, n, tick)


def tx_fixed(backend, rate, n):
    bench, loom, rec = transmitter(backend, T115)
    data = [0x0F, 0xC4]
    loom.push(0, [CONFIG | FIXED | n] + data)
    bench.step(int((len(data) + 1) * 10 * 16 * n * tick_clocks(T115)))
    check_tx(rec, data, rate, n, 3, T115)
    for _, w in rec.pulses:
        assert abs(w / CLOCK * 1e6 - irda.PULSE_115K_US) < 0.1


def test_tx_fixed_1_63_us_pulses_at_57600(backend):
    """F = 1, N = 2 on the 115.2 kbit/s tick: every pulse 3 ticks, 1.63 us,
    at 57.6 kbit/s, which IrPHY allows at every rate (table 2's minimum
    1.41 us)."""
    tx_fixed(backend, 57600, 2)


@model_only("9.6 kbit/s: 52 k clocks a character")
def test_tx_fixed_1_63_us_pulses_at_9600(backend):
    """F = 1, N = 12: 1.63 us pulses in 104 us cells."""
    tx_fixed(backend, 9600, 12)


def test_tx_back_to_back_and_after_idle(backend):
    """Queued bytes go out back to back, 10 bits from start pulse to start
    pulse; one pushed after the line was idle starts a whole cell after
    the POP that took it, and still decodes."""
    bench, loom, rec = transmitter(backend, T115)
    bit = 16 * tick_clocks(T115)
    loom.push(0, [0x00, 0xFF, 0x00])
    bench.step(int(4 * 10 * bit))
    loom.push(0, [0x81])
    bench.step(int(2 * 10 * bit))
    frames = check_tx(rec, [0x00, 0xFF, 0x00, 0x81], 115200, 1, 3, T115)
    starts = [f.start for f in frames]
    for a, b in zip(starts[:2], starts[1:3]):
        assert abs(b - a - 10 * bit) <= SLOT, (a, b)
    assert starts[3] - starts[2] > 11 * bit


# ---------------------------------------------------------------- receiver
def receiver(backend, tick, thread=0):
    bench = sir_bench(backend)
    src = bench.add(irda.SirSource("IN0"))
    loom = boot(backend, bench, [(program(RX_SRC), 0, tick)])
    bench.step(64)
    return bench, loom, src


def stream(data, rate, *, width_us=None, offset=7 / 16, factor=1.0,
           gap_bits=0.0):
    """``data`` as a sender at ``rate`` * (1 / ``factor``) would send it:
    pulses ``width_us`` long (3/16 of a nominal bit if None) starting
    ``offset`` of a bit into each cell."""
    bit = CLOCK / rate
    width = (irda.nominal_us(rate) if width_us is None else width_us) * CLOCK / 1e6
    pulses = irda.encode(data, bit, width, offset=offset, gap_bits=gap_bits)
    return irda.stretch(pulses, factor, widths=False), bit * factor


def receive(bench, loom, src, pulses, bit, count):
    src.send(pulses, at=bench.cycle + 100)
    bench.step(int(pulses[-1][0] + 2 * bit) + 100)
    return loom.pop(0, count)


DATA = [0x00, 0xFF, 0x55, 0xA5]


@pytest.mark.parametrize("factor, width_us, offset", [
    (1 / (1 + irda.RATE_TOL), irda.PULSE_MIN_US, 0.0),
    (1 / (1 - irda.RATE_TOL), 2.23, 13 / 16),
    (1 / (1 + irda.RATE_TOL), 2.23, 13 / 16),
    (1 / (1 - irda.RATE_TOL), irda.PULSE_MIN_US, 0.0),
])
def test_rx_at_the_115200_limits(backend, factor, width_us, offset):
    """Table 2's limits at 115.2 kbit/s: the sender 0.87 % fast or slow,
    pulses 1.41 us (the minimum) or 2.23 us (the maximum), at the start of
    their cells or ending at the end, back to back."""
    bench, loom, src = receiver(backend, T115)
    pulses, bit = stream(DATA, 115200, width_us=width_us, offset=offset,
                         factor=factor)
    assert [f.byte for f in irda.decode(pulses, bit)] == DATA
    assert receive(bench, loom, src, pulses, bit, 4) == DATA


@model_only("the margin past the spec: a sweep of slow cases")
@pytest.mark.parametrize("factor", [0.96, 1.04])
@pytest.mark.parametrize("offset", [0.0, 13 / 16])
def test_rx_beyond_the_rate_tolerance(backend, factor, offset):
    """+-4 %, more than four times table 2's tolerance: still received."""
    bench, loom, src = receiver(backend, T115)
    pulses, bit = stream(DATA, 115200, offset=offset, factor=factor)
    assert receive(bench, loom, src, pulses, bit, 4) == DATA


@model_only("9.6 kbit/s: 52 k clocks a character")
@pytest.mark.parametrize("width_us", [irda.PULSE_MIN_US, 19.53, 22.13])
def test_rx_at_9600(backend, width_us):
    """9.6 kbit/s with the 1.41 us minimum, the nominal 19.53 us and the
    22.13 us maximum pulse, the sender 0.87 % slow."""
    bench, loom, src = receiver(backend, T9600)
    pulses, bit = stream(DATA, 9600, width_us=width_us,
                         factor=1 + irda.RATE_TOL)
    assert receive(bench, loom, src, pulses, bit, 4) == DATA


def test_rx_framing_error_missing_and_misplaced_pulses(backend):
    """Faults from the model's error injection, two idle bits between
    characters: a stray pulse in the stop cell (bit 8, the byte still
    given), a missing pulse (a 0 read as 1), a pulse 0.6 bit late (read in
    the next cell), and a 3-clock glitch on the idle line (ignored). Each
    expected word is what the spec's decoder makes of the faulted stream."""
    bench, loom, src = receiver(backend, T115)
    data = [0x3C, 0x00, 0xFE, 0x81]
    pulses, bit = stream(data, 115200, gap_bits=2)
    width = pulses[0][1]
    # character 0: a pulse in its stop cell
    pulses = irda.add_pulse(pulses, 9 * bit + 7 / 16 * bit, width)
    # character 1 (0x00): drop data bit 3's pulse
    first1 = next(j for j, p in enumerate(pulses) if p[0] >= 12 * bit)
    pulses = irda.drop_pulse(pulses, first1 + 4)
    # character 2 (0xFE): bit 0's pulse 0.6 bit late, into bit 1's cell
    first2 = next(j for j, p in enumerate(pulses) if p[0] >= 24 * bit)
    pulses = irda.move_pulse(pulses, first2 + 1, 0.6 * bit)
    # a glitch on the idle line, after character 3
    pulses = irda.add_pulse(pulses, 47 * bit, 3)
    model = irda.decode(pulses[:-1], bit)
    expected = [f.byte | (FERR if f.ferr else 0) for f in model]
    assert expected == [0x3C | FERR, 0x08, 0xFD, 0x81]
    assert receive(bench, loom, src, pulses, bit, 4) == expected
    bench.step(int(3 * bit))
    assert loom.pop_available(0) == []


def test_rx_overrun(backend):
    """Six characters with nobody reading: OUTQ holds the first four, the
    next two are dropped, and the first character after the host makes
    room carries bit 9."""
    bench, loom, src = receiver(backend, T115)
    data = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66]
    pulses, bit = stream(data, 115200)
    src.send(pulses, at=bench.cycle + 100)
    bench.step(int(pulses[-1][0] + 2 * bit) + 100)
    assert loom.pop(0, 4) == data[:4]
    pulses, bit = stream([0x77], 115200)
    assert receive(bench, loom, src, pulses, bit, 1) == [0x77 | OVERRUN]


# ---------------------------------------------------------------- loopback
@pytest.mark.parametrize("config, n", [(0, 1), (CONFIG | FIXED | 2, 2)])
def test_tx_looped_to_rx_on_two_threads(backend, config, n):
    """The transmitter on thread 0 drives the receiver on thread 1 through
    the optics (OUT0 high = light = IN1 low): 115.2 kbit/s with 3/16
    pulses, and 57.6 kbit/s with 1.63 us pulses, the receiver's tick 1/16
    of its bit."""
    bench = sir_bench(backend)
    bench.add(Optics("OUT0", "IN1"))
    rx_tick = T115 if n == 1 else (54, 66)          # 54.258 clocks
    loom = boot(backend, bench, [
        (program(TX_SRC), 0, T115),
        (program(RX_SRC, thread=1, pins=("RX = IN0", "RX = IN1")), 1, rx_tick)])
    data = [0xA5, 0x00, 0xFF, 0x3C]
    loom.push(0, ([config] if config else []) + data)
    bench.step(int(5 * 10 * 16 * n * tick_clocks(T115)))
    assert loom.pop(1, 4) == data
    assert loom.pop_available(1) == []


# ------------------------------------------------------------------- build
def test_tx_assembles_strict_with_every_pair_proved():
    """L2-DEADLINE: every pair proved at the declared 16-clock tick,
    nothing unbounded; the one declaration is the guarded POP."""
    prog = program(TX_SRC)
    report = prog.deadlines[0]
    assert report.period == 16
    assert not report.infeasible and not report.unbounded
    assert len(report.pairs) == 28
    assert report.worst_slack == 24
    assert [d.name for d in report.declarations] == ["POP"]
    assert list(prog.threads) == [0] and len(prog.words) == 64
    assert max(prog.words) < 128                        # thread 0's quarter


def test_rx_assembles_strict_with_no_deadline_pairs():
    """The receiver keeps time with timeouts only (WAITE ... T against a TD
    it writes), so it has no deadline pair; the guarded PUSH is declared."""
    prog = program(RX_SRC)
    report = prog.deadlines[0]
    assert not report.pairs and not report.unbounded
    assert [d.name for d in report.declarations] == ["PUSH"]
    assert list(prog.threads) == [0] and len(prog.words) == 38
    moved = program(RX_SRC, thread=1, pins=("RX = IN0", "RX = IN1"))
    assert min(moved.words) == 128 and max(moved.words) < 256


# ------------------------------------------------------------------ bridge
BRIDGE_SRC = REPO / "firmware" / "bridge_uart_irda.loom"
UART_TICK = (54, 64)                # 1/8 of a 115200-baud bit (uart_rx.loom)
RINGFULL = 1 << 10                  # bridge error word: the ring was full


def bridge(backend, sir_tick=T115, config=None):
    from tools.protomodels.uart import UartTx
    bench = sir_bench(backend, features=("FIFO", "DMEM"))
    uart = bench.add(UartTx("IN0", CLOCK / 115200))
    rec = bench.add(irda.SirRecorder("OUT0"))
    prog = assemble_text(BRIDGE_SRC.read_text(encoding="utf-8"),
                         BRIDGE_SRC.name, isa=ISA, imem_words=IMEM, strict=True)
    assert prog.diagnostics == []
    loom = backend.loom(bench, isa=ISA)
    loom.load(prog.words)
    for thread, (ti, tf) in ((0, UART_TICK), (1, sir_tick)):
        loom.write_csr(thread, "TICK_INT", ti)
        loom.write_csr(thread, "TICK_FRAC", tf)
    if config is not None:
        loom.push(1, [config])
    loom.run([0, 1])
    bench.step(64)
    return bench, loom, uart, rec


def test_bridge_uart_to_sir_at_115200(backend):
    """Eight bytes back to back on the UART at 115200 baud come out as SIR
    at 115.2 kbit/s with 3/16 pulses, every pulse within IrPHY's tables,
    with nothing from the host (more bytes than INQ could hold)."""
    bench, loom, uart, rec = bridge(backend)
    data = [0x00, 0xFF, 0x55, 0xAA, 0x0F, 0xF0, 0x81, 0x7E]
    uart.send(data)
    bench.step(int((len(data) + 2) * 10 * CLOCK / 115200))
    check_tx(rec, data, 115200, 1, 3, T115)
    assert loom.pop_available(0) == []


def test_bridge_framing_error_is_reported_not_sent(backend):
    """A byte with a low stop bit is not forwarded: OUTQ[0] gets it with
    bit 8, and the bytes around it go out."""
    bench, loom, uart, rec = bridge(backend)
    uart.send([0x12])
    uart.send_frame(0x5A, stop=0)
    uart.send([0x34])
    bench.step(int(5 * 10 * CLOCK / 115200))
    check_tx(rec, [0x12, 0x34], 115200, 1, 3, T115)
    assert loom.pop(0, 1) == [0x5A | FERR]


@model_only("a 40-byte stream into a half-rate SIR side: 520 k clocks")
def test_bridge_ring_full_at_half_rate(backend):
    """With the SIR side at 57.6 kbit/s (N = 2, F = 1) behind a 115200-baud
    UART, the ring fills (15 bytes): the first bytes all go out, in order;
    from then on a byte is taken only when sending one has freed a word,
    and every byte not sent is reported with bit 10, in order, up to
    OUTQ's depth (4)."""
    bench, loom, uart, rec = bridge(backend, config=CONFIG | FIXED | 2)
    data = list(range(0x40, 0x40 + 40))
    uart.send(data)
    bench.step(int(len(data) * 30 * CLOCK / 115200))
    sent = [f.byte for f in irda.decode(rec.pulses, 32 * tick_clocks(T115))]
    check_tx(rec, sent, 57600, 2, 3, T115)
    words = loom.pop_available(0)
    assert all(w & ~0xFF == RINGFULL for w in words)
    lost = [w & 0xFF for w in words]
    assert sent[:17] == data[:17] and sent == sorted(sent)
    missing = [b for b in data if b not in sent]
    assert len(missing) >= 4 and lost == missing[:4]


def test_bridge_assembles_strict_and_fits_two_quarters():
    """L2-DEADLINE for both threads, nothing unbounded; thread 0 (with the
    ring) stays in 0x000 .. 0x07F and thread 1 in 0x080 .. 0x0FF, so
    threads 2 and 3 are free."""
    prog = assemble_text(BRIDGE_SRC.read_text(encoding="utf-8"),
                         BRIDGE_SRC.name, isa=ISA, imem_words=IMEM, strict=True)
    assert prog.diagnostics == []
    assert sorted(prog.threads) == [0, 1]
    for thread, pairs, slack, bounded in ((0, 4, 48, ["PUSH"]),
                                          (1, 35, 20, ["POP"])):
        report = prog.deadlines[thread]
        assert report.period == 16
        assert not report.infeasible and not report.unbounded
        assert len(report.pairs) == pairs and report.worst_slack == slack
        assert [d.name for d in report.declarations] == bounded
    low = [a for a in prog.words if a < 0x80]
    high = [a for a in prog.words if a >= 0x80]
    assert max(low) < 0x80 and min(high) == 0x80 and max(high) < 0x100
    assert len(prog.words) == 129
