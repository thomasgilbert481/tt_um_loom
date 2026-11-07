"""L3 for the Centronics (IEEE 1284 compatibility mode) programs, model and RTL.

``firmware/centronics_host.loom`` is the host side of a parallel printer
port (D0..D7 = BIDIR0..7, nSTROBE, nINIT, nSELECTIN, nAUTOFD = OUT0..3,
BUSY, nACK, PE, SELECT, nERROR = IN0..4); ``firmware/centronics_printer.loom``
is the printer side (D0..D7 = BIDIR0..7 as inputs, the four host controls
on IN0..3, the five status lines on OUT0..4). Each side alone takes 17 of
Loom's 19 pads, so the two never run on one chip: the host program runs
against ``tools.protomodels.centronics.ModelPrinter`` and the printer
program against ``ModelHost``. ``CentronicsMonitor`` measures the
handshake on the pads and checks it against the windows of that module's
docstring (all secondary sources: Tektronix wiki, Lexmark C760 table 8-1);
expected timings come from those figures and the program headers, never
from what the model or the RTL did.

Rates (50 MHz core clock): the host at TICK_INT 32 (0.64 us a phase) and
50 (1 us, the Solaris driver's reading of the strobe); the printer at 25
(0.5 us) with the model host at the 0.5 us minimum of every window.

The PHY-GL-SDF candidate is ``test_centronics_host_to_fast_printer``: eight
bytes at 0.64 us a phase to a printer that answers in 0.5 us, every A, B,
C and G window measured.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import centronics as C
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
HOST, PRINTER = FIRMWARE / "centronics_host.loom", FIRMWARE / "centronics_printer.loom"
CLOCK_HZ = 50e6
DATA = ["BIDIR%d" % i for i in range(8)]
# host program: controls out, status in; printer program: the other way
HOST_PINS = dict(strobe="OUT0", init="OUT1", selin="OUT2", autofd="OUT3",
                 busy="IN0", ack="IN1", pe="IN2", select="IN3", error="IN4")
PRN_PINS = dict(strobe="IN0", init="IN1", selin="IN2", autofd="IN3",
                busy="OUT0", ack="OUT1", pe="OUT2", select="OUT3", error="OUT4")
TMO = 0x20                      # host status bit 5 (its header)
BUSY, NACK, PE, SELECT, NERROR = 1, 2, 4, 8, 16     # status bits 4:0
READY = 0x18                    # printer status word: SELECT, nERROR high
PAPER_OUT = 0x05                # BUSY (hold), PE high; SELECT, nERROR low
HOLD = 0x01
INIT_EVENT = 0x8000
STARTUP = 300                   # clocks for a program's set-up after RUN


def mbench(backend):
    if backend.name == "model":
        return backend.bench(features=("FIFO",), imem_words=IMEM)
    return backend.bench()


def monitor(bench, pins, **kw):
    return bench.add(C.CentronicsMonitor(DATA, pins["strobe"], pins["busy"],
                                         pins["ack"], clock_hz=CLOCK_HZ, **kw))


# ------------------------------------------------------------- assembly
def test_centronics_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: every pair of both programs proved (the host at its
    .tick of 24 clocks, the printer at 8), nothing unbounded and no
    ``.bounded``; each fits thread 0's quarter."""
    host = assemble_file(HOST, isa=ISA, strict=True)
    prn = assemble_file(PRINTER, isa=ISA, strict=True)
    for prog, period, words in ((host, 24, 63), (prn, 8, 41)):
        assert prog.diagnostics == [] and list(prog.threads) == [0]
        report = prog.deadlines[0]
        assert report.period == period
        assert report.pairs and not report.infeasible and not report.unbounded
        assert report.worst_slack >= 0
        assert len(prog.words) == words and max(prog.words) < 0x80


# ------------------------------------------- the host, model printer
def host_setup(backend, tick=32, timeout=1000, **printer):
    bench = mbench(backend)
    prn = bench.add(C.ModelPrinter(DATA, **HOST_PINS, **printer))
    mon = monitor(bench, HOST_PINS)
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(HOST, isa=ISA, strict=True))
    loom.write_csr(0, "TICK_INT", tick)
    loom.run(0)
    loom.push(0, [timeout])
    return bench, prn, mon, loom


def send(loom, data):
    """Each byte, one status word back."""
    out = []
    for v in data:
        loom.push(0, [v])
        out += loom.pop(0, 1)
    return out


def phases_within(mon, tick, slack=3):
    """A, B, C (hold: until the data next change) each one tick, to the
    3-clock slot dither (the host header), and G at least a tick. Set-up
    is from the last data change, so a repeated byte has a longer one."""
    prev = None
    for r in mon.records:
        assert r["setup"] >= tick - slack, r
        if prev is not None and r["data"] != prev:
            assert r["setup"] <= tick + slack, r
        prev = r["data"]
        assert tick - slack <= r["width"] <= tick + slack, r
        assert r["hold"] is None or r["hold"] >= tick - slack, r
        assert r["gap"] >= tick - slack, r


def test_centronics_host_to_fast_printer(backend):
    """Eight bytes at TICK 32 (0.64 us a phase) to a printer that raises BUSY
    in 0.1 us and pulses nACK for 0.5 us right after the strobe (the
    minimum of time E). Every byte arrives in order, with nAUTOFD high and
    nSELECTIN low; A, B and C are one tick each, G at least one tick, all
    >= 0.5 us (25 clocks); every status word has no timeout and SELECT and
    nERROR high."""
    data = b"Loom\r\n\x00\xff"
    bench, prn, mon, loom = host_setup(backend, busy_delay=5, process=5,
                                       ack_width=25)
    status = send(loom, data)
    assert [r[0] for r in prn.received] == list(data)
    assert all(r[1] == 1 and r[2] == 0 for r in prn.received)
    assert mon.bytes == list(data) and mon.violations == []
    phases_within(mon, 32)
    assert all(s & TMO == 0 and s & (SELECT | NERROR) == SELECT | NERROR
               for s in status)
    assert prn.errors == [] and mon.ack_widths == [25] * len(data)
    assert bench.contention_free() and loom.badop() == 0


def test_centronics_host_to_slow_printer(backend):
    """TICK 50 (1 us a phase) and a printer that takes 40 us a byte (BUSY
    high 0.4 us after the strobe, nACK 5 us long, BUSY falling 1 us before
    nACK rises, an order table 8-1 allows). The host waits out BUSY for
    every byte: no strobe while BUSY is high or nACK low, and each strobe
    at least a tick after BUSY low and nACK high (G)."""
    data = [0x41, 0x42, 0x43]
    bench, prn, mon, loom = host_setup(backend, tick=50, busy_delay=20,
                                       process=2000, ack_width=250,
                                       busy_offset=-50)
    status = send(loom, data)
    assert [r[0] for r in prn.received] == data
    assert mon.violations == [] and prn.errors == []
    phases_within(mon, 50)
    for prev, nxt in zip(mon.records, mon.records[1:]):
        assert nxt["fall"] > prev["ack_fall"] + 250
        assert nxt["fall"] - prev["busy_fall"] >= 50 - 3
    assert all(s & TMO == 0 for s in status)
    assert bench.contention_free() and loom.badop() == 0


def test_centronics_host_busy_timeout(backend):
    """A printer that holds BUSY high (buffer full): with a timeout of 20
    ticks the byte is not sent, the status has bit 5 and BUSY, and the
    host gave up no earlier than 20 ticks (640 clocks) after it started to
    wait. Once the hold is off the same byte goes through."""
    bench, prn, mon, loom = host_setup(backend, timeout=20)
    prn.set_status(hold=True)
    bench.step(100)
    t0 = bench.cycle
    loom.push(0, [0x55])
    s = loom.pop(0, 1)[0]
    assert s & TMO and s & BUSY and s & (SELECT | NERROR) == SELECT | NERROR
    assert bench.cycle - t0 >= 20 * 32
    assert prn.received == [] and mon.records == []
    prn.set_status(hold=False)
    assert send(loom, [0x55])[0] & TMO == 0
    assert [r[0] for r in prn.received] == [0x55] and mon.violations == []


def test_centronics_host_paper_out(backend):
    """Paper out (BUSY and PE high, nERROR and SELECT low as the model
    printer has it): the byte times out and the status says why (PE set,
    nERROR clear). Paper loaded again, the byte prints."""
    bench, prn, mon, loom = host_setup(backend, timeout=30)
    prn.set_status(paper_out=True)
    bench.step(100)
    s = send(loom, [0x0C])[0]
    assert s & TMO and s & PE and not s & NERROR and s & BUSY
    assert prn.received == []
    prn.set_status(paper_out=False)
    s = send(loom, [0x0C])[0]
    assert not s & TMO and not s & PE and s & NERROR
    assert [r[0] for r in prn.received] == [0x0C] and mon.violations == []


def test_centronics_host_init_and_control_lines(backend):
    """A control word with bit 14: nINIT low for 60 ticks (60 us at TICK 50;
    the >= 50 us width is from memory, unchecked); the printer holds BUSY
    while nINIT is low and the reply comes after it. Then nAUTOFD low and
    nSELECTIN high (bits 13:12 = nAUTOFD, nSELECTIN = 0b01), a byte, and
    back (0b10, the start-up levels)."""
    bench, prn, mon, loom = host_setup(backend, tick=50)
    loom.push(0, [0x8000 | 0x4000 | 0x2000 | 60])      # levels unchanged
    loom.pop(0, 1)
    assert len(prn.inits) == 1 and 60 * 50 <= prn.inits[0] <= 61 * 50 + 16
    loom.push(0, [0x8000 | 0x1000])                     # nAUTOFD 0, nSELECTIN 1
    assert loom.pop(0, 1)[0] & TMO == 0
    send(loom, [0x31])
    loom.push(0, [0x8000 | 0x2000])                     # nAUTOFD 1, nSELECTIN 0
    loom.pop(0, 1)
    send(loom, [0x32])
    assert [(r[0], r[1], r[2]) for r in prn.received] == [(0x31, 0, 1),
                                                          (0x32, 1, 0)]
    assert mon.violations == []


# ------------------------------------------- the printer, model host
PTICK = 25                      # 0.5 us at 50 MHz
CTL = 0x0A00                    # nAUTOFD 1, nSELECTIN 0, nINIT 1, nSTROBE 0
BUSY_MAX = 40                   # the printer header: BUSY <= 0.8 us


def prn_setup(backend, ackw=2, status=READY, **host):
    bench = mbench(backend)
    for k, v in dict(setup=25, width=25, hold=25, gap=25).items():
        host.setdefault(k, v)                   # every window at 0.5 us
    h = bench.add(C.ModelHost(DATA, **PRN_PINS, **host))
    mon = monitor(bench, PRN_PINS)
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(PRINTER, isa=ISA, strict=True))
    loom.write_csr(0, "TICK_INT", PTICK)
    loom.run(0)
    loom.push(0, [ackw] + ([status] if status != READY else []))
    bench.step(STARTUP + 16 * PTICK)        # a poll period: the status word
    return bench, h, mon, loom


def run_host(bench, h, clocks):
    assert bench.run_until(lambda: not h.busy, clocks, every=16)


@pytest.mark.parametrize("wait_ack", [False, True])
def test_centronics_printer_takes_bytes(backend, wait_ack):
    """A host at the 0.5 us minimum of A, B, C and G, with and without a
    wait for nACK: every byte reaches the FIFO with the control lines
    (nAUTOFD 1, nSELECTIN 0, nINIT 1, nSTROBE 0); BUSY rises within 0.8 us
    of the strobe (the program's bound; table 8-1's D is 0.5 us, see its
    header); nACK is 2 to 3 ticks and 16 clocks (1 to 1.82 us, E >= 0.5 us)
    and BUSY falls
    as nACK rises."""
    data = [0x48, 0x69, 0x0D, 0x0A]
    bench, h, mon, loom = prn_setup(backend, wait_ack=wait_ack)
    h.send(data)
    run_host(bench, h, 20000)
    bench.step(200)
    assert loom.pop(0, 4) == [CTL | v for v in data]
    assert h.sent and [v for v, _ in h.sent] == data and h.timeouts == []
    assert mon.violations == [] and mon.bytes == data
    for r in mon.records:
        assert r["busy"] is not None and r["busy"] <= BUSY_MAX, r
        assert 2 * PTICK <= r["ack_width"] <= 3 * PTICK + 16, r
        assert r["busy_fall"] == r["ack_fall"] + r["ack_width"], r
    assert bench.contention_free() and loom.badop() == 0


def test_centronics_printer_paper_out(backend):
    """Status word 0x05 (paper out): BUSY and PE high, SELECT and nERROR
    low on the pads; a host with a 2,000-clock timeout gives up on its byte.
    With 0x18 the printer is ready and the byte prints."""
    bench, h, mon, loom = prn_setup(backend, status=PAPER_OUT, timeout=2000)
    assert h.status == [1, 1, 1, 0, 0]
    h.send([0x50])
    run_host(bench, h, 5000)
    assert h.timeouts == [0x50] and mon.records == []
    loom.push(0, [READY])
    bench.step(1000)
    assert h.status == [0, 1, 0, 1, 1]
    h.send([0x50])
    run_host(bench, h, 5000)
    assert loom.pop(0, 1) == [CTL | 0x50] and mon.violations == []


def test_centronics_printer_buffer_full(backend):
    """Two buffer-full conditions. The host's HOLD bit: after a byte
    nACK comes but BUSY stays high, and the next byte waits until the host
    clears it. A full result FIFO: with nothing popped the fifth byte waits
    in the PUSH with BUSY high and no nACK, and goes on once the host pops."""
    bench, h, mon, loom = prn_setup(backend, wait_ack=True)
    loom.push(0, [READY | HOLD])
    bench.step(1000)
    assert h.status[0] == 1
    h.send([0x01])
    bench.step(3000)
    assert h.sent == [] and mon.records == []
    loom.push(0, [READY])
    run_host(bench, h, 5000)
    loom.push(0, [READY | HOLD])
    bench.step(200)
    assert h.status[0] == 1 and loom.pop(0, 1) == [CTL | 0x01]
    loom.push(0, [READY])
    h.send([0x10, 0x11, 0x12, 0x13, 0x14])
    bench.run_until(lambda: len(h.sent) == 5, 20000, every=16)
    bench.step(2000)
    assert h.busy and h.status[0] == 1 and mon.records[-1]["ack_fall"] is None
    assert loom.pop(0, 4) == [CTL | v for v in (0x10, 0x11, 0x12, 0x13)]
    run_host(bench, h, 5000)
    assert loom.pop(0, 1) == [CTL | 0x14] and mon.violations == []


def test_centronics_printer_init(backend):
    """nINIT low for 60 us: BUSY high within a poll period (16 ticks, 8 us)
    and while it is low, one 0x8000 in the FIFO after it rises, and the
    printer takes the next byte."""
    bench, h, mon, loom = prn_setup(backend)
    h.init(3000)
    bench.step(16 * PTICK + 40)
    assert h.status[0] == 1
    run_host(bench, h, 5000)
    bench.step(100)
    assert loom.pop(0, 1) == [INIT_EVENT]
    h.send([0x21])
    run_host(bench, h, 5000)
    assert loom.pop(0, 1) == [CTL | 0x21] and mon.violations == []
