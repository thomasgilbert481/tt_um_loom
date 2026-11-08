"""L3-UNIO on the model and the RTL: firmware/unio_master.loom and
firmware/unio_device.loom, Microchip's UNI/O bus of the 11AA02E48 serial
EEPROM (W-092, D-035).

The master (thread 0, SCIO = BIDIR0) is fed through ``tools.loomhost.Loom``
and runs against ``tools.protomodels.unio.UnioDevice`` on a ``OneWireBus``
line (the wire and its pull-up); the device program (thread 1, SCIO =
BIDIR1) answers ``UnioMaster``. Together, the two pads are one line: the
bench joins them, as two Loom threads never share a pin (BUGS 14: both set
their OD_MASK bit first and read it back).

What is expected comes from Microchip DS20002122B (Table 1-2, sections 3
and 4) as ``protomodels.unio`` cites it, and from the program headers (the
host words, the result codes, each program's chosen point in each window),
never from what the model or the RTL did. The model device holds the chip
master to Table 1-2 (THDR, TSS, TE, TIJIT, TSTBY) and the model master
holds the chip device's SAK and data edges to TOJIT; the tests also measure
the chip's header and bit period on the pad.

RTL time: a byte is 10 bits, 1 ms at 10 kbit/s (50000 clocks), so

* at 100 kbit/s (TE = 10 us, the master's quarter q = 125 clocks) the RTL
  runs a master read, the device's read, the two programs together (WREN,
  a write, its poll and a read-back) and the missing device;
* 10 kbit/s runs on a time-scaled build, as test_fw_hdq does: the models
  are clocked at 10 MHz, so q = 250 chip clocks is a 25 us quarter, TE =
  100 us in the models' time and every interval the same number of ticks
  (the programs count in ticks, not in clocks). The device's idle tick is
  a fixed 1250 clocks, so its standby threshold (16 of them) is 2 ms of the
  models' time there, and the model master's standby is 2.5 ms (TSTBY has
  only a minimum). The RTL runs one master read so;
* the window edges, the faults, the refused write and the start-phase
  sweep run on the model only.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import functools

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.unio import (READ, RDSR, TABLE, WREN, WRITE, OneWireBus,
                                    UnioDevice, UnioMaster, bits_byte, msb_bits)
from tools.tests.fw_backend import model_only

ISA = load()
MASTER = REPO / "firmware" / "unio_master.loom"
DEVICE = REPO / "firmware" / "unio_device.loom"
MHZ = 50                        # chip clocks a microsecond
SLOW = 10                       # the time-scaled build's model clock, MHz
STARTUP = 400                   # clocks for a program's set-up after RUN
# unio_master.loom's header: host words, result codes, times in quarters.
Q100 = 125                      # q at 100 kbit/s
THDR_Q, TSS_Q, STBY_Q = 4, 8, 250
NOSAK, BADBIT = 0x8000, 0xC000
# unio_device.loom's header: the array, the write cycle in ticks.
ARRAY = [0x11 * i for i in range(16)]
TBUSY_Q = 196
SCALED = "a time-scaled build (module doc): %s"


@functools.lru_cache(maxsize=None)
def program(path):
    """The program assembled with ``--strict``."""
    return assemble_text(path.read_text(encoding="utf-8"), path.name, isa=ISA,
                         strict=True)


def w_rate(q):
    return 0x8000 | q


def w_read(addr, n):
    return 0x1000 | (n - 1) << 8 | addr


def w_write(addr, n, wren=True):
    return (0x2000 if wren else 0x3000) | (n - 1) << 8 | addr


W_RDSR = 0x4000


def unio_bench(backend, bus):
    """The chip with the FIFOs, the bit engine and its encoder, the deadline
    latch, the data memory and the 512-word memory; the RTL has all of
    them and ``RtlBench`` takes none of those keywords."""
    kwargs = dict(pullups=bus.pullups)
    if backend.name == "model":
        kwargs.update(features=("FIFO", "BE", "BEENC", "SETPD", "DMEM"),
                      imem_words=512)
    bench = backend.bench(**kwargs)
    bench.add(bus)
    return bench


# ------------------------------------------------------------- the programs
@pytest.mark.parametrize("path,thread,words", [(MASTER, 0, 204), (DEVICE, 1, 238)])
def test_unio_programs_assemble_strict_with_no_diagnostic(path, thread, words):
    """L2-DEADLINE: every pair proved at the fastest rate's quarter bit (125
    clocks, 100 kbit/s at 50 MHz); the device's one PUSH is declared bounded
    behind its WAITB OUTQ_NF test. The master spills into thread 3's
    quarter, the device into thread 2's; the two load together."""
    prog = program(path)
    assert prog.diagnostics == []
    report = prog.deadlines[thread]
    assert report.period == Q100
    assert report.pairs and not report.infeasible and not report.unbounded
    assert report.worst_slack >= 0
    assert list(prog.threads) == [thread] and len(prog.words) == words
    assert not set(program(MASTER).words) & set(program(DEVICE).words)
    m = set(program(MASTER).words)
    assert all(a < 0x80 or 0x180 <= a < 0x1F0 for a in m)
    assert all(0x80 <= a < 0x180 for a in program(DEVICE).words)


# ---------------------------------------------------------------- master
def master_setup(backend, device=None, mhz=MHZ, q=Q100):
    """The master program on thread 0 (BIDIR0), a model device on the line,
    the rate word sent first unless it is the default."""
    bus = OneWireBus(["BIDIR0"], clock=mhz * 1_000_000)
    if device is not None:
        bus.add(device)
    bench = unio_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(MASTER))
    loom.run(0)
    bench.step(STARTUP)
    if q != Q100:
        loom.push(0, [w_rate(q)])
    return bench, bus, loom


def results(loom, n=1):
    return loom.pop(0, n, max_polls=200_000)


def chip_lows(bus, pin):
    return [p for p in bus.intervals(bus.chip_name(pin)) if p[1] is not None]


def check_master_line(bus, device, q, mhz=MHZ):
    """The chip master's timing on the pad: the power-up low (THDR
    quarters), then before every header the line high for at least the
    standby or TSS and the header's low THDR quarters plus the 4 to 10
    clocks of the SHO that ends it (header); the device model measured TE =
    4q on every header to a few clocks and found nothing outside Table
    1-2."""
    lows = chip_lows(bus, "BIDIR0")
    assert lows[0][1] - lows[0][0] == THDR_Q * Q100   # at start, both edges latched
    us = mhz
    for start in device.headers:
        fall, rise = next(p for p in lows if p[0] == start)
        assert THDR_Q * q + 4 <= rise - fall <= THDR_Q * q + 12
        assert rise - fall >= TABLE["THDR"][0] * us
        before = max(p[1] for p in lows if p[1] <= fall)
        assert fall - before >= TABLE["TSS"][0] * us
    assert all(abs(te - 4 * q) <= 4 for te in device.te)
    assert device.violations == []


def test_unio_master_reads_a_model_device_at_100k(backend):
    """READ 4 bytes from 0x10 at 100 kbit/s (q = 125): four results 0x00dd,
    the model device's bytes; it took one READ at 0x10 after one standby;
    the header and TE on the pad as the header says, inside Table 1-2."""
    dev = UnioDevice(clock=MHZ * 1_000_000)
    bench, bus, loom = master_setup(backend, dev)
    loom.push(0, [w_read(0x10, 4)])
    want = list(dev.memory[0x10:0x14])
    assert results(loom, 4) == want
    assert dev.commands == [(READ, 0x10, want)] and dev.standbys == 1
    check_master_line(bus, dev, Q100)


@model_only("the RTL budget: test_unio_together writes and polls on the RTL")
def test_unio_master_writes_a_page_and_polls_at_100k(backend):
    """WRITE 0xA1 0xA2 0xA3 at 0x23 (100 kbit/s) to a device whose write
    cycle is 1 ms: WREN, WRITE, then RDSR with MAK while WIP is 1. The
    result is 0xpp00 (WIP 0, WEL reset after the write, 4.4) with pp, the
    status bytes read busy, between 5 and 8: TSS (8 quarters, 20 us) and
    the header, address and RDSR (30 bits, 300 us) take 320 us of the
    cycle, and each status byte with its acknowledge is 10 bits, 100 us;
    the device wrote the page once; READ gives the bytes back."""
    dev = UnioDevice(clock=MHZ * 1_000_000, busy_us=1000)
    bench, bus, loom = master_setup(backend, dev)
    loom.push(0, [w_write(0x23, 3), 0xA1, 0xA2, 0xA3])
    res = results(loom)[0]
    assert res & 0xFF == 0x00 and 5 <= res >> 8 <= 8, hex(res)
    assert dev.writes == [(0x23, [0xA1, 0xA2, 0xA3])]
    loom.push(0, [w_read(0x23, 3)])
    assert results(loom, 3) == [0xA1, 0xA2, 0xA3]
    kinds = [c[0] for c in dev.commands]
    assert kinds == [WREN, WRITE, RDSR, READ]
    assert dev.commands[2][2][-1] == 0 and len(dev.commands[2][2]) == (res >> 8) + 1
    check_master_line(bus, dev, Q100)


def test_unio_master_reads_at_10k_time_scaled(backend):
    """10 kbit/s (time-scaled build, module doc: q = 250 clocks against
    models at 10 MHz, a 25 us quarter, TE = 100 us): RATE, then READ 2
    bytes from 0xFF, which the device rolls over to 0x00 (4.1). The device
    measured TE = 100 us; the standby before it is 250 quarters, 6.25 ms."""
    dev = UnioDevice(clock=SLOW * 1_000_000)
    bench, bus, loom = master_setup(backend, dev, SLOW, 250)
    loom.push(0, [w_read(0xFF, 2)])
    assert results(loom, 2) == [dev.memory[0xFF], dev.memory[0x00]]
    assert dev.commands == [(READ, 0xFF, [dev.memory[0xFF], dev.memory[0]])]
    check_master_line(bus, dev, 250, SLOW)
    assert dev.te[0] / SLOW == pytest.approx(100, abs=0.5)


def test_unio_master_reports_a_missing_device(backend):
    """No device on the line: READ gives 0x8001 (no SAK after the device
    address, header)."""
    bench, bus, loom = master_setup(backend, None)
    loom.push(0, [w_read(0, 1)])
    assert results(loom) == [NOSAK | 1]


@model_only("the standby after an error: the RTL runs the missing device once")
def test_unio_master_sends_a_standby_after_an_error(backend):
    """No device: READ then RDSR both give 0x8001, and before the second
    header the line is high STBY quarters (3.3: a NoSAK needs a standby)."""
    bench, bus, loom = master_setup(backend, None)
    loom.push(0, [w_read(0, 1), W_RDSR])
    assert results(loom, 2) == [NOSAK | 1, NOSAK | 1]
    lows = chip_lows(bus, "BIDIR0")
    headers = [p for p in lows if THDR_Q * Q100 + 4 <= p[1] - p[0] <= THDR_Q * Q100 + 12]
    assert len(headers) == 2
    before = max(p[1] for p in lows if p[1] <= headers[1][0])
    assert headers[1][0] - before >= STBY_Q * Q100


# ----------------------------------------------------------------- device
def device_setup(backend, mhz=MHZ, **master_kw):
    """The device program on thread 1 (BIDIR1), a model master on the line."""
    bus = OneWireBus(["BIDIR1"], clock=mhz * 1_000_000)
    host = bus.add(UnioMaster(clock=mhz * 1_000_000, **master_kw))
    bench = unio_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(DEVICE))
    loom.run(1)
    bench.step(STARTUP)
    return bench, bus, loom, host


def run_ops(bench, host, ms, mhz=MHZ):
    assert bench.run_until(lambda: host.idle, int(ms * 1000 * mhz), every=500)


def test_unio_device_answers_a_model_master_at_100k(backend):
    """A model master at 100 kbit/s with Table 1-2's minimum THDR (5 us)
    and TSS (10 us): standby, READ 2 bytes from 3. The device measured the
    rate on the header: the bytes are the image's 0x33 0x44, every SAK and
    data edge inside TOJIT. (RDSR, WREN and the write run on the RTL in
    test_unio_together.)"""
    bench, bus, loom, host = device_setup(backend)
    op = host.read(3, 2)
    run_ops(bench, host, 4)
    assert (op.status, op.data) == ("ok", [0x33, 0x44])
    assert host.violations == []


@model_only("a page write, its poll and the read-back: test_unio_together writes on the RTL")
def test_unio_device_takes_a_page_write_after_wren(backend):
    """WREN, WRITE 0x5A 0xA5 0x3C at 0x0E (rolling over in the 16-byte
    page to 0x00, 4.3), RDSR polled: WIP reads 1 at least once (the write
    cycle, 196 ticks = 490 us), then 0 with WEL 0 (4.4); the device reports
    0x5703; READ from 0x0E gives the three bytes back, from 0x01 the image."""
    bench, bus, loom, host = device_setup(backend)
    ops = [host.wren(), host.rdsr(), host.write(0x0E, [0x5A, 0xA5, 0x3C]),
           host.rdsr(poll=True), host.read(0x0E, 3), host.read(0x01, 1)]
    run_ops(bench, host, 10)
    assert all(op.status == "ok" for op in ops), ops
    assert ops[1].data == [0x02]                        # WEL after WREN
    assert ops[3].data[0] & 1 and ops[3].data[-1] == 0x00
    assert ops[4].data == [0x5A, 0xA5, 0x3C] and ops[5].data == [0x11]
    assert loom.pop_available(1) == [0x5703]
    assert host.violations == []


@model_only("the refused write: the RTL runs the accepted one together")
def test_unio_device_refuses_a_write_without_wren(backend):
    """WRITE without WREN (5.0, Table 6-1): every byte gets its SAK, but no
    write cycle starts (RDSR reads 0x00 at once), nothing is reported, and
    READ gives the image's bytes."""
    bench, bus, loom, host = device_setup(backend)
    ops = [host.write(4, [0xDE, 0xAD]), host.rdsr(poll=True), host.read(4, 2)]
    run_ops(bench, host, 6)
    assert [op.status for op in ops] == ["ok"] * 3
    assert ops[1].data == [0x00] and ops[2].data == [0x44, 0x55]
    assert loom.pop_available(1) == []


@model_only(SCALED % "the device at 10 kbit/s")
def test_unio_device_at_10k_time_scaled(backend):
    """A model master at 10 kbit/s (time-scaled build): the device measures
    TE = 100 us on the header and answers READ 3 bytes from 0x0F, rolling
    over to 0x00 (its 16-byte array, header), every edge inside TOJIT."""
    bench, bus, loom, host = device_setup(backend, SLOW, te_us=100, tstby_us=2500)
    op = host.read(0x0F, 3)
    run_ops(bench, host, 60, SLOW)
    assert op.status == "ok" and op.data == [0xFF, 0x00, 0x11]
    assert host.violations == []


@model_only("a master at the edge of TIJIT")
@pytest.mark.parametrize("te_us,jitter", [(10, 0.06), (10, -0.06), (37, 0.05)])
def test_unio_device_follows_a_jittery_master(backend, te_us, jitter):
    """A master whose every other edge is off by the jitter TIJIT allows
    (0.06 UI), and one at 27 kbit/s: READ and WREN/RDSR are answered."""
    bench, bus, loom, host = device_setup(backend, te_us=te_us, jitter=jitter)
    ops = [host.read(9, 2), host.wren(), host.rdsr()]
    run_ops(bench, host, 12)
    assert [op.data for op in ops] == [[0x99, 0xAA], [], [0x02]]
    assert host.violations == []


@model_only("the device's refusals")
def test_unio_device_refusals_and_recovery(backend):
    """Each refusal is a NoSAK, then Idle until a standby (3.3, 3.7): another
    device address (0xA2: NoSAK on byte 1); WREN followed by a MAK (4.4:
    NoSAK on the RDSR the model sends after it, byte 3); an instruction the
    device does not have (WRSR, byte 2); READ during the write cycle (byte
    2; the RDSR after it, behind a standby, finds the cycle over). After
    each, a READ with a standby first is answered."""
    bench, bus, loom, host = device_setup(backend)
    ops = [host.command([READ, 0, 0], 1, address=0xA2), host.read(1, 1),
           host.command([WREN, RDSR], 1), host.read(2, 1),
           host.command([0x6E, 0x00]), host.read(3, 1),
           host.wren(), host.write(7, [0x01]), host.read(7, 1), host.rdsr(),
           host.read(8, 1)]
    run_ops(bench, host, 20)
    got = [(op.status, op.failed_at, op.data) for op in ops]
    assert got[0] == ("nosak", 1, []) and got[1] == ("ok", None, [0x11])
    assert got[2] == ("nosak", 2, []) and got[3] == ("ok", None, [0x22])
    assert got[4] == ("nosak", 2, []) and got[5] == ("ok", None, [0x33])
    assert got[8] == ("nosak", 2, [])
    assert got[9] == ("ok", None, [0x00])
    assert got[10] == ("ok", None, [0x88])
    assert host.violations == []


# ----------------------------------------------------------- the two together
def together(backend, start):
    """Master (thread 0, BIDIR0) and device (thread 1, BIDIR1), the pads
    joined by the bench as one line, started by ``start(bench, loom)``."""
    bus = OneWireBus(["BIDIR0", "BIDIR1"], clock=MHZ * 1_000_000)
    bench = unio_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(MASTER).words)
    words.update(program(DEVICE).words)
    loom.load(words)
    start(bench, loom)
    bench.step(STARTUP)
    return bench, bus, loom


def test_unio_together(backend):
    """Both programs under one RUN at 100 kbit/s: WRITE 0xC4 at 0x06 (WREN,
    WRITE, poll), then READ 2 from 0x05. The write's result is
    0xpp00 with pp 1 or 2: the device's write cycle is 490 us (196 ticks)
    and the poll's status bytes are sampled from about 400 us after the
    write's end (TSS, 30 bits of header, address and RDSR, then 8 bits),
    100 us apart; the read gives 0x55 (the image) and 0xC4; the device
    reports 0x5701; nobody drove the line both ways."""
    bench, bus, loom = together(backend, lambda bench, loom: loom.run([0, 1]))
    loom.push(0, [w_write(0x06, 1), 0xC4, w_read(0x05, 2)])
    res = results(loom, 3)
    assert res[0] & 0xFF == 0 and 1 <= res[0] >> 8 <= 2, hex(res[0])
    assert res[1:] == [0x55, 0xC4]
    assert loom.pop_available(1) == [0x5701]
    assert not bench.contentions


def _phase(p, order):
    def start(bench, loom):
        bench.step(p)
        if order == "one_run":
            loom.run([0, 1])
        else:
            first, second = (0, 1) if order == "master_first" else (1, 0)
            loom.run(first)
            bench.step(p)
            loom.run(second)
    return start


@model_only("twelve starts")
@pytest.mark.parametrize("order", ["one_run", "master_first", "device_first"])
@pytest.mark.parametrize("p", [0, 1, 2, 3])
def test_unio_together_at_every_start_phase(backend, p, order):
    """BUGS 14: both programs set their OD_MASK bit first and read it back.
    Under one RUN at each of the four phases of the slot grid, and each
    started p clocks after the other in both orders, READ 0x0B gives 0xBB."""
    bench, bus, loom = together(backend, _phase(p, order))
    loom.push(0, [w_read(0x0B, 1)])
    assert results(loom) == [0xBB]


# ------------------------------------------------- the reference model alone
def test_unio_model_table_and_helpers():
    """DS20002122B Table 1-2 as the model holds it; MSB-first bytes."""
    assert TABLE["TE"] == (10, 100) and TABLE["TSTBY"] == (600, None)
    assert TABLE["TSS"] == (10, None) and TABLE["THDR"] == (5, None)
    assert TABLE["TIJIT"] == 0.06 and TABLE["TOJIT"] == 0.25
    assert msb_bits(0xA0) == [1, 0, 1, 0, 0, 0, 0, 0] and bits_byte(msb_bits(0x6C)) == 0x6C


@pytest.mark.parametrize("te_us", [10, 100])
def test_unio_model_master_and_device(te_us):
    """The reference master against the reference device, no chip: read,
    WREN, a page write with its poll, the read-back, a refused write, and
    a device that never answers (NoSAK after the address)."""
    bus = OneWireBus(["BIDIR0"])
    dev = bus.add(UnioDevice(busy_us=500))
    m = bus.add(UnioMaster(te_us=te_us))
    ops = [m.read(5, 2), m.wren(), m.write(0x22, [1, 2]), m.rdsr(poll=True),
           m.read(0x22, 2), m.write(0x30, [9]), m.read(0x30, 1)]
    assert bus.run_alone(lambda: m.idle, 100_000_000)
    assert [op.status for op in ops] == ["ok"] * 7
    assert ops[0].data == list(dev.memory[5:7]) and ops[4].data == [1, 2]
    assert ops[6].data == [dev.memory[0x30]] and dev.writes == [(0x22, [1, 2])]
    assert dev.violations == [] and m.violations == []
    bus = OneWireBus(["BIDIR0"])
    bus.add(UnioDevice(respond=False))
    m = bus.add(UnioMaster(te_us=te_us))
    op = m.read(0, 1)
    assert bus.run_alone(lambda: m.idle, 10_000_000)
    assert (op.status, op.failed_at) == ("nosak", 1)


def test_unio_model_catches_a_bad_master_and_a_bad_device():
    """A 4 us THDR, a 5 us TSS and 0.1 UI of jitter are caught by the
    model device; a device 0.2 UI late (inside TOJIT) is read, one 0.3 UI
    late (past it: its middle edge falls after the 0.75 UI sample) is a
    NoSAK to the model master."""
    bus = OneWireBus(["BIDIR0"])
    dev = bus.add(UnioDevice())
    m = bus.add(UnioMaster(thdr_us=4, tss_us=5, jitter=0.1))
    m.read(0, 1)
    m.read(0, 1)
    assert bus.run_alone(lambda: m.idle, 10_000_000)
    assert {v[0] for v in dev.violations} == {"THDR", "TSS", "TIJIT"}
    for skew, status in ((0.2, "ok"), (0.3, "nosak")):
        bus = OneWireBus(["BIDIR0"])
        bus.add(UnioDevice(skew=skew))
        m = bus.add(UnioMaster())
        op = m.read(0, 1)
        assert bus.run_alone(lambda: m.idle, 10_000_000)
        assert op.status == status and m.violations == []
