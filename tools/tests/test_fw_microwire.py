"""L3 for the Microwire (93C46 serial EEPROM) programs, model and RTL.

``firmware/microwire_host.loom`` (CS = OUT0, SK = OUT1, DI = OUT2, DO =
IN0) is the host; ``firmware/microwire_device.loom`` (CS = IN1, SK = IN2,
DI = IN3, DO = BIDIR0 with its output enable) is a 93C46 in x16 with 64
words in data memory. The references are ``tools.protomodels.microwire``:
``MicrowireMonitor`` decodes the pads and checks the host's edges against
a row of the data sheet's AC table and the part's DO against TPD,
``Eeprom93C46`` is the part the host program talks to, ``MicrowireHost``
drives the device program. Expected timings come from those figures and
the program headers' stated windows, never from what the model or RTL did.

Rates (50 MHz core clock): the host at TICK_INT 16 (SK 1.56 MHz, high and
low 320 ns) against the 2.5..4.5 V row, and at 28 (893 kHz) with the device
program, which answers like a 93LC46 at 1.8..2.5 V (its header). Write
cycles are a few microseconds, set by the host, not the data sheet's
milliseconds, so every body is cheap enough for the RTL except the two
fault-injection bodies of the models alone.

The PHY-GL-SDF candidate is ``test_microwire_host_and_device_together``: two
threads, four board wires, EWEN, a write with its ready/busy poll, a
sequential read across the wrap, EWDS and a refused write, every edge
checked against the 1.8..2.5 V row.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import microwire as M
from tools.protomodels.mdio import Wire
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
HOST, DEVICE = FIRMWARE / "microwire_host.loom", FIRMWARE / "microwire_device.loom"
CLOCK_HZ = 50e6
TICK = 16                       # the host's half SK period: 320 ns
SLOW = 50                       # 1 us: 500 kHz, the device's rate (its header)
MEM = 0x180                     # the device's array (its header)
TIMEOUT, BUSY_ID = 0x1, 0x2     # host status bits (its header)
STARTUP = 300                   # clocks for a program's set-up after RUN
HOST_PINS = ("OUT0", "OUT1", "OUT2", "IN0")
DEV_PINS = ("IN1", "IN2", "IN3", "BIDIR0")


def program(path, thread=0):
    """A program from ``firmware/`` with ``--strict``, moved to ``thread``."""
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, path.name, isa=ISA, imem_words=IMEM, strict=True)


def mbench(backend, pullups=0):
    """FIFOs, the bit engine and data memory, 512 words; the RTL has them
    all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM, pullups=pullups)
    return backend.bench(pullups=pullups)


def cmd(op, addr=0, n=1):
    """The host's command word: the instruction, and for READ n - 1."""
    return M.instr(op, addr) | ((n - 1) << 9 if op == M.OP_READ else 0)


def hread(loom, addr, n=1):
    """A (sequential) READ: ``(words, status)``."""
    loom.push(0, [cmd(M.OP_READ, addr, n)])
    got = loom.pop(0, n + 1)
    return got[:n], got[n]


def hwrite(loom, addr, data):
    loom.push(0, [cmd(M.OP_WRITE, addr), data])
    return loom.pop(0, 1)[0]


def hext(loom, e, data=None):
    loom.push(0, [M.ext(e)] + ([data] if data is not None else []))
    return loom.pop(0, 1)[0]


def phases(mon):
    """``(highs, lows)`` of SK in clocks while CS is high, from the monitor."""
    highs = [f - r for r, f in zip(mon.rises, mon.falls)]
    lows = [r - f for f, r in zip(mon.falls, mon.rises[1:]) if r - f < 200]
    return highs, lows


# ------------------------------------------------------------- assembly
def test_microwire_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: the host's every pair is proved at its .tick of 16
    clocks, nothing unbounded and no ``.bounded``; the device has no
    deadline pair by design (it follows SK). The host fits its quarter; the
    device's code is 127 words of its quarter, its array is 0x180..0x1BF, and moved to thread 1 it
    does not overlap the host."""
    host = program(HOST)
    assert host.diagnostics == [] and list(host.threads) == [0]
    report = host.deadlines[0]
    assert report.period == 16
    assert report.pairs and not report.infeasible and not report.unbounded
    assert report.worst_slack >= 0
    assert len(host.words) == 87 and max(host.words) < 0x80
    dev = program(DEVICE)
    assert dev.diagnostics == [] and dev.deadlines[0].pairs == []
    code = [a for a in dev.words if a < MEM]
    assert len(code) == 127 and max(code) < 0x80
    assert sorted(a for a in dev.words if a >= MEM) == list(range(MEM, MEM + 64))
    moved = program(DEVICE, thread=1)
    moved_code = [a for a in moved.words if a < MEM]
    assert min(moved_code) == 0x80 and max(moved_code) < 0x100
    assert not set(host.words) & set(moved.words)


# ------------------------------------------------- the host, model part
def host_setup(backend, tick=TICK, timeout=400, row="2.5-4.5V", **part):
    bench = mbench(backend)
    part.setdefault("delay", 12)                    # 240 ns: TPD 250
    part.setdefault("tw", 1500)                     # 30 us
    eep = bench.add(M.Eeprom93C46(*HOST_PINS, **part))
    mon = bench.add(M.MicrowireMonitor(*HOST_PINS, row=row, clock_hz=CLOCK_HZ))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(HOST))
    loom.write_csr(0, "TICK_INT", tick)
    loom.run(0)
    loom.push(0, [timeout])
    return bench, eep, mon, loom


def test_host_writes_reads_and_erases(backend):
    """At 1.56 MHz against a part 240 ns slow (the 2.5..4.5 V row's TPD
    limit is 250): EWEN, a WRITE that the poll sees finish, a READ back, a
    sequential READ of three words across the wrap 3F -> 00, an ERASE. The
    instructions decoded off the pads are the ones sent; every host edge
    is inside the row; SK is high and low one tick each."""
    mem = [0x1000 + i for i in range(64)]
    bench, eep, mon, loom = host_setup(backend, mem=mem)
    assert hext(loom, M.EXT_EWEN) == 0
    assert hwrite(loom, 9, 0xBEEF) == 0
    assert eep.mem[9] == 0xBEEF
    assert hread(loom, 9) == ([0xBEEF], 0)
    assert hread(loom, 62, 3) == ([0x103E, 0x103F, 0x1000], 0)
    loom.push(0, [cmd(M.OP_ERASE, 9)])
    assert loom.pop(0, 1) == [0]
    assert eep.mem[9] == 0xFFFF
    names = [i["name"] for i in mon.instructions]
    assert names[:4] == ["EWEN", "WRITE", "READ", "READ"]
    assert mon.instructions[1]["data"] == 0xBEEF
    assert mon.instructions[3]["words"][:3] == [0x103E, 0x103F, 0x1000]
    assert mon.violations == []
    highs, lows = phases(mon)
    assert min(highs) >= TICK - 3 and min(lows) >= TICK - 3
    assert bench.contention_free() and loom.badop() == 0


def test_host_write_refused_before_ewen(backend):
    """Erase/write is disabled at power-up: a WRITE before EWEN starts no
    cycle, DO never shows ready, and the host reports a timeout (status
    bit 0, its header) after its 40-tick timeout (640 clocks, the cycle
    is 400); the word is unchanged.
    After EWEN the same WRITE lands; after EWDS a WRITE is refused again."""
    bench, eep, mon, loom = host_setup(backend, timeout=40, tw=400,
                                       mem=[0x0123] * 64)
    assert hwrite(loom, 4, 0x4444) == TIMEOUT
    assert eep.mem[4] == 0x0123 and eep.log[0]["refused"]
    assert hread(loom, 4) == ([0x0123], 0)
    assert hext(loom, M.EXT_EWEN) == 0
    assert hwrite(loom, 4, 0x4444) == 0
    assert hext(loom, M.EXT_EWDS) == 0
    assert hwrite(loom, 4, 0x5555) == TIMEOUT
    assert hread(loom, 4) == ([0x4444], 0)
    poll = mon.polls[0]
    assert poll["ready_at"] is None
    assert poll["end"] - poll["start"] >= 40 * TICK
    assert mon.violations == []


def test_host_polls_ready_busy(backend):
    """The poll (data sheet: DO is 0 while the cycle runs and 1 after it,
    TSV after CS rises) on a 3,000-clock WRITE, WRAL and ERAL: the host
    sees ready once the part's cycle is over, within the tick it polls
    at, and not before."""
    bench, eep, mon, loom = host_setup(backend, tw=3000, timeout=400)
    assert hext(loom, M.EXT_EWEN) == 0
    assert hwrite(loom, 1, 0x1111) == 0
    end = eep.log[-1]["cycle"] + 3000
    p = mon.polls[-1]
    assert end <= p["ready_at"] <= end + 2 * TICK + 20
    assert hext(loom, M.EXT_WRAL, 0x2222) == 0
    assert eep.mem == [0x2222] * 64
    assert hext(loom, M.EXT_ERAL) == 0
    assert eep.mem == [0xFFFF] * 64
    assert all(q["ready_at"] for q in mon.polls) and mon.violations == []


def test_host_times_out_on_a_long_cycle(backend):
    """A timeout of 100 ticks (1,600 clocks) against a 3,000-clock cycle:
    status bit 0, the poll ends with DO still 0; once the cycle is over the
    word reads back written (the timeout is the host's, not the part's)."""
    bench, eep, mon, loom = host_setup(backend, tw=3000, timeout=100)
    assert hext(loom, M.EXT_EWEN) == 0
    assert hwrite(loom, 7, 0x7777) == TIMEOUT
    assert mon.polls[-1]["ready_at"] is None and mon.polls[-1]["do"] == 0
    bench.step(3000)
    assert hread(loom, 7) == ([0x7777], 0)


def test_host_reports_a_missing_dummy_bit(backend):
    """Error injection: DO held high (no part, a board pull-up): the bit
    before D15 is 1, status bit 1, and the data reads 0xFFFF."""
    bench = mbench(backend)
    bench.add(_Level("IN0", 1))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(HOST))
    loom.write_csr(0, "TICK_INT", TICK)
    loom.run(0)
    loom.push(0, [40])
    assert hread(loom, 0) == ([0xFFFF], BUSY_ID)


class _Level(M.Model):
    """A pad held at one level."""

    def __init__(self, pin, level):
        self.pad, self.level = M.pad_of(pin), level

    def drive(self, drive, cycle):
        drive.set(self.pad, self.level)


# ------------------------------------------------- the device, model host
def dev_setup(backend, half=SLOW, tw_units=100, gap=40, poll=5000):
    bench = mbench(backend)
    host = bench.add(M.MicrowireHost(*DEV_PINS, half=half, gap=gap, poll=poll))
    mon = bench.add(M.MicrowireMonitor(*DEV_PINS, row="1.8-2.5V",
                                       clock_hz=CLOCK_HZ))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(DEVICE))
    loom.run(0)
    loom.push(0, [tw_units])
    bench.step(STARTUP)
    return bench, host, mon, loom


def run_host(bench, host, clocks):
    assert bench.run_until(lambda: not host.busy, clocks, every=32)


def test_device_answers_the_instruction_set(backend):
    """The model host at 500 kHz (1 us high and low): a WRITE refused
    before EWEN, EWEN, WRITE, READ, a sequential READ across the wrap,
    ERASE, EWDS, a refused ERASE. A write cycle of 100 units is 1,600
    clocks (its header: 16 clocks a unit): the poll sees DO 0 then 1 after
    that. Every DO change in a read is 8 to 19 clocks after its rise (the
    header's window; the 1.8..2.5 V row's TPD is 400 ns, 20 clocks)."""
    bench, host, mon, loom = dev_setup(backend)
    host.write(5, 0xDEAD)
    host.command(M.EXT_EWEN)
    host.write(5, 0x5A5A)
    host.write(63, 0x0063)
    host.read(5)
    host.read(63, 2)
    host.erase(5)
    host.read(5)
    host.command(M.EXT_EWDS)
    host.erase(63)
    run_host(bench, host, 60000)
    r = host.results
    assert [x["name"] for x in r] == ["WRITE", "EWEN", "WRITE", "WRITE", "READ",
                                      "READ", "ERASE", "READ", "EWDS", "ERASE"]
    assert r[0]["ready"] is None and r[9]["ready"] is None   # refused
    for x in (r[2], r[3], r[6]):
        assert 1600 - 40 <= x["busy"] <= 1600 + 80, x   # gap 40 before the poll
    assert (r[4]["dummy"], r[4]["words"]) == (0, [0x5A5A])
    assert r[5]["words"] == [0x0063, 0xFFFF]
    assert r[7]["words"] == [0xFFFF]
    assert mon.violations == []
    assert min(mon.do_delays) >= 8 and max(mon.do_delays) <= 19
    loom.halt(0)
    assert loom.read_imem(MEM + 63) == [0x0063]
    assert loom.read_imem(MEM + 5) == [0xFFFF]
    assert bench.contention_free() and loom.badop() == 0


def test_device_wral_and_eral(backend):
    """WRAL and ERAL store all 64 words (1,792 clocks more than the set
    time, the header says) and show busy throughout."""
    bench, host, mon, loom = dev_setup(backend, tw_units=20)
    host.command(M.EXT_EWEN)
    host.command(M.EXT_WRAL, 0x3C3C)
    host.read(0, 2)
    host.read(40)
    host.command(M.EXT_ERAL)
    host.read(17)
    run_host(bench, host, 60000)
    r = host.results
    assert 320 + 1792 - 40 <= r[1]["busy"] <= 320 + 1792 + 80, r[1]
    assert r[2]["words"] == [0x3C3C, 0x3C3C] and r[3]["words"] == [0x3C3C]
    assert r[5]["words"] == [0xFFFF] and r[4]["ready"] is not None
    assert mon.violations == []


def test_device_shows_busy_until_the_cycle_ends(backend):
    """Ready/busy: the model host polls DO the whole cycle; the device
    drives 0 from TSV after CS rises (TSV <= 500 ns at 1.8 V: 25 clocks)
    to the end of the cycle, then 1."""
    bench, host, mon, loom = dev_setup(backend, tw_units=200)
    host.command(M.EXT_EWEN)
    host.write(2, 0x0202)
    run_host(bench, host, 20000)
    p = mon.polls[-1]
    assert p["ready_at"] - p["start"] >= 3200 - 200
    assert host.results[1]["ready"] is not None
    assert mon.violations == []


@model_only("the margin case of the header's budget; 500 kHz covers the RTL")
def test_device_at_its_budget_limit(backend):
    """The header's budget: twelve slots between two waits, so a period of
    64 clocks (781 kHz, 640 ns high and low) still works; checked against
    the 1.8..2.5 V row (FCLK 1 MHz) with a sequential read and a write."""
    bench, host, mon, loom = dev_setup(backend, half=32)
    host.command(M.EXT_EWEN)
    host.write(10, 0xA5C3)
    host.read(9, 3)
    run_host(bench, host, 30000)
    assert host.results[2]["words"] == [0xFFFF, 0xA5C3, 0xFFFF]
    assert mon.violations == []


# ------------------------------------------------------------- together
def test_microwire_host_and_device_together(backend):
    """The host on thread 0 (CS, SK, DI = OUT0..2, DO = IN0) and the device
    on thread 1 (CS, SK, DI = IN1..3, DO = BIDIR0), the bench wiring
    OUT0..2 to IN1..3 and BIDIR0 to IN0. The host at TICK_INT 50 (500 kHz,
    the device's rate): a WRITE refused before EWEN, EWEN, a WRITE with its
    poll, a sequential READ across the wrap, EWDS, a refused WRITE. Every
    edge on the device's pads is inside the 1.8..2.5 V row. The PHY-GL-SDF
    candidate."""
    bench = mbench(backend)
    for src, dst in zip(HOST_PINS[:3], DEV_PINS[:3]):
        bench.add(Wire(src, dst))
    bench.add(Wire("BIDIR0", "IN0"))
    mon = bench.add(M.MicrowireMonitor(*DEV_PINS, row="1.8-2.5V",
                                       clock_hz=CLOCK_HZ))
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(HOST).words)
    dev = program(DEVICE, thread=1).words
    assert not set(words) & set(dev)
    words.update(dev)
    loom.load(words)
    loom.write_csr(0, "TICK_INT", SLOW)
    loom.push(1, [100])                         # 1,600 clocks a write
    loom.push(0, [60])                          # 60 ticks: 3,000 clocks
    loom.run([0, 1])
    bench.step(STARTUP)
    assert hwrite(loom, 63, 0x1357) == TIMEOUT  # refused: erase/write disabled
    assert hext(loom, M.EXT_EWEN) == 0
    assert hwrite(loom, 63, 0x2468) == 0
    assert hread(loom, 63, 2) == ([0x2468, 0xFFFF], 0)
    assert hext(loom, M.EXT_EWDS) == 0
    assert hwrite(loom, 0, 0x0BAD) == TIMEOUT
    assert [i["name"] for i in mon.instructions] == \
        ["WRITE", "EWEN", "WRITE", "READ", "EWDS", "WRITE"]
    assert mon.violations == []
    assert min(mon.do_delays) >= 8 and max(mon.do_delays) <= 19
    loom.halt([0, 1])
    assert loom.read_imem(MEM + 63) == [0x2468] and loom.read_imem(MEM) == [0xFFFF]
    assert bench.contention_free() and loom.badop() == 0


# ------------------------------------------------- the reference model alone
def test_model_instruction_format():
    """The instruction set table, x16: start bit, op code, six address
    bits; the 00 group's extension in A5 A4."""
    assert M.instr(M.OP_READ, 0x2A) == 0b1_10_101010
    assert M.ext(M.EXT_EWEN) == 0b1_00_110000
    assert M.ext(M.EXT_WRAL) == 0b1_00_010000
    assert [M.kind(op, a) for op, a in ((0, 0x30), (0, 0x00), (0, 0x20),
                                        (0, 0x10), (1, 5), (2, 5), (3, 5))] == \
        ["EWEN", "EWDS", "ERAL", "WRAL", "WRITE", "READ", "ERASE"]


def test_model_monitor_catches_faults():
    """The monitor flags a host whose SK high time is short of the row's
    TCKH (at 2.5..4.5 V, 250 ns = 12.5 clocks: a host at 10 clocks) and a
    part slower than TPD (250 ns: a part at 14 clocks); a compliant pair
    is clean."""
    from tools.protomodels.bench import Bench
    pins = (("ui", 1), ("ui", 2), ("ui", 3), "BIDIR0")

    def run(half, delay):
        bench = Bench(features=("FIFO",))
        host = bench.add(M.MicrowireHost(*pins, half=half))
        bench.add(M.Eeprom93C46(*pins, delay=delay, mem=[0x8001] * 64))
        mon = bench.add(M.MicrowireMonitor(*pins, row="2.5-4.5V"))
        host.read(1)
        assert bench.run_until(lambda: not host.busy, 20000, every=16)
        return host, mon

    host, mon = run(16, 12)
    assert mon.violations == [] and host.results[0]["words"] == [0x8001]
    kinds = {t.split(" ")[0] for _, t in run(10, 12)[1].violations}
    assert "TCKH" in kinds
    assert any(t.startswith("TPD") for _, t in run(16, 14)[1].violations)
