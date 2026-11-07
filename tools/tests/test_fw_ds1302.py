"""L3-DS1302 on the model and the RTL: firmware/ds1302_host.loom and
firmware/ds1302_device.loom, the DS1302 timekeeper programs (D-035).

The host (thread 0: CE = OUT0, SCLK = OUT1, I/O = BIDIR0) is fed through
``tools.loomhost.Loom`` and talks to ``tools.protomodels.ds1302.Ds1302Chip``;
the device (thread 1: CE = IN0, SCLK = IN1, I/O = BIDIR1) is driven by
``Ds1302Host``. Together, ``Ds1302Wire`` joins OUT0 to IN0, OUT1 to IN1 and
BIDIR0 to BIDIR1 (two Loom threads never share a pin).

What is expected comes from the DS1302 data sheet dated 070900 as
``protomodels.ds1302`` cites it (the command byte of p.3, the burst and
write-protect rules of p.4 and p.5, the AC table of p.10) and from the
program headers (the host words, the reports, the two rates), never from
what the model or the RTL did. Every edge on the bus goes through
``ds1302.check``, and no cycle may have both sides driving I/O.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import functools

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.ds1302 import (CLOCK_BURST_R, CLOCK_BURST_W, CONTROL,
                                      CONTROL_W, MINUTES, RAM_BURST_R,
                                      RAM_BURST_W, SECONDS, TRICKLE, WP, YEAR,
                                      Ds1302Chip, Ds1302Host, Ds1302Wire,
                                      Probe, Registers, check, command,
                                      loom_drives, model_drives)
from tools.tests.fw_backend import model_only

ISA = load()
HOST = REPO / "firmware" / "ds1302_host.loom"
DEVICE = REPO / "firmware" / "ds1302_device.loom"
US = 50                            # clocks per microsecond: 50 MHz
STARTUP = 200                      # clocks for a program's set-up after RUN
U5, U2 = 12, 28                    # the host's ticks (its "Timing")
BANKA, BANKB, TRK, RAM = 0x0C0, 0x0C8, 0x0D0, 0x0E0      # the device's "Memory"
TIME = [0x30, 0x59, 0x23, 0x31, 0x12, 0x04, 0x26, 0x00]  # 23:59:30 Thu 31/12/26, WP clear


@functools.lru_cache(maxsize=None)
def program(path):
    """The program assembled with ``--strict``."""
    return assemble_text(path.read_text(encoding="utf-8"), path.name, isa=ISA,
                         strict=True)


def ds_bench(backend, *models):
    """The chip with the FIFOs, the data memory and the 512-word memory; the
    RTL has all of them and ``RtlBench`` takes none of those keywords."""
    kwargs = {}
    if backend.name == "model":
        kwargs.update(features=("FIFO", "DMEM"), imem_words=512)
    bench = backend.bench(**kwargs)
    for model in models:
        bench.add(model)
    return bench


def xfer(cmd, arg=0):
    """A host transfer word (ds1302_host.loom "Host words")."""
    return 1 << 15 | cmd << 8 | arg


def bus_ok(probe, vcc, **kw):
    """Every edge inside p.10 at ``vcc`` and no contention: the decoded
    transfers and the measurements."""
    result = check(probe.edges, probe.clock_hz, vcc, **kw)
    assert result["violations"] == []
    return result


# ------------------------------------------------------------- the programs
@pytest.mark.parametrize("path,thread,words", [(HOST, 0, 167), (DEVICE, 1, 223)])
def test_ds1302_programs_assemble_strict_with_no_diagnostic(path, thread, words):
    """L2-DEADLINE: the host's pairs proved at its 12-clock tick (the 5 V
    rate; the 2 V rate's 28 is slower); the device follows SCLK and has no
    pair by design. The host's main loop is in thread 0's quarter, its
    subroutines in thread 3's (0x180..0x1B7) and its setting in the last
    word; the device in thread 1's and 2's quarters and its receive
    subroutines at 0x1C0; the two load together."""
    prog = program(path)
    assert prog.diagnostics == []
    report = prog.deadlines[thread]
    if path == HOST:
        assert report.period == U5
        assert report.pairs and not report.infeasible and not report.unbounded
        assert report.worst_slack >= 0
        assert all(a < 0x080 or 0x180 <= a < 0x200 for a in prog.words)
    else:
        assert report.pairs == []
        assert all(0x080 <= a < 0x180 or 0x1C0 <= a < 0x1FF for a in prog.words)
    assert list(prog.threads) == [thread] and len(prog.words) == words
    assert not set(program(HOST).words) & set(program(DEVICE).words)


def phases(result, low, high):
    """Inside a byte SCLK is exactly ``low`` and ``high`` clocks (the tick
    is a multiple of the slot, SEMANTICS 2); between bytes the low phase is
    longer, and the eighth clock of a read's command is high one slot
    longer (the host's "Timing")."""
    ns = 20.0
    assert min(result["tCL"]) == low * ns and min(result["tCH"]) == high * ns
    assert max(result["tCH"]) <= (high + 4) * ns
    lows = result["tCL"]
    assert lows.count(low * ns) >= len(lows) * 3 // 4
    assert min(result["period"]) == (low + high) * ns


# ------------------------------------------------------------- the host
def host_setup(backend, regs=None, rate=None, **chip):
    """The host on thread 0, a model DS1302 on its pads (5 V delays unless
    given), a probe on the bus."""
    chip = Ds1302Chip("OUT0", "OUT1", "BIDIR0", regs or Registers(), **chip)
    probe = Probe("OUT0", "OUT1", "BIDIR0", loom_drives("BIDIR0"), model_drives(chip))
    bench = ds_bench(backend, chip, probe)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(HOST))
    loom.run(0)
    bench.step(STARTUP)
    if rate is not None:
        assert do(loom, [0x1000 | rate]) == [0x1000 | rate]
    return bench, chip, probe, loom


def do(loom, words, results=1):
    """Push an operation's words, pop its result words."""
    loom.push(0, words)
    return loom.pop(0, results, max_polls=40_000)


def read_words(cmd, data):
    """What the host gives back for a read: command << 8 | byte, a word each."""
    return [(cmd | 1) << 8 | b for b in data]


def test_host_single_byte_write_and_read(backend):
    """Figure 3 both ways at the 5 V rate: clear WP (8EH 00H), write the
    minutes (82H 59H), read them back (83H, the chip's 59H). On the bus:
    exactly those bytes LSB first, every figure of p.10 at 5 V for the host
    and the chip, SCLK 833 kHz (1.2 us: 800 ns low, 400 ns high), and no
    cycle with both sides on I/O (the turnaround)."""
    bench, chip, probe, loom = host_setup(backend)
    assert chip.regs.wp
    w = xfer(CONTROL_W, 0x00)
    assert do(loom, [w]) == [w]
    w = xfer(command(MINUTES), 0x59)
    assert do(loom, [w]) == [w]
    r = command(MINUTES, read=True)
    assert do(loom, [xfer(r)]) == read_words(r, [0x59])
    assert chip.regs.clock[MINUTES] == 0x59 and not chip.regs.wp
    result = bus_ok(probe, 5.0)
    assert result["transfers"] == [(CONTROL_W, [0]), (command(MINUTES), [0x59]),
                                   (r, [0x59])]
    phases(result, low=40, high=20)
    assert min(result["tCC"]) >= 1000 and min(result["tCWH"]) >= 1000
    assert min(result["tCCH"]) >= 60 and min(result["tCDH"]) >= 70
    assert min(result["tDC"]) >= 50
    assert bench.contention_free() and loom.badop() == 0


def test_host_clock_burst_write_and_read(backend):
    """A clock burst write (BEH and the eight registers, the control byte
    last: p.4, "the first eight registers must be written") sets the time;
    a burst read (BFH, argument 0: the whole burst) gives the eight back in
    order; a burst read of 3 stops after three."""
    bench, chip, probe, loom = host_setup(backend, Registers(clock=[0] * 8))
    w = xfer(CLOCK_BURST_W, 0)
    words = [TIME[i] | TIME[i + 1] << 8 for i in range(0, 8, 2)]
    assert do(loom, [w] + words) == [w]       # push waits for room in INQ
    assert chip.regs.clock == TIME
    assert do(loom, [xfer(CLOCK_BURST_R)], 8) == read_words(CLOCK_BURST_R, TIME)
    assert do(loom, [xfer(CLOCK_BURST_R, 3)], 3) == read_words(CLOCK_BURST_R, TIME[:3])
    result = bus_ok(probe, 5.0)
    assert result["transfers"] == [(CLOCK_BURST_W, TIME), (CLOCK_BURST_R, TIME),
                                   (CLOCK_BURST_R, TIME[:3])]
    assert bench.contention_free() and loom.badop() == 0


def test_host_write_blocked_by_write_protect_then_guarded(backend):
    """WP set (p.4: it "prevents a write operation to any other register"):
    a plain write of the year (8CH 26H) changes nothing and reads back the
    old 00H. With write-protect handling on (0x2001) the same write goes out
    as 8EH 00H, 8CH 26H, 8EH 80H, so the year changes and WP is set again
    after it; a clock burst write is preceded by 8EH 00H only (its eighth
    byte is the control register)."""
    bench, chip, probe, loom = host_setup(backend)
    year_w, year_r = command(YEAR), command(YEAR, read=True)
    assert chip.regs.wp
    assert do(loom, [xfer(year_w, 0x26)]) == [xfer(year_w, 0x26)]
    assert do(loom, [xfer(year_r)]) == read_words(year_r, [0x00])
    assert chip.regs.clock[YEAR] == 0 and chip.transfers[0][2] == 0
    assert do(loom, [0x2001]) == [0x2001]
    assert do(loom, [xfer(year_w, 0x26)]) == [xfer(year_w, 0x26)]
    assert chip.regs.clock[YEAR] == 0x26 and chip.regs.wp
    w = xfer(CLOCK_BURST_W, 8)
    words = [TIME[i] | TIME[i + 1] << 8 for i in range(0, 8, 2)]
    assert do(loom, [w] + words) == [w]
    assert chip.regs.clock == TIME and not chip.regs.wp
    result = bus_ok(probe, 5.0)
    assert result["transfers"] == [
        (year_w, [0x26]), (year_r, [0x00]),
        (CONTROL_W, [0x00]), (year_w, [0x26]), (CONTROL_W, [WP]),
        (CONTROL_W, [0x00]), (CLOCK_BURST_W, TIME)]
    assert bench.contention_free() and loom.badop() == 0


def test_host_at_the_2v_rate(backend):
    """The 2 V setting (0x1001, TICK_INT 28): every figure of p.10 at 2.0 V
    (500 kHz maximum, 1 us phases, tCC and tCWH 4 us, tCDD 800 ns from a
    chip at that limit) on a single write, a RAM burst write of 5 (odd: half
    of the third data word) and a RAM burst read of the whole 31."""
    regs = Registers(clock=[0] * 8)
    bench, chip, probe, loom = host_setup(backend, regs, rate=1, tcdd=38, tccz=13, tcdz=13)
    w = xfer(command(SECONDS), 0x45)
    assert do(loom, [w]) == [w]
    data = [0xDE, 0xAD, 0xBE, 0xEF, 0x42]
    w = xfer(RAM_BURST_W, len(data))
    assert do(loom, [w, 0xADDE, 0xEFBE, 0x0042]) == [w]
    want = data + [0] * 26
    assert do(loom, [xfer(RAM_BURST_R)], 31) == read_words(RAM_BURST_R, want)
    assert chip.regs.clock[SECONDS] == 0x45 and chip.regs.ram == want
    result = bus_ok(probe, 2.0)
    phases(result, low=88, high=52)
    assert result["transfers"] == [(command(SECONDS), [0x45]), (RAM_BURST_W, data),
                                   (RAM_BURST_R, want)]
    assert max(result["tCDD"]) == 38 * 20.0
    assert bench.contention_free() and loom.badop() == 0


@model_only("register 8 and an empty address: the single-byte bodies run the same path on the RTL")
def test_host_trickle_and_empty_address(backend):
    """Register 8, the trickle charger (90H/91H: p.4, TCS 1010, one diode,
    2 kOhm = A5H), written and read singly; register 20 holds nothing (p.4)
    and reads 00H."""
    bench, chip, probe, loom = host_setup(backend, Registers(clock=[0] * 8))
    w = xfer(command(TRICKLE), 0xA5)
    assert do(loom, [w]) == [w]
    r = command(TRICKLE, read=True)
    assert do(loom, [xfer(r)]) == read_words(r, [0xA5])
    r20 = command(20, read=True)
    assert do(loom, [xfer(r20)]) == read_words(r20, [0x00])
    assert chip.regs.trickle == 0xA5
    bus_ok(probe, 5.0)
    assert bench.contention_free() and loom.badop() == 0


# ------------------------------------------------------------- the device
def device_setup(backend, **host):
    """The device on thread 1, a model host (the data sheet's 2.0 V minima
    unless given) on its pads, a probe on the bus."""
    ctl = Ds1302Host("IN0", "IN1", "BIDIR1", **host)
    probe = Probe("IN0", "IN1", "BIDIR1", model_drives(ctl), loom_drives("BIDIR1"))
    bench = ds_bench(backend, ctl, probe)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(DEVICE))
    loom.run(1)
    bench.step(STARTUP)
    return bench, ctl, probe, loom


def finish(bench, ctl, max_cycles=400_000):
    assert bench.run_until(lambda: not ctl.busy, max_cycles, every=64)
    bench.step(400)                     # CE low; the report


def memory(loom, addr, n):
    """The device's registers, read through the IMEM space (the threads
    halted: the end of a body)."""
    loom.halt()
    return loom.read_imem(addr, n)


def reports(loom, n):
    return loom.pop(1, n, max_polls=2_000)


def test_device_single_byte_reads_and_writes(backend):
    """A host at the data sheet's 2.0 V limits (500 kHz, 1 us phases, data
    held 280 ns, CE set-up and inactive 4 us): the seconds read at power on
    (80H, the clock halted: the program's choice), WP cleared, the minutes
    and RAM 5 written and read back. Every figure of p.10 at 2.0 V on both
    sides, no contention; a report per write (command << 8 | 1)."""
    bench, ctl, probe, loom = device_setup(backend)
    ctl.read(command(SECONDS))
    ctl.write(CONTROL_W, [0x00])
    ctl.write(command(MINUTES), [0x59])
    ctl.read(command(MINUTES))
    ctl.write(command(5, ram=True), [0x77])
    ctl.read(command(5, ram=True))
    finish(bench, ctl)
    assert ctl.results == [(command(SECONDS, read=True), [0x80]), (CONTROL_W, [0]),
                           (command(MINUTES), [0x59]), (command(MINUTES, read=True), [0x59]),
                           (command(5, ram=True), [0x77]),
                           (command(5, ram=True, read=True), [0x77])]
    assert reports(loom, 3) == [CONTROL_W << 8 | 1, command(MINUTES) << 8 | 1,
                                command(5, ram=True) << 8 | 1]
    result = bus_ok(probe, 2.0)
    late = [t for t in result["tCDD"] if t > 10 * 20]
    assert len(late) <= 3 and max(result["tCCZ"]) <= 14 * 20   # one per byte read
    assert bench.contention_free() and loom.badop() == 0
    assert memory(loom, RAM + 5, 1) == [0x77]


def test_device_clock_and_ram_bursts(backend):
    """The clock burst (p.4, p.5) written whole and read back in order, the
    eighth byte the control register (WP clear); the RAM burst written with
    31 bytes and read back, then a RAM burst of 3 read."""
    bench, ctl, probe, loom = device_setup(backend)
    ram = [(i * 37 + 5) & 0xFF for i in range(31)]
    ctl.write(CONTROL_W, [0x00])
    ctl.write(CLOCK_BURST_W, TIME)
    ctl.read(CLOCK_BURST_R, 8)
    ctl.write(RAM_BURST_W, ram)
    ctl.read(RAM_BURST_R, 31)
    ctl.read(RAM_BURST_R, 3)
    finish(bench, ctl, 800_000)
    assert [r[1] for r in ctl.results[2:]] == [TIME, ram, ram, ram[:3]]
    assert reports(loom, 3) == [CONTROL_W << 8 | 1, CLOCK_BURST_W << 8 | 8,
                                RAM_BURST_W << 8 | 31]
    bus_ok(probe, 2.0)
    assert bench.contention_free() and loom.badop() == 0
    assert memory(loom, RAM, 31) == ram
    assert loom.read_imem(BANKB, 8) == TIME           # the burst landed in bank B
    assert loom.read_reg(1, "r7") == BANKB            # which is now the current one


def test_device_honours_write_protect(backend):
    """WP set (the power-on state here): the minutes, RAM 0, a RAM burst and
    a clock burst are dropped (reports of 0 bytes); the control register is
    still writable (p.4). Once WP is clear, a clock burst cut short by CE
    after five bytes transfers nothing (p.4: all eight must be written).
    OUTQ holds four reports, so the host pops them between the two halves
    (a full OUTQ drops a report rather than stall the bus)."""
    bench, ctl, probe, loom = device_setup(backend)
    ctl.write(command(MINUTES), [0x11])
    ctl.write(command(0, ram=True), [0x22])
    ctl.write(RAM_BURST_W, [1, 2, 3])
    ctl.write(CLOCK_BURST_W, TIME[:7] + [0x00])
    finish(bench, ctl, 800_000)
    assert reports(loom, 4) == [command(MINUTES) << 8, command(0, ram=True) << 8,
                                RAM_BURST_W << 8, CLOCK_BURST_W << 8]
    ctl.write(CONTROL_W, [0x00])
    ctl.write(CLOCK_BURST_W, TIME[:5])
    ctl.read(CLOCK_BURST_R, 8)
    ctl.read(command(0, ram=True))
    finish(bench, ctl, 800_000)
    assert reports(loom, 2) == [CONTROL_W << 8 | 1, CLOCK_BURST_W << 8]
    assert ctl.results[-2:] == [(CLOCK_BURST_R, [0x80, 0, 0, 1, 1, 1, 0, 0]),
                                (command(0, ram=True, read=True), [0])]
    bus_ok(probe, 2.0)
    assert bench.contention_free() and loom.badop() == 0


@model_only("a decode branch; the RTL budget goes to the bodies that move data")
def test_device_ignores_a_command_without_bit_7(backend):
    """Bit 7 clear (p.3: "writes to the DS1302 will be disabled"): a write
    of 02H stores nothing and is not reported, and a read 03H is not
    answered: the device never drives I/O (the host reads the pull-down)."""
    bench, ctl, probe, loom = device_setup(backend)
    ctl.write(0x02, [0x55])
    ctl.read(0x03)
    finish(bench, ctl)
    assert ctl.results == [(0x02, [0x55]), (0x03, [0x00])]
    assert loom.read_reg(1, "OUTQ_CNT") == 0
    assert not any(e[5] for e in probe.edges)
    assert loom.badop() == 0


# ------------------------------------------------------------- together
def together(backend, rate=1):
    """Host (thread 0) and device (thread 1) under one RUN, the board wire
    between their pads; the host at the 2 V rate unless ``rate`` = 0."""
    wire = Ds1302Wire()
    probe = Probe("OUT0", "OUT1", "BIDIR0", loom_drives("BIDIR0"), loom_drives("BIDIR1"))
    bench = ds_bench(backend, wire, probe)
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(HOST).words)
    assert not set(words) & set(program(DEVICE).words)
    words.update(program(DEVICE).words)
    loom.load(words)
    loom.run([0, 1])
    bench.step(STARTUP)
    assert do(loom, [0x1000 | rate]) == [0x1000 | rate]
    return bench, wire, probe, loom


def test_ds1302_host_and_device_together(backend):
    """The two programs on one board at the host's 2 V rate: with
    write-protect handling on, a clock burst write (cleared first by 8EH 00H
    and closing with its own control byte), the burst read back, a RAM byte
    written (wrapped in 8EH 00H and 8EH 80H) and read. The device's reports
    follow each write. Every figure of p.10 at 2.0 V on both sides, and no
    cycle in which both threads drive I/O (the wire and the bench agree)."""
    bench, wire, probe, loom = together(backend)
    assert do(loom, [0x2001]) == [0x2001]
    w = xfer(CLOCK_BURST_W, 0)
    words = [TIME[i] | TIME[i + 1] << 8 for i in range(0, 8, 2)]
    assert do(loom, [w] + words) == [w]
    assert reports(loom, 2) == [CONTROL_W << 8 | 1, CLOCK_BURST_W << 8 | 8]
    assert do(loom, [xfer(CLOCK_BURST_R)], 8) == read_words(CLOCK_BURST_R, TIME)
    ram_w, ram_r = command(7, ram=True), command(7, ram=True, read=True)
    assert do(loom, [xfer(ram_w, 0x5A)]) == [xfer(ram_w, 0x5A)]
    assert reports(loom, 3) == [CONTROL_W << 8 | 1, ram_w << 8 | 1, CONTROL_W << 8 | 1]
    assert do(loom, [xfer(ram_r)]) == read_words(ram_r, [0x5A])
    result = bus_ok(probe, 2.0)
    assert result["transfers"] == [
        (CONTROL_W, [0]), (CLOCK_BURST_W, TIME), (CLOCK_BURST_R, TIME),
        (CONTROL_W, [0]), (ram_w, [0x5A]), (CONTROL_W, [WP]), (ram_r, [0x5A])]
    assert wire.contentions == [] and bench.contention_free() and loom.badop() == 0
    assert memory(loom, RAM + 7, 1) == [0x5A]


@model_only("a second rate of the pair; the 2 V body covers the RTL")
def test_together_the_5v_rate_is_too_fast_for_the_device(backend):
    """At the host's 5 V rate (833 kHz, 400 ns high) the device cannot turn
    a read round in time: it decodes the command in software between the
    sixth fall and the eighth rise, which takes longer than the 100 clocks
    the host gives there (228 at the 2 V rate). Writes still land (they need
    no turnaround), the read comes back wrong, and nothing contends; this
    body pins the limit the program headers state."""
    bench, wire, probe, loom = together(backend, rate=0)
    w = xfer(CONTROL_W, 0x00)
    assert do(loom, [w]) == [w]
    w = xfer(command(MINUTES), 0x42)
    assert do(loom, [w]) == [w]
    assert reports(loom, 2) == [CONTROL_W << 8 | 1, command(MINUTES) << 8 | 1]
    r = command(MINUTES, read=True)
    assert do(loom, [xfer(r)]) != read_words(r, [0x42])
    assert wire.contentions == [] and bench.contention_free()


# ------------------------------------------------------------- the models
def test_model_registers_follow_the_data_sheet():
    """p.4 and p.5 on the register file alone: WP drops writes to every
    other register but not to the control register, whose bits 6:0 read 0;
    a clock burst needs all eight bytes and WP clear; a RAM burst stores
    what arrives; addresses 9..30 hold nothing; bit 7 clear is no command."""
    regs = Registers()
    assert regs.wp and regs.read(command(SECONDS, read=True)) & 0x80
    assert regs.write(command(MINUTES), [0x12]) == 0
    assert regs.write(CONTROL_W, [0x7F]) == 1 and regs.clock[CONTROL] == 0
    assert regs.write(CLOCK_BURST_W, TIME[:7]) == 0             # seven: nothing
    assert regs.write(CLOCK_BURST_W, TIME[:7] + [0xFF]) == 8
    assert regs.clock == TIME[:7] + [WP]                         # bits 6:0 forced
    assert regs.write(RAM_BURST_W, [9, 8, 7]) == 0               # WP set again
    regs.write(CONTROL_W, [0])
    assert regs.write(RAM_BURST_W, [9, 8, 7]) == 3 and regs.ram[:4] == [9, 8, 7, 0]
    assert regs.write(command(20), [0x33]) == 0 and regs.read(command(20, read=True)) == 0
    assert regs.write(0x02, [0x44]) == 0
    assert [regs.read(CLOCK_BURST_R, i) for i in range(9)] == TIME[:7] + [0, TIME[0]]
    assert regs.read(command(1, ram=True, read=True), 5) == 8    # single: retransmit


def test_model_host_drives_model_chip():
    """The two models on one set of pads, at the data sheet's 2.0 V minima:
    every figure inside p.10 and the transfers as sent."""
    from tools.protomodels.bench import Bench
    bench = Bench(image={})
    chip = bench.add(Ds1302Chip("IN0", "IN1", "BIDIR1", tcdd=38, tccz=13, tcdz=13))
    ctl = bench.add(Ds1302Host())
    probe = bench.add(Probe("IN0", "IN1", "BIDIR1", model_drives(ctl), model_drives(chip)))
    ctl.write(CONTROL_W, [0])
    ctl.write(CLOCK_BURST_W, TIME)
    ctl.read(CLOCK_BURST_R, 8)
    ctl.read(command(MINUTES, read=True), 2)
    assert bench.run_until(lambda: not ctl.busy, 100_000, every=64)
    bench.step(300)
    assert ctl.results[2:] == [(CLOCK_BURST_R, TIME), (command(MINUTES, read=True), [0x59, 0x59])]
    result = check(probe.edges, vcc=2.0)
    assert result["violations"] == [] and bench.contention_free()
    assert max(result["tCDD"]) == 760.0 and min(result["tCL"]) == 1000.0


def test_model_checker_catches_timing_faults():
    """A host too fast for 2.0 V (500 ns phases, 1 us CE times), a chip too
    slow (tCDD 900 ns), data changed 100 ns before a rise and released 100 ns
    after one, and both driving at the turnaround (a host that never
    lets go): each fault is named."""
    from tools.protomodels.bench import Bench
    bench = Bench(image={})
    chip = bench.add(Ds1302Chip("IN0", "IN1", "BIDIR1", tcdd=45))
    ctl = bench.add(Ds1302Host(low=25, high=25, hold=20, release=5, cc=50, cch=5, cwh=50))
    probe = bench.add(Probe("IN0", "IN1", "BIDIR1", model_drives(ctl), model_drives(chip)))
    ctl.read(command(SECONDS, read=True))
    assert bench.run_until(lambda: not ctl.busy, 50_000, every=16)
    bench.step(100)
    names = {v[0] for v in check(probe.edges, vcc=2.0)["violations"]}
    assert {"tCL", "tCH", "fCLK", "tCC", "tCCH", "tDC", "tCDH", "tCDD"} <= names
    assert not {"tCL", "tCH", "fCLK"} & {v[0] for v in check(probe.edges, vcc=5.0)["violations"]}
    stuck = [(c, ce, sclk, io, 1 if ce else h, d) for c, ce, sclk, io, h, d in probe.edges]
    assert "contention" in {v[0] for v in check(stuck, vcc=5.0)["violations"]}
