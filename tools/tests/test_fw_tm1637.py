"""L3-TM1637 on the model and the RTL: firmware/tm1637_controller.loom and
firmware/tm1637_device.loom, the TM1637 LED driver programs (D-035).

The controller (thread 0, CLK = BIDIR0, DIO = BIDIR1) is fed through
``tools.loomhost.Loom`` and talks to ``tools.protomodels.tm1637.Tm1637Chip``
on a ``TwoWireBus``; the device (thread 1, CLK = BIDIR2, DIO = BIDIR3) is
driven by ``Tm1637Controller``. Together, the bench joins BIDIR0 to BIDIR2
and BIDIR1 to BIDIR3 (two Loom threads never share a pin), and the two are
started at every phase of the slot grid and in both orders (BUGS 14: both
set their OD_MASK bit first).

What is expected comes from the TM1637 datasheet V2.4 (English translation)
as ``protomodels.tm1637`` cites it: the command sequences of p.4's figures,
the commands and addresses of p.4 and p.5, the key codes of p.2, the ACK of
p.3, and the limits of p.10 (clock pulse width, set-up, hold, tWAIT, the
500 kHz maximum, and p.3's 250 kHz for key reads); and from the program
headers (the host words, the result and report formats, the 200 kHz bit),
never from what the model or the RTL did. Every edge on the bus goes
through ``tm1637.check``.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import functools

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.tm1637 import (ADDRESS, KEY_CODES, NO_KEY, TIMING,
                                      Tm1637Chip, Tm1637Controller, TwoWireBus,
                                      check, frames)
from tools.tests.fw_backend import model_only

ISA = load()
CONTROLLER = REPO / "firmware" / "tm1637_controller.loom"
DEVICE = REPO / "firmware" / "tm1637_device.loom"
US = 50                           # clocks per microsecond: 50 MHz
BIT_NS = 5000                     # the controller's bit: 3 us low, 2 us high
STARTUP = 400                     # clocks for a program's set-up after RUN
OP_AUTO, OP_FIXED, OP_CTRL, OP_KEYS = 1, 2, 3, 4
NACK = 0x8000
KEY_LIMIT = TIMING["f_key_max"][0]
DIGITS = [0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D]     # "012345" on a 7-segment display


@functools.lru_cache(maxsize=None)
def program(path, text_patch=None):
    """The program assembled with ``--strict`` (``text_patch``: ``(old, new)``
    replaced once in the source first)."""
    text = path.read_text(encoding="utf-8")
    if text_patch is not None:
        old, new = text_patch
        assert text.count(old) == 1
        text = text.replace(old, new)
    return assemble_text(text, path.name, isa=ISA, strict=True)


def tm_bench(backend, bus):
    """The chip with the FIFOs, the data memory and the 512-word memory; the
    RTL has all of them and ``RtlBench`` takes none of those keywords."""
    kwargs = dict(pullups=bus.pullups)
    if backend.name == "model":
        kwargs.update(features=("FIFO", "DMEM"), imem_words=512)
    bench = backend.bench(**kwargs)
    bench.add(bus)
    return bench


def pack(data):
    """The data words of an auto write: byte 2k in bits 7:0, 2k+1 in 15:8."""
    data = list(data) + [0] * (len(data) % 2)
    return [data[i] | data[i + 1] << 8 for i in range(0, len(data), 2)]


def op_auto(address, data):
    return [OP_AUTO << 12 | len(data) << 4 | address] + pack(data)


def op_fixed(address, value):
    return [OP_FIXED << 12 | address << 8 | value]


def op_ctrl(on, width):
    return [OP_CTRL << 12 | (8 if on else 0) | width]


def op_keys():
    return [OP_KEYS << 12]


def bus_ok(bus, *, f_limit=KEY_LIMIT):
    """Everything on the bus inside the datasheet's windows, and the clock
    under the 250 kHz the key read asks for (p.3): the decoded events."""
    result = check(bus.edges, bus.clock_hz, f_limit=f_limit)
    assert result["violations"] == []
    return result


# ------------------------------------------------------------- the programs
@pytest.mark.parametrize("path,thread,words", [(CONTROLLER, 0, 172), (DEVICE, 1, 137)])
def test_tm1637_programs_assemble_strict_with_no_diagnostic(path, thread, words):
    """L2-DEADLINE: the controller's pairs proved at the 1 us tick it sets
    itself (TICK_INT 50); the device follows CLK and has no pair by design.
    The controller's main loop is in thread 0's quarter and its subroutines
    in thread 3's; the device in thread 1's and 2's; the two load together."""
    prog = program(path)
    assert prog.diagnostics == []
    report = prog.deadlines[thread]
    if path == CONTROLLER:
        assert report.period == US
        assert report.pairs and not report.infeasible and not report.unbounded
        assert report.worst_slack >= 0
        assert all(a < 0x080 or 0x180 <= a < 0x200 for a in prog.words)
    else:
        assert report.pairs == []
        assert all(0x080 <= a < 0x179 for a in prog.words)
    assert list(prog.threads) == [thread] and len(prog.words) == words
    assert not set(program(CONTROLLER).words) & set(program(DEVICE).words)


def near(values, nominal, slack=2 * 4 * 20):
    """Edges on the thread's slot grid: each 0 to 3 clocks after its deadline
    (SEMANTICS 2: 50 clocks a tick is not a multiple of 4, so they dither),
    the inside-a-byte widths within two slots of the program's nominal."""
    assert values and all(abs(v - nominal) <= slack for v in values), (
        nominal, sorted(set(values)))


# ------------------------------------------------------------- controller
def controller_setup(backend, chip=None):
    """The controller on thread 0 (BIDIR0, BIDIR1), a model TM1637 on the bus."""
    bus = TwoWireBus(["BIDIR0"], ["BIDIR1"], clock=US * 1_000_000)
    chip = bus.add(chip or Tm1637Chip())
    bench = tm_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(CONTROLLER))
    loom.run(0)
    bench.step(STARTUP)
    return bench, bus, chip, loom


def do(loom, words):
    """One operation: its words pushed, its result word popped."""
    loom.push(0, words)
    return loom.pop(0, 1, max_polls=20_000)[0]


def test_controller_writes_digits_brightness_and_reads_a_key(backend):
    """The datasheet's three programs (p.4 figures, p.6 flow chart): six
    digits in auto-increment mode from C0H, the display on at 14/16, then a
    key scan answered with SG3 and K2 (p.2: 1011_0111, 0xED). On the bus:
    exactly the transactions p.4 draws, LSB first, every byte acknowledged,
    each clock pulse, set-up, hold and wait inside p.10 and the clock at
    200 kHz, under p.3's 250 kHz."""
    key = KEY_CODES[(3, 2)]
    bench, bus, chip, loom = controller_setup(backend, Tm1637Chip(key=key))
    assert key == 0xED
    assert do(loom, op_auto(0, DIGITS)) == OP_AUTO << 12
    assert do(loom, op_ctrl(True, 7)) == OP_CTRL << 12
    assert do(loom, op_keys()) == OP_KEYS << 12 | key
    assert chip.registers == DIGITS and chip.control == 0x8F and chip.on
    result = bus_ok(bus)
    assert frames(result["events"]) == [[0x40], [ADDRESS] + DIGITS, [0x8F], [0x42, key]]
    assert all(e[2] for e in result["events"] if e[0] == "byte")
    near(result["periods"], BIT_NS)
    lows = result["lows"]                   # 3 us in a byte, longer before a STOP
    near([v for v in lows if v < 3500], 3000)
    assert min(lows) >= 3000 - 80 and len([v for v in lows if v < 3500]) >= 90
    near([v for v in result["highs"] if v < 3000], 2000)   # longer: STOP to START
    assert bench.contention_free() and loom.badop() == 0


def test_controller_fixed_address_odd_and_empty_writes(backend):
    """A fixed-address write is S 44 P, S C0+a d P (p.4); an auto write of
    three bytes uses half of its second data word; one of none sets the
    address only."""
    bench, bus, chip, loom = controller_setup(backend)
    assert do(loom, op_fixed(4, 0x77)) == OP_FIXED << 12
    assert do(loom, op_auto(1, [0x11, 0x22, 0x33])) == OP_AUTO << 12
    assert do(loom, op_auto(5, [])) == OP_AUTO << 12
    assert chip.registers == [0, 0x11, 0x22, 0x33, 0x77, 0]
    result = bus_ok(bus)
    assert frames(result["events"]) == [[0x44], [0xC4, 0x77], [0x40],
                                        [0xC1, 0x11, 0x22, 0x33], [0x40], [0xC5]]
    assert chip.address == 5 and loom.badop() == 0


def test_controller_reports_a_missing_ack(backend):
    """No chip on the bus: the first byte, 40H, is not acknowledged; the
    result says so (bit 15, operation 1, byte 1, 0x40), the STOP follows at
    once and the write's three data words are dropped, so the next word is
    taken as an operation. A chip that refuses the fourth byte it receives
    (the second data byte) is reported the same way, byte 4."""
    chip = Tm1637Chip(present=False)
    bench, bus, chip, loom = controller_setup(backend, chip)
    assert do(loom, op_auto(0, DIGITS)) == NACK | OP_AUTO << 12 | 1 << 8 | 0x40
    assert frames(check(bus.edges)["events"]) == [[0x40]]
    assert loom.read_reg(0, "INQ_CNT") == 0
    chip.present, chip.nack = True, {chip.received + 3}
    assert do(loom, op_auto(2, [0xA1, 0xB2, 0xC3])) == NACK | OP_AUTO << 12 | 4 << 8 | 0xB2
    assert chip.registers[2:4] == [0xA1, 0xB2]          # the chip took it anyway
    assert do(loom, op_ctrl(False, 3)) == OP_CTRL << 12
    assert chip.control == 0x83 and not chip.on
    assert bus_ok(bus) and loom.badop() == 0


def test_controller_rejects_an_unknown_operation(backend):
    """Operation 5 is none: bit 15 with bits 11:8 = 0xF, and nothing on the bus."""
    bench, bus, chip, loom = controller_setup(backend)
    assert do(loom, [0x5123]) == NACK | 5 << 12 | 0xF << 8
    assert bus.edges == [] and loom.badop() == 0


# ------------------------------------------------------------------ device
def device_setup(backend, keys=(), **controller):
    """The device on thread 1 (BIDIR2, BIDIR3), a model controller on the bus."""
    bus = TwoWireBus(["BIDIR2"], ["BIDIR3"], clock=US * 1_000_000)
    ctl = bus.add(Tm1637Controller(**controller))
    bench = tm_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(DEVICE))
    if keys:
        loom.push(1, list(keys))
    loom.run(1)
    bench.step(STARTUP)
    return bench, bus, ctl, loom


def reports(loom, stops):
    """The device's four words after each of ``stops`` STOPs, unpacked:
    ``(registers, control byte, address)`` each."""
    out = []
    for _ in range(stops):
        w = loom.pop(1, 4, max_polls=20_000)
        regs = [w[i // 2] >> (8 * (i % 2)) & 0xFF for i in range(6)]
        out.append((regs, w[3] & 0xFF, w[3] >> 8 & 0x3F))
        assert w[3] >> 14 == 0
    return out


def finish(bench, ctl, max_cycles=200_000):
    assert bench.run_until(lambda: not ctl.busy, max_cycles, every=64)


def answer_times(bus, pin="BIDIR3"):
    """CLK fall to the device's pull of DIO, in ns, for every pull it starts."""
    falls = [c for (c, clk, _), (_, pclk, _) in zip(bus.edges[1:], bus.edges) if pclk and not clk]
    out = []
    for start, _ in bus.intervals(bus.chip_name(pin), "dio"):
        before = [f for f in falls if f <= start]
        out.append(bus.ns(start - before[-1]))
    return out


def test_device_takes_digits_brightness_and_answers_a_key(backend):
    """The p.4 auto-increment program from a model controller at 200 kHz:
    S 40 P, S C0 d0..d5 P, S 8B P, then S 42 and the key byte read back.
    After each STOP the device reports its registers (zero at start), the
    control byte (80H at start) and the address. Every byte is acknowledged;
    the key byte is the host's (SG8 with K1: 0000_1111, 0xF0); the device
    answers each CLK fall within 1 us, inside the controller's 3 us low."""
    key = KEY_CODES[(8, 1)]
    bench, bus, ctl, loom = device_setup(backend, keys=[key])
    ctl.write_auto(0, DIGITS)
    ctl.display(True, 3)
    ctl.read_keys()
    got = reports(loom, 4)
    finish(bench, ctl)
    assert got == [([0] * 6, 0x80, 0), (DIGITS, 0x80, 6), (DIGITS, 0x8B, 6),
                   (DIGITS, 0x8B, 6)]
    assert key == 0xF0
    assert ctl.results == [("ack", True)] * 10 + [("read", key, True)]
    result = bus_ok(bus)
    assert frames(result["events"]) == [[0x40], [ADDRESS] + DIGITS, [0x8B], [0x42, key]]
    assert max(answer_times(bus)) < 1000
    assert bench.contention_free() and loom.badop() == 0


def test_device_fixed_mode_and_invalid_addresses(backend):
    """Fixed-address mode (44H) writes each byte to the one register; C6H
    and C8H are not addresses (p.5: data ignored until a valid one, and
    p.4's 'Zero should be inserted for irrelevant items'); an auto write that
    runs past C5H drops the rest. Every byte is still acknowledged."""
    bench, bus, ctl, loom = device_setup(backend)
    ctl.transaction(0x44)
    ctl.transaction(ADDRESS + 2, 0x12, 0x34)
    ctl.transaction(ADDRESS + 6, 0x56)
    ctl.transaction(ADDRESS + 8, 0x78)
    ctl.transaction(0x40)
    ctl.transaction(ADDRESS + 4, 0x9A, 0xBC, 0xDE)
    got = reports(loom, 6)
    finish(bench, ctl)
    regs = [0, 0, 0x34, 0, 0, 0]
    assert [g[0] for g in got[1:4]] == [regs] * 3
    assert [g[2] for g in got] == [0, 2, 6, 8, 8, 6]     # stops at C6H
    assert got[5][0] == [0, 0, 0x34, 0, 0x9A, 0xBC]
    assert all(r == ("ack", True) for r in ctl.results) and len(ctl.results) == 13
    assert bus_ok(bus) and loom.badop() == 0


@pytest.mark.parametrize("then", ["stop", "start"])
def test_device_drops_a_byte_cut_by_a_stray_condition(backend, then):
    """p.4: a STOP in mid-byte makes the byte being sent invalid and keeps
    what came before. S C0 d0 then four bits of d1 and a STOP (or a START):
    d0 is written, d1 is not, the device reports at the STOP, and the next
    transaction, at the same address counter, is taken normally."""
    bench, bus, ctl, loom = device_setup(backend)
    ctl.start()
    ctl.byte(ADDRESS)
    ctl.byte(0x11)
    ctl.cut(0x22, 4, then)
    if then == "start":
        ctl.byte(ADDRESS + 3)                   # the START begins a new command
        ctl.byte(0x33)
        ctl.stop()
    ctl.transaction(0x8F)
    got = reports(loom, 2)
    finish(bench, ctl)
    if then == "stop":
        assert got[0] == ([0x11, 0, 0, 0, 0, 0], 0x80, 1)
    else:
        assert got[0] == ([0x11, 0, 0, 0x33, 0, 0], 0x80, 4)
    assert got[-1][1] == 0x8F
    result = check(bus.edges)
    assert [v[0] for v in result["violations"]] == ["cut"]
    assert [e for e in result["events"] if e[0] == "cut"][0][1] == 4
    assert loom.badop() == 0


def test_device_takes_the_newest_key_byte(backend):
    """The host's key bytes are taken at each key scan, the newest winning:
    no key (0xFF) before any is given, then the last of two pushed."""
    bench, bus, ctl, loom = device_setup(backend)
    ctl.read_keys()
    reports(loom, 1)
    finish(bench, ctl)
    loom.push(1, [KEY_CODES[(1, 1)], KEY_CODES[(2, 2)]])
    ctl.read_keys()
    ctl.read_keys()
    reports(loom, 2)
    finish(bench, ctl)
    assert [r[1] for r in ctl.results if r[0] == "read"] == [NO_KEY] + [KEY_CODES[(2, 2)]] * 2
    assert all(r[-1] for r in ctl.results)
    assert bus_ok(bus) and loom.badop() == 0


@model_only("a rate sweep; the 200 kHz bodies cover the RTL")
@pytest.mark.parametrize("low,high", [(110, 90), (50, 50)])
def test_device_follows_faster_controllers(backend, low, high):
    """At 250 kHz (the key-read limit, p.3) the device writes and answers a
    key scan; at 500 kHz, the datasheet's maximum (p.10), with 1 us half
    periods, it writes."""
    bench, bus, ctl, loom = device_setup(backend, keys=[0xFE], low=low, high=high, hold=5)
    ctl.write_auto(0, DIGITS[:4])
    if low + high >= 200:
        ctl.read_keys()
    got = reports(loom, 3 if low + high >= 200 else 2)
    finish(bench, ctl)
    assert got[1][0][:4] == DIGITS[:4]
    assert all(r[-1] for r in ctl.results)
    if low + high >= 200:
        assert ctl.results[-1] == ("read", 0xFE, True)
    result = check(bus.edges, bus.clock_hz)
    assert result["violations"] == [] and loom.badop() == 0


# --------------------------------------------------------------- together
def together(backend, start, controller=CONTROLLER, device=DEVICE):
    """Controller (thread 0, BIDIR0 and 1) and device (thread 1, BIDIR2 and
    3), the pads joined by the bench as one bus, started by ``start(bench,
    loom)``."""
    bus = TwoWireBus(["BIDIR0", "BIDIR2"], ["BIDIR1", "BIDIR3"], clock=US * 1_000_000)
    bench = tm_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(*controller).words if isinstance(controller, tuple)
                 else program(controller).words)
    dev = program(*device).words if isinstance(device, tuple) else program(device).words
    assert not set(words) & set(dev)
    words.update(dev)
    loom.load(words)
    start(bench, loom)
    bench.step(STARTUP)
    return bench, bus, loom


def write_and_read_together(bench, bus, loom, digits, key):
    """Four digits from C1H, the display on at 10/16, a key scan: each
    result word right, the device's reports after each STOP, the bus inside
    the datasheet's windows and no contention."""
    loom.push(1, [key])
    loom.push(0, op_auto(1, digits))
    got = reports(loom, 2)
    assert loom.pop(0, 1, max_polls=20_000) == [OP_AUTO << 12]
    loom.push(0, op_ctrl(True, 3) + op_keys())
    got += reports(loom, 2)
    assert loom.pop(0, 2, max_polls=20_000) == [OP_CTRL << 12, OP_KEYS << 12 | key]
    regs = [0] + list(digits) + [0] * (5 - len(digits))
    assert got == [([0] * 6, 0x80, 0), (regs, 0x80, 1 + len(digits)),
                   (regs, 0x8B, 1 + len(digits)), (regs, 0x8B, 1 + len(digits))]
    result = bus_ok(bus)
    assert frames(result["events"]) == [[0x40], [ADDRESS + 1] + list(digits), [0x8B],
                                        [0x42, key]]
    assert all(e[2] for e in result["events"] if e[0] == "byte")
    assert bench.contention_free() and loom.badop() == 0


def test_controller_and_device_together(backend):
    """The two programs under one RUN on one bus: the controller writes
    "0123" from C1H, turns the display on and reads the device's key byte
    (SG5 with K2, 0xEB), every byte acknowledged by the Loom device. The
    PHY-GL-SDF candidate: 114000 clocks on the RTL, host transfers included."""
    bench, bus, loom = together(backend, lambda bench, loom: loom.run([0, 1]))
    write_and_read_together(bench, bus, loom, DIGITS[:4], KEY_CODES[(5, 2)])


def _phase(p, order):
    def start(bench, loom):
        bench.step(p)
        if order == "one_run":
            loom.run([0, 1])
        else:
            first, second = (0, 1) if order == "controller_first" else (1, 0)
            loom.run(first)
            bench.step(p)
            loom.run(second)
    return start


@model_only("twelve starts, about 30000 clocks each")
@pytest.mark.parametrize("order", ["one_run", "controller_first", "device_first"])
@pytest.mark.parametrize("p", [0, 1, 2, 3])
def test_together_at_every_start_phase(backend, p, order):
    """BUGS 14: both programs set their OD_MASK bits first. Under one RUN at
    each of the four phases of the slot grid, and each started p clocks
    after the other in both orders, the transfer is right: the read-back and
    retry in both headers keep every bit."""
    bench, bus, loom = together(backend, _phase(p, order))
    write_and_read_together(bench, bus, loom, [0x7F, 0x6F], KEY_CODES[(7, 1)])


_NO_RETRY = {
    CONTROLLER: ("        BNZ     start                   ; lost to the other thread's write\n",
                 "        NOP                             ; (removed)\n"),
    DEVICE: ("        BZ      start                   ; lost to the other thread's write\n",
             "        NOP                             ; (removed)\n"),
}


@model_only("the race itself, without the fix")
@pytest.mark.parametrize("p", [0, 1, 2, 3])
def test_without_the_read_back_one_run_loses_a_bit(backend, p):
    """Why the read-back is there (BUGS 14): the same two programs with the
    retry branch replaced by a NOP, under one RUN. The two plain
    read-modify-writes overlap and the controller's write, last, drops the
    device's DIO bit (OD_MASK bit 3). The device's pad then never drives
    (its PIN_OE bit stays 0), so the controller's first byte is not
    acknowledged and it says so."""
    bench, bus, loom = together(backend, _phase(p, "one_run"),
                                (CONTROLLER, _NO_RETRY[CONTROLLER]),
                                (DEVICE, _NO_RETRY[DEVICE]))
    assert bench.machine.od_mask & 0x0F == 0b0011
    assert do(loom, op_ctrl(True, 0)) == NACK | OP_CTRL << 12 | 1 << 8 | 0x88
    assert bus.intervals(bus.chip_name("BIDIR3"), "dio") == []


# ------------------------------------------------- the reference models alone
def test_model_key_codes_and_limits():
    """p.2's table read in wire order, bit 0 first: SG1 with K1 is 0xF7 as
    the datasheet says, no key is 0xFF, and the sixteen codes differ. Every
    limit names its source."""
    assert KEY_CODES[(1, 1)] == 0xF7 and NO_KEY == 0xFF
    assert len(set(KEY_CODES.values())) == 16 and NO_KEY not in KEY_CODES.values()
    assert all(src.startswith("p.") for _, src in TIMING.values())


def test_model_controller_drives_model_chip():
    """The reference controller against the reference chip, no Loom: p.4's
    sequences land in the registers, the key comes back, the bus is clean;
    a stray STOP in mid-byte is reported by both the chip and the checker,
    and a refused byte reads as no ACK."""
    bus = TwoWireBus()
    chip = bus.add(Tm1637Chip(key=KEY_CODES[(4, 1)], nack={9}))
    ctl = bus.add(Tm1637Controller())
    ctl.write_auto(0, DIGITS)
    ctl.write_fixed(3, 0x40)
    ctl.display(True, 1)
    ctl.read_keys()
    ctl.start()
    ctl.byte(ADDRESS)
    ctl.cut(0x55, 5)
    assert bus.run_alone(lambda: not ctl.busy, 2_000_000)
    assert chip.registers == DIGITS[:3] + [0x40] + DIGITS[4:] and chip.control == 0x89
    acks = [r[-1] for r in ctl.results]
    assert acks.count(False) == 1 and acks.index(False) == 9
    assert ctl.results[-2] == ("read", KEY_CODES[(4, 1)], True)
    assert [e[1] for e in chip.log if e[0] == "cut"] == [5]
    result = check(bus.edges, bus.clock_hz, f_limit=KEY_LIMIT)
    assert [v[0] for v in result["violations"]] == ["cut"]


def test_model_checker_catches_timing_faults():
    """A controller whose clock pulses are 300 ns (under PWCLK) and whose
    data changes 40 ns after the fall (under tHOLD) is caught on every count,
    and 1.67 MHz is over the 500 kHz maximum."""
    bus = TwoWireBus()
    bus.add(Tm1637Chip(delay=1))
    ctl = bus.add(Tm1637Controller(low=15, high=15, hold=2))
    ctl.transaction(0x40)
    assert bus.run_alone(lambda: not ctl.busy, 100_000)
    kinds = {v[0] for v in check(bus.edges, bus.clock_hz)["violations"]}
    assert {"pw_low", "pw_high", "hold", "period"} <= kinds
