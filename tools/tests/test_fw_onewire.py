"""L3-OW on the model and the RTL: firmware/onewire_master.loom and
firmware/onewire_device.loom, the 1-Wire programs of D-035.

The master (thread 0, OW = BIDIR0) is fed through ``tools.loomhost.Loom``
and tested against ``tools.protomodels.onewire.Ds18b20`` devices on a
``OneWireBus``; the device (thread 1, OW = BIDIR1) is tested against
``OneWireMaster``, a master with application note 126's delays and note
187's search. Together, the master on BIDIR0 and the device on BIDIR1 are
one bus: the bench joins the two pads, as two Loom threads never share a
pin, and the master finds the device and reads its scratchpad.

What is expected comes from AN126 (table 2, standard speed), AN187 (the
search and its order) and the DS18B20 datasheet (ROM and function
commands, the scratchpad and its CRC, the AC timing), and from the program
headers (the host words), never from what the model or the RTL did. Every
low pulse the chip's master makes is measured on its pad and held to
AN126's values to the clock and to the datasheet's windows; the device
models check each write slot at both ends of the datasheet's sampling
window and hold a read 0 for exactly the 15 us the datasheet guarantees,
so a late sample reads a 1; the model master checks every bit Loom's
device drives and its presence pulse.

1-Wire at standard speed is slow for a simulator: a reset is 48000 clocks,
a byte 28000, a search pass 750000, and the golden model runs about 16000
clocks a second with these programs, Icarus about 6400. So:

* at the real rate (``US = 50`` clocks per microsecond, 50 MHz) run the
  reset and presence, READ ROM, the master's fault cases (whose read slots
  are the search's) and the device at the edges of the datasheet's windows;
  the RTL set is four of these: the master's reset with a device and its
  READ ROM, the device's presence pulse and its READ ROM;
* the searches, the scratchpad scenarios and the two programs together run
  on a time-scaled build, ``US = 10``: the same source with that one
  ``.equ`` changed, so every interval is the same number of ticks and one
  fifth as many clocks (the deadline checker proves it down to 10 clocks a
  tick), against models clocked at 10 MHz. They check the protocol and
  the order; the timing to the clock is the real-rate scenarios' job.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import functools

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.onewire import (AN126, DS18B20, MATCH_ROM, READ_ROM,
                                       READ_SCRATCHPAD, SKIP_ROM, WRITE_SCRATCHPAD,
                                       Ds18b20, OneWireBus, OneWireMaster,
                                       check_master_slots, crc8, make_rom, master_slots,
                                       rom_words, search_order)
from tools.tests.fw_backend import model_only

ISA = load()
MASTER = REPO / "firmware" / "onewire_master.loom"
DEVICE = REPO / "firmware" / "onewire_device.loom"
US = 50                           # clocks per microsecond: 50 MHz, the programs' rate
FAST = 10                         # the time-scaled build (module doc)
RESET, WRITE, READ, SEARCH = 0x100, 0x200, 0x300, 0x400        # master commands
PRESENCE, SHORT = 1, 2                                        # RESET result
END, NO_PRESENCE, BUS_SHORT, NO_DEVICE = 0x8000, 2, 4, 8      # SEARCH end marker
STARTUP = 400                     # clocks for a program's set-up after RUN
SCALED = ("a time-scaled build (US = 10, module doc): at the real rate this is "
          "%s, minutes of simulation")


@functools.lru_cache(maxsize=None)
def program(path, us=US):
    """The program assembled with ``--strict``, for ``us`` clocks a microsecond."""
    text = path.read_text(encoding="utf-8")
    line = ".equ    US = 50 "
    assert text.count(line) == 1
    return assemble_text(text.replace(line, ".equ    US = %d " % us), path.name,
                         isa=ISA, strict=True)


def ow_bench(backend, bus, *, dmem=False):
    """The chip with the FIFOs, the bit engine, the deadline latch and the
    512-word memory (plus slice B for the device); the RTL has all of them
    and ``RtlBench`` takes none of those keywords."""
    kwargs = dict(pullups=bus.pullups)
    if backend.name == "model":
        features = ("FIFO", "BE", "SETPD") + (("DMEM",) if dmem else ())
        kwargs.update(features=features, imem_words=512)
    bench = backend.bench(**kwargs)
    bench.add(bus)
    return bench


def ds(serial_or_rom, us=US, **kwargs):
    """A model DS18B20 clocked like the chip at ``us`` clocks a microsecond."""
    rom = serial_or_rom if isinstance(serial_or_rom, bytes) else make_rom(serial_or_rom)
    return Ds18b20(rom, clock=us * 1_000_000, **kwargs)


# ---------------------------------------------------------------- master
def master_setup(backend, devices=(), us=US):
    bus = OneWireBus(["BIDIR0"], clock=us * 1_000_000)
    for dev in devices:
        bus.add(dev)
    bench = ow_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(MASTER, us))
    loom.run(0)
    bench.step(STARTUP)
    return bench, bus, loom


def m_reset(loom):
    loom.push(0, [RESET])
    return loom.pop(0, 1)[0]


def m_write(loom, data):
    loom.push(0, [WRITE | b for b in data])


def m_read(loom, n):
    """``n`` READ results: ``(bytes, the CRC field of each)``."""
    out = []
    for i in range(0, n, 4):
        k = min(4, n - i)
        loom.push(0, [READ] * k)
        out += loom.pop(0, k)
    return bytes(w & 0xFF for w in out), [w >> 8 for w in out]


def m_search(loom):
    """Every report of one SEARCH: ``([(status, ID bytes)], end marker)``."""
    loom.push(0, [SEARCH])
    found = []
    while True:
        status = loom.pop(0, 1)[0]
        if status & END:
            return found, status
        words = loom.pop(0, 4)
        rom = sum(w << (16 * i) for i, w in enumerate(words)).to_bytes(8, "little")
        found.append((status, rom))


def chip_slots(bus, pin="BIDIR0"):
    return master_slots(bus.intervals(bus.chip_name(pin)), clock=bus.clock_hz)


def check_an126(slots):
    """The master's pulses against AN126 table 2 to the clock (every edge is
    deadline-latched): a write-1 or read low of A, a write-0 low of C, a
    reset low of H; slots inside a byte A + B = C + D apart; the first slot
    after a reset no sooner than I + J after its release; and the
    datasheet's windows for all of them (:func:`check_master_slots`)."""
    for s in slots:
        want = {"short": AN126["A"], "write0": AN126["C"], "reset": AN126["H"]}[s["kind"]]
        assert s["low"] == want, s
    periods = [s["period"] for i, s in enumerate(slots)
               if i and slots[i - 1]["kind"] != "reset"]
    if periods:
        assert min(periods) == AN126["A"] + AN126["B"] == AN126["C"] + AN126["D"]
    after_reset = [s["gap"] for i, s in enumerate(slots) if i and slots[i - 1]["kind"] == "reset"]
    assert all(g >= AN126["I"] + AN126["J"] for g in after_reset), after_reset
    assert check_master_slots(slots) == []


class Stuck(Model):
    """A short to ground on the line."""

    def __init__(self, pin):
        self.pad = pad_of(pin)

    def drive(self, drive, cycle):
        drive.pull_low(self.pad)


class PresenceOnly(Ds18b20):
    """A part that answers the reset and nothing after it (it has left the
    bus, or is not a 1-Wire slave at all): a search reads 1 and 1."""

    def _rom_command(self, byte):
        self._idle()


# ------------------------------------------------------- master, the program
def test_onewire_master_assembles_strict_with_no_diagnostic():
    """L2-DEADLINE for onewire_master.loom: every pair proved at the tick it
    sets itself (TICK_INT 50, 1 us), and in the time-scaled build at 10;
    nothing unbounded, no ``.bounded``. The main loop and the search fill
    thread 0's quarter; the three slot subroutines sit at 0x100, so thread
    2 must not be started with it."""
    for us in (US, FAST):
        prog = program(MASTER, us)
        assert prog.diagnostics == []
        report = prog.deadlines[0]
        assert report.period == us
        assert report.pairs and not report.infeasible and not report.unbounded
        assert report.worst_slack >= 0
    assert list(prog.threads) == [0]
    main = [a for a in prog.words if a < 0x80]
    subs = [a for a in prog.words if a >= 0x80]
    assert len(prog.words) == 157 and len(main) == 124
    assert min(subs) == 0x100 and max(subs) < 0x180
    assert sorted(prog.unstartable) == [2]


# ---------------------------------------------- master at the real rate
def test_master_reset_and_presence(backend):
    """One 480 us reset pulse, exact to the clock; RESET answers PRESENCE
    (a device pulled the line 30 us after the release, for 120 us, so it is
    low at the 70 us sample) and not SHORT. The PHY-GL-SDF candidate: about
    50000 clocks, one latched pulse and one sample on the pad."""
    dev = ds(0x77)
    bench, bus, loom = master_setup(backend, [dev])
    assert m_reset(loom) == PRESENCE
    slots = chip_slots(bus)
    assert [s["kind"] for s in slots] == ["reset"] and slots[0]["low"] == AN126["H"]
    assert dev.events("reset") and dev.violations == []
    assert bench.contention_free() and loom.badop() == 0


@model_only("the same pulse and sample as the case with a device, which runs on "
            "the RTL")
def test_master_reset_with_no_device(backend):
    """Nothing on the line: RESET answers 0, no presence and no short."""
    bench, bus, loom = master_setup(backend)
    assert m_reset(loom) == 0
    assert [s["kind"] for s in chip_slots(bus)] == ["reset"]


def test_master_read_rom_of_one_device(backend):
    """RESET, WRITE 0x33, eight READs: the ROM, and in each result the CRC
    of the bytes read so far, 0 after the eighth. Every pulse on the pad is
    AN126's to the clock, and the device saw every write bit stable over its
    whole 15..60 us window."""
    rom = make_rom(0x0123456789AB)
    dev = ds(rom)
    bench, bus, loom = master_setup(backend, [dev])
    assert m_reset(loom) == PRESENCE
    m_write(loom, [READ_ROM])
    data, crcs = m_read(loom, 8)
    assert data == rom
    assert crcs == [crc8(rom[:i + 1]) for i in range(8)] and crcs[-1] == 0
    assert [e[2] for e in dev.events("rom_command")] == [READ_ROM]
    assert dev.violations == []
    slots = chip_slots(bus)
    assert [s["kind"] for s in slots].count("reset") == 1 and len(slots) == 1 + 8 + 64
    check_an126(slots)
    assert bench.contention_free() and loom.badop() == 0


@model_only("fault cases, 300000 clocks; their read slots are the search's, at the "
            "real rate")
def test_master_search_faults(backend):
    """SEARCH with nothing on the line: END and NO PRESENCE. With a part that
    gives a presence pulse and never answers: the first bit and its
    complement both read 1, END and NO DEVICE (these are the search's own
    read slots, measured at the real rate). With the line shorted: RESET
    sees PRESENCE and SHORT, SEARCH ends at once with END and SHORT."""
    bench, bus, loom = master_setup(backend)
    assert m_search(loom) == ([], END | NO_PRESENCE)
    bus.add(PresenceOnly(make_rom(0x42)))
    assert m_search(loom) == ([], END | NO_DEVICE)
    check_an126(chip_slots(bus))
    bench.add(Stuck("BIDIR0"))
    assert m_reset(loom) == PRESENCE | SHORT
    assert m_search(loom) == ([], END | BUS_SHORT)


# ----------------------------------------- master on the time-scaled build
@model_only(SCALED % "750000 clocks a device")
@pytest.mark.parametrize("n", [1, 2, 3, 8])
def test_master_search_finds_every_device_in_an187_order(backend, n):
    """SEARCH ROM over 1, 2, 3 and 8 devices: every ID, each with a good CRC
    (status 0), in the order AN187's search finds them, then the end marker.
    The eight include IDs that part at their first serial bit and two that
    part only at the last (0x000000000001 and 0x800000000001), so the
    search branches early and late."""
    serials = [0x000000000001, 0x000000000002, 0x000000000003, 0x800000000000,
               0x7FFFFFFFFFFF, 0x0000000000FE, 0x800000000001, 0x123456789ABC][:n]
    devs = [ds(s, FAST, pd_wait_us=15 + 5 * i, pd_low_us=60 + 20 * i)
            for i, s in enumerate(serials)]
    bench, bus, loom = master_setup(backend, devs, FAST)
    found, end = m_search(loom)
    assert [rom for _, rom in found] == search_order(d.rom for d in devs)
    assert [status for status, _ in found] == [0] * n and end == END
    assert all(dev.violations == [] for dev in devs)
    check_an126(chip_slots(bus))


@model_only(SCALED % "1.5 million clocks")
def test_master_search_reports_a_corrupt_rom_crc(backend):
    """A device whose ROM ends in a wrong CRC byte: the search reports its ID
    with the CRC the CRC unit computed over all 64 bits (Maxim's CRC of the
    ID, not 0) and stops there, as AN187 does; a good device that comes
    before it in the search order is reported first, with status 0."""
    good = make_rom(0x000000000002)                 # its first serial bit is 0 ...
    body = make_rom(0x000000000001)                 # ... this one's is 1
    bad = body[:7] + bytes([body[7] ^ 0x5A])
    assert crc8(bad) != 0 and search_order([bad, good]) == [good, bad]
    bench, bus, loom = master_setup(backend, [ds(bad, FAST), ds(good, FAST)], FAST)
    found, end = m_search(loom)
    assert found == [(0, good), (crc8(bad), bad)] and end == END


@model_only(SCALED % "330000 clocks a case")
@pytest.mark.parametrize("corrupt", [False, True])
def test_master_reads_a_scratchpad_and_checks_its_crc(backend, corrupt):
    """RESET, SKIP ROM, READ SCRATCHPAD and nine READs: the DS18B20's nine
    bytes (here +25.0625 C, the datasheet's example), the CRC field 0 after
    the ninth exactly when the scratchpad's CRC byte is right."""
    pad = bytes([0x91, 0x01, 0x4B, 0x46, 0x7F, 0xFF, 0x0F, 0x10])
    dev = ds(0xC0FFEE, FAST, scratchpad=pad,
             scratchpad_crc=crc8(pad) ^ 0x81 if corrupt else None)
    bench, bus, loom = master_setup(backend, [dev], FAST)
    assert m_reset(loom) == PRESENCE
    m_write(loom, [SKIP_ROM, READ_SCRATCHPAD])
    data, crcs = m_read(loom, 9)
    assert data == dev.scratchpad_bytes()
    assert crcs[7] == crc8(pad)                     # the CRC of bytes 0..7
    assert (crcs[8] != 0) == corrupt and crcs[8] == crc8(data)
    assert dev.violations == []
    check_an126(chip_slots(bus))


# ---------------------------------------------------------------- device
DEV_ROM = make_rom(0x00A1B2C3D4E5)                       # family 0x28
DEV_PAD = bytes([0x91, 0x01, 0x4B, 0x46, 0x7F, 0xFF, 0x0F, 0x10])   # +25.0625 C


def pad_words(pad):
    """Scratchpad bytes 0..7 as the program takes them: byte 2k in 7:0."""
    return [pad[2 * k] | (pad[2 * k + 1] << 8) for k in range(4)]


def device_setup(backend, rom=DEV_ROM, pad=DEV_PAD, others=(), us=US, **master_timing):
    """The device program on thread 1 (BIDIR1), given its ROM ID and
    scratchpad, with a model master and any model devices on the line."""
    bus = OneWireBus(["BIDIR1"], clock=us * 1_000_000)
    for dev in others:
        bus.add(dev)
    master = bus.add(OneWireMaster(clock=us * 1_000_000, **master_timing))
    bench = ow_bench(backend, bus, dmem=True)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(DEVICE, us))
    loom.run(1)
    loom.push(1, rom_words(rom) + pad_words(pad))
    bench.step(STARTUP)
    return bench, bus, master, loom


def finish(bench, tr, budget):
    assert bench.run_until(lambda: tr.done, budget, every=500), "not done: %r" % (tr,)
    return tr


def device_holds(bus, master_name="master", pin="BIDIR1"):
    """For every low the chip drove in a read slot: ``(start, end)`` in us
    after the master's fall (the presence pulses, which start long after a
    fall, are left out)."""
    falls = [s for s, _ in bus.intervals(master_name)]
    out = []
    for start, end in bus.intervals(bus.chip_name(pin)):
        before = [f for f in falls if f <= start]
        if before and start - before[-1] < 15 * bus.us:
            out.append(((start - before[-1]) / bus.us, (end - before[-1]) / bus.us))
    return out


def check_device_timing(bus, master, zeros):
    """The presence pulse inside the datasheet's 15..60 and 60..240 us, and
    each of the ``zeros`` 0s the device sent driven before the master let
    go of the line (AN126's A, 6 us; the model master also checks that the
    line never rose before its sample) and held past the datasheet's 15 us,
    released before the 60 us slot ends."""
    assert master.violations == []
    for high, low in master.presence_pulses:
        assert DS18B20["t_pdhigh"][0] <= high <= DS18B20["t_pdhigh"][1], high
        assert DS18B20["t_pdlow"][0] <= low <= DS18B20["t_pdlow"][1], low
    holds = device_holds(bus)
    assert len(holds) == zeros
    for start, end in holds:
        assert start < AN126["A"] and DS18B20["t_rdv"] < end < DS18B20["t_slot"][0], (start, end)


def zeros_in(data):
    return sum(8 - bin(b).count("1") for b in data)


def test_onewire_device_assembles_strict_with_no_diagnostic():
    """L2-DEADLINE for onewire_device.loom: the device times only from the
    master's falls (SETD 0, then WAITD), so its pairs are short and proved
    with a wide margin, at 50 and at 10 clocks a tick; nothing unbounded.
    Main loop and data in thread 1's quarter, the slot subroutines in thread
    2's after the master's, so the two programs load together."""
    for us in (US, FAST):
        prog = program(DEVICE, us)
        assert prog.diagnostics == []
        report = prog.deadlines[1]
        assert report.period == us
        assert report.pairs and not report.infeasible and not report.unbounded
    assert list(prog.threads) == [1]
    main = [a for a in prog.words if 0x80 <= a < 0xF8]
    data = [a for a in prog.words if 0xF8 <= a < 0x100]
    subs = [a for a in prog.words if a >= 0x100]
    assert len(main) + len(data) + len(subs) == len(prog.words) == 157
    assert data == list(range(0xF8, 0x100)) and min(subs) >= 0x140 and max(subs) < 0x180
    assert not set(program(MASTER).words) & set(prog.words)


# ---------------------------------------------- device at the real rate
def test_device_presence_pulse(backend):
    """A reset alone: the presence pulse 15..60 us after the reset's rise,
    60..240 us long. The device's PHY-GL-SDF candidate: about 50000 clocks."""
    bench, bus, master, loom = device_setup(backend)
    tr = finish(bench, master.transaction([("reset",)]), 100_000)
    assert tr.presence == [True] and len(master.presence_pulses) == 1
    check_device_timing(bus, master, 0)
    assert bench.contention_free() and loom.badop() == 0


def test_device_presence_and_read_rom(backend):
    """A reset, the presence pulse, READ ROM: the master reads the ID the
    host gave, every 0 driven within the master's low and held 15..60 us."""
    bench, bus, master, loom = device_setup(backend)
    tr = finish(bench, master.read_rom(), 400_000)
    assert tr.presence == [True] and bytes(tr.data) == DEV_ROM
    check_device_timing(bus, master, zeros_in(DEV_ROM))
    assert bench.contention_free() and loom.badop() == 0


@model_only("READ ROM again, 350000 clocks, at the edges of the datasheet's windows")
def test_device_at_the_edges_of_the_slot_windows(backend):
    """A master at the datasheet's limits: a write-1 low of 14 us (under the
    15 us maximum, so the device must sample after it), write-0 lows of 60
    us (so it must sample before 60) and read slots of a 1 us low sampled 15
    us after the fall (so the device's 0 must start within 1 us and last 15
    us)."""
    bench, bus, master, loom = device_setup(backend, write1_low_us=14, read_low_us=1,
                                            read_sample_us=15)
    tr = finish(bench, master.read_rom(), 400_000)
    assert bytes(tr.data) == DEV_ROM
    check_device_timing(bus, master, zeros_in(DEV_ROM))


# ----------------------------------------- device on the time-scaled build
@model_only(SCALED % "1.1 million clocks")
def test_device_match_rom_answers_only_its_own_id(backend):
    """MATCH ROM with the device's ID, then READ SCRATCHPAD: the nine bytes.
    MATCH ROM with another ID: the device stays silent (it drives nothing
    after its presence pulse) and the master reads all 1s."""
    bench, bus, master, loom = device_setup(backend, us=FAST)
    tr = finish(bench, master.match_rom(DEV_ROM, [("write", [READ_SCRATCHPAD]), ("read", 9)]),
                300_000)
    assert bytes(tr.data) == DEV_PAD + bytes([crc8(DEV_PAD)])
    before = len(bus.intervals(bus.chip_name("BIDIR1")))
    other = make_rom(0x00A1B2C3D4E4)
    tr = finish(bench, master.match_rom(other, [("write", [READ_SCRATCHPAD]), ("read", 9)]),
                300_000)
    assert tr.presence == [True] and bytes(tr.data) == b"\xFF" * 9
    assert len(bus.intervals(bus.chip_name("BIDIR1"))) == before + 1      # the presence only
    assert master.violations == []


@model_only(SCALED % "2.3 million clocks")
def test_device_is_found_by_search_among_model_devices(backend):
    """AN187's search (the model master's) over the Loom device and two
    DS18B20 models finds all three in AN187's order: the device sends each
    bit and its complement and drops out when the master's choice differs.
    One model differs from the device only in the top serial bit, so the
    two part at the 56th bit of the search; the other parts at the 9th."""
    others = [ds(s, FAST) for s in (0x00A1B2C3D4E5 ^ (1 << 47), 0x5A)]
    bench, bus, master, loom = device_setup(backend, others=others, us=FAST)
    tr = finish(bench, master.search(), 800_000)
    assert tr.search_error is None
    assert tr.roms == search_order([DEV_ROM] + [d.rom for d in others])
    assert master.violations == [] and all(d.violations == [] for d in others)


PADS = {"25C": DEV_PAD, "power_up": bytes([0x50, 0x05, 0x4B, 0x46, 0x7F, 0xFF, 0x0C, 0x10])}


@model_only(SCALED % "360000 clocks a case")
@pytest.mark.parametrize("which", sorted(PADS))
def test_device_read_scratchpad_ends_in_its_crc(backend, which):
    """SKIP ROM, READ SCRATCHPAD: bytes 0..7 as the host gave them and their
    CRC-8, computed by the program on the CRC unit, as byte 8 (here the
    datasheet's +25.0625 C and the power-up +85 C scratchpads)."""
    pad = PADS[which]
    bench, bus, master, loom = device_setup(backend, pad=pad, us=FAST)
    tr = finish(bench, master.skip_rom([("write", [READ_SCRATCHPAD]), ("read", 9)]), 200_000)
    assert bytes(tr.data) == pad + bytes([crc8(pad)]) and crc8(tr.data) == 0
    check_device_timing(bus, master, zeros_in(tr.data))


@model_only(SCALED % "700000 clocks")
def test_device_write_scratchpad_reaches_the_host(backend):
    """SKIP ROM, WRITE SCRATCHPAD TH TL config: the three bytes reach the
    host as {TL, TH} and the configuration, and a READ SCRATCHPAD after it
    shows them as bytes 2..4 with a CRC to match."""
    bench, bus, master, loom = device_setup(backend, us=FAST)
    finish(bench, master.skip_rom([("write", [WRITE_SCRATCHPAD, 0x1E, 0xF6, 0x3F])]), 100_000)
    assert loom.pop(1, 2) == [0xF61E, 0x3F]
    tr = finish(bench, master.skip_rom([("write", [READ_SCRATCHPAD]), ("read", 9)]), 200_000)
    pad = DEV_PAD[:2] + bytes([0x1E, 0xF6, 0x3F]) + DEV_PAD[5:]
    assert bytes(tr.data) == pad + bytes([crc8(pad)])
    assert master.violations == []


# --------------------------------------------------------------- together
@model_only(SCALED % "1.2 million clocks")
def test_master_and_device_together(backend):
    """The master on thread 0 (BIDIR0) and the device on thread 1 (BIDIR1),
    the two pins joined by the bench as one 1-Wire line: the master's SEARCH
    finds the device's ID with a good CRC, then MATCH ROM and READ
    SCRATCHPAD read its nine bytes, the CRC field 0 after the ninth. Every
    pulse of the master's pad is AN126's to the clock, every 0 of the
    device's is driven within the master's low and held 15..60 us."""
    bus = OneWireBus(["BIDIR0", "BIDIR1"], clock=FAST * 1_000_000)
    bench = ow_bench(backend, bus, dmem=True)
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(MASTER, FAST).words)
    device = program(DEVICE, FAST).words
    assert not set(words) & set(device)
    words.update(device)
    loom.load(words)
    loom.run([0, 1])
    loom.push(1, rom_words(DEV_ROM) + pad_words(DEV_PAD))
    bench.step(STARTUP)
    found, end = m_search(loom)
    assert found == [(0, DEV_ROM)] and end == END
    assert m_reset(loom) == PRESENCE
    m_write(loom, [MATCH_ROM] + list(DEV_ROM) + [READ_SCRATCHPAD])
    data, crcs = m_read(loom, 9)
    assert data == DEV_PAD + bytes([crc8(DEV_PAD)]) and crcs[-1] == 0
    check_an126(chip_slots(bus, "BIDIR0"))
    holds = device_holds(bus, master_name=bus.chip_name("BIDIR0"))
    assert holds and all(h[0] < AN126["A"] and 15 < h[1] < 60 for h in holds), holds
    assert bench.contention_free() and loom.badop() == 0


# ------------------------------------------------- the reference model alone
def test_model_crc8_and_rom_helpers():
    """Maxim's CRC-8 has the catalogue check value 0xA1 for "123456789"
    (CRC-8/MAXIM: poly 0x31 reflected, init 0); a ROM made by make_rom checks
    to 0 over all eight bytes; AN187's order compares the bits in wire
    order, first bit first."""
    assert crc8(b"123456789") == 0xA1
    rom = make_rom(0x0000000001B81C, family=0x02)
    assert rom[:7] == bytes([0x02, 0x1C, 0xB8, 0x01, 0, 0, 0]) and crc8(rom) == 0
    a, b = make_rom(2), make_rom(1)              # serial bit 0: 0 before 1
    assert search_order([b, a]) == [a, b]
    assert rom_words(rom)[0] == 0x1C02


def test_model_master_finds_model_devices():
    """The reference master against the reference devices, no chip: AN187's
    search finds all five in order, MATCH ROM reads one scratchpad with its
    CRC, and neither side records a timing violation."""
    roms = [make_rom(s) for s in (5, 3, 0x800000000000, 6, 0x123456)]
    bus = OneWireBus(["BIDIR0"])
    devs = [bus.add(Ds18b20(r, pd_wait_us=15 + 10 * i, pd_low_us=60 + 40 * i))
            for i, r in enumerate(roms)]
    master = bus.add(OneWireMaster())
    tr = master.search()
    assert bus.run_alone(lambda: tr.done, 5_000_000)
    assert tr.search_error is None and tr.roms == search_order(roms)
    tr = master.match_rom(roms[3], [("write", [READ_SCRATCHPAD]), ("read", 9)])
    assert bus.run_alone(lambda: tr.done, 1_000_000)
    assert bytes(tr.data) == devs[3].scratchpad_bytes() and crc8(tr.data) == 0
    assert master.violations == [] and all(d.violations == [] for d in devs)
