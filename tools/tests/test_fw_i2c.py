"""L3-I2C-MASTER on the model and the RTL: firmware/i2c_master.loom and a 24C02.

The program is loaded and fed through ``tools.loomhost.Loom``. SCL (BIDIR0)
and SDA (BIDIR1) are open drain with bench pull-ups;
``tools.protomodels.i2c.I2cEeprom`` answers, ``I2cMonitor`` decodes the bus.
Modelled rate: TICK_INT 8, 16 ticks per SCL period, so 128 clocks per bit:
390 kHz at 50 MHz.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model over ``ModelTransport``, ``test/test_fw.py`` runs
the same bodies on the RTL over ``SimTransport`` and the real SPI pads, where
the open-drain wired-AND is resolved the same way by ``RtlBench``. The error
cases (NACK, refused data byte, clock stretching, stuck SCL) run on both.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.i2c import I2cEeprom, I2cMonitor, I2cTiming

ISA = load()
CLOCK_HZ = 50e6                                          # the datasheet's clock
TICK = 8
Q, LOW = 4, 8                                            # as in the program
SLOT = 4
START, STOP, READ, NACK, NOBYTE = 0x100, 0x200, 0x400, 0x800, 0x1000
R_NACK, R_TIMEOUT = 0x8000, 0x4000
MEMORY = bytes((0x40 + 3 * i) & 0xFF for i in range(256))


class Scl(Model):
    """Low and high phase lengths of SCL, and an optional stuck-low fault."""

    def __init__(self):
        self.pad = pad_of("BIDIR0")
        self.prev, self.since = 1, 0
        self.low, self.high = [], []
        self.stuck = False

    def drive(self, drive, cycle):
        if self.stuck:
            drive.pull_low(self.pad)

    def observe(self, lines):
        v = lines.get(self.pad)
        if v != self.prev:
            (self.high if v == 0 else self.low).append(lines.cycle - self.since)
            self.since, self.prev = lines.cycle, v


def setup(backend, **eeprom):
    bench = backend.bench(pullups=0b11)
    ee = bench.add(I2cEeprom("BIDIR0", "BIDIR1", memory=MEMORY, **eeprom))
    mon = bench.add(I2cMonitor("BIDIR0", "BIDIR1"))
    scl = bench.add(Scl())
    program = assemble_file(REPO / "firmware" / "i2c_master.loom", isa=ISA, strict=True)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program)
    loom.write_csr(0, "TICK_INT", TICK)
    loom.run(0)
    return bench, ee, mon, scl, loom


def settle(bench, mon):
    """Let the last STOP finish (results are pushed before it is sent)."""
    assert bench.run_until(lambda: mon.events and mon.events[-1][0] == "stop", 5000, every=16)
    bench.step(LOW * TICK)


def test_write_then_random_read_back(backend):
    bench, ee, mon, scl, loom = setup(backend)
    loom.push(0, [START | 0xA0, 0x10, 0x11, STOP | 0x22])
    assert loom.pop(0, 4) == [0xA0, 0x10, 0x11, 0x22]    # all acknowledged
    settle(bench, mon)
    assert bytes(ee.memory[0x10:0x12]) == b"\x11\x22"
    loom.push(0, [START | 0xA0, 0x10, START | 0xA1, READ, READ | NACK | STOP])
    assert loom.pop(0, 5) == [0xA0, 0x10, 0xA1, 0x11, R_NACK | 0x22]
    settle(bench, mon)
    assert mon.kinds() == ["start", "byte", "byte", "byte", "byte", "stop",
                           "start", "byte", "byte", "rstart", "byte", "byte", "byte", "stop"]
    assert bench.contention_free()
    assert loom.badop() == 0


def test_current_address_read_continues_after_the_last_byte(backend):
    bench, ee, mon, scl, loom = setup(backend)
    loom.push(0, [START | 0xA0, 0xFE, NOBYTE | STOP])    # set the address pointer
    loom.pop(0, 2)
    loom.push(0, [START | 0xA1, READ, READ, READ | NACK | STOP])
    assert loom.pop(0, 4) == [0xA1, MEMORY[0xFE], MEMORY[0xFF], R_NACK | MEMORY[0x00]]


def test_nack_from_an_absent_address_then_a_bare_stop(backend):
    bench, ee, mon, scl, loom = setup(backend)
    loom.push(0, [START | 0xA4, NOBYTE | STOP])          # 0x52: nobody there
    assert loom.pop(0, 1) == [R_NACK | 0xA4]
    settle(bench, mon)
    assert mon.kinds() == ["start", "byte", "stop"]
    assert loom.fifo_status(0)["outq"] == 0               # NOBYTE pushes nothing


def test_a_refused_data_byte_is_reported(backend):
    bench, ee, mon, scl, loom = setup(backend, nack_data_index=0)
    loom.push(0, [START | 0xA0, 0x20, STOP | 0x55])
    assert loom.pop(0, 3) == [0xA0, 0x20, R_NACK | 0x55]
    settle(bench, mon)
    assert ee.memory[0x20] == MEMORY[0x20]


def test_clock_stretching_is_honoured(backend):
    bench, ee, mon, scl, loom = setup(backend, stretch_clocks=400, stretch_bytes=[1])
    loom.push(0, [START | 0xA0, 0x30, STOP | 0x99])
    assert loom.pop(0, 3) == [0xA0, 0x30, 0x99]
    settle(bench, mon)
    assert ee.stretches == 1 and ee.memory[0x30] == 0x99
    # one low phase is the device's 400-clock hold; the other long ones are
    # the gaps between host commands, when the master itself waits
    assert max(scl.low) >= 400 and sorted(scl.low)[-2] < 400
    # every high phase, the one after the stretch included, is a full one
    assert min(scl.high) >= (2 * Q - 1) * TICK - SLOT


def test_scl_phases_follow_the_tick_schedule(backend):
    bench, ee, mon, scl, loom = setup(backend)
    loom.push(0, [START | 0xA0, 0x00, 0x01, 0x02, STOP | 0x03])
    loom.pop(0, 5)
    settle(bench, mon)
    # low: LOW ticks from the fall's deadline to the release's deadline, the
    # release one slot later in its slot than the fall. Inside a byte that is
    # exact (68 clocks); between two host commands the master adds its
    # command decoding, so those low phases are longer, never shorter.
    lows = scl.low
    assert min(lows) >= LOW * TICK
    assert sum(t == LOW * TICK + SLOT for t in lows) >= 5 * 8   # 8 per byte
    # high: re-anchored on the rise, so 2Q ticks less up to one tick, plus
    # the slots between the rise and the SETD that re-anchors (the first
    # entry is the idle bus before the first START)
    highs = scl.high[1:]
    assert all((2 * Q - 1) * TICK - SLOT <= t <= 2 * Q * TICK + 3 * SLOT
               for t in highs), highs


@pytest.mark.parametrize("mode, tick_int, tick_frac", [
    ("standard", 31, 64),                # 100.0 kHz at 50 MHz
    ("fast", 7, 0),                      # 390.6 kHz: shortest period 128 clocks
])
def test_um10204_table_10_holds_at_the_datasheet_rates(backend, mode, tick_int, tick_frac):
    """Every UM10204 table 10 bound a master has to meet, measured on the
    pins by ``I2cTiming``, at the datasheet's settings: standard mode at
    100 kHz, and fast mode (r5 = 1) at 390.6 kHz, the fastest rate at or
    under 400 kHz that the 4-clock slot grid gives at 50 MHz. Writes, a
    random read with a repeated START and a bare START/STOP, so that every
    quantity the table bounds occurs. BUGS 13: START and STOP set-up and
    hold were a quarter period, 2.5 us at 100 kHz against the table's 4.0
    and 4.7 us, and the even 8/8 split could not give fast mode's 1.3 us low
    time near 400 kHz.
    """
    bench = backend.bench(pullups=0b11)
    bench.add(I2cEeprom("BIDIR0", "BIDIR1", memory=MEMORY))
    mon = bench.add(I2cMonitor("BIDIR0", "BIDIR1"))
    timing = bench.add(I2cTiming("BIDIR0", "BIDIR1"))
    program = assemble_file(REPO / "firmware" / "i2c_master.loom", isa=ISA, strict=True)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program)
    loom.write_csr(0, "TICK_INT", tick_int)
    loom.write_csr(0, "TICK_FRAC", tick_frac)
    if mode == "fast":
        loom.write_reg(0, "r5", 1)
    loom.run(0)
    loom.push(0, [START | 0xA0, 0x10, 0x11, STOP | 0x22])
    assert loom.pop(0, 4) == [0xA0, 0x10, 0x11, 0x22]
    loom.push(0, [START | 0xA0, 0x10, START | 0xA1, READ, READ | NACK | STOP])
    assert loom.pop(0, 5) == [0xA0, 0x10, 0xA1, 0x11, R_NACK | 0x22]
    loom.push(0, [START | 0xA0, NOBYTE | STOP])
    assert loom.pop(0, 1) == [0xA0]
    assert bench.run_until(lambda: mon.kinds().count("stop") == 3, 200000, every=64)
    bench.step(16 * tick_int)
    assert timing.violations(mode, CLOCK_HZ) == []
    got = timing.measure()
    assert CLOCK_HZ / min(got["period"]) >= (99_000 if mode == "standard" else 390_000)
    assert all(got[k] for k in ("t_su_sta", "t_hd_sta", "t_su_sto", "t_buf", "t_su_dat"))
    assert bench.contention_free() and loom.badop() == 0


def test_a_stuck_scl_times_out_and_the_bus_is_released(backend):
    bench, ee, mon, scl, loom = setup(backend)
    scl.stuck = True
    loom.push(0, [START | 0xA0])
    assert loom.pop(0, 1) == [R_TIMEOUT]
    assert bench.lines.uio_oe & 0b11 == 0                # both lines let go
    scl.stuck = False
    loom.push(0, [START | 0xA0, NOBYTE | STOP])
    assert loom.pop(0, 1) == [0xA0]                      # works again
