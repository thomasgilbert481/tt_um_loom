"""L3-I2C-MASTER on the golden model: firmware/i2c_master.loom and a 24C02.

The program is loaded and fed through ``tools.loomhost.Loom`` over
``ModelTransport``. SCL (BIDIR0) and SDA (BIDIR1) are open drain with bench
pull-ups; ``tools.protomodels.i2c.I2cEeprom`` answers, ``I2cMonitor``
decodes the bus. Modelled rate: TICK_INT 8, 16 ticks per SCL period, so 128
clocks per bit: 390 kHz at 50 MHz.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomhost import Loom, ModelTransport
from tools.loomisa import REPO, load
from tools.protomodels.bench import Bench, Model, pad_of
from tools.protomodels.i2c import I2cEeprom, I2cMonitor

ISA = load()
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


def setup(**eeprom):
    bench = Bench(pullups=0b11)
    ee = bench.add(I2cEeprom("BIDIR0", "BIDIR1", memory=MEMORY, **eeprom))
    mon = bench.add(I2cMonitor("BIDIR0", "BIDIR1"))
    scl = bench.add(Scl())
    program = assemble_file(REPO / "firmware" / "i2c_master.loom", isa=ISA, strict=True)
    loom = Loom(ModelTransport(bench), isa=ISA)
    loom.load(program)
    loom.write_csr(0, "TICK_INT", TICK)
    loom.run(0)
    return bench, ee, mon, scl, loom


def settle(bench, mon):
    """Let the last STOP finish (results are pushed before it is sent)."""
    assert bench.run_until(lambda: mon.events and mon.events[-1][0] == "stop", 5000, every=16)
    bench.step(LOW * TICK)


def test_write_then_random_read_back():
    bench, ee, mon, scl, loom = setup()
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


def test_current_address_read_continues_after_the_last_byte():
    bench, ee, mon, scl, loom = setup()
    loom.push(0, [START | 0xA0, 0xFE, NOBYTE | STOP])    # set the address pointer
    loom.pop(0, 2)
    loom.push(0, [START | 0xA1, READ, READ, READ | NACK | STOP])
    assert loom.pop(0, 4) == [0xA1, MEMORY[0xFE], MEMORY[0xFF], R_NACK | MEMORY[0x00]]


def test_nack_from_an_absent_address_then_a_bare_stop():
    bench, ee, mon, scl, loom = setup()
    loom.push(0, [START | 0xA4, NOBYTE | STOP])          # 0x52: nobody there
    assert loom.pop(0, 1) == [R_NACK | 0xA4]
    settle(bench, mon)
    assert mon.kinds() == ["start", "byte", "stop"]
    assert loom.fifo_status(0)["outq"] == 0               # NOBYTE pushes nothing


def test_a_refused_data_byte_is_reported():
    bench, ee, mon, scl, loom = setup(nack_data_index=0)
    loom.push(0, [START | 0xA0, 0x20, STOP | 0x55])
    assert loom.pop(0, 3) == [0xA0, 0x20, R_NACK | 0x55]
    settle(bench, mon)
    assert ee.memory[0x20] == MEMORY[0x20]


def test_clock_stretching_is_honoured():
    bench, ee, mon, scl, loom = setup(stretch_clocks=400, stretch_bytes=[1])
    loom.push(0, [START | 0xA0, 0x30, STOP | 0x99])
    assert loom.pop(0, 3) == [0xA0, 0x30, 0x99]
    settle(bench, mon)
    assert ee.stretches == 1 and ee.memory[0x30] == 0x99
    # one low phase is the device's 400-clock hold; the other long ones are
    # the gaps between host commands, when the master itself waits
    assert max(scl.low) >= 400 and sorted(scl.low)[-2] < 400
    # every high phase, the one after the stretch included, is a full one
    assert min(scl.high) >= (2 * Q - 1) * TICK - SLOT


def test_scl_phases_follow_the_tick_schedule():
    bench, ee, mon, scl, loom = setup()
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


def test_a_stuck_scl_times_out_and_the_bus_is_released():
    bench, ee, mon, scl, loom = setup()
    scl.stuck = True
    loom.push(0, [START | 0xA0])
    assert loom.pop(0, 1) == [R_TIMEOUT]
    assert bench.machine.uio_oe & 0b11 == 0               # both lines let go
    scl.stuck = False
    loom.push(0, [START | 0xA0, NOBYTE | STOP])
    assert loom.pop(0, 1) == [0xA0]                       # works again
