"""tools.protomodels.i2c: the EEPROM, the monitor and the master together.

SCL is BIDIR0 and SDA is BIDIR1 with bench pull-ups; the chip only provides
the pads and never drives them, so every level on the bus is the wired AND
of the two models.
"""

import pytest

from tools.protomodels.bench import Bench
from tools.protomodels.i2c import I2cEeprom, I2cMaster, I2cMonitor

MEMORY = bytes((0x80 + i) & 0xFF for i in range(256))


def bus(**eeprom):
    bench = Bench(pullups=0b11)
    ee = bench.add(I2cEeprom("BIDIR0", "BIDIR1", memory=MEMORY, **eeprom))
    mon = bench.add(I2cMonitor("BIDIR0", "BIDIR1"))
    master = bench.add(I2cMaster("BIDIR0", "BIDIR1", quarter=5))
    return bench, ee, mon, master


def settle(bench, master):
    assert bench.run_until(lambda: not master.busy, 100000, every=8)
    bench.step(20)


def write(master, addr, data, stop=True):
    master.start()
    master.write(0xA0)
    master.write(addr)
    for byte in data:
        master.write(byte)
    if stop:
        master.stop()


def test_byte_write_then_random_read():
    bench, ee, mon, m = bus()
    write(m, 0x20, [0x5A])
    m.start(); m.write(0xA0); m.write(0x20)
    m.start(); m.write(0xA1); m.read(ack=False); m.stop()
    settle(bench, m)
    assert m.results == [True, True, True, True, True, True, 0x5A]
    assert ee.memory[0x20] == 0x5A
    assert mon.kinds() == ["start", "byte", "byte", "byte", "stop",
                           "start", "byte", "byte", "rstart", "byte", "byte", "stop"]
    assert bench.contention_free()


def test_page_write_wraps_inside_the_page():
    bench, ee, mon, m = bus()
    write(m, 0x0E, [1, 2, 3, 4])                           # page 0x08..0x0F
    settle(bench, m)
    assert list(ee.memory[0x0E:0x10]) == [1, 2]
    assert list(ee.memory[0x08:0x0A]) == [3, 4]
    assert ee.memory[0x10] == MEMORY[0x10]


def test_current_address_and_sequential_reads():
    bench, ee, mon, m = bus()
    write(m, 0xFE, [], stop=False)                         # set the address only
    m.stop()
    m.start(); m.write(0xA1)
    for _ in range(3):
        m.read(ack=True)
    m.read(ack=False); m.stop()
    m.start(); m.write(0xA1); m.read(ack=False); m.stop()  # current address
    settle(bench, m)
    assert m.results[-6:-2] == [MEMORY[0xFE], MEMORY[0xFF], MEMORY[0x00], MEMORY[0x01]]
    assert m.results[-1] == MEMORY[0x02]


def test_data_is_committed_only_at_stop():
    bench, ee, mon, m = bus()
    write(m, 0x30, [0x11], stop=False)
    m.start(); m.write(0xA1); m.read(ack=False); m.stop()  # repeated START aborts the write
    settle(bench, m)
    assert ee.memory[0x30] == MEMORY[0x30]


def test_nack_for_another_address_an_absent_device_and_a_refused_byte():
    bench, ee, mon, m = bus(nack_data_index=1)
    m.start(); m.write(0xA2); m.stop()                     # address 0x51: nobody
    write(m, 0x40, [0x01, 0x02])                           # the second data byte refused
    settle(bench, m)
    assert m.results == [False, True, True, True, False]
    assert ee.memory[0x40] == 0x01 and ee.memory[0x41] == MEMORY[0x41]
    absent = bus(present=False)
    absent[3].start(); absent[3].write(0xA0); absent[3].stop()
    settle(absent[0], absent[3])
    assert absent[3].results == [False]


def test_write_cycle_nacks_the_address_until_it_ends():
    bench, ee, mon, m = bus(write_cycle_clocks=1500)       # about six polls
    write(m, 0x00, [0x77])
    for _ in range(12):                                    # ACK polling
        m.start(); m.write(0xA0); m.stop()
    settle(bench, m)
    polls = m.results[3:]
    assert polls[0] is False and polls[-1] is True
    assert polls == sorted(polls)                          # NACKs, then ACKs


def test_clock_stretching_holds_scl_and_the_master_waits():
    bench, ee, mon, m = bus(stretch_clocks=200, stretch_bytes=[1])
    write(m, 0x50, [0x99])
    settle(bench, m)
    assert m.results == [True, True, True] and ee.memory[0x50] == 0x99
    assert ee.stretches == 1
    # the ACK clock after the stretched byte still has a full high phase
    assert min(mon.high_times) >= 2 * 5 - 1
    assert bench.contention_free()
