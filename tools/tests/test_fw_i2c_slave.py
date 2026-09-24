"""L3-I2C-S on the model and the RTL: firmware/i2c_slave_eeprom.loom as a 24C02.

The program is loaded and run through ``tools.loomhost.Loom``; its 256 bytes
live in the instruction memory at 0x180..0x1FF (SEMANTICS 6.11), so the
host loads a starting image with the program and reads the array back with
its IMEM commands once the thread is halted. SCL (BIDIR0) and SDA (BIDIR1)
are open drain with bench pull-ups; ``tools.protomodels.i2c_master.
I2cBusMaster`` is the master, timed by the I2C specification at 100 kHz
(Standard mode: SCL low 235, high 265 clocks at 50 MHz) and 400 kHz (Fast
mode: low 65, high 60), with data changing in the clock SCL falls (hold 0),
and it checks every bit the device drives against tVD (3.45 us and 0.9 us).
``I2cMonitor`` decodes the bus for an independent view of STARTs and STOPs.

What is expected comes from the 24C02 datasheet (8-byte page write that
wraps inside the page, reads that wrap at the end of the array, the address
counter shared by reads and writes, no ACK for another device address) and
from the program header (the packing of two bytes per word), never from
what the model or the RTL did.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import random

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.i2c import I2cMonitor
from tools.protomodels.i2c_master import I2cBusMaster
from tools.tests.fw_backend import model_only

ISA = load()
PROGRAM = REPO / "firmware" / "i2c_slave_eeprom.loom"
WINDOW = 0x180                          # the array's first word (program header)
RATES = [100_000, 400_000]
DEVICE = 0x50
STARTUP = 200       # clocks after RUN for the program's set-up (13 slots) to finish


def pack(memory):
    """The 128-word image of 256 bytes: even byte in 15:8, odd byte in 7:0."""
    return {WINDOW + i: (memory[2 * i] << 8) | memory[2 * i + 1] for i in range(128)}


def unpack(words):
    out = bytearray()
    for w in words:
        out += bytes(((w >> 8) & 0xFF, w & 0xFF))
    return bytes(out)


def eeprom_bench(backend):
    """The chip has slice B (``LD``/``ST``) and the 512-word memory.

    The golden model builds them as the ``DMEM`` feature and
    ``imem_words=512``; the RTL has both always and ``RtlBench`` takes
    neither keyword."""
    kwargs = dict(pullups=0b11)
    if backend.name == "model":
        kwargs.update(features=("FIFO", "DMEM"), imem_words=512)
    return backend.bench(**kwargs)


def setup(backend, rate=400_000, memory=None):
    """Load the program (and ``memory`` into the array, else the erased
    image of the program), run it, and put a master and a monitor on the bus."""
    bench = eeprom_bench(backend)
    program = assemble_file(PROGRAM, isa=ISA, strict=True)
    words = dict(program.words)
    if memory is not None:
        words.update(pack(memory))
    loom = backend.loom(bench, isa=ISA)
    loom.load(words)
    loom.run(0)
    bench.step(STARTUP)
    master = bench.add(I2cBusMaster("BIDIR0", "BIDIR1", rate=rate))
    mon = bench.add(I2cMonitor("BIDIR0", "BIDIR1"))
    return bench, loom, master, mon


def finish(bench, master, tr, nbytes):
    """Run until ``tr`` (``nbytes`` bytes on the wire) is over."""
    t = master.t
    budget = (nbytes * 9 + 12) * t.period + 4 * t.buf + 2000
    assert bench.run_until(lambda: tr.done, budget, every=64), \
        "transaction not finished after %d clocks: %r" % (budget, tr)
    return tr


def array(loom):
    """The 256 bytes, read back through the host port (the thread halted)."""
    loom.halt(0)
    return unpack(loom.read_imem(WINDOW, 128))


def clean(master, mon, bench):
    assert master.violations == [], master.violations[:5]
    assert bench.contention_free()


# ------------------------------------------------------------- the program
def test_i2c_slave_eeprom_assembles_strict_with_no_diagnostic():
    """L2-DEADLINE for i2c_slave_eeprom.loom: the master owns SCL, so the
    program has no ``WAITD`` and no pair to prove, and nothing to report.
    Its code fits thread 0's quarter and its data fills thread 3's."""
    program = assemble_file(PROGRAM, isa=ISA, strict=True)
    assert program.diagnostics == []
    assert program.deadlines[0].pairs == []
    assert sorted(program.threads) == [0, 3]
    assert max(a for a in program.words if a < WINDOW) < 512 // 4
    assert sorted(a for a in program.words if a >= WINDOW) == \
        list(range(WINDOW, WINDOW + 128))


# ---------------------------------------------------------------- writes
@pytest.mark.parametrize("rate", RATES)
def test_byte_writes_then_random_reads(backend, rate):
    """Single bytes at even and odd addresses, each read back at once."""
    bench, loom, master, mon = setup(backend, rate)
    cases = [(0x00, 0x5A), (0x11, 0xA5), (0x80, 0x00), (0xFF, 0x3C)]
    for addr, value in cases:
        tr = finish(bench, master, master.eeprom_write(addr, bytes([value])), 4)
        assert tr.acks == [True, True, True]
        tr = finish(bench, master, master.eeprom_read(addr, 1), 5)
        assert tr.acks == [True, True, True] and tr.data == [value]
    clean(master, mon, bench)
    assert mon.kinds()[:6] == ["start", "byte", "byte", "byte", "stop", "start"]
    assert mon.kinds().count("rstart") == len(cases)
    expected = bytearray(b"\xFF" * 256)
    for addr, value in cases:
        expected[addr] = value
    assert array(loom) == bytes(expected)      # the packing of the header


@pytest.mark.parametrize("rate", RATES)
def test_page_write_then_sequential_read(backend, rate):
    bench, loom, master, mon = setup(backend, rate)
    page = bytes([0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE])
    tr = finish(bench, master, master.eeprom_write(0x40, page), 11)
    assert tr.acks == [True] * 10
    tr = finish(bench, master, master.eeprom_read(0x40, 8), 12)
    assert tr.data == list(page)
    clean(master, mon, bench)
    assert array(loom)[0x40:0x48] == page


def test_page_write_wraps_inside_the_page(backend):
    """Five bytes from 0x1D: 0x1D 0x1E 0x1F, then 0x18 0x19 (24C02 page
    write: the low three address bits roll over, the page stays)."""
    memory = bytes(0x80 | (i & 0x7F) for i in range(256))
    bench, loom, master, mon = setup(backend, memory=memory)
    tr = finish(bench, master, master.eeprom_write(0x1D, b"\x01\x02\x03\x04\x05"), 8)
    assert tr.acks == [True] * 7
    # the counter is left inside the page too: 0x1A, not 0x22
    tr = finish(bench, master, master.eeprom_current_read(1), 3)
    assert tr.data == [memory[0x1A]]
    tr = finish(bench, master, master.eeprom_read(0x18, 8), 12)
    assert tr.data == [0x04, 0x05, memory[0x1A], memory[0x1B], memory[0x1C], 0x01, 0x02, 0x03]
    clean(master, mon, bench)
    mem = array(loom)
    assert mem[0x18:0x20] == bytes([0x04, 0x05, memory[0x1A], memory[0x1B], memory[0x1C],
                                    0x01, 0x02, 0x03])
    assert mem[:0x18] == memory[:0x18] and mem[0x20:] == memory[0x20:]   # no spill


# ----------------------------------------------------------------- reads
def test_current_address_reads_follow_the_counter(backend):
    memory = bytes((7 * i + 3) & 0xFF for i in range(256))
    bench, loom, master, mon = setup(backend, memory=memory)
    tr = finish(bench, master, master.eeprom_current_read(2), 4)
    assert tr.data == [memory[0], memory[1]]     # the counter starts at 0
    tr = finish(bench, master, master.eeprom_read(0x30, 2), 6)
    assert tr.data == [memory[0x30], memory[0x31]]
    tr = finish(bench, master, master.eeprom_current_read(3), 5)
    assert tr.data == [memory[0x32], memory[0x33], memory[0x34]]
    tr = finish(bench, master, master.eeprom_write(0x90, b"\x42"), 4)
    tr = finish(bench, master, master.eeprom_current_read(1), 3)
    assert tr.data == [memory[0x91]]             # a write moves the counter too
    clean(master, mon, bench)


def test_sequential_read_wraps_at_the_end_of_the_array(backend):
    memory = bytes(random.Random(5).randrange(256) for _ in range(256))
    bench, loom, master, mon = setup(backend, memory=memory)
    tr = finish(bench, master, master.eeprom_read(0xFA, 12), 16)
    assert tr.data == list(memory[0xFA:] + memory[:6])
    clean(master, mon, bench)


@model_only("257 bytes at 400 kHz is about 300000 clocks, minutes on the RTL; "
            "the wrap at 0xFF is covered on both by the 12-byte read above")
@pytest.mark.parametrize("rate", [400_000])
def test_sequential_read_of_the_whole_array(backend, rate):
    memory = bytes(random.Random(9).randrange(256) for _ in range(256))
    bench, loom, master, mon = setup(backend, rate, memory=memory)
    tr = finish(bench, master, master.eeprom_read(0x00, 257), 261)
    assert tr.data == list(memory) + [memory[0]]
    clean(master, mon, bench)


# ------------------------------------------------------------ addressing
def test_another_device_address_is_not_acknowledged(backend):
    bench, loom, master, mon = setup(backend)
    for device in (0x51, 0x58, 0x28, 0x7F):
        tr = finish(bench, master, master.address_only(device), 1)
        assert tr.acks == [False] and tr.aborted
    tr = finish(bench, master, master.eeprom_write(0x05, b"\x77", device=0x52), 1)
    assert tr.acks == [False]                    # the write never gets through
    tr = finish(bench, master, master.eeprom_write(0x06, b"\x66"), 4)
    assert tr.acks == [True, True, True]         # the next one is answered
    tr = finish(bench, master, master.eeprom_read(0x05, 2), 6)
    assert tr.data == [0xFF, 0x66]
    tr = finish(bench, master, master.address_only(DEVICE, read=False), 1)
    assert tr.acks == [True]
    clean(master, mon, bench)
    assert array(loom)[0x05:0x07] == b"\xFF\x66"
