"""L3-SPI-SLAVE on the model and the RTL: firmware/spi_slave.loom answers a master.

The program is loaded and fed through ``tools.loomhost.Loom``;
``tools.protomodels.spi.SpiMaster`` clocks the bus (SCK IN1, MOSI IN2, CS_n
IN3) and samples MISO on OUT0. Modelled rate: TICK_INT 32, a quarter of an
SCK period, so a period is 128 clocks: 390 kHz at 50 MHz, bytes back to back.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model over ``ModelTransport``, ``test/test_fw.py`` runs
the same bodies on the RTL over ``SimTransport`` and the real SPI host pads.
Error cases: an empty INQ (the slave sends its idle byte), a full OUTQ (the
byte is dropped) and a transaction cut off in the middle of a byte (the
slave drops the partial byte and resynchronises). All four SPI modes: the
host sets r5 = 1 before RUN for modes 1 and 2, which sample on the falling
edge (D-035).
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.bench import Drive, Model, pad_of
from tools.protomodels.spi import SpiMaster
from tools.tests.fw_backend import model_only

ISA = load()
PINS = ("IN1", "IN2", "OUT0", "IN3")                     # SCK, MOSI, MISO, CS_n
TICK = 32                                                # a quarter of an SCK period
HALF = 2 * TICK                                          # half an SCK period
PERIOD = 2 * HALF
SLOT = 4
IDLE = 0xFF                                              # the program's filler byte
#: Ticks of silence after which the slave looks at CS_n (`TMO` in the program).
TMO = 16
#: Cycles that guarantee the slave has noticed a deselect and gone back to
#: watching INQ: the timeout plus a byte's worth of slack.
RESYNC = TMO * TICK + PERIOD
#: Slots from a word landing in INQ (deselected) to the slave waiting for the
#: first sampling edge with its first bit on MISO: the program's bound
#: (BUGS 12). The master's first sampling edge comes HALF after CS_n falls in
#: modes 0 and 2, so a host has to finish queueing LEAD cycles before
#: selecting.
READY = 21
LEAD = READY * SLOT - HALF


def setup(backend, mode=0):
    """A running slave with a master on its pins."""
    bench = backend.bench()
    master = bench.add(SpiMaster(*PINS, mode=mode, half=HALF,
                                 cs_setup=HALF, cs_hold=HALF))
    program = assemble_file(REPO / "firmware" / "spi_slave.loom", isa=ISA, strict=True)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program)
    loom.write_csr(0, "TICK_INT", TICK)
    if mode in (1, 2):
        loom.write_reg(0, "r5", 1)     # sample on the falling edge
    loom.run(0)
    bench.step(16 * SLOT)          # let it reach the deselected state and watch INQ
    return bench, master, loom


def transfer(bench, master, data):
    """Clock ``data`` out of the master and return what came back on MISO.

    The master selects LEAD cycles after the call, as a host that queued the
    response first and then let the master go would (BUGS 12: before that
    rule the tests selected one cycle after the push and passed on a lucky
    phase)."""
    bench.step(LEAD)
    master.transfer(bytes(data))
    assert bench.run_until(lambda: not master.busy, 40000, every=64)
    bench.step(SLOT * 8)
    return master.results[-1]


class Miso(Model):
    """Every SCK rise and fall and every MISO change, by cycle."""

    def __init__(self):
        self.sck, self.miso = pad_of("IN1"), pad_of("OUT0")
        self.prev = None
        self.rises, self.falls, self.changes = [], [], []

    def observe(self, lines):
        now = (lines.get(self.sck), lines.get(self.miso))
        if self.prev is not None:
            if now[0] and not self.prev[0]:
                self.rises.append(lines.cycle)
            if self.prev[0] and not now[0]:
                self.falls.append(lines.cycle)
            if now[1] != self.prev[1]:
                self.changes.append(lines.cycle)
        self.prev = now


class CutOff(Model):
    """A master that clocks a few bits of a byte and then deselects.

    Mode 0, MSB first, the same half period as :class:`SpiMaster`. It is not
    a reference model, only the stimulus for "the transaction ended in the
    middle of a byte".
    """

    def __init__(self, bits=4, value=0xA5, half=HALF, setup=HALF):
        self.sck, self.mosi = pad_of("IN1"), pad_of("IN2")
        self.cs = pad_of("IN3")
        self.bits, self.value, self.half, self.setup = bits, value, half, setup
        self.t0 = None
        self.now = 0

    def start(self, cycle):
        self.t0 = cycle + 1

    @property
    def busy(self):
        return self.t0 is not None and self.now < self.t0 + self.span

    @property
    def span(self):
        return self.setup + 2 * self.half * self.bits + self.setup

    def drive(self, drive: Drive, cycle):
        self.now = cycle
        cs, sck, mosi = 1, 0, 0
        if self.t0 is not None:
            k = cycle - self.t0
            if 0 <= k < self.span:
                cs = 0
                if self.setup <= k < self.span - self.setup:
                    index, within = divmod(k - self.setup, 2 * self.half)
                    sck = int(within >= self.half)
                    mosi = (self.value >> (7 - index)) & 1
        drive.set(self.cs, cs)
        drive.set(self.sck, sck)
        drive.set(self.mosi, mosi)


def test_slave_exchanges_queued_bytes_with_the_master(backend):
    bench, master, loom = setup(backend)
    loom.push(0, [0xA5, 0x3C, 0x00, 0xF0])
    assert transfer(bench, master, b"\x9f\x01\x02\x03") == bytes([0xA5, 0x3C, 0x00, 0xF0])
    assert loom.pop(0, 4) == [0x9F, 0x01, 0x02, 0x03]
    assert bench.contention_free() and loom.badop() == 0


def test_an_empty_queue_sends_the_idle_byte(backend):
    bench, master, loom = setup(backend)
    assert transfer(bench, master, b"\x55\xaa") == bytes([IDLE, IDLE])
    assert loom.pop(0, 2) == [0x55, 0xAA]


@pytest.mark.parametrize("mode", [1, 2, 3])
def test_every_other_mode_exchanges_bytes(backend, mode):
    """Mode 3 samples on the rising edge as mode 0 does; modes 1 and 2 on the
    falling edge, the copy of the loop the host picks with r5 = 1. Each
    exchange is two bytes back to back with a queued pair, so both the first
    bit (on MISO before any edge) and the byte-to-byte path are covered."""
    bench, master, loom = setup(backend, mode=mode)
    loom.push(0, [0x12, 0x34])
    assert transfer(bench, master, b"\xde\xad") == bytes([0x12, 0x34])
    assert loom.pop(0, 2) == [0xDE, 0xAD]
    assert bench.contention_free() and loom.badop() == 0


def test_a_queued_byte_replaces_the_idle_byte_before_the_next_transaction(backend):
    """Two transactions: the second one's bytes are queued after the first."""
    bench, master, loom = setup(backend)
    loom.push(0, [0x11, 0x22])
    assert transfer(bench, master, b"\x01\x02") == bytes([0x11, 0x22])
    assert loom.pop(0, 2) == [0x01, 0x02]
    loom.push(0, [0x33, 0x44])
    bench.step(RESYNC)                                   # the slave notices the deselect
    assert transfer(bench, master, b"\x03\x04") == bytes([0x33, 0x44])
    assert loom.pop(0, 2) == [0x03, 0x04]


@pytest.mark.parametrize("mode", [0, 2])
def test_miso_changes_once_per_bit_soon_after_the_sampling_edge(backend, mode):
    """What the master needs of MISO, measured on the pads.

    0xAA alternates, so every one of the seven intervals between sampling
    edges holds exactly one MISO change. The slave writes it six slots after
    the X cycle that sees the edge, seven when the bit it just sampled was a
    1 (the ORI); that X cycle is the pad's two synchroniser clocks and up to
    three more after the edge, and the pin follows two clocks after the
    write. So the change is after the edge and long before the next one. No
    deadline times it: the first version changed MISO one tick after the
    edge, which after an edge cannot be proved (tools finding T-1). Mode 2
    samples on the falling edge, through the other copy of the loop, and
    must show the same numbers against its falling edges.
    """
    bench, master, loom = setup(backend, mode=mode)
    watch = bench.add(Miso())
    loom.push(0, [0xAA])
    assert transfer(bench, master, bytes([0xA5])) == bytes([0xAA])  # MOSI: 1s and 0s
    edges = watch.rises if mode == 0 else watch.falls
    assert len(edges) == 8
    for edge, nxt in zip(edges, edges[1:]):
        inside = [c for c in watch.changes if edge < c < nxt]
        assert len(inside) == 1, (edge, inside)
        assert 6 * SLOT + 4 <= inside[0] - edge <= 7 * SLOT + 7
        assert nxt - inside[0] >= 2 * TICK                     # half a period of setup


@model_only("sixteen setups, one per clock phase of the poll loop; the RTL "
            "runs the same cycle-exact program (L3 lockstep co-simulation)")
@pytest.mark.parametrize("mode", [0, 2])
def test_a_byte_queued_at_the_stated_lead_is_on_time_at_every_phase(backend, mode):
    """BUGS 12: the program's bound, at every phase of the poll loop.

    One byte queued while deselected, the master selecting LEAD cycles after
    the push returns, so its first sampling edge is READY slots after: the
    master reads that byte, whatever the slave was doing when it landed.
    Modes 0 and 2 are the tight ones (their first sampling edge is the
    leading edge, HALF after CS_n falls); before BUGS 12 one phase in four
    came out one bit late with the master selecting one cycle after the
    push.
    """
    for phase in range(4 * SLOT):
        bench, master, loom = setup(backend, mode=mode)
        bench.step(phase)
        loom.push(0, [0xAA])
        bench.step(LEAD)
        master.transfer(bytes([0xA5]))
        assert bench.run_until(lambda: not master.busy, 40000, every=64)
        bench.step(SLOT * 8)
        assert master.results[-1] == bytes([0xAA]), phase
        assert loom.pop(0, 1) == [0xA5], phase


def test_a_byte_is_dropped_when_outq_is_full(backend):
    """OUTQ holds four; the host does not pop while the master clocks six."""
    bench, master, loom = setup(backend)
    assert loom.fifo_status(0)["depth"] == 4
    assert transfer(bench, master, b"abcdef") == bytes([IDLE] * 6)
    assert loom.pop(0, 4) == list(b"abcd")               # e and f had nowhere to go
    assert loom.fifo_status(0)["outq"] == 0
    assert loom.badop() == 0                             # the slave dropped them, not the host


def test_a_transaction_cut_off_mid_byte_is_dropped(backend):
    bench, master, loom = setup(backend)
    cut = bench.add(CutOff(bits=4, value=0xA5))
    loom.push(0, [0x77])
    bench.step(PERIOD)                                   # let the slave pre-load it
    cut.start(bench.cycle)
    assert bench.run_until(lambda: not cut.busy, 20000, every=64)
    bench.remove(cut)
    bench.step(RESYNC)                                   # the partial byte is dropped
    assert loom.fifo_status(0)["outq"] == 0
    loom.push(0, [0x88])
    bench.step(RESYNC)
    assert transfer(bench, master, b"\x5a\x5a") == bytes([0x88, IDLE])
    assert loom.pop(0, 2) == [0x5A, 0x5A]
    assert loom.badop() == 0
