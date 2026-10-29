"""L3-DMX on the model and the RTL: firmware/dmx_controller.loom and dmx_fixture.loom.

The controller (thread 0, TX = OUT0) sends the packets the host feeds it;
``tools.protomodels.dmx.DmxRx`` decodes them and times every field against
E1.11's transmit column. The fixture (thread 0, RX = IN0) receives packets
from ``DmxTx``, which sits at the receive column's limits or past them,
and reports its window and a status per packet. Together, the controller
on thread 1 drives the fixture on thread 0 through a bench wire.

Expected values come from ANSI E1.11 through the reference model (the
timing table, the slot format) and from the program headers (the command
and report formats), never from what the model or the RTL did.

Rate: 250 kbit/s, the only one DMX has: 200 clocks a bit at 50 MHz, a
controller tick of 200 clocks and a fixture tick of 25. A 512-slot packet
is about 23 ms, 1.1 M clocks, so those scenarios are ``model_only``; the
RTL runs short packets, which take the same code paths.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.dmx import TX, US, DmxRx, DmxTx, packet_clocks
from tools.tests.fw_backend import model_only

ISA = load()
CONTROLLER = REPO / "firmware" / "dmx_controller.loom"
FIXTURE = REPO / "firmware" / "dmx_fixture.loom"
IMEM = 512
BIT = 200                           # clocks a bit at 50 MHz, 250 kbit/s
SLOT = 4                            # the slot grid (SEMANTICS 2)
CTRL_TICK = 200                     # one tick a bit (controller header)
FIX_TICK = 25                       # eight ticks a bit (fixture header)

# The controller's timing, from its header: 44-bit break, 4-bit MAB,
# break to break at least 302 bits.
BREAK, MAB, BTB = 44 * BIT, 4 * BIT, 302 * BIT

# The fixture's report (program header).
S1, S2 = 0x8000, 0x4000
SCERR, MABF, SHORTF, LOSTF, EMPTYF = 1 << 8, 1 << 9, 1 << 10, 1 << 11, 1 << 12
FERR = 1 << 8                       # in a slot word


def dmx_bench(backend):
    """The FIFOs, the bit engine, LD/ST and the 512-word memory on the
    model; the RTL has all of them and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM)
    return backend.bench()


def program(path, thread=0, pins=None):
    """A program for the 512-word memory, moved to ``thread`` and to other
    pins by changing only its ``.thread`` and ``.pins`` lines."""
    text = path.read_text(encoding="utf-8")
    if thread:
        assert text.count(".thread 0") == 1
        text = text.replace(".thread 0", ".thread %d" % thread)
    if pins:
        old, new = pins
        assert text.count(old) == 1, old
        text = text.replace(old, new)
    prog = assemble_text(text, path.name, isa=ISA, imem_words=IMEM, strict=True)
    assert prog.diagnostics == []
    return prog


def boot(backend, bench, programs):
    """Load ``[(program, thread, tick)]`` as one image, set each thread's
    tick, run them all."""
    words = {}
    for prog, _, _ in programs:
        assert not set(words) & set(prog.words), "the programs overlap"
        words.update(prog.words)
    loom = backend.loom(bench, isa=ISA)
    loom.load(words)
    for _, thread, tick in programs:
        loom.write_csr(thread, "TICK_INT", tick)
        loom.write_csr(thread, "TICK_FRAC", 0)
    loom.run([thread for _, thread, _ in programs])
    return loom


class Wire(Model):
    """A board wire from a chip output to a chip input, one clock late
    (a registered model, like every bench model)."""

    def __init__(self, src, dst):
        self.src, self.dst, self.level = pad_of(src), pad_of(dst), 1

    def drive(self, drive, cycle):
        drive.set(self.dst, self.level)

    def observe(self, lines):
        self.level = lines.get(self.src)


# -------------------------------------------------------------- controller
def controller(backend):
    bench = dmx_bench(backend)
    rx = bench.add(DmxRx("OUT0", limits=TX))
    loom = boot(backend, bench, [(program(CONTROLLER), 0, CTRL_TICK)])
    bench.step(64)
    return bench, loom, rx


def send(loom, start_code, slots, n=None, thread=0):
    loom.push(thread, [start_code, len(slots) if n is None else n] + list(slots))


def assert_controller_timing(rx, packet):
    """E1.11's transmit column (DmxRx.violations, every edge of every slot
    included), and the header's figures, which are exact: every edge lands
    the same few clocks after its tick."""
    assert rx.violations(packet, TX) == []
    assert packet.break_clocks == BREAK, packet.break_clocks
    assert packet.mab_clocks == MAB, packet.mab_clocks
    for slot in packet.slots:
        assert all((cycle - slot.start) % BIT == 0 for cycle, _ in slot.edges)
        assert slot.mark_before % BIT == 0


def test_controller_sends_short_packets_timed_to_e111(backend):
    """Two packets pushed together: the slots back to back (no mark between
    slots), and the second break no sooner than 1204 us after the first,
    the controller padding the short packet with mark before break."""
    bench, loom, rx = controller(backend)
    send(loom, 0, [0x00, 0xFF, 0x5A])
    send(loom, 0, [0x12])
    bench.step(BTB + packet_clocks(1, break_us=176, mab_us=16) + 4 * BIT)
    first, second = rx.packets()
    assert (first.start_code, first.data) == (0, [0x00, 0xFF, 0x5A])
    assert (second.start_code, second.data) == (0, [0x12])
    for packet in (first, second):
        assert_controller_timing(rx, packet)
    assert [s.mark_before for s in first.slots] == [0, 0, 0, 0]
    assert BTB <= first.btb_clocks < BTB + BIT, first.btb_clocks
    assert first.btb_clocks >= TX.btb[0] * US
    assert loom.badop() == 0


def test_controller_waits_at_mark_for_a_late_host(backend):
    """The host feeds the slots late: the line stays at mark between slots
    for whole bits, the stop bits stay whole, and nothing else moves."""
    bench, loom, rx = controller(backend)
    send(loom, 0, [0x11], n=3)
    bench.step(BREAK + MAB + 2 * 11 * BIT + 3000)
    loom.push(0, [0x22])
    bench.step(5000)
    loom.push(0, [0x33])
    bench.step(3 * 11 * BIT)
    (packet,) = rx.packets()
    assert packet.data == [0x11, 0x22, 0x33]
    assert_controller_timing(rx, packet)
    marks = [s.mark_before for s in packet.slots]
    assert marks[:2] == [0, 0] and marks[2] > 0 and marks[3] > 0, marks
    assert loom.badop() == 0


@model_only("a 512-slot packet is 1.1 M clocks; the short packets run the same code")
def test_controller_sends_a_full_universe(backend):
    """512 slots, every one timed; the count is pushed as 600, which the
    header says is sent as 512."""
    bench, loom, rx = controller(backend)
    slots = [(7 * i + 3) & 0xFF for i in range(512)]
    send(loom, 0, slots, n=600)
    bench.step(packet_clocks(512, break_us=176, mab_us=16) + BTB)
    send(loom, 0, [0x99])                   # ends the first packet on the wire
    bench.step(packet_clocks(1, break_us=176, mab_us=16) + 4 * BIT)
    first, second = rx.packets()
    assert first.data == slots and len(first.slots) == 513
    assert_controller_timing(rx, first)
    assert all(s.mark_before == 0 for s in first.slots)
    assert second.data == [0x99]


@model_only("three short packets, 2.6 ms; the RTL time goes to the timing scenarios")
def test_controller_start_codes_and_counts(backend):
    """A non-zero start code with its slots, and the start code alone
    (n = 0, header)."""
    bench, loom, rx = controller(backend)
    send(loom, 0xCC, [1, 2])
    send(loom, 0x17, [], n=0)
    send(loom, 0x55, [0x01])
    bench.step(2 * BTB + packet_clocks(1, break_us=176, mab_us=16) + 4 * BIT)
    packets = rx.packets()
    assert [(p.start_code, p.data) for p in packets] == [(0xCC, [1, 2]), (0x17, []),
                                                         (0x55, [1])]
    for packet in packets:
        assert_controller_timing(rx, packet)


# ----------------------------------------------------------------- fixture
def fixture(backend, first, count, **timing):
    """The fixture on thread 0 with window ``first``, ``count`` and a model
    controller with ``timing`` on IN0."""
    bench = dmx_bench(backend)
    tx = bench.add(DmxTx("IN0", **timing))
    loom = boot(backend, bench, [(program(FIXTURE), 0, FIX_TICK)])
    loom.push(0, [first, count])
    bench.step(400)
    return bench, loom, tx


def status(start_code, slots, flags=0, errors=0):
    """The S1 and S2 words the header gives for a packet."""
    return [S1 | flags | start_code, S2 | (errors << 10) | slots]


def pop(loom, count, clocks=200000):
    return loom.pop(0, count, max_polls=clocks // 100)


def test_fixture_keeps_its_window_and_reports_each_packet(backend):
    """Window 1 .. 3 of a five-slot packet: the three slots as they come,
    then, at the next break, the status (start code 0, five slots)."""
    bench, loom, tx = fixture(backend, 1, 3)
    tx.send_packet(0, [0x01, 0x80, 0xFF, 0x04, 0x05])
    tx.send_packet(0, [0x09])
    assert pop(loom, 3) == [0x01, 0x80, 0xFF]
    assert pop(loom, 2) == status(0, 5)
    assert loom.pop_available(0) == [] and loom.badop() == 0


@model_only("a 512-slot packet is 1.1 M clocks")
def test_fixture_window_at_slot_512(backend):
    """The window's last slot is the universe's last: 510 .. 512. The
    packet ends at its 512th slot, so the status follows at once."""
    bench, loom, tx = fixture(backend, 510, 3)
    slots = [(5 * i + 1) & 0xFF for i in range(512)]
    tx.send_packet(0x00, slots)
    assert pop(loom, 5, 1300000) == slots[509:] + status(0, 512)
    tx.send_packet(0x00, [0x42])            # the next packet is still found
    tx.send_packet(0x00, [0x43])
    assert pop(loom, 2) == status(0, 1)


RX_LIMITS = {
    # E1.11's receive column, both ends: the shortest break and MAB with
    # fast and slow bits and back-to-back slots, then long marks.
    "fast_bits_short_marks": dict(bit_us=3.92, break_us=88.0, mab_us=8.0),
    "slow_bits_short_marks": dict(bit_us=4.08, break_us=88.0, mab_us=8.0),
    "slow_bits_long_marks": dict(bit_us=4.08, break_us=1000.0, mab_us=500.0,
                                 mtbf_us=37.0),
    "fast_bits_typical": dict(bit_us=3.92, break_us=176.0, mab_us=12.0),
}
PATTERN = [0x00, 0xFF, 0x55, 0xAA, 0x01, 0x80, 0x7F, 0xFE] * 3


@model_only("four timing corners of 24 slots; the window test runs on the RTL")
@pytest.mark.parametrize("corner", sorted(RX_LIMITS))
def test_fixture_follows_a_sender_at_the_receive_limits(backend, corner):
    bench, loom, tx = fixture(backend, 1, len(PATTERN), **RX_LIMITS[corner])
    tx.send_packet(0x00, PATTERN)
    tx.send_packet(0x00, [0x01])
    assert pop(loom, len(PATTERN) + 2) == PATTERN + status(0, len(PATTERN))


@model_only("five short packets; the window test runs on the RTL")
def test_fixture_reports_framing_errors_and_a_short_mab(backend):
    """A slot whose stop bit is low (bit 8 of its word, one error in S2),
    a 6 us mark after break (under the receive column's 8 us: MAB), a
    start code whose stop bit is low (S1 bit 8), and a short packet."""
    bench, loom, tx = fixture(backend, 1, 3)
    tx.send_packet(0x00, [0x01, 0x02, 0x03], bad_stop=[2])
    tx.send_packet(0x17, [0x04], mab_us=6.0)
    tx.send_packet(0x55, [0x05], bad_stop=[0])
    tx.send_packet(0x00, [0x06, 0x07])
    tx.send_packet(0x00, [])
    assert pop(loom, 5) == [0x01, 0x02 | FERR, 0x03] + status(0, 3, errors=1)
    assert pop(loom, 3) == [0x04] + status(0x17, 1, MABF)
    assert pop(loom, 3) == [0x05] + status(0x55, 1, SCERR, errors=1)
    assert pop(loom, 4) == [0x06, 0x07] + status(0, 2)


@model_only("a few short packets; the window test runs on the RTL")
def test_fixture_rejects_a_break_under_88_us(backend):
    """A low of 84 us (past the receive column's 88 us minimum) and a
    60 us one: neither is a break. Each ends the packet it falls in, with
    SHORT, and the slots after it are not taken until a real break."""
    bench, loom, tx = fixture(backend, 1, 2)
    tx.send_packet(0x00, [0x0A, 0x0B])
    tx.send_packet(0x00, [0x0C, 0x0D], break_us=84.0)    # not a packet
    tx.send_packet(0x00, [0x0E])
    tx.send_low(60.0)
    tx.send_packet(0x00, [0x0F])
    tx.send_packet(0x00, [])
    assert pop(loom, 4) == [0x0A, 0x0B] + status(0, 2, SHORTF)
    assert pop(loom, 3) == [0x0E] + status(0, 1, SHORTF)
    assert pop(loom, 3) == [0x0F] + status(0, 1)


@model_only("needs the host to stay away for a whole packet")
def test_fixture_drops_words_when_outq_is_full_and_says_so(backend):
    """The host does not pop during an eight-slot window: OUTQ takes four
    words and the fixture, which never stalls, drops the rest and sets
    LOST in that packet's status."""
    bench, loom, tx = fixture(backend, 1, 8)
    tx.send_packet(0x00, list(range(1, 9)))
    bench.step(packet_clocks(8, break_us=176.0, mab_us=12.0) + 2000)
    assert loom.pop_available(0) == [1, 2, 3, 4]
    tx.send_packet(0x00, [0x21])
    tx.send_packet(0x00, [])
    assert pop(loom, 2) == status(0, 8, LOSTF)
    assert pop(loom, 3) == [0x21] + status(0, 1)


# ---------------------------------------------------------------- together
def test_controller_drives_the_fixture(backend):
    """The controller on thread 1 (OUT0) and the fixture on thread 0 (IN0),
    a bench wire between them: the fixture keeps slots 2 .. 3 of what the
    controller sends, and the decoder on the wire times the controller."""
    bench = dmx_bench(backend)
    bench.add(Wire("OUT0", "IN0"))
    rx = bench.add(DmxRx("OUT0", limits=TX))
    loom = boot(backend, bench, [(program(FIXTURE), 0, FIX_TICK),
                                 (program(CONTROLLER, 1), 1, CTRL_TICK)])
    loom.push(0, [2, 2])
    send(loom, 0, [10, 20, 30], thread=1)
    send(loom, 0, [40], thread=1)
    assert pop(loom, 4) == [20, 30] + status(0, 3)
    first = rx.packets()[0]
    assert first.data == [10, 20, 30]
    assert_controller_timing(rx, first)
    assert loom.badop() == 0
