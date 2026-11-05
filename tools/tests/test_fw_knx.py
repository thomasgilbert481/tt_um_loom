"""L3-KNX on the model and the RTL: firmware/knx_device.loom.

One program, two devices: thread t drives TX = OUT<t> (1 = an active
pulse) and reads RX = IN<t>. ``tools.protomodels.knx.KnxBus`` makes the bus
the wired AND of the chip's TX pins (through the transceiver's delay) and
the model nodes, seen by the chip's RX pins.

Expected values come from the KNX TP1 coding, telegram layout and timing
through the reference model (with the public sources it names) and from
the program header (host words and report format), never from what the
model or the RTL did. Pin timing is measured on the pins and the bus: each
chip pulse against the 35 us pulse, each character 13 bit times after the
one before, the acknowledge 15 bit times after the telegram, the idle time
before a send.

Rate: 9600 bit/s is 5208 clocks a bit at 50 MHz (TICK 162 + 195/256, 32
ticks a bit) and a telegram with its answer about a million clocks: that
runs on the model only. The other bodies run the whole bus at a test rate,
TICK 16, a 512-clock bit, with every time in the model nodes scaled with
it (``scale_for``); the program counts in ticks, so only the speed changes.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import knx as K
from tools.tests.fw_backend import model_only

ISA = load()
PROGRAM = REPO / "firmware" / "knx_device.loom"
IMEM = 512

STD = (162, 195)                    # a 32nd of a 9600 bit/s bit at 50 MHz
FAST = (16, 0)                      # the test rate: a 512-clock bit

# the report's status bits (program header)
PARITY, CHECK, OWN, LOST, ADDR, FRAMING = 0x20, 0x40, 0x80, 0x100, 0x200, 0x800
ANS_ACK, ANS_NAK, ANS_BUSY = 0x1000, 0x2000, 0x3000

CHIP_IA = K.individual(1, 1, 10)
CHIP_GA = 0x0901                    # group 1/1/1
OTHER_GA = 0x0902

_IMAGE = None


def image():
    global _IMAGE
    if _IMAGE is None:
        _IMAGE = assemble_file(PROGRAM, isa=ISA, strict=True).words
    return _IMAGE


def tick(rate):
    return rate[0] + rate[1] / 256


def us_of(rate):
    """Clocks per microsecond with the bus scaled to ``rate``."""
    return K.scale_for(32 * tick(rate))


def bit_of(rate):
    return 32 * tick(rate)


def setup(backend, rate, nodes=(), threads=(0,), addresses=None, delay=1, sends=None):
    """The chip at ``rate`` with model ``nodes`` on the bus; thread t gets
    ``addresses[t] = (individual, group)``. Returns ``(bench, loom, bus)``."""
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "DMEM"), imem_words=IMEM)
    else:
        bench = backend.bench()
    pins = [("OUT%d" % t, "IN%d" % t) for t in threads]
    bus = bench.add(K.KnxBus(us_of(rate), nodes, tx=[p[0] for p in pins],
                             rx=[p[1] for p in pins], delay=delay))
    loom = backend.loom(bench, isa=ISA)
    loom.load(image())
    addresses = addresses or {0: (CHIP_IA, CHIP_GA)}
    for t in threads:
        loom.write_csr(t, "TICK_INT", rate[0])
        loom.write_csr(t, "TICK_FRAC", rate[1])
        loom.push(t, list(addresses[t]) + list((sends or {}).get(t, [])))
    loom.run(list(threads))
    bits(bench, rate, 2)            # the threads reach their idle loop
    return bench, loom, bus


def bits(bench, rate, n):
    bench.step(int(n * bit_of(rate)))


def report(loom, rate, thread=0, bits_to_wait=400):
    """One report: ``(status, bytes)``."""
    polls = int(bits_to_wait * bit_of(rate)) // 40 + 2000
    status = loom.pop(thread, 1, max_polls=polls)[0]
    n = status & 0x1F
    words = loom.pop(thread, (n + 1) // 2, max_polls=polls) if n else []
    data = []
    for w in words:
        data += [w >> 8, w & 0xFF]
    return status & ~0x1F, data[:n]


def send(loom, thread, dst, apci, ctrl=K.CTRL_LOW):
    loom.push(thread, [dst, (ctrl << 8) | apci])


def chip_answers(bus, us):
    """The answer characters on the bus and their gaps after the telegram before."""
    tels = bus.decoder.telegrams
    return [(b.data, K.answer_gap(a, b, us)) for a, b in zip(tels, tels[1:])
            if len(b.chars) == 1]


# ------------------------------------------------------------ receiving
@model_only("on the RTL test_arbitration_lost receives, checks and ACKs a model telegram to its group")
def test_receive_and_ack(backend):
    """A model device sends a GroupValue_Write to the chip's group: the
    chip reports it ADDRESSED with all nine bytes, and its ACK starts 15
    bit times after the telegram, every pulse 35 us."""
    us = us_of(FAST)
    node = K.KnxNode(us, address=K.individual(1, 1, 1))
    tel = K.group_write(node.address, CHIP_GA, 0x81)
    bench, loom, bus = setup(backend, FAST, [node])
    node.send(tel)
    status, data = report(loom, FAST)
    assert status == ADDR, hex(status)
    assert data == tel
    bits(bench, FAST, 5)
    assert node.sent and node.sent[0].done
    answers = chip_answers(bus, us)
    assert len(answers) == 1 and answers[0][0] == [K.ACK]
    assert K.ACK_GAP[0] <= answers[0][1] <= K.ACK_GAP[1], answers
    assert bus.chip_pulses() and K.check_pulses(bus.chip_pulses(), us) == []
    assert loom.badop() == 0


def test_nak_on_a_bad_check_byte(backend):
    """The same telegram with its check byte wrong: CHECK in the report,
    and the chip answers NAK on time."""
    us = us_of(FAST)
    node = K.KnxNode(us, address=K.individual(1, 1, 1))
    tel = K.group_write(node.address, CHIP_GA, 0x80)
    bench, loom, bus = setup(backend, FAST, [node])
    node.send(tel, bad_check=True)
    status, data = report(loom, FAST)
    assert status == ADDR | CHECK, hex(status)
    assert data == tel[:-1] + [tel[-1] ^ 1]
    bits(bench, FAST, 5)
    answers = chip_answers(bus, us)
    assert len(answers) == 1 and answers[0][0] == [K.NAK]
    assert K.ACK_GAP[0] <= answers[0][1] <= K.ACK_GAP[1], answers


@model_only("the parity check is the same path as the check-byte case on the RTL")
def test_nak_on_bad_parity(backend):
    """A character with its parity bit flipped: PARITY, and NAK."""
    us = us_of(FAST)
    node = K.KnxNode(us, address=K.individual(1, 1, 1))
    bench, loom, bus = setup(backend, FAST, [node])
    node.send(K.group_write(node.address, CHIP_GA, 0x81), bad_parity_at=7)
    status, _ = report(loom, FAST)
    assert status == ADDR | PARITY, hex(status)
    bits(bench, FAST, 5)
    assert [a[0] for a in chip_answers(bus, us)] == [[K.NAK]]


@model_only("address filtering is decided by the same compare as the ACK case")
def test_other_group_is_reported_not_answered(backend):
    """A telegram to another group: reported without ADDRESSED, no answer
    from the chip (the model node answers nothing either)."""
    us = us_of(FAST)
    node = K.KnxNode(us, address=K.individual(1, 1, 1))
    tel = K.group_write(node.address, OTHER_GA, 0x81)
    bench, loom, bus = setup(backend, FAST, [node])
    node.send(tel)
    status, data = report(loom, FAST, bits_to_wait=500)
    assert status == 0, hex(status)
    assert data == tel
    assert chip_answers(bus, us) == [] and bus.chip_pulses() == []


# ------------------------------------------------------------ sending
@model_only("on the RTL test_arbitration_won sends, is decoded by the model and ACKed")
def test_send_is_decoded_and_acked(backend):
    """The host asks for a GroupValue_Write to 1/1/1: the bus carries the
    exact telegram, 13 bit times a character, every pulse 35 us, after at
    least 50 bit times of idle bus; a model device in the group ACKs it and
    the report says OWN with answer ACK."""
    us = us_of(FAST)
    node = K.KnxNode(us, address=K.individual(1, 1, 2), groups=[0x0901])
    bench, loom, bus = setup(backend, FAST, [node])
    send(loom, 0, 0x0901, 0x81)
    status, data = report(loom, FAST, bits_to_wait=600)
    expected = K.group_write(CHIP_IA, 0x0901, 0x81)
    assert status == OWN | ANS_ACK, hex(status)
    assert data == expected
    tel = bus.decoder.telegrams[0]
    assert tel.data == expected and tel.ok
    assert K.check_chars(tel, us, previous_end=0) == []
    assert K.check_pulses(bus.chip_pulses(), us) == []
    assert node.answers and node.answers[0][1] == K.ACK


# ------------------------------------------------------------ arbitration
def collide(backend, node_address):
    """The chip sends to 1/1/1 and a model device in 1/1/1 starts its own
    telegram to 1/1/1 in the same clock as the chip's start bit."""
    us = us_of(FAST)
    node = K.KnxNode(us, address=node_address, groups=[CHIP_GA])
    node_tel = K.group_write(node.address, CHIP_GA, 0x80)
    bench, loom, bus = setup(backend, FAST, [node])
    node.send(node_tel, with_chip=True)
    send(loom, 0, CHIP_GA, 0x81)
    return us, bench, loom, bus, node, node_tel


def test_arbitration_won(backend):
    """Source 1.1.10 against 1.1.11: the first bit that differs is bit 0
    of the source's low byte, where the chip sends 0, so the model device
    loses there and the chip's telegram goes through untouched and is
    ACKed. The device then sends its own after the idle time; the chip
    receives and ACKs it."""
    us, bench, loom, bus, node, node_tel = collide(backend, K.individual(1, 1, 11))
    mine = K.group_write(CHIP_IA, CHIP_GA, 0x81)
    assert report(loom, FAST, bits_to_wait=600) == (OWN | ANS_ACK, mine)
    assert report(loom, FAST, bits_to_wait=600) == (ADDR, node_tel)
    assert node.sent[0].losses == 1 and node.sent[-1].done
    assert node.sent[0].lost_bits == [(2, 1)]   # character 2, data bit 0
    tels = [t for t in bus.decoder.telegrams if len(t.chars) > 1]
    assert [t.data for t in tels] == [mine, node_tel] and all(t.ok for t in tels)
    assert K.check_chars(tels[0], us) == []
    answers = chip_answers(bus, us)             # the device ACKs the chip, the chip the device
    assert [a[0] for a in answers] == [[K.ACK], [K.ACK]], answers
    assert all(K.ACK_GAP[0] <= g <= K.ACK_GAP[1] for _, g in answers), answers
    assert K.check_pulses(bus.chip_pulses(), us) == []


def test_arbitration_lost(backend):
    """Source 1.1.10 against 1.1.8: bit 1 of the low byte is 1 for the
    chip and 0 for the device, so the chip loses there, sends nothing more,
    receives the device's telegram (its group: ADDRESSED and LOST, ACKed),
    and sends its own again after 50 bit times of idle bus."""
    us, bench, loom, bus, node, node_tel = collide(backend, K.individual(1, 1, 8))
    mine = K.group_write(CHIP_IA, CHIP_GA, 0x81)
    assert report(loom, FAST, bits_to_wait=600) == (ADDR | LOST, node_tel)
    assert report(loom, FAST, bits_to_wait=600) == (OWN | ANS_ACK, mine)
    assert node.sent[-1].done and node.sent[-1].losses == 0
    tels = [t for t in bus.decoder.telegrams if len(t.chars) > 1]
    assert [t.data for t in tels] == [node_tel, mine] and all(t.ok for t in tels)
    # the retry: at least 50 bit times after the ACK of the first telegram
    ack = [t for t in bus.decoder.telegrams if len(t.chars) == 1][0]
    assert K.check_chars(tels[1], us, previous_end=K.end_of(ack, us)) == []
    assert K.check_chars(tels[0], us) == [] and K.check_chars(tels[1], us) == []
    # the chip's ACK of the device's telegram, on time, and every chip pulse 35 us
    answers = chip_answers(bus, us)
    assert answers[0][0] == [K.ACK] and K.ACK_GAP[0] <= answers[0][1] <= K.ACK_GAP[1], answers
    assert K.check_pulses(bus.chip_pulses(), us) == []
    first = [r for r, _ in bus.chip_pulses() if tels[0].chars[0].start <= r < K.end_of(tels[0], us)]
    assert first and max(first) < tels[0].chars[2].start + 2 * bit_of(FAST)   # none after the lost bit


# ------------------------------------------------------------ two devices
TWO = {0: (K.individual(1, 1, 10), CHIP_GA), 1: (K.individual(1, 1, 11), CHIP_GA)}


@model_only("about 370 bit times of bus with two threads; the RTL runs the one-thread collisions")
def test_two_threads_arbitrate_and_ack_each_other(backend):
    """The same program on threads 0 and 1 (OUT0/IN0 and OUT1/IN1, wired
    into one bus) and a model device in the group. Both are given a send
    before RUN, so they start in the same slot round: thread 1 (1.1.11)
    loses to thread 0 (1.1.10) at bit 0 of the source's low byte, receives
    thread 0's telegram and ACKs it together with the model device, then
    sends its own after the idle time, which thread 0 and the device ACK."""
    us = us_of(FAST)
    node = K.KnxNode(us, address=K.individual(1, 1, 30), groups=[CHIP_GA])
    w = [CHIP_GA, (K.CTRL_LOW << 8) | 0x81]
    bench, loom, bus = setup(backend, FAST, [node], threads=(0, 1), addresses=TWO,
                             sends={0: w, 1: w})
    t0 = K.group_write(TWO[0][0], CHIP_GA, 0x81)
    t1 = K.group_write(TWO[1][0], CHIP_GA, 0x81)
    assert report(loom, FAST, 0, 600) == (OWN | ANS_ACK, t0)
    assert report(loom, FAST, 1, 600) == (ADDR | LOST, t0)
    assert report(loom, FAST, 1, 600) == (OWN | ANS_ACK, t1)
    assert report(loom, FAST, 0, 600) == (ADDR, t1)
    tels = bus.decoder.telegrams
    assert [t.data for t in tels] == [t0, [K.ACK], t1, [K.ACK]]
    assert all(t.ok for t in (tels[0], tels[2]))
    for a, b in ((tels[0], tels[1]), (tels[2], tels[3])):
        assert K.ACK_GAP[0] <= K.answer_gap(a, b, us) <= K.ACK_GAP[1]
    assert K.check_pulses(bus.chip_pulses(), us) == []
    # thread 1 drove OUT1 in the first telegram up to the bit it lost only
    lost_at = tels[0].chars[2].start + 2 * bit_of(FAST)
    late = [r for r, _ in bus.tx_pulses[("uo", 1)] if tels[0].chars[0].start <= r < K.end_of(tels[0], us)]
    assert late and max(late) < lost_at
    assert loom.badop() == 0


# ------------------------------------------------------------ the answer
@model_only("the answer decoding shares the receive path the RTL runs")
@pytest.mark.parametrize("answer, code", [(K.NAK, ANS_NAK), (K.BUSY, ANS_BUSY), (None, 0)])
def test_own_telegram_answers(backend, answer, code):
    """NAK, BUSY and no answer from the addressed device are reported as
    such and not repeated (program header)."""
    us = us_of(FAST)
    node = K.KnxNode(us, address=K.individual(1, 1, 2), groups=[CHIP_GA], answer=answer)
    bench, loom, bus = setup(backend, FAST, [node])
    send(loom, 0, CHIP_GA, 0x80)
    status, data = report(loom, FAST, bits_to_wait=600)
    assert status == OWN | code, hex(status)
    bits(bench, FAST, 120)
    assert len([t for t in bus.decoder.telegrams if len(t.chars) > 1]) == 1


@model_only("the read-back path is the send test's, which runs on the RTL; the delay is a model parameter")
@pytest.mark.parametrize("delay_us", [10.0])
def test_transceiver_delay(backend, delay_us):
    """A transceiver that delays the chip's TX by 10 us on its way to RX:
    the chip still reads its own pulses back (it samples 22.8 us into the
    bit) and does not take them for a lost arbitration."""
    us = us_of(FAST)
    node = K.KnxNode(us, address=K.individual(1, 1, 2), groups=[CHIP_GA])
    bench, loom, bus = setup(backend, FAST, [node], delay=int(delay_us * us))
    send(loom, 0, CHIP_GA, 0x81)
    assert report(loom, FAST, bits_to_wait=600) == (OWN | ANS_ACK, K.group_write(CHIP_IA, CHIP_GA, 0x81))


@model_only("9600 bit/s: about a million clocks a telegram")
def test_standard_rate(backend):
    """At 9600 bit/s on a 50 MHz chip (TICK 162 + 195/256): receive and ACK,
    then send and be ACKed, every pulse and gap measured in microseconds."""
    us = us_of(STD)
    assert abs(us - 50) < 0.01
    node = K.KnxNode(us, address=K.individual(1, 1, 1), groups=[CHIP_GA])
    bench, loom, bus = setup(backend, STD, [node])
    tel = K.group_write(node.address, CHIP_GA, 0x81)
    node.send(tel)
    assert report(loom, STD, bits_to_wait=300) == (ADDR, tel)
    send(loom, 0, CHIP_GA, 0x80)
    assert report(loom, STD, bits_to_wait=600) == (OWN | ANS_ACK, K.group_write(CHIP_IA, CHIP_GA, 0x80))
    tels = bus.decoder.telegrams
    assert [len(t.chars) for t in tels] == [9, 1, 9, 1]
    for a, b in ((tels[0], tels[1]), (tels[2], tels[3])):
        assert K.ACK_GAP[0] <= K.answer_gap(a, b, us) <= K.ACK_GAP[1]
    assert K.check_chars(tels[2], us, previous_end=K.end_of(tels[1], us)) == []
    widths = [w / us for _, w in bus.chip_pulses()]
    assert widths and all(35.0 <= w <= 36.0 for w in widths), widths


# ------------------------------------------------------------ the build
def test_build():
    """--strict with no diagnostic; the layout of the program header: code
    in 0x000..0x17F with thread 1's reset vector a jump to ``start``, the
    data blocks of threads 0 and 1 at 0x180 and 0x1C0 left free."""
    prog = assemble_file(PROGRAM, isa=ISA, strict=True)
    assert prog.diagnostics == []
    addrs = sorted(prog.words)
    assert addrs[0] == 0 and addrs[-1] < 0x180
    assert len(addrs) == 282
    instr, fields = ISA.decode(prog.words[0x080])
    assert instr.name == "JMP" and fields["abs"] == 0, fields


@model_only("a rate sweep of the receive path that test_receive_and_ack runs on the RTL")
@pytest.mark.parametrize("skew", [-0.005, 0.02])
def test_receive_from_an_off_rate_sender(backend, skew):
    """The grid restarts on every start pulse and samples 7 to 9 ticks
    after it, inside a 10.75-tick pulse: a sender 0.5 % fast (its stop bit's
    pulse ends 1.6 ticks early) or 2 % slow (it starts 6.4 ticks late) is
    still read. The limits measured, not claimed beyond: see the header."""
    us = us_of(FAST)
    node = K.KnxNode(us, address=K.individual(1, 1, 1), skew=skew)
    tel = K.group_write(node.address, CHIP_GA, 0x81)
    bench, loom, bus = setup(backend, FAST, [node])
    node.send(tel)
    assert report(loom, FAST) == (ADDR, tel)
