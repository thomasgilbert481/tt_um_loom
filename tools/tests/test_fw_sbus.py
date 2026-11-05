"""L3-SBUS on the model and the RTL: firmware/sbus_rx.loom, sbus_tx.loom and
bridge_sbus_servo.loom.

Futaba S.BUS: 25-byte frames of inverted 8E2 at 100000 baud, 16 channels
of 11 bits, a flags byte; ``tools.protomodels.sbus`` is the reference model
(frame packing, the line encoder ``SbusTx`` with error injection, the
decoder ``SbusRx``, the servo pulse meter), written from the public decoder
and encoder sources its docstring cites. Expected values come from those
sources through the model, from the programs' headers (the status words,
the counters, the data layout, the frame period, the servo scaling) and
from SEMANTICS (one slot every 4 clocks), never from what the model or the
RTL did.

Rates at 50 MHz: S.BUS is 500 clocks a bit; the RTL scenarios run it at
128 clocks a bit (390.6 kbaud, the fastest the headers prove: RX tick 16,
TX tick 128) and the servo tick at 16 clocks (a "microsecond" of 16
clocks, with the servo frame shortened through FRAMEW). The model runs the
real rates: 100000 baud with a sender 2 % off, 14 ms and 7 ms periods, 1 us
servo ticks and the 20 ms servo frame.

The RTL runs three scenarios: the receiver fed by the model (a good frame,
then a frame-lost one), the transmitter and receiver on two threads with
OUT0 wired to IN0 (TX timed, RX decoding it), and the bridge with all 14
servo pulses measured. The rejections, failsafe, out-of-sync start, the
real rates and the bridge's failsafe hold run on the model only.

Every body that takes a ``backend`` (``tools/tests/fw_backend.py``) runs
on the golden model under pytest and on the RTL under ``test/test_fw.py``.
"""

import math

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import sbus
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.sbus import SbusRx, SbusTx, ServoMeter
from tools.tests.fw_backend import model_only

ISA = load()
IMEM = 512
FW = REPO / "firmware"
RX = assemble_file(FW / "sbus_rx.loom", isa=ISA, imem_words=IMEM, strict=True)
TX = assemble_file(FW / "sbus_tx.loom", isa=ISA, imem_words=IMEM, strict=True)
BRIDGE = assemble_file(FW / "bridge_sbus_servo.loom", isa=ISA, imem_words=IMEM, strict=True)

BIT = 128                       # clocks a bit in the RTL scenarios
RX_TICK = BIT // 8              # sbus_rx: eight ticks a bit (header)
TX_TICK = BIT                   # sbus_tx: one tick a bit (header)
REAL_BIT = 50e6 / sbus.BAUD     # 500 clocks at 50 MHz
GAP_BITS = 48                   # sbus_rx's frame gap (header)
SERVO_TICK = 16                 # bridge servo threads in the RTL scenario

# sbus_rx's data memory (header).
STG, CH, FLAGS, GOOD, LOST, FSAFE, BAD = 0x100, 0x110, 0x120, 0x121, 0x122, 0x123, 0x124
# The reasons of a bad frame (sbus_rx header).
PERR, SERR, HDR, FTR, SHORT = 0x01, 0x02, 0x04, 0x08, 0x10
# sbus_tx's data memory (header).
CHT, FLAGT, PERW = 0x180, 0x190, 0x191
# The bridge's (header): sbus_rx's, 0x80 higher, and the servo frame.
B_CH, B_GOOD, B_FSAFE, FRAMEW = 0x190, 0x1A1, 0x1A3, 0x1F0


def features(backend, **kwargs):
    """FIFOs, the bit engine and LD/ST with the 512-word memory on the
    model; the RTL has all of them and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM, **kwargs)
    return backend.bench(**kwargs)


class Wire(Model):
    """The board wire from OUT0 to IN0, one clock late; low (S.BUS idle)
    before the first cycle."""

    def __init__(self, src="OUT0", dst="IN0"):
        self.src, self.dst, self.level = pad_of(src), pad_of(dst), 0

    def drive(self, drive, cycle):
        drive.set(self.dst, self.level)

    def observe(self, lines):
        self.level = lines.get(self.src)


def start(backend, bench, images, ticks, threads):
    """Load ``images`` (merged), set each thread's tick, run ``threads``."""
    words = {}
    for image in images:
        words.update(image)
    loom = backend.loom(bench, isa=ISA)
    loom.load(words)
    for thread, (tick_int, tick_frac) in ticks.items():
        loom.write_csr(thread, "TICK_INT", tick_int)
        loom.write_csr(thread, "TICK_FRAC", tick_frac)
    loom.run(list(threads), exclusive=True)
    return loom


def memory(loom, addr, count=1):
    """Data memory words, read with every thread halted (IMEM space)."""
    loom.halt([0, 1, 2, 3])
    return loom.read_imem(addr, count)


def channels(seed, n=16):
    """Sixteen 11-bit channel values that differ in every bit position."""
    return [(seed * 131 + k * 389 + (k << 7)) % 2048 for k in range(n)]


def good_word(flags, lost_count):
    """sbus_rx's status word for a good frame (header)."""
    return (flags & 0xFF) | ((lost_count & 0x7F) << 8)


# ================================================================ sbus_rx
def boot_rx(backend, *, bit=BIT, tick=(RX_TICK, 0)):
    """sbus_rx on thread 0 with an S.BUS sender on IN0, running."""
    bench = features(backend)
    tx = bench.add(SbusTx("IN0", bit))
    loom = start(backend, bench, [RX.words], {0: tick}, [0])
    bench.step(64)
    return bench, loom, tx


def settle(bench, tx, bit=BIT):
    """Run until the sender is done and the frame is decided (the 25th
    byte's copy and report take about 100 slots, under 4 bits at 128)."""
    assert bench.run_until(lambda: not tx.busy, 2000000, every=500)
    bench.step(int(8 * bit))


def after_gap(tx, bench, bits=60, bit=BIT):
    """The next frame starts ``bits`` bit times after the last one ended
    (or after now): more than the receiver's 48-bit gap."""
    end = tx.frames_sent[-1][1] if tx.frames_sent else bench.cycle
    tx.idle_until(max(end, bench.cycle) + bits * bit)


def test_sbus_rx_takes_frames_and_counts_frame_lost(backend):
    """After a gap, frame A (no flags), 60 bits later frame B with frame
    lost and channel 18 set, every byte back to back: two good-frame
    words (the second counting one frame lost), CH holding B, the flags
    and the counters as the header lays them out."""
    bench, loom, tx = boot_rx(backend)
    a, b = channels(1), channels(2)
    after_gap(tx, bench)
    tx.send(a)
    after_gap(tx, bench)
    tx.send(b, sbus.FRAME_LOST | sbus.CH18)
    settle(bench, tx)
    assert loom.pop(0, 2) == [good_word(0, 0), good_word(0x06, 1)]
    mem = memory(loom, CH, 21)
    assert mem[:16] == b, mem[:16]
    assert mem[16:] == [0x06, 2, 1, 0, 0]       # FLAGS, GOOD, LOST, FSAFE, BAD
    assert loom.badop() == 0


@model_only("six frames in a row; the RTL scenario runs the same receiver")
def test_sbus_rx_rejects_bad_frames_and_holds_channels_on_failsafe(backend):
    """A good frame, then one each with a parity error, a high stop bit, a
    wrong first byte, a wrong last byte, 20 bytes only (a short frame),
    then a good frame with the failsafe bit: one bad-frame word each with
    exactly its reason, the failsafe frame a good word, and CH still the
    first frame's channels."""
    bench, loom, tx = boot_rx(backend)
    a, b = channels(3), channels(4)
    words = []

    def frame(data, **kwargs):
        after_gap(tx, bench)
        tx.send_bytes(data, **kwargs)
        settle(bench, tx)
        words.extend(loom.pop_available(0))

    good, bad_b = sbus.pack(a), sbus.pack(b)
    frame(good)
    frame(bad_b, bad_parity=[5])
    frame(bad_b, stop_low=[7])
    frame([0x0E] + bad_b[1:])
    frame(bad_b[:-1] + [0x04])
    frame(bad_b[:20])
    after_gap(tx, bench)                         # the short frame's gap first
    bench.step(GAP_BITS * BIT)
    words.extend(loom.pop_available(0))
    frame(sbus.pack(b, sbus.FAILSAFE))
    assert words == [good_word(0, 0), 0x8000 | PERR, 0x8000 | SERR, 0x8000 | HDR,
                     0x8000 | FTR, 0x8000 | SHORT, good_word(sbus.FAILSAFE, 0)], \
        [hex(w) for w in words]
    mem = memory(loom, CH, 21)
    assert mem[:16] == a
    assert mem[16:] == [sbus.FAILSAFE, 1, 0, 1, 5]


@model_only("an out-of-sync start; the RTL scenario starts on an idle line")
def test_sbus_rx_ignores_a_frame_it_did_not_see_begin(backend):
    """A frame that starts before the receiver has seen 48 bits of idle
    is not taken (it might be the tail of one), and is not reported; the
    next, after a gap, is."""
    bench, loom, tx = boot_rx(backend)
    a, b = channels(5), channels(6)
    tx.send(a)                                   # at once: no gap seen yet
    after_gap(tx, bench)
    tx.send(b)
    settle(bench, tx)
    assert loom.pop_available(0) == [good_word(0, 0)]
    mem = memory(loom, CH, 21)
    assert mem[:16] == b and mem[16:] == [0, 1, 0, 0, 0]


@model_only("100000 baud is 500 clocks a bit; the RTL runs 128")
@pytest.mark.parametrize("error", [-0.02, 0.02])
def test_sbus_rx_at_100000_baud_7ms_frames_with_a_rate_error(backend, error):
    """Two frames 7 ms apart (the high-speed rate) from a sender 2 % fast
    or slow, at 100000 baud with the header's tick (62 + 128/256): both
    taken, the second's channels in CH."""
    bit = REAL_BIT * (1 + error)
    bench, loom, tx = boot_rx(backend, bit=bit, tick=(62, 128))
    a, b = channels(7), channels(8)
    after_gap(tx, bench, bit=bit)
    first = tx.send(a)
    tx.idle_until(first + 350000)               # 7 ms at 50 MHz
    tx.send(b, sbus.CH17)
    settle(bench, tx, bit=bit)
    assert loom.pop(0, 2) == [good_word(0, 0), good_word(sbus.CH17, 0)]
    assert memory(loom, CH, 16) == b


# ================================================================ sbus_tx
def tx_image(chans=None, flags=0, period=None):
    """sbus_tx's image with its table, flags and PERIOD words replaced."""
    words = dict(TX.words)
    if chans is not None:
        words.update({CHT + k: v for k, v in enumerate(chans)})
    words[FLAGT] = flags
    if period is not None:
        words[PERW] = period
    return words


def frames_ok(rx, n):
    """The first ``n`` complete frames decoded on OUT0: 25 bytes each, each
    with good parity and stop bits, back to back (every start edge 12 bits
    after the last, exactly: a tick that is a multiple of 4 clocks puts
    every edge the same few clocks after its tick, SEMANTICS 2), every edge
    on the bit grid."""
    frames = rx.complete_frames()[:n]
    assert len(frames) == n, len(frames)
    for f in frames:
        assert f.ok and len(f.bytes) == sbus.FRAME_BYTES, f.data
        for prev, b in zip(f.bytes, f.bytes[1:]):
            assert b.start - prev.start == sbus.BITS_PER_BYTE * rx.bit_clocks
        for b in f.bytes:
            assert all(off == 0 for _, off in rx.edge_offsets(b)), rx.edge_offsets(b)
    return frames


@model_only("14 ms frames at 128 clocks a bit are 180 k clocks each")
@pytest.mark.parametrize("period", [1400, 700])
def test_sbus_tx_sends_frames_at_14_and_7_ms(backend, period):
    """At PERIOD 1400 (14 ms at 100000 baud, the image's default) and 700
    (7 ms): three frames, each the image's 16 centre channels and no
    flags, starting PERIOD ticks apart exactly, the line idle low between."""
    bench = features(backend)
    rx = bench.add(SbusRx("OUT0", BIT))
    loom = start(backend, bench, [tx_image(period=None if period == 1400 else period)],
                 {1: (TX_TICK, 0)}, [1])
    limit = (60 + 3 * period) * BIT
    assert bench.run_until(lambda: len(rx.complete_frames()) >= 3, limit, every=1000)
    frames = frames_ok(rx, 3)
    for f in frames:
        assert sbus.unpack(f.data) == ([sbus.CENTRE] * 16, 0)
    assert [b.start - a.start for a, b in zip(frames, frames[1:])] == [period * BIT] * 2
    assert len(rx.bytes) == 75 and loom.badop() == 0


@model_only("a sweep of PERIOD values below the build time")
def test_sbus_tx_raises_a_period_too_short_for_the_build(backend):
    """PERIOD 320 is under the 352 the header raises it to: frames 352
    ticks apart."""
    bench = features(backend)
    rx = bench.add(SbusRx("OUT0", BIT))
    start(backend, bench, [tx_image(period=320)], {1: (TX_TICK, 0)}, [1])
    assert bench.run_until(lambda: len(rx.complete_frames()) >= 3, 1500 * BIT, every=1000)
    frames = frames_ok(rx, 3)
    assert [b.start - a.start for a, b in zip(frames, frames[1:])] == [352 * BIT] * 2


# ============================================== sbus_rx and sbus_tx together
def test_sbus_tx_to_rx_on_two_threads_through_a_wire(backend):
    """sbus_tx on thread 1 (PERIOD 700, the high-speed rate; channels A,
    flags channel 17 and frame lost) drives OUT0, wired to IN0, where
    sbus_rx on thread 0 takes the frames: the frames on OUT0 are A, 700
    ticks apart and on the bit grid; the receiver reports both, counting
    the frame-lost flag, and its CH is A."""
    a = channels(9)
    flags = sbus.CH17 | sbus.FRAME_LOST
    bench = features(backend)
    bench.add(Wire())
    rx = bench.add(SbusRx("OUT0", BIT))
    loom = start(backend, bench, [RX.words, tx_image(a, flags, 700)],
                 {0: (RX_TICK, 0), 1: (TX_TICK, 0)}, [0, 1])
    assert bench.run_until(lambda: len(rx.complete_frames()) >= 2, 1200 * BIT, every=1000)
    bench.step(8 * BIT)                         # the receiver decides the second
    frames = frames_ok(rx, 2)
    for f in frames:
        assert sbus.unpack(f.data) == (a, flags)
    assert frames[1].start - frames[0].start == 700 * BIT
    assert loom.pop(0, 2) == [good_word(flags, 1), good_word(flags, 2)]
    mem = memory(loom, CH, 21)
    assert mem[:16] == a and mem[16:] == [flags, 2, 2, 0, 0]
    assert loom.badop() == 0


# ====================================================== bridge_sbus_servo
#: Channels 1 .. 16 of the bridge scenarios: both clamps, the centre and
#: values in between; 15 and 16 are not driven (the header's 14 outputs).
RAW = [0, 173, 600, 992, 1200, 1500, 1812, 2047,
       100, 500, 992, 1400, 1811, 2047, 992, 992]
OTHER = [2047 - v for v in RAW]
SERVOS = ["BIDIR%d" % i for i in range(8)] + ["OUT%d" % i for i in range(6)]
FRAME_RTL = 13000               # FRAMEW in the RTL scenario: over the 8 widths of
                                # RAW on BIDIR0 .. 7 (12202 ticks) and the 100-tick set-up


def boot_bridge(backend, *, bit=BIT, rx_tick=(RX_TICK, 0), servo_tick=SERVO_TICK,
                framew=FRAME_RTL):
    """The bridge with an S.BUS sender on IN0 and a pulse meter on each of
    the 14 servo pins; the three threads running."""
    bench = features(backend)
    tx = bench.add(SbusTx("IN0", bit))
    meters = [bench.add(ServoMeter(pin)) for pin in SERVOS]
    image = dict(BRIDGE.words)
    if framew is not None:
        image[FRAMEW] = framew
    loom = start(backend, bench, [image],
                 {0: rx_tick, 1: (servo_tick, 0), 2: (servo_tick, 0)}, [0, 1, 2])
    bench.step(64)
    return bench, loom, tx, meters


def check_servo_frame(meters, raw, index, us_clocks):
    """Pulse ``index`` of every servo has the width ``raw`` asks for
    (protomodels.sbus.servo_violations), and each group's pulses follow
    one another with no gap and no overlap: a servo's rise is the cycle
    the one before it falls (one OUT write ends one and starts the next)."""
    for k, meter in enumerate(meters):
        assert servo_violations_of(meter, raw[k], index, us_clocks) == [], (SERVOS[k], meter.pulses)
    for group in (meters[:8], meters[8:]):
        for prev, nxt in zip(group, group[1:]):
            assert nxt.pulses[index][0] == prev.pulses[index][1]


def servo_violations_of(meter, raw, index, us_clocks):
    return sbus.servo_violations([meter.pulses[index]], raw, us_clocks)


def test_bridge_drives_14_servos_from_one_sbus_frame(backend):
    """One S.BUS frame (channels RAW: the clamps at both ends, the centre)
    after a gap: before it is taken no servo pin moves; after it, one servo
    frame on all 14 pins, each pulse 880 + 5 * raw / 8 us of 16-clock
    ticks (Betaflight's scaling, raw clamped to 173 .. 1812), the eight of
    BIDIR0 .. 7 one after the other, the six of OUT0 .. 5 likewise."""
    bench, loom, tx, meters = boot_bridge(backend)
    after_gap(tx, bench)
    tx.send(RAW)
    end = tx.frames_sent[-1][1]
    limit = end + (1000 + FRAME_RTL) * SERVO_TICK
    done = lambda: meters[7].pulses and meters[13].pulses
    assert bench.run_until(done, limit, every=2000)
    for meter in meters:
        assert meter.edges and meter.edges[0][0] > end, "a pulse before the frame"
    check_servo_frame(meters, RAW, 0, SERVO_TICK)
    assert loom.pop(0, 1) == [good_word(0, 0)]
    assert loom.badop() == 0


@model_only("three servo frames of 13000 ticks, 0.6 M clocks")
def test_bridge_holds_the_servos_through_a_failsafe_frame(backend):
    """Frame A, then, while A's servo frame is going out, a frame with the
    failsafe bit and other channels: the next servo frame is A's again,
    FRAMEW ticks after the first (the hold), and the receiver counted the
    failsafe frame (FSAFE 1) without touching CH."""
    bench, loom, tx, meters = boot_bridge(backend)
    after_gap(tx, bench)
    tx.send(RAW)
    assert bench.run_until(lambda: meters[0].pulses, 400000, every=2000)
    after_gap(tx, bench)
    tx.send(OTHER, sbus.FAILSAFE)
    assert bench.run_until(lambda: len(meters[7].pulses) >= 2 and len(meters[13].pulses) >= 2,
                           3 * FRAME_RTL * SERVO_TICK, every=2000)
    check_servo_frame(meters, RAW, 0, SERVO_TICK)
    check_servo_frame(meters, RAW, 1, SERVO_TICK)
    for meter in meters:
        assert meter.pulses[1][0] - meter.pulses[0][0] == FRAME_RTL * SERVO_TICK
    assert loom.pop(0, 2) == [good_word(0, 0), good_word(sbus.FAILSAFE, 0)]
    assert memory(loom, B_CH, 16) == RAW
    assert memory(loom, B_FSAFE, 1) == [1]


@model_only("100000 baud and a 20 ms servo frame of 1 us ticks: 1.3 M clocks")
def test_bridge_at_full_rate_20ms_servo_frames(backend):
    """The real rates: S.BUS at 100000 baud (tick 62 + 128/256), servo
    ticks of 1 us (TICK_INT 50) and the image's 20 ms servo frame: every
    pulse 880 + 5 * raw / 8 us to the microsecond, the next frame's
    first pulse 20 ms after the first's."""
    bench, loom, tx, meters = boot_bridge(backend, bit=REAL_BIT, rx_tick=(62, 128),
                                          servo_tick=50, framew=None)
    after_gap(tx, bench, bit=REAL_BIT)
    tx.send(RAW)
    assert bench.run_until(lambda: len(meters[0].pulses) >= 2 and len(meters[8].pulses) >= 2,
                           1600000, every=5000)
    check_servo_frame(meters, RAW, 0, 50)
    for k in (0, 8):
        assert meters[k].pulses[1][0] - meters[k].pulses[0][0] == 20000 * 50
        assert sbus.servo_violations(meters[k].pulses[1:2], RAW[k], 50) == []


def test_sbus_programs_assemble_strict_with_every_pair_proved():
    """--strict and no diagnostic; every deadline pair proved for the
    ticks the headers give; the layouts the headers give: sbus_rx and
    sbus_tx share no word (they run together), each program's threads
    start where the headers say, and the bridge's thread 2 is the four
    words that jump into thread 1's code."""
    for prog, ticks in ((RX, {0: RX_TICK}), (TX, {1: TX_TICK}),
                        (BRIDGE, {0: RX_TICK, 1: SERVO_TICK})):
        assert prog.diagnostics == []
        for thread, period in ticks.items():
            report = prog.deadlines[thread]
            assert report.period == period
            assert report.pairs and not report.infeasible and not report.unbounded
            assert report.worst_slack >= 0
    assert sorted(RX.threads) == [0] and sorted(TX.threads) == [1]
    assert sorted(BRIDGE.threads) == [0, 1, 2]
    assert not set(RX.words) & set(TX.words)
    assert len(RX.words) == 163 and len(TX.words) == 111 and len(BRIDGE.words) == 247
    assert max(a for a in RX.words if a < 0x100) < 0x80     # thread 0's quarter
    assert min(TX.words) == 0x80 and max(a for a in TX.words if a < 0x100) < 0x100
    assert sorted(a for a in BRIDGE.words if 0x100 <= a < 0x180) == [0x100 + i for i in range(4)]
    sym = BRIDGE.symbols
    assert (sym["STG"], sym["CH"], sym["FRAMEW"]) == (0x180, B_CH, FRAMEW)
    assert (RX.symbols["STG"], RX.symbols["CH"]) == (STG, CH)
    assert (TX.symbols["CHT"], TX.symbols["PERW"]) == (CHT, PERW)
