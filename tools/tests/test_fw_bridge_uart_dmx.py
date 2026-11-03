"""L3-BRIDGE on the model and the RTL: firmware/bridge_uart_dmx.loom.

Two threads of one program, no host in the loop: thread 0 receives 8N1 on
IN0 (``tools.protomodels.uart.UartTx`` sends it) and parses the DMX USB Pro
framing (label 6: ``0x7E, 6, LEN lo, LEN hi, start code, slots, 0xE7``)
into a universe buffer in data memory; thread 1 sends DMX512 on OUT0 from
the newest complete buffer, and ``tools.protomodels.dmx.DmxRx`` decodes it
and times every field against E1.11's transmit column. The host loads the
image, sets the two ticks, starts both threads and afterwards only pops the
status words (the count of frames published, program header).

Expected values come from E1.11 through the reference model (the timing
table, the slot format), from the framing and the program header (what a
frame does, the buffer capacity, the status word), never from what the
model or the RTL did.

Rates at 50 MHz: DMX 250 kbit/s, a tick of 200 clocks a bit; UART at 16
clocks a tick, 128 clocks a bit (390.6 kbaud, the fastest rate the
program's header proves), and at 115200 baud (54 + 64/256) on the model.
The RTL runs two short scenarios that between them cover a frame reaching
the next packet, an update that lands mid-packet, back-to-back bytes at
the full rate and the DMX timing compared clock for clock with and without
UART traffic; the full universe, the parser's rejections, 115200 baud with
a rate error and the tearing stress run on the model only.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.dmx import TX, DmxRx
from tools.protomodels.uart import UartTx
from tools.tests.fw_backend import model_only

ISA = load()
SOURCE = REPO / "firmware" / "bridge_uart_dmx.loom"
IMEM = 512
PROGRAM = assemble_file(SOURCE, isa=ISA, imem_words=IMEM, strict=True)

BIT = 200                       # DMX: clocks a bit at 50 MHz, 250 kbit/s
DMX_TICK = 200                  # thread 1: one tick a bit (header)
UART_TICK = 16                  # thread 0: eight ticks a bit (header)
UART_BIT = 8 * UART_TICK        # 128 clocks a UART bit in the RTL scenarios
BREAK, MAB, BTB = 44 * BIT, 4 * BIT, 302 * BIT      # the header's figures
SLOT = 11 * BIT                 # 8N2

# The framing and the buffer, from the program header.
SYNC, LABEL, END = 0x7E, 6, 0xE7
N = 209                         # slots a buffer holds
BUF0 = PROGRAM.symbols["BUF0"]  # the first buffer, loaded with the image


def frame(start_code, slots, *, label=LABEL, length=None, end=END):
    """One DMX USB Pro message: the start code and the slots as its data."""
    data = [start_code] + list(slots)
    n = len(data) if length is None else length
    return [SYNC, label, n & 0xFF, n >> 8] + data + [end]


def buffer_words(start_code, slots):
    """A buffer as the header lays it out: the count, then the bytes packed
    two a word, low byte first."""
    data = [start_code] + list(slots)
    data += [0] * (len(data) & 1)
    return [len(slots)] + [data[i] | data[i + 1] << 8 for i in range(0, len(data), 2)]


def bridge_bench(backend):
    """The FIFOs, the bit engine and LD/ST with the 512-word memory on the
    model; the RTL has all of them and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC", "DMEM"), imem_words=IMEM)
    return backend.bench()


class Edges(Model):
    """Every change of OUT0, as (cycle, level), as test_fw_four_at_once.py
    records them."""

    def __init__(self, pin="OUT0"):
        self.pad, self.prev, self.log = pad_of(pin), None, []

    def observe(self, lines):
        level = lines.get(self.pad)
        if self.prev is not None and level != self.prev:
            self.log.append((lines.cycle, level))
        self.prev = level


def boot(backend, *, uart_bit=UART_BIT, tick=(UART_TICK, 0), image=None, threads=(0, 1)):
    """The bridge on a bench with a UART sender on IN0, the DMX decoder and
    an edge log on OUT0; loaded, both ticks set, ``threads`` running."""
    bench = bridge_bench(backend)
    uart = bench.add(UartTx("IN0", uart_bit))
    rx = bench.add(DmxRx("OUT0", limits=TX))
    edges = bench.add(Edges())
    words = dict(PROGRAM.words)
    words.update(image or {})
    loom = backend.loom(bench, isa=ISA)
    loom.load(words)
    loom.write_csr(0, "TICK_INT", tick[0])
    loom.write_csr(0, "TICK_FRAC", tick[1])
    loom.write_csr(1, "TICK_INT", DMX_TICK)
    loom.write_csr(1, "TICK_FRAC", 0)
    loom.run(list(threads), exclusive=True)
    bench.step(64)                              # both threads past their setup
    return bench, loom, uart, rx, edges


def preload(start_code, slots):
    """Image words that put a frame in the first buffer, as if published."""
    return {BUF0 + i: w for i, w in enumerate(buffer_words(start_code, slots))}


def sent_end(uart):
    """The cycle the last queued UART frame's stop bit ends."""
    start = uart.frames_sent[-1][0]
    return int(round(start + 10 * uart.bit_clocks))


def assert_timing(rx, packet):
    """E1.11's transmit column (DmxRx.violations, every edge of every slot
    included), and the header's figures, which are exact: every edge lands
    the same few clocks after its tick, and the slots go back to back."""
    assert rx.violations(packet, TX) == [], rx.violations(packet, TX)
    assert packet.break_clocks == BREAK, packet.break_clocks
    assert packet.mab_clocks == MAB, packet.mab_clocks
    for slot in packet.slots:
        assert all((cycle - slot.start) % BIT == 0 for cycle, _ in slot.edges)
        assert slot.mark_before == 0, slot.mark_before
    if packet.btb_clocks is not None:
        assert packet.btb_clocks >= BTB


def until_packet_after(bench, rx, cycle, slots, limit):
    """Step until the packet whose break follows ``cycle`` has been sent
    whole (``slots`` data slots), at most ``limit`` clocks after ``cycle``;
    returns it."""
    limit += max(cycle - bench.cycle, 0)
    def done():
        later = [p for p in rx.packets() if p.break_start > cycle]
        return later and len(later[0].slots) == slots + 1 and \
            bench.cycle > later[0].break_start + BREAK + MAB + (slots + 1) * SLOT
    assert bench.run_until(done, limit, every=2000), "no packet after cycle %d" % cycle
    return [p for p in rx.packets() if p.break_start > cycle][0]


A = [(17 * i + 5) & 0xFF for i in range(16)]
B = [0xFF - v for v in A]


def test_bridge_frames_reach_the_next_packet_whole(backend):
    """Frame A, bytes back to back, sets 16 slots and the first packet
    whose break follows A's end byte carries all of them. Frame B, the same
    slots inverted, is sent from that packet's break so that its end byte
    lands in the middle of the packet's slots: that packet is still A whole,
    the next is B whole, and no packet ever mixes the two."""
    bench, loom, uart, rx, _ = boot(backend)
    uart.send(frame(0, A))
    end_a = sent_end(uart)
    assert bench.run_until(lambda: any(p.break_start > end_a for p in rx.packets()),
                           3 * BTB, every=500)
    in_flight = [p for p in rx.packets() if p.break_start > end_a][0]
    uart.send(frame(0, B))                      # from just after that break
    end_b = sent_end(uart)
    slots_from = in_flight.break_start + BREAK + MAB
    assert slots_from + SLOT < end_b < slots_from + 16 * SLOT, "B does not end mid-packet"
    after = until_packet_after(bench, rx, end_b, len(B), 3 * BTB)

    packets = rx.packets()
    assert (packets[0].start_code, packets[0].data) == (0, [])    # the image's buffer
    for packet in packets:
        assert (packet.start_code, packet.data) in [(0, []), (0, A), (0, B)], packet.data
        assert_timing(rx, packet)
    in_flight = [p for p in packets if p.break_start == in_flight.break_start][0]
    assert in_flight.data == A, "the packet in flight when B ended"
    assert after.data == B, "the first packet after B ended"
    assert [p.data for p in packets if p.break_start < end_a] in ([[]], [[], []])
    assert loom.pop(0, 2) == [1, 2]             # two frames published
    assert loom.badop() == 0


F = [0x10, 0x20, 0x30, 0x40]
RUN_CLOCKS = 2 * BTB + 10000            # three breaks, the last packet whole


def test_bridge_dmx_timing_is_the_same_with_and_without_uart_traffic(backend):
    """Both runs load the same image, with frame F already in the first
    buffer, and make the same host transactions; in the second the UART
    sends F three times, every byte back to back at the full rate, so
    thread 0 parses, stores and publishes while thread 1 sends, and thread
    1 changes buffers. Every OUT0 edge falls on the same clock in both runs
    (SEMANTICS 2 and 6.11: thread 1 never waits on thread 0), every packet
    carries F, timed to E1.11, and all three frames were published."""
    quiet = boot(backend, image=preload(0, F))
    quiet[0].step(RUN_CLOCKS)
    bench, loom, uart, rx, edges = boot(backend, image=preload(0, F))
    for _ in range(3):
        uart.send(frame(0, F))
    bench.step(RUN_CLOCKS)
    assert not uart.busy

    packets = rx.packets()
    if len(packets[-1].slots) < len(F) + 1:
        packets.pop()                           # cut short by the end of the run
    assert len(packets) >= 2, len(packets)
    for packet in packets:
        assert (packet.start_code, packet.data) == (0, F)
        assert_timing(rx, packet)
    assert edges.log == quiet[4].log, "%d edges quiet, %d with traffic" % (
        len(quiet[4].log), len(edges.log))
    assert loom.pop(0, 3) == [1, 2, 3]          # back to back: none lost
    assert quiet[1].pop_available(0) == []
    assert loom.badop() == 0 and quiet[1].badop() == 0


@model_only("a 518-byte frame and a 209-slot packet are 1.2 M clocks")
def test_bridge_takes_a_full_dmx_usb_pro_universe_and_keeps_the_first_209(backend):
    """The 513 data bytes PC software sends for a whole universe (LEN 0x201,
    so the length's high byte counts): the next packet carries the start
    code and slots 1 .. 209, the buffer's capacity, every one timed."""
    bench, loom, uart, rx, _ = boot(backend)
    slots = [(7 * i + 3) & 0xFF for i in range(512)]
    uart.send(frame(0, slots))
    packet = until_packet_after(bench, rx, sent_end(uart), N, 1500000)
    assert packet.data == slots[:N] and packet.start_code == 0
    assert_timing(rx, packet)
    assert loom.pop(0, 1) == [1]


@model_only("one long stream of short messages; the RTL scenarios run the same parser")
def test_bridge_publishes_only_well_formed_label_6_frames(backend):
    """Bytes before a sync; a message of another label whose data look like
    a frame (skipped by its length, so the frame inside is never seen); a
    frame whose end byte is not 0xE7; LEN 0; a frame with a framing error
    in a data byte: none of them reaches the wire. Then a good frame does,
    and it is the only one counted."""
    bench, loom, uart, rx, _ = boot(backend)
    uart.send([0x00, END, LABEL])
    uart.send(frame(0x55, [], label=0x0A, length=6)[:4]
              + [SYNC, LABEL, 0x01, 0x00, 0x55, END] + [END])
    uart.send(frame(0, [1, 2], end=0xE6))
    uart.send([SYNC, LABEL, 0x00, 0x00, END])
    bad = frame(0, [0x11, 0x12, 0x13])
    uart.send(bad[:6])
    uart.send_frame(bad[6], stop=0)             # the second slot's stop bit low
    uart.send(bad[7:])
    uart.send(frame(0, [0x42, 0x43]))
    packet = until_packet_after(bench, rx, sent_end(uart), 2, 4 * BTB)
    assert packet.data == [0x42, 0x43]
    for p in rx.packets():
        assert (p.start_code, p.data) in [(0, []), (0, [0x42, 0x43])], (p.start_code, p.data)
        assert_timing(rx, p)
    assert loom.pop(0, 1) == [1] and loom.pop_available(0) == []


BAUD_115200 = 50e6 / 115200             # 434.03 clocks a bit
TICK_115200 = (54, 64)                  # one eighth of a bit (header)


@model_only("115200 baud is 434 clocks a bit; the RTL runs the fast rate")
@pytest.mark.parametrize("error", [-0.03, 0.0, 0.03])
def test_bridge_at_115200_baud_with_a_rate_error(backend, error):
    """Three frames back to back at 115200 baud from a sender 3 % fast or
    slow (the program header's tolerance): all three published, the last
    on the wire."""
    bench, loom, uart, rx, _ = boot(backend, uart_bit=BAUD_115200 * (1 + error),
                                    tick=TICK_115200)
    frames = [[k, k + 1, k + 2, k + 3] for k in (0x21, 0x41, 0x61)]
    for slots in frames:
        uart.send(frame(0, slots))
    packet = until_packet_after(bench, rx, sent_end(uart), 4, 3 * BTB)
    assert packet.data == frames[-1]
    for p in rx.packets():
        assert p.data in [[]] + frames
        assert_timing(rx, p)
    assert loom.pop(0, 3) == [1, 2, 3]


@model_only("a back-to-back stream of eight frames, 400 k clocks")
@pytest.mark.parametrize("error", [-0.02, 0.03])
def test_bridge_fast_rate_back_to_back_with_a_rate_error(backend, error):
    """At the fastest rate (128 clocks a bit) a sender 2 % fast or 3 % slow
    (the header's window at this rate), bytes back to back: every frame
    published."""
    bench, loom, uart, rx, _ = boot(backend, uart_bit=UART_BIT * (1 + error))
    for k in range(1, 4):
        uart.send(frame(0, [k] * 8))
    packet = until_packet_after(bench, rx, sent_end(uart), 8, 3 * BTB)
    assert packet.data == [3] * 8
    assert loom.pop(0, 3) == [1, 2, 3]


@model_only("a back-to-back stream of eight frames, 400 k clocks")
def test_bridge_never_tears_a_packet_and_drops_status_words_when_outq_is_full(backend):
    """Eight frames back to back, each setting 24 slots to one value, while
    packets go out at their own pace: every packet holds one value (a
    packet mixing two frames would not), the values never go back, the last
    frame is on the wire. The host pops nothing meanwhile, so OUTQ keeps
    the first four counts and the rest are dropped; the next frame's word
    carries the count of all nine."""
    bench, loom, uart, rx, _ = boot(backend)
    for k in range(1, 9):
        uart.send(frame(0, [k] * 24))
    packet = until_packet_after(bench, rx, sent_end(uart), 24, 3 * BTB)
    assert packet.data == [8] * 24
    seen = [p.data[0] for p in rx.packets() if p.data]
    for p in rx.packets():
        assert p.data in [[]] + [[k] * 24 for k in range(1, 9)], p.data
        assert_timing(rx, p)
    assert seen == sorted(seen) and len(set(seen)) >= 4, seen
    assert loom.pop_available(0) == [1, 2, 3, 4]
    uart.send(frame(0, [9] * 24))
    assert loom.pop(0, 1, max_polls=2000) == [9]


def test_bridge_assembles_strict_with_every_pair_proved():
    """--strict and no diagnostic; every deadline pair of both threads
    proved for the ticks the header gives; the layout the header gives
    (code in the two quarters, the buffers above thread 1's code)."""
    assert PROGRAM.diagnostics == [] and sorted(PROGRAM.threads) == [0, 1]
    for thread, period in ((0, UART_TICK), (1, DMX_TICK)):
        report = PROGRAM.deadlines[thread]
        assert report.period == period
        assert report.pairs and not report.infeasible and not report.unbounded
        assert report.worst_slack >= 0
    sym = PROGRAM.symbols
    assert sym["CAP"] == N + 1 and sym["CAP"] % 2 == 0
    assert (sym["BUF0"], sym["BUF1"], sym["BUF2"]) == (194, 300, 406)
    code0 = [a for a in PROGRAM.words if a < 128]
    code1 = [a for a in PROGRAM.words if 128 <= a < BUF0]
    assert sorted(code0)[-3:] == [125, 126, 127] == [sym["SHARED"] + i for i in range(3)]
    assert sorted(code0)[-4] < sym["SHARED"]
    assert max(code1) < BUF0
