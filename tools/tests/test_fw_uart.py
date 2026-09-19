"""L3-UART-TX and L3-UART-RX through the host port, on the model and the RTL.

``firmware/uart_tx_fifo.loom`` and ``firmware/uart_rx.loom`` are assembled,
loaded, configured and fed entirely through ``tools.loomhost.Loom``, with the
UART reference models of ``tools.protomodels.uart`` on the pads.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``) and builds its
bench and its transport through it, so the same bodies run

* here under pytest on the golden model: ``tools.protomodels.bench.Bench``
  steps a ``tools.loomsim.Machine`` and ``ModelTransport`` moves the host
  bytes (SPI bytes, 64 clocks each, firmware running meanwhile);
* and under cocotb on the RTL (``test/test_fw.py``): ``RtlBench`` clocks
  ``tb.v`` and ``SimTransport`` clocks the same bytes through the real SPI
  pads.

Modelled rates, at a 50 MHz clock: TX at 32 clocks per bit (1.5625 Mbaud)
and at 115200 baud (TICK 434 + 8/256); RX at 64 clocks per bit (781 kbaud,
TICK 8) and at 115200 baud (TICK 54 + 64/256). A 115200-baud bit is 434
clocks, so the RX cases at that rate are marked ``model_only``: a simulated
clock costs a fraction of a millisecond in Icarus, so they would add minutes
to the RTL run, and the fast rates exercise the same code. The TX case stays
on both because it is the one that measures fractional-tick edge placement.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.uart import UartRx, UartTx
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
SLOT = 4


def boot(backend, bench, name, tick_int, tick_frac=0):
    """Load a firmware program through the host port, set its tick, run it."""
    program = assemble_file(FIRMWARE / (name + ".loom"), isa=ISA, strict=True)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program)
    loom.write_csr(0, "TICK_INT", tick_int)
    loom.write_csr(0, "TICK_FRAC", tick_frac)
    loom.run(0)
    return loom, program


# ----------------------------------------------------------------------- TX
def test_uart_tx_sends_a_host_pushed_string(backend):
    bench = backend.bench()
    rx = bench.add(UartRx("OUT0", 32))
    loom, _ = boot(backend, bench, "uart_tx_fifo", 32)
    message = b"Hello, Loom!\r\n"
    loom.push(0, message)                                  # 4 at a time, flow controlled
    assert bench.run_until(lambda: len(rx.frames) == len(message), 80000, every=64)
    assert rx.data == message and not rx.errors
    # 32 clocks is a multiple of the slot: every edge exactly on the bit grid
    for frame in rx.frames:
        assert [off for _, off in rx.edge_offsets(frame)] == [0] * len(frame.edges)
    # the first four words were queued together: back to back, one stop bit
    starts = [f.start for f in rx.frames]
    assert [b - a for a, b in zip(starts[:4], starts[1:4])] == [320, 320, 320]
    # after an idle gap the start bit waits for the tick grid: no drift ever
    assert all((s - starts[0]) % 32 == 0 for s in starts)
    assert all(b - a >= 320 for a, b in zip(starts, starts[1:]))
    assert loom.badop() == 0


def test_uart_tx_at_115200_baud_edges_within_one_slot(backend):
    bits = 434 + 8 / 256                                   # 50 MHz / 115200 = 434.03
    bench = backend.bench()
    rx = bench.add(UartRx("OUT0", bits))
    loom, _ = boot(backend, bench, "uart_tx_fifo", 434, 8)
    loom.push(0, b"U\x00")
    assert bench.run_until(lambda: len(rx.frames) == 2, 40000, every=256)
    assert rx.data == b"U\x00"
    offsets = [off for f in rx.frames for _, off in rx.edge_offsets(f)]
    assert len(offsets) == 12                              # 0x55: ten edges, 0x00: two
    assert max(abs(o) for o in offsets) < SLOT             # SEMANTICS 2: one slot
    assert rx.frames[1].start - rx.frames[0].start == pytest.approx(10 * bits, abs=SLOT)


def test_uart_tx_ignores_the_high_byte_of_a_word(backend):
    bench = backend.bench()
    rx = bench.add(UartRx("OUT0", 32))
    loom, _ = boot(backend, bench, "uart_tx_fifo", 32)
    loom.push(0, [0xFF41, 0x0142])
    assert bench.run_until(lambda: len(rx.frames) == 2, 20000, every=64)
    assert rx.data == b"AB"


# ----------------------------------------------------------------------- RX
def rx_bench(backend, tick_int, tick_frac, bit_clocks, gap_bits=0.0):
    bench = backend.bench()
    tx = bench.add(UartTx("IN0", bit_clocks, gap_bits=gap_bits))
    loom, program = boot(backend, bench, "uart_rx", tick_int, tick_frac)
    bench.step(200)                                        # let it reach the start-bit wait
    return bench, tx, loom, program


def send_and_wait(bench, tx, data=b"", frames=()):
    tx.send(data)
    for value, stop in frames:
        tx.send_frame(value, stop=stop)
    assert bench.run_until(lambda: not tx.busy, 100000, every=64)
    bench.step(200)


def test_uart_rx_delivers_bytes_and_a_framing_error_in_bit_8(backend):
    bench, tx, loom, _ = rx_bench(backend, 8, 0, 64)
    send_and_wait(bench, tx, b"Hi", [(0x5A, 0), (0x21, 1)])
    assert loom.pop(0, 4) == [0x48, 0x69, 0x100 | 0x5A, 0x21]
    assert loom.pop_available(0) == [] and loom.badop() == 0


def test_uart_rx_samples_every_bit_near_its_middle(backend):
    bench, tx, loom, program = rx_bench(backend, 8, 0, 64)
    rx_pin = ISA.pin_by_name["IN0"]
    samples = []

    def watch(record):
        if record is not None and record.thread == 0 and record.mnemonic == "JP":
            if ISA.decode(record.ir)[1]["pin"] == rx_pin:
                samples.append(record.x_cycle - 2)         # the pad cycle it saw
    bench.add_observer(watch)
    send_and_wait(bench, tx, b"\x55\xaa\x0f")
    assert loom.pop(0, 3) == [0x55, 0xAA, 0x0F]
    starts = [start for start, _, _ in tx.frames_sent]
    assert len(samples) == 30                              # start, 8 data, stop per frame
    for i, s in enumerate(samples):
        frame, k = divmod(i, 10)
        error = s - (starts[frame] + (k + 0.5) * 64)
        assert abs(error) <= 8 + 3 * SLOT                  # a tick plus latency


@pytest.mark.parametrize("skew", [0.97, 1.03])
def test_uart_rx_tolerates_three_per_cent_at_the_fastest_rate(backend, skew):
    """64 clocks per bit, one idle bit between frames (see the program header)."""
    bench, tx, loom, _ = rx_bench(backend, 8, 0, 64 * skew, gap_bits=1)
    for chunk in (b"\x00\xff\x5a\xa5", b"\x81\x7e\x33\xcc"):
        send_and_wait(bench, tx, chunk)
        assert loom.pop(0, 4) == list(chunk)


@model_only("three bytes at 434 clocks per bit is 13 k cycles of line time alone")
def test_uart_rx_at_115200_baud(backend):
    bench, tx, loom, _ = rx_bench(backend, 54, 64, 434 + 8 / 256)   # 54.25 clocks per tick
    send_and_wait(bench, tx, b"OK!")
    assert loom.pop(0, 3) == [ord(c) for c in "OK!"]


@model_only("four bytes at 434 clocks per bit, twice over")
@pytest.mark.parametrize("skew", [0.97, 1.03])
def test_uart_rx_back_to_back_at_115200_with_three_per_cent_error(backend, skew):
    bench, tx, loom, _ = rx_bench(backend, 54, 64, (434 + 8 / 256) * skew)
    send_and_wait(bench, tx, b"\x00\xff\x5a\xa5")
    assert loom.pop(0, 4) == [0x00, 0xFF, 0x5A, 0xA5]


def test_uart_rx_flags_an_overrun_in_bit_9(backend):
    bench, tx, loom, _ = rx_bench(backend, 8, 0, 64)
    send_and_wait(bench, tx, b"abcdef")                    # OUTQ holds four
    assert loom.pop(0, 4) == list(b"abcd")
    send_and_wait(bench, tx, b"g")
    assert loom.pop(0, 1) == [0x200 | ord("g")]            # e and f were lost
    send_and_wait(bench, tx, b"h")
    assert loom.pop(0, 1) == [ord("h")]                    # the flag is cleared


def test_uart_rx_ignores_a_glitch_and_a_break_is_one_error(backend):
    bench, tx, loom, _ = rx_bench(backend, 8, 0, 64)

    class Line(UartTx):
        """Drives a scripted level: a short glitch, then a break, then idle."""

        def drive(self, drive, cycle):
            t = cycle - self.t0
            level = 0 if (100 <= t < 110) or (400 <= t < 2400) else 1
            drive.set(self.pad, level)

    bench.remove(tx)
    line = Line("IN0", 64)
    line.t0 = bench.cycle
    bench.add(line)
    bench.step(3000)
    words = loom.pop_available(0)
    assert words == [0x100]                                # one framing error, byte 0
