"""tools.protomodels.uart: the encoder and decoder against each other.

Both run on the bench with no firmware: the encoder drives IN0 (a chip input
pad) and the decoder watches the same resolved line.
"""

import pytest

from tools.protomodels.bench import Bench
from tools.protomodels.uart import UartRx, UartTx


def loop(tx_bits, rx_bits, data=b"", frames=()):
    bench = Bench()
    tx = bench.add(UartTx("IN0", tx_bits))
    rx = bench.add(UartRx("IN0", rx_bits))
    tx.send(data)
    for value, stop in frames:
        tx.send_frame(value, stop=stop)
    bench.run_until(lambda: not tx.busy, 20000, every=16)
    bench.step(int(rx_bits * 2))
    return tx, rx


@pytest.mark.parametrize("bits", [16, 32, 40.5, 434.03])
def test_encoder_into_decoder(bits):
    data = b"Loom\x00\xff\x55" if bits < 100 else b"L\xa5"
    tx, rx = loop(bits, bits, data)
    assert rx.data == data
    assert not rx.errors


def test_frames_are_back_to_back_and_on_the_bit_grid():
    tx, rx = loop(32, 32, b"\x55\x00\xff")
    starts = [f.start for f in rx.frames]
    assert [b - a for a, b in zip(starts, starts[1:])] == [320, 320]
    for frame in rx.frames:
        assert all(offset == 0 for _, offset in rx.edge_offsets(frame))
    assert [k for k, _ in rx.edge_offsets(rx.frames[0])] == list(range(10))  # 0x55: every bit


def test_framing_error_is_detected_and_the_next_frame_survives():
    tx, rx = loop(32, 32, frames=[(0x41, 1), (0x42, 0), (0x43, 1)])
    assert [(f.value, f.framing_error) for f in rx.frames] == [
        (0x41, False), (0x42, True), (0x43, False)]
    assert rx.data == b"AC"


@pytest.mark.parametrize("skew", [0.96, 1.04])
def test_decoder_tolerates_a_few_per_cent_of_baud_error(skew):
    tx, rx = loop(64 * skew, 64, b"\x00\xff\x5a\xa5")
    assert rx.data == b"\x00\xff\x5a\xa5" and not rx.errors


@pytest.mark.parametrize("skew", [0.85, 1.15])
def test_a_large_baud_error_is_caught(skew):
    tx, rx = loop(64 * skew, 64, b"\x00\x00")
    assert rx.data != b"\x00\x00"


def test_a_short_glitch_is_not_a_start_bit():
    bench = Bench()
    rx = bench.add(UartRx("IN0", 64))

    class Glitch(UartTx):
        def drive(self, drive, cycle):
            drive.set(self.pad, 0 if 100 <= cycle < 120 else 1)

    bench.add(Glitch("IN0", 64))
    bench.step(1000)
    assert rx.frames == []


def test_the_decoder_waits_for_idle_before_arming():
    """A line that comes out of reset low is not a start bit."""
    bench = Bench()
    rx = bench.add(UartRx("OUT0", 32))                    # the chip leaves OUT0 low
    bench.step(1000)
    assert rx.frames == []
