"""Self-tests of the reference 10BASE-T receiver, ``tools/protomodels/ethernet.py``.

The waveforms here are synthesised in this file from the line coding the
receiver's docstring cites (IEEE 802.3 clauses 3, 7.3.1.1 and 14, partly
through secondary sources), with ``zlib.crc32`` for the FCS, so the receiver
is checked against something that is neither Loom's model nor its RTL.
"""

import math
import zlib

from tools.protomodels import ethernet as rx

PREAMBLE_SFD = [1, 0] * 31 + [1, 1]


def wire_bits(payload, lose=0):
    data = rx.bytes_to_bits(payload)
    crc = zlib.crc32(payload)
    return PREAMBLE_SFD[lose:] + data + [(crc >> j) & 1 for j in range(32)]


def synth(bits, half, idle_before=7, tail=12.0, idle_after=9):
    """Per-cycle (TD+, TD-) for ``bits``, ``half`` clocks a half bit (may be
    fractional: the level at cycle c is the one at time c), then a high of
    ``tail`` clocks and no drive."""
    halves = []
    for v in bits:
        halves += [1 - v, v]                    # 0: high then low; 1: low then high
    total = idle_before + math.ceil(len(halves) * half + tail) + idle_after
    out = []
    for c in range(total):
        t = c - idle_before
        if t < 0:
            out.append((0, 0))
            continue
        i = int(t // half)
        if i < len(halves):
            level = halves[i]
            out.append((level, 1 - level))
        elif t < len(halves) * half + tail:
            out.append((1, 0))
        else:
            out.append((0, 0))
    return [p for p, _ in out], [n for _, n in out]


PAYLOAD = bytes(range(60))


def test_clean_frame():
    tdp, tdn = synth(wire_bits(PAYLOAD), 2)
    frame = rx.receive_frame(tdp, tdn, 2, clock_hz=40e6)
    assert frame.ok, frame.errors
    assert frame.payload == PAYLOAD and frame.preamble_bits == 64
    assert frame.fcs == zlib.crc32(PAYLOAD) and frame.residue == 0xC704DD7B


def test_flipped_bit_fails_the_fcs():
    bits = wire_bits(PAYLOAD)
    bits[64 + 100] ^= 1
    frame = rx.receive_frame(*synth(bits, 2), 2)
    assert frame.sfd_found and not frame.fcs_ok and not frame.ok


def test_lost_preamble_bits_are_reported_not_fatal():
    frame = rx.receive_frame(*synth(wire_bits(PAYLOAD, lose=10), 2), 2)
    assert frame.fcs_ok and frame.preamble_bits == 54


def test_no_sfd():
    frame = rx.receive_frame(*synth([1, 0] * 40, 2), 2)
    assert not frame.sfd_found and not frame.ok


def test_clock_recovery_follows_a_slow_transmitter():
    """A transmitter 1 per cent slow (10.1 clocks a half bit, receiver
    nominal 10) still decodes: every middle transition re-locks the receiver."""
    tdp, tdn = synth(wire_bits(PAYLOAD), 10.1, tail=30.3)
    frame = rx.receive_frame(tdp, tdn, 10)
    assert frame.ok and frame.payload == PAYLOAD


def test_tp_idl_window():
    """250 to 450 ns at 40 MHz is 10 to 18 clocks of high [S, unchecked]."""
    bits = wire_bits(PAYLOAD)
    tail_after_last = lambda clocks: clocks - (2 if bits[-1] else 0)
    for clocks, ok in ((12, True), (18, True), (8, False), (20, False)):
        tdp, tdn = synth(bits, 2, tail=tail_after_last(clocks))
        frame = rx.receive_frame(tdp, tdn, 2, clock_hz=40e6)
        assert frame.decoded.tp_idl_clocks == clocks
        assert frame.ok == ok, (clocks, frame.errors)


def test_receive_all_two_frames():
    a, b = synth(wire_bits(b"A" * 46), 2), synth(wire_bits(b"B" * 46), 2)
    frames = rx.receive_all(a[0] + b[0], a[1] + b[1], 2, clock_hz=40e6)
    assert [f.payload for f in frames] == [b"A" * 46, b"B" * 46]
    assert all(f.ok for f in frames)
