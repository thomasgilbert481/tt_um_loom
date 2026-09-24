"""The USB low-speed reference model (tools/protomodels/usb.py) on its own.

The expected values are the specification's and the USB-IF's, not the
model's: the token CRC5 examples of the USB-IF paper "Cyclic Redundancy
Checks in USB" (fields and CRC in transmission order), its CRC16 examples,
the residuals of USB 2.0 8.3.5, the catalogue check values of CRC-5/USB and
CRC-16/USB over "123456789", and packets every bus analyser shows
(``2D 00 10`` is SETUP to address 0 endpoint 0; ``C3 80 06 00 01 00 00 40
00 DD 94`` is GET_DESCRIPTOR(device) with its CRC16; ``4B 00 00`` is a
zero-length DATA1).
"""

import pytest

from tools.protomodels import usb as U
from tools.protomodels.bench import Bench, Model

BIT = U.CLOCKS_PER_BIT


def bits(text):
    return [int(c) for c in text]


def reverse(value, n):
    return int(format(value, "0%db" % n)[::-1], 2)


# ------------------------------------------------------------------ CRC5
@pytest.mark.parametrize("addr,ep,crc", [
    (0x15, 0xE, "10111"),       # setup addr 15 endp e
    (0x3A, 0xA, "11100"),       # out addr 3a endp a
    (0x70, 0x4, "01110"),       # in addr 70 endp 4
])
def test_crc5_matches_the_usb_if_token_examples(addr, ep, crc):
    value = addr | (ep << 7)
    assert format(U.crc5(value), "05b") == crc          # transmission order
    field = U.token_bytes(U.IN, addr, ep)
    assert U.check_crc5(field[1] | (field[2] << 8))


@pytest.mark.parametrize("frame,crc", [(0x001, "10111"), (0x710, "10100")])
def test_crc5_matches_the_usb_if_sof_examples(frame, crc):
    assert format(U.crc5(frame), "05b") == crc


def test_setup_token_to_address_0_is_2d_00_10():
    assert U.token_bytes(U.SETUP, 0, 0) == bytes([0x2D, 0x00, 0x10])
    assert U.token_bytes(U.IN, 0, 0) == bytes([0x69, 0x00, 0x10])
    assert U.token_bytes(U.OUT, 0, 0) == bytes([0xE1, 0x00, 0x10])


def test_crc5_check_value_and_residual():
    data = b"123456789"
    assert reverse(U.crc5(int.from_bytes(data, "little"), 72), 5) == 0x19
    # a good field leaves 01100b; one flipped bit does not
    field = 0x15 | (0xE << 7)
    good = field | (U.crc5_field(U.crc5(field)) << 11)
    assert U.check_crc5(good)
    for i in range(16):
        assert not U.check_crc5(good ^ (1 << i))


# ----------------------------------------------------------------- CRC16
@pytest.mark.parametrize("data,crc", [
    ([0x00, 0x01, 0x02, 0x03], "1111011101011110"),
    ([0x23, 0x45, 0x67, 0x89], "0111000000111000"),
])
def test_crc16_matches_the_usb_if_data_examples(data, crc):
    assert format(U.crc16(data), "016b") == crc


def test_crc16_of_known_packets():
    request = [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00]
    assert U.data_bytes(U.DATA0, request).hex() == "c38006000100004000dd94"
    assert U.data_bytes(U.DATA1, b"") == bytes([0x4B, 0x00, 0x00])
    assert reverse(U.crc16(b"123456789"), 16) == 0xB4C8


def test_crc16_residual():
    payload = bytes(range(8))
    packet = U.data_bytes(U.DATA1, payload)[1:]
    assert U._crc(U.bytes_bits(packet), 16, 0x8005, 0xFFFF) == U.CRC16_RESIDUAL
    assert U.check_crc16(packet)
    broken = bytearray(packet)
    broken[3] ^= 0x10
    assert not U.check_crc16(broken)


# ------------------------------------------------------------------ PIDs
@pytest.mark.parametrize("pid,byte", [
    (U.OUT, 0xE1), (U.IN, 0x69), (U.SOF, 0xA5), (U.SETUP, 0x2D),
    (U.DATA0, 0xC3), (U.DATA1, 0x4B), (U.ACK, 0xD2), (U.NAK, 0x5A), (U.STALL, 0x1E),
])
def test_pid_byte_and_check_nibble(pid, byte):
    assert U.pid_byte(pid) == byte
    assert U.pid_ok(byte)
    for i in range(8):
        assert not U.pid_ok(byte ^ (1 << i))


def test_decode_flags_a_bad_pid_or_crc():
    good = U.decode_bytes(U.token_bytes(U.SETUP, 5, 1))
    assert (good.pid, good.addr, good.ep, good.ok) == (U.SETUP, 5, 1, True)
    bad = bytearray(U.token_bytes(U.SETUP, 5, 1))
    bad[2] ^= 0x80
    assert not U.decode_bytes(bad).crc_ok
    bad = bytearray(U.token_bytes(U.SETUP, 5, 1))
    bad[0] ^= 0x10
    assert not U.decode_bytes(bad).pid_ok


# -------------------------------------------------------- bit level
def test_stuffing_inserts_after_six_ones_and_round_trips():
    assert U.stuff(bits("111111")) == bits("1111110")
    assert U.stuff(bits("0111111111111")) == bits("01111110111111" + "0")
    assert U.stuff(bits("11111011111")) == bits("11111011111")
    for n in range(200):
        seq = [(n >> i) & 1 for i in range(8)] * 5 + [1] * (n % 13)
        assert U.unstuff(U.stuff(seq)) == seq


def test_unstuffing_rejects_seven_ones_and_a_missing_final_stuff_bit():
    with pytest.raises(U.UsbError):
        U.unstuff(bits("1111111"))
    assert U.unstuff(bits("111111")) == bits("111111")
    with pytest.raises(U.UsbError):
        U.unstuff(bits("111111"), final=True)


def test_nrzi_toggles_on_zero_and_round_trips():
    assert U.nrzi(U.SYNC_BITS) == list("KJKJKJKK")
    assert U.nrzi(bits("0110"), U.J) == [U.K, U.K, U.K, U.J]
    for n in range(256):
        seq = [(n >> i) & 1 for i in range(8)]
        assert U.nrzi_decode(U.nrzi(seq)) == seq


def test_packet_symbols_sync_pid_eop():
    ack = U.packet_symbols(U.handshake_bytes(U.ACK))
    assert "".join(ack) == "KJKJKJKK" + "JJKJJKKK" + "00J"
    # a data packet whose bits need stuffing survives the round trip
    for payload in (b"\xff\xff\x01", b"\x7f\xfe", b"\xff" * 8, b""):
        sym = U.packet_symbols(U.data_bytes(U.DATA1, payload), eop=False)
        p = U.decode_symbols(sym)
        assert (p.pid, p.data, p.ok) == (U.DATA1, payload, True)


def test_stuffing_counts_the_sync_field():
    # the SYNC's last bit is a 1 and counts (7.1.9): a PID whose low bits
    # continue a run of 1s is stuffed earlier than it would be on its own
    body = U.bytes_bits([0xFF])
    alone = U.stuff(body)
    joined = U.stuff(list(U.SYNC_BITS) + body)[8:]
    assert alone != joined and joined[5] == 0


# ------------------------------------------------------------- the bus
class ScriptedDevice(Model):
    """Answers each packet the host ends with a canned packet ``delay`` bit
    times after its SE0-to-J, driving both lines like a device port."""

    def __init__(self, replies, delay=3.0, bit=BIT):
        self.dp, self.dm = ("uio", 0), ("uio", 1)
        self.replies = list(replies)
        self.delay, self.bit = delay, bit
        self.prev = None
        self.queue = []                   # (cycle, symbol)
        self.out = None

    def drive(self, drive, cycle):
        while self.queue and self.queue[0][0] <= cycle:
            self.out = self.queue.pop(0)[1]
        if self.out is not None:
            dp, dm = U._LEVELS[self.out]
            drive.set(self.dp, dp)
            drive.set(self.dm, dm)

    def observe(self, lines):
        state = U._STATE[(lines.get(self.dp), lines.get(self.dm))]
        if (self.prev == U.SE0 and state == U.J and self.out is None
                and not self.queue and self.replies):
            reply = self.replies.pop(0)       # the host's packet just ended
            if reply is not None:
                start = lines.cycle + int(self.delay * self.bit + 0.5)
                sym = U.packet_symbols(reply)
                for i, s in enumerate(sym):
                    self.queue.append((start + int(i * self.bit + 0.5), s))
                self.queue.append((start + int(len(sym) * self.bit + 0.5), None))
        self.prev = state


def bench_with(device):
    bench = Bench(pullups=0b10)             # D- pulled up: the idle bus is J
    host = bench.add(U.UsbHost())
    bench.add(device)
    return bench, host


def test_host_in_transaction_receives_data_and_acks():
    data = bytes([0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x08])
    device = ScriptedDevice([U.data_bytes(U.DATA1, data), None], delay=3.0)
    bench, host = bench_with(device)
    t = host.in_(0, 0)
    U.run(bench, host, 20000)
    assert t.reply_pid == U.DATA1 and t.data == data and t.acked
    assert 2.9 < t.response_bits < 3.1
    assert t.reply.eop_bits == 2 and not t.reply.glitches
    # the host's ACK is on the line after the device's EOP
    sent = [U.decode_bytes(p).pid for _, _, p in t.sent]
    assert sent == [U.IN, U.ACK]
    assert bench.contention_free() and host.collisions == []


def test_host_setup_gets_a_handshake_and_times_out_on_silence():
    device = ScriptedDevice([None, U.handshake_bytes(U.ACK)])
    bench, host = bench_with(device)
    t = host.setup(0, 0, [0x80, 6, 0, 1, 0, 0, 0x12, 0])
    U.run(bench, host, 20000)
    assert t.reply_pid == U.ACK and t.reply.packet.ok
    # the SETUP data went out 'gap' bit times after the token
    (s0, e0, tok), (s1, e1, dat) = t.sent
    assert tok == U.token_bytes(U.SETUP, 0, 0)
    assert dat == U.data_bytes(U.DATA0, [0x80, 6, 0, 1, 0, 0, 0x12, 0])
    assert abs((s1 - e0) / BIT - host.gap) < 0.1
    quiet = ScriptedDevice([None, None])
    bench, host = bench_with(quiet)
    t = host.out(3, 0, b"", pid=U.DATA1)
    U.run(bench, host, 20000)
    assert t.timeout and t.reply is None


def test_host_does_not_ack_a_bad_crc():
    packet = bytearray(U.data_bytes(U.DATA0, b"\x01\x02"))
    packet[-1] ^= 0x01
    device = ScriptedDevice([bytes(packet)])
    bench, host = bench_with(device)
    t = host.in_(1, 1)
    U.run(bench, host, 20000)
    assert t.reply.packet is not None and not t.reply.packet.crc_ok
    assert not t.acked


def test_host_line_levels_and_bit_edges():
    """J is D- high, K is D+ high; the host's own packet on the line has
    one edge per NRZI transition at the nominal bit time."""
    bench, host = bench_with(ScriptedDevice([]))
    t = host.raw([U.token_bytes(U.IN, 0, 0)], expect_reply=False)
    U.run(bench, host, 20000)
    start, end, _ = t.sent[0]
    changes = [(c, s) for c, s in host.trace if start <= c <= end]
    assert changes[0] == (start, U.K)
    sym = U.packet_symbols(U.token_bytes(U.IN, 0, 0))
    expect = [(start + int(i * BIT + 0.5), s) for i, s in enumerate(sym)
              if i == 0 or s != sym[i - 1]]
    assert changes == expect
    assert host.trace[-1][1] == U.J and t.timeout
