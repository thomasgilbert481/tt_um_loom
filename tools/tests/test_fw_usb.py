"""L3-USB-LS on the model and the RTL: firmware/usb_ls_device.loom.

The program is assembled with ``--strict``, loaded and run through
``tools.loomhost.Loom``; ``tools.protomodels.usb.UsbHost`` is the low-speed
host on the pads (D+ = BIDIR0, D- = BIDIR1, the device's 1.5 kOhm pull-up
on D- as a bench pull-up). Every body takes a ``backend``
(``tools/tests/fw_backend.py``), so pytest runs it on the golden model and
``test/test_fw.py`` runs the same body on the RTL.

Every expected timing comes from the USB 2.0 specification (the constants
in ``tools/protomodels/usb.py``), never from what the model or the RTL does.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import usb as U
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware" / "usb_ls_device.loom"
FEATURES = ("FIFO", "BE", "BEENC", "SETPD", "DMEM")
BIT = U.CLOCKS_PER_BIT
#: Clocks between RUN and the first packet: the program's set-up (the token
#: fields for address 0) takes about 500. A real device is not attached (no
#: pull-up on D-) until it is ready, which this stands for.
SETTLE = 2000


def usb_bench(backend, **kwargs):
    """A bench with the device's pull-up on D- (so an idle bus is J).

    The golden model is built like the RTL: slices A and B (``BEENC``,
    ``DMEM``) and the 512-word memory the program is laid out for.
    ``RtlBench`` takes neither argument: the RTL has both, always.
    """
    kwargs.setdefault("pullups", 0b10)
    if backend.name == "model":
        kwargs.setdefault("features", FEATURES)
        kwargs.setdefault("imem_words", 512)
    return backend.bench(**kwargs)


def setup(backend, **host):
    bench = usb_bench(backend)
    usb = bench.add(U.UsbHost("BIDIR0", "BIDIR1", **host))
    program = assemble_file(FIRMWARE, isa=ISA, strict=True)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program)
    loom.run(0)
    bench.step(SETTLE)                          # past the program's set-up
    return bench, usb, loom


# ------------------------------------------------------------------ U3
# Tokens are decoded if the device answers exactly those for it: an IN to
# its address (endpoint 0 or 1) with nothing to send gets a NAK; every other
# address or endpoint, a bad CRC5 and a bad check nibble get silence.
ANSWERED = [(U.IN, 0, 0), (U.IN, 0, 1)]
IGNORED = [(U.IN, 5, 0), (U.IN, 0x15, 0xE), (U.OUT, 0x7F, 0xF), (U.SETUP, 0x3A, 0xA),
           (U.IN, 0x70, 0x4), (U.IN, 0, 2)]


def test_usb_tokens_are_decoded(backend):
    """SYNC, PID, address, endpoint and CRC5 of IN, OUT and SETUP tokens
    (OUT 0x7F/0xF is eleven 1s: two stuff bits in the token)."""
    bench, usb, loom = setup(backend)
    naks = [usb.in_(a, e) for _, a, e in ANSWERED]
    quiet = [usb.raw([U.token_bytes(*t)], expect_reply=False) for t in IGNORED]
    good = U.token_bytes(U.IN, 0, 0)
    quiet.append(usb.raw([bytes([good[0], good[1], good[2] ^ 0x40])],
                         expect_reply=False))                # bad CRC5
    quiet.append(usb.raw([bytes([good[0] ^ 0x10, good[1], good[2]])],
                         expect_reply=False))                # bad check nibble
    U.run(bench, usb, 60000)
    for t in naks:
        check_reply(t, "NAK")
    for t in quiet:
        assert t.timeout and t.reply is None, t.describe()
    check_bus(usb, bench)
    assert loom.badop() == 0


def check_reply(t, name):
    """The device's reply in transaction ``t`` against USB 2.0 timing:
    the response delay (7.1.18.1, TRSPIPD1), the EOP (TLEOPT, then J), the
    bit rate (+-1.5 %, 7.1.11) and the source jitter at the nominal bit time
    (TUDJ1 +-95 ns to the next transition, TUDJ2 +-150 ns paired)."""
    rec = t.reply
    assert rec is not None and rec.packet is not None, t.describe()
    assert rec.packet.ok and rec.packet.name == name, t.describe()
    assert U.RESPONSE_MIN <= t.response_bits <= U.RESPONSE_MAX, t.response_bits
    se0_us = (rec.eop_end - rec.eop_start) / 50.0
    assert U.EOP_SE0_MIN_US <= se0_us <= U.EOP_SE0_MAX_US, se0_us
    assert rec.eop_bits == 2 and not rec.glitches
    edges = rec.bit_edges
    n = len(edges)
    mc = sum(c for c, _ in edges) / n
    mi = sum(i for _, i in edges) / n
    slope = (sum((c - mc) * (i - mi) for c, i in edges)
             / sum((i - mi) ** 2 for _, i in edges))
    assert abs(slope / BIT - 1) <= U.LS_RATE_TOL, slope
    next_ns = U.DEVICE_JITTER_NEXT_NS / 20.0
    paired_ns = U.DEVICE_JITTER_PAIRED_NS / 20.0
    for (c0, i0), (c1, i1) in zip(edges, edges[1:]):
        assert abs((c1 - c0) - (i1 - i0) * BIT) <= next_ns, (c0, c1, i0, i1)
    for (c0, i0), (c2, i2) in zip(edges, edges[2:]):
        assert abs((c2 - c0) - (i2 - i0) * BIT) <= paired_ns, (c0, c2, i0, i2)
    return rec


def check_bus(usb, bench):
    """Nobody drove against anybody, and the device let go after each of its
    packets (its output enables are off at the end)."""
    bench.step(int(3 * BIT))                    # past the J that ends an EOP
    assert bench.contention_free() and usb.collisions == []
    assert not usb.oe_trace or usb.oe_trace[-1][1] == 0


GET_DEVICE = [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00]
#: The device descriptor (USB 2.0 table 9-8) the program should return: USB
#: 1.1, bMaxPacketSize0 8, VID 0x1209 / PID 0x0001, one configuration.
DEVICE = bytes([18, 1, 0x10, 0x01, 0, 0, 0, 8, 0x09, 0x12, 0x01, 0x00,
                0x00, 0x01, 0, 0, 0, 1])


def test_usb_setup_is_acked_in_time(backend):
    """SETUP + DATA0: the device ACKs within 2 to 6.5 bit times, with a
    proper EOP and bit timing; a SETUP to another address is ignored."""
    bench, usb, loom = setup(backend)
    t = usb.setup(0, 0, GET_DEVICE)
    other = usb.setup(5, 0, GET_DEVICE)
    U.run(bench, usb, 12000)
    check_reply(t, "ACK")
    assert other.timeout and other.reply is None
    check_bus(usb, bench)
    assert loom.badop() == 0


def control_read(bench, usb, addr, request):
    """A control read (8.5.3): SETUP, IN until a short packet or wLength,
    then the status stage (OUT, a ZLP DATA1). Every reply is checked for
    PID, data toggle and timing. Returns the data and the transactions."""
    length = request[6] | request[7] << 8
    stages = [usb.setup(addr, 0, request)]
    U.run(bench, usb, 12000)
    check_reply(stages[0], "ACK")
    data, pid = b"", U.DATA1                   # the data stage starts DATA1
    while len(data) < length:
        t = usb.in_(addr, 0)
        U.run(bench, usb, 12000)
        check_reply(t, U.PID_NAMES[pid])
        assert t.acked
        stages.append(t)
        data += t.data
        pid ^= U.DATA0 ^ U.DATA1
        if len(t.data) < 8:
            break
    t = usb.out(addr, 0, b"", pid=U.DATA1)
    U.run(bench, usb, 12000)
    check_reply(t, "ACK")
    stages.append(t)
    return data, stages


# ------------------------------------------------------------------ U4
def test_usb_get_device_descriptor(backend):
    """GET_DESCRIPTOR(device) end to end: 18 bytes as 8 + 8 + 2 with DATA1,
    DATA0, DATA1, then the status stage."""
    bench, usb, loom = setup(backend)
    data, stages = control_read(bench, usb, 0, GET_DEVICE)
    assert data == DEVICE
    assert [len(t.data) for t in stages[1:-1]] == [8, 8, 2]
    check_bus(usb, bench)
    assert loom.badop() == 0


def test_usb_first_8_bytes_and_a_retry(backend):
    """wLength 64 as a host asks first; an unacknowledged packet is sent
    again, same data and same PID; after the ACK the toggle moves on."""
    bench, usb, loom = setup(backend)
    s = usb.setup(0, 0, [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00])
    lost = usb.in_(0, 0, ack=False)
    again = usb.in_(0, 0)
    nxt = usb.in_(0, 0)
    U.run(bench, usb, 40000)
    check_reply(s, "ACK")
    check_reply(lost, "DATA1")
    check_reply(again, "DATA1")
    check_reply(nxt, "DATA0")
    assert lost.data == again.data == DEVICE[:8] and nxt.data == DEVICE[8:16]
    check_bus(usb, bench)


@model_only("receive margin sweep: the RTL keeps the nominal rate")
@pytest.mark.parametrize("error", [-U.HOST_RATE_TOL, U.HOST_RATE_TOL])
def test_usb_at_the_host_rate_limits(backend, error):
    """The host sends at 1.5 Mbit/s +-0.25 % (7.1.11): the device still
    reads every packet; it answers at its own rate, checked as always."""
    bench, usb, loom = setup(backend, bit=BIT * (1 + error))
    data, stages = control_read(bench, usb, 0, GET_DEVICE)
    assert data == DEVICE


# ------------------------------------------------------------------ U5
#: The configuration descriptor set (USB 2.0 9.6.3, 9.6.5, 9.6.6; HID 1.11
#: 6.2.1): configuration 1, bus powered 100 mA; interface 0, HID, boot
#: subclass, mouse; HID 1.11 with a 50-byte report descriptor; endpoint 0x81
#: interrupt IN, 3 bytes, 10 ms.
CONFIG = bytes([9, 2, 34, 0, 1, 1, 0, 0x80, 50,
                9, 4, 0, 0, 1, 3, 1, 2, 0,
                9, 0x21, 0x11, 0x01, 0, 1, 0x22, 50, 0,
                7, 5, 0x81, 3, 3, 0, 10])
#: The boot mouse report descriptor of HID 1.11 appendix B.2.
REPORT = bytes([0x05, 0x01, 0x09, 0x02, 0xA1, 0x01, 0x09, 0x01, 0xA1, 0x00,
                0x05, 0x09, 0x19, 0x01, 0x29, 0x03, 0x15, 0x00, 0x25, 0x01,
                0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05,
                0x81, 0x01, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81,
                0x25, 0x7F, 0x75, 0x08, 0x95, 0x02, 0x81, 0x06, 0xC0, 0xC0])


def request(bm, b, value=0, index=0, length=0):
    return [bm, b, value & 0xFF, value >> 8, index & 0xFF, index >> 8,
            length & 0xFF, length >> 8]


def control_nodata(bench, usb, addr, req):
    """A control transfer without data (8.5.3): SETUP, then the status
    stage, an IN answered by a ZLP DATA1 that the host ACKs."""
    s = usb.setup(addr, 0, req)
    st = usb.in_(addr, 0)
    U.run(bench, usb, 20000)
    check_reply(s, "ACK")
    check_reply(st, "DATA1")
    assert st.data == b"" and st.acked
    return s, st


def test_usb_set_address_takes_effect_after_its_status_stage(backend):
    """SET_ADDRESS (9.4.6): the status stage still at address 0, then the
    device answers only its new address; a token to the old one is ignored."""
    bench, usb, loom = setup(backend)
    control_nodata(bench, usb, 0, request(0x00, 5, value=0x2A))
    old = usb.setup(0, 0, GET_DEVICE)
    U.run(bench, usb, 20000)
    assert old.timeout and old.reply is None
    data, _ = control_read(bench, usb, 0x2A, GET_DEVICE)
    assert data == DEVICE
    nak = usb.in_(0x2A, 1)                     # endpoint 1 moved with it
    gone = usb.in_(0, 1)
    U.run(bench, usb, 20000)
    check_reply(nak, "NAK")
    assert gone.timeout
    check_bus(usb, bench)
    assert loom.badop() == 0


def test_usb_configuration_and_report_descriptors(backend):
    """GET_DESCRIPTOR(configuration) as a host asks: 9 bytes first (an odd
    count: 8 + 1), then wTotalLength; the HID report descriptor (an
    interface request, HID 1.11 7.1.1); SET_CONFIGURATION."""
    bench, usb, loom = setup(backend)
    data, _ = control_read(bench, usb, 0, request(0x80, 6, 0x0200, 0, 9))
    assert data == CONFIG[:9]
    data, _ = control_read(bench, usb, 0, request(0x80, 6, 0x0200, 0, 0xFF))
    assert data == CONFIG
    data, _ = control_read(bench, usb, 0, request(0x81, 6, 0x2200, 0, 50))
    assert data == REPORT
    control_nodata(bench, usb, 0, request(0x00, 9, value=1))
    check_bus(usb, bench)


def test_usb_unknown_requests_stall(backend):
    """A device-to-host request the device does not know gets a STALL in its
    data stage (a string descriptor: the descriptors name no string); the
    next SETUP clears it (8.5.3.4)."""
    bench, usb, loom = setup(backend)
    s = usb.setup(0, 0, request(0x80, 6, 0x0300, 0, 0xFF))
    st = usb.in_(0, 0)
    U.run(bench, usb, 20000)
    check_reply(s, "ACK")
    check_reply(st, "STALL")
    data, _ = control_read(bench, usb, 0, GET_DEVICE)
    assert data == DEVICE


# ------------------------------------------------------------------ U6
def report_words(buttons, x, y):
    """One boot mouse report as the host pushes it: two INQ words."""
    return [(buttons & 0xFF) | (x & 0xFF) << 8, y & 0xFF]


def test_usb_interrupt_in_returns_the_reports_pushed(backend):
    """Endpoint 1 (interrupt IN): NAK while INQ has no whole report; a
    report pushed through the host port comes back as DATA0, the next as
    DATA1 (8.6.4); SET_CONFIGURATION restarts the toggle at DATA0."""
    bench, usb, loom = setup(backend)
    control_nodata(bench, usb, 0, request(0x00, 9, value=1))
    none = usb.in_(0, 1)
    U.run(bench, usb, 12000)
    check_reply(none, "NAK")
    loom.push(0, report_words(1, 5, -3))
    bench.step(int(300 * BIT))                 # the device looks at INQ when idle
    first = usb.in_(0, 1)
    empty = usb.in_(0, 1)
    U.run(bench, usb, 20000)
    check_reply(first, "DATA0")
    assert first.data == bytes([1, 5, 0xFD]) and first.acked
    check_reply(empty, "NAK")
    loom.push(0, report_words(0, 0x80, 0x7F))
    bench.step(int(300 * BIT))
    second = usb.in_(0, 1)
    U.run(bench, usb, 12000)
    check_reply(second, "DATA1")
    assert second.data == bytes([0, 0x80, 0x7F])
    check_bus(usb, bench)
    assert loom.badop() == 0


def test_usb_interrupt_report_is_repeated_until_acked(backend):
    """A report the host does not acknowledge is sent again, same data and
    same PID; a half report (one word in INQ) is not sent at all."""
    bench, usb, loom = setup(backend)
    loom.push(0, report_words(2, 0x10, 0x20)[:1])
    bench.step(int(300 * BIT))
    half = usb.in_(0, 1)
    U.run(bench, usb, 12000)
    check_reply(half, "NAK")
    loom.push(0, report_words(2, 0x10, 0x20)[1:])
    bench.step(int(300 * BIT))
    lost = usb.in_(0, 1, ack=False)
    again = usb.in_(0, 1)
    U.run(bench, usb, 30000)
    check_reply(lost, "DATA0")
    check_reply(again, "DATA0")
    assert lost.data == again.data == bytes([2, 0x10, 0x20]) and again.acked
    check_bus(usb, bench)


def test_usb_at_the_minimum_inter_packet_gap(backend):
    """The host allowed its fastest (7.1.18.1): 2 bit times between its
    own packets, between transactions, and before its ACK. The device keeps
    up with a whole control read and a no-data request."""
    bench, usb, loom = setup(backend, gap=U.IPD_MIN, ack_gap=U.RESPONSE_MIN)
    data, _ = control_read(bench, usb, 0, GET_DEVICE)
    assert data == DEVICE
    control_nodata(bench, usb, 0, request(0x00, 9, value=1))
    check_bus(usb, bench)
