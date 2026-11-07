"""L3-J1850 on the model and the RTL: firmware/j1850_vpw.loom, SAE J1850 VPW.

One copy of the program serves every node: thread 0 transmits on OUT0,
thread 2 (whose reset vector jumps to the same code) on OUT1, and both
receive the bus on IN0. ``tools.protomodels.j1850.VpwBus`` is the wire: a
wired OR of the chip's TX pins and the model nodes, fed back to IN0 one
clock later, as two transceivers on one bus would do.

Expected values come from the J1850 frame layout, coding and timing
through the reference model (and the sources its docstring names) and from
the program header (host words and report format), never from what the
model or the RTL did. Pin timing is measured on the bus: every symbol the
chip transmits against AN1731's transmit windows (``j1850.check_tx``), and
the receive side is driven at the edges of the receive windows.

Rate: the program counts ticks of 2 us, TICK_INT 100 at 50 MHz
(``test_j1850_real_rate_tx`` runs that on the model only). The other bodies
run a tick of 16 clocks with every bus time scaled with it
(``j1850.scale_for``); the program counts in ticks, so only the speed
changes.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import j1850 as J
from tools.tests.fw_backend import model_only

ISA = load()
SOURCE = REPO / "firmware" / "j1850_vpw.loom"
IMEM = 512
FAST = 16                           # clocks a tick in the tests
REAL = 100                          # 2 us at 50 MHz

# report (j1850_vpw.loom header)
F_CRC, F_SYM, F_OVF, F_IFR, F_SENT, F_LOST, F_RX = 1, 2, 4, 8, 16, 32, 0x8000

OBD_REQ = [0x68, 0x6A, 0xF1, 0x01, 0x00]        # a mode 1 PID 0 request
CHIP_ADDR = 0x10                                # the chip node's address

_IMAGE = None


def image():
    """The program, assembled with --strict, every word."""
    global _IMAGE
    if _IMAGE is None:
        prog = assemble_file(SOURCE, isa=ISA, strict=True)
        assert prog.diagnostics == []
        assert max(prog.words) < 0x180, "code runs into the node data"
        _IMAGE = prog.words
    return _IMAGE


def setup(backend, tick=FAST, nodes=(), threads=(0,), cfg=None):
    """The chip on a bus with model ``nodes``, ``threads`` running the
    program, each with its configuration word (``cfg[t]``, default 0: no
    IFR). Returns ``(bench, loom, bus)``."""
    us = J.scale_for(tick)
    bus = J.VpwBus(us, nodes, tx=("OUT0", "OUT1"), rx=("IN0",))
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "BE", "BEENC", "DMEM"),
                              imem_words=IMEM)
    else:
        bench = backend.bench()
    bench.add(bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(image())
    for t in threads:
        loom.write_csr(t, "TICK_INT", tick)
        loom.write_csr(t, "TICK_FRAC", 0)
        loom.push(t, [(cfg or {}).get(t, 0)])
    loom.run(list(threads))
    bench.step(int(40 * us))
    return bench, loom, bus


def send(loom, thread, data):
    """Queue a frame (header and data, no CRC) on a chip node."""
    loom.push(thread, [len(data)] + [b & 0xFF for b in data])


def polls(tick, us_time):
    return int(us_time * J.scale_for(tick)) // 40 + 2000


def report(loom, thread, tick=FAST, within_us=12000):
    """``(status, bytes)`` of a node's next report."""
    status = loom.pop(thread, 1, max_polls=polls(tick, within_us))[0]
    n = loom.pop(thread, 1, max_polls=polls(tick, 2000))[0]
    data = loom.pop(thread, n, max_polls=polls(tick, 4000)) if n else []
    assert all(0 <= b <= 0xFF for b in data)
    return status, list(data)


def frames(bus):
    return [f for f in bus.frames() if f.symbols]


def assert_tx_timing(frame, us):
    assert J.check_tx(frame, us) == [], J.check_tx(frame, us)


# ------------------------------------------------------------ transmit
def test_j1850_tx_to_model(backend):
    """The chip sends the OBD-II request 68 6A F1 01 00: the bus carries
    exactly those bytes and the CRC 0x17 the model computes, every symbol
    inside AN1731's transmit windows, no IFR; the report is our own frame,
    no arbitration lost, no IFR byte."""
    us = J.scale_for(FAST)
    bench, loom, bus = setup(backend)
    send(loom, 0, OBD_REQ)
    status, data = report(loom, 0)
    assert (status, data) == (0, [])
    fs = frames(bus)
    assert len(fs) == 1
    f = fs[0]
    assert f.data == J.with_crc(OBD_REQ) and f.data[-1] == 0x17
    assert f.crc_ok() and f.nb is None and f.ifr == []
    assert_tx_timing(f, us)
    assert loom.badop() == 0


def test_j1850_tx_gets_ifr(backend):
    """The chip sends a frame to a model node at 0x40 (K = 0), which answers
    with its address as a one-byte IFR after a short NB: the report holds
    that byte and F_IFR; the chip's own symbols are in the windows."""
    us = J.scale_for(FAST)
    ecu = J.VpwNode(us, address=0x40)
    bench, loom, bus = setup(backend, nodes=[ecu])
    send(loom, 0, [0x60, 0x40, 0xF1, 0x3C])
    status, data = report(loom, 0)
    assert (status, data) == (F_IFR, [0x40])
    f = frames(bus)[0]
    assert f.data == J.with_crc([0x60, 0x40, 0xF1, 0x3C]) and f.ifr == [0x40]
    assert len(ecu.answered) == 1
    assert J.check_tx(f, us) == []


# ------------------------------------------------------------ receive
def test_j1850_model_to_rx(backend):
    """A model node sends 48 6B 10 41 00 BE 1F B8 11 and its CRC: the chip
    reports a received frame of 10 bytes with no error, the bytes as
    sent."""
    us = J.scale_for(FAST)
    node = J.VpwNode(us)
    frame = J.with_crc([0x48, 0x6B, 0x10, 0x41, 0x00, 0xBE, 0x1F, 0xB8, 0x11])
    bench, loom, bus = setup(backend, nodes=[node])
    node.send(J.frame_symbols(frame))
    status, data = report(loom, 0, within_us=20000)
    assert status == F_RX | (len(frame) << 8), hex(status)
    assert data == frame
    assert frames(bus)[0].data == frame


def test_j1850_crc_error(backend):
    """The same kind of frame with its CRC byte off by one bit: the chip
    reports it with F_CRC and the bytes as they came."""
    us = J.scale_for(FAST)
    node = J.VpwNode(us)
    frame = J.with_crc([0x68, 0x6A, 0xF1, 0x01, 0x00])
    frame[-1] ^= 0x01
    bench, loom, bus = setup(backend, nodes=[node])
    node.send(J.frame_symbols(frame))
    status, data = report(loom, 0)
    assert status == F_RX | F_CRC | (len(frame) << 8), hex(status)
    assert data == frame


EDGES = {   # microseconds, each 3 us or more inside AN1731's receive windows
    "low": {J.SHORT: 37.0, J.LONG: 99.0, J.SOF: 166.0},
    "high": {J.SHORT: 93.0, J.LONG: 160.0, J.SOF: 236.0},
}


@pytest.mark.parametrize("edge", ["low", "high"])
def test_j1850_rx_window_edges(backend, edge):
    """A model frame whose every symbol sits at one end of its receive
    window (SOF too; the bits alternate short and long): the chip receives
    it intact. Its measurement is in ticks of 2 us, so the margins are 3 us
    (a tick and a half)."""
    us = J.scale_for(FAST)
    node = J.VpwNode(us)
    frame = J.with_crc([0x68, 0x6A, 0xF1, 0x01, 0x00])
    lengths = EDGES[edge]
    bench, loom, bus = setup(backend, nodes=[node])
    node.send([(lvl, lengths[cls]) for lvl, cls in J.frame_symbols(frame)])
    status, data = report(loom, 0)
    assert status == F_RX | (len(frame) << 8), hex(status)
    assert data == frame


@model_only("a second error case; the decoding paths are those of the edge test")
def test_j1850_invalid_symbol(backend):
    """A frame whose fourth symbol is 28 us, under the 34 us a valid symbol
    needs: the chip reports F_SYM and waits for EOF."""
    us = J.scale_for(FAST)
    node = J.VpwNode(us)
    syms = J.frame_symbols(J.with_crc([0x68, 0x6A, 0xF1, 0x01, 0x00]))
    syms[4] = (syms[4][0], 28.0)
    bench, loom, bus = setup(backend, nodes=[node])
    node.send(syms)
    status, data = report(loom, 0)
    assert status & (F_RX | F_SYM) == F_RX | F_SYM, hex(status)


# ------------------------------------------------------------ IFR
def test_j1850_ifr_answer(backend):
    """The chip at address 0x10 with IFR enabled; a model tester sends
    64 10 F1 20 (K = 0, to 0x10): the chip answers with a short NB 200 us
    after the last fall and the IFR byte 0x10, both inside the transmit
    windows; it reports the frame with F_SENT, the frame's 5 bytes."""
    us = J.scale_for(FAST)
    tester = J.VpwNode(us)
    frame = J.with_crc([0x64, 0x10, 0xF1, 0x20])
    bench, loom, bus = setup(backend, nodes=[tester],
                             cfg={0: CHIP_ADDR << 8})
    tester.send(J.frame_symbols(frame))
    status, data = report(loom, 0)
    assert status == F_RX | F_SENT | (len(frame) << 8), hex(status)
    assert data == frame
    f = frames(bus)[0]
    assert f.data == frame and f.ifr == [CHIP_ADDR]
    assert J.classify(f.nb.us(us)) == J.NB_NO_CRC
    assert J.check_tx(f, us) == []


@model_only("the address test of the IFR path; the RTL runs the answer itself")
def test_j1850_no_ifr_for_others(backend):
    """The same chip and a frame to 0x11, and one to 0x10 with K = 1: no
    IFR on the bus, both reported without F_SENT."""
    us = J.scale_for(FAST)
    tester = J.VpwNode(us)
    bench, loom, bus = setup(backend, nodes=[tester],
                             cfg={0: CHIP_ADDR << 8})
    for data in ([0x64, 0x11, 0xF1, 0x20], [0x64 | 0x08, 0x10, 0xF1, 0x20]):
        tester.send(J.frame_symbols(J.with_crc(data)))
        status, got = report(loom, 0)
        assert status == F_RX | (5 << 8) and got == J.with_crc(data), hex(status)
    assert all(f.nb is None for f in frames(bus))


# ------------------------------------------------------------ arbitration
def test_j1850_two_nodes_arbitrate(backend):
    """Thread 0 (OUT0) and thread 2 (OUT1) start frames together; thread 2's
    header 0x48 is lower than thread 0's 0x68, and the first difference is
    the third bit (0 against 1). Thread 0 loses there and stops driving;
    the bus carries thread 2's frame intact, then thread 0's retry after
    the bus has been idle again, both inside the windows. Reports: thread 2
    no loss, thread 0 one loss (12:8 = 1)."""
    us = J.scale_for(FAST)
    bench, loom, bus = setup(backend, threads=(0, 2))
    loom.halt([0, 2])                   # queue both, then start together
    a, b = [0x68, 0x6A, 0xF1], [0x48, 0x6B, 0x10]   # 4 words: INQ holds them
    send(loom, 0, a)
    send(loom, 2, b)
    loom.run([0, 2])
    assert report(loom, 2) == (0, [])
    assert report(loom, 0, within_us=20000) == (1 << 8, [])
    fs = frames(bus)
    assert [f.data for f in fs] == [J.with_crc(b), J.with_crc(a)]
    for f in fs:
        assert_tx_timing(f, us)
    assert fs[1].start - fs[0].symbols[-1].start >= 280 * us     # IFS
    lost = [e for e in bus.edges_of("OUT0") if e[0] < fs[0].symbols[-1].start]
    assert [v for _, v in lost] == [1, 0, 1, 0], lost   # SOF and bit 1 only


# ------------------------------------------------------------ real rate
@model_only("TICK_INT 100: a frame is about 300 k clocks")
def test_j1850_real_rate_tx(backend):
    """At the real rate, 2 us ticks at 50 MHz: the OBD request's symbols
    are in AN1731's transmit windows, within 0.2 us of nominal."""
    us = J.scale_for(REAL)
    bench, loom, bus = setup(backend, tick=REAL)
    send(loom, 0, OBD_REQ)
    assert report(loom, 0, tick=REAL) == (0, [])
    f = frames(bus)[0]
    assert f.data == J.with_crc(OBD_REQ)
    assert_tx_timing(f, us)
    for s in f.symbols[1:-1]:
        nom = J.nominal(J.classify(s.us(us)))
        assert abs(s.us(us) - nom) <= 0.2, (s, nom)
