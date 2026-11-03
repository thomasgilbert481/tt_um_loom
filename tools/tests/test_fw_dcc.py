"""L3-DCC: the NMRA DCC command station and decoder, on the model and the RTL.

``firmware/dcc_station.loom`` (thread 0, the track signal on OUT0) and
``firmware/dcc_decoder.loom`` (thread 1, the track signal on IN0) are
assembled, loaded and driven through ``tools.loomhost.Loom``.
``tools.protomodels.dcc.DccRx`` times every half-bit the station sends and
holds it to the command-station figures of S-9.1 Table 2.1 before it
decodes packets as S-9.2 has them; ``DccPlayer`` plays the encoder's
half-bits (``halves_of``, with settable lengths, stretched 0s and injected
faults) into the decoder; ``Wire`` joins OUT0 to IN0 for the two programs
together. Every body that takes a ``backend`` runs here on the golden model
and under cocotb on the RTL (``test/test_fw.py``).

The expected numbers come from S-9.1, S-9.2 and ``docs/SEMANTICS.md``,
never from what the model or the RTL happened to do: the station's tick is
1 us, a 1 half 58 ticks and a 0 half 100 (Table 2.1 nominals), and
SEMANTICS 6.10 lands each deadline-latched edge on the exact clock edge of
its tick, so on an integer-MHz board every half is exactly 58 or 100 us of
clocks. The decoder must accept every half inside tr1 and tr0 and every 1
whose halves differ by tr1d or less (Table 2.1, Table 2).

The RTL runs at 8 MHz, the slowest clock the decoder's header allows: a
packet is about 5.8 ms, 46 k clocks there. 50 MHz runs on the model.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import dcc as D
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
T1, T0 = 58, 100                              # us, S-9.1 Table 2.1 nominals


class Board:
    """A board clock; the ticks the program headers give for it."""

    def __init__(self, mhz):
        self.mhz = mhz
        self.station_tick = (mhz, 0)                         # 1 us
        self.decoder_tick = (mhz // 2, 128 if mhz % 2 else 0)  # 0.5 us

    def clocks(self, us):
        return int(round(us * self.mhz))

    def __repr__(self):
        return "%dMHz" % self.mhz


B8, B50 = Board(8), Board(50)

_PROGRAMS = {}


def program(name):
    if name not in _PROGRAMS:
        _PROGRAMS[name] = assemble_file(FIRMWARE / (name + ".loom"), isa=ISA,
                                        strict=True)
    return _PROGRAMS[name]


def make_bench(backend):
    """FIFOs, the deadline latch, data memory and the 512-word memory; the
    RTL has them all."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM)
    return backend.bench()


def boot(backend, bench, board, station=False, decoder=False, address=3):
    """Load the programs asked for, set their ticks, start them."""
    loom = backend.loom(bench, isa=ISA)
    threads = []
    if station:
        loom.load(program("dcc_station"))
        loom.write_csr(0, "TICK_INT", board.station_tick[0])
        loom.write_csr(0, "TICK_FRAC", board.station_tick[1])
        threads.append(0)
    if decoder:
        dec = program("dcc_decoder")
        words = dict(dec.words)
        words[dec.symbols["addr"]] = address
        loom.load(words)
        loom.write_csr(1, "TICK_INT", board.decoder_tick[0])
        loom.write_csr(1, "TICK_FRAC", board.decoder_tick[1])
        threads.append(1)
    loom.run(threads)
    return loom


COUNTERS = ("cok", "cxor", "cbit", "cfmt", "clost")


def counters(loom):
    """The decoder's counters, read with every thread halted (its header)."""
    loom.halt()
    sym = program("dcc_decoder").symbols
    return {name: loom.read_imem(sym[name])[0] for name in COUNTERS}


def station_words(data):
    """The station's host format (its header): a header word, then the
    bytes after the first, two a word."""
    words = [len(data) << 8 | data[0]]
    rest = list(data[1:])
    while rest:
        hi = rest.pop(0)
        lo = rest.pop(0) if rest else 0
        words.append(hi << 8 | lo)
    return words


def preamble_word(bits):
    return 0x8000 | bits


def is_idle(p):
    return p.payload == list(D.IDLE)


def packet_us(data, preamble=D.PREAMBLE_TX_MIN):
    return sum(D.halves_of(D.packet_bits(data, preamble=preamble)))


def station_setup(backend, board):
    bench = make_bench(backend)
    rx = bench.add(D.DccRx("OUT0", clock_mhz=board.mhz, transmit=True))
    return bench, rx, boot(backend, bench, board, station=True)


def run_packets(bench, rx, n, board, budget_ms=None):
    """Step until ``rx`` has ``n`` packets that are not idle ones."""
    budget = board.clocks(1000 * (budget_ms or 7 * (n + 2)))
    assert bench.run_until(lambda: len([p for p in rx.packets if not is_idle(p)]) >= n,
                           budget, every=256), rx.report()
    return [p for p in rx.packets if not is_idle(p)]


def check_station(rx, board):
    """Every half inside the transmit windows, every preamble 14 or more,
    and every half exactly 58 or 100 us of clocks (SEMANTICS 6.10)."""
    rx.check_preambles()
    assert rx.violations == [], rx.report()
    assert rx.rejected == [], rx.report()
    assert set(rx.halves) <= {board.clocks(T1), board.clocks(T0)}, sorted(set(rx.halves))


# ------------------------------------------------------- the station
SPEED = D.speed_packet(3, True, 6)            # S-9.2 figure 1's speed step, address 3
LONG = [0xC1, 0x23, 0x3F, 0x9F, 0x55]         # a 5-byte packet (RP-9.2.1 shape)


def test_dcc_station_sends_host_packets(backend):
    """A speed packet and a 5-byte packet, each with its error byte, in
    order; every half exact and inside the transmit windows. (Idle packets
    are checked in the two-thread scenario and the idle test.)"""
    board = B8
    bench, rx, loom = station_setup(backend, board)
    loom.push(0, station_words(SPEED))
    run_packets(bench, rx, 1, board)
    loom.push(0, station_words(LONG))
    sent = run_packets(bench, rx, 2, board)
    assert [p.payload for p in sent] == [SPEED, LONG]
    assert [p.data[-1] for p in sent] == [D.error_byte(d) for d in (SPEED, LONG)]
    check_station(rx, board)
    assert loom.badop() == 0


@model_only("the idle packet and the reset packet; the RTL sees idles in the two-thread scenario")
def test_dcc_station_idles_then_sends_the_reset_packet(backend):
    """With INQ empty an idle packet (FF 00 FF); then the reset packet."""
    board = B8
    bench, rx, loom = station_setup(backend, board)
    assert bench.run_until(lambda: len(rx.packets) >= 1, board.clocks(14000), every=256)
    assert is_idle(rx.packets[0]) and rx.packets[0].data == [0xFF, 0x00, 0xFF]
    loom.push(0, station_words(list(D.RESET)))
    assert run_packets(bench, rx, 1, board)[0].data == [0, 0, 0]
    check_station(rx, board)


@model_only("the preamble word: the RTL runs the host packets at the default preamble")
def test_dcc_station_preamble_is_settable_and_at_least_14(backend):
    """``0x8000 | 20`` gives 20-bit preambles from the next packet; a
    request for 5 gives 14 (S-9.2 section A)."""
    board = B8
    bench, rx, loom = station_setup(backend, board)
    loom.push(0, [preamble_word(20)])
    bench.step(board.clocks(13000))
    n = len(rx.packets)
    assert bench.run_until(lambda: len(rx.packets) >= n + 2, board.clocks(16000), every=256)
    assert all((p.preamble - 2) // 2 == 20 for p in rx.packets[n:]), \
        [p.preamble for p in rx.packets]
    loom.push(0, [preamble_word(5)])
    bench.step(board.clocks(13000))
    n = len(rx.packets)
    assert bench.run_until(lambda: len(rx.packets) >= n + 2, board.clocks(16000), every=256)
    assert all((p.preamble - 2) // 2 == 14 for p in rx.packets[n:])
    check_station(rx, board)


@model_only("header edge cases; the RTL runs the whole-packet path")
def test_dcc_station_ignores_bad_headers_and_waits_for_late_bytes(backend):
    """n = 0 and n = 6 are ignored; a header whose bytes come late
    lengthens that packet's preamble instead of breaking the stream."""
    board = B8
    bench, rx, loom = station_setup(backend, board)
    loom.push(0, [0x0003, 0x0603])
    bench.step(board.clocks(13000))
    assert all(is_idle(p) for p in rx.packets)
    words = station_words(LONG)
    loom.push(0, words[:1])
    bench.step(board.clocks(3000))
    loom.push(0, words[1:])
    sent = run_packets(bench, rx, 1, board)
    assert sent[0].payload == LONG
    assert (sent[0].preamble - 2) // 2 > D.PREAMBLE_TX_MIN
    check_station(rx, board)


@model_only("50 MHz: 70 k clocks a packet; the 8 MHz run covers the same code")
def test_dcc_station_at_50mhz(backend):
    board = B50
    bench, rx, loom = station_setup(backend, board)
    loom.push(0, station_words(SPEED))
    assert run_packets(bench, rx, 1, board)[0].payload == SPEED
    check_station(rx, board)


# ------------------------------------------------------- the decoder
def decoder_setup(backend, board, address=3):
    bench = make_bench(backend)
    player = bench.add(D.DccPlayer("IN0", clock_mhz=board.mhz))
    loom = boot(backend, bench, board, decoder=True, address=address)
    return bench, player, loom


def collect(bench, loom, board, us, thread=1, chunk_us=4000):
    """Step ``us`` of board time, emptying OUTQ every ``chunk_us`` (less
    than a packet, so the host keeps a report's worth of room)."""
    out, left = [], board.clocks(us)
    while left > 0:
        step = min(left, board.clocks(chunk_us))
        bench.step(step)
        left -= step
        out += loom.pop_available(thread)
    return out


def play(bench, player, loom, board, packets, tail=4, **kw):
    """Queue ``packets`` as ``(data, encoder keyword dict)`` after a
    14-bit preamble, then ``tail`` 1s, and run them through; returns the
    decoded reports."""
    halves = D.halves_of([1] * D.PREAMBLE_TX_MIN)
    for data, enc in packets:
        enc = dict(enc)
        pre = enc.pop("preamble", D.PREAMBLE_TX_MIN)
        bad = enc.pop("bad_error_byte", False)
        bits = D.packet_bits(data, preamble=pre, bad_error_byte=bad)
        halves += D.halves_of(bits, **enc)
    halves += D.halves_of([1] * tail)
    player.queue(halves)
    words = collect(bench, loom, board, sum(halves) + 300)
    return D.parse_reports(words)


#: (data, encoder settings): S-9.1's receive windows at their edges.
EDGES = [
    (SPEED, dict(t1=52.0)),                       # every 1 half at tr1 min
    (D.broadcast_stop(), dict(t1=58.0, t1b=64.0, t0=90.0)),   # tr1 max as the
]                                                 # second half, 6 us apart; tr0 min


def test_dcc_decoder_accepts_the_receive_windows_at_their_edges(backend):
    """tr1 = 52 and 64 us, tr0 = 90 us, a 1 whose halves differ by tr1d =
    6 us (58 then 64): every packet reported, nothing counted as dropped."""
    board = B8
    bench, player, loom = decoder_setup(backend, board)
    got = play(bench, player, loom, board, EDGES)
    assert got == [d for d, _ in EDGES]
    c = counters(loom)
    assert c == dict(cok=2, cxor=0, cbit=0, cfmt=0, clost=0), c
    assert loom.badop() == 0


@model_only("a stretched 0 adds 10 ms; the window edges run on the RTL")
def test_dcc_decoder_accepts_stretched_zeros_and_short_preambles(backend):
    """S-9.1 section 2.1: one half of a 0 stretched to 9900 us (t0s max)
    and to 10000 us (tr0 max); S-9.2 section A: a 10-bit preamble is
    accepted, an 8-bit one (9 with the end bit before it) is not a packet."""
    board = B8
    bench, player, loom = decoder_setup(backend, board)
    bits = D.packet_bits(SPEED)
    zeros = [i for i, b in enumerate(bits) if b == 0]
    pk = [(SPEED, dict(stretch={zeros[1]: 9900.0})),
          (list(D.RESET), dict(stretch={zeros[3]: 10000.0, zeros[5]: 2000.0})),
          (SPEED, dict(preamble=10)),
          (list(D.RESET), dict(preamble=8))]       # 8 + the end bit: too few
    assert play(bench, player, loom, board, pk) == [d for d, _ in pk[:3]]
    c = counters(loom)
    assert (c["cok"], c["cbit"], c["cxor"]) == (3, 0, 0), c


@model_only("the 50 MHz clock; the 8 MHz run covers the same code")
def test_dcc_decoder_at_50mhz(backend):
    board = B50
    bench, player, loom = decoder_setup(backend, board)
    assert play(bench, player, loom, board, EDGES) == [d for d, _ in EDGES]


def test_dcc_decoder_drops_bad_packets_and_counts_them(backend):
    """A wrong error byte and a 1 half of 75 us (between the windows):
    neither reported, the good packet after them is; cxor and cbit count
    one each. (The short preamble is in the stretched-zero scenario.)"""
    board = B8
    bench, player, loom = decoder_setup(backend, board)
    bits = D.packet_bits(SPEED)
    third_one = [i for i, b in enumerate(bits) if b][D.PREAMBLE_TX_MIN + 2]
    pk = [(SPEED, dict(bad_error_byte=True)),
          (SPEED, dict(override={2 * third_one: 75.0})),
          (list(D.RESET), {})]
    assert play(bench, player, loom, board, pk) == [list(D.RESET)]
    c = counters(loom)
    assert c == dict(cok=1, cxor=1, cbit=1, cfmt=0, clost=0), c


@model_only("OUTQ full; the RTL host empties OUTQ in every scenario")
def test_dcc_decoder_counts_reports_lost_to_a_full_outq(backend):
    """Four packets for the address and a host that reads nothing: two
    reports fill OUTQ (4 words), the other two are counted in clost and
    the decoder never stalls (cok counts all four)."""
    board = B8
    bench, player, loom = decoder_setup(backend, board)
    halves = D.halves_of([1] * 14)
    for _ in range(4):
        halves += D.halves_of(D.packet_bits(SPEED))
    halves += D.halves_of([1] * 4)
    player.queue(halves)
    bench.step(board.clocks(sum(halves) + 300))
    assert D.parse_reports(loom.pop_available(1)) == [SPEED, SPEED]
    c = counters(loom)
    assert (c["cok"], c["clost"], c["cbit"]) == (4, 2, 0), c


@model_only("packet length; the RTL runs the error byte and the windows")
def test_dcc_decoder_drops_packets_of_the_wrong_length(backend):
    """Two bytes (address and error byte) and seven bytes are dropped and
    counted in cfmt; a 6-byte packet, the longest, is reported."""
    board = B8
    bench, player, loom = decoder_setup(backend, board)
    six = [3, 0x3F, 0x9F, 0x12, 0x34]
    pk = [([3], {}), ([3, 1, 2, 3, 4, 5], {}), (six, {})]
    assert play(bench, player, loom, board, pk) == [six]
    assert counters(loom)["cfmt"] == 2


# ------------------------------------------------- the two together
def two_threads(backend, sends, address=5):
    """The station (thread 0) on OUT0, wired by the bench to the decoder
    (thread 1) on IN0; ``sends`` pushed up to 4 words at a time. Returns
    the packets on the line and the decoder's reports."""
    board = B8
    bench = make_bench(backend)
    bench.add(D.Wire("OUT0", "IN0"))
    rx = bench.add(D.DccRx("OUT0", clock_mhz=board.mhz, transmit=True))
    loom = boot(backend, bench, board, station=True, decoder=True, address=address)
    reports, k = [], 0
    while k < len(sends):
        batch = [sends[k]]
        while k + len(batch) < len(sends) and                 len([w for d in batch + [sends[k + len(batch)]] for w in station_words(d)]) <= 4:
            batch.append(sends[k + len(batch)])
        loom.push(0, [w for d in batch for w in station_words(d)])
        k += len(batch)
        while len([p for p in rx.packets if not is_idle(p)]) < k:
            reports += collect(bench, loom, board, 2000)
    reports += collect(bench, loom, board, 300)
    assert [p.payload for p in rx.packets if not is_idle(p)] == sends
    n = len(rx.packets)
    c = counters(loom)
    assert c == dict(cok=n, cxor=0, cbit=0, cfmt=0, clost=0), (c, n)
    check_station(rx, board)
    return rx, D.parse_reports(reports)


def test_dcc_station_and_decoder_on_two_threads(backend):
    """Station and decoder (address 5) on two threads: of a speed packet
    for 3 and a broadcast stop, the decoder reports the broadcast and
    counts every packet the station sent, idles included. (Its own address
    is matched in the window-edge scenario.)"""
    sends = [D.speed_packet(3, True, 10), D.broadcast_stop()]
    rx, reports = two_threads(backend, sends)
    assert reports == sends[1:]


@model_only("four packets and the idles between; the RTL runs two")
def test_dcc_station_and_decoder_address_filter(backend):
    """Speed packets for 3 and 5, a broadcast stop and the reset packet:
    the decoder at 5 reports the last three, idles among them never."""
    sends = [D.speed_packet(3, True, 10), D.speed_packet(5, False, 3),
             D.broadcast_stop(), list(D.RESET)]
    rx, reports = two_threads(backend, sends)
    assert reports == sends[1:]
    assert any(is_idle(p) for p in rx.packets)


# ------------------------------------------------------- the programs
@pytest.mark.parametrize("name, thread, words, pairs, bounded", [
    ("dcc_station", 0, 105, 8, ["POP"]),
    ("dcc_decoder", 1, 177, 0, ["PUSH"]),
])
def test_dcc_assembles_strict_with_no_diagnostic(name, thread, words, pairs, bounded):
    """L2-DEADLINE: the station's every half-bit pair proved at the
    declared 4-clock tick; the decoder has no WAITD (it follows the line);
    the declarations are the guarded FIFO accesses only."""
    prog = program(name)
    assert prog.diagnostics == []
    assert list(prog.threads) == [thread]
    report = prog.deadlines[thread]
    assert report.period == 4
    assert not report.infeasible and not report.unbounded
    assert len(report.pairs) == pairs
    assert [d.name for d in report.declarations] == bounded
    assert len(prog.words) == words


def test_dcc_programs_fit_together():
    """D-020: the station fits thread 0's quarter; the decoder runs from
    0x080 into thread 2's quarter, which it marks unstartable; together
    they share no word."""
    st, dec = program("dcc_station"), program("dcc_decoder")
    assert max(st.words) < 128
    assert min(dec.words) == 128 and max(dec.words) == 128 + 177 - 1
    assert 2 in dec.unstartable and 0 not in dec.unstartable
    assert not set(st.words) & set(dec.words)


# ------------------------------------------------------ the model itself
def test_dcc_model_speed_table_and_baseline_packets():
    """S-9.2 figure 2: step 1 is C S3..S0 = 0 0010, step 2 1 0010, step
    28 1 1111, stop 0 0000, E-stop 0 0001; figure 1's packet is locomotive
    55 forward at step 6; the idle and reset packets and their error
    bytes (section B)."""
    assert D.speed_byte(True, 1) & 0x1F == 0b00010
    assert D.speed_byte(True, 2) & 0x1F == 0b10010
    assert D.speed_byte(True, 28) & 0x1F == 0b11111
    assert D.speed_byte(False, 0) == 0b01000000
    assert D.speed_byte(False, None) == 0b01000001
    assert D.speed_byte(True, 6) == 0b01110100           # 01DCSSSS, D = 1
    assert D.speed_packet(55, True, 6) == [0b00110111, 0b01110100]
    assert D.error_byte(D.IDLE) == 0xFF and D.error_byte(D.RESET) == 0x00
    bits = D.packet_bits(D.IDLE)
    assert bits[:15] == [1] * 14 + [0] and bits[-1] == 1
    assert len(bits) == 14 + 3 * 9 + 1


def feed(rx, halves, mhz=8.0):
    """Play ``halves`` (us) through ``rx`` on ``uo[0]``, the first high."""
    p = D.DccPlayer(("ui", 0), clock_mhz=mhz)
    p.queue(halves)
    from tools.protomodels.bench import Lines
    lines, c = Lines(), 0
    while not p.idle:
        lines.cycle, lines.uo = c, p.level
        p.observe(lines)
        rx.observe(lines)
        c += 1


def stream(*packets, tail=4, **enc):
    out = D.halves_of([1] * 14)
    for data in packets:
        out += D.halves_of(D.packet_bits(data), **enc)
    return out + D.halves_of([1] * tail)


def test_dcc_model_transmit_checks():
    """A nominal stream passes; a 54 us 1 half (below t1), 1 halves 4 us
    apart (over t1d) and a 12-bit preamble each fail."""
    rx = D.DccRx("OUT0", clock_mhz=8.0, transmit=True)
    feed(rx, stream(SPEED, D.IDLE))
    rx.check_preambles()
    assert rx.violations == [] and len(rx.packets) == 2
    for enc, kind in ((dict(t1=54.0), "t1"), (dict(t1=56.0, t1b=60.0), "t1d")):
        rx = D.DccRx("OUT0", clock_mhz=8.0, transmit=True)
        feed(rx, stream(SPEED, **enc))
        assert kind in [v.kind for v in rx.violations], rx.report()
    rx = D.DccRx("OUT0", clock_mhz=8.0, transmit=True)
    feed(rx, D.halves_of([1] * 14) + D.halves_of(D.packet_bits(SPEED)) +
         D.halves_of(D.packet_bits(SPEED, preamble=12)) + D.halves_of([1] * 4))
    rx.check_preambles()
    assert "preamble" in [v.kind for v in rx.violations]


@pytest.mark.parametrize("enc", [dict(t1=50.0), dict(t1=66.0), dict(t0=88.0),
                                 dict(t1=55.0, t1b=62.0)])
def test_dcc_model_receive_windows_reject(enc):
    """Outside tr1 (50, 66 us), under tr0 (88 us), 1 halves 7 us apart: no
    packet (the preamble or the start bit fails first, or a bit does)."""
    rx = D.DccRx("OUT0", clock_mhz=8.0)
    feed(rx, stream(SPEED, **enc))
    assert rx.packets == [], rx.report()
    rx = D.DccRx("OUT0", clock_mhz=8.0)
    feed(rx, stream(SPEED))
    assert [p.payload for p in rx.packets] == [SPEED]
