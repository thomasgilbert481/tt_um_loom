"""L3-J1708 on the model and the RTL: firmware/j1708_node.loom.

One node per thread, each with its own pins (TX = OUTt, RX = INt);
``tools.protomodels.j1708.J1708Bus`` makes the bus the wired AND of the
chip's TX pins and the model nodes, seen by the chip's RX pins. The
program spills into the next thread's quarter, so two Loom nodes run on
threads 0 and 2.

Expected values come from the public J1708 descriptions through the
reference model (checksum, byte format, the 2-bit-time gap, the bus access
time of 10 + 2 P bit times) and from the program header (host formats),
never from what the model or the RTL did. Pin timing is measured on the
bus: every edge of every byte against the bit grid its start edge sets,
the gaps inside a message, and each message's start against the bus access
time from the end of the previous activity.

Rates: 9600 bit/s at 50 MHz on the model, and for the RTL a test rate of
256 clocks a bit (the deadline proofs do not depend on the rate).

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.j1708 import (CLOCK, J1708Bus, J1708Node,
                                     J1708Sniffer, access_bits, check_access,
                                     check_message, frame, sum_ok)
from tools.tests.fw_backend import model_only

ISA = load()
NODE = REPO / "firmware" / "j1708_node.loom"
IMEM = 512

# A tick is half a bit (program header).
FAST_T = 256                        # the RTL test rate: 256 clocks a bit
FAST = CLOCK / FAST_T
TICKS = {FAST: (128, 0), 9600: (2604, 43)}

# The report (program header).
OWN, LOST, FRAMING, OVERRUN, CKERR = 1, 2, 4, 8, 16
END = 0x8000
SLOT = 4


def bit_time(rate):
    ti, tf = TICKS[rate]
    return 2 * (ti + tf / 256)


def header(mid, n, priority):
    return mid | (n << 8) | ((priority - 1) << 13)


def node_image(threads):
    """The node on each of ``threads``: the file with only its ``.thread``
    and ``.pins`` lines changed, assembled for the 512-word memory."""
    words = {}
    text0 = NODE.read_text(encoding="utf-8")
    for t in threads:
        text = text0
        for old, new in ((".thread 0", ".thread %d" % t),
                         (".pins   TX = OUT0, RX = IN0",
                          ".pins   TX = OUT%d, RX = IN%d" % (t, t))):
            assert text.count(old) == 1, old
            text = text.replace(old, new)
        prog = assemble_text(text, NODE.name, isa=ISA, imem_words=IMEM, strict=True)
        assert prog.diagnostics == []
        overlap = set(words) & set(prog.words)
        assert not overlap, "the nodes overlap at %s" % sorted(overlap)[:4]
        words.update(prog.words)
    return words


def setup(backend, rate, threads=(0,), nodes=()):
    """Nodes on ``threads`` at ``rate``, model ``nodes`` and a sniffer on
    the bus; returns ``(bench, loom, bus, sniffer, run_cycle)``."""
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "DMEM"), imem_words=IMEM)
    else:
        bench = backend.bench()
    T = bit_time(rate)
    loom = backend.loom(bench, isa=ISA)
    # PIN_OUT resets to 0, a dominant TX: the host releases the TX pins
    # before the bus is joined (program header).
    loom.set_pin_out(sum(1 << (8 + t) for t in threads))
    ear = J1708Sniffer(T)
    bus = bench.add(J1708Bus(list(nodes) + [ear],
                             tx=["OUT%d" % t for t in threads],
                             rx=["IN%d" % t for t in threads]))
    loom.load(node_image(threads))
    ti, tf = TICKS[rate]
    for t in threads:
        loom.write_csr(t, "TICK_INT", ti)
        loom.write_csr(t, "TICK_FRAC", tf)
    run_cycle = bench.cycle
    loom.run(list(threads))
    bench.step(64)
    return bench, loom, bus, ear, run_cycle


def collect(loom, wanted, max_rounds=4000):
    """Pops the threads' reports until each thread ``t`` has given
    ``wanted[t]`` messages; returns ``{t: [(bytes, status bits)]}``."""
    got = {t: [] for t in wanted}
    cur = {t: [] for t in wanted}
    for _ in range(max_rounds):
        if all(len(got[t]) >= n for t, n in wanted.items()):
            return got
        for t in wanted:
            for w in loom.pop_available(t):
                if w & END:
                    got[t].append((cur[t], w & 0x7FFF))
                    cur[t] = []
                else:
                    assert w >> 8 == 0, hex(w)
                    cur[t].append(w)
    raise AssertionError("reports incomplete: %r" % got)


def assert_on_wire(msg, T):
    bad = check_message(msg, T)
    assert not bad, bad


# ------------------------------------------------------------ one node
def node_sends(backend, rate):
    """The chip node sends two messages; a model node reads them."""
    T = bit_time(rate)
    peer = J1708Node(CLOCK / T)
    bench, loom, bus, ear, run_cycle = setup(backend, rate, nodes=[peer])
    first = (0x80, [0x54, 0x01, 0xC8], 3)          # J1587-like: MID 128, PID 84
    second = (0x8E, [0x00], 6)
    for mid, data, p in (first, second):
        loom.push(0, [header(mid, len(data), p)] + data)
    rep = collect(loom, {0: 2})[0]
    assert rep == [(frame(*first[:2]), OWN), (frame(*second[:2]), OWN)], rep
    msgs = ear.decoder.messages
    assert [m.values for m in msgs] == [frame(*first[:2]), frame(*second[:2])]
    assert [m.values for m in peer.decoder.messages] == [m.values for m in msgs]
    for m in msgs:
        assert_on_wire(m, T)
    # bus access: the first after the idle that began at RUN (its start is
    # set by the host's pushes), the second after the end of the first,
    # which it follows within one bit and a few slots of its access time
    # (the idle count is in half-bit ticks, the idle loop notices the count
    # within its length, and the start bit then waits for the next tick)
    assert check_access(run_cycle, msgs[0].start, first[2], T)
    wait = msgs[1].start - msgs[0].end
    assert check_access(msgs[0].end, msgs[1].start, second[2], T), wait / T
    assert wait <= (access_bits(second[2]) + 1) * T + 16 * SLOT, wait / T
    assert loom.badop() == 0


def test_j1708_node_sends_fast(backend):
    """Two messages from the chip at the RTL test rate, read whole by a
    model node, on the bit grid, after their bus access times."""
    node_sends(backend, FAST)


@model_only("9600 bit/s: about 700,000 clocks")
def test_j1708_node_sends_9600(backend):
    """The same at J1708's own 9600 bit/s."""
    node_sends(backend, 9600)


def node_receives(backend, rate, *, clock_error=0.0, gaps=None):
    T = bit_time(rate)
    peer = J1708Node(CLOCK / T, clock_error=clock_error)
    bench, loom, bus, ear, run_cycle = setup(backend, rate, nodes=[peer])
    data = [0xBE, 0x00, 0xFF, 0x7E]
    sent = peer.send(0xAC, data, priority=8, gaps=gaps)
    rep = collect(loom, {0: 1})[0]
    assert sent.done and rep == [(frame(0xAC, data), 0)], rep
    assert [m.values for m in ear.decoder.messages] == [frame(0xAC, data)]
    assert loom.badop() == 0


def test_j1708_model_sends_to_node_fast(backend):
    """A model node's message, with gaps of exactly two bit times (the
    most J1708 allows) before two of its bytes, reaches the host as one
    message with a good checksum."""
    node_receives(backend, FAST, gaps=[2, 0, 0, 0, 2])


@model_only("9600 bit/s, two senders 2 % off: about 900,000 clocks")
@pytest.mark.parametrize("clock_error", [-0.02, 0.02])
def test_j1708_model_sends_to_node_9600(backend, clock_error):
    """At 9600 bit/s from a sender whose clock is 2 % off."""
    node_receives(backend, 9600, clock_error=clock_error)


def test_j1708_bad_checksum_fast(backend):
    """A spoiled checksum is reported as CKERR with the bytes as sent."""
    T = bit_time(FAST)
    peer = J1708Node(CLOCK / T)
    bench, loom, bus, ear, run_cycle = setup(backend, FAST, nodes=[peer])
    sent = peer.send(0x80, [0x5C, 0x12], checksum_error=True)
    rep = collect(loom, {0: 1})[0]
    assert rep == [(sent.values, CKERR)] and not sum_ok(sent.values)
    assert loom.badop() == 0


def test_j1708_gap_over_two_bits_ends_message_fast(backend):
    """A gap of three bit times inside a message ends it: the node and the
    model both report two messages, neither with a good sum."""
    T = bit_time(FAST)
    peer = J1708Node(CLOCK / T)
    bench, loom, bus, ear, run_cycle = setup(backend, FAST, nodes=[peer])
    sent = peer.send(0x80, [0x54, 0x01], gaps=[0, 3, 0])
    rep = collect(loom, {0: 2})[0]
    split = [m.values for m in ear.decoder.messages]
    assert split == [sent.values[:2], sent.values[2:]], split
    assert rep == [(sent.values[:2], CKERR), (sent.values[2:], CKERR)], rep
    assert loom.badop() == 0


# ----------------------------------------------------------- two nodes
TWO = (0, 2)                        # the program takes 1.4 quarters


def two_nodes(backend, a, b):
    """Loom nodes on threads 0 and 2 are given ``a`` and ``b`` (``(mid,
    data, priority)``) while a model node's message is on the bus, so both
    measure the idle from the same stop bit and lock their grids to the
    same start edges. Returns ``(ear, reports, trigger, T)``."""
    T = bit_time(FAST)
    peer = J1708Node(CLOCK / T)
    bench, loom, bus, ear, run_cycle = setup(backend, FAST, TWO, nodes=[peer])
    trigger = peer.send(0xF0, [0x11, 0x22], priority=1)
    bench.step(int(5 * T))                          # the trigger is going out
    for t, (mid, data, p) in zip(TWO, (a, b)):
        loom.push(t, [header(mid, len(data), p)] + data)
    reports = collect(loom, {0: 3, 2: 3})
    assert trigger.done and trigger.collisions == 0
    assert loom.badop() == 0
    return ear, reports, trigger, T


def test_j1708_two_nodes_priority_decides_fast(backend):
    """Ready together, the node with priority 2 starts first and the one
    with priority 5 waits for its own access time after that message;
    neither loses a bit."""
    a = (0x81, [0x01, 0x02], 5)                     # thread 0
    b = (0x82, [0x03], 2)                           # thread 2
    ear, rep, trigger, T = two_nodes(backend, a, b)
    msgs = ear.decoder.messages
    assert [m.values for m in msgs] == [trigger.values, frame(*b[:2]), frame(*a[:2])]
    for m in msgs:
        assert_on_wire(m, T)
    assert check_access(msgs[0].end, msgs[1].start, b[2], T)
    assert check_access(msgs[1].end, msgs[2].start, a[2], T)
    assert rep[0] == [(trigger.values, 0), (frame(*b[:2]), 0), (frame(*a[:2]), OWN)]
    assert rep[2] == [(trigger.values, 0), (frame(*b[:2]), OWN), (frame(*a[:2]), 0)]


def test_j1708_two_nodes_collide_fast(backend):
    """Same priority, so both start on the same tick: MID 0x80 beats 0x81
    at bit 0 (a dominant 0 over a released 1). The winner's message goes
    out undisturbed, the loser lets go at that bit, reports LOST with the
    winner's bytes, and sends its own after the next access time."""
    a = (0x81, [0x10, 0x20], 4)                     # thread 0: loses
    b = (0x80, [0x30, 0x40], 4)                     # thread 2: wins
    ear, rep, trigger, T = two_nodes(backend, a, b)
    msgs = ear.decoder.messages
    assert [m.values for m in msgs] == [trigger.values, frame(*b[:2]), frame(*a[:2])]
    for m in msgs:
        assert_on_wire(m, T)
    assert check_access(msgs[0].end, msgs[1].start, 4, T)
    assert check_access(msgs[1].end, msgs[2].start, 4, T)
    assert rep[2] == [(trigger.values, 0), (frame(*b[:2]), OWN), (frame(*a[:2]), 0)]
    assert rep[0] == [(trigger.values, 0), (frame(*b[:2]), LOST), (frame(*a[:2]), OWN)]


# ----------------------------------------------------- the model itself
def test_j1708_model_nodes_arbitrate():
    """The reference model alone: two model nodes queued at once with the
    same priority collide in the MID; the dominant one survives, the other
    retries after the access time (no chip involved)."""
    from tools.protomodels.bench import Bench
    T = 100.0
    x, y = J1708Node(CLOCK / T), J1708Node(CLOCK / T)
    ear = J1708Sniffer(T)
    bench = Bench(image={}, features=("FIFO",))
    bench.add(J1708Bus([x, y, ear], tx=(), rx=()))
    sx = x.send(0x81, [1], priority=3)
    sy = y.send(0x80, [2], priority=3)
    bench.step(int(120 * T))
    assert sy.done and sy.collisions == 0 and sx.done and sx.collisions == 1
    msgs = ear.decoder.messages
    assert [m.values for m in msgs] == [sy.values, sx.values]
    for m in msgs:
        assert not check_message(m, T)
    assert check_access(msgs[0].end, msgs[1].start, 3, T)
