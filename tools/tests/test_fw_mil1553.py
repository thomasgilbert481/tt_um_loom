"""L3 for the MIL-STD-1553B programs, on the model and the RTL.

``firmware/mil1553_bc.loom`` is a bus controller on thread 0 (TXA = OUT0,
TXB = OUT1); ``firmware/mil1553_rt.loom`` a remote terminal on thread 1
(TXA = OUT2, TXB = OUT3); both read the bus on RXA = IN0, RXB = IN1. The
reference is ``tools.protomodels.mil1553``: ``Bus`` resolves the chip's
transmitters and the model terminals into one bus level and drives RXA/RXB
from it, ``decode`` reads every word off the bus independently of the
firmware, ``RTNode`` and ``BCNode`` are the far ends, with faults.

Expected values come from MIL-STD-1553B (the windows in the model's
docstring: response 4 to 12 us, no-response time-out at least 14 us, gap at
least 4 us, odd parity, the word formats) and from the program headers'
host-word formats, never from what the model or the RTL did.

Rates: TICK 25 (1 Mbit/s at 50 MHz) and TICK 24 (at 48 MHz). Zero crossings
are measured on the pins: exactly on the half-bit grid at 24 (a multiple of
the 4-clock slot), within 3 clocks at 25 (SEMANTICS 2), which is the
finding that a compliant transmitter (+-25 ns, 4.5.2.1.1.2) needs 48 MHz.

The PHY-GL-SDF candidate is ``test_bc_and_rt_on_one_bus`` at TICK 24: two
messages between the two threads, every edge on its grid, about 14,000
clocks.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import mil1553 as M
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
BC_SRC, RT_SRC = FIRMWARE / "mil1553_bc.loom", FIRMWARE / "mil1553_rt.loom"
ENGINE = range(0x100, 0x180)            # the shared word engine's addresses
NORESP, INVALID, COUNT, ADDR = 1, 2, 4, 8   # BC result bits (its header)
R_ME, R_BCAST = 1, 2                        # RT result bits (its header)
RT_ADDR = 5
STARTUP = 300


def clock_hz(tick):
    return tick * 2_000_000


def program(path):
    return assemble_file(path, isa=ISA, strict=True)


def bench_for(backend):
    """FIFOs, slice A of the bit engine, LD/ST and the 512-word memory; the
    RTL has them all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC", "DMEM"),
                             imem_words=512)
    return backend.bench()


def setup(backend, tick, bc=False, rt=False, nodes=(), rt_addr=RT_ADDR,
          load=None):
    """The chosen programs on one bus with the model ``nodes``; returns
    ``(bench, loom, bus)``. ``load``: words for the RT's transmit buffer."""
    bench = bench_for(backend)
    tx = ([("OUT0", "OUT1")] if bc else []) + ([("OUT2", "OUT3")] if rt else [])
    bus = bench.add(M.Bus(transmitters=tx))
    for node in nodes:
        bus.add(node)
    loom = backend.loom(bench, isa=ISA)
    image = {}
    for flag, src in ((bc, BC_SRC), (rt, RT_SRC)):
        if flag:
            image.update(program(src).words)
    loom.load(image)
    threads = ([0] if bc else []) + ([1] if rt else [])
    for t in threads:
        loom.write_csr(t, "TICK_INT", tick)
    loom.run(threads)
    if rt:
        loom.push(1, [rt_addr])
        if load:
            loom.push(1, [len(load)] + list(load))
    bench.step(STARTUP)
    return bench, loom, bus


def bc_message(bench, loom, cmd, data=(), budget=40_000):
    """The host has the BC run one message; returns ``(result, words)``."""
    since = bench.cycle
    loom.push(0, [cmd] + list(data))
    result = loom.pop(0, 1)[0]
    words = loom.pop(0, result >> 8) if result >> 8 else []
    return result, words, since


def rt_report(loom):
    """One RT report: ``(command, result, data words)``."""
    cmd, result = loom.pop(1, 2)
    words = []
    if not result & R_ME and result >> 8:
        words = loom.pop(1, result >> 8)
    return cmd, result, words


def tx_on_grid(words, tick):
    """Every crossing of the chip's words on its half-bit grid: exactly at
    TICK 24, within 3 clocks at 25 (SEMANTICS 2)."""
    tol = 0 if tick % 4 == 0 else 3
    for w in words:
        assert w.valid, (hex(w.value), w.errors)
        assert M.grid_deviation(w, clock_hz(tick)) <= tol, hex(w.value)
    return max(M.zc_deviation_ns(w, clock_hz(tick)) for w in words)


# ------------------------------------------------------------- assembly
def test_mil1553_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE: every pair of both programs proved at TICK 24 (so at 25
    too), nothing unbounded, no ``.bounded``. The BC fills thread 0's
    quarter, the RT thread 1's; both carry the word engine at 0x100 word for
    word, so their images agree where they overlap and load together."""
    bc, rt = program(BC_SRC), program(RT_SRC)
    for prog, thread in ((bc, 0), (rt, 1)):
        assert prog.diagnostics == [] and list(prog.threads) == [thread]
        report = prog.deadlines[thread]
        assert report.period == 24
        assert report.pairs and not report.infeasible and not report.unbounded
        assert report.worst_slack >= 0
    bc_main = [a for a in bc.words if a < 0x80]
    rt_main = [a for a in rt.words if 0x80 <= a < 0x100]
    assert len(bc_main) == max(bc_main) + 1 <= 128
    assert len(rt_main) <= 128 and min(rt_main) == 0x80
    engine_bc = {a: w for a, w in bc.words.items() if a in ENGINE}
    engine_rt = {a: w for a, w in rt.words.items() if a in ENGINE}
    assert engine_bc == engine_rt and len(engine_bc) > 80
    shared = set(bc.words) & set(rt.words)
    assert shared == set(engine_bc)
    assert len(bc.words) == 217 and len(rt.words) == 287


# ----------------------------------------------------------- the BC
#
# RTL budget (COMMON.md: about 770 k clocks): loading both programs over the
# host interface alone is about 64 k clocks a scenario, so each body runs one
# case on the RTL (about 630 k clocks in all) and its other cases run on the
# model only, in a twin marked with the reason.
def test_bc_sends_a_receive_command_and_data(backend):
    """BC to RT (4.3.3.6.1), four data words at TICK 25 (50 MHz)."""
    bc_receive(backend, 25, 4)


@model_only("one data word, and TICK 24: the RTL case runs the same path")
@pytest.mark.parametrize("tick, n", [(25, 1), (24, 1), (24, 4)])
def test_bc_sends_a_receive_command_other_cases(backend, tick, n):
    bc_receive(backend, tick, n)


def bc_receive(backend, tick, n):
    """BC to RT (4.3.3.6.1): command and n data words contiguous, every word
    valid with odd parity; the model RT takes them and answers after 8 us;
    the BC reports k = 1 with no flag and the status word."""
    rt = M.RTNode(RT_ADDR, clock_hz(tick))
    bench, loom, bus = setup(backend, tick, bc=True, nodes=[rt])
    data = [0x8001 + 0x1111 * i for i in range(n)]
    cmd = M.command(RT_ADDR, 0, 3, n)
    result, words, since = bc_message(bench, loom, cmd, data)
    assert result == 1 << 8 and words == [M.status(RT_ADDR)]
    assert rt.received == [(cmd, data, [])]
    sent = bus.tx_words(0, since, clock_hz(tick))
    assert [w.value for w in sent] == [cmd] + data
    assert [w.kind for w in sent] == [M.CS] + [M.DATA] * n
    assert M.check_contiguous(sent, clock_hz(tick)) == []
    zc = tx_on_grid(sent, tick)
    assert (zc <= M.ZC_TX_NS) == (tick == 24), zc
    assert not bus.collisions and not bus.both_high


def test_bc_reads_a_transmit_command_reply(backend):
    """RT to BC (4.3.3.6.2) at TICK 24 (48 MHz)."""
    bc_reads(backend, 24)


@model_only("TICK 25: the RTL case runs the same receive path")
def test_bc_reads_a_transmit_command_reply_at_25(backend):
    bc_reads(backend, 25)


def bc_reads(backend, tick):
    """RT to BC (4.3.3.6.2): the BC sends the command alone and reads the
    status and three data words that follow it."""
    data = [0x0000, 0xFFFF, 0x5A5A]
    rt = M.RTNode(RT_ADDR, clock_hz(tick), tx_words=data)
    bench, loom, bus = setup(backend, tick, bc=True, nodes=[rt])
    cmd = M.command(RT_ADDR, 1, 7, 3)
    result, words, since = bc_message(bench, loom, cmd)
    assert result == 4 << 8 and words == [M.status(RT_ADDR)] + data
    sent = bus.tx_words(0, since, clock_hz(tick))
    assert [w.value for w in sent] == [cmd]


@model_only("four 66 k-clock runs of one wait loop; the RTL runs no response")
@pytest.mark.parametrize("resp", [4.0, 12.0, 14.0, 20.0])
def test_bc_takes_a_status_anywhere_in_the_window(backend, resp):
    """4.3.3.8 and 4.3.3.9: a status word 4.0 or 12.0 us after the parity
    mid-bit is a response, and so is one at 14.0 us (the BC must wait at
    least that long before it calls it no response). At 20 us it is not:
    the BC gives up when no sync has started 15 us after the mid-bit."""
    rt = M.RTNode(RT_ADDR, response_us=resp)
    bench, loom, bus = setup(backend, 25, bc=True, nodes=[rt])
    result, words, _ = bc_message(bench, loom, M.command(RT_ADDR, 0, 1, 1), [7])
    if resp > 16.5:
        assert (result, words) == (NORESP, [])
    else:
        assert result == 1 << 8 and words == [M.status(RT_ADDR)]


def test_bc_reports_no_response_and_keeps_the_gap(backend):
    """No response at TICK 25 (50 MHz)."""
    bc_noresp(backend, 25)


@model_only("TICK 24: the RTL case runs the same timeout")
def test_bc_reports_no_response_at_24(backend):
    bc_noresp(backend, 24)


def bc_noresp(backend, tick):
    """A silent RT: result 'no response', k = 0. The next message's command
    starts no sooner than 14 us plus the 4 us gap after the first's parity
    mid-bit (4.3.3.9, 4.3.3.7), and then gets its answer."""
    hz = clock_hz(tick)
    rt = M.RTNode(RT_ADDR, hz)
    rt.fault = "silent"
    bench, loom, bus = setup(backend, tick, bc=True, nodes=[rt])
    cmd = M.command(RT_ADDR, 0, 1, 2)
    loom.push(0, [cmd, 1, 2, cmd, 3, 4])
    assert loom.pop(0, 1) == [NORESP]
    result = loom.pop(0, 1)[0]
    assert result == 1 << 8 and loom.pop(0, 1) == [M.status(RT_ADDR)]
    first, second = M.messages(bus.tx_words(0, 0, hz), hz)
    assert len(first) == len(second) == 3
    assert M.gap_us(first[-1], second[0], hz) >= M.NO_RESP_MIN_US + M.GAP_MIN_US


@model_only("three 74 k-clock runs of the receive path the RTL cases prove")
@pytest.mark.parametrize("fault, flag", [
    ({"word": 0, "parity_error": True}, INVALID),
    ({"word": 1, "violate": 9}, INVALID),
    ({"rt": 6}, ADDR),
])
def test_bc_flags_a_bad_reply(backend, fault, flag):
    """An invalid status or data word (4.4.1.1: parity, a missing mid-bit
    crossing) ends the reply with 'invalid response' and k = the valid
    words before it; a status from another address is flagged as such."""
    rt = M.RTNode(RT_ADDR, tx_words=[0x1234, 0x4321])
    rt.fault = fault
    bench, loom, bus = setup(backend, 25, bc=True, nodes=[rt])
    result, words, _ = bc_message(bench, loom, M.command(RT_ADDR, 1, 2, 2))
    assert result & 0xFF == flag
    if flag == ADDR:
        assert words == [M.status(6), 0x1234, 0x4321]
    else:
        assert words == [M.status(RT_ADDR)][:fault["word"]]
    result, words, _ = bc_message(bench, loom, M.command(RT_ADDR, 1, 2, 2))
    assert result == 3 << 8, "the BC recovers for the next message"


@model_only("command decoding in the BC's host loop; the RTL runs the word engine")
def test_bc_mode_commands(backend):
    """Mode commands (subaddress 0 or 31, 4.3.3.5.1.7): code 2 (transmit
    status word) has no data word; code 17 (synchronize with data word),
    received by the RT, has one."""
    rt = M.RTNode(RT_ADDR)
    bench, loom, bus = setup(backend, 25, bc=True, nodes=[rt])
    result, words, _ = bc_message(bench, loom, M.command(RT_ADDR, 1, 31, 2))
    assert result == 1 << 8 and words == [M.status(RT_ADDR)]
    sync = M.command(RT_ADDR, 0, 0, 17)
    result, words, _ = bc_message(bench, loom, sync, [0x00AB])
    assert result == 1 << 8 and rt.received[-1] == (sync, [0x00AB], [])


# ----------------------------------------------------------- the RT
def rt_bench(backend, tick, load=None):
    bc = M.BCNode(clock_hz(tick))
    bench, loom, bus = setup(backend, tick, rt=True, nodes=[bc], load=load)
    return bench, loom, bus, bc


def bc_sends(bench, bus, bc, cmd, data=(), quiet_us=30, **kw):
    """The model BC sends a message; returns the words the chip RT sent in
    reply and the bus words of the message, after ``quiet_us`` of bus time."""
    start = bench.cycle + 40
    bc.send(start, cmd, data, **kw)
    bench.step(int((len(data) + 1) * 40 * bc.h + quiet_us * bc.clock_hz / 1e6))
    sent = [w for w in bus.words(start - 1, bc.clock_hz) if w.mid_sync < start + 40 * bc.h * (len(data) + 1)]
    return bus.tx_words(0, start, bc.clock_hz), sent


def outq(loom, thread=1):
    return loom.fifo_status(thread)["outq"]


def test_rt_answers_a_receive_command(backend):
    """One data word at TICK 25 (50 MHz)."""
    rt_receive(backend, 25, 1)


@model_only("32 words is 124 k clocks, and TICK 24; the RTL runs one word")
@pytest.mark.parametrize("tick, n", [(24, 1), (25, 32), (24, 32)])
def test_rt_answers_a_receive_command_other_cases(backend, tick, n):
    rt_receive(backend, tick, n)


def rt_receive(backend, tick, n):
    """A valid receive command and its n words: the status word, mid-sync
    4 to 12 us after the last parity mid-bit (4.3.3.8; the program aims at
    the middle, 8 us plus its detection delay), on the half-bit grid; the host gets command, result, words."""
    bench, loom, bus, bc = rt_bench(backend, tick)
    data = [(0x9C3 * (i + 1)) & 0xFFFF for i in range(n)]
    cmd = M.command(RT_ADDR, 0, 9, n % 32)
    reply, sent = bc_sends(bench, bus, bc, cmd, data)
    assert [w.value for w in reply] == [M.status(RT_ADDR)]
    resp = M.response_us(sent[-1], reply[0], clock_hz(tick))
    assert M.RESP_MIN_US + 2 <= resp <= M.RESP_MAX_US - 2, resp
    tx_on_grid(reply, tick)
    assert rt_report(loom) == (cmd, n << 8, data)


def test_rt_answers_a_transmit_command_from_its_buffer(backend):
    """The transmit reply at TICK 24 (48 MHz, exact zero crossings)."""
    rt_transmit(backend, 24)


@model_only("TICK 25: the RTL case runs the same transmit path")
def test_rt_answers_a_transmit_command_at_25(backend):
    rt_transmit(backend, 25)


def rt_transmit(backend, tick):
    """A transmit command: the status word, then the loaded words, all
    contiguous (4.4.1.2) and on the grid; the host gets command and a
    clean result with k = 0."""
    data = [0xC0DE, 0x0001, 0x8000, 0x7FFF]
    bench, loom, bus, bc = rt_bench(backend, tick, load=data)
    bench.step(int(140 * clock_hz(tick) / 1e6))      # the RT polls INQ[1]
    cmd = M.command(RT_ADDR, 1, 4, 4)
    # 12 us response at most, then five 20 us words, then idle (4.3.3.8)
    reply, sent = bc_sends(bench, bus, bc, cmd, quiet_us=140)
    assert [w.value for w in reply] == [M.status(RT_ADDR)] + data
    assert [w.kind for w in reply] == [M.CS] + [M.DATA] * 4
    assert M.check_contiguous(reply, clock_hz(tick)) == []
    zc = tx_on_grid(reply, tick)
    assert (zc <= M.ZC_TX_NS) == (tick == 24), zc
    assert rt_report(loom) == (cmd, 0, [])
    assert not bus.collisions and not bus.both_high


@model_only("four messages, 96 k clocks; the RTL message-error case runs the decoder")
def test_rt_ignores_other_addresses_and_bad_commands(backend):
    """A command for another terminal, and one of ours with even parity or a
    bad sync, get no answer and no report (4.4.3.3); the next good one is
    answered."""
    bench, loom, bus, bc = rt_bench(backend, 25)
    for kw in ({"cmd": M.command(RT_ADDR + 1, 0, 1, 2), "data": [1, 2]},
               {"cmd": M.command(RT_ADDR, 1, 1, 2), "faults": {0: {"parity_error": True}}},
               {"cmd": M.command(RT_ADDR, 1, 1, 2), "faults": {0: {"bad_sync": True}}}):
        reply, _ = bc_sends(bench, bus, bc, kw.pop("cmd"), kw.pop("data", ()), **kw)
        assert reply == [] and outq(loom) == 0
    cmd = M.command(RT_ADDR, 0, 1, 1)
    reply, _ = bc_sends(bench, bus, bc, cmd, [0x0F0F])
    assert [w.value for w in reply] == [M.status(RT_ADDR)]
    assert rt_report(loom) == (cmd, 1 << 8, [0x0F0F])


def test_rt_message_error(backend):
    """A parity error in the second data word."""
    rt_message_error(backend, {"faults": {2: {"parity_error": True}}})


@model_only("five more faults at 93 k clocks each; the RTL runs a parity error")
@pytest.mark.parametrize("fault", [
    {"faults": {1: {"violate": 16}}},
    {"faults": {3: {"bad_sync": True}}},
    {"gaps": {2: 4}},
    {"short": True},
    {"long": True},
])
def test_rt_message_error_other_faults(backend, fault):
    rt_message_error(backend, dict(fault))


def rt_message_error(backend, fault):
    """An invalid data word, a gap before one (4.4.1.2), one word too few or
    too many: no status (4.4.3.6) and result 'message error'; the status
    word of the next message carries ME (4.3.3.5.3.3), the one after it is
    clean again (4.3.3.5.4)."""
    bench, loom, bus, bc = rt_bench(backend, 25)
    cmd = M.command(RT_ADDR, 0, 2, 3)
    data = [0x1111, 0x2222, 0x3333]
    if fault.pop("short", False):
        data = data[:2]
    if fault.pop("long", False):
        data = data + [0x4444]
    reply, _ = bc_sends(bench, bus, bc, cmd, data, **fault)
    assert reply == []
    got_cmd, result, words = rt_report(loom)
    assert got_cmd == cmd and result & R_ME and words == []
    good = M.command(RT_ADDR, 1, 31, 2)             # transmit status word
    for bits in (M.ME, 0):
        reply, _ = bc_sends(bench, bus, bc, good)
        assert [w.value for w in reply] == [M.status(RT_ADDR, bits)]
        assert rt_report(loom) == (good, 0, [])


@model_only("three messages, 93 k clocks; the RTL cases run the same decoder")
def test_rt_broadcast(backend):
    """A broadcast receive command (address 31, 4.3.3.6.7): data taken, no
    status, the broadcast bit in the next status word; a broadcast
    transmit command is ignored."""
    bench, loom, bus, bc = rt_bench(backend, 25)
    cmd = M.command(M.BROADCAST, 0, 6, 2)
    reply, _ = bc_sends(bench, bus, bc, cmd, [0xAB, 0xCD])
    assert reply == [] and rt_report(loom) == (cmd, 2 << 8 | R_BCAST, [0xAB, 0xCD])
    reply, _ = bc_sends(bench, bus, bc, M.command(M.BROADCAST, 1, 6, 2))
    assert reply == [] and outq(loom) == 0
    good = M.command(RT_ADDR, 1, 0, 2)
    reply, _ = bc_sends(bench, bus, bc, good)
    assert [w.value for w in reply] == [M.status(RT_ADDR, M.BCR)]
    rt_report(loom)


def test_bc_and_rt_on_one_bus(backend):
    """Both programs at TICK 24 (48 MHz); the PHY-GL-SDF candidate."""
    bc_and_rt(backend, 24)


@model_only("TICK 25: the RTL case runs the same two programs")
def test_bc_and_rt_on_one_bus_at_25(backend):
    bc_and_rt(backend, 25)


def bc_and_rt(backend, tick):
    """Both programs on one bus, threads 0 and 1: a receive message and a
    transmit message between them, every word valid, the RT's response in
    its window, no collision, every crossing on its grid."""
    hz = clock_hz(tick)
    out = [0x0123, 0x4567]
    bench, loom, bus = setup(backend, tick, bc=True, rt=True, load=out)
    bench.step(int(140 * hz / 1e6))                  # the RT takes its load
    cmd = M.command(RT_ADDR, 0, 1, 2)
    result, words, since = bc_message(bench, loom, cmd, [0xAAAA, 0x5555])
    assert result == 1 << 8 and words == [M.status(RT_ADDR)]
    assert rt_report(loom) == (cmd, 2 << 8, [0xAAAA, 0x5555])
    cmd2 = M.command(RT_ADDR, 1, 1, 2)
    result, words, _ = bc_message(bench, loom, cmd2)
    assert result == 3 << 8 and words == [M.status(RT_ADDR)] + out
    assert rt_report(loom) == (cmd2, 0, [])
    runs = M.messages(bus.words(since, hz), hz)
    assert [len(r) for r in runs] == [3, 1, 1, 3]
    assert 4.0 <= M.response_us(runs[0][-1], runs[1][0], hz) <= 12.0
    assert 4.0 <= M.response_us(runs[2][-1], runs[3][0], hz) <= 12.0
    assert M.gap_us(runs[1][-1], runs[2][0], hz) >= M.GAP_MIN_US
    tx_on_grid([w for r in runs for w in r], tick)
    assert not bus.collisions and not bus.both_high
