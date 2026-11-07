"""L3-SWIM on the model and the RTL: firmware/swim_host.loom and
firmware/swim_target.loom, ST's SWIM debug wire for the STM8 (W-071, D-035).

The host (thread 0, SWIM = BIDIR0) is fed through ``tools.loomhost.Loom``
and runs against ``tools.protomodels.swim.SwimTarget``; the target (thread
1, SWIM = BIDIR1) answers ``SwimHost``. Together, the host on BIDIR0 and
the target on BIDIR1 are one line: the bench joins the two pads
(``OneWireBus``), as two Loom threads never share a pin (BUGS 14: both set
their OD_MASK bit first and read it back).

What is expected comes from ST UM0470 (DocID14024 Rev 4) as
``protomodels.swim`` cites it (the entry of 3.2, the bits of 3.3.2, the
frames of 3.4, the commands of 3.5 and Table 1, the reset of 3.6, Table 3)
and from the program headers (host words, result codes, the cell of 24
SWIM clocks, the 0 low of exactly 10 ticks), never from what the model or
the RTL did. Every low on the chip's pad is decoded and held to Table 3
(``check_lows``); the host's bit stream is compared with the frames the
spec builds (``frame_bits``); its entry with 3.2 (``check_entry``).

Rates: the bits always run at the real 8 MHz SWIM clock (a frame is about
1,500 clocks). The entry's pulse train is 6 ms, 300,000 clocks: the RTL
runs it once (``test_swim_host_enters_a_model_target_at_the_real_rate``);
every other scenario uses the time-scaled entry build, ``ETICK = 10`` in
both programs (the same source with that one ``.equ`` changed: the 1 kHz
and 2 kHz pulses ten times faster, the 16 us low and every bit unchanged;
the deadline checker proves both builds), against models whose entry is
scaled the same way.

On the RTL (about 800 k clocks): the real-rate entry, the host's NACK
retries, the target's NACK and resend, and the two programs together
(the PHY-GL-SDF candidate); the rest is ``model_only`` with its reason.

Every scenario's name starts with ``test_swim_``: test/test_fw.py loads
every module together and refuses a duplicate name.
"""

import functools

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.swim import (LS, SWIM_CSR, SWIM_HZ, SYNC_SC, TABLE3,
                                    OneWireBus, SwimHost, SwimTarget,
                                    check_entry, check_lows, decode,
                                    frame_bits, frames, parity)
from tools.tests.fw_backend import model_only

ISA = load()
HOST = REPO / "firmware" / "swim_host.loom"
TARGET = REPO / "firmware" / "swim_target.loom"
CLOCK = 50_000_000
SC = CLOCK / SWIM_HZ              # 6.25 clocks a SWIM clock
REAL, FAST = 100, 10              # ETICK: the real entry, the scaled build
STARTUP = 400                     # clocks for a program's set-up after RUN
# swim_host.loom's header: the tick, the cell, the 0 low, in clocks.
TICK = 12.5
CELL, LOW0 = 12 * TICK, 10 * TICK
LOW1 = (10, 13)
# Host words and results (swim_host.loom header).
W_SRST, W_ENTRY, W_CRESET = 0x0000, 0x0800, 0x0900
DONE, SYNC = 0x4000, 0x4100
NO_SYNC, NO_ANSWER, NACKED, BAD, STUCK = 0x8001, 0x8002, 0x8003, 0x8004, 0x8005
# swim_target.loom's header: the tick word, the reports, the memory.
TICK_WORD = 0x0C80
R_SRST, R_WOTF = 0x5000, 0x6000
RAM = {a: 0x10 + a for a in range(16)}
SCALED = "a scaled-entry build (ETICK = 10, module doc): %s"


@functools.lru_cache(maxsize=None)
def program(path, etick=REAL):
    """The program assembled with ``--strict``, for an entry tick ``etick``."""
    text = path.read_text(encoding="utf-8")
    line = ".equ    ETICK = 100 "
    assert text.count(line) == 1
    return assemble_text(text.replace(line, ".equ    ETICK = %d " % etick),
                         path.name, isa=ISA, strict=True)


def tick_word(hsi_error=0.0):
    """The target's tick for a SWIM clock ``hsi_error`` off 8 MHz: two SWIM
    clocks in 1/256 clocks, {TICK_INT, TICK_FRAC} (swim_target.loom)."""
    return round(2 * SC / (1 + hsi_error) * 256)


def w_rotf(n, addr):
    return [0x0100 | n, (addr >> 16) & 0xFF, addr & 0xFFFF]


def w_wotf(addr, data):
    return [0x0200 | len(data), (addr >> 16) & 0xFF, addr & 0xFFFF] + list(data)


def head_bits(cmd, n=None, addr=None):
    """The host's frames of 3.5: the command, then N and @E @H @L."""
    bits = frame_bits(0, cmd, 3)
    if n is not None:
        for v in (n, addr >> 16 & 0xFF, addr >> 8 & 0xFF, addr & 0xFF):
            bits += frame_bits(0, v, 8)
    return bits


def swim_bench(backend, bus):
    """The chip with the FIFOs, the bit engine, the latch, the data memory
    and the 512-word memory (the RTL has all of them)."""
    kwargs = dict(pullups=bus.pullups)
    if backend.name == "model":
        kwargs.update(features=("FIFO", "BE", "SETPD", "DMEM"), imem_words=512)
    bench = backend.bench(**kwargs)
    bench.add(bus)
    return bench


# ------------------------------------------------------------- the programs
@pytest.mark.parametrize("path,thread,words", [(HOST, 0, 210), (TARGET, 1, 197)])
def test_swim_programs_assemble_strict_with_no_diagnostic(path, thread, words):
    """L2-DEADLINE: every pair proved at the 12-clock tick (the checker
    rounds 12.5 down) in both entry builds; nothing unbounded, no
    ``.bounded``. Each program's quarter plus its spill quarter; the two
    load together."""
    for etick in (REAL, FAST):
        prog = program(path, etick)
        assert prog.diagnostics == []
        report = prog.deadlines[thread]
        assert report.period == 12
        assert report.pairs and not report.infeasible and not report.unbounded
        assert report.worst_slack >= 0
    prog = program(path)                  # the scaled target's MOV16s are shorter
    assert list(prog.threads) == [thread] and len(prog.words) == words
    quarters = {a // 128 for a in prog.words}
    assert quarters == {thread, thread + 2}
    assert not set(program(HOST).words) & set(program(TARGET).words)


# ------------------------------------------------------------------ host
def host_setup(backend, target=None, etick=FAST):
    """The host program on thread 0 (BIDIR0), a model target on the line."""
    bus = OneWireBus(["BIDIR0"], clock=CLOCK)
    if target is not None:
        bus.add(target)
    bench = swim_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(HOST, etick))
    loom.run(0)
    bench.step(STARTUP)
    return bench, bus, loom


def h_results(loom, n):
    return loom.pop(0, n, max_polls=100_000)


def host_stream(bus, pin="BIDIR0", entry=True, scale=FAST):
    """The chip host's lows, decoded and held to Table 3 and to the header:
    every 0 low exactly 10 ticks, every 1 low 10 to 13 clocks, the bits of
    a frame 12 ticks apart. Every run of lows over 64 SWIM clocks (the
    entry, a reset) is checked and dropped; ``entry`` holds the first nine
    lows to 3.2. Returns the host's bits in order."""
    pulls = [p for p in bus.intervals(bus.chip_name(pin)) if p[1] is not None]
    if entry:
        assert check_entry(pulls, CLOCK, scale=REAL / scale) == []
    items, violations = check_lows(pulls, CLOCK)
    assert violations == []
    bits, prev = [], None
    for (fall, rise), item in zip(pulls, items):
        if item == "long":
            prev = None
            continue
        low = rise - fall
        if item == 0:
            assert low == LOW0, (fall, low)
        else:
            assert LOW1[0] <= low <= LOW1[1], (fall, low)
        if prev is not None and fall - prev < 2 * CELL:
            assert fall - prev == CELL, (fall, fall - prev)
        prev = fall
        bits.append(item)
    return bits


def sync_ok(word, hsi_error=0.0):
    """SYNC, ss the target's 128 SWIM clocks in host ticks, +-1 (NOW at
    both edges, each seen on the slot grid)."""
    ticks = SYNC_SC * SC / (1 + hsi_error) / TICK
    return word & 0xFF00 == SYNC and abs((word & 0xFF) - ticks) <= 1.5


def test_swim_host_enters_a_model_target_at_the_real_rate(backend):
    """ENTRY at the real rate (6 ms of pulses), then WOTF 0xA0 to SWIM_CSR
    (3.2 step 6) and ROTF two bytes from 0x000004: SYNC with the target's
    128 SWIM clocks (64 ticks), DONE, 0x14, 0x15, DONE. On the pad: the
    entry inside 3.2, every bit inside Table 3 and exact to the header,
    the frames of 3.4 and 3.5; the model target found no host low outside
    Table 3 and logged both commands."""
    target = SwimTarget(dict(RAM))
    bench, bus, loom = host_setup(backend, target, REAL)
    loom.push(0, [W_ENTRY])               # 6 ms: the rest waits (INQ holds 4)
    res = h_results(loom, 1)
    loom.push(0, w_wotf(SWIM_CSR, [0xA0]) + w_rotf(2, 4))
    res += h_results(loom, 4)
    assert sync_ok(res[0]) and res[1:] == [DONE, 0x14, 0x15, DONE]
    assert host_stream(bus, scale=REAL) == (
        head_bits(2, 1, SWIM_CSR) + frame_bits(0, 0xA0, 8) + head_bits(1, 2, 4) + [1, 1])
    assert target.log == [(2, 1, SWIM_CSR, [0xA0]), (1, 2, 4, [0x14, 0x15])]
    assert len(target.entries) == 1 and target.violations == []


def test_swim_host_retries_a_nack_and_nacks_a_bad_byte(backend):
    """Scaled entry. The model target NACKs the host's N frame once (a good
    frame: "not ready", 3.4) and sends ROTF's first byte with the wrong
    parity once: the host sends N again, NACKs the bad byte, takes the
    same byte sent again (3.5.2), and every result is as without faults."""
    target = SwimTarget(dict(RAM), nack=[1], bad_parity=[0])
    bench, bus, loom = host_setup(backend, target)
    loom.push(0, [W_ENTRY] + w_wotf(SWIM_CSR, [0xA0]) + w_rotf(2, 0))
    res = h_results(loom, 5)
    assert sync_ok(res[0]) and res[1:] == [DONE, 0x10, 0x11, DONE]
    n_frame = frame_bits(0, 1, 8)
    assert host_stream(bus) == (
        frame_bits(0, 2, 3) + n_frame + n_frame + head_bits(2, 1, SWIM_CSR)[15:]
        + frame_bits(0, 0xA0, 8) + head_bits(1, 2, 0) + [0, 1, 1])
    assert [f[3:] for f in target.frames[:3]] == [(True, 1), (True, 0), (True, 1)]
    assert target.host_acks == [(0, 0), (0, 1), (1, 1)] and target.violations == []


@model_only(SCALED % "the error codes")
@pytest.mark.parametrize("fault,code", [("none", NO_SYNC), ("silent", NO_ANSWER),
                                        ("nack", NACKED), ("parity", BAD)])
def test_swim_host_reports_each_error(backend, fault, code):
    """No target on the line: ENTRY gives NO SYNC. A target that never
    ACKs: SRST gives NO ANSWER. One that NACKs every frame: SRST sends its
    command frame four times, then NACKED. One that sends a byte with the
    wrong parity four times: ROTF gives BAD FRAME after four NACKs. Then a
    good target, COMM RESET (3.6) and a read work."""
    kw = {"none": None, "silent": dict(silent=True), "nack": dict(nack=range(4)),
          "parity": dict(bad_parity=[0] * 4)}[fault]
    target = None if kw is None else SwimTarget(dict(RAM), csr=0xA0, **kw)
    bench, bus, loom = host_setup(backend, target)
    if target is None:
        loom.push(0, [W_ENTRY])
        assert h_results(loom, 1) == [NO_SYNC]
        target = bus.add(SwimTarget(dict(RAM), csr=0xA0, active=True))
    else:
        loom.push(0, [W_ENTRY] + (w_rotf(1, 2) if fault == "parity" else [W_SRST]))
        res = h_results(loom, 2)
        assert sync_ok(res[0]) and res[-1] == code
        if fault == "nack":
            assert [f[1:] for f in target.frames] == [(3, 0, True, 0)] * 4
        if fault == "parity":
            assert [a for _, a in target.host_acks] == [0, 0, 0, 0]
        target.silent, target.nack = False, set()
    loom.push(0, [W_CRESET] + w_rotf(1, 2))
    res = h_results(loom, 3)
    assert sync_ok(res[0]) and res[1:] == [0x12, DONE]


@model_only(SCALED % "the host against targets 2 % off")
@pytest.mark.parametrize("err", [-0.02, 0.02])
def test_swim_host_against_a_target_2_percent_off(backend, err):
    """A model target whose SWIM clock is 2 % off 8 MHz: the host's SYNC
    measures its 128 clocks (65 or 63 ticks), and a write and a read work;
    every target low decoded at the host's fixed sample point."""
    target = SwimTarget(dict(RAM), hsi_error=err)
    bench, bus, loom = host_setup(backend, target)
    loom.push(0, [W_ENTRY] + w_wotf(SWIM_CSR, [0xA0]) + w_wotf(3, [0x4B]) + w_rotf(2, 3))
    res = h_results(loom, 6)
    assert sync_ok(res[0], err) and res[1:] == [DONE, DONE, 0x4B, 0x14, DONE]
    assert target.violations == []


# ---------------------------------------------------------------- target
def target_setup(backend, etick=FAST, hsi_error=0.0, **host_kw):
    """The target program on thread 1 (BIDIR1) with its tick for a SWIM
    clock ``hsi_error`` off nominal, a model host (scaled entry) on the
    line."""
    bus = OneWireBus(["BIDIR1"], clock=CLOCK)
    host = bus.add(SwimHost(clock=CLOCK, entry_scale=REAL / etick, **host_kw))
    bench = swim_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(TARGET, etick))
    loom.run(1)
    loom.push(1, [tick_word(hsi_error)])
    bench.step(STARTUP)
    return bench, bus, loom, host


def run_ops(bench, host, ms=5):
    assert bench.run_until(lambda: host.idle, int(ms * CLOCK / 1000), every=500)


def target_lows(bus, hsi_error=0.0, pin="BIDIR1"):
    """The chip target's lows, held to Table 3 in its own SWIM clocks and to
    its header: the sync 64 ticks (128 SWIM clocks), every 0 low 10 ticks,
    every 1 low 10 to 13 clocks, a frame's bits 12 ticks apart (the tick is
    the word's, so within a clock of 12.5 / (1 + hsi_error) each).
    Returns the items (bits and "long")."""
    tick = tick_word(hsi_error) / 256
    pulls = [p for p in bus.intervals(bus.chip_name(pin)) if p[1] is not None]
    items, violations = check_lows(pulls, CLOCK, SWIM_HZ * (1 + hsi_error))
    assert violations == []
    prev = None
    for (fall, rise), item in zip(pulls, items):
        low = rise - fall
        if item == "long":
            assert abs(low - 64 * tick) <= 1, (fall, low)
            prev = None
            continue
        if item == 0:
            assert abs(low - 10 * tick) <= 1, (fall, low)
        else:
            assert LOW1[0] <= low <= LOW1[1], (fall, low)
        if prev is not None and fall - prev < 24 * tick:
            assert abs(fall - prev - 12 * tick) <= 1, (fall, fall - prev)
        prev = fall
    return items


@model_only("RTL budget: test_swim_host_and_target_together runs every command "
            "against the chip target on the RTL")
def test_swim_target_serves_a_model_host(backend):
    """Scaled entry. The model host enters, writes 0xA0 to SWIM_CSR, writes
    0x5A 0xC3 to 0x000002, reads 0x000001..0x000004 back, sends SRST: SYNC
    of 128 SWIM clocks; 0x11 0x5A 0xC3 0x14; the target reports the two
    WOTFs and the SRST (SWIM_DM set); every frame ACKed, every target low
    inside Table 3 and exact to its header; the host model found no
    target low outside Table 3."""
    bench, bus, loom, host = target_setup(backend)
    ops = [host.entry(), host.wotf(SWIM_CSR, [0xA0]), host.wotf(2, [0x5A, 0xC3]),
           host.rotf(4, 1), host.srst()]
    run_ops(bench, host)
    assert [op.status for op in ops] == ["ok"] * 5
    assert abs(ops[0].sync_sc - SYNC_SC) <= 0.5
    assert ops[3].data == [0x11, 0x5A, 0xC3, 0x14]
    assert loom.pop_available(1) == [R_WOTF, R_WOTF, R_SRST]
    assert all(op.nacks == 0 and op.bad == 0 for op in ops)
    target_lows(bus)
    assert host.violations == []


def test_swim_target_nacks_a_parity_error_and_resends_a_nacked_byte(backend):
    """The model host sends its first data frame (WOTF's N) with the wrong
    parity and NACKs the first byte the target sends: the target NACKs the
    bad frame and takes it again (3.4), and sends the NACKed byte again
    (3.5.2). The write lands (read back), and the host saw one NACK and
    received the byte twice, the same both times."""
    bench, bus, loom, host = target_setup(backend, bad_parity=[1], nack=[0])
    ops = [host.entry(), host.wotf(SWIM_CSR, [0xA0]), host.rotf(1, SWIM_CSR)]
    run_ops(bench, host)
    assert [op.status for op in ops] == ["ok"] * 3
    assert ops[1].nacks == 1 and ops[2].data == [0xA0] and ops[2].bad == 1
    assert host.received == [(0, 0xA0, True, 0), (1, 0xA0, True, 1)]
    target_lows(bus)
    assert host.violations == []


@model_only(SCALED % "the target's HSI 2 % off, both ways")
@pytest.mark.parametrize("hsi", ["slow", "fast"])
def test_swim_target_with_its_hsi_2_percent_off(backend, hsi):
    """The target's tick for a SWIM clock 2 % below or above 8 MHz (word
    0x0CC1 or 0x0C41) against a nominal model host: the entry is detected,
    the sync is 128 of the target's SWIM clocks (130.6 or 125.5 nominal
    ones, the host's measure), and a write and a read work; every target
    low is inside Table 3 in the target's own clocks."""
    err = {"slow": -0.02, "fast": 0.02}[hsi]
    bench, bus, loom, host = target_setup(backend, hsi_error=err)
    ops = [host.entry(), host.wotf(SWIM_CSR, [0xA0]), host.wotf(7, [0x3C]),
           host.rotf(2, 6)]
    run_ops(bench, host)
    assert [op.status for op in ops] == ["ok"] * 4
    assert abs(ops[0].sync_sc - SYNC_SC / (1 + err)) <= 0.5
    assert ops[3].data == [0x16, 0x3C]
    target_lows(bus, err)
    assert host.violations == []


@model_only(SCALED % "the communication reset")
def test_swim_target_resynchronises_on_a_communication_reset(backend):
    """A comm reset (3.6) between commands and one cutting a WOTF after its
    @H frame: each answered by a sync frame; the cut write is not taken
    (a read gives the old byte), the next one is."""
    bench, bus, loom, host = target_setup(backend)
    ops = [host.entry(), host.wotf(SWIM_CSR, [0xA0]), host.comm_reset()]
    run_ops(bench, host)
    cut = SwimHost(clock=CLOCK)
    assert [op.status for op in ops] == ["ok"] * 3
    host.tries = 1
    host.resp_timeout = 1
    bad = host.wotf(5, [0x77])              # times out after its first frame
    run_ops(bench, host)
    assert bad.status == "no_answer"
    host.tries, host.resp_timeout = cut.tries, cut.resp_timeout
    ops = [host.comm_reset(), host.rotf(1, 5), host.wotf(5, [0x66]), host.rotf(1, 5)]
    run_ops(bench, host)
    assert [op.status for op in ops] == ["ok"] * 4
    assert ops[1].data == [0x15] and ops[3].data == [0x66]
    assert abs(ops[0].sync_sc - SYNC_SC) <= 0.5


@model_only(SCALED % "SWIM_DM clear and addresses outside the map")
def test_swim_target_without_swim_dm_and_outside_the_map(backend):
    """With SWIM_CSR 0x00 (3.10.1): a RAM read gives 0x00, a RAM write is
    ignored, SRST does nothing (no report), SWIM_CSR itself is read and
    written. With 0xA0: @E = 1 and 0x000010 read 0x00 and ignore writes."""
    bench, bus, loom, host = target_setup(backend)
    ops = [host.entry(), host.rotf(1, 3), host.wotf(3, [0x99]), host.srst(),
           host.rotf(1, SWIM_CSR), host.wotf(SWIM_CSR, [0xA0]), host.rotf(1, 3),
           host.wotf(0x010003, [0x42]), host.wotf(0x10, [0x42]),
           host.rotf(2, 0x010003), host.rotf(1, 0x10), host.rotf(1, 3)]
    run_ops(bench, host, 10)
    assert all(op.status == "ok" for op in ops)
    assert [op.data for op in ops if op.kind == "rotf"] == [
        [0x00], [0x00], [0x13], [0x00, 0x00], [0x00], [0x13]]
    assert loom.pop_available(1) == [R_WOTF] * 4


# ----------------------------------------------------------- the two together
def together(backend, start, hsi_error=0.0):
    """Host (thread 0, BIDIR0) and target (thread 1, BIDIR1), the pads
    joined by the bench as one line, both scaled-entry builds, started by
    ``start(bench, loom)``; the target given its tick."""
    bus = OneWireBus(["BIDIR0", "BIDIR1"], clock=CLOCK)
    bench = swim_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(HOST, FAST).words)
    target = program(TARGET, FAST).words
    assert not set(words) & set(target)
    words.update(target)
    loom.load(words)
    start(bench, loom)
    loom.push(1, [tick_word(hsi_error)])
    bench.step(STARTUP)
    return bench, bus, loom


def test_swim_host_and_target_together(backend):
    """Both programs under one RUN, one line: ENTRY, WOTF 0xA0 to SWIM_CSR,
    WOTF 0xE7 to 0x000009, ROTF 0x000008..0x000009, SRST. The host gets
    SYNC (64 ticks), DONE, DONE, 0x18, 0xE7, DONE, DONE; the target
    reports both writes and the SRST; the host's bits are the frames of
    3.5; both pads exact to their headers; no contention."""
    bench, bus, loom = together(backend, lambda bench, loom: loom.run([0, 1]))
    loom.push(0, [W_ENTRY] + w_wotf(SWIM_CSR, [0xA0]) + w_wotf(9, [0xE7])
              + w_rotf(2, 8) + [W_SRST])
    res = h_results(loom, 7)
    assert sync_ok(res[0]) and res[1:] == [DONE, DONE, 0x18, 0xE7, DONE, DONE]
    assert loom.pop_available(1) == [R_WOTF, R_WOTF, R_SRST]
    assert host_stream(bus) == (
        head_bits(2, 1, SWIM_CSR) + frame_bits(0, 0xA0, 8) + head_bits(2, 1, 9)
        + frame_bits(0, 0xE7, 8) + head_bits(1, 2, 8) + [1, 1] + frame_bits(0, 0, 3))
    items = target_lows(bus)
    assert items[0] == "long" and items[1:] == (
        [1] * 17 + frame_bits(1, 0x18, 8) + frame_bits(1, 0xE7, 8) + [1])
    assert not bench.contentions


def _phase(p, order):
    def start(bench, loom):
        bench.step(p)
        if order == "one_run":
            loom.run([0, 1])
        else:
            first, second = (0, 1) if order == "host_first" else (1, 0)
            loom.run(first)
            bench.step(p)
            loom.run(second)
    return start


@model_only(SCALED % "twelve starts")
@pytest.mark.parametrize("order", ["one_run", "host_first", "target_first"])
@pytest.mark.parametrize("p", [0, 1, 2, 3])
def test_swim_together_at_every_start_phase(backend, p, order):
    """BUGS 14: both programs set their OD_MASK bit first and read it back.
    Under one RUN at each of the four phases of the slot grid, and each
    started p clocks after the other in both orders, ENTRY and a read of
    SWIM_CSR (0x00 at reset) are right."""
    bench, bus, loom = together(backend, _phase(p, order))
    loom.push(0, [W_ENTRY] + w_rotf(1, SWIM_CSR))
    res = h_results(loom, 3)
    assert sync_ok(res[0]) and res[1:] == [0x00, DONE]


@model_only("RTL budget (about 770 k clocks): the 2 % HSI runs on the model, three ways")
def test_swim_together_with_the_target_hsi_2_percent_slow(backend):
    """The chip host against the chip target with its tick 2 % long: the
    host measures 65 ticks of sync (128 target clocks) and reads back a
    write."""
    bench, bus, loom = together(backend, lambda bench, loom: loom.run([0, 1]), -0.02)
    loom.push(0, [W_ENTRY] + w_wotf(SWIM_CSR, [0xA0]) + w_wotf(0, [0x81]) + w_rotf(1, 0))
    res = h_results(loom, 5)
    assert sync_ok(res[0], -0.02) and res[1:] == [DONE, DONE, 0x81, DONE]
    target_lows(bus, -0.02)


# ------------------------------------------------- the reference model alone
def test_swim_model_constants_and_helpers():
    """UM0470 as the model holds it: 3.3.2's low-speed bit and decision,
    Table 3 read in SWIM clocks, 3.4's parity and frame order, and the
    decoding of a list of lows."""
    assert LS == dict(bit=22, low0=20, low1=2, max1=8)
    assert TABLE3 == dict(tb0=(16.0, 24.0), tb1=(1.5, 2.5))
    assert decode(2) == 1 and decode(8) == 1 and decode(9) == 0 and decode(20) == 0
    assert parity(0xA0) == 0 and parity(0x01) == 1
    assert frame_bits(0, 0b010, 3) == [0, 0, 1, 0, 1]
    assert frame_bits(1, 0x81, 8) == [1, 1, 0, 0, 0, 0, 0, 0, 1, 0]
    lows = [(0, 12), (150, 275), (300, 315), (450, 1450), (2000, 2100), (2200, 2205)]
    items, violations = check_lows(lows, CLOCK)
    assert items == [1, 0, 1, "long", 0, 1]
    assert violations == [(5, "tb1", 0.8)]
    assert frames(items) == [[1, 0, 1], [0, 1]]


@pytest.mark.parametrize("err", [0.0, 0.02, -0.02])
def test_swim_models_against_each_other(err):
    """The model host and the model target alone on a line (no chip), the
    target's clock ``err`` off: entry, WOTF 0xA0 to SWIM_CSR with a NACKed
    frame and a bad-parity frame retried, ROTF with a bad-parity byte, SRST,
    a comm reset, a read of SWIM_CSR. Every result right, no violation."""
    bus = OneWireBus(["BIDIR0"])
    t = bus.add(SwimTarget({0: 0x11, 1: 0x22, 2: 0x33}, hsi_error=err, nack=[3],
                           bad_parity=[1]))
    h = bus.add(SwimHost(bad_parity=[6], entry_scale=10))
    ops = [h.entry(), h.wotf(SWIM_CSR, [0xA0]), h.rotf(3, 0), h.wotf(1, [0x5A]),
           h.rotf(2, 0), h.srst(), h.comm_reset(), h.rotf(1, SWIM_CSR)]
    assert bus.run_alone(lambda: h.idle, 50_000_000)
    assert [op.status for op in ops] == ["ok"] * 8
    assert [ops[i].data for i in (2, 4, 7)] == [[0x11, 0x22, 0x33], [0x11, 0x5A], [0xA0]]
    assert ops[1].nacks == 2 and ops[2].bad == 1
    assert abs(ops[0].sync_sc - SYNC_SC / (1 + err)) < 0.5
    assert t.srsts == 1 and len(t.resets) == 1 and len(t.entries) == 1
    assert t.violations == [] and h.violations == []
