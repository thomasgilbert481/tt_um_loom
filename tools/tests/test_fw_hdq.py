"""L3-HDQ on the model and the RTL: firmware/hdq_host.loom and
firmware/hdq_gauge.loom, TI's HDQ battery-gauge interface (W-050, D-035).

The host (thread 0, HDQ = BIDIR0) is fed through ``tools.loomhost.Loom`` and
runs against ``tools.protomodels.hdq.HdqGauge`` on a ``OneWireBus`` line
(the open-drain wire with its pull-up; HDQ is not 1-Wire, but the wire is
the same); the gauge (thread 1, HDQ = BIDIR1) answers ``HdqHost``. Together,
the host on BIDIR0 and the gauge on BIDIR1 are one line: the bench joins
the two pads, as two Loom threads never share a pin (BUGS 14: both set
their OD_MASK bit first and read it back).

What is expected comes from TI SLUS556D (the bq27000 datasheet: the page 4
timing table, Figure 1, the HDQ section on pages 24-25) as
``protomodels.hdq`` cites it, and from the program headers (the host words,
the result codes, each program's chosen point inside each window), never
from what the model or the RTL did. Every low on the chip's pad is
measured: the host's cells and break to the clock and against the table,
the gauge's reply (t_RSPS, t_DW1, t_DW0, t_CYCD) against the table.

A register read is about 3.6 ms, 180000 clocks at 50 MHz, so:

* at the real rate (``US = 50``) the RTL runs one host read and one gauge
  reply, about 400000 clocks;
* the two programs together (a write and its read-back), the timeout and
  the break in mid-byte run on a time-scaled build, ``US = 10``: the same
  source with that one ``.equ`` changed, so every interval is the same
  number of ticks and one fifth as many clocks (the deadline checker
  proves both programs at 10), against models clocked at 10 MHz; the RTL
  runs those three too, about 350000 clocks;
* the window edges, the other faults and the start-phase sweep run on the
  model only.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import functools

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.hdq import (TABLE, HdqGauge, HdqHost, OneWireBus,
                                   bits_byte, check_gauge_pulls,
                                   check_host_pulls, command, host_bytes,
                                   lsb_bits)
from tools.protomodels.onewire import Agent
from tools.tests.fw_backend import model_only

ISA = load()
HOST = REPO / "firmware" / "hdq_host.loom"
GAUGE = REPO / "firmware" / "hdq_gauge.loom"
US = 50                           # clocks per microsecond: 50 MHz, the programs' rate
FAST = 10                         # the time-scaled build (module doc)
STARTUP = 400                     # clocks for a program's set-up after RUN
# hdq_host.loom's header: its times in us, the result codes.
HW1, HW0, CYCH, B, BR_MIN = 25, 115, 205, 250, 90
NO_ANSWER, SHORT, STUCK = 0x8001, 0x8002, 0x8003
# hdq_gauge.loom's header: its times in us from the R/W fall it saw.
DW1, DW0, CYCD, RSPS = 41, 112, 210, 266
REGS = {a: 0x11 * a for a in range(16)}       # the gauge image's file
SCALED = "a time-scaled build (US = 10, module doc): %s"


@functools.lru_cache(maxsize=None)
def program(path, us=US):
    """The program assembled with ``--strict``, for ``us`` clocks a microsecond."""
    text = path.read_text(encoding="utf-8")
    line = ".equ    US = 50 "
    assert text.count(line) == 1
    return assemble_text(text.replace(line, ".equ    US = %d " % us), path.name,
                         isa=ISA, strict=True)


def w_write(addr, data):
    """The host word for a WRITE (hdq_host.loom header)."""
    return command(addr, True) << 8 | data


def w_read(addr):
    return command(addr, False) << 8


W_BREAK = 0x0001


def hdq_bench(backend, bus):
    """The chip with the FIFOs, the bit engine, the deadline latch, the data
    memory and the 512-word memory; the RTL has all of them and
    ``RtlBench`` takes none of those keywords."""
    kwargs = dict(pullups=bus.pullups)
    if backend.name == "model":
        kwargs.update(features=("FIFO", "BE", "SETPD", "DMEM"), imem_words=512)
    bench = backend.bench(**kwargs)
    bench.add(bus)
    return bench


# ------------------------------------------------------------- the programs
@pytest.mark.parametrize("path,thread,words", [(HOST, 0, 94), (GAUGE, 1, 87)])
def test_hdq_programs_assemble_strict_with_no_diagnostic(path, thread, words):
    """L2-DEADLINE: every pair proved at the tick each program sets itself
    (TICK_INT 50, 1 us) and in the time-scaled build at 10; nothing
    unbounded, no ``.bounded``. Each program fits its thread's quarter, so
    the two load together."""
    for us in (US, FAST):
        prog = program(path, us)
        assert prog.diagnostics == []
        report = prog.deadlines[thread]
        assert report.period == us
        assert report.pairs and not report.infeasible and not report.unbounded
        assert report.worst_slack >= 0
    assert list(prog.threads) == [thread] and len(prog.words) == words
    assert all(thread * 128 <= a < (thread + 1) * 128 for a in prog.words)
    assert not set(program(HOST).words) & set(program(GAUGE).words)


# ------------------------------------------------------------------ host
def host_setup(backend, gauge=None, us=US):
    """The host program on thread 0 (BIDIR0), a model gauge on the line."""
    bus = OneWireBus(["BIDIR0"], clock=us * 1_000_000)
    if gauge is not None:
        bus.add(gauge)
    bench = hdq_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(HOST, us))
    loom.run(0)
    bench.step(STARTUP)
    return bench, bus, loom


def h_result(loom):
    """One result word (a read is ~3.6 ms: more polls than the default)."""
    return loom.pop(0, 1, max_polls=100_000)[0]


def host_items(bus, us, pin="BIDIR0"):
    """The chip host's lows, decoded and held to the table and to the
    header's times exactly: every 1 low HW1, every 0 low HW0, every break
    B, the bits of a byte CYCH apart (bytes at least CYCH), a break's rise
    at least BR_MIN before the next fall. Returns ``host_bytes`` of them."""
    pulls = [p for p in bus.intervals(bus.chip_name(pin)) if p[1] is not None]
    items, violations = check_host_pulls(pulls, us * 1_000_000)
    assert violations == []
    exact = {"1": HW1, "0": HW0, "break": B}
    for (fall, rise), kind in zip(pulls, items):
        assert rise - fall == exact[kind] * us, (kind, fall, rise)
    n = 0                                   # the bit's place in its byte
    for (f0, r0), (f1, _), k0, k1 in zip(pulls, pulls[1:], items, items[1:]):
        n = 0 if k0 == "break" else n + 1
        if k0 == "break":
            assert f1 - r0 >= BR_MIN * us
        elif k1 != "break" and n % 8:
            assert f1 - f0 == CYCH * us     # inside a byte: exact
        elif k1 != "break":
            assert f1 - f0 >= CYCH * us     # between bytes
    return host_bytes(items)


def test_hdq_host_reads_a_gauge_at_the_datasheet_rate(backend):
    """The host at 50 MHz: the break at start, then READ 0x05 against a
    model gauge that answers 0xA5 with its typical times. The result is
    0x00A5; on the pad, the break and the command byte 0x05, every low and
    cell exact to the clock (HW1 25 us, HW0 115 us, cell 205 us, break
    250 us), inside SLUS556D's windows; the gauge model found no host cell
    outside the table."""
    gauge = HdqGauge({5: 0xA5}, clock=US * 1_000_000)
    bench, bus, loom = host_setup(backend, gauge)
    loom.push(0, [w_read(5)])
    assert h_result(loom) == 0x00A5
    assert host_items(bus, US) == ["break", 0x05]
    assert gauge.reads == [(5, 0xA5)] and gauge.violations == [] and len(gauge.breaks) == 1


@model_only(SCALED % "test_hdq_host_and_gauge_together writes and reads back on the RTL")
def test_hdq_host_writes_and_reads_back(backend):
    """WRITE 0x3C to 0x03 and READ 0x03 (time-scaled build): the gauge model
    takes the write, the read gives 0x003C, the pad carries the break and
    the bytes 0x83, 0x3C, 0x03, each low exact and inside the table."""
    gauge = HdqGauge(clock=FAST * 1_000_000)
    bench, bus, loom = host_setup(backend, gauge, FAST)
    loom.push(0, [w_write(3, 0x3C), w_read(3)])
    assert h_result(loom) == 0x003C
    assert gauge.writes == [(3, 0x3C)] and gauge.violations == []
    assert host_items(bus, FAST) == ["break", 0x83, 0x3C, 0x03]


def test_hdq_host_times_out_and_breaks(backend):
    """A silent gauge (time-scaled build): READ 0x07 gives NO ANSWER
    (0x8001) and the host sends its recovery break by itself; the gauge
    back, the next READ gives its byte."""
    gauge = HdqGauge({7: 0x5E}, clock=FAST * 1_000_000, respond=False)
    bench, bus, loom = host_setup(backend, gauge, FAST)
    loom.push(0, [w_read(7)])
    assert h_result(loom) == NO_ANSWER
    bench.step((B + 20) * FAST)
    assert host_items(bus, FAST) == ["break", 0x07, "break"]
    assert len(gauge.breaks) == 2
    gauge.respond = True
    loom.push(0, [w_read(7)])
    assert h_result(loom) == 0x005E
    assert gauge.violations == []


#: A gauge at the edges of SLUS556D's windows (and one past reading A).
GAUGE_EDGES = {
    "fast": dict(dw1_us=32, dw0_us=80, cycd_us=190, rsps_us=190),
    "slow": dict(dw1_us=50, dw0_us=145, cycd_us=250, rsps_us=320),
    "rsps_from_cell_end": dict(rsps_us=CYCH + 320),   # QUESTIONS item 1, reading B
}


@model_only(SCALED % "three gauges at the window edges")
@pytest.mark.parametrize("edge", sorted(GAUGE_EDGES))
def test_hdq_host_reads_gauges_at_the_window_edges(backend, edge):
    """The host reads 0x96 (both bit values in every position pair) from a
    gauge whose every low, cell and response time is at the minimum, or at
    the maximum, of its window; and from one that answers t_RSPS after the
    end of the host's R/W cell, the other reading of the datasheet, which
    the host's window covers (header)."""
    gauge = HdqGauge({0x2A: 0x96}, clock=FAST * 1_000_000, **GAUGE_EDGES[edge])
    bench, bus, loom = host_setup(backend, gauge, FAST)
    loom.push(0, [w_read(0x2A), w_read(0x2A)])
    assert loom.pop(0, 2, max_polls=100_000) == [0x0096, 0x0096]
    assert gauge.violations == []


@model_only(SCALED % "the reply faults")
@pytest.mark.parametrize("fault,code", [("stop_after", SHORT), ("stuck", STUCK),
                                        ("late", NO_ANSWER)])
def test_hdq_host_reports_reply_faults(backend, fault, code):
    """A gauge that stops after three bits (SHORT BYTE, 0x8002), holds its
    third bit low 400 us (STUCK LOW, 0x8003), or answers 30 us past the
    host's response window (NO ANSWER): the code, then the host's recovery
    break, then a good read. (The stuck gauge still holds the line when
    the recovery break starts, so its end looks like a short low to the
    gauge model: the test sends one more BREAK before the good read.)"""
    kw = {"stop_after": dict(stop_after=3),
          "stuck": dict(stop_after=3, bits_us={2: 400}),
          "late": dict(rsps_us=CYCH + 320 + 15 + 30)}[fault]
    gauge = HdqGauge({1: 0x3B}, clock=FAST * 1_000_000, **kw)
    bench, bus, loom = host_setup(backend, gauge, FAST)
    loom.push(0, [w_read(1)])
    assert h_result(loom) == code
    bench.step(800 * FAST)
    assert host_items(bus, FAST)[-1] == "break"
    for name in kw:
        setattr(gauge, name, {"stop_after": None, "bits_us": {}, "rsps_us": 255}[name])
    loom.push(0, [W_BREAK, w_read(1)])
    assert h_result(loom) == 0x003B


@model_only(SCALED % "words the host program ignores")
def test_hdq_host_ignores_other_words_and_breaks_on_request(backend):
    """A word with bit 15 clear and bits 7:0 neither 0 nor 1 does nothing
    on the line; BREAK (0x0001) sends a break; then a read."""
    gauge = HdqGauge({0x7F: 0xC3}, clock=FAST * 1_000_000)
    bench, bus, loom = host_setup(backend, gauge, FAST)
    loom.push(0, [0x0502, 0x00FF])
    bench.step(2000 * FAST)
    assert host_items(bus, FAST) == ["break"]
    loom.push(0, [W_BREAK, w_read(0x7F)])
    assert h_result(loom) == 0x00C3
    assert host_items(bus, FAST) == ["break", "break", 0x7F]
    assert len(gauge.breaks) == 2 and gauge.violations == []


# ----------------------------------------------------------------- gauge
def gauge_setup(backend, us=US, **host_kw):
    """The gauge program on thread 1 (BIDIR1), a model host on the line."""
    bus = OneWireBus(["BIDIR1"], clock=us * 1_000_000)
    host = bus.add(HdqHost(clock=us * 1_000_000, **host_kw))
    bench = hdq_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(GAUGE, us))
    loom.run(1)
    bench.step(STARTUP)
    return bench, bus, loom, host


def run_ops(bench, host, ms=20, us=US):
    assert bench.run_until(lambda: host.idle, ms * 1000 * us, every=500)


def gauge_reply(bus, op, us, pin="BIDIR1"):
    """The chip gauge's reply to ``op``, held to the table and to the
    header's times: t_RSPS from the R/W fall 266 us (less up to one tick,
    plus up to 10 clocks for seeing the fall), every low DW1 or DW0 and
    every cell CYCD exact to the clock. Returns the byte."""
    pulls = [p for p in bus.intervals(bus.chip_name(pin)) if p[1] is not None]
    bits, violations = check_gauge_pulls(pulls, op.rw_fall, us * 1_000_000)
    assert violations == [] and len(bits) == 8
    lows = [p for p in pulls if p[0] > op.rw_fall][:8]
    assert (RSPS - 1) * us <= lows[0][0] - op.rw_fall <= RSPS * us + 10
    for (fall, rise), bit in zip(lows, bits):
        assert rise - fall == (DW1 if bit else DW0) * us
    assert all(b[0] - a[0] == CYCD * us for a, b in zip(lows, lows[1:]))
    assert bits == op.bits and op.violations == []
    return bits_byte(bits)


def test_hdq_gauge_answers_a_host_at_the_datasheet_rate(backend):
    """The gauge at 50 MHz against a model host with the datasheet's typical
    times: break, READ 0x0B. The model host reads 0xBB (the image's file);
    the reply on the pad is inside every window of SLUS556D and exact to
    the header's times."""
    bench, bus, loom, host = gauge_setup(backend)
    host.break_()
    op = host.read(0x0B)
    run_ops(bench, host)
    assert op.status == "ok" and op.result == 0xBB
    assert gauge_reply(bus, op, US) == 0xBB


def test_hdq_gauge_resynchronises_after_a_break_in_mid_byte(backend):
    """Breaks in mid-byte (time-scaled build): a write cut after four bits
    of its command byte, another after four bits of its data byte. Neither
    is taken: the only report in OUTQ is the full WRITE 0x5A to 0x02 that
    follows, and the read after it gives 0x5A."""
    bench, bus, loom, host = gauge_setup(backend, FAST)
    host.break_()
    host.write(2, 0x99, break_after=4)
    host.write(2, 0x99, break_after=12)
    host.write(2, 0x5A)
    op = host.read(2)
    run_ops(bench, host, 20, FAST)
    assert op.result == 0x5A and gauge_reply(bus, op, FAST) == 0x5A
    assert loom.pop_available(1) == [0x825A]


#: A host at the edges of SLUS556D's windows (t_CYCH, t_B, t_BR have no
#: maximum: the slow host's cell is twice the minimum, the line idle long).
HOST_EDGES = {
    "fast": dict(hw1_us=0.5, hw0_us=86, cych_us=190, b_us=190, br_us=40),
    "slow": dict(hw1_us=50, hw0_us=145, cych_us=380, b_us=1000, br_us=500),
}


@model_only(SCALED % "two hosts at the window edges")
@pytest.mark.parametrize("edge", sorted(HOST_EDGES))
def test_hdq_gauge_serves_hosts_at_the_window_edges(backend, edge):
    """Every host low and cell at the minimum, or the maximum, of its
    window (a 0.5 us 1, an 86 us 0, a 190 us cell and break, 40 us of
    recovery; a 50 us 1, a 145 us 0): WRITE 0x6C to 0x09 is taken and
    reported, READ 0x09 gives it back with the reply inside the table."""
    bench, bus, loom, host = gauge_setup(backend, FAST, **HOST_EDGES[edge])
    host.break_()
    host.write(9, 0x6C)
    op = host.read(9)
    run_ops(bench, host, 30, FAST)
    assert op.result == 0x6C and gauge_reply(bus, op, FAST) == 0x6C
    assert loom.pop_available(1) == [0x896C]


@model_only(SCALED % "addresses outside the register file")
def test_hdq_gauge_outside_the_file(backend):
    """QUESTIONS item 4: a read of 0x40 answers 0x00; a write to 0x40 is
    ignored (a read still gives 0x00) but reported as received."""
    bench, bus, loom, host = gauge_setup(backend, FAST)
    host.break_()
    ops = [host.read(0x40), host.write(0x40, 0x12), host.read(0x40), host.read(0x0F)]
    run_ops(bench, host, 30, FAST)
    assert [op.result for op in ops] == [0x00, None, 0x00, 0xFF]
    assert loom.pop_available(1) == [0xC012]


@model_only(SCALED % "a full OUTQ")
def test_hdq_gauge_full_outq_drops_reports_not_writes(backend):
    """Five writes with nobody popping OUTQ (depth 4): the first four are
    reported, the fifth report is dropped, all five registers are written
    (read back), and the gauge kept every cell in step."""
    bench, bus, loom, host = gauge_setup(backend, FAST)
    host.break_()
    for a in range(5):
        host.write(a, 0xF0 | a)
    ops = [host.read(a) for a in range(5)]
    run_ops(bench, host, 60, FAST)
    assert [op.result for op in ops] == [0xF0 | a for a in range(5)]
    assert loom.pop_available(1) == [0x80F0 | a << 8 | a for a in range(4)]


class Pulse(Agent):
    """Another driver on the line: pulls it low for ``length`` clocks from
    cycle ``at``."""

    name = "pulse"

    def __init__(self, at, length):
        self.at, self.end = at, at + length
        self.wake = at - 1

    def clock(self, level, cycle):
        self.wake = (self.at - 1 if cycle + 1 < self.at else
                     self.end - 1 if cycle + 1 < self.end else 1 << 62)
        return self.at <= cycle + 1 < self.end


@model_only(SCALED % "a break while the gauge sends")
def test_hdq_gauge_stops_its_reply_at_a_break(backend):
    """A break (250 us low) laid over the gauge's reply to READ 0x0C (0xCC),
    starting in the high part of its third cell: the gauge makes no fall of
    its own after it (it sees the line low before its next fall), takes it
    as a break, and answers the next read in full."""
    bench, bus, loom, host = gauge_setup(backend, FAST)
    host.break_()
    op = host.read(0x0C)
    assert bench.run_until(lambda: op.rw_fall is not None, 2_000_000, every=100)
    at = op.rw_fall + (RSPS + 2 * CYCD + 100) * FAST
    bus.add(Pulse(at, B * FAST))
    run_ops(bench, host, 20, FAST)
    bench.step((B + 300) * FAST)
    mine = [p for p in bus.intervals(bus.chip_name("BIDIR1")) if p[0] > op.rw_fall]
    assert len(mine) == 3 and mine[-1][1] < at
    again = host.read(0x0C)
    run_ops(bench, host, 20, FAST)
    assert again.result == 0xCC and gauge_reply(bus, again, FAST) == 0xCC


# ----------------------------------------------------------- the two together
def together(backend, start, us=FAST):
    """Host (thread 0, BIDIR0) and gauge (thread 1, BIDIR1), the pads joined
    by the bench as one line, started by ``start(bench, loom)``."""
    bus = OneWireBus(["BIDIR0", "BIDIR1"], clock=us * 1_000_000)
    bench = hdq_bench(backend, bus)
    loom = backend.loom(bench, isa=ISA)
    words = dict(program(HOST, us).words)
    gauge = program(GAUGE, us).words
    assert not set(words) & set(gauge)
    words.update(gauge)
    loom.load(words)
    start(bench, loom)
    bench.step(STARTUP)
    return bench, bus, loom


class _Reply:
    """What ``gauge_reply`` needs of a read whose host is the chip."""

    def __init__(self, rw_fall, bits):
        self.rw_fall, self.bits, self.violations = rw_fall, bits, []


def chip_reads(bus, us, reads):
    """The gauge's replies to the chip host's reads: ``reads`` are the
    indices (among the host's bytes after its first break) of the read
    command bytes; each R/W fall is that byte's eighth bit."""
    pulls = [p for p in bus.intervals(bus.chip_name("BIDIR0")) if p[1] is not None]
    out = []
    for k in reads:
        rw = pulls[8 + 8 * k][0]
        gp = [p for p in bus.intervals(bus.chip_name("BIDIR1")) if p[0] > rw][:8]
        bits = [1 if (r - f) < 65 * us else 0 for f, r in gp]
        out.append(gauge_reply(bus, _Reply(rw, bits), us))
    return out


def test_hdq_host_and_gauge_together(backend):
    """Both programs under one RUN, one line (time-scaled build): WRITE 0xC4
    to 0x03, READ 0x03. The host gets 0x00C4, the gauge reports 0x83C4;
    every host low exact, the gauge's reply inside the table and exact to
    its header. (The start-phase sweep reads the image's 0xEE from 0x0E.)"""
    bench, bus, loom = together(backend, lambda bench, loom: loom.run([0, 1]))
    loom.push(0, [w_write(3, 0xC4), w_read(3)])
    assert h_result(loom) == 0x00C4
    assert loom.pop_available(1) == [0x83C4]
    assert host_items(bus, FAST) == ["break", 0x83, 0xC4, 0x03]
    assert chip_reads(bus, FAST, [2]) == [0xC4]
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
@pytest.mark.parametrize("order", ["one_run", "host_first", "gauge_first"])
@pytest.mark.parametrize("p", [0, 1, 2, 3])
def test_hdq_together_at_every_start_phase(backend, p, order):
    """BUGS 14: both programs set their OD_MASK bit first and read it back.
    Under one RUN at each of the four phases of the slot grid, and each
    started p clocks after the other in both orders, a read is right."""
    bench, bus, loom = together(backend, _phase(p, order))
    loom.push(0, [w_read(0x0E)])
    assert h_result(loom) == 0x00EE
    assert chip_reads(bus, FAST, [0]) == [0xEE]


# ------------------------------------------------- the reference model alone
def test_hdq_model_table_and_helpers():
    """SLUS556D page 4 as the model holds it, the command byte (W/R in bit
    7), LSB-first bits and the classification of a host low."""
    assert TABLE == dict(t_B=(190, None), t_BR=(40, None), t_CYCH=(190, None),
                         t_HW1=(0.5, 50), t_HW0=(86, 145), t_RSPS=(190, 320),
                         t_CYCD=(190, 250), t_DW1=(32, 50), t_DW0=(80, 145))
    assert command(0x05, False) == 0x05 and command(0x05, True) == 0x85
    assert lsb_bits(0x85) == [1, 0, 1, 0, 0, 0, 0, 1] and bits_byte(lsb_bits(0x5A)) == 0x5A
    clk, us = 1_000_000, 1
    lows = [(0, 25), (205, 320), (410, 470), (615, 775), (1000, 1250), (1260, 1300)]
    items, violations = check_host_pulls(lows, clk)
    assert items == ["1", "0", "bad", "bad", "break", "1"]
    assert [v[0] for v in violations] == ["t_HW0", "t_B", "t_BR"] and us == 1


def test_hdq_model_host_and_gauge():
    """The reference host against the reference gauge, no chip: writes and
    reads at the typical times give no violation; a host 1 of 60 us, a
    150 us cell and a gauge 1 of 55 us are each caught by the other side."""
    bus = OneWireBus(["BIDIR0"])
    gauge = bus.add(HdqGauge())
    host = bus.add(HdqHost())
    ops = [host.break_(), host.write(0x11, 0x42), host.read(0x11)]
    assert bus.run_alone(lambda: host.idle, 2_000_000)
    assert ops[2].result == 0x42 and gauge.writes == [(0x11, 0x42)]
    assert gauge.violations == [] and ops[2].violations == []
    host.bits_us = {0: 60}
    host.cych_us = 150
    gauge.dw1_us = 55
    op = host.read(0x11)
    assert bus.run_alone(lambda: host.idle, 2_000_000)
    assert {v[0] for v in gauge.violations} == {"t_HW0", "t_CYCH"}
    assert {v[0] for v in op.violations} == {"t_DW1"}


def test_hdq_model_gauge_takes_a_break_mid_byte_and_a_silent_gauge_times_out():
    """A write cut by a break after five bits is not taken, the next one is;
    a silent gauge leaves the host's read at "no_answer"."""
    bus = OneWireBus(["BIDIR0"])
    gauge = bus.add(HdqGauge({3: 0x77}))
    host = bus.add(HdqHost())
    host.break_()
    host.write(3, 0x01, break_after=5)
    host.write(3, 0x02, break_after=13)
    host.write(3, 0x03)
    op = host.read(3)
    assert bus.run_alone(lambda: host.idle, 3_000_000)
    assert gauge.writes == [(3, 0x03)] and op.result == 0x03 and len(gauge.breaks) == 3
    gauge.respond = False
    op = host.read(3)
    assert bus.run_alone(lambda: host.idle, 3_000_000)
    assert op.status == "no_answer" and op.result is None
