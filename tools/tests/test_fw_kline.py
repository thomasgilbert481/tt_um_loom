"""L3-KLINE on the model and the RTL: firmware/kline_tester.loom and
firmware/kline_ecu.loom, the ISO 9141-2 K line with its 5-baud
initialisation (W-081, D-035).

The tester (thread 0, TX = OUT0, RX = IN0) runs against
``tools.protomodels.kline.KLineEcu``; the ECU (thread 2, TX = OUT1, RX =
IN1) answers ``KLineTester``; together, the bench joins OUT0 and OUT1
into one line that both RX pins see (``KLineBus``), as two Loom threads
never share a pin.

What is expected comes from the secondary sources ``protomodels.kline``
cites (the init sequence, the headers, the checksum, the W1 .. W5 and
P1 .. P4 windows of its ``TIMING``) and from the program headers (host
words, reports), never from what the model or the RTL did. Every session
on the line is decoded and held to ``TIMING`` and every bit edge to its
byte's grid (``check_session``).

Time scale. A real session is about 2.4 s, 120 M clocks, of which the
5-baud address is 2 s; the golden model runs about 60 k clocks a second
and Icarus about 6 k. So the programs are assembled in three builds by
changing their three rate lines (``.equ MS``, ``HBI``, ``HBF``; the
deadline checker proves all three):

* ``real``: MS = 50000 (1 ms), the half bit 2403 + 218/256 (10400 bit/s);
* ``model``: MS = 500 (every W and P time and the 5-baud bit a hundred
  times shorter), the real 10400 bit/s bytes: about 2.4 M clocks a session;
* ``rtl``: MS = 25 (two thousand times shorter: a 5-baud bit is 5000
  clocks) and a 300-clock bit (166.7 kbit/s, 16 times faster): about
  130 k clocks a session.

The models take the same ``ms`` and ``bit`` and ``check_session`` holds
the line to the standard's windows in those ms. Real time runs on the
model in one opt-in scenario (``LOOM_KLINE_REAL_TIME=1``; it passed in
1 h 53 min on 2026-11-07). On the RTL: the tester with a model ECU, the
ECU with a model tester, the two together (the PHY-GL-SDF candidate), a
wrong inverted key and a bad request checksum; the rest is ``model_only``
with its reason.

Every scenario's name starts with ``test_kline_``: test/test_fw.py loads
every module together and refuses a duplicate name.
"""

import functools
import os

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.kline import (ADDR, KLineBus, KLineEcu, KLineTester,
                                     TIMING, check_session, response)
from tools.tests.fw_backend import model_only

ISA = load()
TESTER = REPO / "firmware" / "kline_tester.loom"
ECU = REPO / "firmware" / "kline_ecu.loom"
BUILDS = {"real": (50000, 2403, 218), "model": (500, 2403, 218), "rtl": (25, 150, 0)}
LINES = (".equ    MS = 50000 ", ".equ    HBI = 2403 ", ".equ    HBF = 218 ")
# kline_tester.loom's header: the status word.
OK, NO_SYNC, NO_KEYS, NO_ADDR, NO_RESP, CKERR = range(6)
OVERFLOW = 1 << 5
# kline_ecu.loom's header: the reports, its keys and address.
R_BADKEY, R_NOKEY, R_OPEN, R_REQ, R_CLOSED = 0x2000, 0x2100, 0x3000, 0x4000, 0x5000
ANSWERED, F_CK, F_FMT, F_TAB = 0x100, 0x200, 0x400, 0x800
KEYS, SRC = (0x08, 0x08), 0x10
PID00 = [0x41, 0x00, 0xBE, 0x1F, 0xA8, 0x13]      # mode 1 PID 0: supported PIDs
TABLE = {(1, 0): PID00, (1, 0x0C): [0x41, 0x0C, 0x1A, 0xF8]}


@functools.lru_cache(maxsize=None)
def program(path, build="real"):
    """The program assembled with ``--strict`` for one of :data:`BUILDS`."""
    text = path.read_text(encoding="utf-8")
    for line, value in zip(LINES, BUILDS[build]):
        assert text.count(line) == 1, line
        text = text.replace(line, line.split("=")[0] + "= %d " % value)
    return assemble_text(text, path.name, isa=ISA, strict=True)


def scale(build):
    """``(ms, bit)`` in clocks for a build."""
    ms, hbi, hbf = BUILDS[build]
    return ms, 2 * (hbi + hbf / 256)


def status(code, n=0):
    return 0x8000 | code << 12 | n


def table_words(table):
    words = [len(table)]
    for (mode, pid), data in table.items():
        words += [mode << 8 | pid, len(data)] + list(data)
    return words


def kline_bench(backend, *, tester=False, ecu=False, nodes=(), build="rtl"):
    """The chip with the chosen programs on one K line and model ``nodes``
    (added later with ``bus.add``, so that their own times start once the
    chip runs); returns ``(bench, loom, bus)``."""
    tx = (["OUT0"] if tester else []) + (["OUT1"] if ecu else [])
    rx = (["IN0"] if tester else []) + (["IN1"] if ecu else [])
    bus = KLineBus(list(nodes), tx=tx, rx=rx)
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "DMEM"), imem_words=512)
    else:
        bench = backend.bench()
    bench.add(bus)
    loom = backend.loom(bench, isa=ISA)
    words = {}
    for on, path in ((tester, TESTER), (ecu, ECU)):
        if on:
            words.update(program(path, build).words)
    # The read-back costs as many SPI clocks as the load: the model checks
    # it, the RTL (where every clock counts) trusts the load.
    loom.load(words, verify=backend.name == "model")
    loom.run(([0] if tester else []) + ([2] if ecu else []))
    if ecu:
        loom.push(2, table_words(TABLE))
    bench.step(200)
    return bench, loom, bus


def drain(bench, loom, thread, done, max_clocks, chunk=2000):
    """Step the bench, popping ``OUTQ[thread]`` every ``chunk`` clocks,
    until ``done(words)``; returns the words."""
    words = []
    spent = 0
    while not done(words):
        assert spent < max_clocks, ("timed out", [hex(w) for w in words])
        bench.step(chunk)
        spent += chunk
        words += loom.pop_available(thread)
    return words


def has_status(words):
    return any(w & 0x8000 for w in words)


def session_clocks(build):
    """A whole session's length bound in clocks: 2700 ms of W/P times and
    the address, and 30 bytes."""
    ms, bit = scale(build)
    return int(2700 * ms + 300 * bit)


def assert_clean(bus, build, *, request=True, response_=True):
    """The line held to TIMING and every edge to its byte's grid."""
    ms, bit = scale(build)
    s = check_session(bus.edges, ms, bit)
    assert s.violations == [], s.violations
    assert s.address is not None and s.address.value == ADDR
    if request:
        assert len(s.request) == 6
    if response_:
        assert s.response
    return s


# ------------------------------------------------------------- the programs
@pytest.mark.parametrize("path,thread,top", [(TESTER, 0, 0x100), (ECU, 2, 0x1D7)])
def test_kline_programs_assemble_strict_with_no_diagnostic(path, thread, top):
    """L2-DEADLINE: every pair proved at the 25-clock tick in all three
    builds; nothing unbounded; one ``.bounded`` in the tester. The tester
    spills into thread 1's quarter and the ECU into thread 3's, below its
    request buffer, so the two load together."""
    spans = []
    for build in BUILDS:
        prog = program(path, build)
        assert prog.diagnostics == []
        assert list(prog.threads) == [thread]
        report = prog.deadlines[thread]
        assert report.period == 25
        assert report.pairs and not report.infeasible and not report.unbounded
        assert report.worst_slack >= 0
        assert thread * 128 <= min(prog.words) and max(prog.words) < top
        spans.append(len(prog.words))
    assert spans[0] == {TESTER: 152, ECU: 210}[path]


# ------------------------------------------------------------- the tester
def run_tester(backend, build, ecu):
    bench, loom, bus = kline_bench(backend, tester=True, build=build)
    bus.add(ecu)
    loom.push(0, [0x0100])                     # mode 1, PID 0
    words = drain(bench, loom, 0, has_status, session_clocks(build))
    return bench, loom, bus, words


def test_kline_tester_with_a_model_ecu(backend):
    """The tester initialises a model ECU, reads mode 1 PID 0 and reports
    the keys, the ten response bytes and status good; the whole line is
    inside TIMING."""
    ms, bit = scale("rtl")
    ecu = KLineEcu(TABLE, ms=ms, bit=bit)
    bench, loom, bus, words = run_tester(backend, "rtl", ecu)
    want = response(SRC, PID00)
    assert words == list(KEYS) + want + [status(OK, len(want))], [hex(w) for w in words]
    assert ecu.events == [("session",), ("request", [0x68, 0x6A, 0xF1, 1, 0, 0xC4], True)]
    assert_clean(bus, "rtl")
    assert loom.badop() == 0


@model_only("the real 10400 bit/s bytes: 2.4 M clocks; the RTL build proves the logic")
def test_kline_tester_at_the_real_bit_rate(backend):
    """The same at 10400 bit/s with every W and P time a hundredth, and
    0x94 0x94 keys (the other ISO 9141-2 pair)."""
    ms, bit = scale("model")
    ecu = KLineEcu(TABLE, ms=ms, bit=bit, keys=(0x94, 0x94), src=0x11)
    bench, loom, bus, words = run_tester(backend, "model", ecu)
    want = response(0x11, PID00)
    assert words == [0x94, 0x94] + want + [status(OK, len(want))]
    assert_clean(bus, "model")


@model_only("covered on the RTL by the ECU's bad-checksum scenario's twin")
def test_kline_tester_reports_a_bad_response_checksum(backend):
    ms, bit = scale("rtl")
    ecu = KLineEcu(TABLE, ms=ms, bit=bit, spoil_checksum=True)
    bench, loom, bus, words = run_tester(backend, "rtl", ecu)
    want = response(SRC, PID00)
    want[-1] = (want[-1] + 1) & 0xFF
    assert words == list(KEYS) + want + [status(CKERR, len(want))]
    assert_clean(bus, "rtl")


@model_only("a time-out path; 60 k clocks of silence")
def test_kline_tester_reports_no_sync_without_an_ecu(backend):
    bench, loom, bus = kline_bench(backend, tester=True, build="rtl")
    loom.push(0, [0x0100])
    words = drain(bench, loom, 0, has_status, session_clocks("rtl"))
    assert words == [status(NO_SYNC)]
    s = check_session(bus.edges, *scale("rtl"))
    assert s.violations == [] and s.address.value == ADDR and not s.init


@model_only("a time-out path, beside the wrong-key scenario on the RTL")
def test_kline_tester_reports_no_response(backend):
    """A model ECU whose table lacks the PID ignores the request: status
    4, the keys reported, no response byte."""
    ms, bit = scale("rtl")
    ecu = KLineEcu({}, ms=ms, bit=bit)
    bench, loom, bus, words = run_tester(backend, "rtl", ecu)
    assert words == list(KEYS) + [status(NO_RESP)]
    assert ecu.events[-1] == ("request", [0x68, 0x6A, 0xF1, 1, 0, 0xC4], False)
    assert_clean(bus, "rtl", response_=False)


# ---------------------------------------------------------------- the ECU
def ecu_session(backend, build, tester, n_reports):
    bench, loom, bus = kline_bench(backend, ecu=True, build=build)
    bus.add(tester)
    reports = drain(bench, loom, 2, lambda w: len(w) >= n_reports or tester.done,
                    session_clocks(build))
    bench.run_until(lambda: tester.done, session_clocks(build), every=1000)
    reports += loom.pop_available(2)
    return bench, loom, bus, reports


def test_kline_ecu_with_a_model_tester(backend):
    """A model tester initialises the ECU and reads mode 1 PID 0: the keys
    are 0x08 0x08, the answer is the table's, and the ECU reports the
    session and the answered six-byte request."""
    ms, bit = scale("rtl")
    tester = KLineTester(1, 0, ms=ms, bit=bit)
    bench, loom, bus, reports = ecu_session(backend, "rtl", tester, 2)
    assert tester.result.status == "ok", tester.result
    assert tester.result.keys == list(KEYS)
    assert tester.result.response == response(SRC, PID00)
    assert reports == [R_OPEN, R_REQ | ANSWERED | 6], [hex(w) for w in reports]
    assert_clean(bus, "rtl")
    assert loom.badop() == 0


def test_kline_ecu_refuses_a_wrong_inverted_key(backend):
    """The tester sends 0x55 instead of ~KB2 = 0xF7: no inverted address,
    no session, report 0x2055."""
    ms, bit = scale("rtl")
    tester = KLineTester(1, 0, ms=ms, bit=bit, inverted_key=0x55)
    bench, loom, bus, reports = ecu_session(backend, "rtl", tester, 1)
    assert tester.result.status == "address" and tester.result.inverted_address is None
    assert reports == [R_BADKEY | 0x55]
    s = check_session(bus.edges, ms, bit)
    assert s.violations == []
    assert [b.value for b in s.init] == [0x55, 0x08, 0x08, 0x55]


def test_kline_ecu_ignores_a_bad_request_checksum(backend):
    """A request whose checksum is one off is not answered: the tester's
    P2 runs out, and the ECU reports a checksum error on six bytes."""
    ms, bit = scale("rtl")
    tester = KLineTester(1, 0, ms=ms, bit=bit, spoil_checksum=True)
    bench, loom, bus, reports = ecu_session(backend, "rtl", tester, 2)
    assert tester.result.status == "response"
    assert reports == [R_OPEN, R_REQ | F_CK | 6]
    assert_clean(bus, "rtl", response_=False)


@model_only("table look-up paths; the answered path runs on the RTL")
def test_kline_ecu_answers_the_second_entry_and_ignores_unknown_pids(backend):
    ms, bit = scale("rtl")
    tester = KLineTester(1, 0x0C, ms=ms, bit=bit)
    bench, loom, bus, reports = ecu_session(backend, "rtl", tester, 2)
    assert tester.result.response == response(SRC, TABLE[(1, 0x0C)])
    assert reports == [R_OPEN, R_REQ | ANSWERED | 6]
    tester = KLineTester(1, 0x0D, ms=ms, bit=bit)
    bench, loom, bus, reports = ecu_session(backend, "rtl", tester, 2)
    assert tester.result.status == "response"
    assert reports == [R_OPEN, R_REQ | F_TAB | 6]


@model_only("a 5-baud reject path: 60 k clocks")
def test_kline_ecu_ignores_another_address(backend):
    ms, bit = scale("rtl")
    tester = KLineTester(1, 0, ms=ms, bit=bit, address=0x34)
    bench, loom, bus, reports = ecu_session(backend, "rtl", tester, 1)
    assert tester.result.status == "sync" and reports == []
    assert len(check_session(bus.edges, ms, bit).init) == 0


@model_only("the real 10400 bit/s bytes: 2.4 M clocks; the RTL build proves the logic")
def test_kline_ecu_at_the_real_bit_rate(backend):
    ms, bit = scale("model")
    tester = KLineTester(1, 0, ms=ms, bit=bit)
    bench, loom, bus, reports = ecu_session(backend, "model", tester, 2)
    assert tester.result.status == "ok"
    assert tester.result.response == response(SRC, PID00)
    assert reports == [R_OPEN, R_REQ | ANSWERED | 6]
    assert_clean(bus, "model")


# -------------------------------------------------------------- together
def together(backend, build):
    bench, loom, bus = kline_bench(backend, tester=True, ecu=True, build=build)
    loom.push(0, [0x0100])
    words = drain(bench, loom, 0, has_status, session_clocks(build))
    bench.step(200)
    return loom, bus, words, loom.pop_available(2)


def test_kline_tester_and_ecu_together(backend):
    """The PHY-GL-SDF candidate: both programs on one line, a whole
    session from the 5-baud address to the checked response."""
    loom, bus, words, reports = together(backend, "rtl")
    want = response(SRC, PID00)
    assert words == list(KEYS) + want + [status(OK, len(want))], [hex(w) for w in words]
    assert reports == [R_OPEN, R_REQ | ANSWERED | 6]
    s = assert_clean(bus, "rtl")
    assert [b.value for b in s.request] == [0x68, 0x6A, 0xF1, 1, 0, 0xC4]
    assert loom.badop() == 0


@model_only("real time: 2.4 s, about 120 M clocks, about 2 hours on the model")
@pytest.mark.skipif(not os.environ.get("LOOM_KLINE_REAL_TIME"),
                    reason="set LOOM_KLINE_REAL_TIME=1: about 2 hours")
def test_kline_together_in_real_time(backend):
    """The real build: a 2 s address at 5 baud, 1 ms ticks, 10400 bit/s."""
    loom, bus, words, reports = together(backend, "real")
    want = response(SRC, PID00)
    assert words == list(KEYS) + want + [status(OK, len(want))]
    assert reports == [R_OPEN, R_REQ | ANSWERED | 6]
    s = assert_clean(bus, "real")
    print({k: [round(v, 3) for v in vs] for k, vs in s.times.items()})
