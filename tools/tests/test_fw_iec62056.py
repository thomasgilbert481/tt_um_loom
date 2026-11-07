"""L3-IEC62056 on the model and the RTL: firmware/iec62056_client.loom and iec62056_meter.loom.

IEC 62056-21 protocol mode C, data readout, at logic level. The client
(thread 0: TX = OUT0, RX = IN0) sends the request at 300 baud, takes the
rate the meter's identification offers, acknowledges, switches its own
divider and reads the data message; the meter (thread 1: TX = OUT1, RX =
IN1) answers with the identification and the data block the host gives.
The reference models are ``tools.protomodels.iec62056.Meter`` and
``Client``; ``check_session`` decodes both lines and checks them.

Expected values come from IEC 62056-21:2002 through the reference model
(the 7E1 frame of 5.4, the messages of 6.3, the baud table of 6.3.14 item
13, the switch rule of 6.4.3.2, the times tr, tt and ta of 6.4.3.6, the BCC
of 6.2) and from the program headers (host formats, the status words),
never from what the model or the RTL did.

Clocks. Every scenario keeps the standard's rates and milliseconds and
runs the chip on a slower clock instead: 230.4 kHz, where a 300-baud bit is
768 clocks (a tick of 96, eight a bit) and a 1 200-baud bit 192 (a tick of
24, the fastest the programs are proved for). So no rate is scaled; the
clock is. One model scenario runs a fractional divider at 1 MHz and 4 800
baud. 19 200 baud would need a 3.69 MHz clock for the same tick, about 3.3
million clocks a session, and is not simulated: its divider is the same
shift as 4 800 baud's, two places further.

Every body that takes a ``backend`` (``tools/tests/fw_backend.py``) runs
on the golden model under pytest and on the RTL under ``test/test_fw.py``,
unless it is marked ``model_only``.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.iec62056 import (ACK, BAUD, ETX, STX, TR_MAX_MS,
                                        TR_MIN_FAST_MS, TR_MIN_MS, TT_MAX_MS,
                                        TT_MIN_MS, Client, Link, Meter, Wire,
                                        bcc, check_session, data_message,
                                        identification, negotiated_baud)
from tools.tests.fw_backend import model_only

ISA = load()
CLIENT = REPO / "firmware" / "iec62056_client.loom"
METER = REPO / "firmware" / "iec62056_meter.loom"
IMEM = 512
SLOW = 230_400

# The client's status word (program header).
STATUS = 1 << 15
PARE, FRAME, TMO, BCCE, FMT, OVR = 1, 2, 4, 8, 16, 32
# The meter's status word (program header).
M_BADACK, M_NOACK = 1, 2

# Short messages, to keep the RTL time down: a lower-case third letter
# (6.3.14 item 12) lets the meter answer after 20 ms.
MFR, IDENT, BLOCK = "LGz", "1", "0(1)\r\n"


def divider(clock, baud=300):
    """TICK_INT, TICK_FRAC for eight ticks a bit (program headers)."""
    period = round(clock / (8 * baud) * 256)
    return period >> 8, period & 0xFF


def iec_bench(backend):
    """FIFOs, the bit engine, LD/ST and the 512-word memory on the model;
    the RTL has all of them and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "DMEM"), imem_words=IMEM)
    return backend.bench()


def start(loom, words, clock, threads):
    loom.load(words)
    ti, tf = divider(clock)
    for t in threads:
        loom.write_csr(t, "TICK_INT", ti)
        loom.write_csr(t, "TICK_FRAC", tf)
    loom.run(list(threads))


def image(*paths):
    words = {}
    for path in paths:
        prog = assemble_file(path, isa=ISA, strict=True)
        assert not set(words) & set(prog.words)
        words.update(prog.words)
    return words


def bit(clock, baud=300):
    return clock / baud


def meter_words(mfr=MFR, z="2", ident=IDENT, block=BLOCK):
    """The meter's set-up (program header): what follows the '/', then the
    data block, each ended by a 0 word."""
    return [ord(c) for c in mfr + z + ident] + [0] + [ord(c) for c in block] + [0]


def client_read(bench, loom, clock, max_z=6, limit_ms=4000):
    """The host asks the chip client for a readout and collects OUTQ[0] up
    to the status word: ``(characters, status)``."""
    loom.push(0, [max_z])
    out = []
    end = bench.cycle + int(limit_ms * clock / 1000)
    step = int(bit(clock, 1200) * 2)
    while bench.cycle < end:
        for w in loom.pop_available(0):
            if w & STATUS:
                return "".join(chr(c) for c in out), w
            out.append(w)
        bench.step(step)
    raise AssertionError("no status word in %d ms: %r" % (limit_ms, out))


def wait_for(bench, loom, cond, clock, limit_ms, thread=1):
    """Step until ``cond()``, popping OUTQ[thread]; returns what was popped."""
    out = []
    end = bench.cycle + int(limit_ms * clock / 1000)
    step = int(bit(clock, 1200) * 2)
    while bench.cycle < end and not cond():
        out += loom.pop_available(thread)
        bench.step(step)
    assert cond(), "not done in %d ms" % limit_ms
    bench.step(step)
    return out + loom.pop_available(thread)


def assert_clean(ses, baud):
    assert ses.violations == [], (ses.violations, ses)
    assert ses.baud == baud, ses
    assert ses.bcc_ok, ses


# ------------------------------------------------------------------- build
#: program -> (proved tick, words in its thread's quarter, all words) (headers)
BUILD = {CLIENT: (24, 127, 174), METER: (24, 123, 226)}


@pytest.mark.parametrize("path", sorted(BUILD), ids=lambda p: p.stem)
def test_iec62056_programs_assemble_with_no_diagnostic(path):
    """--strict and no diagnostic: every deadline pair proved at the tick
    the header gives, nothing unbounded but the declared OUTQ pushes, and
    the word counts the headers give."""
    prog = assemble_file(path, isa=ISA, strict=True)
    tick, main, total = BUILD[path]
    assert prog.diagnostics == []
    (thread,) = prog.threads
    report = prog.deadlines[thread]
    assert report.period == tick
    assert report.pairs and not report.unbounded and not report.infeasible
    assert report.worst_slack >= 0
    entry = prog.threads[thread].entry
    assert entry == thread * 128
    assert sum(1 for a in prog.words if entry <= a < entry + 128) == main
    assert len(prog.words) == total


def test_iec62056_programs_fit_together():
    """Client and meter in one 512-word image without overlap (threads 2
    and 3 are not started: their quarters hold the spill and the data)."""
    words = image(CLIENT, METER)
    assert len(words) == sum(t for _, _, t in BUILD.values())
    assert max(words) < IMEM


def test_iec62056_model_messages():
    """The reference model's messages against the standard's formats (6.3)
    and a BCC worked by hand: STX "0(1)" CR LF "!" CR LF ETX; the XOR of
    '0' '(' '1' ')' CR LF '!' CR LF ETX is 0x22."""
    assert identification("LGZ", "5", "ZMD") == "/LGZ5ZMD\r\n"
    msg = data_message("0(1)\r\n")
    assert msg[0] == chr(STX) and msg[-2] == chr(ETX)
    assert ord(msg[-1]) == 0x22 == bcc(msg[1:-1])
    assert negotiated_baud("4", chr(ACK) + "040\r\n") == BAUD[4] == 4800
    assert negotiated_baud("4", chr(ACK) + "030\r\n") == 300     # Zs differ
    assert negotiated_baud("4", chr(ACK) + "041\r\n") == 300     # not readout
    assert negotiated_baud("4", None) == 300                     # no ack


# ------------------------------------------------------------------ client
def client_setup(backend, clock=SLOW, **meter):
    """The chip client with a model meter facing it; returns
    ``(bench, loom, meter, client line, meter line)``."""
    bench = iec_bench(backend)
    dev = bench.add(Meter("IN0", "OUT0", clock, **{"mfr": MFR, "ident": IDENT,
                                                   "block": BLOCK, **meter}))
    tx, rx = bench.add(Wire("OUT0")), bench.add(Wire("IN0"))
    loom = backend.loom(bench, isa=ISA)
    start(loom, image(CLIENT), clock, (0,))
    bench.step(64)
    return bench, loom, dev, tx, rx


def test_iec62056_client_reads_model_meter(backend):
    """The meter offers '2' (1 200 baud): the client acknowledges 2, both
    switch, the data message comes in at 1 200 baud with a good BCC. On the
    lines: every reaction time inside 6.4.3.6, the client's ack 66 bits
    (220 ms) after the identification, the data message at the rate
    6.4.3.2 gives, every edge on its bit grid."""
    bench, loom, meter, tx, rx = client_setup(backend)
    chars, status = client_read(bench, loom, SLOW)
    ident = identification(MFR, "2", IDENT)
    assert chars == ident + BLOCK + "!\r\n", repr(chars)
    assert status == STATUS | 2 << 8, hex(status)
    ses = check_session(tx.edges, rx.edges, SLOW)
    assert_clean(ses, 1200)
    assert ses.request == "/?!\r\n" and ses.ack == chr(ACK) + "020\r\n"
    assert 219.9 <= ses.tr_ack_ms <= 221.0, ses.tr_ack_ms           # 66 bits
    assert ses.edge_err <= 0.06, ses.edge_err
    assert meter.rates == [1200]


@model_only("the BCC compare is the RTL scenario's code path with the other result; RTL time")
def test_iec62056_client_rejects_a_corrupted_bcc(backend):
    """A meter whose BCC has one bit wrong: the client passes the block on
    and sets the BCC bit (6.2: the HHU may repeat the readout)."""
    bench, loom, meter, tx, rx = client_setup(backend, bad_bcc=True)
    chars, status = client_read(bench, loom, SLOW)
    assert chars.endswith(BLOCK + "!\r\n")
    assert status == STATUS | 2 << 8 | BCCE, hex(status)
    ses = check_session(tx.edges, rx.edges, SLOW)
    assert ses.violations == [] and not ses.bcc_ok


@model_only("the 300-baud data message alone is 92,000 clocks; the RTL writes the Z = 0 divider at every session's start")
def test_iec62056_client_with_a_meter_that_offers_only_300(backend):
    """Z = '0': the client acknowledges 0 and stays at 300 baud; the data
    message comes at 300 (6.4.3.2, first case)."""
    bench, loom, meter, tx, rx = client_setup(backend, z="0")
    chars, status = client_read(bench, loom, SLOW)
    assert status == STATUS, hex(status)
    ses = check_session(tx.edges, rx.edges, SLOW)
    assert_clean(ses, 300)
    assert ses.ack == chr(ACK) + "000\r\n"


@model_only("one more choice case; the RTL runs the taken rate")
@pytest.mark.parametrize("z, max_z", [("3", 2), ("8", 6)])
def test_iec62056_client_refuses_a_rate(backend, z, max_z):
    """A rate over the host's limit, or a reserved Z ('7' .. '9'): the ack
    asks for '0' and the session stays at 300 baud (Figure 15's
    rejection); the status word says Z = 0."""
    bench, loom, meter, tx, rx = client_setup(backend, z=z)
    chars, status = client_read(bench, loom, SLOW, max_z=max_z)
    assert status == STATUS, hex(status)
    ses = check_session(tx.edges, rx.edges, SLOW)
    assert ses.violations == [] and ses.baud == 300 and ses.bcc_ok, ses
    assert ses.ack == chr(ACK) + "000\r\n"
    assert meter.rates == [300]


@model_only("1.6 s of silence, 370,000 clocks")
def test_iec62056_client_times_out_on_a_silent_meter(backend):
    """No identification: the timeout bit after 15 x 255 ticks (1 594 ms),
    which is more than tr's 1 500 ms (6.4.3.6)."""
    bench, loom, meter, tx, rx = client_setup(backend, answers=False)
    chars, status = client_read(bench, loom, SLOW, limit_ms=2500)
    assert chars == "" and status == STATUS | TMO, hex(status)
    req_end = tx.edges[-1][0] + bit(SLOW)          # the <LF>'s stop bit
    waited = (bench.cycle - req_end) * 1000 / SLOW
    assert TR_MAX_MS < waited < 1594 + 10, waited


# ------------------------------------------------------------------- meter
def meter_setup(backend, clock=SLOW, words=None, **client):
    """The chip meter (thread 1) with a model client facing it; returns
    ``(bench, loom, client, client line, meter line)``."""
    bench = iec_bench(backend)
    hhu = bench.add(Client("IN1", "OUT1", clock, **client))
    tx, rx = bench.add(Wire("IN1")), bench.add(Wire("OUT1"))
    loom = backend.loom(bench, isa=ISA)
    start(loom, image(METER), clock, (1,))
    loom.push(1, words or meter_words())
    bench.step(int(bit(clock)))         # the set-up runs (and the line idles)
    return bench, loom, hhu, tx, rx


def meter_read(bench, loom, hhu, clock=SLOW, limit_ms=3500):
    """A model readout of the chip meter: ``(Readout, status words)``."""
    hhu.read()
    n = len(hhu.results)
    out = wait_for(bench, loom, lambda: len(hhu.results) > n, clock, limit_ms)
    return hhu.results[-1], out


def test_iec62056_meter_answers_model_client(backend):
    """The model client reads the chip meter: identification "/LGz21",
    ack 2, the data message at 1 200 baud with the BCC the model computes;
    the meter's reaction times at least 20 ms (lower-case third letter) and
    at most 1 500 ms; its status word: rate 2, the ack taken."""
    bench, loom, hhu, tx, rx = meter_setup(backend)
    res, words = meter_read(bench, loom, hhu)
    assert res.ident == identification(MFR, "2", IDENT)
    assert res.block == BLOCK + "!\r\n" and res.bcc_ok and res.chars_ok, res
    assert res.baud == 1200
    assert words == [STATUS | 2 << 8], [hex(w) for w in words]
    ses = check_session(tx.edges, rx.edges, SLOW)
    assert_clean(ses, 1200)
    assert TR_MIN_FAST_MS <= ses.tr_ident_ms <= 27.0, ses.tr_ident_ms     # 7.5 bits
    assert TR_MIN_FAST_MS <= ses.tr_data_ms <= 25.0, ses.tr_data_ms
    assert ses.edge_err <= 0.06, ses.edge_err


@model_only("200 ms reaction times, 90,000 more clocks; the RTL runs the 20 ms ones")
def test_iec62056_meter_upper_case_waits_200_ms(backend):
    """Third letter upper case: tr >= 200 ms for both answers (6.4.3.6);
    the meter's TR is 66.5 bits, 221.7 ms, before the identification."""
    words = meter_words(mfr="LGZ", z="3")
    bench, loom, hhu, tx, rx = meter_setup(backend, words=words)
    res, out = meter_read(bench, loom, hhu)
    assert res.baud == 2400 and res.bcc_ok and out == [STATUS | 3 << 8]
    ses = check_session(tx.edges, rx.edges, SLOW)
    assert_clean(ses, 2400)
    assert TR_MIN_MS <= ses.tr_ident_ms <= 223.0, ses.tr_ident_ms
    assert TR_MIN_MS <= ses.tr_data_ms <= 223.0, ses.tr_data_ms


@model_only("1.8 s of line time; the fallback is a branch of the RTL's ack code")
def test_iec62056_meter_falls_back_without_ack(backend):
    """No acknowledgement: after tt (1 500 < tt <= 2 200 ms) the data
    message at 300 baud (6.4.3.2 last case, Figure 13); status: no ack."""
    bench, loom, hhu, tx, rx = meter_setup(backend, acks=False)
    res, out = meter_read(bench, loom, hhu, limit_ms=3000)
    assert res.baud == 300 and res.bcc_ok and res.block == BLOCK + "!\r\n"
    assert out == [STATUS | M_NOACK], [hex(w) for w in out]
    ses = check_session(tx.edges, rx.edges, SLOW)
    assert ses.ack is None and ses.violations == [], ses
    assert TT_MIN_MS < ses.tr_data_ms <= TT_MAX_MS, ses.tr_data_ms


@model_only("one more ack case; the RTL runs the ack taken")
@pytest.mark.parametrize("ack_z", ["1", "0"])
def test_iec62056_meter_keeps_300_for_another_z(backend, ack_z):
    """An ack with a Z that is not the one offered ('1' for '2'), or '0':
    the data message at 300 baud (6.4.3.2); a Z other than '0' and its own
    is also reported as a bad ack."""
    bench, loom, hhu, tx, rx = meter_setup(backend, z_override=ack_z)
    res, out = meter_read(bench, loom, hhu)
    assert res.baud == 300 and res.bcc_ok
    assert out == [STATUS | M_BADACK], [hex(w) for w in out]
    ses = check_session(tx.edges, rx.edges, SLOW)
    assert_clean(ses, 300)


# ---------------------------------------------------------------- together
def together(backend, clock=SLOW, words=None):
    """Client on thread 0 and meter on thread 1 of one chip, facing each
    other through ``Link`` (OUT0 -> IN1, OUT1 -> IN0)."""
    bench = iec_bench(backend)
    link = bench.add(Link("OUT0", "IN0", "OUT1", "IN1"))
    loom = backend.loom(bench, isa=ISA)
    start(loom, image(CLIENT, METER), clock, (0, 1))
    loom.push(1, words or meter_words())
    bench.step(int(bit(clock)))
    return bench, loom, link


def together_read(bench, loom, link, clock, max_z=6, limit_ms=4000):
    chars, status = client_read(bench, loom, clock, max_z=max_z, limit_ms=limit_ms)
    bench.step(int(bit(clock, 1200)))
    return chars, status, loom.pop_available(1), check_session(link.a.edges, link.b.edges, clock)


def test_iec62056_client_and_meter_together(backend):
    """Both programs on one chip: a readout at 1 200 baud with no
    violation, the same data on both status words, and a second readout
    after it (both are back at 300 baud)."""
    bench, loom, link = together(backend)
    chars, status, mwords, ses = together_read(bench, loom, link, SLOW)
    assert chars == identification(MFR, "2", IDENT) + BLOCK + "!\r\n", repr(chars)
    assert status == STATUS | 2 << 8 and mwords == [STATUS | 2 << 8]
    assert_clean(ses, 1200)
    if backend.name == "model":         # the second session: model only (RTL time)
        since = bench.cycle
        chars, status, mwords, _ = together_read(bench, loom, link, SLOW)
        assert status == STATUS | 2 << 8 and mwords == [STATUS | 2 << 8]
        ses = check_session(link.a.edges, link.b.edges, SLOW, since=since)
        assert_clean(ses, 1200)


@model_only("1 MHz: about 930,000 clocks")
def test_iec62056_together_on_a_fractional_divider(backend):
    """At 1 MHz the 300-baud tick is 416.67 clocks (TICK_INT 416, TICK_FRAC
    171); Z = 4 shifts it to 26.04 clocks, 4 800 baud. Upper case, so both
    sides keep 200 ms. Every edge on its grid at both rates."""
    clock = 1_000_000
    words = meter_words(mfr="ABC", z="4", block="1.8.0(42.0*kWh)\r\n")
    bench, loom, link = together(backend, clock, words)
    chars, status, mwords, ses = together_read(bench, loom, link, clock)
    assert status == STATUS | 4 << 8 and mwords == [STATUS | 4 << 8], hex(status)
    assert chars.endswith("1.8.0(42.0*kWh)\r\n!\r\n")
    assert_clean(ses, 4800)
    assert ses.edge_err <= 0.02, ses.edge_err


# ----------------------------------------------------------------- checker
def test_iec62056_checker_sees_a_fast_reaction():
    """A check of the checker, models only: an upper-case meter answering
    after 50 ms breaks tr >= 200 ms (6.4.3.6); the same meter with a lower
    case third letter does not."""
    for mfr, bad in (("LGZ", True), ("LGz", False)):
        meter = Meter(("ui", 1), ("ui", 0), SLOW, mfr=mfr, tr_ms=50.0, block=BLOCK)
        hhu = Client(("ui", 0), ("ui", 1), SLOW)
        lines = _wire_models(hhu, meter, 400_000, lambda: hhu.results)
        ses = check_session(*lines, SLOW)
        assert any("identification after" in v for v in ses.violations) == bad, ses


def _wire_models(a, b, cycles, done):
    """Step two pin models facing each other on raw pads, no chip."""
    from tools.protomodels.bench import Drive, Lines
    wa, wb = Wire(a.tx_pad), Wire(b.tx_pad)
    d, lines = Drive(), Lines()
    a.read(start=100)
    for cyc in range(cycles):
        d.clear()
        a.drive(d, cyc)
        b.drive(d, cyc)
        lines.cycle, lines.ui = cyc, d.ui_val
        for m in (a, b, wa, wb):
            m.observe(lines)
        if done():
            break
    return wa.edges, wb.edges
