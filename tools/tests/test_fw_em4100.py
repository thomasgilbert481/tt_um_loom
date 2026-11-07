"""L3 for firmware/em4100_tag.loom and firmware/em4100_reader.loom (W-063).

The EM4100 is a read-only 125 kHz RFID tag: it counts the reader's field
and sends a 64-bit frame (nine 1s, ten rows of four data bits and an even
parity, four even column parities, a stop 0), Manchester at RF/64 or RF/32,
over and over. ``tools.protomodels.em4100`` is written from the data sheet:
the frame and its checks, a model reader that drives a field and reads MOD
back on the field's own time base, a model tag clocked by the field, and a
wire.

Scenarios: the chip's tag read by the model reader; the model tag read by
the chip's reader; both programs on one chip through two wires; frames with
a parity or stop-bit error rejected; the tag under a field 2 % off nominal.

Expected values come from the data sheet (the frame, Fig. 6's code, N field
periods per half-bit) and from SEMANTICS, never from what the model or the
RTL did. The tag's timing requirement is the protocol's: every MOD edge on
a half-bit boundary counted in field cycles (``Decoded.offsets`` all 0).
Its lag behind the field edge, 12 to 15 clocks, is SEMANTICS 2 and 3
arithmetic (the tag program's header). The reader's field is checked edge
by edge: SEMANTICS 6.10 puts each latched edge on its tick, so every half
period is exactly P clocks.

Rates: the field is 125 kHz (400 clocks a period at 50 MHz) in the model-only
scenarios; the RTL scenarios run it at 32 clocks a period, 1.5625 MHz, so
that a frame is 131,072 clocks at RF/64 and 65,536 at RF/32 and the whole
module stays near the brief's RTL budget.

Every scenario's name starts with ``test_em4100_``: test/test_fw.py loads
every module together and refuses a duplicate name.
"""

from bisect import bisect_right

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.em4100 import (FRAME_BITS, HALF, EdgeLog, ModelReader,
                                      TagModel, Wire, check_frame, decode,
                                      frame_bits, halves)
from tools.tests.fw_backend import model_only

ISA = load()
TAG = REPO / "firmware" / "em4100_tag.loom"
READER = REPO / "firmware" / "em4100_reader.loom"
TAG_THREAD = 2
# pins: the reader's RF_CLK and DEMOD, the tag's RF and MOD
RF_CLK, DEMOD, TAG_RF, TAG_MOD = "OUT0", "IN0", "IN1", "OUT1"
REAL = 400                      # clocks per field period at 125 kHz, 50 MHz
FAST = 32                       # clocks per field period, model field, RTL
READER_P = 20                   # the chip reader's half period in the RTL runs
LAG = (12, 15)                  # field rise to MOD edge, clocks (tag header)
BUILD = 6_000                   # clocks the tag may take to build its frame
IDENT = 0x0102ABCDEF            # D00..D13 = 0x01, then 0x02ABCDEF
ROW, COLUMN, STOP, VIOL = 1, 2, 4, 8    # reader status bits (program header)


def em_bench(backend):
    """The latch (``SETPD``), slice A (``BE``, ``BEENC``), the FIFOs and the
    512-word memory; the
    RTL has all of them and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "BEENC"), imem_words=512)
    return backend.bench()


def host_words(ident, rate=64):
    """The tag's three INQ words (em4100_tag.loom header)."""
    return [((ident >> 32) & 0xFF) | ((rate == 32) << 8),
            (ident >> 16) & 0xFFFF, ident & 0xFFFF]


def id_words(ident):
    """The reader's three ID words after status 0."""
    return [(ident >> 32) & 0xFF, (ident >> 16) & 0xFFFF, ident & 0xFFFF]


def frame_clocks(rate, period, bits=FRAME_BITS):
    return int(bits * rate * period)


def load(backend, bench, *programs):
    loom = backend.loom(bench, isa=ISA)
    words = {}
    for path in programs:
        words.update(assemble_file(path, isa=ISA, strict=True).words)
    loom.load(words)
    return loom


def start_tag(loom, ident, rate):
    loom.push(TAG_THREAD, host_words(ident, rate))
    loom.run([TAG_THREAD])


def start_reader(loom, half_period, rate):
    loom.write_csr(0, "TICK_INT", half_period)
    loom.write_csr(1, "TICK_INT", HALF[rate] * half_period)
    loom.run([0, 1])


def wait_report(bench, loom, budget, every=4096):
    """Step until OUTQ[1] holds a whole report, then pop it:
    ``(status, id words or [])``."""
    spent = 0
    while loom.fifo_status(1)["outq"] == 0:
        assert spent < budget, "no report from the reader in %d clocks" % budget
        bench.step(every)
        spent += every
    status = loom.pop(1, 1)[0]
    return status, (loom.pop(1, 3) if status == 0 else [])


def held_to_the_field(d, frames=1):
    """The protocol's timing: every MOD edge on the half-bit grid counted in
    field cycles, each 12 to 15 clocks after its field rise, no Manchester
    violation, and at least ``frames`` whole frames read."""
    assert d.offsets and set(d.offsets) == {0}, sorted(set(d.offsets))
    assert LAG[0] <= min(d.lags) and max(d.lags) <= LAG[1], (min(d.lags), max(d.lags))
    assert not d.violations, d.violations[:8]
    assert len(d.frames) >= frames, (len(d.bits), d.frames)


# ------------------------------------------------------------- the programs
def test_em4100_programs_assemble_strict_with_no_diagnostic():
    """L2-DEADLINE. The tag has no deadline pair at all (it keeps no time of
    its own); the reader's field thread is proved at P = 20 and its decoder
    at Q = 256, with nothing unbounded and no ``.bounded`` declaration. The
    tag sits in thread 2's quarter and the reader in threads 0 and 1, so
    the two load together."""
    tag = assemble_file(TAG, isa=ISA, strict=True)
    reader = assemble_file(READER, isa=ISA, strict=True)
    assert tag.diagnostics == [] and reader.diagnostics == []
    assert sorted(tag.threads) == [TAG_THREAD] and not tag.deadlines[TAG_THREAD].pairs
    assert all(0x100 <= a < 0x180 for a in tag.words)
    assert sorted(reader.threads) == [0, 1]
    assert max(reader.words) < 0x100            # threads 0 and 1 only
    assert reader.threads[1].entry == 0x080     # thread 1 at its reset vector
    for thread, period in ((0, READER_P), (1, 256)):
        rep = reader.deadlines[thread]
        assert rep.period == period and rep.pairs
        assert not rep.infeasible and not rep.unbounded and not rep.declarations
        assert rep.worst_slack >= 0
    assert not set(tag.words) & set(reader.words)


def test_em4100_model_frame_matches_the_data_sheet():
    """The model's frame for 0x0102ABCDEF, written out by hand from the p. 5
    table: nine 1s, rows D?0..D?3 P?, columns, S0; and Fig. 6's levels."""
    rows = "00000 00011 00000 00101 10100 10111 11000 11011 11101 11110"
    want = "111111111" + rows.replace(" ", "") + "0010" + "0"
    assert "".join(map(str, frame_bits(IDENT))) == want
    assert check_frame(frame_bits(IDENT)).ident == IDENT
    assert halves([1, 0]) == [0, 1, 1, 0]
    assert check_frame(frame_bits(IDENT, 9)).bad_rows == [0]
    assert not check_frame(frame_bits(IDENT, 60)).column_ok
    assert not check_frame(frame_bits(IDENT, 63)).stop_ok


# ----------------------------------------------------- chip tag, model reader
def tag_run(backend, ident, rate, period, frames=1, keep=False):
    """The chip's tag under a model reader's field of ``period`` clocks;
    returns the model reader's decode after ``frames`` whole frames (and
    the model reader, with ``keep``)."""
    bench = em_bench(backend)
    reader = bench.add(ModelReader(TAG_RF, TAG_MOD, period, rate=rate))
    loom = load(backend, bench, TAG)
    start_tag(loom, ident, rate)
    bench.step(BUILD + frame_clocks(rate, period, FRAME_BITS * frames + 2))
    return (reader.decoded(), reader) if keep else reader.decoded()


def tag_checks(d, ident):
    held_to_the_field(d)
    assert [check.ident for _, check in d.frames] == [ident] * len(d.frames)
    starts = [i for i, _ in d.frames]
    assert all(b - a == FRAME_BITS for a, b in zip(starts, starts[1:]))


@pytest.mark.parametrize("rate,ident", [(64, 0x0102ABCDEF), (32, 0xFFFFFFFFFF)])
def test_em4100_tag_to_model_reader(backend, rate, ident):
    """The chip's tag sends ``ident`` under a fast field (32 clocks): the
    model reader finds whole frames, back to back, with the right ID, and
    every MOD edge on the field's half-bit grid."""
    tag_checks(tag_run(backend, ident, rate, FAST), ident)


@model_only("125 kHz: 1.6 M clocks a frame at RF/64, about four minutes in Icarus")
@pytest.mark.parametrize("rate,ident", [(64, 0x0000000000), (64, 0x0102ABCDEF),
                                        (32, 0x5A5A0FF0C3)])
def test_em4100_tag_at_125khz(backend, rate, ident):
    """The same at the real field, 400 clocks a period; the all-0 ID is the
    run-length extreme (50 zeros between the header and the columns)."""
    tag_checks(tag_run(backend, ident, rate, REAL), ident)


@model_only("a sweep of short field periods; the RTL runs the 32-clock field")
@pytest.mark.parametrize("period", [24, 25, 26, 28])
def test_em4100_tag_shortest_field_period(backend, period):
    """The tag header's limit: a field period of 24 clocks or more is
    followed (the longest path from a field rise back to WAITP RF, 0 is
    five slots). Below it the tag would miss field cycles."""
    tag_checks(tag_run(backend, IDENT, 32, period), IDENT)


@model_only("the failure side of the sweep above")
@pytest.mark.parametrize("period", [20, 22])
def test_em4100_tag_falls_behind_below_the_limit(backend, period):
    """Below 24 clocks the limit shows: some MOD edges land a field cycle
    late (22: the WAITP RF, 0 after a SHO misses the low half), or the
    count is lost altogether (20). Measured 2026-11-07: 21 to 23 slip by
    one cycle, 20 and below break the frame."""
    d = tag_run(backend, IDENT, 32, period)
    assert set(d.offsets) != {0}


def test_em4100_tag_follows_a_field_2pc_fast(backend):
    """The field 2 % fast (RF/32, 31.36 clocks a period), the harder side
    for the tag's per-cycle budget; the slow side is model only below."""
    tag_follows(backend, -0.02)


@model_only("the slow side; the RTL runs the fast one")
def test_em4100_tag_follows_a_field_2pc_slow(backend):
    """The field 2 % slow (RF/32, 32.64 clocks a period)."""
    tag_follows(backend, +0.02)


def tag_follows(backend, off):
    """The field 2 % slow and 2 % fast (RF/32, 32.64 and 31.36 clocks): the
    tag's half-bits stay N field cycles long, so in clocks they are 2 % off
    nominal too. The tag follows the field, not a time of its own."""
    period = FAST * (1 + off)
    d, reader = tag_run(backend, IDENT, 32, period, keep=True)
    tag_checks(d, IDENT)
    edges = [c for c, _ in reader.mod.edges[1:]]     # after the startup edge
    k0, k1 = (bisect_right(reader.rises, c) - 1 for c in (edges[0], edges[-1]))
    n = HALF[32]
    clocks_per_half = (edges[-1] - edges[0]) / ((k1 - k0) / n)
    assert abs(clocks_per_half / (n * period) - 1) < 0.002, clocks_per_half
    assert abs(clocks_per_half / (n * FAST) - 1) > 0.015, clocks_per_half


# ----------------------------------------------------- model tag, chip reader
def reader_run(backend, ident, rate, half_period=READER_P, **tag_kw):
    """The chip's reader (field on OUT0, DEMOD on IN0) with the model tag;
    the tag starts at bit 48 of its frame unless told otherwise, so that the
    reader meets a header soon."""
    tag_kw.setdefault("start_bit", 48)
    bench = em_bench(backend)
    tag = bench.add(TagModel(RF_CLK, DEMOD, ident, rate=rate, **tag_kw))
    loom = load(backend, bench, READER)
    start_reader(loom, half_period, rate)
    return bench, loom, tag


def frames_budget(rate, half_period, frames):
    return frame_clocks(rate, 2 * half_period, FRAME_BITS * frames) + 20_000


def exact_field(tag, half_period):
    """SEMANTICS 6.10: every latched edge on its tick, so every half period
    of RF_CLK is exactly P clocks, high and low alike."""
    edges = [c for c, _ in tag.field.edges]
    assert len(edges) > 100
    assert {b - a for a, b in zip(edges, edges[1:])} == {half_period}


@pytest.mark.parametrize("rate,ident", [(32, IDENT)])
def test_em4100_model_tag_to_reader(backend, rate, ident):
    """The model tag sends ``ident`` on the chip's own field: the reader
    reports status 0 and the ID, and its field has no jitter."""
    bench, loom, tag = reader_run(backend, ident, rate)
    assert wait_report(bench, loom, frames_budget(rate, READER_P, 2)) == (0, id_words(ident))
    exact_field(tag, READER_P)


@model_only("RF/64 and 125 kHz: 1.6 M clocks a frame")
@pytest.mark.parametrize("rate,ident,half_period", [
    (64, 0x0000000000, READER_P), (64, 0xFFFFFFFFFF, READER_P), (64, IDENT, 200),
    (32, 0x5A5A0FF0C3, 200)])
def test_em4100_model_tag_to_reader_slow(backend, rate, ident, half_period):
    """RF/64, the run-length extremes, and the real 125 kHz field (P = 200,
    400 clocks a period: 125.000 kHz at 50 MHz)."""
    bench, loom, tag = reader_run(backend, ident, rate, half_period)
    budget = frames_budget(rate, half_period, 2)
    assert wait_report(bench, loom, budget, every=16384) == (0, id_words(ident))
    exact_field(tag, half_period)


@model_only("two frames from power-up at RF/64; the RTL starts the tag at bit 48")
def test_em4100_model_tag_from_the_first_bit(backend):
    """The tag starts at its first bit as the field comes up: the reader
    still finds the phase and the header (in the second frame at worst)."""
    bench, loom, tag = reader_run(backend, IDENT, 64, start_bit=0)
    assert wait_report(bench, loom, frames_budget(64, READER_P, 3)) == (0, id_words(IDENT))


# --------------------------------------------------------- both on one chip
@pytest.mark.parametrize("rate", [32])
def test_em4100_tag_and_reader_on_one_chip(backend, rate):
    """em4100_tag.loom on thread 2 and em4100_reader.loom on threads 0 and
    1, wired OUT0 -> IN1 (the field) and OUT1 -> IN0 (MOD to DEMOD). The
    reader reports the tag's ID, and MOD, read independently, is held to
    the reader's field."""
    bench = em_bench(backend)
    bench.add(Wire(RF_CLK, TAG_RF))
    bench.add(Wire(TAG_MOD, DEMOD, initial=1))
    mod, field = bench.add(EdgeLog(TAG_MOD)), bench.add(EdgeLog(RF_CLK))
    loom = load(backend, bench, TAG, READER)
    start_tag(loom, IDENT, rate)
    start_reader(loom, READER_P, rate)
    budget = frames_budget(rate, READER_P, 3)
    assert wait_report(bench, loom, budget) == (0, id_words(IDENT))
    rises = [c + 1 for c in field.rises()]      # the wire's clock of delay
    held_to_the_field(decode(mod.edges, rises, HALF[rate]))


# ---------------------------------------------------------------- rejection
REJECT_RTL = [(9, ROW | 0 << 4)]                # D00: row 0 odd
REJECT = [(36, ROW | 5 << 4),                   # D52: row 5 odd (and columns)
          (59 + 1, COLUMN)]                     # PC1: the columns alone


def rejected_then_read(backend, status, **tag_kw):
    """The reader rejects the tag's frame with ``status`` and reports no ID;
    once the tag sends good frames it reads the ID (after at most one more
    rejection, of the bad frame that was already going out)."""
    bench, loom, tag = reader_run(backend, IDENT, 32, **tag_kw)
    budget = frames_budget(32, READER_P, 2)
    got = wait_report(bench, loom, budget)
    if status is None:
        assert got[0] != 0 and got[1] == [], got
    else:
        assert got == (status, []), (hex(got[0]), got[1])
    tag.flip = tag.violate = None
    # the bad frame in flight may still be read once more (a stop bit of 1
    # joins it to the next frame's header); then the next good frame reads
    for _ in range(2):
        got = wait_report(bench, loom, frames_budget(32, READER_P, 3))
        if got[0] == 0:
            break
        assert got[1] == [], got
    assert got == (0, id_words(IDENT)), got


@pytest.mark.parametrize("flip,status", REJECT_RTL)
def test_em4100_reader_rejects_a_row_parity_error(backend, flip, status):
    """A data bit flipped: the row's even parity fails, the reader reports
    status 1 with the row number in bits 7:4, then reads the next good frame."""
    rejected_then_read(backend, status, flip=flip)


@model_only("more rejection cases; the RTL runs the row-0 one")
@pytest.mark.parametrize("flip,status", REJECT)
def test_em4100_reader_rejects_other_parity_errors(backend, flip, status):
    """Row 5 (checked before the columns) and a column parity bit alone."""
    rejected_then_read(backend, status, flip=flip)


@model_only("more rejection cases; the RTL runs the row-0 one")
def test_em4100_reader_rejects_a_stop_bit_of_1(backend):
    """S0 = 1 makes ten 1s before the next frame's data, so where the
    header ends is ambiguous; the data sheet's structure only says the frame
    is not valid. Whatever the reader takes as the header, it must reject
    the frame (some nonzero status) and never report a wrong ID."""
    rejected_then_read(backend, None, flip=63)


@model_only("more rejection cases; the RTL runs the row-0 one")
def test_em4100_reader_rejects_a_code_violation(backend):
    """A bit of row 2 with no transition in its middle: status 8."""
    rejected_then_read(backend, VIOL, violate=9 + 2 * 5 + 1)
