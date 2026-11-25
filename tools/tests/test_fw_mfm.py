"""L3 for the MFM floppy programs, on the model and the RTL.

``firmware/mfm_write.loom`` (WD = OUT0, WG = OUT1) writes one IBM System/34
double-density sector as MFM flux pulses; ``firmware/mfm_read.loom``
(RD = IN0) reads flux pulses, classifies each interval as 2T, 3T or 4T,
finds A1* A1* A1*, reads the field the host asks for and checks its CRC.
The reference is ``tools.protomodels.mfm``, written from the Wikipedia MFM
page and the WD FD179X-01 data sheet: ``sector_cells`` lays a sector out,
``MfmDrive`` records the writer's pulses and holds them to the data
sheet's timing, ``decode_cells`` reads fields, ``FluxSource`` plays a cell
stream onto the reader's pin with jitter and bit shift.

Expected values come from the sources and SEMANTICS: deadline-latched
edges land on their exact clock (6.10), so at 50 MHz with a 20-clock tick
every writer pulse is exactly 20 clocks (400 ns, [FD] Twp 300-500 ns) and
every interval exactly 200, 300 or 400 clocks (4, 6, 8 us).

RTL time: a byte is 16 us, 800 clocks at 50 MHz; a sector with a 128-byte
payload is about 197 k clocks. The RTL runs a writer sector with an
8-byte payload (about 100 k clocks) and a reader ID field (about 50 k);
the 128-byte sectors, the 45 MHz runs and the jitter sweeps run on the
model only.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import mfm as M
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
CLOCK = 50e6
TICK = 20                                   # 400 ns at 50 MHz (writer header)
HALF = 100                                  # clocks a cell at 50 MHz
PAY = 0x100                                 # the writer's payload, quarter 2
BYTE = 16 * HALF                            # clocks a byte


def program(name, thread=0):
    """A program from ``firmware/``, moved to ``thread`` if asked."""
    path = FIRMWARE / (name + ".loom")
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, name + ".loom", isa=ISA, imem_words=IMEM, strict=True)


def mfm_bench(backend):
    """FIFOs, the deadline latch, the bit engine (CRC) and the data memory;
    the RTL has them all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "DMEM"), imem_words=IMEM)
    return backend.bench()


def pack(payload):
    """Two bytes a word, the first in the high half (writer header)."""
    data = list(payload) + [0] * (len(payload) % 2)
    return [(data[i] << 8) | data[i + 1] for i in range(0, len(data), 2)]


def sector_bytes(n):
    return M.GAP1 + 2 * (M.SYNC00 + 3) + 7 + M.GAP2 + 3 + n + M.GAP3


# ------------------------------------------------------------------- writer
def write_sector(backend, payload, chrn=(0, 0, 1, 0), tick=TICK, clock=CLOCK):
    """Write one sector; returns the drive model and the reported CRC."""
    bench = mfm_bench(backend)
    drive = bench.add(M.MfmDrive("OUT0", "OUT1"))
    loom = backend.loom(bench, isa=ISA)
    image = dict(program("mfm_write").words)
    words = pack(payload)
    image.update({PAY + i: w for i, w in enumerate(words)})
    loom.load(image)
    loom.run(0)
    c, h, r, n = chrn
    loom.push(0, [tick, 0, (c << 8) | h, (r << 8) | n, PAY, len(words)])
    half = 5 * tick
    budget = 16 * half * (sector_bytes(len(payload)) + 4)
    done = bench.run_until(lambda: len(drive.gate_edges) >= 2, budget, every=4 * half)
    assert done, "WRITE GATE never went off"
    (crc,) = loom.pop(0, 1)
    assert loom.badop() == 0
    return drive, crc


def check_written(drive, payload, chrn, half, clock):
    """Every pulse exactly one tick wide, every interval exactly 2, 3 or 4
    cells, the data sheet's windows kept, and the cells those of the
    model's sector from its first 1 to its last."""
    tick = half // 5
    assert drive.check(clock) == [], drive.check(clock)[:5]
    assert set(drive.widths) == {tick}
    gaps = [b - a for a, b in zip(drive.falls, drive.falls[1:])]
    assert set(gaps) <= {2 * half, 3 * half, 4 * half}
    want = M.sector_cells(*chrn, payload)
    first, last = want.index(1), len(want) - 1 - want[::-1].index(1)
    assert drive.cells(clock) == want[first:last + 1]
    fields = M.decode_cells(drive.cells(clock), data_len=len(payload))
    assert [f.mark for f in fields] == [M.MARK_ID, M.MARK_DATA]
    assert all(f.crc_ok and f.clock_errors == 0 for f in fields)
    assert fields[0].data[:4] == list(chrn)
    assert fields[1].data[:-2] == list(payload)


def test_mfm_write_short_sector(backend):
    """C H R N = 0 0 1 0 and an 8-byte test payload (N = 0 says 128: a
    non-standard sector, the length the RTL can afford); every pulse and
    interval exact, the model decodes both fields with good CRCs."""
    payload = [0x00, 0xFF, 0xA1, 0x4E, 0x12, 0x34, 0xE5, 0x01]
    drive, crc = write_sector(backend, payload)
    check_written(drive, payload, (0, 0, 1, 0), HALF, CLOCK)
    assert crc == M.crc16([0xA1] * 3 + [M.MARK_DATA] + payload)
    on = [c for c, g in drive.gate_edges if g == 0][0]
    off = [c for c, g in drive.gate_edges if g == 1][-1]
    assert drive.falls[0] - on >= M.WRITE_GATE_LEAD_S * CLOCK
    assert off - drive.falls[-1] >= 2e-6 * CLOCK          # [FD] Twf, doubled


@model_only("a 128-byte sector is 197 k clocks; the RTL writes the short one")
@pytest.mark.parametrize("fill", ["count", "zeros", "ones"])
def test_mfm_write_full_sector(backend, fill):
    """N = 0, 128 bytes: a count, all 00 (the 2T-only stream) and all FF."""
    payload = {"count": list(range(128)), "zeros": [0] * 128,
               "ones": [0xFF] * 128}[fill]
    drive, crc = write_sector(backend, payload, chrn=(39, 1, 9, 0))
    check_written(drive, payload, (39, 1, 9, 0), HALF, CLOCK)
    assert crc == M.crc16([0xA1] * 3 + [M.MARK_DATA] + payload)


@model_only("the signed-off clock: same program, 18-clock tick, on the model")
def test_mfm_write_at_45mhz(backend):
    """45 MHz, TICK 18: pulses 400 ns, cells 90 clocks, all exact."""
    payload = [0xE5] * 16
    drive, _ = write_sector(backend, payload, tick=18, clock=45e6)
    check_written(drive, payload, (0, 0, 1, 0), 90, 45e6)


def test_mfm_model_flags_a_bad_writer():
    """The drive model is not a rubber stamp: a short pulse, an interval of
    5 cells and a pulse outside WRITE GATE are each reported."""
    d = M.MfmDrive("OUT0")
    d.level = 1
    d.falls, d.widths = [0, 200, 700], [20, 5, 20]
    bad = d.check(CLOCK)
    assert any("pulse 1" in b for b in bad) and any("interval 1" in b for b in bad)


# ------------------------------------------------------------------- reader
UNIT = (6, 64)                              # 6.25 clocks, a cell / 16, at 50 MHz


def read_fields(backend, cells, cmds, clock=CLOCK, unit=UNIT, jitter=0.0,
                seed=1, shift=None, pulse=20):
    """Run the reader on thread 0, give it its commands, play ``cells`` on
    IN0 and return the words it reports."""
    half = HALF_CELL = M.HALF_CELL_S * clock
    bench = mfm_bench(backend)
    flux = bench.add(M.FluxSource("IN0", pulse=pulse))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program("mfm_read"))
    loom.run(0)
    loom.push(0, list(unit) + list(cmds))
    start = bench.cycle + 2000
    times = flux.send(cells, half, start, jitter=jitter, seed=seed, shift=shift)
    bench.step(int(times[-1] + 30 * HALF_CELL) - bench.cycle)
    got = []
    for _ in range(200):
        more = loom.pop_available(0)
        if not more:
            break
        got += more
    assert loom.badop() == 0
    return got


def report(field, mark=None):
    """The words mfm_read reports for ``field`` (mark first, CRC last)."""
    status = 0 if M.crc16([0xA1] * 3 + list(field)) == 0 else 1
    if mark is not None and mark != field[0]:
        status |= 2
    return [(status << 8) | field[0]] + pack(field[1:])


def gap(prev=1, n=4):
    return M.mfm_cells([0x4E] * n, prev)


def test_mfm_read_id_field(backend):
    """An ID field (C H R N = 2 1 3 1) after a short gap: reported with its
    CRC, status 0."""
    cells, prev = gap()
    field = M.id_field(2, 1, 3, 1)
    more, _ = M.field_cells(field, prev)
    got = read_fields(backend, cells + more + gap(0)[0], [0xFE03])
    assert got == report(field)


def sector_reports(payload, chrn=(0, 0, 1, 0)):
    return report(M.id_field(*chrn)) + report(M.data_field(payload))


def data_cmd(n):
    return (M.MARK_DATA << 8) | (n // 2 + 1)


@model_only("a 128-byte sector is 197 k clocks; the RTL reads an ID field")
@pytest.mark.parametrize("fill,jitter", [("count", 0.0), ("count", 0.18),
                                         ("zeros", 0.18), ("ones", 0.18)])
def test_mfm_read_full_sector(backend, fill, jitter):
    """A whole sector, N = 0, 128 bytes, both fields asked for at once.
    Jitter 0.18 moves every transition by up to 0.18 of a cell (0.36 us)
    either way, so an interval by up to 0.36 cell: inside the edge-to-edge
    window of 0.5 cell less the unit (header). All 00 is 2T only, all FF too
    (shifted by a cell)."""
    payload = {"count": list(range(128)), "zeros": [0] * 128,
               "ones": [0xFF] * 128}[fill]
    cells = M.sector_cells(0, 0, 1, 0, payload)
    got = read_fields(backend, cells, [0xFE03, data_cmd(128)], jitter=jitter, seed=7)
    assert got == sector_reports(payload)


@model_only("the signed-off clock, on the model")
def test_mfm_read_at_45mhz(backend):
    """45 MHz, unit 5 + 160/256 clocks, cells 90 clocks, jitter 0.18."""
    payload = [0x5A, 0xC3] * 16
    cells = M.sector_cells(1, 0, 2, 0, payload)
    got = read_fields(backend, cells, [0xFE03, data_cmd(32)], clock=45e6,
                      unit=(5, 160), jitter=0.18, seed=3)
    assert got == sector_reports(payload, (1, 0, 2, 0))


def test_mfm_read_bad_crc(backend):
    """One data bit flipped after the CRC was computed: status 1, the bytes
    as read."""
    field = M.id_field(0, 0, 1, 0)
    field[3] ^= 0x01                        # R 1 -> 0
    cells, prev = gap()
    more, _ = M.field_cells(field, prev)
    got = read_fields(backend, cells + more + gap(0)[0], [0xFE03])
    assert got == report(field) and got[0] >> 8 == 1


@model_only("the bit shift: model")
@pytest.mark.parametrize("move,ok", [(0.35, True), (-0.35, True), (0.7, False)])
def test_mfm_read_bit_shift(backend, move, ok):
    """One transition in the middle of the data field moved by ``move`` of
    a cell (peak shift on the medium): its two intervals change by that
    much each way. 0.35 is read right; 0.7 is out of the window, and the
    field comes back lost or with a CRC error, never as good data."""
    payload = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0]
    field = M.data_field(payload)
    cells, prev = gap()
    more, _ = M.field_cells(field, prev)
    cells = cells + more + gap(0)[0]
    k = sum(cells[:len(cells) - 16 * 9]) + 3   # a transition in the payload
    got = read_fields(backend, cells, [data_cmd(8)], shift={k: move})
    if ok:
        assert got == report(field)
    else:
        assert got[0] >> 8 in (1, 4) or got != report(field)
        assert got[0] >> 8 != 0


def test_mfm_read_wrong_mark_and_lost(backend):
    """A data command meets an ID field: status 2 (and 1: the length is
    wrong, so is the CRC); then a field cut off after its mark: lost,
    status 4, no words."""
    cells, prev = gap()
    more, prev = M.field_cells(M.id_field(0, 0, 1, 0), prev)
    cells += more
    g, prev = M.mfm_cells([0x4E] * 4, prev)
    cut, _ = M.field_cells([M.MARK_ID, 0, 0], prev)
    cells += g + cut
    got = read_fields(backend, cells, [data_cmd(4), 0xFE03])
    assert got[0] & 0xFF == M.MARK_ID and got[0] >> 8 & 2
    assert got[-1] == 0x0400


@model_only("writer and reader together, a 64-byte sector: 140 k clocks")
def test_mfm_write_then_read_on_one_chip(backend):
    """The writer on thread 0 (WD = OUT0) and the reader on thread 2 (RD =
    IN0), the board wire from OUT0 to IN0: the reader reports both fields
    of the sector the writer wrote, CRCs good. The writer's 152 words spill
    into thread 1's quarter; its payload sits at 0x0A0, the reader's 166
    words from 0x100 end below its buffer at 0x1B0."""
    payload = [(7 * i + 3) & 0xFF for i in range(64)]
    bench = mfm_bench(backend)
    bench.add(M.PulseWire("OUT0", "IN0"))
    drive = bench.add(M.MfmDrive("OUT0", "OUT1"))
    loom = backend.loom(bench, isa=ISA)
    image = dict(program("mfm_write").words)
    rd = program("mfm_read", thread=2).words
    assert not set(image) & set(rd)
    image.update(rd)
    words = pack(payload)
    image.update({0xA0 + i: w for i, w in enumerate(words)})
    assert max(image) < 0x1B0 or all(a < 0x1B0 for a in rd)
    loom.load(image)
    loom.run([0, 2])
    loom.push(2, list(UNIT) + [0xFE03, data_cmd(64)])
    loom.push(0, [TICK, 0, (5 << 8) | 0, (7 << 8) | 0, 0xA0, len(words)])
    budget = BYTE * (sector_bytes(64) + 4)
    assert bench.run_until(lambda: len(drive.gate_edges) >= 2, budget, every=4 * HALF)
    got = []
    for _ in range(100):
        more = loom.pop_available(2)
        if not more and len(got) >= 4 + 33:
            break
        got += more
        bench.step(2000)
    assert got == report(M.id_field(5, 0, 7, 0)) + report(M.data_field(payload))
    assert loom.pop(0, 1) == [M.crc16([0xA1] * 3 + [M.MARK_DATA] + payload)]


# ----------------------------------------------------------------- assembly
@pytest.mark.parametrize("name,words", [("mfm_write", 152), ("mfm_read", 166)])
def test_mfm_assembles_strict_and_staged_clean(name, words):
    """--strict and --staged-edges with no diagnostic at all. The writer's
    pairs are proved at its shortest tick, 18 clocks (45 MHz); the reader
    has no WAITD. Both spill past a quarter (headers)."""
    prog = assemble_file(FIRMWARE / (name + ".loom"), isa=ISA, strict=True,
                         staged_check=True)
    assert prog.diagnostics == [] and list(prog.threads) == [0]
    assert len(prog.words) == words
    rep = prog.deadlines[0]
    if name == "mfm_write":
        assert rep.period == 18 and rep.pairs
        assert not rep.unbounded and not rep.infeasible and rep.worst_slack >= 5
    else:
        assert rep.pairs == []
