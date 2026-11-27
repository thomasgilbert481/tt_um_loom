"""L3-RDS: the RDS / RBDS baseband encoder and decoder, on the model and the RTL.

``firmware/rds_enc.loom`` (thread 0, the biphase baseband on OUT0) and
``firmware/rds_dec.loom`` (thread 1, a data slicer's biphase on IN0) are
assembled, loaded and driven through ``tools.loomhost.Loom``.
``tools.protomodels.rds`` supplies the reference, written from NRSC-4-A
and ITU-R BS.643-3: :class:`~rds.Line` records the encoder's pin, holds
every edge to the half-symbol grid and the rate to BS.643-3 1.4's
+-0.125 bit/s, and decodes it; :func:`~rds.receive` is a receiver that
syncs by syndrome (Annex C) and corrects bursts (2.3, Annex B);
:class:`~rds.Player` plays groups into the decoder from any bit, with
data-bit errors, slips and a rate offset; :class:`~rds.Wire` joins the two.

Expected numbers come from the standards and ``docs/SEMANTICS.md``: the
encoder's tick is a half-symbol and SEMANTICS 6.10 lands every edge on the
exact clock of its tick, so with an integer tick every edge sits exactly
on the grid, and with a fractional one within one clock of it.

Clocks. RDS is slow: a bit is 42,105 clocks at 50 MHz, a group 4.4 M. The
RTL runs both programs at a stated faster rate set by the tick, as the
shipped slow protocols do: the encoder at 512 clocks a bit (half-symbol
256, its proved minimum), the decoder at 1024 (tick 64, its proved
minimum); the rate check then uses the board clock that makes those the
standard's 1187.5 bit/s (608 kHz and 1.216 MHz). The real ticks at 50 and
45 MHz run on the model.
"""

import dataclasses

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import rds as R
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
ENC, DEC = 0, 1                                 # threads
ENC_HS = 256                                    # encoder half-symbol on the RTL
DEC_BIT = 1024                                  # decoder bit on the RTL (tick 64)
PI = 0x54A8
GROUPS = R.ps_groups(PI, "LOOM RDS")            # the encoder image's table

_PROGRAMS = {}


def program(name):
    if name not in _PROGRAMS:
        _PROGRAMS[name] = assemble_file(FIRMWARE / (name + ".loom"), isa=ISA,
                                        strict=True)
    return _PROGRAMS[name]


def make_bench(backend):
    """FIFOs, the deadline latch, the bit engine (CRC), data memory, the
    512-word memory; the RTL has them all."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM)
    return backend.bench()


def tick(clocks):
    """``(TICK_INT, TICK_FRAC)`` for a tick of ``clocks`` (SEMANTICS 4)."""
    p = int(round(clocks * 256))
    return p >> 8, p & 0xFF


def boot(backend, bench, *, enc_hs=None, dec_bit=None, groups=None, loss=None):
    """Load the programs asked for (the encoder's tick a half-symbol of
    ``enc_hs`` clocks, the decoder's a sixteenth of ``dec_bit``), patch the
    encoder's table and the decoder's LOSS, start them."""
    loom = backend.loom(bench, isa=ISA)
    threads = []
    for name, thread, clocks in (("rds_enc", ENC, enc_hs), ("rds_dec", DEC, dec_bit and dec_bit / 16)):
        if not clocks:
            continue
        prog = program(name)
        words = dict(prog.words)
        if name == "rds_enc" and groups is not None:
            for k, w in enumerate(w for g in groups for w in g):
                words[prog.symbols["tab"] + k] = w
        if name == "rds_dec" and loss is not None:
            words[prog.symbols["cfg"]] = loss
        loom.load(words)
        ti, tf = tick(clocks)
        loom.write_csr(thread, "TICK_INT", ti)
        loom.write_csr(thread, "TICK_FRAC", tf)
        threads.append(thread)
    loom.run(threads)
    return loom


# ------------------------------------------------------- the encoder
def tx(backend, hs, nbits, groups=None):
    """Run the encoder until ``nbits`` bits have gone out; return the line."""
    bench = make_bench(backend)
    line = bench.add(R.Line("OUT0"))
    loom = boot(backend, bench, enc_hs=hs, groups=groups)
    bench.run_until(lambda: len(line.edges) > 0, 100_000, every=64)
    bench.step(int(nbits * 2 * hs + hs))
    return line, loom


def check_tx(line, hs, nbits, groups, tol, clock_hz):
    """Every edge on the half-symbol grid, the rate within BS.643-3 1.4,
    the bits those of the groups in order, and the model receiver finds
    every block clean, offsets in group order."""
    assert line.check(hs, tol_clocks=tol, clock_hz=clock_hz) == []
    bits = line.bits(hs)[:nbits]
    want = R.stream(groups, repeat=nbits // 104 + 1)[:nbits]
    assert bits == want
    return bits


def test_rds_enc_group_cycle(backend):
    """The image's table (four 0A groups, PI 0x54A8, PS "LOOM RDS") at 512
    clocks a bit (a 608 kHz board at 1187.5 bit/s): one group and a block,
    bit-exact, every edge exactly on the 256-clock grid, and the model
    receiver syncs and reads PI, block 2 and the first two PS characters."""
    nbits = 104 + 26
    line, loom = tx(backend, ENC_HS, nbits)
    bits = check_tx(line, ENC_HS, nbits, GROUPS, 0, ENC_HS * 2375)
    reps = R.receive(bits)
    assert all(not (r.bad or r.corrected) for r in reps)
    assert [(r.offset, r.info) for r in reps][-3:] == [("C", 0xE0CD), ("D", 0x4C4F), ("A", PI)]
    assert loom.badop() == 0


@model_only("a 4.4 M-clock group at 50 MHz; the RTL run covers the code at a short tick")
@pytest.mark.parametrize("clock_hz", [50e6, 45e6])
def test_rds_enc_real_rate(backend, clock_hz):
    """The real half-symbol, 21052 + 162/256 at 50 MHz and 18947 + 94/256 at
    45 MHz: 40 bits, every edge within one clock of the ideal grid and the
    bit rate within +-0.125 bit/s of 1187.5 (BS.643-3 1.4)."""
    hs = R.half_symbol_clocks(clock_hz)
    ti, tf = tick(hs)
    nbits = 40
    line, _ = tx(backend, ti + tf / 256, nbits)
    check_tx(line, ti + tf / 256, nbits, GROUPS, 1.0, clock_hz)
    assert line.check(hs, tol_clocks=1.0, clock_hz=clock_hz) == []


@model_only("a second table; the RTL run covers the code")
def test_rds_enc_version_b_and_new_ps(backend):
    """A 0B cycle (B0 = 1: PI again in block 3, offset C') with another PS
    name: block 3 carries C', every block clean, the PS name read back."""
    groups = R.ps_groups(0xC0DE, "KLOOM 99", version_b=True)
    nbits = 4 * 104 + 30
    line, _ = tx(backend, ENC_HS, nbits, groups)
    bits = check_tx(line, ENC_HS, nbits, groups, 0, ENC_HS * 2375)
    reps = R.receive(bits)
    assert all(not (r.bad or r.corrected) for r in reps)
    assert {r.offset for r in reps} == {"A", "B", "C'", "D"}
    ps = {}
    for k in range(len(reps) - 2):
        if reps[k].offset == "B" and reps[k + 2].offset == "D":
            ps[reps[k].info & 3] = reps[k + 2].info
    assert "".join(chr(ps[s] >> 8) + chr(ps[s] & 0xFF) for s in range(4)) == "KLOOM 99"


# ------------------------------------------------------- the decoder
def expected(bits, *, loss=8, flips=(), drop=()):
    """The model receiver's reports for the bits the decoder is played,
    the same whichever of the first four bits it starts from (the decoder
    spends its first bit or two finding the bit clock)."""
    data = [b ^ (k in set(flips)) for k, b in enumerate(bits) if k not in set(drop)]
    runs = [R.receive(data[j:], loss=loss) for j in range(4)]
    assert all(r == runs[0] for r in runs), "the scenario depends on the first bits"
    return runs[0]


def collect(bench, loom, until, budget, chunk=4096):
    """Step, popping OUTQ[DEC] as it fills, until ``until(words)``."""
    words = []
    spent = 0
    while spent < budget and not until(words):
        bench.step(chunk)
        spent += chunk
        words += loom.pop_available(DEC)
    return words


def rx_play(backend, bits, *, bit=DEC_BIT, rate_scale=1.0, flips=(), drop=(),
            loss=None, nreports=None):
    """Play ``bits`` into the decoder; return its reports and the model's."""
    bench = make_bench(backend)
    player = bench.add(R.Player("IN0"))
    loom = boot(backend, bench, dec_bit=bit, loss=loss)
    player.play(bits, bit / 2 * rate_scale, flips=flips, drop=drop, lead=2000)
    want = expected(bits, loss=8 if loss is None else loss, flips=flips, drop=drop)
    n = len(want) if nreports is None else nreports
    words = collect(bench, loom, lambda w: len(w) >= 2 * n,
                    int((len(bits) + 30) * bit * rate_scale) + 20_000)
    return R.parse_reports(words), want, loom


def test_rds_dec_syncs_mid_block(backend):
    """Groups from the model encoder played from bit 13 of block 1 (an
    arbitrary bit offset): the decoder finds the blocks by syndrome alone,
    declares sync on the second block in sequence (block 3, flagged sync)
    and reports every block after it clean with its offset name, PI and
    the PS characters as sent."""
    bits = R.stream(GROUPS, 2)[13:13 + 104 + 50]
    got, want, loom = rx_play(backend, bits)
    assert want[0].sync and want[0].offset == "C"
    assert [(r.offset, r.info) for r in want[1:4]] == \
        [("D", 0x4C4F), ("A", PI), ("B", GROUPS[1][1])]
    assert got == want
    assert loom.badop() == 0


def test_rds_dec_corrects_loses_and_regains(backend):
    """From bit 13 of a group. Second group: a burst of 5 in block 2's
    information word (corrected, as sent), two errors 15 bits apart in
    block 4 (bad: no burst of span 5 has that syndrome). Third group: one
    error in block 3's checkword (corrected), then a slip in block 4 (one
    bit lost, the block bad): with LOSS = 2 those two in a row lose sync
    after block 4, and the decoder syncs again on the slipped stream (on
    the next group's block 2) and reads on."""
    bits = R.stream(GROUPS, 5)[13:13 + 4 * 104 - 20]
    o = -13                                     # stream index -> played index
    flips = [o + k for k in range(133, 138)] + [o + 184, o + 199, o + 282]
    drop = [o + 300]
    got, want, loom = rx_play(backend, bits, flips=flips, drop=drop, loss=2)
    kinds = [(r.offset, int(r.corrected), int(r.bad), int(r.sync), int(r.lost)) for r in want]
    assert kinds[:11] == [("C", 0, 0, 1, 0), ("D", 0, 0, 0, 0), ("A", 0, 0, 0, 0),
                          ("B", 1, 0, 0, 0), ("C", 0, 0, 0, 0), ("D", 0, 1, 0, 0),
                          ("A", 0, 0, 0, 0), ("B", 0, 0, 0, 0), ("C", 1, 0, 0, 0),
                          ("D", 0, 1, 0, 1), ("B", 0, 0, 1, 0)], kinds
    assert want[3].info == GROUPS[1][1] and want[8].info == GROUPS[2][2]
    lost = next(k for k, r in enumerate(want) if r.lost)
    assert want[lost + 1].sync and len(want) >= lost + 3
    assert got == want
    assert loom.badop() == 0


@model_only("eight blocks a run, three runs; the RTL scenario covers the trapping code")
@pytest.mark.parametrize("span", [1, 3, 5])
def test_rds_dec_corrects_bursts_everywhere(backend, span):
    """A burst of ``span`` bits (first and last bit wrong, the rest
    alternating) in each of eight blocks in a row, at a different place in
    each, from the top of the information word to the end of the
    checkword: every block reported corrected (LOSS 8 is not reached:
    eight are not clean) with its information word as sent."""
    bits = R.stream(GROUPS, 4)[13:13 + 3 * 104 + 30]
    o = -13
    flips = []
    for j in range(8):
        start = 104 + 26 * j + (j * 3) % (27 - span)        # stream index
        pattern = [1] + [(i & 1) for i in range(span - 2)] + [1] if span > 1 else [1]
        flips += [o + start + i for i, f in enumerate(pattern) if f]
    got, want, loom = rx_play(backend, bits, flips=flips, loss=9)
    sent = [w for g in GROUPS for w in g] * 4
    fixed = [r for r in want if r.corrected]
    assert len(fixed) == 8 and not any(r.bad for r in want)
    assert want[0].offset == "C" and [r.info for r in want[2:10]] == sent[4:12]
    assert got == want


@model_only("bit-rate offsets: robustness of the clock recovery")
@pytest.mark.parametrize("offset", [-105e-6, 105e-6, -0.01, 0.01])
def test_rds_dec_rate_offset(backend, offset):
    """The stream at BS.643-3 1.4's limits (+-0.125 bit/s, +-105 ppm) and
    at 1 % (95 times them): every block read as sent; the clock recovery
    re-centres on every mid-bit edge."""
    bits = R.stream(GROUPS, 2)[7:7 + 104 + 50]
    got, want, loom = rx_play(backend, bits, rate_scale=1 + offset)
    assert got == want and len(got) >= 4 and not any(r.bad for r in got)


@model_only("3.4 M clocks at 45 MHz; the RTL runs the code at a short tick")
def test_rds_dec_real_rate_45mhz(backend):
    """The real tick at the signed-off 45 MHz, 2368 + 108/256 (a 16th of a
    37,894.7-clock bit), from bit 20 of a group: sync and two blocks."""
    bits = R.stream(GROUPS, 2)[20:20 + 90]
    got, want, loom = rx_play(backend, bits, bit=45e6 / R.BIT_RATE)
    assert got == want and len(got) >= 2 and got[0].sync


@model_only("both programs on one chip; each is covered on the RTL alone")
def test_rds_enc_and_dec_together(backend):
    """The encoder (thread 0, a 0B cycle, so block 3 carries C') wired to
    the decoder (thread 1) at 1024 clocks a bit: the decoder reports what
    the encoder sends, every block clean, C' named."""
    groups = R.ps_groups(0x1F2E, "TWO ON 1", version_b=True)
    bench = make_bench(backend)
    bench.add(R.Wire("OUT0", "IN0"))
    loom = boot(backend, bench, enc_hs=DEC_BIT // 2, dec_bit=DEC_BIT, groups=groups)
    words = collect(bench, loom, lambda w: len(w) >= 2 * 6, 200 * DEC_BIT)
    got = R.parse_reports(words)
    sent = [w for g in groups for w in g] * 3
    assert got[0].sync and not any(r.bad or r.corrected for r in got)
    k = ("A", "B", "C'", "D").index(got[0].offset)     # sync in the first group
    assert [r.offset for r in got[:6]] == [("A", "B", "C'", "D")[(k + j) % 4] for j in range(6)]
    assert [r.info for r in got[:6]] == sent[k:k + 6]
    assert loom.badop() == 0


@model_only("a host that stops reading; the RTL scenarios cover the push path")
def test_rds_dec_overflow_is_reported(backend):
    """The host reads nothing for six blocks: OUTQ (four words) keeps the
    first reports whole, the rest are dropped, and the first report after
    the host reads again carries the overflow bit; every report that
    arrives is one of the model's, whole."""
    bits = R.stream(GROUPS, 4)[13:13 + 3 * 104 + 40]
    bench = make_bench(backend)
    player = bench.add(R.Player("IN0"))
    loom = boot(backend, bench, dec_bit=DEC_BIT)
    player.play(bits, DEC_BIT / 2, lead=2000)
    want = expected(bits)
    bench.step(2000 + (51 + 6 * 26) * DEC_BIT)         # sync, then six blocks
    words = collect(bench, loom, lambda w: False, (len(bits) - 200) * DEC_BIT)
    got = R.parse_reports(words)
    plain = [dataclasses.replace(r, overflow=False) for r in got]
    assert got[:2] == want[:2]                          # OUTQ held two whole reports
    k = next(i for i, r in enumerate(got) if r.overflow)
    assert k == 2 and not any(r.overflow for r in got[k + 1:])
    assert all(r in want for r in plain) and plain[-1] == want[-1]
