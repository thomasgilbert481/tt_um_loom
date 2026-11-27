"""L3-ZB868: the IEEE 802.15.4 868 MHz BPSK PHY (baseband chips), both ends.

``firmware/zb868_tx.loom`` (thread 0; chips on OUT0, the raw bits on TXD =
OUT1) and ``firmware/zb868_rx.loom`` (thread 2; chips on IN0, the decoded
bits on RXD = OUT2) are assembled, loaded and driven through
``tools.loomhost.Loom``. ``tools.protomodels.ieee802154`` is the reference,
written from IEEE Std 802.15.4-2006: :class:`~ieee802154.ChipLine` records
the transmitter's chip line, checks every edge against the chip grid and
the chip rate against 6.6.3.3's +-40 ppm, and decodes the PPDU
(:func:`~ieee802154.decode_chips`: correlation, 6.6.2.2's differential
decoding, SFD, PHR, the 7.2.1.9 FCS); :class:`~ieee802154.ChipSource`
plays :func:`~ieee802154.ppdu_chips` into the receiver with a chip-rate
error and :func:`~ieee802154.flip_chips` wrong chips in every bit;
:class:`~ieee802154.Wire` joins OUT0 to IN0.

Expected numbers come from the standard and ``docs/SEMANTICS.md``: the
transmitter's tick is one chip and SEMANTICS 6.10 lands every staged edge
on its tick's exact clock, so at 45 MHz (150 clocks a chip, an integer
tick) every edge is exactly on the chip grid, and at 50 MHz (166.67, a
fractional tick) within one clock of it. The receiver's report words are
the program's documented host interface (its header).

Clocks: 45 MHz, the signed-off clock (300 kchip/s = 150 clocks a chip),
and 50 MHz (166.67). RTL time: the two RTL scenarios send a 5-octet PSDU
(an acknowledgment-sized frame, 1320 chips, about 200 k clocks each); the
rest run on the model only.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import ieee802154 as Z
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
TX, RX = 0, 2                                   # threads
TC45 = Z.chip_clocks(868, 45e6)                 # 150.0
TC50 = Z.chip_clocks(868, 50e6)                 # 166.67
EXAMPLE_MHR = [0x02, 0x00, 0x6A]                # 7.2.1.9's worked example
MPDU = [0x41, 0x88, 0x2C, 0xCD, 0xAB, 0xFF, 0xFF, 0x34, 0x12, 0x4C, 0x6F, 0x6F, 0x6D]

_PROGRAMS = {}


def program(name):
    if name not in _PROGRAMS:
        _PROGRAMS[name] = assemble_file(FIRMWARE / (name + ".loom"), isa=ISA,
                                        strict=True)
    return _PROGRAMS[name]


def make_bench(backend):
    """FIFOs, the deadline latch, the bit engine (shift register, CRC), the
    512-word memory; the RTL has them all."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE"), imem_words=IMEM)
    return backend.bench()


def tick(clocks):
    """``(TICK_INT, TICK_FRAC)`` for a tick of ``clocks`` (SEMANTICS 4)."""
    p = int(round(clocks * 256))
    return p >> 8, p & 0xFF


def boot(backend, bench, *, tx_tc=None, rx_tc=None):
    """Load the programs asked for: the transmitter's tick is one chip of
    ``tx_tc`` clocks, the receiver's a fifteenth of ``rx_tc``; start them."""
    loom = backend.loom(bench, isa=ISA)
    threads = []
    for name, thread, tc, div in (("zb868_tx", TX, tx_tc, 1), ("zb868_rx", RX, rx_tc, 15)):
        if tc is None:
            continue
        loom.load(program(name).words)
        ti, tf = tick(tc / div)
        loom.write_csr(thread, "TICK_INT", ti)
        loom.write_csr(thread, "TICK_FRAC", tf)
        threads.append(thread)
    loom.run(threads)
    return loom


def frame_clocks(psdu_len, tc):
    return (Z.PREAMBLE_OCTETS + 2 + psdu_len) * 8 * Z.NCHIP * tc


def rx_words(psdu, flags=0x8000):
    """What the receiver's header says it reports for ``psdu``."""
    return list(psdu) + [flags | len(psdu)]


def collect(bench, loom, until, budget, chunk=4096):
    """Step, popping OUTQ[RX] as it fills, until ``until(words)``."""
    words = []
    spent = 0
    while spent < budget and not until(words):
        bench.step(chunk)
        spent += chunk
        words += loom.pop_available(RX)
    return words


# ------------------------------------------------------- the model alone
def test_zb868_model_fcs_is_the_standards_example():
    """7.2.1.9: the MHR 0100 0000 0000 0000 0101 0110 has the FCS bits
    0010 0111 1001 1110, r0 first."""
    assert Z.octet_bits(EXAMPLE_MHR) == [int(c) for c in "010000000000000001010110"]
    assert Z.fcs_bits(EXAMPLE_MHR) == [int(c) for c in "0010011110011110"]
    assert Z.fcs_ok(Z.with_fcs(MPDU)) and not Z.fcs_ok(Z.with_fcs(MPDU)[:-1] + [0])


def test_zb868_model_spreading_and_differential_coding():
    """Table 27 and 6.6.2.2: the preamble is the 0 sequence 32 times; a
    raw 1 inverts the chips from that bit on; flipped chips up to 7 a bit
    still despread right."""
    chips = Z.ppdu_chips(Z.with_fcs(EXAMPLE_MHR))
    assert chips[:15] == [1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0]
    assert chips[:32 * 15] == list(Z.CHIPS0) * 32
    assert chips[32 * 15:33 * 15] == list(Z.CHIPS1)          # SFD bit 0 is a 1
    assert Z.diff_decode(Z.diff_encode([1, 0, 1, 1, 0])) == [1, 0, 1, 1, 0]
    f = Z.decode_chips(Z.flip_chips(chips, 7, seed=5))
    assert f.psdu == Z.with_fcs(EXAMPLE_MHR) and f.fcs_ok and f.chip_errors == 7


# ------------------------------------------------------- the transmitter
def tx_frames(backend, tc, mpdus):
    """Send each MPDU as a frame (the program appends the FCS); return the
    line, the host reports and the loom once every report is in."""
    bench = make_bench(backend)
    line = bench.add(Z.ChipLine("OUT0"))
    loom = boot(backend, bench, tx_tc=tc)
    for m in mpdus:
        loom.push(TX, [len(m) + 2] + list(m))      # waits while INQ is full
    budget = 150_000 + int(sum(frame_clocks(len(m) + 2, tc) for m in mpdus))
    reports, spent = [], 0
    while len(reports) < len(mpdus) and spent < budget:
        bench.step(4096)
        spent += 4096
        reports += loom.pop_available(TX)
    return bench, line, loom, reports


def check_tx(line, tc, mpdus, ti_tf, tol=0.0):
    """Every edge on the chip grid of the tick the program was given,
    the chip rate within 40 ppm of the band's, and each frame decoded by
    the model with no wrong chip and a good FCS."""
    ti, tf = ti_tf
    assert line.check(ti + tf / 256, tol_clocks=tol, nominal=tc) == []
    frames = line.frames(tc)
    assert [f.psdu for f in frames] == [Z.with_fcs(m) for m in mpdus]
    for f in frames:
        assert f.fcs_ok and f.chip_errors == 0 and f.problems == [] and f.reserved == 0


def test_zb868_tx_to_model_decoder(backend):
    """45 MHz, 150 clocks a chip (an exact integer tick): 7.2.1.9's
    example MHR as a 5-octet PSDU. Every edge exactly on the chip grid;
    the model decodes the PPDU; its FCS octets are the standard's
    0010 0111 1001 1110; the host gets the length back."""
    bench, line, loom, reports = tx_frames(backend, TC45, [EXAMPLE_MHR])
    check_tx(line, TC45, [EXAMPLE_MHR], tick(TC45))
    assert line.frames(TC45)[0].psdu[3:] == Z.bits_octets([int(c) for c in "0010011110011110"])
    assert reports == [5]
    assert line.edges[-1][1] == 0 and loom.badop() == 0


@model_only("a fractional tick and two frames: 0.9 M clocks; the 45 MHz RTL run covers the code")
def test_zb868_tx_50mhz_two_frames(backend):
    """50 MHz, 166.67 clocks a chip (tick 166 + 171/256): a 15-octet data
    frame, then the example ACK. Every edge within one clock of the grid,
    the chip rate within 6.6.3.3's 40 ppm, both frames decoded."""
    mpdus = [MPDU, EXAMPLE_MHR]
    bench, line, loom, reports = tx_frames(backend, TC50, mpdus)
    check_tx(line, TC50, mpdus, tick(TC50), tol=1.0)
    assert reports == [15, 5]


@model_only("a length check with nothing on the line")
def test_zb868_tx_refuses_bad_lengths(backend):
    """Lengths 1 and 128 are outside 2..127 (6.3.3, Table 22): refused
    (0xA000), the line stays low; a 2-octet PSDU (the FCS of nothing,
    0x0000) still goes out."""
    bench = make_bench(backend)
    line = bench.add(Z.ChipLine("OUT0"))
    loom = boot(backend, bench, tx_tc=TC45)
    loom.push(TX, [1, 128])
    assert loom.pop(TX, 2) == [0xA000, 0xA000]
    assert line.edges == []
    loom.push(TX, [2])
    bench.step(int(frame_clocks(2, TC45)) + 2000)
    assert loom.pop(TX, 1) == [2]
    assert [f.psdu for f in line.frames(TC45)] == [[0, 0]]


@model_only("a host that stops feeding octets")
def test_zb868_tx_underrun_is_reported(backend):
    """L = 10 but only 3 octets: the frame is cut at the end of the chip
    that found INQ empty, the line low again, and the report says 3
    octets went (0xC000 | 3)."""
    bench = make_bench(backend)
    line = bench.add(Z.ChipLine("OUT0"))
    loom = boot(backend, bench, tx_tc=TC45)
    loom.push(TX, [10, 1, 2, 3])
    assert loom.pop(TX, 1, max_polls=4000) == [0xC000 | 3]
    assert line.edges[-1][1] == 0
    last = line.edges[-1][0] - line.edges[0][0]
    assert (6 + 3) * 120 * TC45 - 8 * TC45 <= last <= (6 + 3) * 120 * TC45


# ------------------------------------------------------- the receiver
def rx_play(backend, chips, nwords, *, tc=TC45, ppm=0.0, rx_tc=None):
    """Play ``chips`` into the receiver; return the words it reported."""
    bench = make_bench(backend)
    src = bench.add(Z.ChipSource("IN0"))
    loom = boot(backend, bench, rx_tc=tc if rx_tc is None else rx_tc)
    src.play(chips, tc, ppm=ppm, lead=2000)
    words = collect(bench, loom, lambda w: len(w) >= nwords,
                    int((len(chips) + 60) * tc) + 60_000)
    return words, loom


def test_zb868_rx_from_model_source(backend):
    """45 MHz: the example ACK (5-octet PSDU) from the model, with 2 of
    the 15 chips wrong in every bit, preamble included: all five octets
    and a good report (0x8000 | 5)."""
    psdu = Z.with_fcs(EXAMPLE_MHR)
    chips = Z.flip_chips(Z.ppdu_chips(psdu), 2, seed=1)
    words, loom = rx_play(backend, chips, len(psdu) + 1)
    assert words == rx_words(psdu)
    assert loom.badop() == 0


@model_only("the RTL scenario covers the code at 2 wrong chips; this is the correlator's range")
@pytest.mark.parametrize("flips", [0, 3])
def test_zb868_rx_wrong_chips_per_bit(backend, flips):
    """0 and 3 wrong chips in every bit (3 is the most the acquisition
    takes, |correlation| >= 9): a 15-octet frame decodes whole."""
    psdu = Z.with_fcs(MPDU)
    chips = Z.flip_chips(Z.ppdu_chips(psdu), flips, seed=7 + flips)
    words, _ = rx_play(backend, chips, len(psdu) + 1)
    assert words == rx_words(psdu)


@model_only("the correlator's range after the lock")
def test_zb868_rx_four_wrong_chips_after_lock(backend):
    """3 wrong chips a bit in the first 16 preamble bits, then 4 in every
    bit from there to the end: after the lock a bit needs |correlation|
    > 5, so 4 still decode."""
    psdu = Z.with_fcs(MPDU)
    chips = Z.flip_chips(Z.flip_chips(Z.ppdu_chips(psdu)[:16 * 15], 3, seed=3)
                         + Z.ppdu_chips(psdu)[16 * 15:], 0)
    chips = chips[:16 * 15] + Z.flip_chips(chips[16 * 15:], 4, seed=4)
    words, _ = rx_play(backend, chips, len(psdu) + 1)
    assert words == rx_words(psdu)


@model_only("chip-rate errors far outside the standard: robustness of the clock recovery")
@pytest.mark.parametrize("ppm", [-10000, -200, 200, 10000])
def test_zb868_rx_chip_rate_error(backend, ppm):
    """The source's chip rate 200 ppm (five times 6.6.3.3's 40) and 1 %
    off either way, with 2 wrong chips a bit: the edge tracking still
    reads every bit."""
    psdu = Z.with_fcs(MPDU)
    chips = Z.flip_chips(Z.ppdu_chips(psdu), 2, seed=11)
    words, _ = rx_play(backend, chips, len(psdu) + 1, ppm=ppm)
    assert words == rx_words(psdu)


@model_only("a corrupted FCS: the RTL scenarios cover the report path")
def test_zb868_rx_bad_fcs_is_reported(backend):
    """One payload octet changed after the FCS was computed: every octet
    as sent, and the report says bad FCS (0xC000 | length)."""
    psdu = Z.with_fcs(MPDU)
    psdu[4] ^= 0x10
    words, _ = rx_play(backend, Z.ppdu_chips(psdu), len(psdu) + 1)
    assert not Z.fcs_ok(psdu)
    assert words == rx_words(psdu, 0xC000)


@model_only("a frame cut short")
def test_zb868_rx_lost_signal_is_reported(backend):
    """The source stops 6 octets into a 15-octet PSDU: the six octets,
    then a lost report (0xA000 | 15) when the idle line gives no
    correlation; then a whole frame is received normally."""
    psdu = Z.with_fcs(MPDU)
    chips = Z.ppdu_chips(psdu)[:(6 + 6) * 120 + 40]
    bench = make_bench(backend)
    src = bench.add(Z.ChipSource("IN0"))
    loom = boot(backend, bench, rx_tc=TC45)
    src.play(chips, TC45, lead=2000)
    src.play(Z.ppdu_chips(psdu), TC45, lead=30_000)
    words = collect(bench, loom, lambda w: len(w) >= 7 + len(psdu) + 1,
                    int((len(chips) + 1900) * TC45) + 100_000)
    assert words == psdu[:6] + [0xA000 | 15] + rx_words(psdu)


@model_only("the code-phase search: the RTL scenario starts at c0")
@pytest.mark.parametrize("late", [1, 7, 14])
def test_zb868_rx_late_start_slips_to_the_code_phase(backend, late):
    """The receiver comes in ``late`` chips into the preamble (the first
    edge it sees is not c0): the serial search slips a chip at a time
    until a bit correlates, with 2 wrong chips a bit."""
    psdu = Z.with_fcs(EXAMPLE_MHR)
    chips = Z.flip_chips(Z.ppdu_chips(psdu), 2, seed=late)[late:]
    while not chips[0]:
        chips = chips[1:]
    words, _ = rx_play(backend, chips, len(psdu) + 1)
    assert words == rx_words(psdu)


@model_only("a fractional tick: the 45 MHz RTL run covers the code")
def test_zb868_rx_50mhz(backend):
    """50 MHz, 166.67 clocks a chip, tick 11 + 28/256 (1/15 chip, 200 ppm
    short): a 15-octet frame with 3 wrong chips a bit."""
    psdu = Z.with_fcs(MPDU)
    chips = Z.flip_chips(Z.ppdu_chips(psdu), 3, seed=2)
    words, _ = rx_play(backend, chips, len(psdu) + 1, tc=TC50)
    assert words == rx_words(psdu)


# ------------------------------------------------------- both ends
def together(backend, tc, mpdu):
    """The transmitter (thread 0) wired to the receiver (thread 2) on one
    chip; returns the receiver's words and the transmitter's report."""
    bench = make_bench(backend)
    bench.add(Z.Wire("OUT0", "IN0"))
    line = bench.add(Z.ChipLine("OUT0"))
    loom = boot(backend, bench, tx_tc=tc, rx_tc=tc)
    pending = [len(mpdu) + 2] + list(mpdu)
    psdu = Z.with_fcs(mpdu)
    words, txrep, spent = [], [], 0
    budget = int(frame_clocks(len(psdu), tc)) + 100_000
    while spent < budget and (len(words) < len(psdu) + 1 or not txrep):
        if pending:                     # feed INQ[TX] while draining OUTQ[RX]
            st = loom.fifo_status(TX)
            room = st["depth"] - st["inq"]
            loom.push(TX, pending[:room])
            pending = pending[room:]
        bench.step(4096)
        spent += 4096
        words += loom.pop_available(RX)
        txrep += loom.pop_available(TX)
    return words, txrep, line


@model_only("one chip, both ends: the two RTL scenarios cover each end")
@pytest.mark.parametrize("clock", ["45", "50"])
def test_zb868_tx_and_rx_together(backend, clock):
    """OUT0 wired to IN0, both programs at the same clock: the receiver
    reports the 15-octet frame the transmitter was given, FCS good."""
    tc = TC45 if clock == "45" else TC50
    words, txrep, line = together(backend, tc, MPDU)
    assert words == rx_words(Z.with_fcs(MPDU))
    assert txrep == [15]


@model_only("a 127-octet PSDU is 15 960 chips, 2.4 M clocks: far past the RTL budget")
def test_zb868_max_length_frame_both_ends(backend):
    """aMaxPHYPacketSize (Table 22): a 127-octet PSDU (125 octets and the
    FCS) from the transmitter, every edge exactly on the 150-clock grid and
    decoded by the model, and into the receiver on the same chip, whole."""
    mpdu = [(37 * k + 11) & 0xFF for k in range(125)]
    words, txrep, line = together(backend, TC45, mpdu)
    assert line.check(TC45, tol_clocks=0) == []
    assert [f.psdu for f in line.frames(TC45)] == [Z.with_fcs(mpdu)]
    assert words == rx_words(Z.with_fcs(mpdu))
    assert txrep == [127]


@model_only("a host that stops reading; the RTL scenario covers the push path")
def test_zb868_rx_overflow_is_reported(backend):
    """A host that reads nothing until the 15-octet frame is over: OUTQ
    keeps the first four octets (FIFO_DEPTH), the rest are dropped, and
    the report, which waits for room, says overflow (0x9000 | 15) with
    the FCS still good (the CRC saw every bit)."""
    psdu = Z.with_fcs(MPDU)
    bench = make_bench(backend)
    src = bench.add(Z.ChipSource("IN0"))
    loom = boot(backend, bench, rx_tc=TC45)
    end = src.play(Z.ppdu_chips(psdu), TC45, lead=2000)
    bench.run_until(lambda: bench.cycle > end + 4000, int(frame_clocks(15, TC45)) + 50_000,
                    every=4096)
    depth = loom.fifo_status(RX)["depth"]
    words = collect(bench, loom, lambda w: len(w) >= depth + 1, 200_000)
    assert words == psdu[:depth] + [0x9000 | 15]
