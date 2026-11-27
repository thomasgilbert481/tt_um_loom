"""L3 for the ISO/IEC 14443 Type A programs (``nfca_card``, ``nfca_reader``),
model and RTL.

The card (ENV = IN0, the reader's field, low = a pause; LM = OUT0, its load
switch) is driven by ``iso14443a.Pcd``, a model reader that lays modified
Miller pauses on ENV and decodes the load line as a reader does
(``decode_manchester``: the subcarrier's edges counted per half bit, every
edge held to the fc/16 grid, the first edge to the FDT window).

The reader (TX = OUT0, its field; RX = IN0, the cards' load modulation)
runs against ``iso14443a.CardField``: model cards whose load lines are ORed
on RX, as load modulation adds on the reader's antenna, each answering as
the standard says. Every frame the reader sends is decoded and held to t1
and the half-bit grid (``decode_miller``).

Expected values come from the sources of ``tools/protomodels/iso14443a.py``:
frames and NVB from [PM3], CRC_A from [NFC], timings from [RC522] Table 4
and the FDTs (1172/fc, 1236/fc).

RTL time (measured 2026-11-27): the card's REQA -> SAK 174 k clocks, the
reader with no card 78 k, one card at 45 MHz 169 k, two colliding cards
241 k: 662 k in all, four RTL scenarios; the 45 MHz card, the late split,
three cards, the error reports and the FDT-window ends run on the model.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import iso14443a as A
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
UID = [0x8E, 0x4F, 0x12, 0xB7]          # a test UID
ATQA, SAK = 0x0004, 0x08


def program(name):
    return assemble_file(FIRMWARE / ("%s.loom" % name), isa=ISA, strict=True)


def nfca_bench(backend):
    """FIFOs, the deadline latch and the data memory; the RTL has them all.
    Both programs are over 400 words, so the RTL loads them without the
    read-back (``verify=False`` halves the load, about 70 k clocks saved
    each; the RTL testbench stops on a read of a word never written)."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=IMEM)
    return backend.bench()


def card_params(clock, uid=UID, atqa=ATQA, sak=SAK):
    """nfca_card's host words (its header) for ``clock``: TICK_INT and
    TICK_FRAC of h = 8/fc, T = 128/fc, the FDTs 1236/fc and 1172/fc in
    clocks, the UID, ATQA, SAK."""
    h256 = int(round(clock * 8 / A.FC * 256))
    return [h256 >> 8, h256 & 0xFF, int(round(clock * A.BIT_FC / A.FC)),
            int(round(clock * A.FDT_FC[1] / A.FC)),
            int(round(clock * A.FDT_FC[0] / A.FC)),
            uid[0] | uid[1] << 8, uid[2] | uid[3] << 8, atqa, sak]


class CardSession:
    """nfca_card on thread 0 and a model reader on its pins."""

    def __init__(self, backend, clock=50e6, uid=UID):
        self.clock = clock
        self.bench = nfca_bench(backend)
        self.pcd = self.bench.add(A.Pcd("IN0", "OUT0", clock))
        self.loom = backend.loom(self.bench, isa=ISA)
        self.loom.load(program("nfca_card"), verify=backend.name == "model")
        self.loom.run(0)
        self.loom.push(0, card_params(clock, uid))
        self.at = self.bench.cycle + 3000

    def frame(self, bits, answer_bits):
        """Send ``bits``, run past the longest answer of ``answer_bits``;
        returns (answer bits, violations, FDT in clocks)."""
        last = self.pcd.send(bits, self.at)
        t = self.pcd.bit_clocks()
        span = A.FDT_FC[1] * self.clock / A.FC + (answer_bits + 3) * t
        self.bench.step(last + int(span) - self.bench.cycle)
        self.at = self.bench.cycle + int(2 * t)
        return self.pcd.answer()


def card_to_sak(s, uid=UID):
    """REQA -> ATQA, ANTICOLLISION -> UID + BCC, SELECT -> SAK + CRC_A, each
    answer held to the standard; returns the three answers' bytes."""
    bits, errs, _ = s.frame(A.short_bits(A.REQA), 18)
    assert errs == [] and bits == A.std_bits([ATQA & 0xFF, ATQA >> 8]), (bits, errs)
    bits, errs, _ = s.frame(A.anticoll_bits([0] * 5, 0), 45)
    assert errs == [] and bits == A.uid_reply_bits(uid, 0), (bits, errs)
    ub, perr = A.bits_to_bytes(bits)
    assert perr == [] and ub == A.uid_bytes(uid) and ub[4] == A.bcc(uid)
    bits, errs, _ = s.frame(A.std_bits(A.select_bytes(uid)), 27)
    assert errs == [] and bits == A.std_bits(A.with_crc([SAK])), (bits, errs)
    sak, perr = A.bits_to_bytes(bits)
    assert perr == [] and A.crc_a(sak) == 0
    return sak


# ------------------------------------------------------------- the model
def test_nfca_model_crc_and_frames():
    """CRC_A of 00 00 is A0 1E and of 12 34 is 26 CF (worked from [NFC]'s
    update); a frame with its CRC_A leaves 0; NVB per [PM3]; the Miller and
    Manchester codings decode back."""
    assert A.with_crc([0x00, 0x00])[2:] == [0xA0, 0x1E]
    assert A.with_crc([0x12, 0x34])[2:] == [0x26, 0xCF]
    assert A.crc_a(A.select_bytes(UID)) == 0
    assert A.nvb(0) == 0x20 and A.nvb(40) == 0x70 and A.nvb(11) == 0x33
    assert A.miller_sequence([0, 0, 1, 0, 1, 1]) == "ZZZXYXXYY"
    for bits in (A.short_bits(A.REQA), A.std_bits(A.select_bytes(UID))):
        pauses, _ = A.miller_pauses(bits, 1000, 50e6)
        got, errs = A.decode_miller(pauses, 50e6)
        assert errs == [] and got == bits
        ch = A.manchester_changes(bits, 1000, 45e6)
        got, coll, errs, t0 = A.decode_manchester(ch, 45e6)
        assert errs == [] and coll == [] and got == bits and t0 == 1000
    late = [(f + 9, r + 9) if i == 3 else (f, r) for i, (f, r) in enumerate(pauses)]
    assert A.decode_miller(late, 50e6)[1]


def test_nfca_card_reqa_to_sak(backend):
    """The card at 50 MHz answers REQA, ANTICOLLISION (NVB 20h) and SELECT,
    each on the fc/16 grid and inside its FDT window."""
    s = CardSession(backend, 50e6)
    assert card_to_sak(s) == A.with_crc([SAK])


@model_only("the same exchange at 45 MHz (TICK_FRAC 140/256): 50 MHz runs on the RTL")
def test_nfca_card_reqa_to_sak_45mhz(backend):
    """The same at 45 MHz, where h is 26.55 clocks: the E-to-D step has its
    least slack (10 clocks) here."""
    s = CardSession(backend, 45e6)
    assert card_to_sak(s) == A.with_crc([SAK])


# ------------------------------------------------------------ the reader
def reader_params(clock):
    """nfca_reader's host words (its header): TICK_INT and TICK_FRAC of
    T/4 = 32/fc, T = 128/fc, T/2, and TMO = 1236/fc + 2 bits (the latest
    FDT and the end of the reader's own frame)."""
    q256 = int(round(clock * 32 / A.FC * 256))
    t = clock * A.BIT_FC / A.FC
    return [q256 >> 8, q256 & 0xFF, int(round(t)), int(round(t / 2)),
            int(round(clock * A.FDT_FC[1] / A.FC + 2 * t))]


def expected_pick(uids):
    """The card a reader that takes a 1 at every collision ends with [PM3]:
    at the first bit (UID0 bit 0 first) where the cards still in differ,
    keep those with a 1."""
    left = [A.uid_bytes(u) for u in uids]
    for j in range(40):
        vals = {(ub[j // 8] >> (j % 8)) & 1 for ub in left}
        if len(vals) == 2:
            left = [ub for ub in left if (ub[j // 8] >> (j % 8)) & 1]
    assert len(left) == 1
    return left[0][:4]


def splits(uids):
    """How many collisions that reader meets: one per split."""
    left, n = [A.uid_bytes(u) for u in uids], 0
    for j in range(40):
        if len({(ub[j // 8] >> (j % 8)) & 1 for ub in left}) == 2:
            n += 1
            left = [ub for ub in left if (ub[j // 8] >> (j % 8)) & 1]
    return n


class ReaderSession:
    """nfca_reader on thread 0 and model cards in its field."""

    def __init__(self, backend, cards, clock=50e6):
        self.clock = clock
        self.bench = nfca_bench(backend)
        self.field = self.bench.add(A.CardField("OUT0", "IN0", clock, cards))
        self.loom = backend.loom(self.bench, isa=ISA)
        self.loom.load(program("nfca_reader"), verify=backend.name == "model")
        self.loom.run(0)
        self.loom.push(0, reader_params(clock))

    def activate(self, polls=1500):
        self.loom.push(0, [1])
        return self.loom.pop(0, 5, max_polls=polls)

    def frames_clean(self):
        """Every frame the reader sent, held to t1 and the half-bit grid."""
        for pauses, bits, errs in self.field.frames:
            assert errs == [], (bits, errs)
        return [bits for _, bits, _ in self.field.frames]


def run_reader(backend, uids, clock=50e6, polls=1500, **card_kw):
    """Cards answer 3 clocks apart (two cards' FDTs never agree exactly)."""
    cards = [A.Picc(u, atqa=ATQA, sak=SAK, offset=3 * n, **card_kw)
             for n, u in enumerate(uids)]
    s = ReaderSession(backend, cards, clock)
    return s, s.activate(polls)


def check_selected(s, rep, uids):
    pick = expected_pick(uids)
    assert rep == [0, ATQA, pick[0] | pick[1] << 8, pick[2] | pick[3] << 8, SAK], rep
    frames = s.frames_clean()
    assert frames[0] == A.short_bits(A.REQA)
    assert len(frames) == 1 + 1 + splits(uids) + 1      # REQA, rounds, SELECT
    assert frames[-1] == A.std_bits(A.select_bytes(pick))
    assert [c.state for c in s.field.cards].count(A.ACTIVE) == 1


def test_nfca_reader_one_card_45mhz(backend):
    """One card at 45 MHz: REQA, one ANTICOLLISION (NVB 20h), SELECT, the
    UID and SAK reported."""
    uids = [UID]
    s, rep = run_reader(backend, uids, 45e6)
    check_selected(s, rep, uids)


@model_only("one card at 50 MHz: the RTL runs 45 MHz and two cards at 50")
def test_nfca_reader_one_card_50mhz(backend):
    """The same at 50 MHz."""
    uids = [UID]
    s, rep = run_reader(backend, uids, 50e6)
    check_selected(s, rep, uids)


def flip(uid, byte, bit):
    u = list(uid)
    u[byte] ^= 1 << bit
    return u


TWO_EARLY = [UID, flip(UID, 0, 0)]                 # differ in bit 0
TWO_LATE = [flip(UID, 3, 7), UID]                  # differ in bit 31 (and BCC)
THREE = [flip(UID, 1, 5), UID, flip(UID, 3, 2)]    # bits 13 and 26


def test_nfca_reader_two_cards_collide(backend, which="early"):
    """Two cards answer at once; their load modulation adds on RX, so the
    bit where they differ carries the subcarrier in both halves. The reader
    sees that collision, splits on it with a 1, selects the card with the 1
    and reports its UID; the other stays silent from then on."""
    uids = TWO_EARLY if which == "early" else TWO_LATE
    s, rep = run_reader(backend, uids)
    check_selected(s, rep, uids)


@model_only("the late split: the early one runs on the RTL")
def test_nfca_reader_two_cards_collide_late(backend):
    """The same, the cards differing in bit 31 (UID3's last) and in BCC."""
    test_nfca_reader_two_cards_collide(backend, "late")


@model_only("three cards, three ANTICOLLISION rounds: the two-card cases run on the RTL")
@pytest.mark.parametrize("clock", [50e6, 45e6])
def test_nfca_reader_three_cards(backend, clock):
    """Three cards that differ in bits 13 and 26: two splits, then SELECT."""
    s, rep = run_reader(backend, THREE, clock)
    check_selected(s, rep, THREE)


@model_only("the error reports: the RTL runs the good paths")
def test_nfca_reader_bad_bcc_and_crc(backend):
    """A card with a wrong BCC is reported with STATUS 3 and no SELECT is
    sent; a SAK with a wrong CRC_A is reported with STATUS 4."""
    s, rep = run_reader(backend, [UID], bad_bcc=True)
    assert rep[0] == 3, rep
    assert len(s.frames_clean()) == 2                   # REQA, ANTICOLLISION
    s, rep = run_reader(backend, [UID], bad_crc=True)
    assert rep[0] == 4 and rep[4] == SAK, rep


def test_nfca_reader_no_card(backend):
    """No card in the field: STATUS 1, nothing else sent."""
    s, rep = run_reader(backend, [], polls=200)
    assert rep[0] == 1, rep
    assert s.frames_clean() == [A.short_bits(A.REQA)]


@model_only("the prefix rules: the RTL runs REQA to SAK")
def test_nfca_card_prefix_split_and_silence(backend):
    """WUPA wakes the card; an ANTICOLLISION whose 11-bit prefix (NVB 33h)
    is its own gets the remaining 29 UID and BCC bits, the split byte's
    parity included; one bit off and it stays silent; a SELECT of another
    UID, or with a wrong CRC_A, gets nothing; its own SELECT gets SAK."""
    s = CardSession(backend, 50e6)
    ub = A.uid_bytes(UID)
    bits, errs, _ = s.frame(A.short_bits(A.WUPA), 18)
    assert errs == [] and bits == A.std_bits([ATQA & 0xFF, ATQA >> 8])
    bits, errs, _ = s.frame(A.anticoll_bits(ub, 11), 45)
    assert errs == [] and bits == A.uid_reply_bits(UID, 11), (bits, errs)
    wrong = list(ub)
    wrong[1] ^= 0x04                                    # bit 10
    bits, errs, _ = s.frame(A.anticoll_bits(wrong, 11), 45)
    assert errs == ["no answer"], (bits, errs)
    other = A.std_bits(A.select_bytes(flip(UID, 2, 0)))
    bits, errs, _ = s.frame(other, 27)
    assert errs == ["no answer"], (bits, errs)
    bad = A.select_bytes(UID)
    bad[-1] ^= 0x10
    bits, errs, _ = s.frame(A.std_bits(bad), 27)
    assert errs == ["no answer"], (bits, errs)
    bits, errs, _ = s.frame(A.std_bits(A.select_bytes(UID)), 27)
    assert errs == [] and bits == A.std_bits(A.with_crc([SAK]))
    bits, errs, _ = s.frame(A.short_bits(A.REQA), 18)
    assert errs == ["no answer"]                        # ACTIVE: silent



@model_only("the reader's margins: answers at both ends of the FDT window")
@pytest.mark.parametrize("clock", [50e6, 45e6])
def test_nfca_reader_fdt_window_ends(backend, clock):
    """Cards may answer anywhere in FDT + 0.4 us (from memory, unchecked):
    one card at the window's start and one at its end, and two cards that
    collide answering at opposite ends, are all read and selected."""
    end = int(A.FDT_TOL_S * clock)
    for offsets, uids in (([0], [UID]), ([end], [UID]), ([0, end], TWO_LATE)):
        cards = [A.Picc(u, atqa=ATQA, sak=SAK, offset=o)
                 for u, o in zip(uids, offsets)]
        s = ReaderSession(backend, cards, clock)
        check_selected(s, s.activate(), uids)
