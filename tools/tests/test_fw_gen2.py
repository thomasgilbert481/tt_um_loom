"""L3 for the EPC Gen2 programs (``gen2_tag``, ``gen2_reader``), model and RTL.

The tag (ENV = IN0, the reader's demodulated envelope; BS = OUT0, the
backscatter modulator) answers Query with an RN16 and ACK with PC, EPC and
PacketCRC. The reference is ``tools.protomodels.gen2``, written from the
GS1 Gen2 standard (Release 3.0.1): ``Gen2Reader`` lays PIE frames on ENV and
records BS; ``Reply`` and ``check_reply`` measure the backscatter (BLF
against Table 6-9, every edge on its grid, T1 against Table 6-16, the
waveform exactly the preamble and the code of the decoded bits, the dummy 1)
and decode it the way a reader does.

Expected values come from the standard: BLF = DR / TRcal (eq. 1), T1 from
Table 6-16, the CRCs from Tables 6-11 and 6-12, PC from 6.3.2.1.2.2.

``gen2_reader`` (TX = OUT0, RXB = IN0) runs one inventory round against
``gen2.Gen2Tag``, a tag model that reads PIE as 6.3.1.2.8 tells a tag to
and backscatters at T1 nominal; every reader frame is held to 6.3.1.2.4/8
and Table 6-5 (``check_frame``) and T2 is measured.

RTL time (measured): the tag's Query -> RN16 in FM0 134 k clocks, in
Miller-4 168 k, Query -> RN16 -> ACK -> EPC 214 k; the reader's FM0 round
221 k; 738 k in all. Four RTL scenarios; the sweeps and error cases run
on the model.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import gen2 as G
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
EPC = [0x3034, 0x257B, 0xF400, 0xB240, 0x0000, 0x0001]     # 96 bits, a test EPC
SEED = 0x1D2C

# (Tari, RTcal, TRcal) in seconds, DR (0 = 8, 1 = 64/3)
FAST = (12.5e-6, 37.5e-6, 50e-6, 0)            # BLF 160 kHz, RTcal 3 Tari
SLOW = (25e-6, 62.5e-6, 400e-6 / 3, 1)         # BLF 160 kHz, RTcal 2.5 Tari


def program():
    return assemble_file(FIRMWARE / "gen2_tag.loom", isa=ISA, strict=True)


def gen2_bench(backend):
    """FIFOs, the deadline latch, the bit engine (CRC) and the data memory;
    the RTL has them all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "DMEM"), imem_words=IMEM)
    return backend.bench()


class Session:
    """The tag running on thread 0 and a reader on its pins."""

    def __init__(self, backend, clock=50e6, epc=EPC, seed=SEED):
        self.clock = clock
        self.bench = gen2_bench(backend)
        self.reader = self.bench.add(G.Gen2Reader("IN0", "OUT0", clock))
        self.loom = backend.loom(self.bench, isa=ISA)
        self.loom.load(program())
        self.loom.run(0)
        self.loom.push(0, [seed, len(epc)] + list(epc))
        self.at = self.bench.cycle + 3000          # configuration is done by then
        self.pw = None                           # Gen2Reader default PW

    def frame(self, bits, link, preamble, m, trext, nbits):
        """Send one frame and run past the end of the longest reply it can
        draw (``nbits`` with the dummy). Returns the frame's last rising
        edge and every BS edge after it; the next frame goes 10 Tpri later."""
        tari, rtcal, trcal, dr = link
        last = self.reader.send(bits, self.at, tari, rtcal, pw=self.pw,
                                trcal=trcal if preamble else None)
        tpri = 1.0 / G.blf_hz(dr, trcal)
        k = 1 if m == 0 else 1 << m
        halves = G.preamble_halves(m, trext) + 2 * k * nbits + 2
        span = G.t1_window_s(dr, rtcal, trcal)[1] + halves * tpri / 2 * 1.25
        self.bench.step(last + int(span * self.clock) - self.bench.cycle)
        self.at = self.bench.cycle + int(10 * tpri * self.clock)
        return last, self.reader.edges_after(last)

    def query(self, link, m=0, trext=0, q=0, bad_crc=False):
        bits = G.query_bits(dr=link[3], m=m, trext=trext, q=q, bad_crc=bad_crc)
        return self.frame(bits, link, True, m, trext, 17)

    def ack(self, rn16, link, m=0, trext=0, words=len(EPC)):
        return self.frame(G.ack_bits(rn16), link, False, m, trext,
                          16 * (words + 2) + 1)


def check(s, last, edges, link, m, trext, nbits, expected=None):
    """Hold a reply to the standard (``gen2.check_reply``); its bits."""
    tari, rtcal, trcal, dr = link
    assert edges, "no reply"
    half = s.clock * trcal / (2 * (64 / 3 if dr else 8))
    reply = G.Reply(edges, half)
    bits, errs = G.check_reply(reply, expected, m, trext, nbits, dr, rtcal,
                               trcal, last, s.clock)
    assert errs == [], errs
    return bits


def inventory(s, link, m, trext):
    """Query -> RN16 (checked), returns the RN16."""
    last, edges = s.query(link, m, trext)
    bits = check(s, last, edges, link, m, trext, 17)
    return G.bits_word(bits[:16])


def read_epc(s, rn16, link, m, trext):
    """ACK -> PC, EPC, PacketCRC (checked as a reader would: L from PC)."""
    last, edges = s.ack(rn16, link, m, trext)
    tari, rtcal, trcal, dr = link
    half = s.clock * trcal / (2 * (64 / 3 if dr else 8))
    reply = G.Reply(edges, half)
    k = 1 if m == 0 else 1 << m
    head = G.decode_halves(reply.halves(G.preamble_halves(m, trext) + 2 * k * 17),
                           m, trext, 16)
    words = G.bits_word(head) >> 11
    nbits = 16 * (words + 2) + 1
    bits = check(s, last, edges, link, m, trext, nbits)
    out = [G.bits_word(bits[i:i + 16]) for i in range(0, nbits - 1, 16)]
    assert G.crc16_bits(bits[:-1]) == 0x1D0F            # Table 6-11 residue
    return out


# ------------------------------------------------------------- the model
def test_gen2_model_crcs_and_figures():
    """The model against the standard's own checks: a Query with its CRC-5
    leaves residue 00000 (Annex F.1), a message with its PacketCRC leaves
    1D0Fh (Table 6-11); the FM0 preamble is Figure 6-11's; a Miller reply
    keeps its subcarrier through the pilot (Figure 6-15); a flipped bit is
    caught."""
    q = G.query_bits(dr=1, m=2, trext=1, q=4)
    assert len(q) == 22 and G.crc5(q) == 0
    assert G.crc5(G.query_bits(bad_crc=True)) != 0
    words = G.epc_reply_words(EPC)
    assert words[0] == 6 << 11
    bits = [b for w in words for b in G.word_bits(w)]
    assert G.crc16_bits(bits) == 0x1D0F
    assert G.reply_halves([1], 0, 0)[:12] == [1, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1]
    pre = G.reply_halves([1], 2, 0)
    assert pre[:32] == [1, 0] * 16                      # 4 M / BLF of pilot
    assert G.decode_halves(G.reply_halves([0, 1, 1, 0, 1], 2, 1), 2, 1, 5) == [0, 1, 1, 0, 1]
    assert G.decode_halves(G.reply_halves([0, 1, 1, 0, 1], 0, 0), 0, 0, 5) == [0, 1, 1, 0, 1]
    assert G.frt(0, 50e-6) == 0.07 and G.frt(1, 400e-6 / 3) == 0.07
    assert G.frt(0, 300e-6) is None


# --------------------------------------------------------------- the tag
def test_gen2_query_rn16_fm0(backend):
    """Query (preamble, Tari 12.5 us, RTcal 3 Tari, TRcal 50 us, DR = 8:
    BLF 160 kHz) -> RN16 in FM0, TRext 0; held to Tables 6-9 and 6-16."""
    s = Session(backend)
    inventory(s, FAST, 0, 0)
    assert s.loom.badop() == 0


def test_gen2_query_rn16_miller4(backend):
    """The same Query asking for Miller-4 (M = 4, TRext 0)."""
    s = Session(backend)
    inventory(s, FAST, 2, 0)


def test_gen2_ack_epc(backend):
    """Query -> RN16, ACK with that RN16 (frame-sync) -> PC + 96-bit EPC +
    PacketCRC in FM0: the reader reads L from PC, the EPC, the CRC."""
    s = Session(backend)
    rn = inventory(s, FAST, 0, 0)
    assert read_epc(s, rn, FAST, 0, 0) == G.epc_reply_words(EPC)


# (Tari, RTcal, TRcal, DR): the limits of 6.3.1.2.4 and eq. (2)
LINKS = {
    "fast": FAST,
    "slow": SLOW,
    "tari6": (6.25e-6, 15.625e-6, 46.875e-6, 0),     # Tari min, TRcal 3 RTcal: 171 kHz
    "blf40": (25e-6, 75e-6, 200e-6, 0),             # Tari max, data-1 2 Tari: 40 kHz
    "tr11": (25e-6, 75e-6, 82.5e-6, 0),             # TRcal 1.1 RTcal: 97 kHz
}


@model_only("the link sweep: each case repeats what the three RTL cases prove")
@pytest.mark.parametrize("link,m,trext,clock,pw", [
    ("slow", 0, 0, 50e6, None), ("slow", 2, 1, 50e6, None),
    ("fast", 0, 1, 50e6, None), ("fast", 2, 1, 50e6, None),
    ("fast", 1, 0, 50e6, None), ("fast", 3, 1, 50e6, None),
    ("tari6", 0, 0, 50e6, "min"), ("tari6", 2, 1, 50e6, "max"),
    ("blf40", 0, 0, 50e6, "max"), ("blf40", 1, 1, 50e6, "min"),
    ("tr11", 1, 0, 50e6, None), ("slow", 2, 0, 45e6, "min"),
    ("fast", 0, 1, 45e6, "max"), ("fast", 3, 0, 45e6, None),
])
def test_gen2_link_sweep(backend, link, m, trext, clock, pw):
    """Query -> RN16 -> ACK -> EPC over Tari 6.25 to 25 us, RTcal 2.5 and 3
    Tari, TRcal 1.1 to 3 RTcal, DR 8 and 64/3, BLF 40 to 171 kHz, FM0 and
    M = 2, 4, 8, TRext 0 and 1, PW at Table 6-5's limits, 45 and 50 MHz;
    every reply held to the standard."""
    lk = LINKS[link]
    s = Session(backend, clock=clock)
    if pw == "min":
        s.pw = max(0.265 * lk[0], 2e-6) * 1.01
    elif pw == "max":
        s.pw = 0.525 * lk[0] * 0.99
    rn = inventory(s, lk, m, trext)
    assert read_epc(s, rn, lk, m, trext) == G.epc_reply_words(EPC)
    assert s.loom.badop() == 0


@model_only("a silent case: nothing on the pins to compare")
def test_gen2_bad_crc5_query_ignored(backend):
    """A Query whose CRC-5 is wrong is invalid (6.3.2.12.2.1): no reply;
    the next good Query is answered."""
    s = Session(backend)
    _, edges = s.query(FAST, bad_crc=True)
    assert edges == []
    inventory(s, FAST, 0, 0)


@model_only("a silent case")
def test_gen2_ack_wrong_rn16_ignored(backend):
    """An ACK with the wrong RN16 gets no reply and sends the tag to
    arbitrate (6.3.2.12.2.6), so even the right RN16 is then ignored; a new
    Query starts over and its ACK is answered."""
    s = Session(backend)
    rn = inventory(s, FAST, 0, 0)
    _, edges = s.ack(rn ^ 0x0100, FAST)
    assert edges == []
    _, edges = s.ack(rn, FAST)
    assert edges == []
    rn = inventory(s, FAST, 0, 0)
    assert read_epc(s, rn, FAST, 0, 0) == G.epc_reply_words(EPC)


@model_only("a silent case")
def test_gen2_ack_before_any_query_ignored(backend):
    s = Session(backend)
    _, edges = s.ack(SEED, FAST)
    assert edges == []


@model_only("a refused rate")
def test_gen2_blf_640k_refused(backend):
    """TRcal 33.3 us with DR = 64/3 asks for 640 kHz (Table 6-9's top row):
    a half cycle of 39 clocks, under the program's 128; the tag stays
    silent rather than answer slow, and answers the next Query it can."""
    s = Session(backend)
    _, edges = s.query((6.25e-6, 15.625e-6, 100e-6 / 3, 1))
    assert edges == []
    inventory(s, FAST, 0, 0)


@model_only("a silent case")
def test_gen2_query_q15_silent(backend):
    """Q = 15: the slot is drawn from 0 .. 32767, so the tag is all but
    certainly not in slot 0 and stays silent (no QueryRep is implemented)."""
    s = Session(backend)
    _, edges = s.query(FAST, q=15)
    assert edges == []


def _edges(halves, t0, half):
    out, lvl = [], 0
    for k, v in enumerate(halves + [0]):
        if v != lvl:
            out.append((int(round(t0 + k * half)), v))
            lvl = v
    return out


def test_gen2_model_flags_a_bad_reply():
    """The reply checks are not a rubber stamp: a reply made by the model's
    own coder passes; 10 % off BLF (Table 6-9 allows 7 % at TRcal 50 us),
    late by 10 us (T1), or with one half inverted, each is reported."""
    tari, rtcal, trcal, dr = FAST
    clock, last = 50e6, 1000
    half = clock * trcal / 16
    bits = G.word_bits(0xBEEF) + [1]
    halves = G.reply_halves(bits, 2, 1)
    t0 = last + int(62.5e-6 * clock)

    def errs(h, start, hh):
        r = G.Reply(_edges(h, start, hh), half)
        return G.check_reply(r, None, 2, 1, 17, dr, rtcal, trcal, last, clock)[1]

    assert errs(halves, t0, half) == []
    assert any("BLF" in e for e in errs(halves, t0, half * 1.1))
    assert any("T1" in e for e in errs(halves, t0 + 500, half))
    bad = list(halves)
    bad[100] ^= 1
    assert errs(bad, t0, half) != []


# ------------------------------------------------------------- the reader
def reader_round(backend, clock=50e6, tari=12.5e-6, trcal=50e-6, dr=0, m=0,
                 trext=0, q=0, rn16=0xC0DE, corrupt=False):
    """``gen2_reader`` on thread 0 against the model tag: one inventory
    round. Returns the reported words, the tag model and the link."""
    bench = gen2_bench(backend)
    tag = bench.add(G.Gen2Tag("OUT0", "IN0", clock, EPC, rn16=rn16))
    if corrupt:
        tag.epc_crc_flip = True
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(FIRMWARE / "gen2_reader.loom", isa=ISA, strict=True))
    loom.run(0)
    tick = clock * tari / 4
    ti = int(tick)
    tf = int(round((tick - ti) * 256))
    d, t = int(round(12.5e-6 / (tari / 4))), int(round(trcal / (tari / 4)))
    half = clock * trcal / (2 * (64 / 3 if dr else 8))
    rtcal = 3 * tari
    tmo = int(G.t1_window_s(dr, rtcal, trcal)[1] * clock) + 200
    q13 = (dr << 12) | (m << 10) | (trext << 9) | q
    loom.push(0, [ti, tf, (t << 8) | d, q13, int(half), tmo, 1])
    k = 1 if m == 0 else 1 << m
    halves = 2 * G.preamble_halves(m, trext) + 2 * k * (17 + 16 * (len(EPC) + 2) + 1)
    budget = int((2 * 30 * tari + 2 * trcal + 4 * 70e-6) * clock + halves * half * 1.1) + 60000
    bench.run_until(lambda: loom.fifo_status(0)["outq"] > 0, budget, every=2000)
    (status,) = loom.pop(0, 1)
    words = []
    if status & 1 == 0:
        words = loom.pop(0, 1)
        if status & 2 == 0:
            words += loom.pop(0, len(EPC) + 2)
    assert loom.badop() == 0
    for f in tag.frames:
        assert G.check_frame(f, clock) == [], G.check_frame(f, clock)
    return status, words, tag, (tari, rtcal, trcal, dr)


def check_t2(tag, link):
    """T2 (Table 6-16): 3 to 20 Tpri from the reply's end to the ACK."""
    tpri = 1.0 / G.blf_hz(link[3], link[2])
    assert tag.t2 and all(3 * tpri <= x for x in tag.t2), tag.t2
    return [x / tpri for x in tag.t2]


def test_gen2_reader_fm0(backend):
    """The reader at Tari 12.5 us (tick Tari / 4), TRcal 50 us, DR = 8, FM0:
    every frame held to 6.3.1.2.4/8 and Table 6-5, T2 inside 3..20 Tpri,
    the model tag's RN16 and PC + EPC + PacketCRC reported, status 0."""
    status, words, tag, link = reader_round(backend)
    assert status == 0
    assert words == [0xC0DE] + G.epc_reply_words(EPC)
    assert all(x <= 20 for x in check_t2(tag, link))


@model_only("the reader's sweep: Miller replies are 4 to 8 times longer")
@pytest.mark.parametrize("clock,tari,trcal,dr,m,trext", [
    (50e6, 12.5e-6, 50e-6, 0, 2, 0), (50e6, 12.5e-6, 50e-6, 0, 0, 1),
    (50e6, 12.5e-6, 50e-6, 0, 1, 1), (50e6, 25e-6, 100e-6, 0, 0, 0),
    (50e6, 25e-6, 100e-6, 0, 2, 1), (45e6, 12.5e-6, 50e-6, 0, 0, 0),
    (45e6, 12.5e-6, 50e-6, 0, 2, 1),
])
def test_gen2_reader_sweep(backend, clock, tari, trcal, dr, m, trext):
    """Tari 12.5 and 25 us, BLF 160 and 80 kHz, FM0 and Miller-2/4, TRext 0
    and 1, 45 and 50 MHz: frames to the standard, the words right; T2 is
    measured (at least 3 Tpri; its maximum is reported in PROGRESS.md)."""
    status, words, tag, link = reader_round(backend, clock=clock, tari=tari,
                                            trcal=trcal, dr=dr, m=m, trext=trext)
    assert status == 0
    assert words == [0xC0DE] + G.epc_reply_words(EPC)
    check_t2(tag, link)


@model_only("a silent tag")
def test_gen2_reader_no_reply(backend):
    """Q = 1: the model tag stays silent (it only answers slot 0 of Q = 0),
    the reader reports STATUS 3 after T1's maximum and nothing else."""
    status, words, tag, _ = reader_round(backend, q=1)
    assert status == 3 and words == [] and tag.replies == []


@model_only("an injected error")
def test_gen2_reader_bad_crc16(backend):
    """A damaged PacketCRC: STATUS 4, the words still reported."""
    status, words, tag, _ = reader_round(backend, corrupt=True)
    assert status == 4
    good = G.epc_reply_words(EPC)
    assert words == [0xC0DE] + good[:-1] + [good[-1] ^ 1]
