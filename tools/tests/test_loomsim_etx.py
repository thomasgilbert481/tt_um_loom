"""The Ethernet serializer on the golden model: ``docs/SEMANTICS.md`` 6.12
(D-036), feature ``"ETX"``, and the reference receiver of
``tools/protomodels/ethernet.py``.

Expected values come from the text, never from the model: the bit sequence
of 6.12.3 is rebuilt below with ``zlib.crc32`` for the FCS, the pad trace of
6.12.6/6.12.7 from it (half bit ``i`` from edge ``s + i N``, the tail of
``N * IDL`` clocks of ``(1, 0)``, then ``PIN_OUT``), and the window edges
``K_m`` from 6.12.6.  The vectors are SPEC section C (W-115 ``SPEC.md``)
C1 to C12.

Most scenarios place every ``CSRW`` on an exact commit edge by **stepping**
(SEMANTICS 7): a ``STEP`` of thread ``t`` written in cycle ``c`` makes the
slot whose F cycle is ``c + 1`` valid (when ``c + 1 = t mod 4``), so its X
cycle is ``c + 3`` and its commit edge ``c + 5``.  A write at edge ``e``
therefore comes from thread ``e mod 4``, stepped in cycle ``e - 5``.  Two
writers in commit order are allowed by 6.12.7 and only one commits an edge.
"""

import random
import zlib

import pytest

from tools.loomisa import load
from tools.loomsim import Machine
from tools.loomsim.harness import PadTrace, assemble
from tools.protomodels import ethernet as rx

ISA = load()
CSR = ISA.csr_by_name
CFG, DATA, LAST, LAST1, STAT = (CSR[n] for n in (
    "ETX_CFG", "ETX_DATA", "ETX_LAST", "ETX_LAST1", "ETX_STAT"))
ETX = ("ETX",)
FULL = ("FIFO", "SETPD", "BE", "BEENC", "DMEM", "ETX")
BUSY, RDY, FDONE, UNDER, REF = 1, 2, 4, 8, 16


def cfg(halfn=1, pre=1, fcs=1, idl=6):
    """``ETX_CFG`` = ``{IDL[3:0], FCS, PRE, HALFN[3:0]}`` (6.12.2)."""
    return ((idl & 0xF) << 6) | ((fcs & 1) << 5) | ((pre & 1) << 4) | (halfn & 0xF)


# ------------------------------------------------- reference from 6.12.3
def frame_bytes(words, one_byte_last=False):
    """The bytes of a frame of words, two a word, low byte first (6.12.2)."""
    out = bytearray()
    for i, w in enumerate(words):
        out.append(w & 0xFF)
        if not (one_byte_last and i == len(words) - 1):
            out.append((w >> 8) & 0xFF)
    return bytes(out)


def frame_bits(words, one_byte_last=False, pre=True, fcs=True):
    """Every bit 6.12.3 sends, in order; the FCS from ``zlib.crc32``."""
    data = frame_bytes(words, one_byte_last)
    bits = []
    if pre:
        bits += [1 if (k % 2 == 0 or k == 63) else 0 for k in range(64)]
    bits += rx.bytes_to_bits(data)
    if fcs:
        crc = zlib.crc32(data)
        bits += [(crc >> j) & 1 for j in range(32)]
    return bits


def line(bits, n, idl):
    """``(TD+, TD-)`` per cycle from ``s``: Manchester, then the tail (6.12.7)."""
    out = []
    for v in bits:
        out += [((~v) & 1, v)] * n + [(v, (~v) & 1)] * n
    return out + [(1, 0)] * (n * idl)


# ------------------------------------------------------------- drivers
class Stepper:
    """Schedules single instructions on exact X cycles by host ``STEP``.

    ``at(x, name, ops, reg=None)`` puts ``name`` in the X cycle ``x`` (thread
    ``(x - 2) mod 4``); ``write(e, csr, value)`` is a ``CSRW`` committing at
    edge ``e``.  Each thread's program is its instructions in time order; the
    register a ``CSRW`` reads is preloaded by the host before cycle 0's edge.
    """

    def __init__(self, features=ETX, setup=None, **kwargs):
        self.features = features
        self.kwargs = kwargs
        self.setup = setup
        self.plan = []                         # (x, thread, name, ops, regs)
        self.records = {}                      # x cycle -> RetireRecord

    def at(self, x, name, ops=None, regs=None):
        assert x >= 3
        self.plan.append((x, (x - 2) % 4, name, dict(ops or {}), dict(regs or {})))
        return self

    def write(self, edge, csr, value):
        thread = edge % 4
        used = [r for (_, t, _, _, regs) in self.plan if t == thread for r in regs]
        reg = len(set(used))
        same = [r for (_, t, _, _, regs) in self.plan if t == thread
                for r, v in regs.items() if v == value]
        if same:
            reg = same[0]
        assert reg < 8, "more than 8 distinct values on thread %d" % thread
        return self.at(edge - 2, "CSRW", dict(csr=csr, ra=reg), {reg: value})

    def build(self):
        image = {}
        per_thread = {t: [] for t in range(4)}
        for item in sorted(self.plan):
            per_thread[item[1]].append(item)
        for t, items in per_thread.items():
            prog = [(name, ops) for (_, _, name, ops, _) in items] + [("HALT", {})]
            # SEMANTICS 5: RESET_PC[t] = t * (IMEM_WORDS / 4).
            image.update(assemble(ISA, prog, t * self.kwargs.get("imem_words", 1024) // 4))
        self.trace = PadTrace()
        m = Machine(image, features=self.features, isa=ISA,
                    on_cycle=self.trace, **self.kwargs)
        for t, items in per_thread.items():
            for (_, _, _, _, regs) in items:
                for r, v in regs.items():
                    m.host_write_debug(t, "r%d" % r, v)
        if self.setup is not None:
            self.setup(m)
        self.machine = m
        self.steps = {x - 3: t for (x, t, _, _, _) in self.plan}
        return m

    def run(self, cycles, each=None):
        m = self.build()
        stats = []
        for _ in range(cycles):
            if m.cycle in self.steps:
                m.host_step(self.steps[m.cycle])
            if each is not None:
                each(m)
            stats.append(m.etx.stat)
            rec = m.step_cycle()
            if rec is not None:
                self.records[rec.x_cycle] = rec
        self.stat = stats                      # ETX_STAT during each cycle
        return self

    def tdp_tdn(self):
        return self.trace.uo_bit(0), self.trace.uo_bit(1)

    def pairs(self, start, count):
        tdp, tdn = self.tdp_tdn()
        return list(zip(tdp, tdn))[start:start + count]


def window_edges(s, n, pre, m):
    """``K_m`` of 6.12.6, with every earlier word full."""
    return s + 2 * n * (64 + 16 * m) if pre else s + 32 * n * m


# ------------------------------------------------------- reset and build
def test_reset_values_caps_and_version():
    """6.12.2: CFG 0x01B1 and STAT 0x0002 after reset; 6.12.8: CAPS 0x96BA
    (today's 0x92BA with bit 10) and VERSION 5 with the serializer."""
    st = Stepper(features=FULL, imem_words=512)
    st.at(3, "CSRR", dict(rd=0, csr=CFG)).at(7, "CSRR", dict(rd=1, csr=STAT))
    st.run(20)
    assert st.records[3].val == 0x01B1
    assert st.records[7].val == 0x0002
    m = st.machine
    assert m.host_read_ctrl("CAPS") == 0x96BA
    assert m.host_read_ctrl("VERSION") == 5
    assert Machine(features=FULL[:-1], imem_words=512, isa=ISA).host_read_ctrl("CAPS") == 0x92BA
    assert Machine(features=ETX, isa=ISA).host_read_ctrl("VERSION") == 5


def test_unbuilt_reads_zero_ignores_writes_no_badop():
    """6.6 and 6.12: without ETX the CSRs read 0, writes are ignored without
    BADOP, pin_in(13)/(14) read 0 and the pads stay PIN_OUT."""
    st = Stepper(features=())
    st.write(8, DATA, 0x1234)
    st.at(11, "CSRR", dict(rd=0, csr=STAT)).at(15, "CSRR", dict(rd=1, csr=CFG))
    st.at(19, "JP", dict(pin=13, val=1, rel=5)).at(23, "JP", dict(pin=14, val=0, rel=5))
    st.run(60)
    assert st.records[11].val == 0 and st.records[15].val == 0
    assert st.records[19].next_pc == st.records[19].pc + 1      # RDY reads 0
    assert st.records[23].next_pc == st.records[23].pc + 6      # BUSY reads 0
    assert st.machine.badop == 0
    assert set(st.trace.uo_bit(0)) == {0}


def test_cfg_write_stores_bits_9_0_only_while_idle():
    """6.12.2: CFG stores bits 9:0 while idle; written while busy it is
    ignored and REF is set (C12's CFG write while busy)."""
    st = Stepper()
    st.write(8, CFG, 0xFC00 | cfg(halfn=3, pre=0, fcs=1, idl=7))
    st.at(11, "CSRR", dict(rd=0, csr=CFG))
    st.write(16, LAST, 0x00FF)                 # starts a frame at s = 16
    st.write(24, CFG, cfg())                   # busy: refused
    st.at(27, "CSRR", dict(rd=1, csr=CFG)).at(31, "CSRR", dict(rd=2, csr=STAT))
    st.run(60)
    assert st.records[11].val == cfg(halfn=3, pre=0, fcs=1, idl=7)
    assert st.records[27].val == cfg(halfn=3, pre=0, fcs=1, idl=7)
    assert st.records[31].val & (REF | BUSY) == REF | BUSY


@pytest.mark.parametrize("pin_out", [0x0000, 0x0100, 0x0200, 0x0300])
def test_idle_pads_follow_pin_out_bits(pin_out):
    """6.12.7: while not BUSY, uo_out[1:0] = PIN_OUT[9:8]; while BUSY they
    are the line, whatever PIN_OUT holds."""
    st = Stepper(setup=lambda m: m.host_write_pin_out(pin_out))
    st.write(20, LAST1, 0x00A5)
    st.write(8, CFG, cfg(pre=0, fcs=0, idl=2))
    st.run(20 + 2 * 2 * 8 + 4 + 10)
    s = 20
    exp = line(frame_bits([0xA5], True, pre=False, fcs=False), 2, 2)
    idle = ((pin_out >> 8) & 1, (pin_out >> 9) & 1)
    assert st.pairs(2, s - 2) == [idle] * (s - 2)
    assert st.pairs(s, len(exp)) == exp
    assert st.pairs(s + len(exp), 10) == [idle] * 10


def load16(rd, value):
    return [("LDI", dict(rd=rd, imm=value & 0xFF)),
            ("LDIH", dict(rd=rd, imm=(value >> 8) & 0xFF))]


def feeder(words, last_csr=LAST, config=None, stop_after=None):
    """A firmware feeder (thread 0): ``WAITP 13, 1`` before every word, as
    6.12.1 says firmware polls ``RDY``; then ``WAITP 14, 0`` and ``HALT``.
    ``stop_after`` halts after that many words without a last word."""
    prog = []
    if config is not None:
        prog += load16(2, config) + [("CSRW", dict(csr=CFG, ra=2))]
    for i, w in enumerate(words):
        if stop_after is not None and i == stop_after:
            return prog + [("HALT", {})]
        kind = last_csr if i == len(words) - 1 else DATA
        prog += load16(1, w) + [("WAITP", dict(pin=13, val=1, tmo=0)),
                                ("CSRW", dict(csr=kind, ra=1))]
    return prog + [("WAITP", dict(pin=14, val=0, tmo=0)), ("HALT", {})]


def run_feeder(prog, features=ETX, cycles=20000, each=None, setup=None):
    trace = PadTrace()
    m = Machine(assemble(ISA, prog, 0), features=features, isa=ISA, on_cycle=trace)
    if setup is not None:
        setup(m)
    m.host_set_run(1)
    records = []
    for _ in range(cycles):
        if each is not None:
            each(m)
        rec = m.step_cycle()
        if rec is not None:
            records.append(rec)
        if m.halted & 1:
            break
    else:
        raise AssertionError("feeder did not halt")
    m.run_cycles(40)
    starts = [r for r in records if r.mnemonic == "CSRW" and r.done
              and ISA.decode(r.ir)[1]["csr"] in (DATA, LAST, LAST1)]
    return m, trace, records, starts[0].x_cycle + 2


#: A 60-byte broadcast ARP request (who has 192.168.1.1, tell 192.168.1.10).
ARP = (b"\xff" * 6 + bytes.fromhex("020000000001") + b"\x08\x06"
       + bytes.fromhex("0001080006040001") + bytes.fromhex("020000000001")
       + bytes([192, 168, 1, 10]) + b"\x00" * 6 + bytes([192, 168, 1, 1]))
ARP = ARP + b"\x00" * (60 - len(ARP))
ARP_WORDS = [ARP[i] | (ARP[i + 1] << 8) for i in range(0, 60, 2)]


def test_c1_fcs_check_frame_123456789():
    """C1: PRE = 0, FCS = 1, N = 2, ``123456789`` as four ETX_DATA and one
    ETX_LAST1: the 32 bits before the tail read LSB first are 0xCBF43926,
    and the start edge folds the first data bit into CRC32 (MF-2)."""
    words = [0x3231, 0x3433, 0x3635, 0x3837, 0x0039]
    s = 20
    st = Stepper()
    st.write(8, CFG, cfg(pre=0, fcs=1))
    st.write(s, DATA, words[0])
    for m_ in range(1, 5):
        st.write(window_edges(s, 2, False, m_ - 1) + 4, LAST1 if m_ == 4 else DATA,
                 words[m_])
    st.run(s + 2 * 2 * 104 + 12 + 20)
    exp = line(frame_bits(words, True, pre=False), 2, 6)
    assert st.pairs(s, len(exp) + 5) == exp + [(0, 0)] * 5
    dec = rx.decode_manchester(rx.line_levels(*st.tdp_tdn()), 2)
    assert dec.start == s and len(dec.bits) == 72 + 32
    assert rx.lsb_first_value(dec.bits[-32:]) == 0xCBF43926 == zlib.crc32(b"123456789")
    assert rx.crc32_register(dec.bits) == rx.CRC32_RESIDUE == 0xC704DD7B
    assert rx.bits_to_bytes(dec.bits[:72]) == b"123456789"
    assert dec.tp_idl_clocks in (12, 14)
    assert st.stat[s + len(exp)] & (FDONE | BUSY | UNDER | REF) == FDONE


def test_c2_minimum_frame_through_the_receiver():
    """C2: a 60-byte ARP request, 29 ETX_DATA and one ETX_LAST, N = 2,
    PRE = 1: 2,304 clocks of Manchester from cycle s, 12 clocks of (1, 0),
    then PIN_OUT; the receiver finds the SFD, checks the residue and returns
    the 60 bytes, with a TP_IDL of 300 or 350 ns at 40 MHz."""
    m, trace, records, s = run_feeder(feeder(ARP_WORDS))
    tdp, tdn = trace.uo_bit(0), trace.uo_bit(1)
    pairs = list(zip(tdp, tdn))
    exp = line(frame_bits(ARP_WORDS), 2, 6)
    assert len(exp) == 2304 + 12
    assert pairs[s - 1] == (0, 0)
    assert pairs[s:s + len(exp)] == exp
    assert pairs[s + len(exp):] == [(0, 0)] * (len(pairs) - s - len(exp))
    frame = rx.receive_frame(tdp, tdn, half=2, clock_hz=40e6)
    assert frame.ok, frame.errors
    assert frame.payload == ARP and frame.preamble_bits == 64
    assert frame.fcs == zlib.crc32(ARP)
    assert frame.tp_idl_ns in (300.0, 350.0)
    assert m.etx.stat == FDONE | RDY and m.badop == 0


@pytest.mark.parametrize("pre,p,offset", [
    (True, 1, -1), (True, 1, 0), (True, 1, 63), (True, 1, 64),
    (True, 29, -1), (True, 29, 0), (True, 29, 63), (True, 29, 64),
    (False, 1, 1), (False, 1, 63), (False, 1, 64),
    (False, 29, -1), (False, 29, 0), (False, 29, 63), (False, 29, 64)])
def test_c3_c4_window_ends(pre, p, offset):
    """C3 (PRE = 1) and C4 (PRE = 0), N = 2: word ``p`` written at
    ``K_{p-1} + offset``.  The window is ``K_{p-1} .. K_p - 1`` (6.12.6):
    inside it the word is accepted (at ``K_{p-1}`` through the load-edge
    case, ``HV == 1`` before it) and the underrun is at ``K_{p+1}``; one edge
    before it the word is refused (``HOLD`` still full); at ``K_p`` it is the
    underrun: the tail starts there, ``UNDER``, and the write is refused."""
    s, n, idl = 20, 2, 6
    K = lambda m_: window_edges(s, n, pre, m_)
    st = Stepper()
    st.write(8, CFG, cfg(pre=int(pre), fcs=1, idl=idl))
    st.write(s, DATA, 0x0F0F)
    for m_ in range(1, p):
        st.write(K(m_ - 1) + 8, DATA, 0x5AA5)
    probe = K(p - 1) + offset
    st.write(probe, DATA, 0x1234)
    accepted = 0 <= offset < 32 * n
    under = K(p + 1) if accepted else K(p)
    release = under + n * idl
    st.run(release + 10)
    words = [0x0F0F] + [0x5AA5] * (p - 1) + ([0x1234] if accepted else [])
    exp = line(frame_bits(words, pre=pre, fcs=False), n, idl)
    assert s + len(exp) == release
    assert st.pairs(s, len(exp) + 5) == exp + [(0, 0)] * 5
    ref = 0 if accepted else REF
    assert st.stat[probe - 1] & REF == 0 and st.stat[probe] & REF == ref
    assert st.stat[under - 1] & (UNDER | BUSY) == BUSY
    assert st.stat[under] == BUSY | UNDER | ref
    assert st.stat[release - 1] == BUSY | UNDER | ref
    assert st.stat[release] == FDONE | UNDER | ref


def test_c5_underrun_with_idl_0_sets_under_and_fdone_at_the_load_edge():
    """C5 (MF-4): with IDL = 0 the underrun releases at the load edge and the
    frame ends with UNDER and FDONE both set at that edge."""
    s = 20
    st = Stepper()
    st.write(8, CFG, cfg(pre=0, fcs=1, idl=0))
    st.write(s, DATA, 0xBEEF)
    st.run(s + 80)
    exp = line(frame_bits([0xBEEF], pre=False, fcs=False), 2, 0)
    assert st.pairs(s, 70) == exp + [(0, 0)] * 6
    assert st.stat[s + 63] == BUSY | RDY
    assert st.stat[s + 64] == UNDER | FDONE


def test_c6_data_write_while_under_is_refused_then_clear_then_start():
    """C6: in P_IDLE with UNDER == 1 a data write is refused (REF, no start);
    the ETX_STAT clear, then a data write starts a frame."""
    s = 20
    st = Stepper()
    st.write(8, CFG, cfg(pre=0, fcs=0, idl=0))
    st.write(s, DATA, 0x1111)                      # underrun at s + 64
    st.write(s + 72, LAST, 0x2222)                 # refused: UNDER
    st.write(s + 80, STAT, UNDER | REF | FDONE)
    st.write(s + 88, LAST, 0x3333)                 # starts
    st.run(s + 88 + 64 + 10)
    assert st.stat[s + 72] == UNDER | FDONE | REF
    assert st.stat[s + 79] == UNDER | FDONE | REF
    assert st.stat[s + 80] == RDY
    assert st.stat[s + 88] == BUSY                 # LSEEN: RDY 0
    exp = line(frame_bits([0x3333], pre=False, fcs=False), 2, 0)
    assert st.pairs(s + 72, 16) == [(0, 0)] * 16
    assert st.pairs(s + 88, len(exp) + 2) == exp + [(0, 0)] * 2
    assert st.stat[s + 88 + 64] == FDONE | RDY


@pytest.mark.parametrize("kind", ["fdone", "under"])
def test_c7_set_wins_over_clear_at_the_same_edge(kind):
    """C7: an ETX_STAT write of 0x0004 committing at the release edge leaves
    FDONE = 1, and one of 0x0008 at an underrun edge leaves UNDER = 1; the
    same write one slot later clears the bit."""
    s = 20
    st = Stepper()
    st.write(8, CFG, cfg(pre=0, fcs=0, idl=6))
    if kind == "fdone":
        st.write(s, LAST, 0x00FF)                  # tail s + 64, release s + 76
        edge, bit = s + 76, FDONE
    else:
        st.write(s, DATA, 0x00FF)                  # underrun at s + 64
        edge, bit = s + 64, UNDER
    st.write(edge, STAT, bit)
    st.write(edge + 4, STAT, bit)
    st.run(s + 100)
    assert st.stat[edge - 1] & bit == 0
    assert st.stat[edge] & bit == bit
    assert st.stat[edge + 3] & bit == bit
    assert st.stat[edge + 4] & bit == 0


@pytest.mark.parametrize("csr", [CFG, DATA, LAST1])
def test_c8_writes_at_the_release_edge_are_refused(csr):
    """C8: a CFG write and a data write committing at the release edge are
    refused (REF; EPH is P_TAIL before it): nothing stored, no frame started.
    One slot later the same write is the idle case."""
    s = 20
    st = Stepper()
    st.write(8, CFG, cfg(pre=0, fcs=0, idl=6))
    st.write(s, LAST, 0x00FF)                      # release at s + 76
    value = cfg(halfn=0, pre=0, fcs=0, idl=1) if csr == CFG else 0x0001
    st.write(s + 76, csr, value)
    st.at(s + 79, "CSRR", dict(rd=0, csr=CFG))
    st.run(s + 120)
    assert st.stat[s + 75] == BUSY
    assert st.stat[s + 76] == FDONE | RDY | REF
    assert st.records[s + 79].val == cfg(pre=0, fcs=0, idl=6)
    assert st.pairs(s + 76, 40) == [(0, 0)] * 40


def test_refused_after_a_last_word_and_in_the_tail():
    """6.12.4: refused while LSEEN (after a last word) and in P_TAIL; the
    frame is unchanged by either."""
    s = 20
    st = Stepper()
    st.write(8, CFG, cfg(pre=0, fcs=0, idl=6))
    st.write(s, LAST, 0x8001)
    st.write(s + 4, DATA, 0x7777)                  # after the last word
    st.write(s + 68, DATA, 0x6666)                 # in P_TAIL (s+64 .. s+75)
    st.run(s + 100)
    assert st.stat[s + 3] & REF == 0 and st.stat[s + 4] & REF == REF
    exp = line(frame_bits([0x8001], pre=False, fcs=False), 2, 6)
    assert st.pairs(s, len(exp) + 4) == exp + [(0, 0)] * 4
    assert st.stat[s + 76] == FDONE | RDY | REF


@pytest.mark.parametrize("how", ["halt", "run0", "reset"])
def test_c9_feeder_stopped_mid_frame_ends_by_underrun(how):
    """C9: a feeder that executes HALT after three words, or whose RUN is
    cleared (and, for "reset", is then CTRL.RESET) after its fourth word was
    accepted: the frame ends by underrun at the next load edge, and the pads
    go back to PIN_OUT (here PIN_OUT[9:8] = 0b11) from the release.

    With RUN written in cycle 0 the feeder's slots have X cycles 6, 10, ...:
    LDI, LDIH, WAITP (RDY is 1) and the CSRW at X = 18 start at s = 20."""
    words = list(range(0x0101, 0x0109))
    prog = feeder(words, stop_after=3 if how == "halt" else None)
    s, n, idl = 20, 2, 6
    K = lambda m_: window_edges(s, n, True, m_)
    stop = K(2) + 20                       # word 3 accepted, word 4 not written
    trace = PadTrace()
    m = Machine(assemble(ISA, prog, 0), features=ETX, isa=ISA, on_cycle=trace)
    m.host_write_pin_out(0x0300)
    m.host_set_run(1)
    first = None
    for _ in range(K(5) + 40):
        if how != "halt" and m.cycle == stop:
            m.host_set_run(0)
        if how == "reset" and m.cycle == stop + 8:
            m.host_reset_thread(0)
        rec = m.step_cycle()
        if rec is not None and rec.mnemonic == "CSRW" and first is None:
            first = rec.x_cycle + 2
    assert first == s
    sent = 3 if how == "halt" else 4
    release = K(sent) + n * idl
    exp = line(frame_bits(words[:sent], fcs=False), n, idl)
    pairs = list(zip(trace.uo_bit(0), trace.uo_bit(1)))
    assert pairs[s - 1] == (1, 1) and s + len(exp) == release
    assert pairs[s:release] == exp
    assert set(pairs[release:]) == {(1, 1)}
    assert m.etx.stat == UNDER | FDONE


#: (X cycle offset from the edge, expected (STAT, IN over 13..14, JP 14 taken)).
C10_CASES = {"start-1": (RDY, 0b01, False), "start": (BUSY, 0b10, True),
             "release-1": (BUSY, 0b10, True), "release": (FDONE | RDY, 0b01, False)}


@pytest.mark.parametrize("op", ["CSRR", "IN", "JP"])
@pytest.mark.parametrize("when", sorted(C10_CASES))
def test_c10_reads_in_the_cycles_of_a_start_and_a_release(op, when):
    """C10: ``CSRR ETX_STAT``, ``IN`` with ``INGRP`` over 13..14 and ``JP 14``
    with X cycle ``x`` read the state visible in cycle ``x`` (6.12.2, 6.12.7):
    the old state at ``x = e - 1`` and the new one at ``x = e``, for the start
    edge ``s`` and the release edge ``r = s + 2 N 16 + N IDL``."""
    s = 20
    r = s + 64 + 12
    x = {"start-1": s - 1, "start": s, "release-1": r - 1, "release": r}[when]
    ingrp = (2 << 5) | 13
    st = Stepper(setup=lambda m: [m.host_write_debug(t, 0x13, ingrp) for t in range(4)])
    st.write(8, CFG, cfg(pre=0, fcs=0, idl=6))
    st.write(s, LAST, 0x0F0F)
    if op == "CSRR":
        st.at(x, "CSRR", dict(rd=0, csr=STAT))
    elif op == "IN":
        st.at(x, "IN", dict(rd=0))
    else:
        st.at(x, "JP", dict(pin=14, val=1, rel=5))
    st.run(r + 20)
    stat, pins, taken = C10_CASES[when]
    rec = st.records[x]
    if op == "CSRR":
        assert rec.val == stat
    elif op == "IN":
        assert rec.val == pins
    else:
        assert rec.next_pc == rec.pc + (6 if taken else 1)


def test_c11_pin_writes_to_16_17_during_a_frame():
    """C11: SHO with DIFF, OUT and SETP to indices 16/17 during a frame
    commit to PIN_OUT (CSRR and pin_in(16) see them) but not to the pad,
    which shows them from the cycle after the release."""
    s, r = 20, 20 + 64 + 12

    def setup(m):
        m.host_write_debug(1, 0x15, 16)                 # BE_PINS.out = 16
        m.host_write_debug(1, 0x14, 1 << 10)            # BE_CFG: DIFF, NRZ
        m.host_write_debug(1, "SR", 1)
        m.host_write_debug(1, 0x12, (2 << 5) | 16)      # OUTGRP 16..17
        m.host_write_debug(1, "r0", 0b10)
    st = Stepper(features=("BE", "BEENC", "ETX"), setup=setup)
    st.write(8, CFG, cfg(pre=0, fcs=0, idl=6))
    st.write(s, LAST, 0x3C3C)
    st.at(31, "SHO").at(35, "OUT", dict(ra=0)).at(39, "SETP", dict(pin=16, val=1))
    for x in (36, 40, 44):
        st.at(x, "CSRR", dict(rd=1, csr=CSR["PIN_OUT"]))
    st.at(48, "JP", dict(pin=17, val=1, rel=5))
    st.run(r + 10)
    assert [st.records[x].val for x in (36, 40, 44)] == [0x0100, 0x0200, 0x0300]
    assert st.records[48].next_pc == st.records[48].pc + 6
    exp = line(frame_bits([0x3C3C], pre=False, fcs=False), 2, 6)
    assert st.pairs(s, len(exp)) == exp and s + len(exp) == r
    assert st.pairs(r, 10) == [(1, 1)] * 10


@pytest.mark.parametrize("n,pre,fcs", [(n, 1, 1) for n in range(1, 17)]
                         + [(n, p, f) for n in (1, 2, 16) for p, f in ((0, 0), (0, 1), (1, 0))])
def test_c12_odd_length_every_n_and_pre_fcs_off(n, pre, fcs):
    """C12 and N from 1 to 16: an odd-length frame (ETX_DATA then ETX_LAST1),
    with PRE and FCS on and off and IDL from 0 to 9.  The pad trace is the
    6.12.3 bit sequence at N clocks a half bit from s, then N IDL of (1, 0);
    with the preamble and FCS the receiver returns the three bytes."""
    words = [0xA5C3, 0x0042]
    idl = (3 * n) % 10
    s = 24
    st = Stepper()
    st.write(8, CFG, cfg(halfn=n - 1, pre=pre, fcs=fcs, idl=idl))
    st.write(s, DATA, words[0])
    st.write(window_edges(s, n, pre, 0) + 4, LAST1, words[1])
    exp = line(frame_bits(words, True, pre=bool(pre), fcs=bool(fcs)), n, idl)
    st.run(s + len(exp) + 8)
    assert st.pairs(s, len(exp) + 8) == exp + [(0, 0)] * 8
    assert st.stat[s + len(exp) - 1] & BUSY and st.stat[s + len(exp)] == FDONE | RDY
    if pre and fcs:
        frame = rx.receive_frame(*st.tdp_tdn(), half=n)
        assert frame.fcs_ok and frame.payload == bytes([0xC3, 0xA5, 0x42])
        # IDL = 0 (loopback only, 6.12.2) has no start-of-idle pulse, which
        # the receiver reports; any other IDL gives a pulse of N IDL clocks
        # after a last 0 and N (IDL + 1) after a last 1.
        if idl == 0:
            assert frame.errors == ["no start-of-idle high before the line went idle"]
        else:
            assert frame.ok
            assert frame.decoded.tp_idl_clocks == n * (idl + frame.decoded.bits[-1])


@pytest.mark.parametrize("seed", range(40))
def test_random_frames_on_the_block(seed):
    """6.12.4/6.12.5 on the block alone (``tools.loomsim.etx.Etx``), driven
    edge by edge: random configuration, length, last-word kind and write
    edge inside each window.  The pads must be the 6.12.3 sequence and the
    frame must end cleanly; with PRE and FCS the receiver returns the bytes."""
    from tools.loomsim.etx import Etx
    rnd = random.Random(seed)
    n, pre, fcs, idl = rnd.randint(1, 16), rnd.randint(0, 1), rnd.randint(0, 1), rnd.randint(0, 9)
    words = [rnd.randrange(0x10000) for _ in range(rnd.randint(1, 6))]
    one = rnd.random() < 0.5
    last = LAST1 if one else LAST
    s = 10
    writes = {3: (CFG, cfg(n - 1, pre, fcs, idl)), s: (last if len(words) == 1 else DATA, words[0])}
    for m_ in range(1, len(words)):
        lo = 1 if (m_ == 1 and not pre) else 0
        e = window_edges(s, n, pre, m_ - 1) + rnd.randint(lo, 32 * n - 1)
        writes[e] = (last if m_ == len(words) - 1 else DATA, words[m_])
    exp = line(frame_bits(words, one, bool(pre), bool(fcs)), n, idl)
    block = Etx()
    pairs, stats = [], []
    for e in range(1, s + len(exp) + 5):
        block.edge(writes.get(e))
        pairs.append(block.pads(0))
        stats.append(block.stat)
    assert pairs[s - 1:s - 1 + len(exp) + 4] == exp + [(0, 0)] * 4
    assert stats[-1] == FDONE | RDY
    if pre and fcs and idl:
        tdp, tdn = zip(*pairs)
        frame = rx.receive_frame(tdp, tdn, half=n)
        assert frame.ok and frame.payload == frame_bytes(words, one)


def test_fdone_already_set_is_set_again_at_a_release_with_a_clear():
    """W-117 Q1 (reading taken, for the director to rule): FDONE left at 1 by
    an earlier frame, and the next release edge carries an ETX_STAT clear of
    it: the release's set wins, so FDONE stays 1."""
    s = 20
    st = Stepper()
    st.write(8, CFG, cfg(pre=0, fcs=0, idl=0))
    st.write(s, LAST, 0x00FF)                      # release at s + 64
    st.write(s + 72, LAST, 0x0F00)                 # release at s + 136
    st.write(s + 136, STAT, FDONE)
    st.run(s + 150)
    assert st.stat[s + 135] == BUSY | FDONE
    assert st.stat[s + 136] == FDONE | RDY
