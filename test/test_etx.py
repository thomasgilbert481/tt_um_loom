# SPDX-License-Identifier: Apache-2.0
"""The Ethernet serializer (docs/SEMANTICS.md 6.12, D-036): L1 tests.

Two kinds. The unit tests drive the standalone `etx_unit` instance of
tb.v (RTL only) with a write at any edge chosen, so the window ends of
6.12.6 can be hit exactly; every cycle the block's ETX_CFG, ETX_STAT, RDY,
BUSY and pad pair are compared with etx_ref.py, a literal transcription of
6.12.4 and 6.12.5 written from the text (not the golden model, METH-1), and
each test also checks its outcome against values computed independently:
the wire from IEEE 802.3 conventions (zlib.crc32, octets LSB first,
etx_ref.frame_bits) and the edges from 6.12.6's formulas. The chip tests
run firmware on the real core: CAPS, VERSION, the CSRs, pin_in(13/14), the
uo_out[1:0] mux against PIN_OUT, and a frame a thread sends.

The test vectors are W-115 SPEC.md section C, C1 to C12 (named in each
docstring)."""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

from etx_ref import EtxRef, crc_step, frame_bits, manchester, IDLE, TAIL

WE_BIT = {"CFG": 1, "DATA": 2, "LAST": 4, "LAST1": 8, "STAT": 16}


def cfg_word(halfn=1, pre=1, fcs=1, idl=6):
    return (idl << 6) | (fcs << 5) | (pre << 4) | halfn


def gl(dut):
    return not hasattr(dut, "etx_unit")


class Unit:
    """Edge-exact driver of tb.etx_unit with the reference beside it.
    Edge 0 is the last edge with rst_n_etx low; `cyc[e]` is what the block
    shows in cycle e (after edge e)."""

    def __init__(self, dut):
        self.dut = dut
        self.ref = EtxRef()
        self.e = 0
        self.cyc = []          # per cycle: (busy, rdy, stat, cfg, tdp, tdn)

    async def reset(self):
        d = self.dut
        cocotb.start_soon(Clock(d.clk_etx, 10, unit="ns").start())
        d.rst_n_etx.value = 0
        d.etx_we.value = 0
        d.etx_val.value = 0
        for _ in range(3):
            await RisingEdge(d.clk_etx)
        await Timer(1, unit="ns")
        d.rst_n_etx.value = 1
        self._sample()
        self._check()

    def _sample(self):
        d = self.dut
        self.cyc.append((int(d.etx_busy.value), int(d.etx_rdy.value),
                         int(d.etx_stat_rd.value), int(d.etx_cfg_rd.value),
                         int(d.etx_tdp.value), int(d.etx_tdn.value)))

    def _check(self):
        busy, rdy, stat, cfg, tdp, tdn = self.cyc[-1]
        r = self.ref
        where = f"cycle {self.e} (eph {r.eph}, pos {r.pos}, div {r.div})"
        assert (busy, rdy, stat, cfg) == (r.busy, r.rdy, r.stat, r.cfg), \
            f"{where}: rtl busy/rdy/stat/cfg {busy} {rdy} {stat:#x} " \
            f"{cfg:#x}, text {r.busy} {r.rdy} {r.stat:#x} {r.cfg:#x}"
        if busy:
            assert (tdp, tdn) == r.pads(), \
                f"{where}: pads rtl {(tdp, tdn)}, text {r.pads()}"

    async def run(self, writes, until=None, extra=0):
        """Run edges self.e+1 .. with `writes` {edge: (kind, value)}. Stops
        `extra` cycles after the block is idle with no write left, or at
        edge `until`."""
        d = self.dut
        idle_for = 0
        while True:
            e = self.e + 1
            w = writes.get(e)
            d.etx_we.value = WE_BIT[w[0]] if w else 0
            d.etx_val.value = (w[1] & 0xFFFF) if w else 0
            await RisingEdge(d.clk_etx)
            await Timer(1, unit="ns")
            self.e = e
            self.ref.edge(w)
            self._sample()
            self._check()
            if until is not None:
                if e >= until:
                    break
                continue
            pending = any(k > e for k in writes)
            idle_for = idle_for + 1 if (not self.ref.busy and not pending) else 0
            if idle_for > extra:
                break
        d.etx_we.value = 0

    def trace(self, s):
        """Pad pairs from cycle s while busy."""
        out = []
        for c in self.cyc[s:]:
            if not c[0]:
                break
            out.append((c[4], c[5]))
        return out

    def stat(self, e):
        return self.cyc[e][2]

    def busy_until(self, s):
        """First cycle from s with BUSY low: the release edge."""
        for e in range(s, len(self.cyc)):
            if not self.cyc[e][0]:
                return e
        return None


async def unit(dut):
    u = Unit(dut)
    await u.reset()
    return u


def feed(s, kinds, words, edges):
    return {s + de: (k, w) for k, w, de in zip(kinds, words, edges)}


def skip_gl(dut):
    if gl(dut):
        dut._log.info("standalone etx_unit is RTL only; skipped")
        return True
    return False


def bytes_from_trace(trace, n):
    """Decode a Manchester pad trace at n clocks a half bit (independent of
    the edge rules): the bit is the second half's TD+ level."""
    assert len(trace) % (2 * n) == 0, len(trace)
    bits = []
    for i in range(0, len(trace), 2 * n):
        first, second = trace[i:i + n], trace[i + n:i + 2 * n]
        assert len(set(first)) == 1 and len(set(second)) == 1, \
            f"half bits not constant at bit {i // (2 * n)}"
        assert first[0] != second[0], f"no mid-bit transition at bit {i // (2 * n)}"
        assert first[0][0] != first[0][1], "TD+ and TD- must be complementary"
        bits.append(second[0][0])
    return bits


C1_WORDS = [0x3231, 0x3433, 0x3635, 0x3837, 0x0039]
C1_KINDS = ["DATA"] * 4 + ["LAST1"]


@cocotb.test()
async def test_etx_reset_values(dut):
    """ETX_CFG reads 0x01B1 and ETX_STAT 0x0002 after reset (6.12.2)."""
    if skip_gl(dut):
        return
    u = await unit(dut)
    assert u.cyc[0][3] == 0x01B1 and u.cyc[0][2] == 0x0002
    await u.run({}, until=5)


@cocotb.test()
async def test_etx_c1_fcs_check_value(dut):
    """C1: PRE = 0, FCS = 1, N = 2, '123456789' as four ETX_DATA and one
    ETX_LAST1, each word at the last edge of its window (s + 32 N m - 1).
    The FCS bits read LSB first are 0xCBF43926 (zlib.crc32), a receiver's
    CRC over data and FCS ends at 0xC704DD7B, the frame ends N * IDL after
    the last bit, and the start folds the first data bit into the CRC
    (MF-2)."""
    if skip_gl(dut):
        return
    u = await unit(dut)
    await u.run({2: ("CFG", cfg_word(pre=0))}, until=4)
    s, n = 10, 2
    edges = [0] + [32 * n * m - 1 for m in range(1, 5)]
    await u.run(feed(s, C1_KINDS, C1_WORDS, edges), extra=3)
    bits, data = frame_bits(C1_WORDS, True, pre=0, fcs=1)
    assert data == b"123456789"
    nbits = len(bits)
    tr = u.trace(s)
    assert tr == manchester(bits, n) + [(1, 0)] * (n * 6)
    got = bytes_from_trace(tr[:2 * n * nbits], n)
    fcs = sum(b << k for k, b in enumerate(got[-32:]))
    assert fcs == 0xCBF43926, hex(fcs)
    c = 0xFFFFFFFF
    for b in got:
        c = crc_step(c, b)
    assert c == 0xC704DD7B, hex(c)
    assert u.busy_until(s) == s + n * (2 * nbits + 6)       # 6.12.6 release
    assert u.stat(u.busy_until(s)) & 0x1C == 0x04            # FDONE only


def c2_frame():
    dst = [0xFF] * 6
    src = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01]
    arp = [0x08, 0x06, 0, 1, 0x08, 0, 6, 4, 0, 1] + src + [10, 0, 0, 1] \
        + [0] * 6 + [10, 0, 0, 2]
    b = dst + src + arp
    b += [0] * (60 - len(b))
    return [b[2 * i] | (b[2 * i + 1] << 8) for i in range(30)], bytes(b)


@cocotb.test()
async def test_etx_c2_minimum_frame(dut):
    """C2: a 60-byte ARP frame, 29 ETX_DATA and one ETX_LAST, N = 2,
    PRE = 1, each word at the first edge of its window (K_m). The pad
    trace is 2,304 clocks of Manchester from cycle s, then 12 clocks of
    (1, 0), then the idle line; the decoded frame has the preamble, the
    SFD, the 60 bytes and a good FCS."""
    if skip_gl(dut):
        return
    u = await unit(dut)
    words, data = c2_frame()
    s, n = 5, 2
    edges = [0] + [2 * n * (64 + 16 * (m - 1)) for m in range(1, 30)]
    await u.run(feed(s, ["DATA"] * 29 + ["LAST"], words, edges), extra=3)
    tr = u.trace(s)
    assert len(tr) == 2304 + 12 and tr[2304:] == [(1, 0)] * 12
    bits = bytes_from_trace(tr[:2304], n)
    octets = bytes(sum(bits[8 * i + k] << k for k in range(8))
                   for i in range(len(bits) // 8))
    assert octets[:8] == bytes([0x55] * 7 + [0xD5])
    assert octets[8:68] == data
    import zlib
    assert int.from_bytes(octets[68:72], "little") == zlib.crc32(data)
    assert u.stat(u.busy_until(s)) == 0x06                   # FDONE, RDY


async def frame(u, writes_rel, gap=5, extra=3):
    """Start a frame `gap` edges from now; writes_rel {edge - s: (k, v)}.
    Returns s."""
    s = u.e + gap
    await u.run({s + d: w for d, w in writes_rel.items()}, extra=extra)
    return s


async def clear(u, bits=0x1C):
    await u.run({u.e + 1: ("STAT", bits)}, until=u.e + 2)


@cocotb.test()
async def test_etx_c3_window_ends_pre(dut):
    """C3: PRE = 1, N = 2, K_0 = s + 256, K_m = s + 256 + 64 m. Word 1 at
    s + 255 is refused (REF), at s + 256 accepted at the load edge, at
    s + 319 accepted, at s + 320 the underrun (tail from s + 320, UNDER,
    the write refused). The same at word 29: K_28 and K_29 - 1 accepted,
    K_29 the underrun."""
    if skip_gl(dut):
        return
    u = await unit(dut)
    n, idl = 2, 6
    # s + 255 refused, then s + 256 accepted: a good 4-byte frame.
    s = await frame(u, {0: ("DATA", 0x2211), 255: ("LAST", 0x4433),
                        256: ("LAST", 0x4433)})
    assert u.stat(s + 254) & 0x10 == 0 and u.stat(s + 255) & 0x10
    assert u.busy_until(s) == s + n * (2 * (64 + 32 + 32) + idl)
    assert u.stat(u.busy_until(s)) & 0x08 == 0, "no underrun"
    await clear(u)
    s = await frame(u, {0: ("DATA", 0x2211), 319: ("LAST", 0x4433)})
    assert u.stat(u.busy_until(s)) & 0x18 == 0, "accepted at s + 319"
    s = await frame(u, {0: ("DATA", 0x2211), 320: ("LAST", 0x4433)})
    assert u.stat(s + 319) & 0x18 == 0
    assert u.stat(s + 320) & 0x18 == 0x18, "underrun and refused at K_1"
    assert u.trace(s)[320:] == [(1, 0)] * (n * idl)
    assert u.busy_until(s) == s + 320 + n * idl
    await clear(u)
    for d29, under in ((256 + 64 * 28, 0), (256 + 64 * 29 - 1, 0),
                       (256 + 64 * 29, 1)):
        w = {0: ("DATA", 0x0100)}
        w.update({256 + 64 * (m - 1): ("DATA", 0x0100 + m) for m in range(1, 29)})
        w[d29] = ("LAST", 0xBEEF)
        s = await frame(u, w)
        rel = u.busy_until(s)
        assert bool(u.stat(rel) & 0x08) == bool(under), f"word 29 at s+{d29}"
        if under:
            assert rel == s + 256 + 64 * 29 + n * idl
        else:
            assert rel == s + n * (2 * (64 + 16 * 30 + 32) + idl)
        await clear(u)


@cocotb.test()
async def test_etx_c4_window_ends_no_pre(dut):
    """C4: PRE = 0, N = 2: word 1 at s + 63 accepted, at s + 64 the
    underrun."""
    if skip_gl(dut):
        return
    u = await unit(dut)
    await u.run({2: ("CFG", cfg_word(pre=0))}, until=3)
    s = await frame(u, {0: ("DATA", 0xA5A5), 63: ("LAST", 0x5A5A)})
    assert u.stat(u.busy_until(s)) & 0x18 == 0
    s = await frame(u, {0: ("DATA", 0xA5A5), 64: ("LAST", 0x5A5A)})
    assert u.stat(s + 64) & 0x18 == 0x18
    assert u.busy_until(s) == s + 64 + 2 * 6


@cocotb.test()
async def test_etx_c5_underrun_idl0(dut):
    """C5: an underrun with IDL = 0: UNDER and FDONE are both set at the
    release edge, which is the load edge (MF-4)."""
    if skip_gl(dut):
        return
    u = await unit(dut)
    await u.run({2: ("CFG", cfg_word(idl=0))}, until=3)
    s = await frame(u, {0: ("DATA", 0x1234)})
    k1 = s + 256 + 64                 # K_0 loads word 0; K_1 finds HV == 0
    assert u.busy_until(s) == k1
    assert u.stat(k1 - 1) & 0x0C == 0 and u.stat(k1) & 0x0C == 0x0C


@cocotb.test()
async def test_etx_c6_start_while_under(dut):
    """C6: a data write while UNDER == 1 in P_IDLE is refused (REF, no
    start); after the ETX_STAT clear a start succeeds."""
    if skip_gl(dut):
        return
    u = await unit(dut)
    await u.run({2: ("CFG", cfg_word(pre=0))}, until=3)
    s = await frame(u, {0: ("DATA", 0x1111)})            # underrun
    assert u.stat(u.e) == 0x0C                            # UNDER, FDONE, !RDY
    e = u.e + 2
    await u.run({e: ("LAST", 0x2222)}, until=e + 3)
    assert u.stat(e) == 0x1C and not u.cyc[e][0]
    await clear(u)
    assert u.stat(u.e) == 0x02
    s = await frame(u, {0: ("LAST", 0x2222)})
    assert u.cyc[s][0] and u.stat(u.busy_until(s)) == 0x06


@cocotb.test()
async def test_etx_c7_set_wins_over_clear(dut):
    """C7: an ETX_STAT write of 0x0004 committing at the release edge leaves
    FDONE = 1, and one of 0x0008 at an underrun edge leaves UNDER = 1; the
    same writes one edge later clear them."""
    if skip_gl(dut):
        return
    u = await unit(dut)
    await u.run({2: ("CFG", cfg_word(pre=0))}, until=3)
    rel = 2 * (2 * (16 + 32) + 6)                          # s + N (2 NBITS + IDL)
    s = await frame(u, {0: ("LAST", 0x1234), rel: ("STAT", 0x04),
                        rel + 1: ("STAT", 0x04)})
    assert u.busy_until(s) == s + rel
    assert u.stat(s + rel - 1) & 0x04 == 0
    assert u.stat(s + rel) & 0x04, "set wins over clear"
    assert u.stat(s + rel + 1) & 0x04 == 0
    s = await frame(u, {0: ("DATA", 0x1234), 64: ("STAT", 0x08),
                        65: ("STAT", 0x08)})
    assert u.stat(s + 64) & 0x08 and u.stat(s + 65) & 0x08 == 0


@cocotb.test()
async def test_etx_c8_writes_at_release(dut):
    """C8: a CFG write and, in another frame, a data write committing at the
    release edge are both refused (REF): EPH is P_TAIL before it. Neither
    stores anything nor starts a frame."""
    if skip_gl(dut):
        return
    u = await unit(dut)
    rel = 2 * (2 * (64 + 16 + 32) + 6)
    s = await frame(u, {0: ("LAST", 0x00FF), rel: ("CFG", cfg_word(halfn=3))})
    assert u.busy_until(s) == s + rel and u.stat(s + rel) & 0x10
    assert u.cyc[s + rel][3] == 0x01B1, "CFG not stored"
    await clear(u)
    s = await frame(u, {0: ("LAST", 0x00FF), rel: ("DATA", 0x1234)})
    assert u.stat(s + rel) & 0x10 and not any(c[0] for c in u.cyc[s + rel:])


@cocotb.test()
async def test_etx_c12_odd_frames_and_cfg(dut):
    """C12: an odd-length frame (ETX_LAST1), PRE = 0 with FCS = 0, N = 1 and
    N = 16, and a CFG write while busy (refused, REF, CFG unchanged)."""
    if skip_gl(dut):
        return
    u = await unit(dut)
    cases = [(0, 1, 1, 6), (0, 0, 0, 6), (15, 1, 1, 3), (0, 0, 1, 0)]
    words = [0x8001, 0x7F00, 0x00C3]
    for halfn, pre, fcs, idl in cases:
        n = halfn + 1
        cfg = cfg_word(halfn, pre, fcs, idl)
        await u.run({u.e + 1: ("CFG", cfg)}, until=u.e + 2)
        assert u.cyc[u.e][3] == cfg
        # word 1 at K_0 (PRE) or s + 1 (no PRE), word 2 at K_1 (6.12.6)
        k0 = 2 * n * 64 if pre else 0
        w = {0: ("DATA", words[0]), (k0 if pre else 1): ("DATA", words[1]),
             k0 + 32 * n: ("LAST1", words[2]), 3: ("CFG", 0x0012)}
        s = await frame(u, w)
        assert u.stat(s + 3) & 0x10 and u.cyc[s + 3][3] == cfg
        bits, _ = frame_bits(words, True, pre, fcs)
        assert u.trace(s) == manchester(bits, n) + [(1, 0)] * (n * idl)
        assert u.busy_until(s) == s + n * (2 * len(bits) + idl)
        assert u.stat(u.busy_until(s)) & 0x0C == 0x04
        await clear(u)


@cocotb.test()
async def test_etx_idl_top_of_its_range(dut):
    """IDL is a 4-bit field the block does not range-check (6.12.2), so
    IDL = 15 and 9 must give TP_IDL like any other value, on each of the
    three ways into the tail (6.12.5 rule 5 without FCS, rule 6, rule 3's
    underrun): N * IDL clocks of (1, 0) after a last 0, the release at
    s + N (2 NBITS + IDL) (6.12.6), FDONE, and UNDER only after the
    underrun. Added by W-122: every other test used IDL <= 6."""
    if skip_gl(dut):
        return
    u = await unit(dut)
    n = 2
    for idl in (15, 9):
        for fcs in (0, 1):
            await u.run({u.e + 1: ("CFG", cfg_word(1, 0, fcs, idl))}, until=u.e + 2)
            s = await frame(u, {0: ("LAST1", 0x0080)})
            bits, _ = frame_bits([0x0080], True, 0, fcs)
            assert u.trace(s) == manchester(bits, n) + [(1, 0)] * (n * idl)
            assert u.busy_until(s) == s + n * (2 * len(bits) + idl)
            assert u.stat(u.busy_until(s)) & 0x0C == 0x04, "FDONE, no UNDER"
            await clear(u)
        s = await frame(u, {0: ("DATA", 0x1234)})       # K_1 = s + 32 N: underrun
        assert u.busy_until(s) == s + 32 * n + n * idl
        assert u.trace(s)[32 * n:] == [(1, 0)] * (n * idl)
        assert u.stat(u.busy_until(s)) & 0x0C == 0x0C, "UNDER and FDONE"
        await clear(u)


@cocotb.test()
async def test_etx_random_against_text(dut):
    """Random configurations and write streams (every kind, random edges,
    gaps that hit the windows and miss them), compared with the
    transcription of 6.12.4/6.12.5 in every cycle."""
    if skip_gl(dut):
        return
    u = await unit(dut)
    rng = random.Random(0xE7)
    for _ in range(40):
        n = rng.choice([1, 1, 2, 2, 3, 4])
        cfg = cfg_word(n - 1, rng.getrandbits(1), rng.getrandbits(1),
                       rng.choice([0, 1, 2, 5, 6]))
        e = u.e + 1
        writes = {e: ("CFG", cfg)}
        t = e + rng.randint(1, 4)
        for _ in range(rng.randint(1, 8)):
            k = rng.choice(["DATA"] * 5 + ["LAST", "LAST1", "STAT", "CFG"])
            writes[t] = (k, rng.getrandbits(16))
            t += rng.choice([1, 2, 3, 31 * n, 32 * n - 1, 32 * n, 32 * n + 1,
                             128 * n, rng.randint(1, 64 * n)])
        await u.run(writes, extra=2)
        await clear(u)


# ===================================================================== chip
from cocotb.triggers import ClockCycles, FallingEdge          # noqa: E402
from spi_host import (                                        # noqa: E402
    LoomHost, asm, run_snippet, resolve, SP_CTRL, CTRL_CAPS, CTRL_VERSION,
    CTRL_PIN_OUT, CSR_INGRP, CSR_OUTGRP)

ETX_CFG, ETX_DATA, ETX_LAST, ETX_LAST1, ETX_STAT = 0x16, 0x17, 0x18, 0x19, 0x1A
CSR_KIND = {ETX_CFG: "CFG", ETX_DATA: "DATA", ETX_LAST: "LAST",
            ETX_LAST1: "LAST1", ETX_STAT: "STAT"}


class ChipMon:
    """Every cycle (sampled mid-cycle): uo_out and the retire record. Entry
    c is cycle c of this monitor; a slot in W in cycle c has X cycle c - 1
    and commits at the edge that starts cycle c + 1."""

    def __init__(self, dut):
        self.dut, self.rows = dut, []
        loom = dut.user_project.u_loom
        self.sig = (loom.tr_valid, loom.tr_thread, loom.tr_ir, loom.tr_val,
                    loom.tr_next_pc, loom.tr_pc, loom.tr_done)

    async def _run(self):
        while True:
            await FallingEdge(self.dut.clk)
            self.rows.append((resolve(self.dut.uo_out),)
                             + tuple(resolve(s) for s in self.sig))

    def start(self):
        self.task = cocotb.start_soon(self._run())
        return self

    def stop(self):
        self.task.cancel()


def replay(mon, decode, pin_pair=(0, 0)):
    """Rebuild the ETX write stream from the retire record (`decode` maps an
    instruction word to (kind, value), ('PIN', (bit, v)) or None), run the
    transcription over it and compare the pads in every cycle and every
    ETX read in its X cycle. Returns the reference states per cycle."""
    ref, writes, pins, reads = EtxRef(), {}, {}, []
    for c, (uo, v, th, ir, val, npc, pc, done) in enumerate(mon.rows):
        if not (v and done):
            continue
        d = decode(th, ir)
        if d is None:
            continue
        if d[0] == "PIN":
            pins.setdefault(c + 1, []).extend(d[1])
        elif d[0] in WE_BIT:
            writes[c + 1] = d
        else:
            reads.append((c - 1, d[0], val, npc, pc))
    states, pp = [], list(pin_pair)
    for c, row in enumerate(mon.rows):
        if c:
            ref.edge(writes.get(c))
            for bit, b in pins.get(c, []):
                pp[bit] = b
        states.append((ref.busy, ref.rdy, ref.stat, ref.cfg, ref.eph))
        uo = row[0]
        assert (uo & 1, (uo >> 1) & 1) == ref.pads(tuple(pp)), \
            f"cycle {c}: uo_out[1:0] {uo & 3:02b}, text {ref.pads(tuple(pp))}"
    for x, kind, val, npc, pc in reads:
        busy, rdy, stat, cfg, _ = states[x]
        if kind == "RSTAT":
            assert val == stat, f"CSRR ETX_STAT in cycle {x}: {val:#x} != {stat:#x}"
        elif kind == "RCFG":
            assert val == cfg, f"CSRR ETX_CFG in cycle {x}: {val:#x}"
        elif kind == "IN":
            assert val == (busy << 1) | rdy, f"IN 13..14 in cycle {x}: {val:#x}"
        elif kind == "JP14":
            want = (pc + 2) if busy else (pc + 1)
            assert npc == want & 0x3FF, f"JP 14 in cycle {x}: next {npc:#x}"
    return states, writes, reads


async def chip(dut):
    host = LoomHost(dut)
    await host.start()
    return host


async def run_threads(host, progs, regs=None, csrs=None):
    """Load per-thread programs at t * 128, preset registers and CSRs,
    reset and run all threads in `progs` together."""
    image = {}
    for t, words in progs.items():
        image.update({128 * t + i: w for i, w in enumerate(words)})
    await host.load_program(image, verify=False)
    mask = 0
    for t in progs:
        await host.reset_thread(t)
        for n, v in (regs or {}).get(t, {}).items():
            await host.write_reg(t, n, v)
        for n, v in (csrs or {}).get(t, {}).items():
            await host.write_csr(t, n, v)
        mask |= 1 << t
    await host.run(mask)


def decoder(regs, extra=None):
    """Map the program's instruction words to replay() events."""
    table = {}
    for t in range(4):
        r = regs.get(t, {})
        for csr, kind in CSR_KIND.items():
            for ra in range(8):
                table[(t, asm("CSRW", csr=csr, ra=ra))] = (kind, r.get(ra, 0))
        for rd in range(8):
            table[(t, asm("CSRR", rd=rd, csr=ETX_STAT))] = ("RSTAT",)
            table[(t, asm("CSRR", rd=rd, csr=ETX_CFG))] = ("RCFG",)
    table.update(extra or {})
    return lambda th, ir: table.get((th, ir))


@cocotb.test()
async def test_etx_chip_ids_and_csrs(dut):
    """CAPS reads 0x96BA and VERSION 5 (6.12.8); through firmware ETX_CFG
    reads 0x01B1 and ETX_STAT 0x0002 after reset, the three data CSRs read
    0, IN over 13..15 reads {0, BUSY, RDY} = 0b001, PIN_IN keeps 15:13 at
    0, ETX_CFG takes 10 bits while idle, and an ETX_STAT write of 1s to
    bits 1:0 changes nothing."""
    host = await chip(dut)
    assert await host.read1(SP_CTRL, CTRL_CAPS) == 0x96BA
    assert await host.read1(SP_CTRL, CTRL_VERSION) == 5
    prog = [asm("CSRR", rd=1, csr=ETX_CFG), asm("CSRR", rd=2, csr=ETX_STAT),
            asm("CSRR", rd=3, csr=ETX_DATA), asm("CSRR", rd=4, csr=ETX_LAST),
            asm("CSRR", rd=5, csr=ETX_LAST1), asm("IN", rd=6),
            asm("CSRR", rd=7, csr=0x13)]
    await run_snippet(host, prog, csrs={CSR_INGRP: (3 << 5) | 13})
    got = [await host.read_reg(0, n) for n in range(1, 8)]
    assert got[:6] == [0x01B1, 0x0002, 0, 0, 0, 0b001], [hex(g) for g in got]
    assert got[6] >> 13 == 0, "PIN_IN bits 15:13 stay 0"
    prog = [asm("CSRW", csr=ETX_CFG, ra=1), asm("CSRW", csr=ETX_STAT, ra=2),
            asm("CSRR", rd=3, csr=ETX_CFG), asm("CSRR", rd=4, csr=ETX_STAT),
            asm("CSRW", csr=ETX_CFG, ra=5), asm("CSRR", rd=6, csr=ETX_CFG)]
    await run_snippet(host, prog, regs={1: 0xFFFF, 2: 0x0003, 5: 0x01B1})
    assert await host.read_reg(0, 3) == 0x03FF
    assert await host.read_reg(0, 4) == 0x0002
    assert await host.read_reg(0, 6) == 0x01B1


def reader_threads():
    """Threads 1..3 poll the block in every one of their X cycles: CSRR
    ETX_STAT (1), IN over 13..14 (2), and CSRR / JP 14 (3)."""
    st = asm("CSRR", rd=1, csr=ETX_STAT)
    jp = asm("JP", pin=14, val=1, rel=1)
    return {1: [st] * 8 + [asm("JMP", abs=128)],
            2: [asm("IN", rd=2)] * 8 + [asm("JMP", abs=256)],
            3: [st, jp, st, st, jp, st, st] + [asm("JMP", abs=384)]}, \
        {(2, asm("IN", rd=2)): ("IN",), (3, jp): ("JP14",)}


async def finish(host, mon, cycles=600):
    await host.wait_halted(1)
    await ClockCycles(host.dut.clk, cycles)
    await host.halt()
    mon.stop()


@cocotb.test()
async def test_etx_chip_frame_from_firmware(dut):
    """C1 sent by thread 0 with CSRW and WAITP on pin_in(13), while threads
    1..3 read ETX_STAT, IN 13..14 and JP 14 in every other cycle (C10: the
    cycles of the start and of the release are among them). The pads are
    compared with the transcription in every cycle and decode to
    '123456789' with the zlib FCS."""
    if skip_gl(dut):
        return
    host = await chip(dut)
    regs0 = {0: cfg_word(pre=0)}
    regs0.update({i + 1: w for i, w in enumerate(C1_WORDS)})
    rdy = asm("WAITP", pin=13, val=1)
    feeder = [asm("CSRW", csr=ETX_CFG, ra=0), asm("CSRW", csr=ETX_DATA, ra=1),
              asm("CSRW", csr=ETX_DATA, ra=2), rdy,
              asm("CSRW", csr=ETX_DATA, ra=3), rdy,
              asm("CSRW", csr=ETX_DATA, ra=4), rdy,
              asm("CSRW", csr=ETX_LAST1, ra=5),
              asm("WAITP", pin=14, val=0), asm("HALT")]
    readers, extra = reader_threads()
    mon = ChipMon(dut).start()
    await run_threads(host, {0: feeder, **readers}, regs={0: regs0},
                      csrs={2: {CSR_INGRP: (2 << 5) | 13}})
    await finish(host, mon)
    states, writes, reads = replay(mon, decoder({0: regs0}, extra))
    s = min(c for c, w in writes.items() if w[0] != "CFG")
    xs = {x for x, *_ in reads}
    rel = next(c for c in range(s, len(states)) if not states[c][0])
    assert {s - 1, s, s + 1, rel - 1, rel} <= xs, "C10 cycles not all read"
    tr = [((r[0] & 1), (r[0] >> 1) & 1) for r in mon.rows[s:rel]]
    bits, _ = frame_bits(C1_WORDS, True, 0, 1)
    assert tr == manchester(bits, 2) + [(1, 0)] * 12
    assert states[rel][2] & 0x1C == 0x04, "FDONE, no UNDER, no REF"


@cocotb.test()
async def test_etx_chip_feeder_stops(dut):
    """C9: a feeder that HALTs after its first word, and one that is halted
    and given CTRL.RESET by the host mid-frame: each frame ends by itself
    by underrun (UNDER, FDONE), and the pads return to PIN_OUT."""
    if skip_gl(dut):
        return
    host = await chip(dut)
    regs0 = {1: 0x1234}
    mon = ChipMon(dut).start()
    await run_threads(host, {0: [asm("CSRW", csr=ETX_DATA, ra=1), asm("HALT")]},
                      regs={0: regs0})
    await finish(host, mon, cycles=2 * 2 * (64 + 16 + 16) + 100)
    states, writes, _ = replay(mon, decoder({0: regs0}))
    assert len(writes) == 1 and states[-1][2] == 0x0C       # UNDER, FDONE
    assert mon.rows[-1][0] & 3 == 0
    # A looping feeder, halted and reset by the host after about 1,000 cycles.
    host = await chip(dut)
    regs0 = {1: 0xA55A}
    rdy = asm("WAITP", pin=13, val=1)
    loop = [asm("CSRW", csr=ETX_DATA, ra=1), rdy,
            asm("CSRW", csr=ETX_DATA, ra=1), asm("JMP", abs=1)]
    mon = ChipMon(dut).start()
    await run_threads(host, {0: loop}, regs={0: regs0})
    await ClockCycles(dut.clk, 1000)
    await host.halt()
    await host.reset_thread(0)
    await ClockCycles(dut.clk, 2 * 2 * 32 + 200)
    mon.stop()
    states, writes, _ = replay(mon, decoder({0: regs0}))
    assert len(writes) > 10 and states[-1][2] == 0x0C
    assert not states[-1][0] and mon.rows[-1][0] & 3 == 0


@cocotb.test()
async def test_etx_chip_pin_writes_during_frame(dut):
    """C11: SETP 16, SETP 17 and OUT to 16..17 during a frame commit to
    PIN_OUT, are not seen on the pads while BUSY, and are seen from the
    cycle after the release; pin_in(16/17) and PIN_OUT keep the register
    value throughout."""
    if skip_gl(dut):
        return
    host = await chip(dut)
    regs0 = {0: cfg_word(pre=0)}
    regs0.update({i + 1: w for i, w in enumerate(C1_WORDS)})
    rdy = asm("WAITP", pin=13, val=1)
    feeder = [asm("CSRW", csr=ETX_CFG, ra=0), asm("CSRW", csr=ETX_DATA, ra=1),
              asm("CSRW", csr=ETX_DATA, ra=2), rdy,
              asm("CSRW", csr=ETX_DATA, ra=3), rdy,
              asm("CSRW", csr=ETX_DATA, ra=4), rdy,
              asm("CSRW", csr=ETX_LAST1, ra=5), asm("HALT")]
    s16, s17 = asm("SETP", pin=16, val=1), asm("SETP", pin=17, val=1)
    out = asm("OUT", ra=3)
    pinner = [asm("WAITP", pin=14, val=1), s16, s17, asm("IN", rd=4), out,
              asm("IN", rd=5), asm("WAITP", pin=14, val=0), asm("IN", rd=6),
              asm("HALT")]
    extra = {(1, s16): ("PIN", [(0, 1)]), (1, s17): ("PIN", [(1, 1)]),
             (1, out): ("PIN", [(0, 0), (1, 1)])}
    mon = ChipMon(dut).start()
    await run_threads(host, {0: feeder, 1: pinner},
                      regs={0: regs0, 1: {3: 0b10}},
                      csrs={1: {CSR_OUTGRP: (2 << 5) | 16,
                                CSR_INGRP: (2 << 5) | 16}})
    await host.wait_halted(0b11)
    await ClockCycles(dut.clk, 20)
    mon.stop()
    states, writes, _ = replay(mon, decoder({0: regs0}, extra))
    assert [await host.read_reg(1, n) for n in (4, 5, 6)] == [0b11, 0b10, 0b10]
    assert mon.rows[-1][0] & 3 == 0b10, "PIN_OUT[9:8] = (0, 1) after release"
    assert (await host.read1(SP_CTRL, CTRL_PIN_OUT) >> 8) & 3 == 0b10


@cocotb.test()
async def test_etx_chip_waite_on_status_pins_is_never_true(dut):
    """6.12.7 and 6.4: WAITE on pin_in(13) (RDY) is never true, because
    PREV_PINS holds indices 0..12 only, even though RDY reads 1 while the
    block is idle. With the T bit, a WAITE 13 rise or any must therefore end
    at the deadline with T set, while WAITP 13 = 1 (the control: RDY really
    is 1) ends by its condition and clears T. Added by W-122: before D-036
    index 13 read 0, so nothing checked that the edge rule still excludes
    it now that it reads a live level."""
    host = await chip(dut)
    for edge in (0, 2):                              # rise, any
        await run_snippet(host, [asm("CSRW", csr=0x0B, ra=1),
                                 asm("SETD", imm=40),
                                 asm("WAITE", pin=13, edge=edge, tmo=1),
                                 asm("CSRR", rd=2, csr=0x0B)],
                          regs={1: 0, 2: 0})
        assert await host.read_reg(0, 2) & 0x4, \
            f"WAITE 13 edge {edge} fired on RDY; it must time out (T = 1)"
    await run_snippet(host, [asm("CSRW", csr=0x0B, ra=1),
                             asm("SETD", imm=40),
                             asm("WAITP", pin=13, val=1, tmo=1),
                             asm("CSRR", rd=2, csr=0x0B)],
                      regs={1: 0x4, 2: 0})
    assert await host.read_reg(0, 2) & 0x4 == 0, "WAITP 13 = 1 must succeed"
