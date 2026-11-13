# SPDX-License-Identifier: Apache-2.0
"""L2-COV for the Ethernet serializer (``docs/SEMANTICS.md`` 6.12, D-036).

SEMANTICS 6.12.9 asks the co-simulation generator for bins that need the
serializer's state at every ETX write: was it accepted, which window edge
was it at, was ``HV`` set. No read exposes that state, and the golden model
and the RTL may not be looked into for it (METH-1). :class:`EtxTracker` is
therefore a third transcription of 6.12.4 and 6.12.5, written from the text
alone by the W-119 verification agent (``test/etx_ref.py``, the RTL agent's,
was not used). It is fed the ETX writes the model's retire records commit,
one edge at a time, and serves two purposes:

* it classifies every write for the bins (:class:`EtxCoverage`);
* it predicts what the two implementations already agree on, as a check of
  the check: the pad pair while ``BUSY``, every ``CSRR ETX_STAT`` /
  ``ETX_CFG`` value, and ``pin_in(13)``/``pin_in(14)`` as ``JP``, ``IN`` and
  ``WAITP`` saw them. A disagreement is recorded (``mismatches``), not
  raised: the harness's own RTL-against-model comparison is the test, and a
  disagreement here means the tracker, or both implementations, read
  SEMANTICS differently, which a person decides.
"""

from __future__ import annotations

IDLE, PRE, DATA, FCS, TAIL = "P_IDLE", "P_PRE", "P_DATA", "P_FCS", "P_TAIL"
POLY = 0x04C11DB7
DATA_KINDS = ("ETX_DATA", "ETX_LAST", "ETX_LAST1")


def _crc(c: int, v: int) -> int:
    """6.12.3: one data bit into CRC32."""
    fb = ((c >> 31) & 1) ^ v
    return ((c << 1) & 0xFFFFFFFF) ^ (POLY if fb else 0)


def _pre_bit(k: int) -> int:
    """6.12.3: p_k = 1 for even k, 0 for odd k, except p_63 = 1."""
    return 1 if (k % 2 == 0 or k == 63) else 0


class EtxTracker:
    """The state of 6.12.1 under its own names, stepped edge by edge."""

    FIELDS = ("halfn", "pre", "fcs", "idl", "eph", "div", "hb", "pos", "txb",
              "txl", "sh", "sl", "s1", "hold", "hv", "hl", "h1", "lseen", "crc",
              "fdone", "under", "ref")

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        for f in self.FIELDS:
            setattr(self, f, 0)
        self.halfn, self.pre, self.fcs, self.idl, self.eph = 1, 1, 1, 6, IDLE

    # ------------------------------------------------------------ views
    @property
    def busy(self) -> int:
        return int(self.eph != IDLE)

    @property
    def rdy(self) -> int:
        return int(not self.under and (self.eph == IDLE or (
            self.eph in (PRE, DATA) and not self.lseen and not self.hv)))

    @property
    def cfg(self) -> int:
        return self.idl << 6 | self.fcs << 5 | self.pre << 4 | self.halfn

    @property
    def stat(self) -> int:
        return (self.ref << 4 | self.under << 3 | self.fdone << 2 |
                self.rdy << 1 | self.busy)

    @property
    def pads(self):
        """``(uo_out[0], uo_out[1])`` while ``BUSY`` (6.12.7), else None."""
        if self.eph == IDLE:
            return None
        if self.eph == TAIL:
            return (1, 0)
        return (self.txl, 1 - self.txl)

    def snapshot(self):
        return {f: getattr(self, f) for f in self.FIELDS}

    def at_last_bit(self, s) -> bool:
        return s["pos"] == (7 if s["s1"] else 15)

    # ------------------------------------------------------------- 6.12.4
    def edge(self, w=None):
        """One edge. ``w`` is ``(csr_name, value)`` of the ETX write that
        commits at it, or None. Every right-hand side is the value before
        the edge (``o``). Returns what happened, for the bins."""
        o = self.snapshot()
        ev = {}
        n = o["halfn"] + 1
        kind, value = w if w is not None else (None, 0)
        if o["eph"] == IDLE:
            if kind == "ETX_CFG":
                self.halfn, self.pre = value & 0xF, (value >> 4) & 1
                self.fcs, self.idl = (value >> 5) & 1, (value >> 6) & 0xF
                ev["cfg"] = "stored"
            elif kind in DATA_KINDS:
                if o["under"]:
                    self.ref = 1
                    ev["data"] = ("refused", "idle_UNDER")
                else:
                    self._start(kind, value)
                    ev["data"] = ("start", "")
                    ev["start"] = True
        else:
            if o["div"] != 0:
                self.div = o["div"] - 1
            else:
                self._advance(o, n, ev)
            if kind == "ETX_CFG":
                self.ref = 1
                ev["cfg"] = "refused_busy"
            elif kind in DATA_KINDS:
                self._accept(kind, value, o, ev)
        if kind == "ETX_STAT":
            ev["stat"] = value
            if value & 0x10:
                self.ref = 0
            if value & 0x08:
                if ev.get("underrun"):
                    ev["clear_vs_set"] = "UNDER"
                else:
                    self.under = 0
            if value & 0x04:
                if ev.get("release"):
                    ev["clear_vs_set"] = "FDONE"
                else:
                    self.fdone = 0
        return ev

    def _begin(self, v: int, data: bool, n: int) -> None:
        self.txb, self.hb, self.div, self.txl = v, 0, n - 1, 1 - v
        if data:
            self.crc = _crc(self.crc, v)

    def _start(self, kind: str, value: int) -> None:
        n = self.halfn + 1
        last, one = int(kind != "ETX_DATA"), int(kind == "ETX_LAST1")
        self.lseen, self.div, self.pos = last, n - 1, 0
        self.crc = 0xFFFFFFFF
        if self.pre:
            self.eph = PRE
            self.hold, self.hv, self.hl, self.h1 = value, 1, last, one
            self._begin(_pre_bit(0), False, n)
        else:
            self.eph = DATA
            self.sh, self.sl, self.s1, self.hv = value, last, one, 0
            self._begin(value & 1, True, n)

    def _enter_tail(self, o, n: int, ev) -> None:
        self.hv = 0
        ev["tail"] = True
        if o["idl"] == 0:
            self._release(ev)
        else:
            self.eph, self.pos, self.div = TAIL, 0, n - 1

    def _release(self, ev) -> None:
        self.eph, self.hv, self.lseen, self.fdone = IDLE, 0, 0, 1
        ev["release"] = True

    # ------------------------------------------------------------- 6.12.5
    def _advance(self, o, n: int, ev) -> None:
        e = o["eph"]
        last_bit = self.at_last_bit(o)
        if e in (PRE, DATA, FCS) and o["hb"] == 0:                       # 1
            self.hb, self.div, self.txl = 1, n - 1, o["txb"]
        elif e == PRE and o["pos"] < 63:                                 # 2
            self.pos = o["pos"] + 1
            self._begin(_pre_bit(o["pos"] + 1), False, n)
        elif (e == PRE) or (e == DATA and last_bit and not o["sl"]):    # 3
            ev["load_edge"] = True
            if o["hv"]:
                self.sh, self.sl, self.s1, self.hv = o["hold"], o["hl"], o["h1"], 0
                self.pos, self.eph = 0, DATA
                self._begin(o["hold"] & 1, True, n)
                ev["load"] = True
            else:
                self.under = 1
                ev["underrun"] = True
                self._enter_tail(o, n, ev)
        elif e == DATA and not last_bit:                                 # 4
            self.pos = o["pos"] + 1
            self._begin((o["sh"] >> (o["pos"] + 1)) & 1, True, n)
        elif e == DATA:                                                  # 5
            if o["fcs"]:
                self.eph, self.pos = FCS, 0
                self._begin(1 - ((o["crc"] >> 31) & 1), False, n)
            else:
                self._enter_tail(o, n, ev)
        elif e == FCS:                                                   # 6
            if o["pos"] < 31:
                self.pos = o["pos"] + 1
                self._begin(1 - ((o["crc"] >> (30 - o["pos"])) & 1), False, n)
            else:
                self._enter_tail(o, n, ev)
        else:                                                            # 7: P_TAIL
            if o["pos"] + 1 == o["idl"]:
                self._release(ev)
            else:
                self.pos, self.div = o["pos"] + 1, n - 1

    def _accept(self, kind: str, value: int, o, ev) -> None:
        """6.12.4 "Accept or refuse", at an edge where ``EPH != P_IDLE``."""
        last, one = int(kind != "ETX_DATA"), int(kind == "ETX_LAST1")
        if (o["eph"] in (PRE, DATA) and self.eph in (PRE, DATA) and not o["lseen"]
                and not o["under"] and (not o["hv"] or ev.get("load"))):
            self.hold, self.hv, self.hl, self.h1, self.lseen = value, 1, last, one, last
            ev["data"] = ("accepted", "at_load_HV1" if ev.get("load") and o["hv"] else "HV0")
            return
        self.ref = 1
        if ev.get("release"):
            why = "release_edge"
        elif o["eph"] == TAIL:
            why = "in_tail"
        elif o["eph"] == FCS:
            why = "in_fcs"
        elif ev.get("underrun"):
            why = "underrun_edge"
        elif self.eph not in (PRE, DATA):
            why = "frame_end_edge"
        elif o["lseen"]:
            why = "after_last"
        elif o["under"]:
            why = "UNDER"
        else:
            why = "before_K0" if o["eph"] == PRE else "hold_full"
        ev["data"] = ("refused", why)


# ------------------------------------------------------------------ bins
GROUP = "etx"
#: Bins one writer cannot reach, with the reason (see the module docstring
#: of tools/loomgen/etxgen.py).
ONE_WRITER = ("one writer commits only at edges = s (mod 4) (SEMANTICS 2: a "
              "thread's slots are 4 clocks apart) and K_m - s is a multiple of "
              "32 N (6.12.6); K - 1 and K + 1 need a second writer, which "
              "6.12.9 leaves untested")
UNREACHABLE = {
    "edge:K-1_refused_hold_full": ONE_WRITER,
    "edge:K-1_accepted_last_edge": ONE_WRITER,
    "edge:K+1_accepted": ONE_WRITER,
    "edge:underrun_K+1_refused": ONE_WRITER,
    "feeder:reset_mid_frame": "neither the backdoor harness nor the SPI host "
                              "plan issues CTRL.RESET; a HALTed feeder is "
                              "generated instead",
}
READ_KINDS = ("CSRR_ETX_STAT", "IN_13_14", "JP14")


def bins():
    out = ["frame:N=%d" % n for n in range(1, 17)]
    out += ["frame:PRE%d_FCS%d" % (p, f) for p in (0, 1) for f in (0, 1)]
    out += ["frame:IDL=0", "frame:IDL1-4", "frame:IDL5-8", "frame:IDL9-15",
            "frame:last_2bytes", "frame:last_1byte", "frame:end_normal",
            "frame:end_underrun", "frame:underrun_and_FDONE_same_edge"]
    out += ["CFG:stored", "CFG:refused_busy", "CFG:bits15_10_ignored"]
    out += ["CFG:N=%d" % n for n in range(1, 17)]
    out += ["CFG:IDL=0", "CFG:PRE0", "CFG:PRE1", "CFG:FCS0", "CFG:FCS1"]
    out += ["accept:HV0", "accept:at_load_HV1"]
    out += ["refuse:%s" % r for r in ("before_K0", "hold_full", "after_last",
                                      "in_tail", "release_edge", "idle_UNDER",
                                      "underrun_edge", "in_fcs", "frame_end_edge")]
    out += ["edge:K_accepted_HV1", "edge:K+4_accepted", "edge:K-4_refused_hold_full",
            "edge:K-4_accepted_last_slot", "edge:underrun_K_refused",
            "edge:underrun_K+4_refused"] + sorted(k for k in UNREACHABLE if k.startswith("edge:"))
    out += ["STAT:clear_REF", "STAT:clear_UNDER", "STAT:clear_FDONE",
            "STAT:clear_at_set_edge_FDONE", "STAT:clear_at_set_edge_UNDER",
            "STAT:bits1_0_ignored"]
    out += ["STAT_read:%s" % b for b in ("BUSY", "RDY", "FDONE", "UNDER", "REF")]
    out += ["read:CSRR_ETX_STAT", "read:CSRR_ETX_CFG", "read:CSRR_write_only",
            "read:IN_13_14", "read:JP13", "read:JP14", "read:WAITP13", "read:WAITP14"]
    out += ["read:%s@%s" % (r, c) for r in READ_KINDS for c in ("start", "release")]
    out += ["feeder:halted_mid_frame", "feeder:frame_ended_after_halt",
            "feeder:reset_mid_frame"]
    out += ["pin16:written_while_busy", "pin17:written_while_busy"]
    return out


ETX_CSR_NAMES = ("ETX_CFG", "ETX_DATA", "ETX_LAST", "ETX_LAST1", "ETX_STAT")


class EtxCoverage:
    """Feeds :class:`EtxTracker` from the harness, one call per cycle, and
    hits the ``etx`` bins (``hit(group, name)`` is the coverage object's)."""

    def __init__(self, isa, hit):
        from tools.loomgen.generator import static_target
        self.isa, self.hit, self.static_target = isa, hit, static_target
        self.csr_names = {isa.csr_by_name[n]: n for n in ETX_CSR_NAMES}
        self.pin_out_csr = isa.csr_by_name["PIN_OUT"]
        self.tracker = EtxTracker()
        self.enabled = False
        self.mismatches = []
        self.mismatch_count = 0
        self.begin(None, ())

    def begin(self, seed, features) -> None:
        """A new seed: the chip was reset (6.12.1: all of it by ``rst_n``)."""
        self.enabled = "ETX" in tuple(features)
        self.seed = seed
        self.tracker.reset()
        self.prev_view = None
        self.starts, self.releases = set(), set()
        self.last_load = self.underrun_edge = None
        self.pending = []
        self.halted = self.frame_underrun = False
        self.writer = None

    def _mismatch(self, k, what, want, got) -> None:
        self.mismatch_count += 1
        if len(self.mismatches) < 50:
            self.mismatches.append("seed %s cycle %d: %s: SEMANTICS 6.12 (tracker) "
                                   "gives %s, both implementations %s"
                                   % (self.seed, k, what, want, got))

    def cycle(self, k, record, regs, ingrp, outgrp, pads) -> None:
        """Cycle ``k``: ``record`` is the model's slot in W (commits at edge
        ``k + 1``), ``regs``/``ingrp``/``outgrp`` its thread's state as seen
        in X, ``pads`` the RTL's ``uo_out[1:0]`` in cycle ``k``."""
        if not self.enabled:
            return
        t = self.tracker
        view = (t.stat, t.cfg, t.rdy, t.busy)
        p = t.pads
        if p is not None and pads != (p[0] | p[1] << 1):
            self._mismatch(k, "uo_out[1:0] while BUSY (6.12.7)", p[0] | p[1] << 1, pads)
        w = None
        if record is not None and self.prev_view is not None:
            w = self._record(k, record, regs, ingrp, outgrp)
        ev = t.edge(w)
        self._edge(k + 1, w, ev, view[0])
        self.prev_view = view

    # ------------------------------------------------------------ the slot
    def _at(self, kind: str, x: int) -> None:
        if x in self.starts:
            self.hit(GROUP, "read:%s@start" % kind)
        if x in self.releases:
            self.hit(GROUP, "read:%s@release" % kind)

    def _record(self, k, rec, regs, ingrp, outgrp):
        if getattr(rec, "mnemonic", None) in ("LD", "ST"):
            return None                      # a completion slot's ir is data (6.11)
        decoded = self.isa.decode(rec.ir)
        if decoded is None or not rec.done and decoded[0].name != "WAITP":
            return None
        name, f = decoded[0].name, decoded[1]
        x = k - 1
        stat, cfg, rdy, busy = self.prev_view                # as visible in cycle x
        pin = {13: rdy, 14: busy}
        if name in ("CSRR", "CSRW") and f["csr"] in self.csr_names:
            cname = self.csr_names[f["csr"]]
            if name == "CSRW":
                self.writer = rec.thread
                return (cname, regs[f["ra"]] & 0xFFFF)
            want = stat if cname == "ETX_STAT" else cfg if cname == "ETX_CFG" else 0
            if rec.val != want:
                self._mismatch(k, "CSRR %s in X cycle %d" % (cname, x), want, rec.val)
            if cname == "ETX_STAT":
                self.hit(GROUP, "read:CSRR_ETX_STAT")
                self._at("CSRR_ETX_STAT", x)
                for bit, label in enumerate(("BUSY", "RDY", "FDONE", "UNDER", "REF")):
                    if rec.val >> bit & 1:
                        self.hit(GROUP, "STAT_read:" + label)
            else:
                self.hit(GROUP, "read:CSRR_ETX_CFG" if cname == "ETX_CFG"
                         else "read:CSRR_write_only")
        elif name == "IN":
            base, cnt = ingrp & 0x1F, (ingrp >> 5) & 0x1F
            hits = [j for j in range(min(cnt, 16)) if (base + j) % 32 in (13, 14)]
            for j in hits:
                if (rec.val >> j) & 1 != pin[(base + j) % 32]:
                    self._mismatch(k, "IN bit %d (index %d)" % (j, (base + j) % 32),
                                   pin[(base + j) % 32], (rec.val >> j) & 1)
            if hits:
                self.hit(GROUP, "read:IN_13_14")
                self._at("IN_13_14", x)
        elif name == "JP" and f["pin"] in (13, 14):
            target = self.static_target(self.isa, rec.pc, rec.ir)
            if target is not None and target != (rec.pc + 1) & 0x3FF:
                taken = rec.next_pc == target
                if taken != (pin[f["pin"]] == f["val"]):
                    self._mismatch(k, "JP %d taken" % f["pin"], not taken, taken)
            self.hit(GROUP, "read:JP%d" % f["pin"])
            if f["pin"] == 14:
                self._at("JP14", x)
        elif name == "WAITP" and f["pin"] in (13, 14):
            self.hit(GROUP, "read:WAITP%d" % f["pin"])
            met = pin[f["pin"]] == f["val"]
            if (not rec.done and met) or (rec.done and not f["tmo"] and not met) \
                    or (rec.done and f["tmo"] and not rec.t and not met):
                self._mismatch(k, "WAITP %d,%d done=%d T=%d" % (f["pin"], f["val"], rec.done, rec.t),
                               "pin %d" % pin[f["pin"]], "that outcome")
        elif name == "HALT" and self.tracker.busy and rec.thread == self.writer:
            self.hit(GROUP, "feeder:halted_mid_frame")
            self.halted = True
        elif self.tracker.busy:
            self._pin_write(name, f, outgrp)
        return None

    def _pin_write(self, name, f, outgrp) -> None:
        """Pin writes to indices 16 and 17 that commit while ``BUSY``."""
        touched = ()
        if name == "SETP" and not f.get("lat"):
            touched = (f["pin"],)
        elif name == "OUT":
            base, cnt = outgrp & 0x1F, (outgrp >> 5) & 0x1F
            touched = tuple((base + j) % 32 for j in range(min(cnt, 16)))
        elif name == "CSRW" and f["csr"] == self.pin_out_csr:
            touched = (16, 17)
        for index in (16, 17):
            if index in touched:
                self.hit(GROUP, "pin%d:written_while_busy" % index)

    # ------------------------------------------------------------ the edge
    def _edge(self, e, w, ev, pre_stat) -> None:
        t, hit = self.tracker, self.hit
        if "cfg" in ev:
            if ev["cfg"] == "stored":
                value = w[1]
                hit(GROUP, "CFG:stored")
                hit(GROUP, "CFG:N=%d" % ((value & 0xF) + 1))
                hit(GROUP, "CFG:PRE%d" % ((value >> 4) & 1))
                hit(GROUP, "CFG:FCS%d" % ((value >> 5) & 1))
                if not (value >> 6) & 0xF:
                    hit(GROUP, "CFG:IDL=0")
                if value >> 10:
                    hit(GROUP, "CFG:bits15_10_ignored")
            else:
                hit(GROUP, "CFG:refused_busy")
        if "stat" in ev:
            value = ev["stat"]
            for bit, label in ((4, "REF"), (3, "UNDER"), (2, "FDONE")):
                if value >> bit & 1 and pre_stat >> bit & 1:
                    hit(GROUP, "STAT:clear_" + label)
            if "clear_vs_set" in ev:
                hit(GROUP, "STAT:clear_at_set_edge_" + ev["clear_vs_set"])
            if value & 3:
                hit(GROUP, "STAT:bits1_0_ignored")
        if ev.get("load_edge"):
            for pe, what in self.pending:
                d = e - pe
                if what == "accepted" and d in (1, 4):
                    hit(GROUP, "edge:K-1_accepted_last_edge" if d == 1
                        else "edge:K-4_accepted_last_slot")
                elif what == "hold_full" and d in (1, 4):
                    hit(GROUP, "edge:K-%d_refused_hold_full" % d)
            self.pending = []
            self.last_load = e
            if ev.get("underrun"):
                self.underrun_edge = e
                self.frame_underrun = True
        if "data" in ev:
            self._data(e, w, ev)
        if ev.get("release"):
            self.releases.add(e)
            hit(GROUP, "frame:end_underrun" if self.frame_underrun else "frame:end_normal")
            if ev.get("underrun"):
                hit(GROUP, "frame:underrun_and_FDONE_same_edge")
            if self.halted:
                hit(GROUP, "feeder:frame_ended_after_halt")
            self.halted = False

    def _data(self, e, w, ev) -> None:
        t, hit = self.tracker, self.hit
        status, why = ev["data"]
        kind = w[0]
        if status == "start":
            n = t.halfn + 1
            hit(GROUP, "frame:N=%d" % n)
            hit(GROUP, "frame:PRE%d_FCS%d" % (t.pre, t.fcs))
            idl = t.idl
            hit(GROUP, "frame:IDL=0" if idl == 0 else "frame:IDL1-4" if idl <= 4
                else "frame:IDL5-8" if idl <= 8 else "frame:IDL9-15")
            self.starts.add(e)
            self.last_load = None if t.pre else e          # K_0 = s without PRE
            self.pending, self.underrun_edge = [], None
            self.halted = self.frame_underrun = False
            if kind != "ETX_DATA":
                hit(GROUP, "frame:last_1byte" if kind == "ETX_LAST1" else "frame:last_2bytes")
            return
        if status == "accepted":
            hit(GROUP, "accept:" + why)
            if why == "at_load_HV1":
                hit(GROUP, "edge:K_accepted_HV1")
            elif self.last_load is not None and e - self.last_load in (1, 4):
                hit(GROUP, "edge:K+%d_accepted" % (e - self.last_load))
            self.pending.append((e, "accepted"))
            if kind != "ETX_DATA":
                hit(GROUP, "frame:last_1byte" if kind == "ETX_LAST1" else "frame:last_2bytes")
            return
        hit(GROUP, "refuse:" + why)
        if why == "underrun_edge":
            hit(GROUP, "edge:underrun_K_refused")
        elif self.underrun_edge is not None and e - self.underrun_edge in (1, 4):
            hit(GROUP, "edge:underrun_K+%d_refused" % (e - self.underrun_edge))
        if why in ("hold_full", "before_K0"):
            self.pending.append((e, "hold_full"))
