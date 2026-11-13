"""Blocks that drive the Ethernet serializer (``docs/SEMANTICS.md`` 6.12, D-036).

Written from SEMANTICS 6.12 alone (METH-1): nothing here looks at
``tools/loomsim`` or ``src/``. In a build with the serializer (``CAPS[10]``,
feature ``ETX``) one running thread is the **writer**: only it writes the ETX
CSRs (6.12.7, "one thread writes the ETX CSRs"; two writers are untested,
6.12.9). The other threads are **observers**: they read ``ETX_STAT``,
``ETX_CFG``, ``pin_in(13)`` (``RDY``) and ``pin_in(14)`` (``BUSY``).

A writer's frame block
----------------------

``[wait for idle] [CSRR/clear ETX_STAT] CSRW ETX_CFG; CSRW ETX_DATA (the
start, edge s) ... words ... [the end of the frame]``. Everything after the
start write is straight-line code of one-slot instructions and ``DJNZ``
delay loops, so the edge at which each later write commits is known exactly:
the ``j``-th slot after the start commits at ``s + 4 j`` (SEMANTICS 2). With
``N``, ``PRE``, ``FCS`` and ``IDL`` chosen by the block, 6.12.6 gives every
load edge ``K_m``, the tail and the release, and the block aims writes at
window ends and one write slot either side of them, at the underrun edge,
inside the tail and at the release edge, with random gaps in between. A
paced variant waits on ``WAITP 13`` (``RDY``) instead, as firmware does;
after the first wait the block stops aiming.

One writer commits only at edges ``= s (mod 4)``, and every ``K_m`` is one
too (``K_m - s`` is a multiple of ``32 N``), so the edges ``K - 1`` and
``K + 1`` are out of its reach; ``K - 4`` and ``K + 4`` are the nearest.

Registers: ``r7`` (the scratch) carries every value; a ``DJNZ`` delay uses
one value register as its counter.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

#: The ETX CSRs by name (isa.yaml 0x16..0x1A).
ETX_CSRS = ("ETX_CFG", "ETX_DATA", "ETX_LAST", "ETX_LAST1", "ETX_STAT")
PIN_RDY, PIN_BUSY = 13, 14
#: ``ETX_STAT`` write-1-to-clear bits: REF, UNDER, FDONE (6.12.2).
STAT_W1C = 0x1C
STAT_VALUES = (0x1C, 0x1C, 0x1F, 0x04, 0x08, 0x10, 0x0C, 0x18, 0x14, 0x03, 0x00)


def cfg_value(halfn: int, pre: int, fcs: int, idl: int) -> int:
    """``ETX_CFG = {IDL[9:6], FCS[5], PRE[4], HALFN[3:0]}`` (6.12.2)."""
    return (halfn & 0xF) | (pre & 1) << 4 | (fcs & 1) << 5 | (idl & 0xF) << 6


class Schedule:
    """The edges of a frame relative to its start edge ``s = 0`` (6.12.6),
    for ``words`` words of which the last has ``last_bytes`` bytes."""

    def __init__(self, n: int, pre: int, fcs: int, idl: int):
        self.n, self.pre, self.fcs, self.idl = n, pre, fcs, idl

    def k(self, m: int) -> int:
        """The load edge of word ``m`` (every earlier word full)."""
        return 128 * self.n * self.pre + 32 * self.n * m

    def window(self, m: int) -> Tuple[int, int]:
        """Edges at which word ``m >= 1`` is accepted, both ends included."""
        lo = self.k(m - 1) if (self.pre or m > 1) else 1
        return lo, self.k(m) - 1

    def tail(self, words: int, last_bytes: int) -> int:
        """The edge the tail starts at after a normal end."""
        nbits = 64 * self.pre + 16 * (words - 1) + 8 * last_bytes + 32 * self.fcs
        return 2 * self.n * nbits

    def release(self, tail_start: int) -> int:
        return tail_start + self.n * self.idl


class Seq:
    """Words of one block with exact slot accounting from the start write.

    ``slot`` is the slot of the last word emitted, counted from the start
    write (slot 0), or None while timing is unknown (before the start, and
    after any wait)."""

    def __init__(self, b):
        self.b = b
        self.words: List = []
        self.slot: Optional[int] = None
        self.counter = b.rng.choice(tuple(range(0, 7)))

    def add(self, word, slots: int = 1) -> None:
        self.words.append(word)
        if self.slot is not None:
            self.slot += slots

    def one(self, name: str, **ops) -> None:
        self.add(self.b.enc(name, **ops))

    def unknown(self) -> None:
        self.slot = None

    def delay(self, d: int) -> None:
        """Exactly ``d`` slots of nothing: ``LDI c, n; NOP; DJNZ c, -2`` is
        ``1 + 2 n`` slots (SEMANTICS 6.2: DJNZ branches while the new count
        is not 0), then up to three NOPs."""
        while d >= 4:
            n = min((d - 1) // 2, 255)
            self.one("LDI", rd=self.counter, imm=n)
            self.add(self.b.enc("NOP"))
            self.add(self.b.enc("DJNZ", rd=self.counter, rel=-2), slots=0)
            if self.slot is not None:
                self.slot += 2 * n - 1          # NOP + DJNZ, n times, minus the two counted
            d -= 1 + 2 * n
        for _ in range(d):
            self.one("NOP")

    def load(self, value: int) -> List:
        return self.b.load_scratch(value & 0xFFFF)

    def csrw(self, name: str, value: int, at: Optional[int] = None) -> None:
        """``CSRW name`` with ``value``; with ``at`` and known timing, padded
        so that the ``CSRW`` is slot ``at`` (or the first one after it that
        the loads allow)."""
        words = self.load(value)
        if at is not None and self.slot is not None:
            self.delay(at - (self.slot + 1) - len(words))
        for w in words:
            self.add(w)
        self.one("CSRW", csr=self.b.csr[name], ra=7)

    def cost(self, value: int) -> int:
        return len(self.load(value)) + 1


# ------------------------------------------------------------ configuration
def choose_frame(rng):
    """``(halfn, pre, fcs, idl, words, last1)``: every N 1..16, small N most
    of the time; long frames only at small N, so a run stays short."""
    if rng.random() < 0.6:
        halfn = rng.choice((0, 0, 1, 1, 1, 2, 3))
    else:
        halfn = rng.randrange(16)
    n = halfn + 1
    pre = 1 if rng.random() < (0.6 if n <= 3 else 0.2) else 0
    fcs = 1 if rng.random() < (0.6 if n <= 4 else 0.3) else 0
    idl = rng.choice((0, 0, 0, 1, 2, 3, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 11, 13, 15))
    words = rng.choice((1, 1, 2, 2, 3, 4)) if n <= 3 else rng.choice((1, 1, 2))
    last1 = rng.random() < 0.3
    sch = Schedule(n, pre, fcs, idl)
    while words > 1 and sch.tail(words, 2) > 1600:
        words -= 1
    if sch.tail(words, 2) > 1600:
        pre = 0
    return halfn, pre, fcs, idl, words, last1


def _extra(seq: Seq, room: int) -> None:
    """Something else for the writer to do inside a frame, if ``room``
    slots are free before the next aimed write: read ``ETX_STAT``, write
    ``PIN_OUT[8]``/``[9]`` (indices 16, 17: they reach the pad only once the
    block is idle, 6.12.7), write ``ETX_CFG`` while busy (refused, ``REF``)
    or clear ``ETX_STAT``."""
    b, rng = seq.b, seq.b.rng
    roll = rng.random()
    if roll < 0.3 and room >= 1:
        seq.one("CSRR", rd=b.reg(), csr=b.csr["ETX_STAT"])
    elif roll < 0.6 and room >= 1:
        seq.one("SETP", pin=rng.choice((16, 17)), val=rng.randrange(2), lat=0)
    elif roll < 0.8:
        value = cfg_value(rng.randrange(16), rng.randrange(2), rng.randrange(2),
                          rng.randrange(16))
        if seq.cost(value) <= room:
            seq.csrw("ETX_CFG", value)
    else:
        value = rng.choice(STAT_VALUES)
        if seq.cost(value) <= room:
            seq.csrw("ETX_STAT", value)


def frame_block(b) -> List:
    """One frame of the writer thread (module docstring)."""
    rng = b.rng
    seq = Seq(b)
    roll = rng.random()
    if roll < 0.75:
        seq.one("JP", pin=PIN_BUSY, val=1, rel=-1)       # spin while BUSY
    elif roll < 0.9:                                     # may time out: CFG refused
        seq.one("SETD", imm=b.small_imm())
        seq.one("WAITP", pin=PIN_BUSY, val=0, tmo=1)
    if rng.random() < 0.35:
        seq.one("CSRR", rd=b.reg(), csr=b.csr["ETX_STAT"])
    if rng.random() < 0.75:
        seq.csrw("ETX_STAT", rng.choice(STAT_VALUES))
    halfn, pre, fcs, idl, words, last1 = choose_frame(rng)
    cfg = cfg_value(halfn, pre, fcs, idl)
    if rng.random() < 0.12:
        cfg |= rng.randrange(1, 64) << 10                # bits 15:10 are not stored
    seq.csrw("ETX_CFG", cfg)
    sch = Schedule(halfn + 1, pre, fcs, idl)
    ending = rng.choices(("last", "underrun", "halt"), (0.72, 0.24, 0.04))[0]

    def kind(m: int) -> str:
        if m < words - 1 or ending != "last":
            return "ETX_DATA"
        return "ETX_LAST1" if last1 else "ETX_LAST"

    values = [rng.randrange(1 << 16) for _ in range(words)]
    seq.csrw(kind(0), values[0])
    seq.slot = 0                                         # the start: edge s
    paced = rng.random() < 0.3
    for m in range(1, words):
        lo, hi = sch.window(m)
        lo4, hi4 = (lo + 3) // 4, hi // 4
        if paced:
            seq.one("SETD", imm=rng.choice((4, 8, 12, 16, 24, 32)))
            seq.one("WAITP", pin=PIN_RDY, val=1, tmo=1)
            seq.unknown()
            if rng.random() < 0.3:
                _extra(seq, 3)
            seq.csrw(kind(m), values[m])
            continue
        mode = rng.choices(("first", "after", "last", "before", "random", "miss"),
                           (5, 3, 5, 2, 5, 2))[0]
        if mode == "before" and lo4 - 1 > 0:
            seq.csrw(kind(m), values[m], at=lo4 - 1)     # HOLD still full: refused
            mode = "random"
        target = {"first": lo4, "after": lo4 + 1, "last": hi4, "miss": hi4 + 1,
                  "random": rng.randrange(lo4, hi4 + 1) if hi4 >= lo4 else lo4,
                  "before": lo4}[mode]
        if seq.slot is not None and rng.random() < 0.3:
            _extra(seq, target - seq.slot - 1 - seq.cost(values[m]))
        seq.csrw(kind(m), values[m], at=target)
        if mode == "miss":                               # the underrun edge itself
            ending = "underrun_late"
            break
    if ending == "halt":
        seq.one("HALT")                                  # the feeder stops mid-frame
        return seq.words
    _frame_end(seq, sch, words, last1, ending)
    return seq.words


def _frame_end(seq: Seq, sch: Schedule, words: int, last1: bool, ending: str) -> None:
    """Writes aimed at the end of the frame (6.12.6): after the last word,
    at the underrun edge, inside ``P_TAIL``, at the release edge (only
    reachable when ``N * IDL`` is a multiple of 4) and right after it."""
    b, rng = seq.b, seq.b.rng
    data_kinds = ("ETX_DATA", "ETX_LAST", "ETX_LAST1")
    if ending == "last":
        if rng.random() < 0.25:
            seq.csrw(rng.choice(data_kinds), rng.randrange(1 << 16))   # after the last word
        tail = sch.tail(words, 1 if last1 else 2)
    elif ending == "underrun":
        tail = sch.k(words)                          # word `words` is never written
        roll = rng.random()
        if seq.slot is not None and tail // 4 >= seq.slot + 3:
            if roll < 0.3:
                seq.csrw("ETX_DATA", rng.randrange(1 << 16), at=tail // 4)
            elif roll < 0.6:
                seq.csrw("ETX_STAT", rng.choice((0x08, 0x1C, 0x1F)), at=tail // 4)
    else:                                            # underrun_late: the miss write was at K
        tail = 4 * seq.slot if seq.slot is not None else 0
    release = sch.release(tail)
    under = ending != "last"
    if seq.slot is not None:
        events = []
        if release - tail >= 12 and rng.random() < 0.4:
            events.append((rng.randrange(tail // 4 + 1, (release - 1) // 4 + 1), "tail"))
        if release % 4 == 0 and rng.random() < 0.7:
            events.append((release // 4, rng.choice(("data", "stat"))))
        if under and rng.random() < 0.4:
            events.append((release // 4 + rng.randrange(1, 4), "data"))
        for at, what in sorted(events):
            value = rng.randrange(1 << 16) if what != "stat" else rng.choice((0x04, 0x1C, 0x1F))
            if at < seq.slot + seq.cost(value):
                continue
            seq.csrw("ETX_STAT" if what == "stat" else rng.choice(data_kinds), value, at=at)
    if rng.random() < 0.5:
        seq.one("JP", pin=PIN_BUSY, val=1, rel=-1)
        seq.unknown()
        if rng.random() < 0.6:
            seq.one("CSRR", rd=b.reg(), csr=b.csr["ETX_STAT"])
        if rng.random() < 0.4:
            seq.csrw("ETX_STAT", STAT_W1C)


# ------------------------------------------------------------- observers
def observer_block(b) -> List:
    """A read of the serializer by a thread that does not write it: the
    status CSRs, ``IN`` over indices 13..14, ``JP 13``/``JP 14`` and timed
    ``WAITP``s, so that reads land in the cycles of starts and releases."""
    rng = b.rng
    roll = rng.random()
    if roll < 0.25:
        return [b.enc("CSRR", rd=b.reg(), csr=b.csr["ETX_STAT"])]
    if roll < 0.33:
        return [b.enc("CSRR", rd=b.reg(), csr=b.csr["ETX_CFG"])]
    if roll < 0.36:
        name = rng.choice(("ETX_DATA", "ETX_LAST", "ETX_LAST1"))   # write-only: reads 0
        return [b.enc("CSRR", rd=b.reg(), csr=b.csr[name])]
    if roll < 0.54:
        base, count = rng.choice(((13, 2), (13, 2), (12, 4), (14, 1), (13, 1), (8, 8)))
        return b.load_scratch(base | count << 5) + [
            b.enc("CSRW", csr=b.csr["INGRP"], ra=7), b.enc("IN", rd=b.reg())]
    if roll < 0.69:
        return [b.fixup("rel6", "JP", "fwd", pin=rng.choice((PIN_RDY, PIN_BUSY, PIN_BUSY)),
                        val=rng.randrange(2))]
    if roll < 0.81:
        return [b.enc("JP", pin=PIN_BUSY, val=1, rel=-1)]           # spin while BUSY
    words = [b.enc("SETD", imm=rng.choice((2, 4, 8, 16, 32)))]
    words.append(b.enc("WAITP", pin=rng.choice((PIN_RDY, PIN_BUSY)),
                       val=rng.randrange(2), tmo=1))
    if rng.random() < 0.5:
        words.append(b.enc("CSRR", rd=b.reg(), csr=b.csr["ETX_STAT"]))
    return words
