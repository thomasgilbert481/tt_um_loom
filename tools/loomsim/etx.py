"""The Ethernet serializer of ``docs/SEMANTICS.md`` 6.12 (D-036), feature ``"ETX"``.

One shared transmit block: 16-bit words arrive by ``CSRW`` to ``ETX_DATA``,
``ETX_LAST`` or ``ETX_LAST1``; the block sends them Manchester coded with the
preamble, the SFD and the FCS added, ends the frame with TP_IDL, and drives
TD+ and TD- on ``uo_out[0]`` and ``uo_out[1]`` while it is busy (6.12.7).

This file is a literal transcription of 6.12.4 (every edge) and 6.12.5 (the
advance at a half-bit boundary), the two normative subsections.  Every
right-hand side reads the pre-edge snapshot ``o``; every assignment goes to
``self``, so two rules that touch one register at one edge resolve by the
order the text gives (the accepted write after the load for ``HOLD``/``HV``,
the sticky set against an ``ETX_STAT`` clear).  Written from SEMANTICS only,
never from ``src/`` (``docs/VERIFICATION.md``, METH-1).  The state names are
those of 6.12.1 in lower case.
"""

from __future__ import annotations

import types
from typing import Optional, Tuple

#: Phases (``EPH``), 6.12.1.
P_IDLE, P_PRE, P_DATA, P_FCS, P_TAIL = range(5)
PHASE_NAMES = ("P_IDLE", "P_PRE", "P_DATA", "P_FCS", "P_TAIL")

#: CSR numbers, 6.12.2 (also in ``isa/isa.yaml``; the machine maps by these).
CSR_ETX_CFG = 0x16
CSR_ETX_DATA = 0x17
CSR_ETX_LAST = 0x18
CSR_ETX_LAST1 = 0x19
CSR_ETX_STAT = 0x1A
ETX_CSRS = frozenset(range(CSR_ETX_CFG, CSR_ETX_STAT + 1))
DATA_CSRS = frozenset((CSR_ETX_DATA, CSR_ETX_LAST, CSR_ETX_LAST1))

#: Pin indices of the two status pins, 6.12.7.
PIN_RDY = 13
PIN_BUSY = 14

CRC32_POLY = 0x04C11DB7
CRC32_INIT = 0xFFFFFFFF
#: Reset values of 6.12.1.
RESET_HALFN, RESET_PRE, RESET_FCS, RESET_IDL = 1, 1, 1, 6

_FIELDS = ("halfn", "pre", "fcs", "idl", "eph", "div", "hb", "pos", "txb",
           "txl", "sh", "sl", "s1", "hold", "hv", "hl", "h1", "lseen",
           "crc32", "fdone", "under", "ref")


def crc32_step(crc: int, v: int) -> int:
    """The update of 6.12.3: ``fb = CRC32[31] ^ v``, shift left, XOR the poly."""
    fb = ((crc >> 31) & 1) ^ (v & 1)
    crc = (crc << 1) & 0xFFFFFFFF
    return crc ^ CRC32_POLY if fb else crc


def preamble_bit(k: int) -> int:
    """``p_k`` of 6.12.3: 1 for even ``k``, 0 for odd, except ``p_63 = 1``."""
    return 1 if (k % 2 == 0 or k == 63) else 0


class Etx:
    """The serializer's global state (6.12.1) and its edge function."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """``rst_n``: the reset column of 6.12.1.  Nothing else resets it."""
        self.halfn, self.pre, self.fcs, self.idl = (
            RESET_HALFN, RESET_PRE, RESET_FCS, RESET_IDL)
        self.eph = P_IDLE
        self.div = self.hb = self.pos = self.txb = self.txl = 0
        self.sh = self.sl = self.s1 = 0
        self.hold = self.hv = self.hl = self.h1 = 0
        self.lseen = 0
        self.crc32 = 0
        self.fdone = self.under = self.ref = 0

    # ------------------------------------------------------------ views
    @property
    def n(self) -> int:
        """``N = HALFN + 1`` clocks a half bit."""
        return self.halfn + 1

    @property
    def busy(self) -> int:
        return int(self.eph != P_IDLE)

    @property
    def rdy(self) -> int:
        """``RDY`` of 6.12.1 (a hint, not the acceptance rule)."""
        return int(not self.under and (
            self.eph == P_IDLE
            or (self.eph in (P_PRE, P_DATA) and not self.lseen and not self.hv)))

    @property
    def cfg(self) -> int:
        """``ETX_CFG`` read value: ``{IDL[3:0], FCS, PRE, HALFN[3:0]}``."""
        return ((self.idl & 0xF) << 6) | ((self.fcs & 1) << 5) \
            | ((self.pre & 1) << 4) | (self.halfn & 0xF)

    @property
    def stat(self) -> int:
        """``ETX_STAT`` read value: ``{REF, UNDER, FDONE, RDY, BUSY}``."""
        return ((self.ref & 1) << 4) | ((self.under & 1) << 3) \
            | ((self.fdone & 1) << 2) | (self.rdy << 1) | self.busy

    def read(self, csr: int) -> int:
        """``CSRR`` value of an ETX CSR (the three data CSRs are write-only)."""
        if csr == CSR_ETX_CFG:
            return self.cfg
        if csr == CSR_ETX_STAT:
            return self.stat
        return 0

    def pads(self, pin_out: int) -> Tuple[int, int]:
        """``(uo_out[0], uo_out[1])`` this cycle (6.12.7); ``pin_out`` is the register."""
        if self.eph == P_IDLE:
            return (pin_out >> 8) & 1, (pin_out >> 9) & 1
        if self.eph == P_TAIL:
            return 1, 0
        return self.txl & 1, (~self.txl) & 1

    def phase_name(self) -> str:
        return PHASE_NAMES[self.eph]

    def snapshot(self) -> types.SimpleNamespace:
        """Every 6.12.1 register by name, as a value (the pre-edge view)."""
        return types.SimpleNamespace(**{f: getattr(self, f) for f in _FIELDS})

    # ------------------------------------------------------------ the edge
    def edge(self, write: Optional[Tuple[int, int]] = None) -> None:
        """Apply one rising edge (6.12.4); ``write`` is ``(csr, value)`` of the
        ETX write committing at this edge, if any (at most one, 6.12.2)."""
        o = self.snapshot()
        self._sets = set()                     # sticky bits set at this edge
        csr, value = write if write is not None else (None, 0)
        value &= 0xFFFF
        data = csr in DATA_CSRS
        last = int(csr in (CSR_ETX_LAST, CSR_ETX_LAST1))
        one = int(csr == CSR_ETX_LAST1)
        if o.eph == P_IDLE:
            if csr == CSR_ETX_CFG:
                value &= 0x3FF
                self.halfn = value & 0xF
                self.pre = (value >> 4) & 1
                self.fcs = (value >> 5) & 1
                self.idl = (value >> 6) & 0xF
            elif data:
                if o.under:
                    self._set("ref")
                else:
                    self._start(o, value, last, one)
        else:
            took_hold = False
            if o.div != 0:                     # not bnd
                self.div = o.div - 1
            else:
                took_hold = self._advance(o)
            if csr == CSR_ETX_CFG:
                self._set("ref")               # nothing stored
            elif data:
                if (o.eph in (P_PRE, P_DATA) and self.eph in (P_PRE, P_DATA)
                        and not o.lseen and not o.under
                        and (not o.hv or took_hold)):
                    self.hold, self.hv, self.hl, self.h1 = value, 1, last, one
                    self.lseen = last
                else:
                    self._set("ref")
        if csr == CSR_ETX_STAT:
            # Write 1 to clear bits 4:2; a bit set at this same edge stays set.
            # "Set" is the set event, whatever the bit held before (W-117 Q1).
            for bit, name in ((4, "ref"), (3, "under"), (2, "fdone")):
                if (value >> bit) & 1 and name not in self._sets:
                    setattr(self, name, 0)

    def _set(self, name: str) -> None:
        setattr(self, name, 1)
        self._sets.add(name)

    def _start(self, o, w: int, last: int, one: int) -> None:
        """**Start** of 6.12.4, at the edge of the data write that starts a frame."""
        n = o.halfn + 1
        self.lseen = last
        self.div = n - 1
        self.pos = 0
        if o.pre:
            self.eph = P_PRE
            self.hold, self.hv, self.hl, self.h1 = w, 1, last, one
            self.crc32 = CRC32_INIT
            self._begin(o, preamble_bit(0), data=False)
        else:
            self.eph = P_DATA
            self.sh, self.sl, self.s1, self.hv = w, last, one, 0
            self._begin(o, w & 1, data=False)
            # One assignment: the first data bit's update applied to all ones.
            self.crc32 = crc32_step(CRC32_INIT, w & 1)

    def _begin(self, o, v: int, data: bool) -> None:
        """**Begin bit v**: ``TXB``, ``HB``, ``DIV``, ``TXL`` and, for a data
        bit (other than at the start), the CRC update from the pre-edge CRC."""
        self.txb = v & 1
        self.hb = 0
        self.div = o.halfn
        self.txl = (~v) & 1
        if data:
            self.crc32 = crc32_step(o.crc32, v)

    @staticmethod
    def _last_bit(o) -> bool:
        """At the word's last bit: ``POS == 15``, or ``POS == 7`` if ``S1``."""
        return o.pos == (7 if o.s1 else 15)

    def _advance(self, o) -> bool:
        """6.12.5 at an edge where ``bnd`` holds.  True iff the advance is a
        load that takes ``HOLD`` (the second case of the accept rule)."""
        n = o.halfn + 1
        if o.eph in (P_PRE, P_DATA, P_FCS) and o.hb == 0:            # 1
            self.hb, self.div, self.txl = 1, n - 1, o.txb
        elif o.eph == P_PRE and o.pos < 63:                          # 2
            self.pos = o.pos + 1
            self._begin(o, preamble_bit(o.pos + 1), data=False)
        elif ((o.eph == P_PRE and o.pos == 63)                       # 3
              or (o.eph == P_DATA and self._last_bit(o) and not o.sl)):
            if o.hv:
                self.sh, self.sl, self.s1, self.hv = o.hold, o.hl, o.h1, 0
                self.pos = 0
                self.eph = P_DATA
                self._begin(o, o.hold & 1, data=True)
                return True
            self._set("under")                                       # underrun
            self._enter_tail(o)
        elif o.eph == P_DATA and not self._last_bit(o):              # 4
            self.pos = o.pos + 1
            self._begin(o, (o.sh >> (o.pos + 1)) & 1, data=True)
        elif o.eph == P_DATA:                                        # 5 (SL == 1)
            if o.fcs:
                self.eph = P_FCS
                self.pos = 0
                self._begin(o, (~o.crc32 >> 31) & 1, data=False)
            else:
                self._enter_tail(o)
        elif o.eph == P_FCS:                                         # 6
            if o.pos < 31:
                self.pos = o.pos + 1
                self._begin(o, (~o.crc32 >> (30 - o.pos)) & 1, data=False)
            else:
                self._enter_tail(o)
        elif o.eph == P_TAIL:                                        # 7
            if o.pos + 1 == o.idl:
                self._release()
            else:
                self.pos = o.pos + 1
                self.div = n - 1
        return False

    def _enter_tail(self, o) -> None:
        self.hv = 0
        if o.idl == 0:
            self._release()
        else:
            self.eph = P_TAIL
            self.pos = 0
            self.div = o.halfn

    def _release(self) -> None:
        """Writes neither ``UNDER`` nor ``REF`` (an underrun's ``UNDER`` stands)."""
        self.eph = P_IDLE
        self.hv = 0
        self.lseen = 0
        self._set("fdone")
