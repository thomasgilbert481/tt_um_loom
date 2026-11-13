# SPDX-License-Identifier: Apache-2.0
"""A literal transcription of docs/SEMANTICS.md 6.12.4 and 6.12.5 (the
Ethernet serializer, D-036), written by the RTL agent from the text alone
for the L1 tests in test_etx.py. It is not the golden model
(tools/loomsim, METH-1): it keeps every register of 6.12.1 under its own
name, with CRC32 and SH exactly as the text has them, so where the RTL
encodes them differently (loom_etx.v header) only the observable values
are compared: ETX_CFG, ETX_STAT, RDY, BUSY and the pad pair."""

IDLE, PRE, DATA, FCS, TAIL = "P_IDLE", "P_PRE", "P_DATA", "P_FCS", "P_TAIL"
POLY = 0x04C11DB7
KINDS = ("CFG", "DATA", "LAST", "LAST1", "STAT")


def crc_step(c, v):
    """6.12.3: one data bit v into CRC32."""
    fb = ((c >> 31) & 1) ^ v
    return (((c << 1) & 0xFFFFFFFF) ^ (POLY if fb else 0))


class EtxRef:
    FIELDS = ("halfn", "pre", "fcs", "idl", "eph", "div", "hb", "pos", "txb",
              "txl", "sh", "sl", "s1", "hold", "hv", "hl", "h1", "lseen",
              "crc", "fdone", "under", "ref")

    def __init__(self):
        self.halfn, self.pre, self.fcs, self.idl = 1, 1, 1, 6
        self.eph = IDLE
        for f in self.FIELDS[5:]:
            setattr(self, f, 0)

    # ---------------------------------------------------------- views
    @property
    def busy(self):
        return int(self.eph != IDLE)

    @property
    def rdy(self):
        return int(not self.under and (
            self.eph == IDLE
            or (self.eph in (PRE, DATA) and not self.lseen and not self.hv)))

    @property
    def cfg(self):
        return (self.idl << 6) | (self.fcs << 5) | (self.pre << 4) | self.halfn

    @property
    def stat(self):
        return ((self.ref << 4) | (self.under << 3) | (self.fdone << 2)
                | (self.rdy << 1) | self.busy)

    def pads(self, pin_out=(0, 0)):
        """(uo_out[0], uo_out[1]) in this cycle (6.12.7)."""
        if self.eph == IDLE:
            return pin_out
        if self.eph == TAIL:
            return (1, 0)
        return (self.txl, 1 - self.txl)

    # ----------------------------------------------------------- edge
    def edge(self, w=None):
        """One edge. w is None or (kind, value), kind in KINDS."""
        o = {f: getattr(self, f) for f in self.FIELDS}   # pre-edge values
        self._o = o
        self._sets = set()
        kind, val = w if w else (None, 0)
        val &= 0xFFFF
        data = kind in ("DATA", "LAST", "LAST1")
        last, one = int(kind != "DATA"), int(kind == "LAST1")
        n1 = o["halfn"]                                 # N - 1
        if o["eph"] == IDLE:
            if kind == "CFG":
                self.halfn, self.pre = val & 15, (val >> 4) & 1
                self.fcs, self.idl = (val >> 5) & 1, (val >> 6) & 15
            elif data:
                if o["under"]:
                    self._sets.add("ref")
                else:
                    self._start(val, last, one, n1)
        else:
            take = False
            if o["div"] != 0:
                self.div = o["div"] - 1
            else:
                take = self._advance(n1)
            if kind == "CFG":
                self._sets.add("ref")
            elif data:
                ok = (o["eph"] in (PRE, DATA) and self.eph in (PRE, DATA)
                      and not o["lseen"] and not o["under"]
                      and (not o["hv"] or take))
                if ok:
                    self.hold, self.hv, self.hl, self.h1 = val, 1, last, one
                    self.lseen = last
                else:
                    self._sets.add("ref")
        clr = (val >> 2) & 7 if kind == "STAT" else 0
        for bit, f in ((4, "ref"), (2, "under"), (1, "fdone")):
            if f in self._sets:
                setattr(self, f, 1)
            elif clr & bit:
                setattr(self, f, 0)

    def _begin(self, v, n1, data, base=None):
        self.txb, self.hb, self.div, self.txl = v, 0, n1, 1 - v
        if data:
            self.crc = crc_step(self._o["crc"] if base is None else base, v)

    def _start(self, val, last, one, n1):
        o = self._o
        self.lseen, self.div, self.pos = last, n1, 0
        if o["pre"]:
            self.eph = PRE
            self.hold, self.hv, self.hl, self.h1 = val, 1, last, one
            self.crc = 0xFFFFFFFF
            self._begin(1, n1, False)
        else:
            self.eph = DATA
            self.sh, self.sl, self.s1, self.hv = val, last, one, 0
            self._begin(val & 1, n1, True, base=0xFFFFFFFF)

    def _at_last(self):
        o = self._o
        return o["pos"] == (7 if o["s1"] else 15)

    def _advance(self, n1):
        """6.12.5 at a boundary. Returns True if a load takes HOLD."""
        o = self._o
        e, hb, pos = o["eph"], o["hb"], o["pos"]
        if e in (PRE, DATA, FCS) and hb == 0:                     # rule 1
            self.hb, self.div, self.txl = 1, n1, o["txb"]
        elif e == PRE and pos < 63:                               # rule 2
            self.pos = pos + 1
            k = pos + 1
            self._begin(1 if (k % 2 == 0 or k == 63) else 0, n1, False)
        elif ((e == PRE and pos == 63)
              or (e == DATA and self._at_last() and not o["sl"])):  # rule 3
            if o["hv"]:
                self.sh, self.sl, self.s1 = o["hold"], o["hl"], o["h1"]
                self.hv, self.pos, self.eph = 0, 0, DATA
                self._begin(o["hold"] & 1, n1, True)
                return True
            self._sets.add("under")
            self._tail(n1)
        elif e == DATA and not self._at_last():                   # rule 4
            self.pos = pos + 1
            self._begin((o["sh"] >> (pos + 1)) & 1, n1, True)
        elif e == DATA:                                           # rule 5
            if o["fcs"]:
                self.eph, self.pos = FCS, 0
                self._begin(1 - ((o["crc"] >> 31) & 1), n1, False)
            else:
                self._tail(n1)
        elif e == FCS:                                            # rule 6
            if pos < 31:
                self.pos = pos + 1
                self._begin(1 - ((o["crc"] >> (30 - pos)) & 1), n1, False)
            else:
                self._tail(n1)
        elif e == TAIL:                                           # rule 7
            if pos + 1 == o["idl"]:
                self._release()
            else:
                self.pos, self.div = pos + 1, n1
        return False

    def _tail(self, n1):
        self.hv = 0
        if self._o["idl"] == 0:
            self._release()
        else:
            self.eph, self.pos, self.div = TAIL, 0, n1

    def _release(self):
        self.eph, self.hv, self.lseen = IDLE, 0, 0
        self._sets.add("fdone")


# ------------------------------------------------- independent checks
def frame_bits(words, last_one, pre, fcs):
    """The bit sequence of 6.12.3 computed from IEEE 802.3 conventions
    directly (octets LSB first, FCS = zlib.crc32 sent LSB first), not
    from the edge rules: what the wire must carry."""
    import zlib
    data = bytearray()
    for i, w in enumerate(words):
        data.append(w & 0xFF)
        if not (i == len(words) - 1 and last_one):
            data.append((w >> 8) & 0xFF)
    bits = []
    if pre:
        for b in [0x55] * 7 + [0xD5]:
            bits += [(b >> k) & 1 for k in range(8)]
    for b in data:
        bits += [(b >> k) & 1 for k in range(8)]
    if fcs:
        f = zlib.crc32(bytes(data)) & 0xFFFFFFFF
        bits += [(f >> k) & 1 for k in range(32)]
    return bits, bytes(data)


def manchester(bits, n):
    """The pad trace (TD+, TD-) of those bits at n clocks a half bit: a 0
    is high then low, a 1 low then high (6.12.3)."""
    trace = []
    for v in bits:
        trace += [(1 - v, v)] * n + [(v, 1 - v)] * n
    return trace
