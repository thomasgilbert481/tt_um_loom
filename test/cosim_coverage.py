# SPDX-License-Identifier: Apache-2.0
"""Functional coverage for the random co-simulation (docs/VERIFICATION.md L2-COV).

No dependencies beyond the standard library and ``tools.loomisa``. The
harness feeds the collector one golden-model retire record per retired slot
(the RTL record is identical, or the run has already failed), plus the context
the record does not carry, all as visible in the slot's X cycle:

* ``depth``, ``outgrp``, ``ingrp``, ``regs``: the thread's own ``DEPTH``,
  ``OUTGRP``, ``INGRP`` and ``r0..r7`` (unchanged between X and the slot's own
  commit, so the harness reads them just before the model steps the W cycle);
* ``od_mask``: the global ``OD_MASK`` during the X cycle;
* ``w_record``: the record of the slot that was in W during that X cycle.

Bins, grouped as the report prints them:

* ``mnemonic``: every M1 mnemonic, every unbuilt one and ``reserved`` x thread;
* ``badop``: the two causes, reserved word and unbuilt instruction;
* ``wait``: done versus stall for each wait-class mnemonic;
* ``timed_end``: timed ``WAITP``/``WAITE``/``WAITS`` ended by the condition
  (``T`` cleared) versus by the deadline (``T`` set);
* ``branch``: every ``Bcc``, ``DJNZ`` and ``JP`` taken and not taken (``Bcc`` by
  its condition on the flags, which a branch leaves alone; ``DJNZ`` by the new
  count; ``JP`` by the next PC);
* ``stack``: ``CALL`` at depth 0, 1 and 2 (overflow, drops the oldest) and
  ``RET`` at depth 0 (empty: falls through), 1 and 2;
* ``pin_write``: which index class each ``SETP``/``OUT`` bit wrote (BIDIR push
  pull, BIDIR open drain, OUT, read-only, reserved), open-drain writes of 0
  and 1, ``OUT``/``IN`` with a count of 0 and of 16 or more, ``OEP`` on a
  BIDIR index and on one it ignores;
* ``csr``: ``CSRR``/``CSRW`` of every built CSR, of an unbuilt feature's CSR
  and of a number ``isa.yaml`` does not define;
* ``flags``: ``Z`` and ``C`` after every flag-producing instruction, 0 and 1;
* ``sflags_forward``: a ``WAITS`` in X while a ``SIG``, ``CLR``, ``CSRW SFLAGS``
  or completing ``WAITS`` of the same flag is in W (SEMANTICS 2: SFLAGS is
  forwarded from W to X, which is what makes ``WAITS`` atomic).

Every bin is declared up front, so a hole is a zero row, not a missing line.
Bins that cannot be reached at M1 are listed with the reason.
"""

from __future__ import annotations

import collections
import json
from typing import Dict, List, Optional, Sequence, Tuple

from tools.loomgen import M1_CSR_NAMES, M1_MNEMONICS, UNBUILT_MNEMONICS
from tools.loomisa import Isa

THREADS = 4
PC_MASK = 0x3FF

BRANCHES = ("BZ", "BNZ", "BC", "BNC", "BT", "BNT")
WAITS = ("WAITD", "DLY", "WAITP", "WAITE", "WAITS")
TIMED = ("WAITP", "WAITE", "WAITS")
#: Instructions that set Z and C (SEMANTICS 6.1: all ALU, ALUI and unary
#: except MOV).
FLAG_PRODUCERS = ("ADD", "SUB", "AND", "OR", "XOR", "SHL", "SHR", "ROR",
                  "ADDI", "SUBI", "ANDI", "ORI", "XORI", "SHLI", "SHRI", "CMPI",
                  "NOT", "NEG", "CMP", "TEST", "REV", "PAR", "SWAP")
#: Producers whose C is 0 by definition (SEMANTICS 6.1).
C_ALWAYS_ZERO = ("AND", "OR", "XOR", "ANDI", "ORI", "XORI", "NOT", "TEST",
                 "REV", "SWAP")

GROUPS = ("mnemonic", "badop", "wait", "timed_end", "branch", "stack",
          "pin_write", "csr", "flags", "sflags_forward")


def pin_class(index: int, od_mask: int) -> str:
    """The index class of ARCHITECTURE 3.1, BIDIR split by open-drain mode."""
    if 0 <= index <= 7:
        return "bidir_od" if (od_mask >> index) & 1 else "bidir"
    if 8 <= index <= 12:
        return "readonly"
    if 16 <= index <= 21:
        return "out"
    return "reserved"


def _declared_bins() -> Dict[str, List[str]]:
    bins: Dict[str, List[str]] = {g: [] for g in GROUPS}
    for name in M1_MNEMONICS + UNBUILT_MNEMONICS + ("reserved",):
        bins["mnemonic"] += ["%s@t%d" % (name, t) for t in range(THREADS)]
    bins["badop"] = ["reserved_word", "unbuilt_instruction"]
    for name in WAITS:
        bins["wait"] += ["%s:done" % name, "%s:stall" % name]
    for name in TIMED:
        bins["timed_end"] += ["%s:condition" % name, "%s:deadline" % name]
    for name in BRANCHES + ("DJNZ", "JP"):
        bins["branch"] += ["%s:taken" % name, "%s:not_taken" % name]
    bins["stack"] = ["CALL@0", "CALL@1", "CALL@2(overflow)",
                     "RET@0(empty)", "RET@1", "RET@2"]
    for op in ("SETP", "OUT"):
        bins["pin_write"] += ["%s:%s" % (op, c) for c in
                              ("bidir", "bidir_od", "out", "readonly", "reserved")]
    bins["pin_write"] += ["od_write:0", "od_write:1", "OUT:cnt0", "OUT:cnt16",
                          "IN:cnt0", "IN:cnt16", "OEP:bidir", "OEP:ignored"]
    for op in ("CSRR", "CSRW"):
        bins["csr"] += ["%s:%s" % (op, n) for n in M1_CSR_NAMES]
        bins["csr"] += ["%s:unbuilt" % op, "%s:undefined" % op]
    for name in FLAG_PRODUCERS:
        bins["flags"] += ["%s:Z0" % name, "%s:Z1" % name,
                          "%s:C0" % name, "%s:C1" % name]
    bins["sflags_forward"] = ["WAITS_over_SIG", "WAITS_over_CLR",
                              "WAITS_over_CSRW_SFLAGS", "WAITS_over_WAITS"]
    return bins


def _unreachable() -> Dict[str, str]:
    out = {"%s:C1" % n: "C is 0 by definition (SEMANTICS 6.1)"
           for n in C_ALWAYS_ZERO}
    return out


UNREACHABLE: Dict[str, str] = _unreachable()


class Coverage:
    """Counts of every functional coverage bin, across every seed of a run."""

    def __init__(self, isa: Isa):
        self.isa = isa
        self.bins: Dict[str, collections.Counter] = {
            group: collections.Counter() for group in GROUPS}
        for group, names in _declared_bins().items():
            for name in names:
                self.bins[group][name] = 0
        self.slots = 0
        self.seeds: List[str] = []
        self.unbuilt_csrs = set()
        for number, csr in isa.csrs.items():
            if csr["name"] not in M1_CSR_NAMES:
                self.unbuilt_csrs.add(number)

    # ------------------------------------------------------------------ feed
    def note_seed(self, label: str) -> None:
        self.seeds.append(label)

    def hit(self, group: str, name: str) -> None:
        self.bins[group][name] += 1

    def note(self, record, depth: int, outgrp: int, ingrp: int,
             regs: Sequence[int], od_mask: int, w_record=None) -> None:
        """Account for one retired slot (see the module docstring)."""
        self.slots += 1
        decoded = self.isa.decode(record.ir)
        if decoded is None:
            self.hit("mnemonic", "reserved@t%d" % record.thread)
            self.hit("badop", "reserved_word")
            return
        instr, fields = decoded
        name = instr.name
        self.hit("mnemonic", "%s@t%d" % (name, record.thread))
        if name in UNBUILT_MNEMONICS:
            self.hit("badop", "unbuilt_instruction")
            return

        if name in WAITS:
            self.hit("wait", "%s:%s" % (name, "done" if record.done else "stall"))
            if name in TIMED and fields.get("tmo") and record.done:
                self.hit("timed_end", "%s:%s" % (
                    name, "deadline" if record.t else "condition"))

        if name in BRANCHES:
            taken = {"BZ": record.z, "BNZ": 1 - record.z, "BC": record.c,
                     "BNC": 1 - record.c, "BT": record.t,
                     "BNT": 1 - record.t}[name]
            self.hit("branch", "%s:%s" % (name, "taken" if taken else "not_taken"))
        elif name == "DJNZ":
            self.hit("branch", "DJNZ:%s" % ("taken" if record.val else "not_taken"))
        elif name == "JP":
            fell = record.next_pc == ((record.pc + 1) & PC_MASK)
            self.hit("branch", "JP:%s" % ("not_taken" if fell else "taken"))

        if name == "CALL":
            self.hit("stack", "CALL@2(overflow)" if depth >= 2 else "CALL@%d" % depth)
        elif name == "RET":
            self.hit("stack", "RET@0(empty)" if depth == 0 else "RET@%d" % depth)

        if name == "SETP":
            self._pin_writes("SETP", [fields["pin"]], [fields["val"]], od_mask)
        elif name == "OUT":
            base, count = outgrp & 0x1F, min((outgrp >> 5) & 0x1F, 16)
            if count == 0:
                self.hit("pin_write", "OUT:cnt0")
            if count == 16:
                self.hit("pin_write", "OUT:cnt16")
            value = regs[fields["ra"]]
            self._pin_writes("OUT", [(base + j) % 32 for j in range(count)],
                             [(value >> j) & 1 for j in range(count)], od_mask)
        elif name == "IN":
            count = min((ingrp >> 5) & 0x1F, 16)
            if count in (0, 16):
                self.hit("pin_write", "IN:cnt%d" % count)
        elif name == "OEP":
            self.hit("pin_write", "OEP:%s" % ("bidir" if fields["pin"] <= 7
                                               else "ignored"))

        if name in ("CSRR", "CSRW"):
            number = fields["csr"]
            if number not in self.isa.csrs:
                kind = "undefined"
            elif number in self.unbuilt_csrs:
                kind = "unbuilt"
            else:
                kind = self.isa.csrs[number]["name"]
            self.hit("csr", "%s:%s" % (name, kind))

        if name in FLAG_PRODUCERS:
            self.hit("flags", "%s:Z%d" % (name, record.z))
            self.hit("flags", "%s:C%d" % (name, record.c))

        if name == "WAITS" and w_record is not None:
            self._forwarding(fields["flag"], w_record)

    def _forwarding(self, flag: int, w_record) -> None:
        other = self.isa.decode(w_record.ir)
        if other is None:
            return
        instr, fields = other
        if instr.name in ("SIG", "CLR") and fields["flag"] == flag:
            self.hit("sflags_forward", "WAITS_over_%s" % instr.name)
        elif instr.name == "CSRW" and self.isa.csrs.get(fields["csr"], {}).get(
                "name") == "SFLAGS":
            self.hit("sflags_forward", "WAITS_over_CSRW_SFLAGS")
        elif instr.name == "WAITS" and fields["flag"] == flag and w_record.done \
                and not w_record.t:
            # Only a WAITS that completed on its condition clears the flag.
            self.hit("sflags_forward", "WAITS_over_WAITS")

    def _pin_writes(self, op: str, indices, values, od_mask: int) -> None:
        seen = set()
        for index, value in zip(indices, values):
            klass = pin_class(index, od_mask)
            if klass not in seen:
                self.hit("pin_write", "%s:%s" % (op, klass))
                seen.add(klass)
            if klass == "bidir_od":
                self.hit("pin_write", "od_write:%d" % (value & 1))

    # ---------------------------------------------------------------- report
    def holes(self) -> List[Tuple[str, str]]:
        """Reachable bins with a zero count."""
        return [(g, n) for g in GROUPS for n, c in sorted(self.bins[g].items())
                if c == 0 and n not in UNREACHABLE]

    def table(self, only_holes: bool = False) -> str:
        lines = ["", "L2-COV functional coverage: %d retired slots over %d programs"
                 % (self.slots, len(self.seeds)), ""]
        for group in GROUPS:
            counter = self.bins[group]
            reachable = [n for n in counter if n not in UNREACHABLE]
            hit = sum(1 for n in reachable if counter[n])
            lines.append("  %-15s %4d/%4d reachable bins hit" % (group, hit,
                                                                 len(reachable)))
            if group == "mnemonic":
                lines += self._mnemonic_matrix(counter)
                continue
            for name, count in sorted(counter.items()):
                if only_holes and count:
                    continue
                note = ""
                if count == 0:
                    note = "  <-- unreachable: " + UNREACHABLE[name] \
                        if name in UNREACHABLE else "  <-- HOLE"
                lines.append("      %-28s %9d%s" % (name, count, note))
            lines.append("")
        holes = self.holes()
        lines.append("  reachable bins not hit: %d%s" % (
            len(holes), "" if not holes else ": " + ", ".join(
                "%s/%s" % h for h in holes)))
        return "\n".join(lines)

    @staticmethod
    def _mnemonic_matrix(counter) -> List[str]:
        names = []
        for key in counter:
            name = key.split("@")[0]
            if name not in names:
                names.append(name)
        lines = ["      %-10s %9s %9s %9s %9s" % ("", "t0", "t1", "t2", "t3")]
        for name in names:
            counts = [counter["%s@t%d" % (name, t)] for t in range(THREADS)]
            mark = "  <-- HOLE" if 0 in counts else ""
            lines.append("      %-10s %9d %9d %9d %9d%s" % ((name,) + tuple(counts) + (mark,)))
        lines.append("")
        return lines

    def to_obj(self) -> Dict:
        return {
            "slots": self.slots,
            "programs": self.seeds,
            "bins": {g: dict(sorted(self.bins[g].items())) for g in GROUPS},
            "unreachable": UNREACHABLE,
            "holes": ["%s/%s" % h for h in self.holes()],
        }

    def write(self, path) -> None:
        with open(path, "w", encoding="utf-8", newline="\n") as handle:
            json.dump(self.to_obj(), handle, indent=2)
            handle.write("\n")
