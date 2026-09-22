# SPDX-License-Identifier: Apache-2.0
"""L2-COV for the M2 features: FIFOs, bit engine, latched SETP, host IRQ.

``cosim_coverage.Coverage`` counts the M1 bins and treats every M2 word as an
unbuilt instruction, which is what it is in an M1 build. The co-simulation now
builds both sides for whatever ``CTRL.CAPS`` reports, so in an M2 build those
words execute; :class:`M2Coverage` takes them over and adds the bins the M2
text asks for (``docs/SEMANTICS.md`` 6.7 to 6.10, ``docs/VERIFICATION.md``
L2-COV):

* ``m2_fifo``: ``PUSH``/``POP`` completing and stalling (a full ``OUTQ``, an
  empty ``INQ``), the push that fills a queue, every ``WAITB`` condition
  ending by its condition, by its deadline and stalling, the host's side of
  the queues (a push accepted and dropped, a read that finds a word, one that
  peeks an empty queue and sets ``BADOP[14]``, the word loaded while the
  previous one is being popped) and a thread and the host touching the same
  queue at the same edge;
* ``m2_be``: ``SHO`` and ``SHI`` for both shift directions, with and without
  ``INV`` and ``CRC_EN``, the ``CNT`` decrement reaching zero and staying
  there, which index class the shifted bit was written to or read from,
  ``LDSR``/``STSR``/``CRCI``/``STCRC``, and both CRC feedback branches;
* ``m2_csr``: ``CSRR``/``CSRW`` of each bit-engine CSR, and a ``BE_CFG``
  write with bits M2 ignores;
* ``m2_setpd``: a staged write, one that replaces a staged write, one that
  fires by rule 1 (``NOW`` ticks onto ``TD``) and by rule 2 (``TD`` written
  and already reached), the index class it lands on, and an ordinary pin
  write on the same pin at the same edge (the ordinary write wins);
* ``m2_irq``: ``HOST_IRQ`` rising and clearing and which cause was active;
* ``m2_host``: the host-side events of ``docs/HOST_PROTOCOL.md``:
  ``BADOP[14]`` from a full push and from an empty read, ``BADOP`` and
  ``SWIRQ`` cleared, ``SFLAGS`` written, ``RUN`` rewritten, a multi-word read.

Bins that a build cannot reach are listed with the reason, as in the base
class: in a build with the bit engine every CSR ``isa.yaml`` defines is built,
so ``CSRR:unbuilt``/``CSRW:unbuilt`` become unreachable and are marked as such
(that is a build fact, not a hole).

The harness feeds one :class:`SlotContext` per retired slot, with the thread
state as visible in the slot's X cycle, plus events for the things that have
no retire record of their own (host transactions, latch firings, ``HOST_IRQ``
edges).
"""

from __future__ import annotations

import collections
import dataclasses
from typing import Dict, List, Sequence, Tuple

import cosim_coverage
from cosim_coverage import Coverage, pin_class
from tools.loomgen import (M2_CSR_NAMES, M2_MNEMONICS, WAITB_BE_IDLE,
                           WAITB_INQ_NE, WAITB_OUTQ_NF, WAITB_TICK,
                           instruction_built)

THREADS = 4

#: ``WAITB`` conditions by name, in operand order (``isa.yaml`` enums.cond).
WAITB_NAMES = {WAITB_BE_IDLE: "BE_IDLE", WAITB_OUTQ_NF: "OUTQ_NF",
               WAITB_INQ_NE: "INQ_NE", WAITB_TICK: "TICK"}

#: ``BE_CFG`` fields of M2 (SEMANTICS 6.9).
BE_CFG_DIR, BE_CFG_INV, BE_CFG_CRC_EN = 1 << 1, 1 << 7, 1 << 9
BE_CFG_M2_MASK = BE_CFG_DIR | BE_CFG_INV | BE_CFG_CRC_EN
#: Slice A fields (SEMANTICS 6.9.1, feature ``BEENC``, CAPS[9]).
BE_CFG_ENC_SHIFT, BE_CFG_STUFF_SHIFT, BE_CFG_DIFF = 3, 5, 1 << 10

M2_GROUPS = ("m2_fifo", "m2_be", "m2_csr", "m2_setpd", "m2_irq", "m2_host",
             "m3a_be", "m3b_mem")

IRQ_CAUSES = ("SWIRQ", "SFLAGS", "INQ_NOT_FULL", "OUTQ_NOT_EMPTY", "HALTED")
PIN_CLASSES = ("bidir", "bidir_od", "out", "readonly", "reserved")


@dataclasses.dataclass
class SlotContext:
    """The retiring thread's state as its slot saw it in X, plus the build.

    The retire record of SEMANTICS 8 carries none of this, and by the time
    the harness has the record the model has stepped, so the harness reads it
    out just before the model steps the W cycle (the thread's own state does
    not change between X and its own commit).
    """

    features: Tuple[str, ...] = ()
    fifo_depth: int = 4
    depth: int = 0
    outgrp: int = 0
    ingrp: int = 0
    regs: Sequence[int] = ()
    od_mask: int = 0
    inq: int = 0
    outq: int = 0
    sr: int = 0
    cnt: int = 0
    crc: int = 0
    be_cfg: int = 0
    be_pins: int = 0
    lat_valid: int = 0
    w_record: object = None


class M2Coverage(Coverage):
    """The M1 bins of :class:`cosim_coverage.Coverage` plus the M2 ones."""

    def __init__(self, isa):
        super().__init__(isa)
        self.groups = tuple(cosim_coverage.GROUPS) + M2_GROUPS
        for group in M2_GROUPS:
            self.bins[group] = collections.Counter()
        for name in _m2_bins():
            group, bin_name = name
            self.bins[group][bin_name] = 0
        self.be_csr_numbers = {isa.csr_by_name[n]: n for n in M2_CSR_NAMES}
        self.builds = set()
        # "BE idle" is true whenever the bit engine is built (auto mode is
        # M3) and the instruction is NOP + BADOP when it is not, so the wait
        # always completes on its first issue: it can neither stall nor reach
        # its deadline in any build this model can be.
        for name in ("WAITB:BE_IDLE:stall", "WAITB:BE_IDLE:deadline"):
            cosim_coverage.UNREACHABLE.setdefault(
                name, "BE_IDLE is always true until auto mode exists "
                      "(SEMANTICS 6.4, 6.7)")

    # ------------------------------------------------------------ the build
    def note_build(self, features: Sequence[str], fifo_depth: int) -> None:
        """Record which build a seed ran with; it decides which bins exist."""
        self.builds.add((tuple(sorted(features)), fifo_depth))
        # Every CSR isa.yaml defines is built when the bit engine is, so the
        # "unbuilt CSR" bins only exist if some seed ran without it.
        every_build_has_be = all("BE" in f for f, _ in self.builds)
        for name in ("CSRR:unbuilt", "CSRW:unbuilt"):
            if every_build_has_be:
                cosim_coverage.UNREACHABLE.setdefault(
                    name, "the bit engine is built, so no CSR of isa.yaml is unbuilt")
            else:
                cosim_coverage.UNREACHABLE.pop(name, None)
        # The slice-A bins exist only in a build with the encoders (CAPS[9]),
        # the slice-B bins only with the data memory (CAPS[5]).
        for group, feature, why in (("m3a_be", "BEENC", "slice A (the encoders) is not built: CAPS[9] = 0"),
                                    ("m3b_mem", "DMEM", "slice B (LD/ST) is not built: CAPS[5] = 0")):
            present = any(feature in f for f, _ in self.builds)
            for g, bin_name in _m2_bins():
                if g != group:
                    continue
                if present:
                    cosim_coverage.UNREACHABLE.pop(bin_name, None)
                else:
                    cosim_coverage.UNREACHABLE.setdefault(bin_name, why)

    def _built(self, name: str, fields: Dict[str, int],
               features: Sequence[str]) -> bool:
        return instruction_built(name, fields, features)

    # ------------------------------------------------------------------ feed
    def note(self, record, ctx: SlotContext) -> None:            # type: ignore[override]
        """Account for one retired slot (M1 bins through the base class)."""
        # Slice B (6.11): the model names both slots of an LD/ST by their
        # mnemonic; the completion slot's `ir` is data, never decoded.
        mem = getattr(record, "mnemonic", None)
        if mem in ("LD", "ST") and "DMEM" in ctx.features:
            self.slots += 1
            self.hit("mnemonic", "%s@t%d" % (mem, record.thread))
            self.hit("m3b_mem", "%s:%s" % (mem, "complete" if record.done else "issue"))
            if record.done and mem == "LD" and record.we:
                self.hit("m3b_mem", "LD:writes_rd")
            return
        decoded = self.isa.decode(record.ir)
        if decoded is not None:
            name, fields = decoded[0].name, decoded[1]
            if name in M2_MNEMONICS and self._built(name, fields, ctx.features):
                self.slots += 1
                self.hit("mnemonic", "%s@t%d" % (name, record.thread))
                self._note_m2(name, fields, record, ctx)
                return
            if name in ("CSRR", "CSRW") and "BE" in ctx.features \
                    and fields["csr"] in self.be_csr_numbers:
                self.slots += 1
                self.hit("mnemonic", "%s@t%d" % (name, record.thread))
                self._note_be_csr(name, fields, ctx)
                return
            if name == "SETP" and fields["lat"] and "SETPD" in ctx.features:
                self._note_setp_d(fields, record, ctx)
        super().note(record, ctx.depth, ctx.outgrp, ctx.ingrp, ctx.regs,
                     ctx.od_mask, ctx.w_record)

    # ---------------------------------------------------------------- FIFOs
    def _note_m2(self, name, fields, record, ctx: SlotContext) -> None:
        if name == "PUSH":
            self.hit("m2_fifo", "PUSH:done" if record.done else "PUSH:stall(full)")
            if record.done and ctx.outq + 1 == ctx.fifo_depth:
                self.hit("m2_fifo", "PUSH:fills_the_OUTQ")
        elif name == "POP":
            self.hit("m2_fifo", "POP:done" if record.done else "POP:stall(empty)")
            if record.done and ctx.inq == 1:
                self.hit("m2_fifo", "POP:empties_the_INQ")
        elif name == "WAITB":
            cond = WAITB_NAMES[fields["cond"]]
            if not record.done:
                self.hit("m2_fifo", "WAITB:%s:stall" % cond)
            elif fields.get("tmo") and record.t:
                self.hit("m2_fifo", "WAITB:%s:deadline" % cond)
            else:
                self.hit("m2_fifo", "WAITB:%s:condition" % cond)
        elif name in ("SHO", "SHI"):
            self._note_shift(name, record, ctx)
        else:
            self.hit("m2_be", name)

    # ----------------------------------------------------------- bit engine
    def _note_shift(self, name, record, ctx: SlotContext) -> None:
        cfg = ctx.be_cfg
        if "BEENC" in ctx.features:
            self._note_enc(name, record, cfg)
        direction = 1 if cfg & BE_CFG_DIR else 0
        self.hit("m2_be", "%s:DIR%d" % (name, direction))
        self.hit("m2_be", "%s:INV%d" % (name, 1 if cfg & BE_CFG_INV else 0))
        self.hit("m2_be", "%s:CRC_EN%d" % (name, 1 if cfg & BE_CFG_CRC_EN else 0))
        self.hit("m2_be", "%s:Z%d" % (name, record.z))
        if ctx.cnt == 0:
            self.hit("m2_be", "%s:CNT_already_zero" % name)
        if name == "SHO":
            index = ctx.be_pins & 0x1F
            self.hit("m2_be", "SHO:pin_%s" % pin_class(index, ctx.od_mask))
            bit = (ctx.sr >> 15) & 1 if direction else ctx.sr & 1
            if cfg & BE_CFG_CRC_EN:
                feedback = ((ctx.crc >> 15) & 1) ^ bit
                self.hit("m2_be", "CRC:feedback%d" % feedback)
        else:
            index = (ctx.be_pins >> 5) & 0x1F
            self.hit("m2_be", "SHI:in_%s" % _in_class(index))

    def _note_enc(self, name, record, cfg: int) -> None:
        """Slice A (6.9.1): which encoder, stuffer and DIFF each shift ran with,
        and a SHI that left T set (a violation, or one already there)."""
        self.hit("m3a_be", "%s:ENC%d" % (name, (cfg >> BE_CFG_ENC_SHIFT) & 3))
        self.hit("m3a_be", "%s:STUFF%d" % (name, (cfg >> BE_CFG_STUFF_SHIFT) & 3))
        if name == "SHO":
            self.hit("m3a_be", "SHO:DIFF%d" % (1 if cfg & BE_CFG_DIFF else 0))
        elif record.t:
            self.hit("m3a_be", "SHI:T1")

    def _note_be_csr(self, name, fields, ctx: SlotContext) -> None:
        csr = self.be_csr_numbers[fields["csr"]]
        self.hit("m2_csr", "%s:%s" % (name, csr))
        if name == "CSRW" and csr == "BE_CFG" and ctx.regs \
                and "BEENC" in ctx.features:
            written = ctx.regs[fields["ra"]]
            self.hit("m3a_be", "CSRW:BE_CFG:ENC%d" % ((written >> BE_CFG_ENC_SHIFT) & 3))
            self.hit("m3a_be", "CSRW:BE_CFG:STUFF%d" % ((written >> BE_CFG_STUFF_SHIFT) & 3))
            self.hit("m3a_be", "CSRW:BE_CFG:DIFF%d" % (1 if written & BE_CFG_DIFF else 0))
        if name == "CSRW" and csr == "BE_CFG" and ctx.regs \
                and ctx.regs[fields["ra"]] & ~BE_CFG_M2_MASK & 0xFFFF:
            self.hit("m2_csr", "CSRW:BE_CFG:bits_M2_ignores")

    # --------------------------------------------------------- latched SETP
    def _note_setp_d(self, fields, record, ctx: SlotContext) -> None:
        self.hit("m2_setpd", "staged:replaces" if ctx.lat_valid else "staged:new")
        self.hit("m2_setpd", "staged:%s" % pin_class(fields["pin"], ctx.od_mask))

    # -------------------------------------------------------------- events
    def latch_fired(self, rule: int, pin: int, od_mask: int,
                    ordinary_write: bool = False) -> None:
        """A staged pin write landed (SEMANTICS 6.10)."""
        self.hit("m2_setpd", "fired:rule%d" % rule)
        self.hit("m2_setpd", "fired:%s" % pin_class(pin, od_mask))
        if ordinary_write:
            self.hit("m2_setpd", "fired:ordinary_write_wins")

    def host_event(self, name: str) -> None:
        group = "m2_host" if name.startswith(("BADOP", "SWIRQ", "SFLAGS", "RUN",
                                              "IRQ_EN")) else "m2_fifo"
        self.hit(group, name)

    def irq_event(self, name: str) -> None:
        self.hit("m2_irq", name)

    # ---------------------------------------------------------------- report
    def holes(self):
        return [(g, n) for g in self.groups
                for n, c in sorted(self.bins[g].items())
                if c == 0 and n not in cosim_coverage.UNREACHABLE]

    def table(self, only_holes: bool = False) -> str:
        lines = [super().table(only_holes)]
        lines.append("")
        lines.append("  M2 bins (SEMANTICS 6.7-6.10), builds seen: %s"
                     % "; ".join("%s depth %d" % (",".join(f) or "M1", d)
                                 for f, d in sorted(self.builds)))
        for group in M2_GROUPS:
            counter = self.bins[group]
            reachable = [n for n in counter if n not in cosim_coverage.UNREACHABLE]
            hit = sum(1 for n in reachable if counter[n])
            lines.append("  %-15s %4d/%4d reachable bins hit"
                         % (group, hit, len(reachable)))
            for name, count in sorted(counter.items()):
                if only_holes and count:
                    continue
                note = ""
                if count == 0:
                    note = "  <-- unreachable: " + cosim_coverage.UNREACHABLE[name] \
                        if name in cosim_coverage.UNREACHABLE else "  <-- EMPTY"
                lines.append("      %-34s %9d%s" % (name, count, note))
            lines.append("")
        empty = self.holes()
        lines.append("  empty bins at the end of the run: %d%s" % (
            len(empty), "" if not empty else ":"))
        lines += ["      %s/%s" % h for h in empty]
        return "\n".join(lines)

    def to_obj(self) -> Dict:
        obj = super().to_obj()
        obj["bins"].update({g: dict(sorted(self.bins[g].items()))
                            for g in M2_GROUPS})
        obj["builds"] = ["%s depth %d" % (",".join(f) or "M1", d)
                         for f, d in sorted(self.builds)]
        obj["unreachable"] = dict(cosim_coverage.UNREACHABLE)
        obj["holes"] = ["%s/%s" % h for h in self.holes()]
        return obj


def _in_class(index: int) -> str:
    """Where ``SHI`` reads its bit from (SEMANTICS 3): a synchronised pad, the
    ``PIN_OUT`` register of an OUT index, or nothing at all."""
    if 0 <= index <= 12:
        return "pad"
    if 16 <= index <= 21:
        return "pin_out"
    return "zero"


def _m2_bins() -> List[Tuple[str, str]]:
    """Every M2 bin, declared up front so a hole is a zero row."""
    out: List[Tuple[str, str]] = []

    def add(group, *names):
        out.extend((group, n) for n in names)

    add("m2_fifo", "PUSH:done", "PUSH:stall(full)", "PUSH:fills_the_OUTQ",
        "POP:done", "POP:stall(empty)", "POP:empties_the_INQ")
    for cond in WAITB_NAMES.values():
        add("m2_fifo", "WAITB:%s:condition" % cond, "WAITB:%s:deadline" % cond,
            "WAITB:%s:stall" % cond)
    add("m2_fifo", "host_push:accepted", "host_push:dropped(full)",
        "host_read:word", "host_read:empty_peek", "host_read:next_entry",
        "host_read:multi_word", "host_read:filled_after_empty_peek",
        "same_edge:thread_push+host_read", "same_edge:thread_pop+host_push")
    for name in ("SHO", "SHI"):
        add("m2_be", "%s:DIR0" % name, "%s:DIR1" % name, "%s:INV0" % name,
            "%s:INV1" % name, "%s:CRC_EN0" % name, "%s:CRC_EN1" % name,
            "%s:Z0" % name, "%s:Z1" % name, "%s:CNT_already_zero" % name)
    add("m2_be", *["SHO:pin_%s" % c for c in PIN_CLASSES])
    add("m2_be", "SHI:in_pad", "SHI:in_pin_out", "SHI:in_zero")
    add("m2_be", "CRC:feedback0", "CRC:feedback1")
    add("m2_be", "LDSR", "STSR", "CRCI", "STCRC")
    for op in ("CSRR", "CSRW"):
        add("m2_csr", *["%s:%s" % (op, n) for n in M2_CSR_NAMES])
    add("m2_csr", "CSRW:BE_CFG:bits_M2_ignores")
    for name in ("SHO", "SHI"):
        add("m3a_be", *["%s:ENC%d" % (name, v) for v in range(3)])
        add("m3a_be", *["%s:STUFF%d" % (name, v) for v in range(3)])
    add("m3a_be", "SHO:DIFF0", "SHO:DIFF1", "SHI:T1")
    add("m3a_be", *["CSRW:BE_CFG:ENC%d" % v for v in range(4)])
    add("m3a_be", *["CSRW:BE_CFG:STUFF%d" % v for v in range(4)])
    add("m3a_be", "CSRW:BE_CFG:DIFF0", "CSRW:BE_CFG:DIFF1")
    add("m3b_mem", "LD:issue", "LD:complete", "LD:writes_rd", "ST:issue", "ST:complete")
    add("m2_setpd", "staged:new", "staged:replaces", "fired:rule1", "fired:rule2",
        "fired:ordinary_write_wins")
    add("m2_setpd", *["staged:%s" % c for c in PIN_CLASSES])
    add("m2_setpd", *["fired:%s" % c for c in PIN_CLASSES])
    add("m2_irq", "HOST_IRQ:rise", "HOST_IRQ:clear")
    add("m2_irq", *["cause:%s" % c for c in IRQ_CAUSES])
    add("m2_host", "BADOP14:host_push_full", "BADOP14:host_read_empty",
        "BADOP:cleared", "SWIRQ:cleared", "SFLAGS:set", "SFLAGS:cleared",
        "RUN:rewritten", "IRQ_EN:written", "IRQ_EN2:written")
    return out
