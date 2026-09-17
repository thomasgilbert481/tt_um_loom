"""Architectural state, commit records and trace records for the Loom model.

The shapes here follow ``docs/SEMANTICS.md`` sections 5 (per-thread state and
reset values) and 8 (the retire record used for co-simulation).

A :class:`Commit` is the model's implementation of the visibility rule in
SEMANTICS section 2: everything a slot does is collected while the slot is in X
and then applied at one edge, tagged with the cycle from which it is visible.
Host actions produce the same record, which is what makes "the thread wins on a
same-edge conflict" a matter of application order rather than special cases.
"""

from __future__ import annotations

import dataclasses
from typing import Dict, List, Optional, Tuple

WORD_MASK = 0xFFFF
PC_MASK = 0x3FF
THREADS = 4
SLOT_CLOCKS = 4

#: Per-thread CSRs that this milestone builds, CSR number -> attribute name.
THREAD_CSRS = {
    0x00: "tick_int",
    0x01: "tick_frac",
    0x02: "outgrp",
    0x03: "ingrp",
}

#: Widths of every CSR, from ``isa/isa.yaml`` (filled in by the Machine at
#: construction so that the model never hard-codes a second copy).
CSR_TICK_INT = 0x00
CSR_TICK_FRAC = 0x01
CSR_OUTGRP = 0x02
CSR_INGRP = 0x03
CSR_NOW = 0x09
CSR_TD = 0x0A
CSR_FLAGS = 0x0B
CSR_TID = 0x0C
CSR_OD_MASK = 0x10
CSR_PIN_OUT = 0x11
CSR_PIN_OE = 0x12
CSR_PIN_IN = 0x13
CSR_SFLAGS = 0x14
CSR_HOST_IRQ = 0x15

#: CSRs built at M1 (SEMANTICS 6.6).  Every other CSR reads 0 and ignores
#: writes until its feature exists.
M1_CSRS = frozenset(
    list(range(0x00, 0x04)) + list(range(0x09, 0x0D)) + list(range(0x10, 0x16))
)


@dataclasses.dataclass
class ThreadState:
    """Everything SEMANTICS section 5 lists as per-thread architectural state.

    Attributes are the committed values, i.e. what the thread's next slot sees
    in its X cycle.  ``regs`` is ``r0..r7``; ``flags`` is exposed as the three
    separate bits ``z``, ``c`` and ``t`` and as the packed ``{T, C, Z}`` word
    through :attr:`flags`.
    """

    tid: int
    reset_pc: int = 0

    regs: List[int] = dataclasses.field(default_factory=lambda: [0] * 8)
    pc: int = 0
    z: int = 0
    c: int = 0
    t: int = 0
    rs0: int = 0
    rs1: int = 0
    depth: int = 0
    wait_active: int = 0
    prev_pins: int = 0

    now: int = 0
    td: int = 0
    dt: int = 0
    acc: int = 0
    tick_seen: int = 0

    tick_int: int = 1
    tick_frac: int = 0
    outgrp: int = 0
    ingrp: int = 0

    steps: int = 0

    inq: List[int] = dataclasses.field(default_factory=list)
    outq: List[int] = dataclasses.field(default_factory=list)

    @property
    def flags(self) -> int:
        """Packed ``{T, C, Z}`` in bits 2:0, the ``FLAGS`` CSR view."""
        return (self.t << 2) | (self.c << 1) | self.z

    @flags.setter
    def flags(self, value: int) -> None:
        self.z = value & 1
        self.c = (value >> 1) & 1
        self.t = (value >> 2) & 1

    @property
    def csrs(self) -> Dict[str, int]:
        """The per-thread CSR values this build implements, by name."""
        return {
            "TICK_INT": self.tick_int,
            "TICK_FRAC": self.tick_frac,
            "OUTGRP": self.outgrp,
            "INGRP": self.ingrp,
            "NOW": self.now,
            "TD": self.td,
            "FLAGS": self.flags,
            "TID": self.tid,
        }

    @property
    def tick_period(self) -> int:
        """``max(TICK_INT, 1) * 256 + TICK_FRAC`` in 1/256 clock units."""
        return max(self.tick_int, 1) * 256 + (self.tick_frac & 0xFF)

    def copy(self) -> "ThreadState":
        other = dataclasses.replace(self)
        other.regs = list(self.regs)
        other.inq = list(self.inq)
        other.outq = list(self.outq)
        return other


@dataclasses.dataclass
class RetireRecord:
    """SEMANTICS section 8, plus the X cycle the model knows and RTL does not.

    Returned by :meth:`~tools.loomsim.Machine.step_cycle` in the cycle where a
    valid slot is in W, which is the cycle the RTL's ``tr_valid`` is high.
    """

    x_cycle: int
    thread: int
    pc: int
    ir: int
    done: bool
    we: bool
    rd: int
    val: int
    flags: int
    next_pc: int

    #: Mnemonic of the decoded instruction, or ``None`` for a reserved word.
    mnemonic: Optional[str] = None

    @property
    def z(self) -> int:
        return self.flags & 1

    @property
    def c(self) -> int:
        return (self.flags >> 1) & 1

    @property
    def t(self) -> int:
        return (self.flags >> 2) & 1

    def as_tuple(self) -> Tuple:
        """The fields the RTL exposes, in the order of the SEMANTICS table."""
        return (self.thread, self.pc, self.ir, int(self.done), int(self.we),
                self.rd, self.val, self.flags, self.next_pc)

    def __str__(self) -> str:
        write = "r%d=%04X" % (self.rd, self.val) if self.we else "-"
        name = self.mnemonic or "reserved"
        return ("x=%-6d t%d pc=%03X ir=%04X %-6s %-4s %-9s flags=%d%d%d next=%03X"
                % (self.x_cycle, self.thread, self.pc, self.ir, name,
                   "done" if self.done else "stall", write,
                   self.t, self.c, self.z, self.next_pc))


@dataclasses.dataclass
class PadState:
    """The chip's pad outputs as they stand during one cycle."""

    uo_out: int = 0
    uio_out: int = 0
    uio_oe: int = 0


@dataclasses.dataclass
class CycleTrace:
    """One entry of the optional per-cycle trace callback.

    All values describe the state *during* ``cycle``, before the edge that ends
    it, so a pad write with X cycle ``x`` first shows up in the entry for cycle
    ``x + 2``.
    """

    cycle: int
    ph: int
    uo_out: int
    uio_out: int
    uio_oe: int
    ui_in: int
    uio_in: int
    retire: Optional[RetireRecord] = None


@dataclasses.dataclass
class Commit:
    """Everything one slot (or one host action) changes, applied at one edge.

    ``visible_from`` is the cycle number from which the effect is visible, which
    for a slot with X cycle ``x`` is ``x + 2`` (SEMANTICS section 2) and for a
    host action is the cycle after the one in which the host acted.  ``SFLAGS``
    is the documented exception: :attr:`sflags_set` and :attr:`sflags_clr` are
    forwarded to the X stage one cycle earlier, from W.
    """

    visible_from: int
    thread: Optional[int] = None
    is_slot: bool = False

    # --- per-thread ------------------------------------------------------
    reg_we: bool = False
    reg_rd: int = 0
    reg_val: int = 0
    pc: Optional[int] = None
    flags: Optional[int] = None
    wait_active: Optional[int] = None
    td: Optional[int] = None
    dt: Optional[int] = None
    rs0: Optional[int] = None
    rs1: Optional[int] = None
    depth: Optional[int] = None
    prev_pins: Optional[int] = None
    steps_inc: bool = False
    tick_int: Optional[int] = None
    tick_frac: Optional[int] = None
    outgrp: Optional[int] = None
    ingrp: Optional[int] = None
    acc_clear: bool = False
    #: Thread FIFOs: value pushed into OUTQ, and whether INQ is popped.
    outq_push: Optional[int] = None
    inq_pop: bool = False

    # --- shared ----------------------------------------------------------
    pin_out_mask: int = 0
    pin_out_val: int = 0
    pin_oe_mask: int = 0
    pin_oe_val: int = 0
    od_mask: Optional[int] = None
    sflags_set: int = 0
    sflags_clr: int = 0
    run_set: Optional[int] = None
    run_clr: int = 0
    halted_set: int = 0
    halted_clr: int = 0
    step_req_set: int = 0
    step_req_clr: int = 0
    badop_set: int = 0
    badop_clr: int = 0
    swirq_set: int = 0
    reset_pc: Optional[Tuple[int, int]] = None
    imem_write: Optional[Tuple[int, int]] = None
    #: Host FIFO side: value pushed into INQ, and whether OUTQ is popped.
    inq_push: Optional[int] = None
    outq_pop: bool = False
    #: Host thread reset (SEMANTICS 7, ``CTRL.RESET``).
    thread_reset: bool = False

    def touches_pin_out(self) -> bool:
        return self.pin_out_mask != 0

    def write_pin(self, index: int, value: int, od_mask: int) -> None:
        """Apply the pin-write rule of SEMANTICS 6.3 into this commit.

        ``index`` is a 5-bit pin index; writes to indices that are not writable
        are ignored.  ``od_mask`` is ``OD_MASK`` as visible in the X cycle.
        """
        value &= 1
        if 16 <= index <= 21:
            bit = index - 8
            self.pin_out_mask |= 1 << bit
            self.pin_out_val = (self.pin_out_val & ~(1 << bit)) | (value << bit)
        elif 0 <= index <= 7:
            if (od_mask >> index) & 1:
                self.pin_out_mask |= 1 << index
                self.pin_out_val &= ~(1 << index)
                self.pin_oe_mask |= 1 << index
                oe = (~value) & 1
                self.pin_oe_val = (self.pin_oe_val & ~(1 << index)) | (oe << index)
            else:
                self.pin_out_mask |= 1 << index
                self.pin_out_val = (self.pin_out_val & ~(1 << index)) | (value << index)
