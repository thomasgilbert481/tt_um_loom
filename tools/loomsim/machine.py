"""The Loom golden model: a cycle-based reference implementation.

This module implements ``docs/SEMANTICS.md`` directly.  It is written from that
document and from ``isa/isa.yaml`` only; it never reads the RTL, which is what
makes co-simulation meaningful (``docs/VERIFICATION.md``, METH-1).

The model is a clock, not an interpreter:

* :meth:`Machine.step_cycle` advances exactly one clock cycle.  Cycle ``k`` is
  the interval between rising edge ``k`` and edge ``k + 1``; a fresh
  :class:`Machine` sits at the start of cycle 0, where all state holds its reset
  value.
* Thread ``t`` owns the slot that starts (stage F) in every cycle with
  ``k mod 4 == t``.  The slot's stages are F at ``k``, D at ``k + 1``, X at
  ``x = k + 2`` and W at ``k + 3``; every effect of the slot is registered at
  edge ``x + 2`` and is visible from cycle ``x + 2``.
* Effects are therefore not applied when they are computed.  Each slot builds a
  :class:`~tools.loomsim.state.Commit` tagged with the cycle it becomes visible
  in, and the edge that ends that cycle's predecessor applies it.  ``SFLAGS`` is
  the one documented exception: it is forwarded from W to X, so it is visible
  one cycle earlier, which is what makes ``WAITS`` an atomic test-and-clear
  across threads.

Host actions are methods.  Each one commits at the edge that ends the cycle the
call was made in and is visible from the next cycle, exactly like a host write
over SPI; when a host write and a thread commit hit the same bits at the same
edge the thread wins, which the model gets by applying host commits first.

Only the ``[M1]`` feature set is built.  Bit-engine and data-memory
instructions execute as ``NOP`` and set ``BADOP`` (SEMANTICS section 9) unless
their feature is named in ``features``; ``FIFO`` enables ``PUSH``/``POP`` and
``WAITB`` conditions 1, 2 and 3.
"""

from __future__ import annotations

import dataclasses
import json
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Tuple

from tools.loomisa import Isa, Instr
from tools.loomisa import load as load_isa

from . import alu
from .state import (
    THREADS, SLOT_CLOCKS, PC_MASK, WORD_MASK,
    CSR_FLAGS, CSR_HOST_IRQ, CSR_INGRP, CSR_NOW, CSR_OD_MASK, CSR_OUTGRP,
    CSR_PIN_IN, CSR_PIN_OE, CSR_PIN_OUT, CSR_SFLAGS, CSR_TD, CSR_TICK_FRAC,
    CSR_TICK_INT, CSR_TID,
    Commit, CycleTrace, PadState, RetireRecord, ThreadState,
)

#: Pin index groups (ARCHITECTURE 3.1 / isa.yaml ``pins``).
BIDIR_PINS = range(0, 8)
IN_PINS = range(8, 13)
OUT_PINS = range(16, 22)
SYNC_BITS = 13          # pin_in(0..12) goes through the two-flop synchroniser

DEFAULT_IMEM_WORDS = 1024
DEFAULT_FIFO_DEPTH = 4

#: Debug registers of the host DEBUG space that this milestone implements.
DEBUG_REGS = ("r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "PC", "FLAGS",
              "TD", "NOW", "STEPS", "RS0", "RS1", "DEPTH", "WAIT_ACTIVE",
              "TICK_INT", "TICK_FRAC", "OUTGRP", "INGRP", "DT", "ACC",
              "TICK_SEEN", "PREV_PINS")


class LoomsimError(RuntimeError):
    """Raised for misuse of the model API (never for guest program errors)."""


@dataclasses.dataclass
class _Slot:
    """One in-flight slot.  Internal; the public view is :class:`RetireRecord`."""

    thread: int
    f_cycle: int
    valid: bool
    pc: int = 0
    ir: int = 0
    record: Optional[RetireRecord] = None


@dataclasses.dataclass
class RunResult:
    """What :meth:`Machine.run_until` gives back."""

    fired: bool
    cycles: int
    records: List[RetireRecord] = dataclasses.field(default_factory=list)


def load_image_file(path) -> Dict[int, int]:
    """Read the assembler's JSON image format, ``{"words": {addr: word}}``.

    Addresses may be decimal or hex strings or integers.  Returns a plain
    ``{int: int}`` mapping suitable for :meth:`Machine.load_image`.
    """
    with open(path, "r", encoding="utf-8") as handle:
        raw = json.load(handle)
    return image_from_obj(raw)


def image_from_obj(raw) -> Dict[int, int]:
    """Normalise a parsed image object (or a bare address->word mapping)."""
    words = raw.get("words", raw) if isinstance(raw, Mapping) else raw
    out: Dict[int, int] = {}
    for key, value in dict(words).items():
        addr = key if isinstance(key, int) else int(str(key), 0)
        out[addr] = int(value) & WORD_MASK
    return out


class Machine:
    """A whole Loom chip: four threads, the pipeline, the pins and the host port.

    Args:
        image: instruction memory contents, ``{address: word}``.  Instruction
            memory is not reset, so anything not given reads 0.
        features: optional features that are built in this configuration.
            ``"FIFO"`` enables ``PUSH``/``POP`` and ``WAITB`` 1..3.  The default
            (nothing) is the M1 RTL build: every instruction of an unbuilt
            feature is a ``NOP`` that sets ``BADOP``.
        imem_words: instruction memory size; addresses wrap within it.
        fifo_depth: INQ and OUTQ depth per thread.
        loopback: drive ``uio_in`` from ``uio_out`` for bits with ``uio_oe`` set,
            as the Tiny Tapeout pad does.
        on_cycle: optional callable invoked once per cycle with a
            :class:`~tools.loomsim.state.CycleTrace`.
        isa: a preloaded :class:`tools.loomisa.Isa` (one is loaded if omitted).

    The public state is :attr:`threads` (a list of
    :class:`~tools.loomsim.state.ThreadState`) plus the global registers
    :attr:`run`, :attr:`halted`, :attr:`step_req`, :attr:`badop`,
    :attr:`sflags`, :attr:`od_mask`, :attr:`pin_out` and :attr:`pin_oe`.
    """

    def __init__(self, image: Optional[Mapping[int, int]] = None,
                 features: Iterable[str] = (),
                 imem_words: int = DEFAULT_IMEM_WORDS,
                 fifo_depth: int = DEFAULT_FIFO_DEPTH,
                 loopback: bool = False,
                 on_cycle: Optional[Callable[[CycleTrace], None]] = None,
                 isa: Optional[Isa] = None):
        if imem_words & (imem_words - 1) or not 1 <= imem_words <= 1024:
            raise LoomsimError("imem_words must be a power of two up to 1024")
        self.isa = isa if isa is not None else load_isa()
        self.features = frozenset(str(f).upper() for f in features)
        self.imem_words = imem_words
        self.imem_mask = imem_words - 1
        self.fifo_depth = fifo_depth
        self.loopback = loopback
        self.on_cycle = on_cycle
        self.imem: Dict[int, int] = {}
        if image:
            self.load_image(image)
        self.reset()

    # ------------------------------------------------------------------ setup
    def reset(self) -> None:
        """Return to the state of cycle 0 (SEMANTICS 5).  Keeps instruction memory."""
        self.cycle = 0
        # SEMANTICS 5: RESET_PC[t] = t * (IMEM_WORDS / 4), so the four threads
        # never alias whatever the memory size.
        stride = max(self.imem_words // THREADS, 1)
        self.reset_pc = [(t * stride) & PC_MASK for t in range(THREADS)]
        self.threads = [ThreadState(tid=t, reset_pc=self.reset_pc[t])
                        for t in range(THREADS)]
        for t in range(THREADS):
            self.threads[t].pc = self.reset_pc[t]
        self.run = 0
        self.halted = 0
        self.step_req = 0
        self.badop = 0
        self.swirq = 0
        self.sflags = 0
        self.od_mask = 0
        self.pin_out = 0
        self.pin_oe = 0
        self.ui_in = 0
        self.uio_in = 0
        self._ff1 = 0
        self._ff2 = 0
        self._pending: List[Commit] = []
        self._stage_f: Optional[_Slot] = None
        self._stage_d: Optional[_Slot] = None
        self._stage_x: Optional[_Slot] = None
        self._stage_w: Optional[_Slot] = None

    def load_image(self, words: Mapping) -> None:
        """Back-door write of instruction memory (the host loads it while halted)."""
        for key, value in image_from_obj(words).items():
            self.imem[key & self.imem_mask] = value & WORD_MASK

    def set_pad_inputs(self, ui_in: Optional[int] = None,
                       uio_in: Optional[int] = None) -> None:
        """Set the pad values that will be held at the coming edge.

        The caller sets these before :meth:`step_cycle`; they are what the pads
        hold at the edge that ends the cycle about to run.  Because of the
        two-flop synchroniser a change here is first visible to an instruction
        two X cycles later (SEMANTICS section 3).
        """
        if ui_in is not None:
            self.ui_in = ui_in & 0xFF
        if uio_in is not None:
            self.uio_in = uio_in & 0xFF

    # ------------------------------------------------------------------ views
    @property
    def uo_out(self) -> int:
        """``uo_out[5:0]``, i.e. ``PIN_OUT[13:8]``, during the current cycle."""
        return (self.pin_out >> 8) & 0x3F

    @property
    def uio_out(self) -> int:
        """``uio_out[7:0]``, i.e. ``PIN_OUT[7:0]``, during the current cycle."""
        return self.pin_out & 0xFF

    @property
    def uio_oe(self) -> int:
        """``uio_oe[7:0]``, i.e. ``PIN_OE[7:0]``, during the current cycle."""
        return self.pin_oe & 0xFF

    @property
    def pads(self) -> PadState:
        """All three pad output registers as they stand during this cycle."""
        return PadState(self.uo_out, self.uio_out, self.uio_oe)

    @property
    def ph(self) -> int:
        """The free-running phase counter, ``cycle mod 4``."""
        return self.cycle % SLOT_CLOCKS

    @property
    def pin_in_word(self) -> int:
        """``PIN_IN``: the synchronised pad inputs, bits 0..12."""
        return self._ff2 & ((1 << SYNC_BITS) - 1)

    def pin_in(self, index: int) -> int:
        """``pin_in(index)`` as an instruction in the current cycle would see it."""
        if 0 <= index <= 12:
            return (self._ff2 >> index) & 1
        if 16 <= index <= 21:
            return (self.pin_out >> (index - 8)) & 1
        return 0

    @property
    def caps(self) -> int:
        """``CTRL.CAPS`` as fixed by SEMANTICS 5.

        ``[2:0]`` log2 of the FIFO depth (0 when the FIFOs are not built),
        ``[3]`` FIFOs, ``[4]`` bit engine, ``[5]`` data memory, ``[6]`` boot
        ROM, ``[11:7]`` zero, ``[15:12]`` log2 of ``IMEM_WORDS``.  The M1 build
        with 256 words reads 0x8000.
        """
        value = ((self.imem_words.bit_length() - 1) & 0xF) << 12
        if "FIFO" in self.features:
            value |= max(self.fifo_depth.bit_length() - 1, 0) & 0x7
            value |= 1 << 3
        if "BE" in self.features:
            value |= 1 << 4
        if "DMEM" in self.features:
            value |= 1 << 5
        if "BOOTROM" in self.features:
            value |= 1 << 6
        return value & 0xFFFF

    # ------------------------------------------------------------- the clock
    def step_cycle(self) -> Optional[RetireRecord]:
        """Advance one clock cycle.

        Returns the :class:`RetireRecord` of the slot that is in W during this
        cycle, or ``None`` if that slot is a bubble.  The record's ``x_cycle``
        is the cycle before this one.
        """
        c = self.cycle

        # --- W: the commit computed in X is presented (it lands at this edge).
        w_slot = self._stage_w
        record = w_slot.record if (w_slot is not None and w_slot.valid) else None

        # --- X: evaluate with the state visible in this cycle.
        x_slot = self._stage_x
        if x_slot is not None and x_slot.valid:
            self._execute(x_slot, c)

        # --- D: the synchronous instruction-memory read returns.
        d_slot = self._stage_d
        if d_slot is not None and d_slot.valid:
            d_slot.ir = self.imem.get(d_slot.pc & self.imem_mask, 0)

        # --- F: a new slot starts for the thread that owns this phase.
        t = c % SLOT_CLOCKS
        valid = bool((self.run >> t) & 1) or bool((self.step_req >> t) & 1)
        f_slot = _Slot(thread=t, f_cycle=c, valid=valid)
        if valid:
            f_slot.pc = self.threads[t].pc
            # A valid slot consumes STEP_REQ[t] in its F cycle (SEMANTICS 7).
            self._pending.append(
                Commit(visible_from=c + 1, is_slot=True, step_req_clr=1 << t))
        self._stage_f = f_slot

        if self.on_cycle is not None:
            self.on_cycle(CycleTrace(cycle=c, ph=t, uo_out=self.uo_out,
                                     uio_out=self.uio_out, uio_oe=self.uio_oe,
                                     ui_in=self.ui_in, uio_in=self.uio_in,
                                     retire=record))

        self._edge(c)
        self._stage_w, self._stage_x, self._stage_d = self._stage_x, self._stage_d, f_slot
        self._stage_f = None
        self.cycle = c + 1
        return record

    def run_until(self, predicate: Callable[["Machine"], bool],
                  max_cycles: int = 100000,
                  collect: bool = True) -> RunResult:
        """Step until ``predicate(self)`` holds, or ``max_cycles`` are spent.

        The predicate is tested at each cycle boundary before stepping, so a
        predicate that is already true costs no cycles.  Retire records are
        collected unless ``collect`` is false.
        """
        records: List[RetireRecord] = []
        for spent in range(max_cycles + 1):
            if predicate(self):
                return RunResult(True, spent, records)
            if spent == max_cycles:
                break
            record = self.step_cycle()
            if record is not None and collect:
                records.append(record)
        return RunResult(False, max_cycles, records)

    def run_cycles(self, count: int) -> List[RetireRecord]:
        """Step ``count`` cycles and return every retire record produced."""
        out = []
        for _ in range(count):
            record = self.step_cycle()
            if record is not None:
                out.append(record)
        return out

    # ------------------------------------------------------------ host actions
    def _host(self, **kwargs) -> Commit:
        commit = Commit(visible_from=self.cycle + 1, **kwargs)
        self._pending.append(commit)
        return commit

    def host_write_imem(self, addr: int, word: int) -> None:
        """Write one instruction word.  Dropped (and ``BADOP[15]`` set) while running."""
        self._host(imem_write=(addr & PC_MASK, word & WORD_MASK))

    def host_read_imem(self, addr: int) -> int:
        """Read one instruction word; returns 0 and sets ``BADOP[15]`` while running."""
        if self._imem_host_blocked():
            self._host(badop_set=1 << 15)
            return 0
        return self.imem.get(addr & self.imem_mask, 0)

    def host_set_run(self, mask: int) -> None:
        """``CTRL.RUN <= mask``; bits going 0 to 1 clear ``HALTED``."""
        self._host(run_set=mask & 0xF)

    def host_reset_thread(self, thread: int) -> None:
        """``CTRL.RESET`` bit: PC to ``RESET_PC``, flags 0, ``TD <= NOW``, stack empty."""
        self._host(thread=thread, thread_reset=True)

    def host_step(self, thread: int) -> None:
        """Request one slot of ``thread``; ignored while that thread runs."""
        self._host(step_req_set=1 << thread)

    def host_write_reset_pc(self, thread: int, value: int) -> None:
        """``RESET_PC[t] <= value``."""
        self._host(reset_pc=(thread, value & PC_MASK))

    def host_write_sflags_set(self, mask: int) -> None:
        """``CTRL.SFLAGS`` write: set the bits written as 1."""
        self._host(sflags_set=mask & 0xFF)

    def host_write_sflags_clr(self, mask: int) -> None:
        """``CTRL.SFLAGS_CLR`` write: clear the bits written as 1."""
        self._host(sflags_clr=mask & 0xFF)

    def host_clear_badop(self, mask: int = 0xFFFF) -> None:
        """``CTRL.BADOP`` write-1-to-clear."""
        self._host(badop_clr=mask & 0xFFFF)

    def host_write_pin_out(self, value: int) -> None:
        """Host write of the raw ``PIN_OUT`` register."""
        self._host(pin_out_mask=0xFFFF, pin_out_val=value & WORD_MASK)

    def host_write_pin_oe(self, value: int) -> None:
        """Host write of the raw ``PIN_OE`` register."""
        self._host(pin_oe_mask=0xFF, pin_oe_val=value & 0xFF)

    def host_write_od_mask(self, value: int) -> None:
        """Host write of ``OD_MASK``."""
        self._host(od_mask=value & 0xFF)

    def thread_halted_for_debug(self, thread: int) -> bool:
        """SEMANTICS 7: ``RUN[t] == 0``, ``STEP_REQ[t] == 0``, no slot of ``t`` in flight.

        This, not ``HALTED[t]``, is the condition for host access to ``r0..r7``:
        ``HALTED[t]`` is only the sticky record that a ``HALT`` instruction ran,
        so registers can be preloaded before a thread's first run.
        """
        if (self.run >> thread) & 1 or (self.step_req >> thread) & 1:
            return False
        return not self._slot_in_flight(thread)

    def thread_running(self, thread: int) -> bool:
        """The negation of :meth:`thread_halted_for_debug`, for readability."""
        return not self.thread_halted_for_debug(thread)

    def host_read_debug(self, thread: int, name: str) -> int:
        """Read one DEBUG-space register.  ``r0..r7`` read 0 while the thread runs."""
        th = self.threads[thread]
        if name.lower().startswith("r") and name[1:].isdigit():
            index = int(name[1:])
            if not 0 <= index <= 7:
                raise LoomsimError("no register " + name)
            if self.thread_running(thread):
                return 0
            return th.regs[index]
        key = name.upper()
        if key == "PC":
            return th.pc
        if key == "FLAGS":
            return th.flags
        if key == "TD":
            return th.td
        if key == "DT":
            return th.dt
        if key == "NOW":
            return th.now
        if key == "ACC":
            return th.acc
        if key == "TICK_SEEN":
            return th.tick_seen
        if key == "PREV_PINS":
            return th.prev_pins
        if key == "STEPS":
            return th.steps
        if key == "RS0":
            return th.rs0
        if key == "RS1":
            return th.rs1
        if key == "DEPTH":
            return th.depth
        if key == "WAIT_ACTIVE":
            return th.wait_active
        if key in ("TICK_INT", "TICK_FRAC", "OUTGRP", "INGRP", "TID"):
            return th.csrs[key]
        raise LoomsimError("unknown debug register " + name)

    def host_write_debug(self, thread: int, name: str, value: int) -> None:
        """Write one DEBUG-space register (``r0..r7`` only while halted)."""
        value &= WORD_MASK
        if name.lower().startswith("r") and name[1:].isdigit():
            index = int(name[1:])
            if not 0 <= index <= 7:
                raise LoomsimError("no register " + name)
            if self.thread_running(thread):
                return
            self._host(thread=thread, reg_we=True, reg_rd=index, reg_val=value)
            return
        key = name.upper()
        if key == "PC":
            # A debug PC write also clears WAIT_ACTIVE (SEMANTICS 7).
            self._host(thread=thread, pc=value & PC_MASK, wait_active=0)
        elif key == "FLAGS":
            self._host(thread=thread, flags=value & 7)
        elif key == "TD":
            self._host(thread=thread, td=value)
        elif key == "DT":
            self._host(thread=thread, dt=value)
        elif key == "TICK_INT":
            self._host(thread=thread, tick_int=value)
        elif key == "TICK_FRAC":
            self._host(thread=thread, tick_frac=value & 0xFF)
        elif key == "OUTGRP":
            self._host(thread=thread, outgrp=value & 0x3FF)
        elif key == "INGRP":
            self._host(thread=thread, ingrp=value & 0x3FF)
        elif key == "RS0":
            self._host(thread=thread, rs0=value & PC_MASK)
        elif key == "RS1":
            self._host(thread=thread, rs1=value & PC_MASK)
        elif key == "DEPTH":
            self._host(thread=thread, depth=value & 3)
        elif key == "WAIT_ACTIVE":
            self._host(thread=thread, wait_active=value & 1)
        else:
            raise LoomsimError("debug register " + name + " is not writable")

    # ---- host FIFO side (feature "FIFO")
    def _require_fifo(self) -> None:
        if "FIFO" not in self.features:
            raise LoomsimError("this build has no FIFOs; construct with features={'FIFO'}")

    def host_fifo_push(self, thread: int, word: int) -> None:
        """Push one word into ``INQ[t]``; dropped if full (HOST_PROTOCOL space 3)."""
        self._require_fifo()
        self._host(thread=thread, inq_push=word & WORD_MASK)

    def host_fifo_pop(self, thread: int) -> int:
        """Pop one word from ``OUTQ[t]``; returns 0 and pops nothing if empty."""
        self._require_fifo()
        queue = self.threads[thread].outq
        if not queue:
            return 0
        self._host(thread=thread, outq_pop=True)
        return queue[0]

    def host_fifo_status(self, thread: int) -> Dict[str, int]:
        """Occupancy of both FIFOs of ``thread`` as visible in this cycle."""
        self._require_fifo()
        th = self.threads[thread]
        return {"inq": len(th.inq), "outq": len(th.outq), "depth": self.fifo_depth}

    # ------------------------------------------------------------- the edge
    def _slot_in_flight(self, thread: Optional[int] = None) -> bool:
        """Is a valid slot (of ``thread``, or of any thread) in F, D, X or W?"""
        stages = (self._stage_f, self._stage_d, self._stage_x, self._stage_w)
        return any(slot is not None and slot.valid
                   and (thread is None or slot.thread == thread)
                   for slot in stages)

    def _imem_host_blocked(self) -> bool:
        """SEMANTICS 7 "no step in flight": RUN and STEP_REQ clear, pipeline empty."""
        if self.run or self.step_req:
            return True
        return self._slot_in_flight()

    def _edge(self, c: int) -> None:
        """Apply everything registered at the edge that ends cycle ``c``."""
        landing = [cm for cm in self._pending if cm.visible_from == c + 1]
        self._pending = [cm for cm in self._pending if cm.visible_from != c + 1]

        # The tick generator uses pre-edge TICK_INT/TICK_FRAC and ACC.
        ticks = []
        for th in self.threads:
            period = th.tick_period
            if th.acc + 256 >= period:
                ticks.append((1, (th.acc + 256 - period) & 0xFFFFFF))
            else:
                ticks.append((0, (th.acc + 256) & 0xFFFFFF))
        pre_now = [th.now for th in self.threads]
        pad_sample = self._sample_pads()

        # Host first, thread second: on a same-edge conflict the thread wins.
        for commit in [cm for cm in landing if not cm.is_slot]:
            self._apply_commit(commit, pre_now)
        for commit in [cm for cm in landing if cm.is_slot]:
            self._apply_commit(commit, pre_now)

        slot_commit = [cm for cm in landing if cm.is_slot and cm.steps_inc]
        # SEMANTICS 4: any write to TICK_INT or TICK_FRAC clears ACC at the same
        # edge, by a committed CSRW or by a host debug-space write alike.
        acc_clear = {cm.thread for cm in landing
                     if cm.tick_int is not None or cm.tick_frac is not None}
        for t, th in enumerate(self.threads):
            tick, new_acc = ticks[t]
            if t in acc_clear:
                # The clear wins over the accumulate; NOW does not tick here.
                th.acc = 0
                tick = 0
            else:
                th.acc = new_acc
            th.now = (th.now + tick) & WORD_MASK
            if tick:
                th.tick_seen = 1
            elif any(cm.thread == t for cm in slot_commit):
                th.tick_seen = 0

        self._ff2 = self._ff1
        self._ff1 = pad_sample

    def _sample_pads(self) -> int:
        """The 13-bit pad value the input flops latch at this edge."""
        uio = self.uio_in
        if self.loopback:
            oe = self.uio_oe
            uio = (uio & ~oe) | (self.uio_out & oe)
        value = uio & 0xFF
        value |= (self.ui_in & 0x0F) << 8
        value |= ((self.ui_in >> 7) & 1) << 12
        return value & ((1 << SYNC_BITS) - 1)

    def _apply_commit(self, cm: Commit, pre_now: List[int]) -> None:
        th = self.threads[cm.thread] if cm.thread is not None else None

        if cm.imem_write is not None:
            if self._imem_host_blocked():
                self.badop |= 1 << 15
            else:
                addr, word = cm.imem_write
                self.imem[addr & self.imem_mask] = word
        if cm.reset_pc is not None:
            thread, value = cm.reset_pc
            self.reset_pc[thread] = value
        if cm.thread_reset and th is not None:
            th.pc = self.reset_pc[cm.thread]
            th.z = th.c = th.t = 0
            th.td = pre_now[cm.thread]
            th.depth = 0
            th.wait_active = 0
        if cm.run_set is not None:
            rising = cm.run_set & ~self.run
            self.run = cm.run_set
            self.halted &= ~rising
        if cm.run_clr:
            self.run &= ~cm.run_clr
        if cm.halted_clr:
            self.halted &= ~cm.halted_clr
        if cm.halted_set:
            self.halted |= cm.halted_set
        if cm.step_req_set:
            # A step request for a running thread is ignored (SEMANTICS 7).
            self.step_req |= cm.step_req_set & ~self.run
        if cm.step_req_clr:
            self.step_req &= ~cm.step_req_clr
        if cm.badop_clr:
            self.badop &= ~cm.badop_clr
        if cm.badop_set:
            self.badop |= cm.badop_set
        if cm.swirq_set:
            self.swirq |= cm.swirq_set

        if cm.pin_out_mask:
            self.pin_out = (self.pin_out & ~cm.pin_out_mask) | (cm.pin_out_val & cm.pin_out_mask)
        if cm.pin_oe_mask:
            self.pin_oe = ((self.pin_oe & ~cm.pin_oe_mask)
                           | (cm.pin_oe_val & cm.pin_oe_mask)) & 0xFF
        if cm.od_mask is not None:
            self.od_mask = cm.od_mask & 0xFF
        if cm.sflags_set:
            self.sflags |= cm.sflags_set
        if cm.sflags_clr:
            self.sflags &= ~cm.sflags_clr
        self.sflags &= 0xFF

        if th is None:
            return
        if cm.reg_we:
            th.regs[cm.reg_rd] = cm.reg_val & WORD_MASK
        if cm.pc is not None:
            th.pc = cm.pc & PC_MASK
        if cm.flags is not None:
            th.flags = cm.flags
        if cm.wait_active is not None:
            th.wait_active = cm.wait_active
        if cm.td is not None:
            th.td = cm.td & WORD_MASK
        if cm.dt is not None:
            th.dt = cm.dt & WORD_MASK
        if cm.rs0 is not None:
            th.rs0 = cm.rs0 & PC_MASK
        if cm.rs1 is not None:
            th.rs1 = cm.rs1 & PC_MASK
        if cm.depth is not None:
            th.depth = cm.depth & 3
        if cm.prev_pins is not None:
            th.prev_pins = cm.prev_pins
        if cm.tick_int is not None:
            th.tick_int = cm.tick_int & WORD_MASK
        if cm.tick_frac is not None:
            th.tick_frac = cm.tick_frac & 0xFF
        if cm.outgrp is not None:
            th.outgrp = cm.outgrp & 0x3FF
        if cm.ingrp is not None:
            th.ingrp = cm.ingrp & 0x3FF
        if cm.steps_inc:
            th.steps = (th.steps + 1) & WORD_MASK
        if cm.inq_push is not None and len(th.inq) < self.fifo_depth:
            th.inq.append(cm.inq_push)
        if cm.outq_pop and th.outq:
            th.outq.pop(0)
        if cm.inq_pop and th.inq:
            th.inq.pop(0)
        if cm.outq_push is not None and len(th.outq) < self.fifo_depth:
            th.outq.append(cm.outq_push)

    # --------------------------------------------------------------- execute
    def _visible_sflags(self, x: int) -> int:
        """``SFLAGS`` as seen in X: committed, plus the slot currently in W."""
        value = self.sflags
        for cm in self._pending:
            if cm.is_slot and cm.visible_from == x + 1:
                value |= cm.sflags_set
                value &= ~cm.sflags_clr
        return value & 0xFF

    def _feature_built(self, instr: Instr, ops: Dict[str, int]) -> bool:
        """SEMANTICS 9: an instruction of an unbuilt feature is a ``NOP`` + ``BADOP``."""
        if instr.cls == "be":
            return "BE" in self.features
        if instr.cls == "fifo":
            return "FIFO" in self.features
        if instr.name == "WAITB":
            # Condition 0 asks the bit engine whether it is idle; conditions 1
            # and 2 ask the FIFOs and 3 asks this thread's tick.
            if ops.get("cond") == 0:
                return "BE" in self.features
            return "FIFO" in self.features
        if instr.optional == "DMEM":
            return "DMEM" in self.features
        return True

    def _execute(self, slot: _Slot, x: int) -> None:
        """Evaluate the slot in X at cycle ``x`` and queue its commit."""
        t = slot.thread
        th = self.threads[t]
        ir = slot.ir
        pc = slot.pc
        nxt = alu.next_pc(pc)

        cm = Commit(visible_from=x + 2, thread=t, is_slot=True, steps_inc=True)
        cm.prev_pins = self.pin_in_word
        z, c_flag, t_flag = th.z, th.c, th.t
        done = True
        cm.pc = nxt

        decoded = self.isa.decode(ir)
        if decoded is None or not self._feature_built(decoded[0], decoded[1]):
            # Reserved word, or a feature this build does not have.
            cm.badop_set = 1 << t
            name = None
        else:
            instr, ops = decoded
            name = instr.name
            done = self._dispatch(instr, ops, cm, th, t, pc, nxt, x)
            z, c_flag, t_flag = self._flags_out
        cm.flags = (t_flag << 2) | (c_flag << 1) | z

        slot.record = RetireRecord(
            x_cycle=x, thread=t, pc=pc, ir=ir, done=done, we=cm.reg_we,
            rd=cm.reg_rd, val=cm.reg_val & WORD_MASK if cm.reg_we else 0,
            flags=cm.flags, next_pc=cm.pc & PC_MASK, mnemonic=name)
        self._pending.append(cm)

    def _dispatch(self, instr: Instr, ops: Dict[str, int], cm: Commit,
                  th: ThreadState, t: int, pc: int, nxt: int, x: int) -> bool:
        """Run one decoded instruction.  Returns ``done``; fills ``cm`` in place."""
        name = instr.name
        z, c_flag, t_flag = th.z, th.c, th.t
        regs = th.regs
        done = True

        def write(rd: int, value: int) -> None:
            cm.reg_we = True
            cm.reg_rd = rd
            cm.reg_val = value & WORD_MASK

        # ---------------------------------------------------------- ALU / ALUI
        if instr.cls in ("alu", "alui"):
            if instr.cls == "alu":
                a, b = regs[ops["ra"]], regs[ops["rb"]]
            else:
                a, b = regs[ops["rd"]], ops["imm"]
            if name in ("ADD", "ADDI"):
                value, c_flag = alu.add16(a, b)
            elif name in ("SUB", "SUBI", "CMPI"):
                value, c_flag = alu.sub16(a, b)
            elif name in ("AND", "ANDI"):
                value, c_flag = a & b, 0
            elif name in ("OR", "ORI"):
                value, c_flag = a | b, 0
            elif name in ("XOR", "XORI"):
                value, c_flag = a ^ b, 0
            elif name in ("SHL", "SHLI"):
                value, c_flag = alu.shl16(a, b)
            elif name in ("SHR", "SHRI"):
                value, c_flag = alu.shr16(a, b)
            elif name == "ROR":
                value, c_flag = alu.ror16(a, b)
            else:
                raise LoomsimError("unhandled ALU op " + name)
            z = 1 if (value & WORD_MASK) == 0 else 0
            if name != "CMPI":
                write(ops["rd"], value)

        # ------------------------------------------------------------- LDI/LDIH
        elif name == "LDI":
            write(ops["rd"], ops["imm"] & 0xFF)
        elif name == "LDIH":
            write(ops["rd"], alu.ldih16(regs[ops["rd"]], ops["imm"]))

        # --------------------------------------------------------------- unary
        elif instr.cls == "unary":
            a = regs[ops["ra"]]
            if name == "MOV":
                write(ops["rd"], a)                      # sets no flags
            elif name == "NOT":
                value, c_flag = alu.not16(a)
                z = 1 if value == 0 else 0
                write(ops["rd"], value)
            elif name == "NEG":
                value, c_flag = alu.neg16(a)
                z = 1 if value == 0 else 0
                write(ops["rd"], value)
            elif name == "CMP":
                value, c_flag = alu.sub16(regs[ops["rd"]], a)
                z = 1 if value == 0 else 0
            elif name == "TEST":
                z = 1 if (regs[ops["rd"]] & a) == 0 else 0
                c_flag = 0
            elif name == "REV":
                value, c_flag = alu.rev16(a)
                z = 1 if value == 0 else 0
                write(ops["rd"], value)
            elif name == "PAR":
                value, c_flag = alu.par16(a)
                z = 1 if value == 0 else 0
                write(ops["rd"], value)
            elif name == "SWAP":
                value, c_flag = alu.swap16(a)
                z = 1 if value == 0 else 0
                write(ops["rd"], value)

        # ------------------------------------------------------------- control
        elif name == "JMP":
            cm.pc = ops["abs"] & PC_MASK
        elif name == "CALL":
            cm.rs1 = th.rs0
            cm.rs0 = nxt
            cm.depth = min(th.depth + 1, 2)
            cm.pc = ops["abs"] & PC_MASK
        elif name == "RET":
            if th.depth > 0:
                cm.pc = th.rs0
                cm.rs0 = th.rs1
                cm.depth = th.depth - 1
            else:
                cm.pc = nxt
        elif name == "HALT":
            cm.run_clr = 1 << t
            cm.halted_set = 1 << t
            cm.pc = nxt
        elif instr.cls == "branch":
            if name == "DJNZ":
                value = (regs[ops["rd"]] - 1) & WORD_MASK
                write(ops["rd"], value)
                if value != 0:
                    cm.pc = alu.branch_target(pc, ops["rel"])
            elif name == "JP":
                if self.pin_in(ops["pin"]) == (ops["val"] & 1):
                    cm.pc = alu.branch_target(pc, ops["rel"])
            else:
                taken = {"BZ": z, "BNZ": 1 - z, "BC": c_flag, "BNC": 1 - c_flag,
                         "BT": t_flag, "BNT": 1 - t_flag}[name]
                if taken:
                    cm.pc = alu.branch_target(pc, ops["rel"])

        # ---------------------------------------------------------------- pins
        elif name == "SETP":
            cm.write_pin(ops["pin"], ops["val"], self.od_mask)
        elif name == "OEP":
            pin = ops["pin"]
            if 0 <= pin <= 7:
                cm.pin_oe_mask |= 1 << pin
                cm.pin_oe_val = ((cm.pin_oe_val & ~(1 << pin))
                                 | ((ops["val"] & 1) << pin))
        elif name == "OUT":
            base, count = self._group(th.outgrp)
            value = regs[ops["ra"]]
            for j in range(count):
                cm.write_pin((base + j) % 32, (value >> j) & 1, self.od_mask)
        elif name == "IN":
            base, count = self._group(th.ingrp)
            value = 0
            for j in range(count):
                value |= self.pin_in((base + j) % 32) << j
            write(ops["rd"], value)

        # --------------------------------------------------------------- waits
        elif name == "WAITD":
            first = th.wait_active == 0
            deadline = (th.td + ops["imm"]) & WORD_MASK if first else th.td
            cm.td = deadline
            done = alu.reached(th.now, deadline)
            cm.wait_active = 0 if done else 1
            cm.pc = nxt if done else pc
        elif name == "DLY":
            first = th.wait_active == 0
            deadline = (th.now + ops["imm"]) & WORD_MASK if first else th.dt
            cm.dt = deadline
            done = alu.reached(th.now, deadline)
            cm.wait_active = 0 if done else 1
            cm.pc = nxt if done else pc
        elif name == "SETD":
            cm.td = (th.now + ops["imm"]) & WORD_MASK
        elif name == "NOP":
            pass
        elif name in ("WAITP", "WAITE", "WAITS", "WAITB"):
            cond = self._wait_condition(name, ops, th, x)
            timed = bool(ops.get("tmo", 0))
            if cond:
                done = True
                if timed:
                    t_flag = 0
                if name == "WAITS":
                    cm.sflags_clr |= 1 << ops["flag"]
            elif timed and alu.reached(th.now, th.td):
                done = True
                t_flag = 1
            else:
                done = False
            cm.wait_active = 0 if done else 1
            cm.pc = nxt if done else pc

        # ---------------------------------------------------------------- FIFO
        elif name == "PUSH":
            if len(th.outq) < self.fifo_depth:
                cm.outq_push = regs[ops["ra"]]
                done = True
            else:
                done = False
            cm.wait_active = 0 if done else 1
            cm.pc = nxt if done else pc
        elif name == "POP":
            if th.inq:
                write(ops["rd"], th.inq[0])
                cm.inq_pop = True
                done = True
            else:
                done = False
            cm.wait_active = 0 if done else 1
            cm.pc = nxt if done else pc

        # ----------------------------------------------------------------- CSR
        elif name == "CSRR":
            write(ops["rd"], self._csr_read(th, ops["csr"], x))
        elif name == "CSRW":
            z, c_flag, t_flag = self._csr_write(
                cm, th, t, ops["csr"], regs[ops["ra"]], (z, c_flag, t_flag))

        # -------------------------------------------------------------- SFLAGS
        elif name == "SIG":
            cm.sflags_set |= 1 << ops["flag"]
        elif name == "CLR":
            cm.sflags_clr |= 1 << ops["flag"]

        else:
            raise LoomsimError("no model for instruction " + name)

        self._flags_out = (z & 1, c_flag & 1, t_flag & 1)
        return done

    @staticmethod
    def _group(reg: int) -> Tuple[int, int]:
        """``OUTGRP``/``INGRP``: base in bits 4:0, count in 9:5 capped at 16."""
        return reg & 0x1F, min((reg >> 5) & 0x1F, 16)

    def _wait_condition(self, name: str, ops: Dict[str, int],
                        th: ThreadState, x: int) -> bool:
        if name == "WAITP":
            return self.pin_in(ops["pin"]) == (ops["val"] & 1)
        if name == "WAITE":
            pin = ops["pin"]
            edge = ops["edge"]
            if pin > 12 or edge == 3:
                return False
            previous = (th.prev_pins >> pin) & 1
            current = self.pin_in(pin)
            if edge == 0:
                return previous == 0 and current == 1
            if edge == 1:
                return previous == 1 and current == 0
            return previous != current
        if name == "WAITS":
            return bool((self._visible_sflags(x) >> ops["flag"]) & 1)
        if name == "WAITB":
            cond = ops["cond"]
            if cond == 1:
                return len(th.outq) < self.fifo_depth
            if cond == 2:
                return len(th.inq) > 0
            if cond == 3:
                return bool(th.tick_seen)
            # Condition 0 (BE idle) never reaches here: without the bit engine
            # the instruction is a NOP with BADOP set, see _feature_built.
            return False
        raise LoomsimError("no condition for " + name)

    def _csr_read(self, th: ThreadState, number: int, x: int) -> int:
        """``CSRR``: the CSR value visible in X, zero-extended (0 if unbuilt)."""
        if number == CSR_TICK_INT:
            return th.tick_int & WORD_MASK
        if number == CSR_TICK_FRAC:
            return th.tick_frac & 0xFF
        if number == CSR_OUTGRP:
            return th.outgrp & 0x3FF
        if number == CSR_INGRP:
            return th.ingrp & 0x3FF
        if number == CSR_NOW:
            return th.now & WORD_MASK
        if number == CSR_TD:
            return th.td & WORD_MASK
        if number == CSR_FLAGS:
            return th.flags & 7
        if number == CSR_TID:
            return th.tid & 3
        if number == CSR_OD_MASK:
            return self.od_mask & 0xFF
        if number == CSR_PIN_OUT:
            return self.pin_out & WORD_MASK
        if number == CSR_PIN_OE:
            return self.pin_oe & 0xFF
        if number == CSR_PIN_IN:
            return self.pin_in_word
        if number == CSR_SFLAGS:
            return self._visible_sflags(x)
        return 0                    # write-only and unimplemented CSRs read 0

    def _csr_write(self, cm: Commit, th: ThreadState, t: int, number: int,
                   value: int, flags: Tuple[int, int, int]) -> Tuple[int, int, int]:
        """``CSRW``: the CSR takes ``ra`` truncated to its width."""
        z, c_flag, t_flag = flags
        if number == CSR_TICK_INT:
            cm.tick_int = value & WORD_MASK
        elif number == CSR_TICK_FRAC:
            cm.tick_frac = value & 0xFF
        elif number == CSR_OUTGRP:
            cm.outgrp = value & 0x3FF
        elif number == CSR_INGRP:
            cm.ingrp = value & 0x3FF
        elif number == CSR_TD:
            cm.td = value & WORD_MASK
        elif number == CSR_FLAGS:
            z, c_flag, t_flag = value & 1, (value >> 1) & 1, (value >> 2) & 1
        elif number == CSR_OD_MASK:
            cm.od_mask = value & 0xFF
        elif number == CSR_PIN_OUT:
            cm.pin_out_mask = 0xFFFF
            cm.pin_out_val = value & WORD_MASK
        elif number == CSR_PIN_OE:
            cm.pin_oe_mask = 0xFF
            cm.pin_oe_val = value & 0xFF
        elif number == CSR_SFLAGS:
            cm.sflags_set |= value & 0xFF
        elif number == CSR_HOST_IRQ:
            cm.swirq_set |= 1 << t
        # NOW, TID, PIN_IN are read-only; unimplemented CSRs ignore writes.
        return z, c_flag, t_flag

    # ------------------------------------------------------------------ extras
    def dump_thread(self, thread: int) -> Dict[str, int]:
        """The DEBUG-space view of one thread, for co-simulation comparisons."""
        th = self.threads[thread]
        out = {name: self.host_read_debug(thread, name) for name in DEBUG_REGS}
        out["RUN"] = (self.run >> thread) & 1
        out["HALTED"] = (self.halted >> thread) & 1
        out["BADOP"] = (self.badop >> thread) & 1
        return out

    def __repr__(self) -> str:
        return ("<Machine cycle=%d run=%X halted=%X pc=%s>"
                % (self.cycle, self.run, self.halted,
                   [th.pc for th in self.threads]))
