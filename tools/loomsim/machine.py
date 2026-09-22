"""The Loom golden model: a cycle-based reference implementation.

This module implements ``docs/SEMANTICS.md`` directly.  It is written from that
document, ``docs/HOST_PROTOCOL.md`` and ``isa/isa.yaml`` only; it never reads
the RTL, which is what makes co-simulation meaningful
(``docs/VERIFICATION.md``, METH-1).

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
Host reads answer from the state visible in the cycle they are made in.

Optional features (SEMANTICS 9: an instruction of an unbuilt feature is a
``NOP`` that sets ``BADOP``, and its CSRs read 0) are chosen at construction:

* ``"FIFO"``: ``PUSH``, ``POP``, ``WAITB`` and the host FIFO space (6.7);
* ``"BE"``: the bit engine in manual mode, ``SHO``, ``SHI``, ``LDSR``,
  ``STSR``, ``CRCI``, ``STCRC`` and its CSRs (6.9), plus ``WAITB 0`` (which
  needs ``"FIFO"`` as well, because ``WAITB`` is built with the FIFOs);
* ``"BEENC"`` (M3 slice A, 6.9.1, needs ``"BE"``): the ``ENC``, ``STUFF`` and
  ``DIFF`` fields of ``BE_CFG``, the encoder state of section 5 at debug 0x27,
  and with them NRZI and Manchester coding, USB and CAN stuffing and the
  differential output on ``SHO``/``SHI``.  ``CTRL.VERSION`` reads 3 in such a
  build and 2 without it;
* ``"SETPD"``: the deadline-latched ``SETP pin, v, D`` (6.10).  Without it the
  ``D`` bit is ignored and the instruction is an ordinary ``SETP``.

``Machine(features=())`` is the M1 core: its instruction behaviour, pads and
retire records are unchanged by everything above.  The host registers of
``docs/HOST_PROTOCOL.md`` 0.2 (``ID``, ``VERSION``, the interrupt registers,
the whole DEBUG space) exist in every build.
"""

from __future__ import annotations

import dataclasses
import json
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Tuple, Union

from tools.loomisa import Isa, Instr
from tools.loomisa import load as load_isa

from . import alu
from . import hostmap as H
from .state import (
    THREADS, SLOT_CLOCKS, PC_MASK, WORD_MASK,
    CSR_BE_CFG, CSR_BE_PINS, CSR_BE_RELOAD, CSR_CNT, CSR_CRC, CSR_CRC_INIT,
    CSR_CRC_POLY, CSR_FLAGS, CSR_HOST_IRQ, CSR_INGRP, CSR_NOW, CSR_OD_MASK,
    CSR_OUTGRP, CSR_PIN_IN, CSR_PIN_OE, CSR_PIN_OUT, CSR_SFLAGS, CSR_SR,
    CSR_TD, CSR_TICK_FRAC, CSR_TICK_INT, CSR_TID,
    BE_CFG_CRC_EN, BE_CFG_DIFF, BE_CFG_DIR, BE_CFG_ENC, BE_CFG_ENC_SHIFT,
    BE_CFG_INV, BE_CFG_STUFF, BE_CFG_STUFF_SHIFT, BE_CSRS,
    BE_PINS_MASK, BE_RELOAD_MASK, CNT_MASK,
    ENC_MANCHESTER, ENC_NRZI, STUFF_NONE,
    Commit, CycleTrace, PadState, RetireRecord, ThreadState, be_cfg_stored,
)

#: Pin index groups (ARCHITECTURE 3.1 / isa.yaml ``pins``).
BIDIR_PINS = range(0, 8)
IN_PINS = range(8, 13)
OUT_PINS = range(16, 22)
SYNC_BITS = 13          # pin_in(0..12) goes through the two-flop synchroniser

DEFAULT_IMEM_WORDS = 1024
DEFAULT_FIFO_DEPTH = 4

#: The named view of one thread that :meth:`Machine.dump_thread` returns.  It
#: is kept exactly as the M1 model had it (co-simulation compares it); the
#: full HOST_PROTOCOL DEBUG space by number is :meth:`Machine.dump_debug_space`.
DEBUG_REGS = ("r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "PC", "FLAGS",
              "TD", "NOW", "STEPS", "RS0", "RS1", "DEPTH", "WAIT_ACTIVE",
              "TICK_INT", "TICK_FRAC", "OUTGRP", "INGRP", "DT", "ACC",
              "TICK_SEEN", "PREV_PINS")

#: Debug-register names that are model views rather than HOST_PROTOCOL
#: addresses, and whether the host may write them.
_DEBUG_VIEWS_WRITABLE = frozenset({"RS1", "DEPTH"})
#: The six fields of debug 0x27 (SEMANTICS 6.9.1) by name, as model views.
_ENC_VIEWS = ("ENC_LVL", "ENC_RUN", "ENC_RVAL", "ENC_PEND", "ENC_HALF",
              "ENC_FIRST")
_DEBUG_VIEWS_READ_ONLY = frozenset({"ACC", "PREV_PINS", "LAT_VALID", "LAT_PIN",
                                    "LAT_VAL", "INQ_CNT", "OUTQ_CNT"}
                                   | set(_ENC_VIEWS))
#: HOST_PROTOCOL debug registers (by name) that are read-only.
_DEBUG_READ_ONLY_NAMES = frozenset({"NOW", "TID", "FIFO_CNT"})

RegRef = Union[int, str]


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
        features: optional features that are built in this configuration,
            any of ``"FIFO"``, ``"BE"``, ``"BEENC"`` (which needs ``"BE"``),
            ``"SETPD"`` (and ``"DMEM"``, ``"BOOTROM"``, which only set their
            ``CAPS`` bits).  The default (nothing) is the M1 build: every
            instruction of an unbuilt feature is a ``NOP`` that sets
            ``BADOP``.
        imem_words: instruction memory size; addresses wrap within it.
        fifo_depth: INQ and OUTQ depth per thread, a power of two from 2 to 8
            (SEMANTICS 6.7); checked when ``"FIFO"`` is built.
        loopback: drive ``uio_in`` from ``uio_out`` for bits with ``uio_oe`` set,
            as the Tiny Tapeout pad does.
        on_cycle: optional callable invoked once per cycle with a
            :class:`~tools.loomsim.state.CycleTrace`.
        isa: a preloaded :class:`tools.loomisa.Isa` (one is loaded if omitted).
        version: what ``CTRL.VERSION`` reads; by default 3 in a ``"BEENC"``
            build (SEMANTICS 6.9.1) and 2 (HOST_PROTOCOL 0.2) without it.

    The public state is :attr:`threads` (a list of
    :class:`~tools.loomsim.state.ThreadState`) plus the global registers
    :attr:`run`, :attr:`halted`, :attr:`step_req`, :attr:`badop`,
    :attr:`sflags`, :attr:`od_mask`, :attr:`pin_out`, :attr:`pin_oe`,
    :attr:`swirq`, :attr:`irq_en`, :attr:`irq_en2` and the registered output
    :attr:`host_irq`.
    """

    def __init__(self, image: Optional[Mapping[int, int]] = None,
                 features: Iterable[str] = (),
                 imem_words: int = DEFAULT_IMEM_WORDS,
                 fifo_depth: int = DEFAULT_FIFO_DEPTH,
                 loopback: bool = False,
                 on_cycle: Optional[Callable[[CycleTrace], None]] = None,
                 isa: Optional[Isa] = None,
                 version: Optional[int] = None):
        if imem_words & (imem_words - 1) or not 1 <= imem_words <= 1024:
            raise LoomsimError("imem_words must be a power of two up to 1024")
        built = frozenset(str(f).upper() for f in features)
        unknown = built - H.FEATURES
        if unknown:
            raise LoomsimError("unknown feature(s) %s; known: %s"
                               % (", ".join(sorted(unknown)), ", ".join(sorted(H.FEATURES))))
        if "FIFO" in built and fifo_depth not in H.FIFO_DEPTHS:
            raise LoomsimError("fifo_depth must be 2, 4 or 8 (SEMANTICS 6.7), not %r"
                               % (fifo_depth,))
        if "BEENC" in built and "BE" not in built:
            raise LoomsimError("BEENC is the slice-A part of the bit engine "
                               "(SEMANTICS 6.9.1); build it with BE")
        self.isa = isa if isa is not None else load_isa()
        self.features = built
        self._fifo = "FIFO" in built
        self._be = "BE" in built
        self._be_enc = "BEENC" in built
        self._setpd = "SETPD" in built
        self.imem_words = imem_words
        self.imem_mask = imem_words - 1
        self.fifo_depth = fifo_depth
        self.loopback = loopback
        self.on_cycle = on_cycle
        if version is None:
            # SEMANTICS 6.9.1: VERSION reads 3 from slice A on.
            version = H.ENC_VERSION if self._be_enc else H.DEFAULT_VERSION
        self.version = version & WORD_MASK
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
        self.irq_en = 0
        self.irq_en2 = 0
        self.host_irq = 0
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
        """``uo_out[5:0]``, i.e. ``PIN_OUT[13:8]``, during the current cycle.

        ``uo_out[6]`` is :attr:`host_irq`; ``uo_out[7]`` (HOST_MISO) belongs to
        the SPI port, which the model does not contain.
        """
        return (self.pin_out >> 8) & 0x3F

    @property
    def uio_out(self) -> int:
        """``uio_out[7:0]`` during the current cycle.

        ``PIN_OUT[7:0]`` with the open-drain pins masked out (SEMANTICS 3): a
        pin in open-drain mode never drives high, whatever order the three
        registers were written in.
        """
        return self.pin_out & 0xFF & ~self.od_mask & 0xFF

    @property
    def uio_oe(self) -> int:
        """``uio_oe[7:0]`` during the current cycle.

        ``PIN_OE[7:0]``, except that an open-drain pin whose ``PIN_OUT`` bit is
        1 is released: under ``OD_MASK`` a 1 means "let go" everywhere else
        too (SEMANTICS 3 and 6.3).
        """
        return self.pin_oe & 0xFF & ~(self.od_mask & self.pin_out) & 0xFF

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
        ``[3]`` FIFOs, ``[4]`` bit engine (manual mode), ``[5]`` data memory,
        ``[6]`` boot ROM, ``[7]`` deadline-latched ``SETP``, ``[8]`` bit engine
        auto mode (M3 slice C, never set here), ``[9]`` the slice-A encoders,
        stuffing and DIFF (feature ``"BEENC"``, 6.9.1), ``[11:10]`` zero,
        ``[15:12]`` log2 of ``IMEM_WORDS``.  The M1 build with 256 words reads
        0x8000.
        """
        value = ((self.imem_words.bit_length() - 1) & 0xF) << 12
        if self._fifo:
            value |= max(self.fifo_depth.bit_length() - 1, 0) & 0x7
            value |= H.CAPS_FIFO
        if self._be:
            value |= H.CAPS_BE
        if "DMEM" in self.features:
            value |= H.CAPS_DMEM
        if "BOOTROM" in self.features:
            value |= H.CAPS_BOOTROM
        if self._setpd:
            value |= H.CAPS_SETPD
        if self._be_enc:
            value |= H.CAPS_BEENC
        return value & 0xFFFF

    @property
    def irq_stat(self) -> int:
        """``CTRL.IRQ_STAT = {SFLAGS[7:0], INQ_NOT_FULL[3:0], OUTQ_NOT_EMPTY[3:0]}``.

        ``SFLAGS`` is the register (not the W-to-X forwarded value).  The two
        FIFO fields read 0 when the FIFOs are not built (HOST_PROTOCOL).
        """
        inq_not_full = outq_not_empty = 0
        if self._fifo:
            for t, th in enumerate(self.threads):
                if len(th.inq) < self.fifo_depth:
                    inq_not_full |= 1 << t
                if th.outq:
                    outq_not_empty |= 1 << t
        return H.pack_irq_stat(self.sflags, inq_not_full, outq_not_empty)

    @property
    def irq_stat2(self) -> int:
        """``CTRL.IRQ_STAT2 = {12'b0, HALTED[3:0]}``."""
        return self.halted & 0xF

    def _irq_level(self) -> int:
        """SEMANTICS 6.8: the value ``HOST_IRQ`` takes at the coming edge."""
        return int(bool((self.irq_stat & self.irq_en)
                        or (self.irq_stat2 & self.irq_en2)
                        or (self.swirq & 0xF)))

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
                                     retire=record, host_irq=self.host_irq))

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

    @staticmethod
    def _check_thread(thread: int) -> None:
        if not isinstance(thread, int) or not 0 <= thread < THREADS:
            raise LoomsimError("thread %r outside 0..3" % (thread,))

    def host_write_imem(self, addr: int, word: int) -> None:
        """Write one instruction word.  Dropped (and ``BADOP[15]`` set) while running."""
        self._host(imem_write=(addr & PC_MASK, word & WORD_MASK))

    def host_read_imem(self, addr: int) -> int:
        """Read one instruction word; returns 0 and sets ``BADOP[15]`` while running."""
        if self._imem_host_blocked():
            self._host(badop_set=H.BADOP_ACCESS)
            return 0
        return self.imem.get(addr & self.imem_mask, 0)

    def host_set_run(self, mask: int) -> None:
        """``CTRL.RUN <= mask``; bits going 0 to 1 clear ``HALTED``."""
        self._host(run_set=mask & 0xF)

    def host_reset_thread(self, thread: int) -> None:
        """``CTRL.RESET`` bit (SEMANTICS 7, 6.7, 6.10).

        ``PC <= RESET_PC[t]``, flags 0, ``DEPTH <= 0`` (``RS0``/``RS1`` kept),
        ``WAIT_ACTIVE <= 0``, ``TD <=`` the ``NOW`` visible in this cycle, both
        FIFOs of the thread emptied and its staged pin write discarded.
        Registers and CSRs are untouched.
        """
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

    def host_write_irq_en(self, value: int) -> None:
        """``CTRL.IRQ_EN <= value``: the mask over ``IRQ_STAT`` (SEMANTICS 6.8)."""
        self._host(irq_en=value & WORD_MASK)

    def host_write_irq_en2(self, value: int) -> None:
        """``CTRL.IRQ_EN2 <= value[3:0]``: the mask over ``IRQ_STAT2``."""
        self._host(irq_en2=value & H.IRQ_EN2_MASK)

    def host_clear_swirq(self, mask: int = 0xF) -> None:
        """``CTRL.SWIRQ`` write-1-to-clear.  A ``CSRW HOST_IRQ`` of the same
        thread committing at the same edge wins (SEMANTICS 7)."""
        self._host(swirq_clr=mask & 0xF)

    # ---- the CTRL space by address or name (HOST_PROTOCOL space 0)
    @staticmethod
    def _ctrl_name(reg: RegRef, for_write: bool) -> Optional[str]:
        if isinstance(reg, str):
            name = reg.strip().upper()
            if name not in H.CTRL:
                raise LoomsimError("no CTRL register " + reg)
            if for_write and name in H.CTRL_READ_ONLY:
                raise LoomsimError("CTRL register %s is read-only" % name)
            return name
        return H.CTRL_NAMES.get(reg & WORD_MASK)

    def host_read_ctrl(self, reg: RegRef) -> int:
        """Read one CTRL register, by address (0x00..0x1C) or name.

        Write-only registers (``RESET``, ``SFLAGS_CLR``) and addresses that
        name nothing read 0.  The value is the one visible in this cycle.
        """
        name = self._ctrl_name(reg, for_write=False)
        if name == "ID":
            return H.ID_VALUE
        if name == "VERSION":
            return self.version
        if name == "RUN":
            return self.run & 0xF
        if name == "HALTED":
            return self.halted & 0xF
        if name is not None and name.startswith("RESET_PC"):
            return self.reset_pc[int(name[-1])] & PC_MASK
        if name == "IRQ_EN":
            return self.irq_en & WORD_MASK
        if name == "IRQ_STAT":
            return self.irq_stat
        if name == "IRQ_STAT2":
            return self.irq_stat2
        if name == "SFLAGS":
            return self.sflags & 0xFF
        if name == "OD_MASK":
            return self.od_mask & 0xFF
        if name == "PIN_OUT":
            return self.pin_out & WORD_MASK
        if name == "PIN_OE":
            return self.pin_oe & 0xFF
        if name == "PIN_IN":
            return self.pin_in_word
        if name == "CAPS":
            return self.caps
        if name == "BADOP":
            return self.badop & WORD_MASK
        if name == "SWIRQ":
            return self.swirq & 0xF
        if name == "IRQ_EN2":
            return self.irq_en2 & H.IRQ_EN2_MASK
        return 0

    def host_write_ctrl(self, reg: RegRef, value: int) -> None:
        """Write one CTRL register, by address or name; commits at the next edge.

        By address, read-only registers and unused addresses ignore the write,
        as the port does; by name, writing a read-only register raises.
        """
        name = self._ctrl_name(reg, for_write=True)
        value &= WORD_MASK
        if name == "RUN":
            self.host_set_run(value)
        elif name == "RESET":
            for t in range(THREADS):
                if (value >> t) & 1:
                    self.host_reset_thread(t)
        elif name is not None and name.startswith("RESET_PC"):
            self.host_write_reset_pc(int(name[-1]), value)
        elif name == "IRQ_EN":
            self.host_write_irq_en(value)
        elif name == "SFLAGS":
            self.host_write_sflags_set(value)
        elif name == "SFLAGS_CLR":
            self.host_write_sflags_clr(value)
        elif name == "OD_MASK":
            self.host_write_od_mask(value)
        elif name == "PIN_OUT":
            self.host_write_pin_out(value)
        elif name == "PIN_OE":
            self.host_write_pin_oe(value)
        elif name == "BADOP":
            self.host_clear_badop(value)
        elif name == "SWIRQ":
            self.host_clear_swirq(value)
        elif name == "IRQ_EN2":
            self.host_write_irq_en2(value)
        # ID, VERSION, HALTED, IRQ_STAT*, PIN_IN, CAPS: read-only, ignored.

    # ---- run state as the host sees it
    def thread_halted_for_debug(self, thread: int) -> bool:
        """SEMANTICS 7: ``RUN[t] == 0``, ``STEP_REQ[t] == 0``, no slot of ``t`` in flight.

        This, not ``HALTED[t]``, is the condition for every host debug write
        and for host reads of ``r0..r7``: ``HALTED[t]`` is only the sticky
        record that a ``HALT`` instruction ran, so registers can be preloaded
        before a thread's first run.
        """
        if (self.run >> thread) & 1 or (self.step_req >> thread) & 1:
            return False
        return not self._slot_in_flight(thread)

    def thread_running(self, thread: int) -> bool:
        """The negation of :meth:`thread_halted_for_debug`, for readability."""
        return not self.thread_halted_for_debug(thread)

    # ---- the DEBUG space (HOST_PROTOCOL space 4)
    @staticmethod
    def _debug_key(name: str) -> str:
        text = str(name).strip()
        if text[:1] in ("r", "R") and text[1:].isdigit():
            index = int(text[1:])
            if not 0 <= index <= 7:
                raise LoomsimError("no register " + text)
            return "r%d" % index
        return text.upper()

    def _debug_number(self, key: str) -> Optional[int]:
        """HOST_PROTOCOL register number of a debug name, or None for a view."""
        if key in H.DEBUG:
            return H.DEBUG[key]
        csr = self.isa.csr_by_name.get(key)
        if csr is not None and csr < H.DEBUG_CSR_COUNT:
            return H.DEBUG_CSR_BASE + csr
        return None

    def host_read_debug(self, thread: int, reg: RegRef) -> int:
        """Read one DEBUG-space register, by name or by number (0x00..0xFF).

        ``r0..r7`` read 0 unless the thread is halted in the sense of
        :meth:`thread_halted_for_debug`; everything else reads at any time.
        Registers of a feature that is not built read 0.  Besides the
        HOST_PROTOCOL names (``PC``, ``STEPS``, ``LAT``, ``FIFO_CNT``, ``ENC``,
        the CSR names of the window 0x10..0x1F ...) the model offers the views
        ``RS1``, ``DEPTH``, ``ACC``, ``PREV_PINS``, ``LAT_VALID``,
        ``LAT_PIN``, ``LAT_VAL``, ``INQ_CNT``, ``OUTQ_CNT`` and the six
        encoder fields ``ENC_LVL``, ``ENC_RUN``, ``ENC_RVAL``, ``ENC_PEND``,
        ``ENC_HALF`` and ``ENC_FIRST``.
        """
        self._check_thread(thread)
        if isinstance(reg, int):
            return self._debug_read(thread, reg & 0xFF)
        key = self._debug_key(reg)
        number = self._debug_number(key)
        if number is not None:
            return self._debug_read(thread, number)
        th = self.threads[thread]
        if key == "RS1":
            return th.rs1
        if key == "DEPTH":
            return th.depth
        if key == "ACC":
            return th.acc
        if key == "PREV_PINS":
            return th.prev_pins
        if key in ("LAT_VALID", "LAT_PIN", "LAT_VAL"):
            if not self._setpd:
                return 0
            return {"LAT_VALID": th.lat_valid, "LAT_PIN": th.lat_pin,
                    "LAT_VAL": th.lat_val}[key]
        if key in ("INQ_CNT", "OUTQ_CNT"):
            if not self._fifo:
                return 0
            return len(th.inq) if key == "INQ_CNT" else len(th.outq)
        if key in _ENC_VIEWS:
            if not self._be_enc:
                return 0
            return getattr(th, key.lower())
        raise LoomsimError("unknown debug register " + str(reg))

    def host_write_debug(self, thread: int, reg: RegRef, value: int) -> None:
        """Write one DEBUG-space register, by name or by number (0x00..0xFF).

        Every debug write is dropped unless the thread is halted in the sense
        of :meth:`thread_halted_for_debug`, judged in the cycle of the call;
        an accepted write commits at the edge that ends that cycle.  By
        number, read-only and unused registers ignore the write as the port
        does; by name, an unknown or read-only register raises.  A write of
        a register whose feature is not built is ignored.
        """
        self._check_thread(thread)
        if isinstance(reg, int):
            number: Optional[int] = reg & 0xFF
            key = None
        else:
            key = self._debug_key(reg)
            number = self._debug_number(key)
            if number is None:
                if key in _DEBUG_VIEWS_READ_ONLY:
                    raise LoomsimError("debug register " + str(reg) + " is not writable")
                if key not in _DEBUG_VIEWS_WRITABLE:
                    raise LoomsimError("unknown debug register " + str(reg))
            elif key in _DEBUG_READ_ONLY_NAMES:
                raise LoomsimError("debug register " + str(reg) + " is not writable")
        if not self.thread_halted_for_debug(thread):
            return                                   # dropped (SEMANTICS 7)
        value &= WORD_MASK
        if number is not None:
            self._debug_write(thread, number, value)
        elif key == "RS1":
            self._host(thread=thread, rs1=value & PC_MASK)
        elif key == "DEPTH":
            self._host(thread=thread, depth=value & 3)

    def _debug_read(self, t: int, number: int) -> int:
        """A DEBUG-space read by register number, as the port answers it."""
        th = self.threads[t]
        if 0 <= number <= 7:
            return 0 if self.thread_running(t) else th.regs[number] & WORD_MASK
        if number == 0x08:
            return th.pc & PC_MASK
        if number == 0x09:
            return th.flags & 7
        if number == 0x0A:
            return th.td & WORD_MASK
        if number == 0x0B:
            return th.now & WORD_MASK
        if number == 0x0C:
            return th.sr & WORD_MASK if self._be else 0
        if number == 0x0D:
            return th.cnt & CNT_MASK if self._be else 0
        if number == 0x0E:
            return th.crc & WORD_MASK if self._be else 0
        if number == 0x0F:
            return th.rs0 & PC_MASK
        if H.DEBUG_CSR_BASE <= number < H.DEBUG_CSR_BASE + H.DEBUG_CSR_COUNT:
            return self._thread_csr_value(th, number - H.DEBUG_CSR_BASE)
        if number == 0x20:
            return th.steps & WORD_MASK
        if number == 0x21:
            return H.pack_rs1_depth(th.rs1, th.depth)
        if number == 0x22:
            return th.wait_active & 1
        if number == 0x23:
            return th.dt & WORD_MASK
        if number == 0x24:
            return th.tick_seen & 1
        if number == 0x25:
            return th.lat if self._setpd else 0
        if number == 0x26:
            return H.pack_fifo_counts(len(th.inq), len(th.outq)) if self._fifo else 0
        if number == H.DEBUG_ENC:
            # HOST_PROTOCOL 0x27: reads 0 until slice A is built.
            return th.enc if self._be_enc else 0
        return 0

    def _debug_write(self, t: int, number: int, value: int) -> None:
        """A DEBUG-space write by register number (already gated on halted)."""
        if 0 <= number <= 7:
            self._host(thread=t, reg_we=True, reg_rd=number, reg_val=value)
        elif number == 0x08:
            # A debug PC write also clears WAIT_ACTIVE (SEMANTICS 7).
            self._host(thread=t, pc=value & PC_MASK, wait_active=0)
        elif number == 0x09:
            self._host(thread=t, flags=value & 7)
        elif number == 0x0A:
            self._host(thread=t, td=value)      # not a rule-2 write (D-028)
        elif number in (0x0C, 0x0D, 0x0E):
            if self._be:
                csr = {0x0C: CSR_SR, 0x0D: CSR_CNT, 0x0E: CSR_CRC}[number]
                self._host(thread=t, **self._be_csr_fields(csr, value))
        elif number == 0x0F:
            self._host(thread=t, rs0=value & PC_MASK)
        elif H.DEBUG_CSR_BASE <= number < H.DEBUG_CSR_BASE + H.DEBUG_CSR_COUNT:
            self._debug_csr_write(t, number - H.DEBUG_CSR_BASE, value)
        elif number == 0x20:
            self._host(thread=t, steps=value)
        elif number == 0x21:
            self._host(thread=t, rs1=value & PC_MASK, depth=(value >> 10) & 3)
        elif number == 0x22:
            self._host(thread=t, wait_active=value & 1)
        elif number == 0x23:
            self._host(thread=t, dt=value)
        elif number == 0x24:
            self._host(thread=t, tick_seen=value & 1)
        elif number == 0x25:
            if self._setpd:
                valid, val, pin = H.unpack_lat(value)
                self._host(thread=t, lat_set=(valid, pin, val))
        elif number == H.DEBUG_ENC:
            if self._be_enc:
                lvl, run, rval, pend, half, first = H.unpack_enc(value)
                self._host(thread=t, enc_lvl=lvl, enc_run=run, enc_rval=rval,
                           enc_pend=pend, enc_half=half, enc_first=first)
        # 0x0B NOW, 0x26 FIFO counts and unused numbers ignore writes.

    def _debug_csr_write(self, t: int, csr: int, value: int) -> None:
        """Host write of the thread's CSR ``csr`` (0x00..0x0F) through the window."""
        if csr == CSR_TICK_INT:
            self._host(thread=t, tick_int=value)
        elif csr == CSR_TICK_FRAC:
            self._host(thread=t, tick_frac=value & 0xFF)
        elif csr == CSR_OUTGRP:
            self._host(thread=t, outgrp=value & 0x3FF)
        elif csr == CSR_INGRP:
            self._host(thread=t, ingrp=value & 0x3FF)
        elif csr == CSR_TD:
            self._host(thread=t, td=value)      # not a rule-2 write (D-028)
        elif csr == CSR_FLAGS:
            self._host(thread=t, flags=value & 7)
        elif csr in BE_CSRS:
            if self._be:
                self._host(thread=t, **self._be_csr_fields(csr, value))
        # NOW and TID are read-only.

    def _be_csr_fields(self, csr: int, value: int) -> Dict[str, int]:
        """Commit fields for a write of bit-engine CSR ``csr`` (truncated to width)."""
        if csr == CSR_BE_CFG:
            return {"be_cfg": be_cfg_stored(value, self._be_enc)}
        if csr == CSR_BE_PINS:
            return {"be_pins": value & BE_PINS_MASK}
        if csr == CSR_BE_RELOAD:
            return {"be_reload": value & BE_RELOAD_MASK}
        if csr == CSR_CRC_POLY:
            return {"crc_poly": value & WORD_MASK}
        if csr == CSR_CRC_INIT:
            return {"crc_init": value & WORD_MASK}
        if csr == CSR_SR:
            return {"sr": value & WORD_MASK}
        if csr == CSR_CNT:
            return {"cnt": value & CNT_MASK}
        if csr == CSR_CRC:
            return {"crc": value & WORD_MASK}
        raise LoomsimError("CSR 0x%02X is not a bit-engine CSR" % csr)

    # ---- host FIFO side (feature "FIFO", SEMANTICS 6.7)
    def _require_fifo(self) -> None:
        if not self._fifo:
            raise LoomsimError("this build has no FIFOs; construct with features={'FIFO'}")

    def host_fifo_push(self, thread: int, word: int) -> None:
        """Push one word into ``INQ[t]`` at the edge that ends this cycle.

        Accepted iff ``INQ_CNT[t] < FIFO_DEPTH`` just before that edge (a thread
        ``POP`` committing at the same edge does not make room for it);
        otherwise the word is dropped and ``BADOP[14]`` is set at that edge.
        """
        self._require_fifo()
        self._host(thread=thread, inq_push=word & WORD_MASK)

    def host_fifo_peek(self, thread: int) -> Optional[int]:
        """The head of ``OUTQ[t]`` as visible in this cycle, or ``None`` if empty.

        No side effect.  A transport uses this for the SPI port's peek, then
        :meth:`host_fifo_pop` at the edge where the word has gone out.
        """
        self._require_fifo()
        queue = self.threads[thread].outq
        return queue[0] if queue else None

    def host_fifo_pop(self, thread: int) -> int:
        """Pop one word from ``OUTQ[t]``, atomically, at the edge that ends this cycle.

        Returns the head if ``OUTQ_CNT[t] > 0`` in this cycle and removes it at
        the edge; otherwise returns 0, removes nothing and sets ``BADOP[14]``
        at the edge.  (The port splits a pop into a peek and a pop; see
        :meth:`host_fifo_peek`.)
        """
        self._require_fifo()
        queue = self.threads[thread].outq
        if not queue:
            self._host(badop_set=H.BADOP_FIFO)
            return 0
        self._host(thread=thread, outq_pop=True)
        return queue[0]

    def host_fifo_error(self) -> None:
        """Set ``BADOP[14]`` at the edge that ends this cycle, and nothing else.

        The host FIFO error of a pop whose peek found ``OUTQ`` empty
        (SEMANTICS 6.7).  :meth:`host_fifo_pop` raises it too when the queue is
        empty in the pop cycle, but a thread ``PUSH`` may have filled the queue
        between the peek and the pop: the port still pops nothing and still
        flags the error, which is what this call is for.
        """
        self._require_fifo()
        self._host(badop_set=H.BADOP_FIFO)

    def host_fifo_status(self, thread: int) -> Dict[str, int]:
        """Occupancy of both FIFOs of ``thread`` as visible in this cycle."""
        self._require_fifo()
        th = self.threads[thread]
        return {"inq": len(th.inq), "outq": len(th.outq), "depth": self.fifo_depth}

    def host_fifo_status_word(self, thread: int) -> int:
        """The FIFO-space status word at ``0x0100 + t`` (HOST_PROTOCOL space 3)."""
        self._require_fifo()
        th = self.threads[thread]
        return H.pack_fifo_status(len(th.inq), len(th.outq), self.fifo_depth)

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

        # HOST_IRQ is a register fed from the values visible in this cycle.
        irq_next = self._irq_level()

        # The tick generator uses pre-edge TICK_INT/TICK_FRAC and ACC.
        ticks = []
        for th in self.threads:
            period = th.tick_period
            if th.acc + 256 >= period:
                ticks.append((1, (th.acc + 256 - period) & 0xFFFFFF))
            else:
                ticks.append((0, (th.acc + 256) & 0xFFFFFF))
        pre_now = [th.now for th in self.threads]
        pre_od_mask = self.od_mask
        pre_lat = [(th.lat_valid, th.lat_pin, th.lat_val) for th in self.threads]
        pre_td = [th.td for th in self.threads]
        pad_sample = self._sample_pads()

        # Host first, thread second: on a same-edge conflict the thread wins.
        host = [cm for cm in landing if not cm.is_slot]
        slots = [cm for cm in landing if cm.is_slot]
        for commit in host:
            self._apply_commit(commit, pre_now)
        slot_out = slot_oe = slot_index = 0
        for commit in slots:
            self._apply_commit(commit, pre_now)
            slot_out |= commit.pin_out_mask
            slot_oe |= commit.pin_oe_mask
            slot_index |= commit.pin_index_mask

        slot_commit = [cm for cm in slots if cm.steps_inc]
        # SEMANTICS 4: any write to TICK_INT or TICK_FRAC clears ACC at the same
        # edge, by a committed CSRW or by a host debug-space write alike.
        acc_clear = {cm.thread for cm in landing
                     if cm.tick_int is not None or cm.tick_frac is not None}
        host_tick_seen = {cm.thread: cm.tick_seen for cm in host
                          if cm.tick_seen is not None}
        ticked = [0] * THREADS
        for t, th in enumerate(self.threads):
            tick, new_acc = ticks[t]
            if t in acc_clear:
                # The clear wins over the accumulate; NOW does not tick here.
                th.acc = 0
                tick = 0
            else:
                th.acc = new_acc
            th.now = (th.now + tick) & WORD_MASK
            ticked[t] = tick
            if tick:
                th.tick_seen = 1                     # a tick wins over any clear
            elif t in host_tick_seen:
                th.tick_seen = host_tick_seen[t] & 1
            else:
                # SEMANTICS 4: a slot's commit clears only the TICK_SEEN it saw
                # in its X cycle, so a tick at edge x + 1 survives the commit.
                for cm in slot_commit:
                    if cm.thread == t:
                        th.tick_seen &= ~cm.seen_tick & 1

        if self._setpd:
            self._deadline_latches(landing, pre_lat, pre_td, pre_od_mask, ticked,
                                   slot_out, slot_oe, slot_index)

        self.host_irq = irq_next
        self._ff2 = self._ff1
        self._ff1 = pad_sample

    def _deadline_latches(self, landing: List[Commit],
                          pre_lat: List[Tuple[int, int, int]],
                          pre_td: List[int], od_mask: int,
                          ticked: List[int], slot_out: int, slot_oe: int,
                          slot_index: int) -> None:
        """SEMANTICS 6.10 at one edge, after every commit and the tick have landed.

        A latch valid before the edge fires when (rule 1) ``NOW`` ticked to
        exactly ``TD`` and the thread's own slot did not write ``TD`` at this
        edge, or (rule 2) the thread's own slot wrote ``TD`` at this edge and
        ``reached(NOW', TD')`` holds for the values after it.  A host debug
        write of ``TD`` is neither (D-028): it is not a rule-2 write, and at
        its own edge rule 1 compares against ``TD`` as it was before the edge
        (``pre_td``); from the next edge on rule 1 sees the written value.
        A latch loaded at this edge (``SETP ... D`` or a host write of debug
        0x25) cannot fire before the next edge; the old content may still fire
        at the loading edge.  ``CTRL.RESET`` discards: a latch whose thread is
        reset at this edge does not fire.

        The staged write uses the pin-write rule of 6.3 with ``OD_MASK`` as
        visible before the edge.  A slot's ordinary pin write (SETP, OUT, SHO)
        to the same pin index at this edge wins, and the whole staged write is
        dropped; other bits a slot commit wrote (OEP, CSRW PIN_OUT/PIN_OE) keep
        the slot's value; the staged write beats a host write of the same
        bits.  If staged writes of two threads land on one bit at one edge the
        higher-numbered thread's value stays.
        """
        for t, th in enumerate(self.threads):
            valid, pin, value = pre_lat[t]
            mine = [cm for cm in landing if cm.thread == t]
            if not valid or any(cm.thread_reset for cm in mine):
                continue
            if any(cm.td_written for cm in mine):
                fire = alu.reached(th.now, th.td)                  # rule 2
            else:
                fire = bool(ticked[t]) and th.now == pre_td[t]     # rule 1
            if not fire:
                continue
            if not (slot_index >> pin) & 1:
                staged = Commit(visible_from=self.cycle + 1)
                staged.write_pin(pin, value, od_mask)
                out_mask = staged.pin_out_mask & ~slot_out
                oe_mask = staged.pin_oe_mask & ~slot_oe
                self.pin_out = ((self.pin_out & ~out_mask)
                                | (staged.pin_out_val & out_mask)) & WORD_MASK
                self.pin_oe = ((self.pin_oe & ~oe_mask)
                               | (staged.pin_oe_val & oe_mask)) & 0xFF
            if not any(cm.lat_set is not None for cm in mine):
                th.lat_valid = 0

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
                self.badop |= H.BADOP_ACCESS
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
            # SEMANTICS 6.7, 6.9.1 and 6.10: the FIFOs are emptied, the staged
            # pin write is discarded and the encoder state is cleared (all are
            # already empty or zero in builds without those features).
            th.inq.clear()
            th.outq.clear()
            th.lat_valid = 0
            th.clear_encoder()
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
        if cm.swirq_clr:
            self.swirq &= ~cm.swirq_clr
        if cm.swirq_set:
            self.swirq |= cm.swirq_set
        self.swirq &= 0xF
        if cm.irq_en is not None:
            self.irq_en = cm.irq_en & WORD_MASK
        if cm.irq_en2 is not None:
            self.irq_en2 = cm.irq_en2 & H.IRQ_EN2_MASK

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
        if cm.steps is not None:
            th.steps = cm.steps & WORD_MASK
        if cm.steps_inc:
            th.steps = (th.steps + 1) & WORD_MASK
        if cm.sr is not None:
            th.sr = cm.sr & WORD_MASK
        if cm.cnt is not None:
            th.cnt = cm.cnt & CNT_MASK
        if cm.crc is not None:
            th.crc = cm.crc & WORD_MASK
        if cm.be_cfg is not None:
            th.be_cfg = be_cfg_stored(cm.be_cfg, self._be_enc)
            # SEMANTICS 6.9.1: every write to BE_CFG clears the encoder state.
            th.clear_encoder()
        if cm.enc_lvl is not None:
            th.enc_lvl = cm.enc_lvl & 1
        if cm.enc_run is not None:
            th.enc_run = cm.enc_run & 7
        if cm.enc_rval is not None:
            th.enc_rval = cm.enc_rval & 1
        if cm.enc_pend is not None:
            th.enc_pend = cm.enc_pend & 1
        if cm.enc_half is not None:
            th.enc_half = cm.enc_half & 1
        if cm.enc_first is not None:
            th.enc_first = cm.enc_first & 1
        if cm.be_pins is not None:
            th.be_pins = cm.be_pins & BE_PINS_MASK
        if cm.be_reload is not None:
            th.be_reload = cm.be_reload & BE_RELOAD_MASK
        if cm.crc_poly is not None:
            th.crc_poly = cm.crc_poly & WORD_MASK
        if cm.crc_init is not None:
            th.crc_init = cm.crc_init & WORD_MASK
        if cm.lat_set is not None:
            th.lat_valid, th.lat_pin, th.lat_val = (
                cm.lat_set[0] & 1, cm.lat_set[1] & 0x1F, cm.lat_set[2] & 1)
        # FIFOs (SEMANTICS 6.7).  Host pushes see the count from before the
        # edge because host commits are applied before the slot's; a thread
        # PUSH/POP decided in X can never be invalidated before its commit.
        if cm.inq_push is not None:
            if len(th.inq) < self.fifo_depth:
                th.inq.append(cm.inq_push)
            else:
                self.badop |= H.BADOP_FIFO
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
            return self._be
        if instr.cls == "fifo":
            return self._fifo
        if instr.name == "WAITB":
            # WAITB is built with the FIFOs; condition 0 (bit engine idle)
            # additionally needs the bit engine (SEMANTICS 6.4).
            if not self._fifo:
                return False
            if ops.get("cond") == 0:
                return self._be
            return True
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
        cm.seen_tick = th.tick_seen & 1
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
            if ops.get("lat", 0) and self._setpd:
                # SEMANTICS 6.10: stage the write instead of doing it.
                cm.lat_set = (1, ops["pin"] & 0x1F, ops["val"] & 1)
            else:
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
            cm.td_written = first
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
            cm.td_written = True
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

        # ---------------------------------------------------------- bit engine
        elif name == "SHO":
            z = self._bit_engine_out(cm, th)             # C and T unchanged
        elif name == "SHI":
            z, t_flag = self._bit_engine_in(cm, th, t_flag)      # C unchanged
        elif name == "LDSR":
            cm.sr = regs[ops["ra"]]
        elif name == "STSR":
            write(ops["rd"], th.sr)
        elif name == "CRCI":
            cm.crc = th.crc_init
        elif name == "STCRC":
            write(ops["rd"], th.crc)

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

    # ------------------------------------------ the bit engine (6.9, 6.9.1)
    def _be_config(self, th: ThreadState) -> Tuple[int, int, int, int, int, int]:
        """``BE_CFG`` as ``(DIR, INV, CRC_EN, ENC, STUFF, DIFF)``.

        The fields slice A adds read 0 in a build without it, so one code path
        serves both: ``ENC`` and ``STUFF`` are then 0 (NRZ, no stuffing) and
        ``DIFF`` is 0, which is exactly 6.9.
        """
        cfg = be_cfg_stored(th.be_cfg, self._be_enc)
        return (1 if cfg & BE_CFG_DIR else 0,
                1 if cfg & BE_CFG_INV else 0,
                1 if cfg & BE_CFG_CRC_EN else 0,
                (cfg & BE_CFG_ENC) >> BE_CFG_ENC_SHIFT,
                (cfg & BE_CFG_STUFF) >> BE_CFG_STUFF_SHIFT,
                1 if cfg & BE_CFG_DIFF else 0)

    def _bit_engine_out(self, cm: Commit, th: ThreadState) -> int:
        """``SHO``: the four ordered steps of SEMANTICS 6.9.1.  Returns ``Z``.

        Step 1 picks the bit sent (none for the second half of a Manchester
        bit, the stuff bit when one is due, else the data bit of 6.9), step 2
        does the run accounting, step 3 encodes it into a line level and step
        4 writes the pin (and, with ``DIFF``, the complement on the next
        index at the same edge).
        """
        msb_first, inv, crc_en, enc, stuff, diff = self._be_config(th)
        cnt = th.cnt

        # 1. The bit sent, ``x``.
        if enc == ENC_MANCHESTER and th.enc_half:
            bit = None                          # the second half of ``FIRST``
        elif stuff != STUFF_NONE and th.enc_pend:
            bit = alu.be_stuff_value(stuff, th.enc_rval)
            cm.enc_pend = 0                     # SR, CNT and the CRC stand still
        else:
            bit = alu.be_out_bit(th.sr, bool(msb_first))
            cm.sr = alu.be_shift_out(th.sr, bool(msb_first))
            cm.cnt = cnt = alu.be_count(th.cnt)
            if crc_en:
                cm.crc = alu.crc_step(th.crc, bit, th.crc_poly)

        # 2. Run accounting on ``x``, if there was one.
        if bit is not None and stuff != STUFF_NONE:
            run, rval, due = alu.be_run_step(th.enc_run, th.enc_rval, bit, stuff)
            cm.enc_run, cm.enc_rval = run, rval
            if due:
                cm.enc_pend = 1

        # 3. Encoding into the level ``l``.
        if enc == ENC_NRZI:
            level = th.enc_lvl if bit else 1 - th.enc_lvl    # a 0 toggles
            cm.enc_lvl = level
        elif enc == ENC_MANCHESTER:
            if th.enc_half:
                level = th.enc_first & 1
                cm.enc_half = 0
            else:
                cm.enc_first = bit
                cm.enc_half = 1
                level = 1 - bit                 # 802.3: a 0 is high then low
        else:
            level = bit

        # 4. The pin write of ``l ^ INV`` (6.3 rules, open drain included).
        out = th.be_pins & 0x1F
        cm.write_pin(out, level ^ inv, self.od_mask)
        if diff:
            cm.write_pin((out + 1) % 32, 1 - (level ^ inv), self.od_mask)
        return 1 if cnt == 0 else 0

    def _bit_engine_in(self, cm: Commit, th: ThreadState,
                       t_flag: int) -> Tuple[int, int]:
        """``SHI``: the four ordered steps of 6.9.1.  Returns ``(Z, T)``.

        Step 1 samples the pin, step 2 decodes it into the bit received (none
        for the first half of a Manchester bit, whose second half also sets
        ``T`` when the line did not change in the middle), step 3 drops a
        stuff bit or shifts a data bit into ``SR``, and step 4 does the run
        accounting.  ``T`` is set by a violation and never cleared here.
        """
        msb_first, inv, crc_en, enc, stuff, _diff = self._be_config(th)
        cnt = th.cnt

        # 1. The sample.
        p = self.pin_in((th.be_pins >> 5) & 0x1F) ^ inv

        # 2. Decoding into the bit received, ``s``.
        if enc == ENC_NRZI:
            s = 1 if p == th.enc_lvl else 0
            cm.enc_lvl = p
        elif enc == ENC_MANCHESTER:
            if th.enc_half:
                s = p
                cm.enc_half = 0
                if (th.enc_first & 1) == p:
                    t_flag = 1                  # no transition in mid-bit
            else:
                cm.enc_first = p
                cm.enc_half = 1
                s = None                        # steps 3 and 4 are skipped
        else:
            s = p

        if s is not None:
            # 3. A stuff bit is dropped; a data bit goes into ``SR``.
            if stuff != STUFF_NONE and th.enc_pend:
                cm.enc_pend = 0
                if s != alu.be_stuff_value(stuff, th.enc_rval):
                    t_flag = 1                  # a stuffing violation
            else:
                cm.sr = alu.be_shift_in(th.sr, s, bool(msb_first))
                cm.cnt = cnt = alu.be_count(th.cnt)
                if crc_en:
                    cm.crc = alu.crc_step(th.crc, s, th.crc_poly)

            # 4. Run accounting on ``s``.
            if stuff != STUFF_NONE:
                run, rval, due = alu.be_run_step(th.enc_run, th.enc_rval, s, stuff)
                cm.enc_run, cm.enc_rval = run, rval
                if due:
                    cm.enc_pend = 1
        return (1 if cnt == 0 else 0), t_flag

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
            # Condition 0, bit engine idle: always true until auto mode (M3).
            # Only reached when the bit engine is built (see _feature_built).
            return True
        raise LoomsimError("no condition for " + name)

    def _thread_csr_value(self, th: ThreadState, number: int) -> int:
        """Per-thread CSR 0x00..0x0F as visible now (0 if its feature is unbuilt)."""
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
        if number in BE_CSRS and self._be:
            if number == CSR_BE_CFG:
                return be_cfg_stored(th.be_cfg, self._be_enc)
            if number == CSR_BE_PINS:
                return th.be_pins & BE_PINS_MASK
            if number == CSR_BE_RELOAD:
                return th.be_reload & BE_RELOAD_MASK
            if number == CSR_CRC_POLY:
                return th.crc_poly & WORD_MASK
            if number == CSR_CRC_INIT:
                return th.crc_init & WORD_MASK
            if number == CSR_SR:
                return th.sr & WORD_MASK
            if number == CSR_CNT:
                return th.cnt & CNT_MASK
            if number == CSR_CRC:
                return th.crc & WORD_MASK
        return 0

    def _csr_read(self, th: ThreadState, number: int, x: int) -> int:
        """``CSRR``: the CSR value visible in X, zero-extended (0 if unbuilt)."""
        if number < 0x10:
            return self._thread_csr_value(th, number)
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
            cm.td_written = True
        elif number == CSR_FLAGS:
            z, c_flag, t_flag = value & 1, (value >> 1) & 1, (value >> 2) & 1
        elif number in BE_CSRS:
            if self._be:
                for field, field_value in self._be_csr_fields(number, value).items():
                    setattr(cm, field, field_value)
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
        """The named debug view of one thread (:data:`DEBUG_REGS` plus run state).

        Unchanged since M1 so that co-simulation comparisons keep their keys;
        :meth:`dump_debug_space` has the HOST_PROTOCOL DEBUG space by number.
        """
        out = {name: self.host_read_debug(thread, name) for name in DEBUG_REGS}
        out["RUN"] = (self.run >> thread) & 1
        out["HALTED"] = (self.halted >> thread) & 1
        out["BADOP"] = (self.badop >> thread) & 1
        return out

    def dump_debug_space(self, thread: int) -> Dict[int, int]:
        """Every DEBUG-space register of ``thread``, as the port reads it.

        0x00..0x26 always, and 0x27 (the encoder state, SEMANTICS 6.9.1) in a
        build that has the slice-A bit engine, where it stops reading 0.
        """
        self._check_thread(thread)
        last = H.DEBUG_ENC if self._be_enc else H.DEBUG_LAST
        return {number: self._debug_read(thread, number)
                for number in range(last + 1)}

    def __repr__(self) -> str:
        return ("<Machine cycle=%d run=%X halted=%X pc=%s>"
                % (self.cycle, self.run, self.halted,
                   [th.pc for th in self.threads]))
