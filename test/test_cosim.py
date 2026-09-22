# SPDX-License-Identifier: Apache-2.0
"""L2-RAND, L2-TRACE, L2-SLOT, L2-COV: the RTL and the golden model in lockstep.

``tools/loomgen`` generates a constrained-random program and a pad stimulus
plan; this module runs them on ``src/`` (Icarus) and on ``tools/loomsim`` one
clock at a time and compares, on **every cycle**:

* the retire record of ``docs/SEMANTICS.md`` section 8 (``tr_*``, reached
  hierarchically in ``loom_top``) against the model's ``RetireRecord`` for the
  slot in W that cycle (``rd``/``val`` only when ``we`` is set);
* the pad outputs ``uo_out[6:0]`` (``uo_out[6]`` is ``HOST_IRQ``, SEMANTICS
  6.8), ``uio_out`` and ``uio_oe``; only ``uo_out[7]``, the host MISO pin the
  model has no port for, is masked;
* as guards, ``ph`` (and that it equals ``k mod 4``), ``RUN``, ``HALTED``,
  ``BADOP``, ``SFLAGS`` and ``OD_MASK``;
* L2-SLOT: a thread's ``PC`` register changes only at the edge that ends a W
  cycle of that thread, that W cycle has ``ph == (t + 3) mod 4``, and the new
  value is that slot's ``tr_next_pc``; a slot that does not complete is a
  wait-class or blocking (``PUSH``/``POP``) instruction and re-issues its own
  ``PC``.

Every ``LOOM_COSIM_STATE_EVERY`` cycles and at the end of every seed the whole
architectural state of SEMANTICS 5 is read out of the RTL hierarchy and
compared with the model (registers, PC, flags, TD, DT, NOW, ACC, TICK_SEEN,
RS0/RS1/DEPTH, WAIT_ACTIVE, PREV_PINS, STEPS, TICK_INT/FRAC, OUTGRP/INGRP,
RUN/HALTED/STEP_REQ/BADOP/SFLAGS/SWIRQ, PIN_OUT/PIN_OE/OD_MASK, and for the
features the build has: ``INQ_CNT``/``OUTQ_CNT``, the bit-engine state
``SR``/``CNT``/``CRC`` and its CSRs, the deadline latch, ``IRQ_EN``/``IRQ_EN2``
and the registered ``HOST_IRQ``).

Alignment
---------

SEMANTICS 1: "**Edge 0** is the last rising edge at which ``rst_n`` is sampled
low. **Cycle k** is the interval between edge k and edge k+1", ``ph(k) = k mod
4``. A fresh ``Machine`` sits at the start of cycle 0 and ``step_cycle()``
simulates one cycle and the edge that ends it; host actions called during
cycle ``k`` commit at edge ``k + 1``; ``set_pad_inputs`` takes what the pads
hold at the coming edge. The harness therefore does, per cycle ``k``::

    await FallingEdge(clk)          # mid cycle k: every RTL register is settled
    sample RTL                      # tr_*, pads, ph, RUN, ... = cycle-k values
    host actions due at edge k+1    # model: host_*();  RTL: remembered
    drive pads for edge k+1         # dut.ui_in / dut.uio_drv, set_pad_inputs
    record = machine.step_cycle()   # the slot in W during cycle k
    compare
    if RTL deposits are due:        # backdoor host actions
        await RisingEdge(clk)       # edge k+1 ...
        deposit                     # ... applied in its ReadWrite phase

Why this is exact:

* ``rst_n`` is released by a write issued right after the last reset edge.
  cocotb 2 applies writes in the ReadWrite phase, after the edge's processes
  ran, so that edge sampled ``rst_n`` low: it is edge 0, and the next falling
  edge is in cycle 0 (``ph`` reads 0 there, which the guard checks).
* Pad values written at the falling edge of cycle ``k`` are what the pads hold
  at edge ``k + 1``; the model is told the same values before it steps cycle
  ``k``.
* A deposit written right after ``RisingEdge`` lands in the ReadWrite phase of
  edge ``k + 1``, after every always block ran for that edge, which is
  indistinguishable from the register having been loaded at that edge. The
  model's host call in cycle ``k`` commits at the same edge. So ``RUN``
  becomes visible to both in cycle ``k + 1``, and the ``RUN``/``ph`` guards
  confirm it on every cycle.

Starting the threads
--------------------

**Backdoor (default).** Host actions follow ``tools.loomgen.runner``: in cycle
``LOAD_CYCLE`` (0) the model gets ``host_write_imem`` for every word
(``IMEM_WORDS``, 512 with the SRAM macro) and the RTL instruction array is
deposited after edge 1: the ``memory`` array of the macro's behavioural model
(``u_imem.g_macro.u_macro.sram.i_SRAM_1P_behavioral_bm_bist``), or ``mem`` of
the flop array in a FLOPS build (``test_flops.py``); in cycle ``RUN_CYCLE - 1``
the model gets ``host_set_run`` and ``u_core.run_r`` is deposited after edge
``RUN_CYCLE`` (4), so both see ``RUN`` from cycle 4 and thread 0 has the first
slot.

**Over the pins (``test_cosim_over_the_host_port``).** The image and the
``CTRL.RUN`` write go through the real SPI pads with ``test/spi_host.py``. The
commit cycle is observed, not chosen, as SEMANTICS 10 allows ("the host actions
with the cycle at which each commits (observed from the RTL in co-simulation)"):
every host-control pulse of ``loom_host_ctl`` (IMEM write, RUN write, FIFO
push and pop, the CTRL registers the run touches) is one clock wide in the
cycle before its commit edge, so seeing it at the falling edge of cycle ``k``
and making the same ``host_*`` call before the model steps cycle ``k`` commits
both at edge ``k + 1``. The lockstep comparison runs through the whole load as
well.

The build
---------

At the start of every seed the harness resets the RTL, reads ``CTRL.CAPS``
through the SPI host port and **builds the golden model from it**
(:class:`_Build`): ``[15:12]`` the instruction memory size, ``[3]`` the FIFOs
with depth ``2 ** CAPS[2:0]``, ``[4]`` the bit engine, ``[7]`` the
deadline-latched ``SETP``. The generated program is made for the same build,
so both sides execute the M2 instructions, or treat them as ``NOP`` + ``BADOP``
(SEMANTICS 9), together. A ``CAPS`` bit the model cannot build (data memory,
boot ROM, bit-engine auto mode, a reserved bit) fails the test rather than
skipping anything.

Host traffic (``test_cosim_over_the_host_port``)
------------------------------------------------

A program generated with ``host_traffic=True`` carries a
``tools.loomgen.hostplan.HostPlan``: FIFO pushes into ``INQ[t]``, FIFO reads
that pop ``OUTQ[t]``, and CTRL writes (``IRQ_EN``, ``IRQ_EN2``, ``SWIRQ``,
``BADOP``, ``SFLAGS``, ``SFLAGS_CLR``, ``RUN``). A cocotb task sends them over
the SPI pads, one after another, for as long as the run lasts, and the
lockstep loop mirrors each one into the model at the cycle the RTL commits it:

* the byte layer's ``byte_done`` pulse says which byte of the transaction has
  just finished, so the harness knows when a word's effect is due (the next
  cycle: HOST_PROTOCOL's "registered at edge E+3, target register at E+4")
  and when a FIFO read **peeks** ``OUTQ`` (at the end of the dummy byte, or of
  the previous word, taking the entry after the head while that one is being
  popped, SEMANTICS 6.7);
* the model is then given that host action with the harness's own data, and
  the ``loom_host_ctl`` pulse and data bus for the same cycle are **compared**
  against what was expected, so a word that is dropped, doubled, mistimed or
  corrupted on the way through the SPI port is a divergence like any other;
* what the read returned on MISO is compared with the model's peek.

The model has no public call for "a FIFO read whose peek was empty while a
thread push has filled the queue since", which sets ``BADOP[14]`` and pops
nothing; ``tools.loomgen.runner.fifo_error`` does it through the model's own
host-commit path.

Environment
-----------

``LOOM_COSIM_SEEDS`` (default 12) and ``LOOM_COSIM_CYCLES`` (default 4000, per
seed, counted from ``RUN``) size the random run; ``LOOM_COSIM_SEED_BASE``
(default 1) is the first seed; ``LOOM_COSIM_SPI_SEEDS`` (default 2) and
``LOOM_COSIM_SPI_CYCLES`` (default 8000) size the host-path variant;
``LOOM_COSIM_STATE_EVERY`` (default 500) sets how often the full state is
compared; ``LOOM_COSIM_KEEP_GOING=1`` records a failure and moves on to the
next seed instead of stopping; ``LOOM_COSIM_REPLAY=path.json`` runs one saved
program (a file from ``test/cosim_failures/`` or ``python -m tools.loomgen
-o``) instead of the random set.

A divergence fails with the seed, cycle, thread, PC, the disassembled
instruction, both records and the thread's last 12 retire records, and writes
``test/cosim_failures/seed_<n>.json`` (image, stimulus, host log, failure)
plus a ``.lst`` listing; ``python -m tools.loomgen --replay FILE --run N
--trace T`` shows the model's side of it, replaying the recorded host actions
at the cycles this harness observed them. The module skips itself on a
gate-level netlist (``GATES=yes``), which has none of the signals it reads. In
an RTL run a signal it cannot find fails the tests instead of skipping them
(the instruction array moved when the SRAM macro came in, D-020, and a silent
skip would have looked like a pass).
"""

from __future__ import annotations

import collections
import inspect
import json
import os
import pathlib
import sys
import time

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, Timer

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from cosim_coverage_m2 import M2Coverage, SlotContext                 # noqa: E402
from spi_host import (CLK_NS, CTRL_BADOP, CTRL_CAPS, CTRL_IRQ_EN,     # noqa: E402
                      CTRL_IRQ_EN2, CTRL_RUN, CTRL_SFLAGS,
                      CTRL_SFLAGS_CLR, CTRL_SWIRQ, FIFO_QUEUE, SP_CTRL,
                      SP_FIFO, LoomHost)
from tools.loomasm.disasm import disassemble                          # noqa: E402
from tools.loomgen import (GeneratedProgram, LOAD_CYCLE, RUN_CYCLE,   # noqa: E402
                           generate, host_actions, peek_word,
                           pop_after_peek)
from tools.loomisa import load as load_isa                            # noqa: E402
from tools.loomsim import Machine                                     # noqa: E402

THREADS = 4
#: Instruction memory size of the default build: the 512 x 16 SRAM macro
#: (D-020). The harness takes the real size from CAPS[15:12] and checks it
#: against the RTL array.
IMEM_WORDS = 512


def _env_int(name: str, default: int) -> int:
    return int(os.environ.get(name, str(default)) or default)


SEEDS = _env_int("LOOM_COSIM_SEEDS", 12)
CYCLES = _env_int("LOOM_COSIM_CYCLES", 4000)
SEED_BASE = _env_int("LOOM_COSIM_SEED_BASE", 1)
SPI_SEEDS = _env_int("LOOM_COSIM_SPI_SEEDS", 2)
SPI_CYCLES = _env_int("LOOM_COSIM_SPI_CYCLES", 8000)
STATE_EVERY = _env_int("LOOM_COSIM_STATE_EVERY", 500)
KEEP_GOING = os.environ.get("LOOM_COSIM_KEEP_GOING", "") not in ("", "0")
REPLAY = os.environ.get("LOOM_COSIM_REPLAY", "")

#: The profiles the seeds rotate through. ``m2`` comes first and the seeds
#: with fewer than four running threads are the last of every six, so the
#: early seeds are the densest: a broken M2 feature shows up in seed 1 or 2
#: rather than five seeds later (the mutants of docs/VERIFICATION.md L7).
PROFILE_ORDER = ("m2", "timing", "pins", "alu", "mixed")

#: Constructs ``docs/spec-questions/cosim.md`` has open. Neither side may be
#: changed for them until the director rules, so the random programs leave
#: them out and the rest of the run stays useful.
AVOID = ("csrw_pin_out_high_bits",)

HERE = pathlib.Path(__file__).resolve().parent
FAILURE_DIR = HERE / "cosim_failures"
COVERAGE_PATH = HERE / "cosim_coverage.json"

ISA = load_isa()
COVERAGE = M2Coverage(ISA)

#: SPI host pins in ui_in, idle: CS_n high, SCK low, MOSI low.
UI_CS = 1 << 4
UI_HOST_MASK = 0x70

#: CTRL register address of every write a host plan may make, and the pulse
#: (write strobe, data bus) ``loom_host_ctl`` raises for it in the cycle
#: before its commit edge. ``IRQ_EN``/``IRQ_EN2`` live inside the module, so
#: their strobes are read from it by name.
CTRL_WRITE_ADDR = {"IRQ_EN": CTRL_IRQ_EN, "IRQ_EN2": CTRL_IRQ_EN2,
                   "SWIRQ": CTRL_SWIRQ, "BADOP": CTRL_BADOP,
                   "SFLAGS": CTRL_SFLAGS, "SFLAGS_CLR": CTRL_SFLAGS_CLR,
                   "RUN": CTRL_RUN}
CTRL_WRITE_PULSE = {"SWIRQ": ("h_swirq_clr_we", "h_swirq_clr", 0xF),
                    "BADOP": ("h_badop_clr_we", "h_badop_clr", 0xFFFF),
                    "SFLAGS": ("h_sfset_we", "h_sfset", 0xFF),
                    "SFLAGS_CLR": ("h_sfclr_we", "h_sfclr", 0xFF),
                    "RUN": ("h_run_we", "h_run", 0xF)}
#: Coverage names for the CTRL writes of a host plan.
CTRL_WRITE_EVENT = {"IRQ_EN": "IRQ_EN:written", "IRQ_EN2": "IRQ_EN2:written",
                    "SWIRQ": "SWIRQ:cleared", "BADOP": "BADOP:cleared",
                    "SFLAGS": "SFLAGS:set", "SFLAGS_CLR": "SFLAGS:cleared",
                    "RUN": "RUN:rewritten"}


# ------------------------------------------------------------ gate level
def imem_array(loom):
    """The instruction-memory array of whichever ``loom_imem`` backend the
    ``loom_top`` instance ``loom`` was built with: ``memory`` in the SRAM
    macro's behavioural model (IMEM_IMPL "MACRO", D-020) or the flop array
    ``mem`` ("FLOPS"). Raises AttributeError if neither exists."""
    imem = loom.u_imem
    try:
        return imem.g_macro.u_macro.sram.i_SRAM_1P_behavioral_bm_bist.memory
    except AttributeError:
        return imem.g_flops.mem


def _rtl_hierarchy_present(dut) -> bool:
    """True when the RTL hierarchy this module reads exists under ``dut``."""
    try:
        dut.user_project.u_loom.u_core.ph                      # noqa: B018
        dut.user_project.u_loom.byte_done                      # noqa: B018
        imem_array(dut.user_project.u_loom)
    except AttributeError:
        return False
    return True


def _require_rtl(dut):
    """Fail, rather than skip, an RTL run that lacks the signals read here."""
    assert _rtl_hierarchy_present(dut), (
        "RTL hierarchy not found: user_project.u_loom.u_core.ph, "
        "u_loom.byte_done and the instruction array of loom_imem (macro model "
        "memory or g_flops.mem) are needed; gate-level runs must set GATES=yes")


#: cocotb 2 has no run-time skip, so the flag is decided at import time.
#: The gate-level flows (test/Makefile, Tiny Tapeout's gl_test) set GATES=yes.
GATE_LEVEL = os.environ.get("GATES", "").lower() == "yes"


# ------------------------------------------------------------------- build
class BuildError(AssertionError):
    """``CTRL.CAPS`` reports something the golden model cannot be built for."""


class _Build:
    """The build ``CTRL.CAPS`` reports (SEMANTICS 5), for both sides.

    ``[2:0]`` log2 of the FIFO depth, ``[3]`` FIFOs, ``[4]`` bit engine
    (manual mode), ``[5]`` data memory (model feature ``"DMEM"``), ``[6]`` boot ROM, ``[7]``
    deadline-latched ``SETP``, ``[8]`` bit-engine auto mode, ``[9]`` the
    slice-A encoders, stuffing and DIFF (model feature ``"BEENC"``),
    ``[11:10]`` zero, ``[15:12]`` log2 of ``IMEM_WORDS``.
    """

    def __init__(self, caps: int):
        self.caps = caps & 0xFFFF
        self.imem_words = 1 << ((caps >> 12) & 0xF)
        features = []
        if caps & (1 << 3):
            features.append("FIFO")
            self.fifo_depth = 1 << (caps & 0x7)
            if self.fifo_depth not in (2, 4, 8):
                raise BuildError("CAPS %04X asks for FIFOs of depth %d; "
                                 "SEMANTICS 6.7 allows 2, 4 or 8"
                                 % (self.caps, self.fifo_depth))
        else:
            self.fifo_depth = 4
            if caps & 0x7:
                raise BuildError("CAPS %04X has no FIFOs but CAPS[2:0] = %d"
                                 % (self.caps, caps & 0x7))
        if caps & (1 << 4):
            features.append("BE")
        if caps & (1 << 7):
            features.append("SETPD")
        if caps & (1 << 9):
            if not caps & (1 << 4):
                raise BuildError("CAPS %04X reports the slice-A encoders (bit 9) "
                                 "without the bit engine (bit 4)" % self.caps)
            features.append("BEENC")
        if caps & (1 << 5):
            features.append("DMEM")      # slice B: LD/ST on the instruction memory (6.11)
        for bit, what in ((6, "a boot ROM"), (8, "bit-engine auto mode")):
            if caps & (1 << bit):
                raise BuildError(
                    "CAPS %04X reports %s (bit %d); the golden model cannot be "
                    "built for it, so this run would compare nothing. Teach "
                    "tools/loomsim the feature (or take the bit out of the RTL)."
                    % (self.caps, what, bit))
        if caps & 0x0C00:
            raise BuildError("CAPS %04X sets a reserved bit (11:10)" % self.caps)
        self.features = tuple(sorted(features))
        #: Width of one ``INQ_CNT``/``OUTQ_CNT`` field in the RTL vectors.
        self.fifo_width = self.fifo_depth.bit_length()

    def machine(self) -> Machine:
        machine = Machine(imem_words=self.imem_words, features=self.features,
                          fifo_depth=self.fifo_depth, loopback=True, isa=ISA)
        if machine.caps != self.caps:
            raise BuildError("the model built from CAPS %04X reports %04X"
                             % (self.caps, machine.caps))
        return machine

    def __str__(self) -> str:
        return "caps=%04X %d words, %s" % (
            self.caps, self.imem_words,
            ", ".join(self.features) + (" depth %d" % self.fifo_depth
                                        if "FIFO" in self.features else "")
            if self.features else "M1")


# ------------------------------------------------------------------ probe
def _optional(obj, name):
    """A hierarchical signal that only some builds have."""
    try:
        return getattr(obj, name)
    except AttributeError:
        return None


class _Probe:
    """Cached handles for everything the harness reads or deposits."""

    def __init__(self, dut):
        self.dut = dut
        loom = dut.user_project.u_loom
        core = loom.u_core
        self.loom, self.core = loom, core
        self.mem = imem_array(loom)
        self.mem_words = len(self.mem)
        self.regs = core.u_rf.regs
        self.acc = [core.u_timer.g_thread[t].acc for t in range(THREADS)]
        for name in ("ph", "run_r", "halted_r", "step_req_r", "badop_r",
                     "sflags_r", "swirq_r", "pc_all", "z_all", "c_all", "t_all",
                     "wa_all", "rs0_all", "rs1_all", "depth_all", "pp_all",
                     "outgrp_all", "ingrp_all", "steps_all", "now_all",
                     "td_all_w", "dt_all_w", "tick_int_all", "tick_frac_all",
                     "tick_seen_all", "resetpc_r"):
            setattr(self, name, getattr(core, name))
        for name in ("tr_valid", "tr_thread", "tr_pc", "tr_ir", "tr_done",
                     "tr_we", "tr_rd", "tr_val", "tr_flags", "tr_next_pc"):
            setattr(self, name, getattr(loom, name))
        self.pin_out = loom.u_pins.pin_out
        self.pin_oe = loom.u_pins.pin_oe
        self.od_mask = loom.u_pins.od_mask
        self.uo_out, self.uio_out, self.uio_oe = dut.uo_out, dut.uio_out, dut.uio_oe
        # SPI byte layer, for the host-traffic mirror.
        self.byte_done, self.cs_active = loom.byte_done, loom.cs_active
        # Host-control pulses (loom_host_ctl outputs), for the SPI variant.
        self.host = {name: getattr(loom, name) for name in (
            "h_imem_req", "h_imem_we", "h_imem_addr", "h_imem_wdata",
            "h_run_we", "h_run", "h_reset", "h_step_we", "h_step",
            "h_rpc_we", "h_rpc_sel", "h_rpc", "h_sfset_we", "h_sfset",
            "h_sfclr_we", "h_sfclr", "h_badop_clr_we", "h_badop_clr",
            "h_badop_set15", "h_swirq_clr_we", "h_swirq_clr", "h_pout_we", "h_pout",
            "h_poe_we", "h_poe", "h_od_we", "h_od", "h_dbg_req",
            "h_inq_push", "h_fifo_wdata", "h_outq_pop", "h_badop_set14")}
        # The two CTRL registers loom_host_ctl keeps itself (IRQ_EN, IRQ_EN2)
        # pulse an internal write strobe in the cycle before their edge.
        self.irq_en_wr = _optional(loom.u_host, "irq_en_wr")
        self.irq_en2_wr = _optional(loom.u_host, "irq_en2_wr")
        self.irq_en = _optional(loom.u_host, "irq_en")
        self.irq_en2 = _optional(loom.u_host, "irq_en2")
        # M2 architectural state, per feature (SEMANTICS 5).
        self.inq_cnt_all = _optional(core, "inq_cnt_all")
        self.outq_cnt_all = _optional(core, "outq_cnt_all")
        self.be = {name: _optional(core, name) for name in (
            "be_sr_all", "be_cnt_all", "be_crc_all", "be_poly_all",
            "be_init_all", "be_reload_all", "be_cfg_all", "be_pins_all")}
        self.be_enc_all = _optional(core, "be_enc_all")     # slice A (6.9.1)
        self.lat_valid_all = _optional(core, "lat_valid_all")
        self.lat_pin_all = _optional(core, "lat_pin_all")
        self.lat_val_all = _optional(core, "lat_val_all")


def _i(handle) -> int:
    """``int(handle.value)``, naming the signal when it holds X or Z."""
    try:
        return int(handle.value)
    except ValueError:
        raise AssertionError("signal %s is not resolvable: %s"
                             % (handle._path, handle.value)) from None


def _field(word: int, index: int, width: int) -> int:
    return (word >> (index * width)) & ((1 << width) - 1)


def _width(handle) -> int:
    """The width in bits of a vector handle (cocotb 2.x)."""
    try:
        return len(handle)
    except TypeError:
        return len(handle.value)


# --------------------------------------------------------------- records
FIELDS = ("thread", "pc", "ir", "done", "we", "rd", "val", "flags", "next_pc")


def _loose(handle) -> int:
    """``int(handle.value)``, or -1 when it holds X or Z.

    Only for the ``tr_rd``/``tr_val`` of a slot that writes no register:
    ``POP`` reads the FIFO entry, and SEMANTICS 5 says FIFO entries are not
    reset, so the array is X in simulation until something is pushed. The
    comparison ignores both fields unless the slot writes (``_diff_record``),
    and a ``tr_val`` that is X while ``tr_we`` is 1 still fails.
    """
    try:
        return int(handle.value)
    except ValueError:
        return -1


def _rtl_record(p: _Probe):
    we = _i(p.tr_we)
    return (_i(p.tr_thread), _i(p.tr_pc), _i(p.tr_ir), _i(p.tr_done),
            we, _i(p.tr_rd) if we else _loose(p.tr_rd),
            _i(p.tr_val) if we else _loose(p.tr_val), _i(p.tr_flags),
            _i(p.tr_next_pc))


def _model_tuple(record):
    return (record.thread, record.pc, record.ir, int(record.done),
            int(record.we), record.rd, record.val, record.flags,
            record.next_pc)


def _diff_record(rtl, model):
    """SEMANTICS 8 fields that differ. ``rd``/``val`` count only on a write."""
    if (rtl is None) != (model is None):
        return [("tr_valid", int(rtl is not None), int(model is not None))]
    if model is None:
        return []
    mine = _model_tuple(model)
    out = []
    for name, got, want in zip(FIELDS, rtl, mine):
        if name in ("rd", "val") and not (rtl[4] and mine[4]):
            continue
        if got != want:
            out.append(("tr_" + name, got, want))
    return out


def _fmt(rec) -> str:
    if rec is None:
        return "no valid slot"
    thread, pc, ir, done, we, rd, val, flags, next_pc = rec
    write = "r%d=%04X" % (rd, val) if we else "-"
    return ("t%d pc=%03X ir=%04X %-5s %-9s TCZ=%d%d%d next=%03X  %s"
            % (thread, pc, ir, "done" if done else "stall", write,
               (flags >> 2) & 1, (flags >> 1) & 1, flags & 1, next_pc,
               disassemble(ir, ISA)))


# ------------------------------------------------------ architectural state
def _state_pairs(p: _Probe, machine: Machine, build: _Build):
    """Every architectural value of SEMANTICS 5, as (name, RTL, model)."""
    pairs = []
    add = pairs.append
    pc_all, rs0_all, rs1_all = _i(p.pc_all), _i(p.rs0_all), _i(p.rs1_all)
    z_all, c_all, t_all, wa_all = _i(p.z_all), _i(p.c_all), _i(p.t_all), _i(p.wa_all)
    depth_all, pp_all, steps_all = _i(p.depth_all), _i(p.pp_all), _i(p.steps_all)
    outgrp_all, ingrp_all = _i(p.outgrp_all), _i(p.ingrp_all)
    now_all, td_all, dt_all = _i(p.now_all), _i(p.td_all_w), _i(p.dt_all_w)
    tint_all, tfrac_all = _i(p.tick_int_all), _i(p.tick_frac_all)
    tseen_all, rpc_all = _i(p.tick_seen_all), _i(p.resetpc_r)
    for t in range(THREADS):
        th = machine.threads[t]
        for r in range(8):
            add(("t%d.r%d" % (t, r), _i(p.regs[t * 8 + r]), th.regs[r]))
        add(("t%d.PC" % t, _field(pc_all, t, 10), th.pc))
        add(("t%d.Z" % t, (z_all >> t) & 1, th.z))
        add(("t%d.C" % t, (c_all >> t) & 1, th.c))
        add(("t%d.T" % t, (t_all >> t) & 1, th.t))
        add(("t%d.WAIT_ACTIVE" % t, (wa_all >> t) & 1, th.wait_active))
        add(("t%d.RS0" % t, _field(rs0_all, t, 10), th.rs0))
        add(("t%d.RS1" % t, _field(rs1_all, t, 10), th.rs1))
        add(("t%d.DEPTH" % t, _field(depth_all, t, 2), th.depth))
        add(("t%d.PREV_PINS" % t, _field(pp_all, t, 13), th.prev_pins))
        add(("t%d.STEPS" % t, _field(steps_all, t, 16), th.steps))
        add(("t%d.OUTGRP" % t, _field(outgrp_all, t, 10), th.outgrp))
        add(("t%d.INGRP" % t, _field(ingrp_all, t, 10), th.ingrp))
        add(("t%d.NOW" % t, _field(now_all, t, 16), th.now))
        add(("t%d.TD" % t, _field(td_all, t, 16), th.td))
        add(("t%d.DT" % t, _field(dt_all, t, 16), th.dt))
        add(("t%d.ACC" % t, _i(p.acc[t]), th.acc))
        add(("t%d.TICK_SEEN" % t, (tseen_all >> t) & 1, th.tick_seen))
        add(("t%d.TICK_INT" % t, _field(tint_all, t, 16), th.tick_int))
        add(("t%d.TICK_FRAC" % t, _field(tfrac_all, t, 8), th.tick_frac))
        add(("t%d.RESET_PC" % t, _field(rpc_all, t, 10), machine.reset_pc[t]))
    add(("RUN", _i(p.run_r), machine.run))
    add(("HALTED", _i(p.halted_r), machine.halted))
    add(("STEP_REQ", _i(p.step_req_r), machine.step_req))
    add(("BADOP", _i(p.badop_r), machine.badop))
    add(("SFLAGS", _i(p.sflags_r), machine.sflags))
    add(("SWIRQ", _i(p.swirq_r), machine.swirq))
    add(("PIN_OUT", _i(p.pin_out), machine.pin_out))
    add(("PIN_OE", _i(p.pin_oe), machine.pin_oe))
    add(("OD_MASK", _i(p.od_mask), machine.od_mask))
    pairs += _m2_state_pairs(p, machine, build)
    return pairs


def _needed(handle, name, feature):
    assert handle is not None, (
        "CAPS says this build has %s, but the RTL signal %s is not there; the "
        "co-simulation cannot compare that state" % (feature, name))
    return handle


def _m2_state_pairs(p: _Probe, machine: Machine, build: _Build):
    """The M2 state of SEMANTICS 5, for the features the build has."""
    pairs = []
    add = pairs.append
    if "FIFO" in build.features:
        inq = _i(_needed(p.inq_cnt_all, "u_core.inq_cnt_all", "FIFOs"))
        outq = _i(_needed(p.outq_cnt_all, "u_core.outq_cnt_all", "FIFOs"))
        width = build.fifo_width
        for t in range(THREADS):
            add(("t%d.INQ_CNT" % t, _field(inq, t, width),
                 len(machine.threads[t].inq)))
            add(("t%d.OUTQ_CNT" % t, _field(outq, t, width),
                 len(machine.threads[t].outq)))
    if "BE" in build.features:
        be = {name: _i(_needed(handle, "u_core." + name, "the bit engine"))
              for name, handle in p.be.items()}
        # BE_CFG is stored packed: {CRC_EN, INV, DIR} at M2 (3 bits per
        # thread) and {DIFF, STUFF[1:0], ENC[1:0], CRC_EN, INV, DIR} with
        # slice A (8 bits, SEMANTICS 6.9.1).
        cfg_width = _width(p.be["be_cfg_all"]) // THREADS
        enc_all = 0
        if "BEENC" in build.features:
            enc_all = _i(_needed(p.be_enc_all, "u_core.be_enc_all",
                                 "the slice-A encoders"))
            assert cfg_width >= 8, (
                "CAPS says slice A is built but be_cfg_all is %d bits per thread"
                % cfg_width)
        for t in range(THREADS):
            th = machine.threads[t]
            add(("t%d.SR" % t, _field(be["be_sr_all"], t, 16), th.sr))
            add(("t%d.CNT" % t, _field(be["be_cnt_all"], t, 5), th.cnt))
            add(("t%d.CRC" % t, _field(be["be_crc_all"], t, 16), th.crc))
            add(("t%d.CRC_POLY" % t, _field(be["be_poly_all"], t, 16), th.crc_poly))
            add(("t%d.CRC_INIT" % t, _field(be["be_init_all"], t, 16), th.crc_init))
            add(("t%d.BE_RELOAD" % t, _field(be["be_reload_all"], t, 5), th.be_reload))
            add(("t%d.BE_PINS" % t, _field(be["be_pins_all"], t, 10), th.be_pins))
            cfg = _field(be["be_cfg_all"], t, cfg_width)
            add(("t%d.BE_CFG.DIR" % t, cfg & 1, (th.be_cfg >> 1) & 1))
            add(("t%d.BE_CFG.INV" % t, (cfg >> 1) & 1, (th.be_cfg >> 7) & 1))
            add(("t%d.BE_CFG.CRC_EN" % t, (cfg >> 2) & 1, (th.be_cfg >> 9) & 1))
            if cfg_width >= 8:
                add(("t%d.BE_CFG.ENC" % t, (cfg >> 3) & 3, (th.be_cfg >> 3) & 3))
                add(("t%d.BE_CFG.STUFF" % t, (cfg >> 5) & 3, (th.be_cfg >> 5) & 3))
                add(("t%d.BE_CFG.DIFF" % t, (cfg >> 7) & 1, (th.be_cfg >> 10) & 1))
            if "BEENC" in build.features:
                add(("t%d.ENC" % t, _field(enc_all, t, 8), th.enc))
    if "SETPD" in build.features:
        valid = _i(_needed(p.lat_valid_all, "u_core.lat_valid_all", "SETP D"))
        value = _i(_needed(p.lat_val_all, "u_core.lat_val_all", "SETP D"))
        pin = _i(_needed(p.lat_pin_all, "u_core.lat_pin_all", "SETP D"))
        for t in range(THREADS):
            th = machine.threads[t]
            add(("t%d.LAT_VALID" % t, (valid >> t) & 1, th.lat_valid))
            add(("t%d.LAT_VAL" % t, (value >> t) & 1, th.lat_val))
            add(("t%d.LAT_PIN" % t, _field(pin, t, 5), th.lat_pin))
    if p.irq_en is not None:
        add(("IRQ_EN", _i(p.irq_en), machine.irq_en))
        add(("IRQ_EN2", _i(p.irq_en2), machine.irq_en2))
    return pairs


# ------------------------------------------------------------- the run
class Divergence(AssertionError):
    """The RTL and the model disagree; the message is the full report."""


class _Run:
    """One program, one reset, one lockstep comparison."""

    def __init__(self, dut, probe: _Probe, prog: GeneratedProgram, label: str,
                 cycles: int, build: _Build, spi: bool = False):
        self.dut = dut
        self.p = probe
        self.prog = prog
        self.label = label
        self.cycles = cycles
        self.spi = spi
        self.build = build
        self.machine = build.machine()
        self.history = [collections.deque(maxlen=12) for _ in range(THREADS)]
        self.host = LoomHost(dut) if spi else None
        self.traffic = prog.host if (spi and prog.host is not None) else None
        self.host_log = []
        self.run_cycle = RUN_CYCLE if not spi else None
        self.retired = 0
        # --- host traffic mirror (SPI variant)
        self.txn = None             # transaction the host task is sending
        self.peeks = []             # model peek per word of a FIFO read
        self.byte_index = 0         # byte counter of the current transaction
        self.expect = []            # [(cycle, action)] the RTL must perform
        self.host_error = None      # set by the host task, checked every cycle
        self.host_touched = []      # (thread, "push"/"read") applied this cycle
        self.txn_count = 0

    # ---------------------------------------------------------------- report
    def fail(self, cycle, diffs, rtl=None, model=None, kind="retire record",
             thread=None, note=""):
        if thread is None:
            thread = model.thread if model is not None else \
                (rtl[0] if rtl is not None else None)
        pc = model.pc if model is not None else (rtl[1] if rtl is not None else None)
        ir = model.ir if model is not None else (rtl[2] if rtl is not None else None)
        lines = ["", "co-simulation divergence (%s): seed %d, %s, cycle %d"
                 % (kind, self.prog.seed, self.label, cycle)]
        if thread is not None:
            lines.append("  thread %d  PC %s  IR %s  %s" % (
                thread, "%03X" % pc if pc is not None else "-",
                "%04X" % ir if ir is not None else "-",
                disassemble(ir, ISA) if ir is not None else ""))
        if note:
            lines.append("  " + note)
        lines.append("")
        lines.append("  %-16s %-20s %-20s" % ("field", "RTL", "model"))
        for name, got, want in diffs:
            lines.append("  %-16s %-20s %-20s" % (
                name, "%d (0x%X)" % (got, got), "%d (0x%X)" % (want, want)))
        if kind == "retire record":
            lines.append("")
            lines.append("  RTL   " + _fmt(rtl))
            lines.append("  model " + (_fmt(_model_tuple(model)) if model else "no valid slot"))
        if thread is not None:
            lines.append("")
            lines.append("  last %d retire records of thread %d (W cycle, model):"
                         % (len(self.history[thread]), thread))
            lines += ["    " + h for h in self.history[thread]]
        info = {"label": self.label, "cycle": cycle, "kind": kind,
                "run_cycle": self.run_cycle, "cycles": self.cycles,
                "spi": self.spi, "thread": thread, "caps": self.build.caps,
                "diffs": [[n, g, w] for n, g, w in diffs],
                "rtl": _fmt(rtl) if rtl is not None else None,
                "model": _fmt(_model_tuple(model)) if model is not None else None,
                "host_log": [list(e) for e in self.host_log],
                "history": list(self.history[thread]) if thread is not None else []}
        path = _dump(self.prog, info)
        raise Divergence("\n".join(lines) + "\n\n  replay: %s\n" % path)

    def check_state(self, cycle):
        bad = [(n, g, w) for n, g, w in _state_pairs(self.p, self.machine, self.build)
               if g != w]
        if bad:
            thread = int(bad[0][0][1]) if bad[0][0].startswith("t") and \
                bad[0][0][1].isdigit() else None
            self.fail(cycle, bad, kind="architectural state", thread=thread,
                      note="full state compare (SEMANTICS 5), RTL hierarchy "
                           "vs model, both as visible in this cycle")

    # ------------------------------------------------------------ host side
    def backdoor_deposits(self, cycle):
        """Model host calls for ``cycle``; the RTL writes to make after the
        edge that ends it (same commit edge on both sides)."""
        if not host_actions(self.machine, self.prog, cycle):
            return []
        p, prog = self.p, self.prog
        if cycle == LOAD_CYCLE:
            writes = [(p.mem[a], prog.image.get(a, 0)) for a in range(prog.imem_words)]
            if prog.entries != prog.default_entries:
                pc_all = 0
                for t in range(THREADS):
                    pc_all |= (prog.entries[t] & 0x3FF) << (10 * t)
                writes.append((p.pc_all, pc_all))
            return writes
        if cycle == RUN_CYCLE - 1:
            return [(p.run_r, prog.run_mask)]
        raise AssertionError("unexpected host action cycle %d" % cycle)

    # ---- the SPI variant: mirror every host action at the cycle it commits
    def mirror_host_pulses(self, cycle):
        """SPI variant: the host-control pulses visible in ``cycle`` commit at
        the edge that ends it; give the model the same actions now."""
        h, m = self.p.host, self.machine
        self.host_touched = []
        consumed = self._apply_expected(cycle)
        if _i(h["h_imem_req"]):
            addr = _i(h["h_imem_addr"])
            if _i(h["h_imem_we"]):
                m.host_write_imem(addr, _i(h["h_imem_wdata"]))
                self.host_log.append((cycle, "imem_write", addr,
                                      _i(h["h_imem_wdata"])))
            else:
                m.host_read_imem(addr)
                self.host_log.append((cycle, "imem_read", addr))
        if _i(h["h_run_we"]) and "h_run_we" not in consumed:
            m.host_set_run(_i(h["h_run"]))
            self.host_log.append((cycle, "run", _i(h["h_run"])))
            if self.run_cycle is None:
                self.run_cycle = cycle + 1
        others = [n for n in ("h_reset", "h_step_we", "h_rpc_we", "h_sfset_we",
                              "h_sfclr_we", "h_badop_clr_we", "h_badop_set15",
                              "h_swirq_clr_we", "h_pout_we", "h_poe_we",
                              "h_od_we", "h_dbg_req", "h_inq_push", "h_outq_pop",
                              "h_badop_set14")
                  if _i(h[n]) and n not in consumed]
        for name in ("irq_en_wr", "irq_en2_wr"):
            handle = getattr(self.p, name)
            if handle is not None and _i(handle) and name not in consumed:
                others.append(name)
        if others:
            self.fail(cycle, [], kind="host action", thread=None,
                      note="host-control pulse %s with no host word due in this "
                           "cycle (the harness mirrors every host action it "
                           "sends; an extra one is the RTL's own)" % others)
        # The byte that ends in this cycle says what is due in the next one.
        if _i(self.p.cs_active):
            if _i(self.p.byte_done):
                self._byte_done(cycle, self.byte_index)
                self.byte_index += 1
        else:
            self.byte_index = 0

    def _byte_done(self, cycle, index):
        """Byte ``index`` of the transaction the host task is sending has just
        gone out: schedule the model action its word commits at the next edge,
        and take the model's peek when a FIFO read word is loaded."""
        txn = self.txn
        if txn is None:
            return                          # the program load: pulse-mirrored
        if txn.kind == "push":
            word = index - 4                # cmd, addr hi, addr lo, then words
            if word >= 0 and word % 2 == 0 and word // 2 < len(txn.words):
                self.expect.append((cycle + 1,
                                    ("push", txn.thread, txn.words[word // 2])))
        elif txn.kind == "pop":
            load = index - 3                # the dummy byte, then every word
            if load >= 0 and load % 2 == 0 and load // 2 < txn.count:
                j = load // 2
                popping = j > 0 and self.peeks[j - 1] is not None
                self.peeks.append(peek_word(self.machine, txn.thread, popping))
                if popping:
                    COVERAGE.host_event("host_read:next_entry")
            end = index - 5
            if end >= 0 and end % 2 == 0 and end // 2 < txn.count:
                self.expect.append((cycle + 1, ("pop", txn.thread, end // 2)))
        elif index == 4:                    # cmd, addr hi, addr lo, hi, lo
            self.expect.append((cycle + 1, ("ctrl", txn.reg, txn.value)))

    def _apply_expected(self, cycle):
        """Give the model every host action due in ``cycle`` and check that
        the RTL raised the matching pulse with the matching data."""
        h, m = self.p.host, self.machine
        due = [a for c, a in self.expect if c == cycle]
        if not due:
            return set()
        self.expect = [(c, a) for c, a in self.expect if c != cycle]
        consumed = set()
        for action in due:
            if action[0] == "push":
                _, thread, word = action
                self._expect_pulse(cycle, "h_inq_push", thread,
                                   "the host push into INQ[%d]" % thread)
                got = _i(h["h_fifo_wdata"])
                if got != word:
                    self.fail(cycle, [("h_fifo_wdata", got, word)],
                              kind="host action", thread=thread,
                              note="the word the host pushed into INQ[%d]" % thread)
                full = len(m.threads[thread].inq) >= self.build.fifo_depth
                m.host_fifo_push(thread, word)
                self.host_log.append((cycle, "fifo_push", thread, word))
                COVERAGE.host_event("host_push:dropped(full)" if full
                                    else "host_push:accepted")
                if full:
                    COVERAGE.host_event("BADOP14:host_push_full")
                self.host_touched.append((thread, "push"))
                consumed.add("h_inq_push")
            elif action[0] == "pop":
                _, thread, word = action
                peeked = self.peeks[word] if word < len(self.peeks) else None
                if peeked is not None:
                    self._expect_pulse(cycle, "h_outq_pop", thread,
                                       "the host pop of OUTQ[%d]" % thread)
                    consumed.add("h_outq_pop")
                    COVERAGE.host_event("host_read:word")
                    self.host_log.append((cycle, "fifo_pop", thread))
                else:
                    if not _i(h["h_badop_set14"]):
                        self.fail(cycle, [("h_badop_set14", 0, 1)],
                                  kind="host action", thread=thread,
                                  note="the model peeked nothing in OUTQ[%d], so "
                                       "the word that just went out should set "
                                       "BADOP[14] (SEMANTICS 6.7)" % thread)
                    consumed.add("h_badop_set14")
                    COVERAGE.host_event("host_read:empty_peek")
                    COVERAGE.host_event("BADOP14:host_read_empty")
                    if m.threads[thread].outq:
                        COVERAGE.host_event("host_read:filled_after_empty_peek")
                    self.host_log.append((cycle, "fifo_pop_empty", thread))
                pop_after_peek(m, thread, peeked)
                self.host_touched.append((thread, "read"))
            else:
                _, reg, value = action
                consumed |= self._ctrl_write(cycle, reg, value)
        return consumed

    def _expect_pulse(self, cycle, name, thread, what):
        got = _i(self.p.host[name])
        if not (got >> thread) & 1:
            self.fail(cycle, [(name, got, 1 << thread)], kind="host action",
                      thread=thread,
                      note="%s commits at this edge (HOST_PROTOCOL: the word's "
                           "effect is registered at E+3 and lands at E+4), so "
                           "%s should be set" % (what, name))

    def _ctrl_write(self, cycle, reg, value):
        """One CTRL write of the host plan: check the strobe, tell the model."""
        m = self.machine
        if reg in CTRL_WRITE_PULSE:
            strobe, bus, mask = CTRL_WRITE_PULSE[reg]
            if not _i(self.p.host[strobe]):
                self.fail(cycle, [(strobe, 0, 1)], kind="host action",
                          note="the host wrote CTRL.%s = %04X; its write strobe "
                               "is due in this cycle" % (reg, value))
            got = _i(self.p.host[bus])
            if got != value & mask:
                self.fail(cycle, [(bus, got, value & mask)], kind="host action",
                          note="the value the host wrote to CTRL.%s" % reg)
            consumed = {strobe}
        else:
            name = "irq_en_wr" if reg == "IRQ_EN" else "irq_en2_wr"
            handle = getattr(self.p, name)
            assert handle is not None, \
                "loom_host_ctl.%s is needed to mirror CTRL.%s writes" % (name, reg)
            if not _i(handle):
                self.fail(cycle, [(name, 0, 1)], kind="host action",
                          note="the host wrote CTRL.%s = %04X; loom_host_ctl's "
                               "write strobe is due in this cycle" % (reg, value))
            consumed = {name}
        if reg == "RUN":
            m.host_set_run(value & 0xF)
            self.host_log.append((cycle, "run", value & 0xF))
        elif reg == "SWIRQ":
            m.host_clear_swirq(value & 0xF)
            self.host_log.append((cycle, "swirq_clr", value & 0xF))
        elif reg == "BADOP":
            m.host_clear_badop(value & 0xFFFF)
            self.host_log.append((cycle, "badop_clr", value & 0xFFFF))
        elif reg == "SFLAGS":
            m.host_write_sflags_set(value & 0xFF)
            self.host_log.append((cycle, "sflags_set", value & 0xFF))
        elif reg == "SFLAGS_CLR":
            m.host_write_sflags_clr(value & 0xFF)
            self.host_log.append((cycle, "sflags_clr", value & 0xFF))
        elif reg == "IRQ_EN":
            m.host_write_irq_en(value & 0xFFFF)
            self.host_log.append((cycle, "irq_en", value & 0xFFFF))
        else:
            m.host_write_irq_en2(value & 0xF)
            self.host_log.append((cycle, "irq_en2", value & 0xF))
        COVERAGE.host_event(CTRL_WRITE_EVENT[reg])
        return consumed

    async def host_traffic(self):
        """Send the program's host plan over the SPI pads, over and over."""
        plan, host = self.traffic, self.host
        index = 0
        while True:
            txn = plan[index]
            index += 1
            if txn.gap:
                await ClockCycles(self.dut.clk, txn.gap)
            self.peeks = []
            self.txn = txn
            try:
                if txn.kind == "push":
                    await host.write(SP_FIFO, FIFO_QUEUE + txn.thread,
                                     list(txn.words))
                elif txn.kind == "pop":
                    got = await host.read(SP_FIFO, FIFO_QUEUE + txn.thread,
                                          txn.count)
                    want = [0 if p is None else p for p in self.peeks]
                    if got != want and self.host_error is None:
                        self.host_error = (
                            "the host read %s from OUTQ[%d] over SPI; the model "
                            "had %s in the cycles the words were loaded "
                            "(SEMANTICS 6.7 peek)"
                            % (["%04X" % w for w in got], txn.thread,
                               ["%04X" % w for w in want]))
                    if txn.count > 1:
                        COVERAGE.host_event("host_read:multi_word")
                else:
                    await host.write(SP_CTRL, CTRL_WRITE_ADDR[txn.reg], txn.value)
            finally:
                self.txn = None
            self.txn_count += 1

    # ------------------------------------------------------------ lockstep
    async def reset(self):
        dut, p = self.dut, self.p
        dut.rst_n.value = 0
        dut.ui_in.value = UI_CS
        dut.uio_drv.value = 0
        await ClockCycles(dut.clk, 3)
        if self.spi:
            # Power-on contents: the model reads 0 wherever nothing was written.
            for addr in range(self.prog.imem_words):
                p.mem[addr].value = 0
        await ClockCycles(dut.clk, 2)
        dut.rst_n.value = 1        # applied after this edge: it was edge 0

    def ui_host_bits(self):
        if self.host is None:
            return UI_CS
        return ((self.host._cs & 1) << 4) | ((self.host._sck & 1) << 5) \
            | ((self.host._mosi & 1) << 6)

    def _irq_causes(self):
        """Which cause of SEMANTICS 6.8 is active in this cycle."""
        m = self.machine
        stat = m.irq_stat & m.irq_en
        causes = []
        if m.swirq & 0xF:
            causes.append("SWIRQ")
        if stat & 0xFF00:
            causes.append("SFLAGS")
        if stat & 0x00F0:
            causes.append("INQ_NOT_FULL")
        if stat & 0x000F:
            causes.append("OUTQ_NOT_EMPTY")
        if m.irq_stat2 & m.irq_en2:
            causes.append("HALTED")
        return causes

    def _latch_events(self, lat_before, model, w_wait_active):
        """A staged pin write that landed at this edge, and by which rule
        (SEMANTICS 6.10): rule 2 when the thread's own slot wrote ``TD`` at
        this edge, rule 1 when ``NOW`` ticked onto ``TD``."""
        m = self.machine
        for t, was in enumerate(lat_before):
            th = m.threads[t]
            if not was or th.lat_valid:
                continue
            rule = 1
            if model is not None and model.thread == t and model.done:
                decoded = ISA.decode(model.ir)
                if decoded is not None:
                    name, fields = decoded[0].name, decoded[1]
                    if name == "SETD" or (name == "WAITD" and not w_wait_active) \
                            or (name == "CSRW"
                                and ISA.csrs.get(fields["csr"], {}).get("name") == "TD"):
                        rule = 2
            ordinary = False
            if model is not None and model.done:
                decoded = ISA.decode(model.ir)
                if decoded is not None and decoded[0].name == "SETP" \
                        and not decoded[1]["lat"] and decoded[1]["pin"] == th.lat_pin:
                    ordinary = True
            COVERAGE.latch_fired(rule, th.lat_pin, m.od_mask, ordinary)

    async def run(self):
        dut, p, m, plan = self.dut, self.p, self.machine, self.prog.stimulus
        await self.reset()
        loader = traffic = None
        if self.spi:
            self.host._cs, self.host._sck, self.host._mosi = 1, 0, 0

            async def load_and_run():
                await ClockCycles(dut.clk, 2)
                await self.host.load_program(self.prog.image, verify=False)
                await self.host.write(SP_CTRL, CTRL_RUN, self.prog.run_mask)
                if self.traffic is not None:
                    await self.host_traffic()

            loader = cocotb.start_soon(load_and_run())

        try:
            k = await self._lockstep(loader)
        finally:
            for task in (loader, traffic):
                if task is not None and not task.done():
                    task.cancel()
        return k

    def _check_host_task(self, loader, cycle):
        """A host task that died takes the traffic with it, and the run would
        then pass for the wrong reason."""
        if not loader.done():
            return
        try:
            exc = loader.exception()
        except BaseException:                    # cancelled: nothing to report
            return
        if exc is not None:
            raise AssertionError("cycle %d: the SPI host task failed: %r"
                                 % (cycle, exc)) from exc

    async def _lockstep(self, loader):
        dut, p, m, plan = self.dut, self.p, self.machine, self.prog.stimulus
        prev_rtl = None          # record in W during the previous cycle
        prev_model = None
        prev_od = 0              # OD_MASK as visible in the previous cycle
        pc_prev = None
        last_w = [None] * THREADS   # (cycle, next_pc) of each thread's last W
        k = 0
        limit = (RUN_CYCLE + self.cycles) if not self.spi else 400000
        while True:
            if self.run_cycle is not None and k >= self.run_cycle + self.cycles:
                break
            if k >= limit:
                raise AssertionError("seed %d: RUN never committed over SPI"
                                     % self.prog.seed)
            await FallingEdge(dut.clk)
            if self.host_error is not None:
                self.fail(k, [], kind="host data", note=self.host_error)
            if loader is not None and k % 64 == 0:
                self._check_host_task(loader, k)

            # ---- RTL during cycle k
            ph = _i(p.ph)
            regs_rtl = (_i(p.run_r), _i(p.halted_r), _i(p.badop_r),
                        _i(p.sflags_r), _i(p.od_mask))
            uo_out = _i(p.uo_out)
            pads_rtl = (uo_out & 0x3F, (uo_out >> 6) & 1, _i(p.uio_out),
                        _i(p.uio_oe))
            pc_now = _i(p.pc_all)
            x_error = None
            try:
                rtl = _rtl_record(p) if _i(p.tr_valid) else None
            except AssertionError as exc:
                # An X or Z in the retire record. Report it with the model's
                # view of the same slot rather than as a bare exception.
                rtl, x_error = None, str(exc)

            # ---- model during cycle k (before it steps)
            regs_model = (m.run, m.halted, m.badop, m.sflags, m.od_mask)
            pads_model = (m.uo_out, m.host_irq, m.uio_out, m.uio_oe)
            guards = []
            if ph != m.ph or ph != k % 4:
                guards.append(("ph", ph, m.ph))
            for name, got, want in zip(("RUN", "HALTED", "BADOP", "SFLAGS", "OD_MASK"),
                                       regs_rtl, regs_model):
                if got != want:
                    guards.append((name, got, want))
            for name, got, want in zip(("uo_out[5:0]", "HOST_IRQ uo_out[6]",
                                        "uio_out", "uio_oe"),
                                       pads_rtl, pads_model):
                if got != want:
                    guards.append((name, got, want))
            if guards:
                # A shared-state or pad change lands at the edge that ends a
                # W cycle, so the slot to blame was in W during cycle k-1.
                blame = prev_model.thread if prev_model is not None else None
                self.fail(k, guards, kind="pads/shared state", thread=blame,
                          note="checked at the start of cycle %d; the slot "
                               "in W during cycle %d committed at this edge"
                               % (k, k - 1))
            if STATE_EVERY and k and k % STATE_EVERY == 0:
                self.check_state(k)

            # ---- L2-SLOT on the RTL's own PC registers
            if pc_prev is not None and pc_now != pc_prev:
                for t in range(THREADS):
                    old, new = _field(pc_prev, t, 10), _field(pc_now, t, 10)
                    if old == new:
                        continue
                    ok = (prev_rtl is not None and prev_rtl[0] == t
                          and (k - 1) % 4 == (t + 3) % 4 and prev_rtl[8] == new)
                    if not ok and not (k - 1) in (LOAD_CYCLE, RUN_CYCLE - 1):
                        self.fail(k, [("t%d.PC" % t, new, old)], kind="L2-SLOT",
                                  thread=t, note="PC[%d] changed at edge %d, "
                                  "which does not end a W cycle of thread %d "
                                  "(RTL record in W at %d: %s)"
                                  % (t, k, t, k - 1, _fmt(prev_rtl)))
            pc_prev = pc_now

            # ---- host actions that commit at edge k+1
            deposits = []
            if self.spi:
                self.mirror_host_pulses(k)
            else:
                deposits = self.backdoor_deposits(k)

            # ---- pads held at edge k+1, the same on both sides
            ui, uio = plan.at(k)
            if self.host is not None:
                self.host.ui_ext = ui
            dut.ui_in.value = ui | self.ui_host_bits()
            dut.uio_drv.value = uio
            m.set_pad_inputs(ui_in=ui, uio_in=uio)

            # ---- one model cycle: the slot in W during cycle k
            # The W slot of cycle k belongs to thread (k - 3) mod 4 and commits
            # at edge k+1, so right now that thread's own state is still what
            # the slot saw in X (coverage context).
            od_now = m.od_mask
            th = m.threads[(k + 1) % 4]
            context = SlotContext(
                features=self.build.features, fifo_depth=self.build.fifo_depth,
                depth=th.depth, outgrp=th.outgrp, ingrp=th.ingrp,
                regs=list(th.regs), od_mask=prev_od, inq=len(th.inq),
                outq=len(th.outq), sr=th.sr, cnt=th.cnt, crc=th.crc,
                be_cfg=th.be_cfg, be_pins=th.be_pins, lat_valid=th.lat_valid,
                w_record=prev_model)
            w_wait_active = th.wait_active
            lat_before = [t.lat_valid for t in m.threads]
            irq_before = m.host_irq
            causes = self._irq_causes()
            model = m.step_cycle()
            if x_error is not None:
                self.fail(k, [], None, model, kind="unresolvable RTL signal",
                          note=x_error + " (the model's record for the slot in "
                                         "W this cycle is below)")
            diffs = _diff_record(rtl, model)
            if diffs:
                self.fail(k, diffs, rtl, model)

            # ---- M2 events that have no retire record of their own
            self._latch_events(lat_before, model, w_wait_active)
            if m.host_irq != irq_before:
                COVERAGE.irq_event("HOST_IRQ:rise" if m.host_irq
                                   else "HOST_IRQ:clear")
                if m.host_irq:
                    for cause in causes:
                        COVERAGE.irq_event("cause:%s" % cause)
            for thread, what in self.host_touched:
                if model is not None and model.thread == thread and model.done:
                    name = (ISA.decode(model.ir) or (None,))[0]
                    name = name.name if name is not None else ""
                    if name == "PUSH" and what == "read":
                        COVERAGE.host_event("same_edge:thread_push+host_read")
                    elif name == "POP" and what == "push":
                        COVERAGE.host_event("same_edge:thread_pop+host_push")
            self.host_touched = []

            if model is not None:
                self.retired += 1
                t = model.thread
                # L2-SLOT: W of thread t only when ph == (t + 3) mod 4; a
                # running thread's slots are exactly 4 cycles apart and each
                # one fetches the previous one's next PC; only waits and the
                # blocking FIFO ops stall.
                problems = []
                if k % 4 != (t + 3) % 4:
                    problems.append("W of thread %d in a cycle with ph=%d" % (t, k % 4))
                if last_w[t] is not None:
                    gap_cycle, gap_pc = last_w[t]
                    if k - gap_cycle == 4 and rtl[1] != gap_pc:
                        problems.append("fetched %03X, previous slot said %03X"
                                        % (rtl[1], gap_pc))
                if not model.done:
                    decoded = ISA.decode(model.ir)
                    if decoded is None or decoded[0].timing not in ("wait", "blocking") \
                            or model.next_pc != model.pc:
                        problems.append("a stall that is not a wait or blocking "
                                        "FIFO re-issue")
                if problems:
                    self.fail(k, [], rtl, model, kind="L2-SLOT",
                              note="; ".join(problems))
                last_w[t] = (k, model.next_pc)
                self.history[t].append("W %6d  %s" % (k, _fmt(_model_tuple(model))))
                COVERAGE.note(model, context)
            prev_rtl, prev_model, prev_od = rtl, model, od_now

            if deposits:
                await RisingEdge(dut.clk)
                for handle, value in deposits:
                    handle.value = value
            k += 1

        # ---- end of seed: the whole state, both as visible in cycle k
        await FallingEdge(dut.clk)
        self.check_state(k)
        return k


# ------------------------------------------------------------ CAPS probe
async def _read_caps(dut) -> int:
    """``CTRL.CAPS`` of the RTL, read through the SPI host port after a reset.

    Every seed starts with its own reset in :meth:`_Run.reset`, so whatever
    this leaves behind does not matter.
    """
    dut.rst_n.value = 0
    dut.ui_in.value = UI_CS
    dut.uio_drv.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 4)
    return await LoomHost(dut).read1(SP_CTRL, CTRL_CAPS)


def _avoid_for(caps: int):
    """The avoid flags for an RTL that reports ``caps``. The M2 features are
    not avoided any more: both sides are built for them (see ``_Build``)."""
    return AVOID


def _make_program(make, build: _Build):
    """Call a program maker. Makers that take the build get it (the M2
    constructs need to know the FIFO depth and which features are built);
    older ones (``test_flops.py``) take the avoid flags only and generate for
    the default build, which is then checked against ``CAPS``."""
    avoid = _avoid_for(build.caps)
    try:
        takes_build = "build" in inspect.signature(make).parameters
    except (TypeError, ValueError):                  # pragma: no cover
        takes_build = False
    prog = make(avoid, build=build) if takes_build else make(avoid)
    if prog.imem_words != build.imem_words:
        raise AssertionError("CAPS %04X reports %d instruction words, the "
                             "program is for %d" % (build.caps, build.imem_words,
                                                    prog.imem_words))
    if tuple(prog.features) != build.features or \
            ("FIFO" in build.features and prog.fifo_depth != build.fifo_depth):
        raise AssertionError(
            "the program was generated for features %s (FIFO depth %d) but "
            "CAPS %04X says the chip has %s (depth %d); the two sides would "
            "not mean the same thing by an M2 instruction"
            % (",".join(prog.features) or "M1", prog.fifo_depth, build.caps,
               ",".join(build.features) or "M1", build.fifo_depth))
    return prog


# ------------------------------------------------------------- failures
def _dump(prog: GeneratedProgram, info) -> pathlib.Path:
    FAILURE_DIR.mkdir(parents=True, exist_ok=True)
    path = FAILURE_DIR / ("seed_%d.json" % prog.seed)
    obj = prog.to_obj()
    obj["failure"] = info
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        json.dump(obj, handle, indent=2, sort_keys=True)
        handle.write("\n")
    with open(path.with_suffix(".lst"), "w", encoding="utf-8",
              newline="\n") as handle:
        handle.write(prog.disassembly(ISA))
    return path


# ------------------------------------------------------------- the tests
def _start_clock(dut) -> Clock:
    """A simulator-side clock (faster than a Python one); cocotb 2 stops it
    when its task is cancelled, and every test here stops it explicitly."""
    clock = Clock(dut.clk, CLK_NS, unit="ns", impl="gpi")
    clock.start()
    dut.ena.value = 1
    return clock


def _seed_plan(count, base):
    """(seed, profile, threads): profiles rotate, most seeds run 4 threads;
    the one- and two-thread programs are the last of every six."""
    for index in range(count):
        seed = base + index
        threads = 4 if index % 6 != 5 else 1 + (index // 6) % 3
        yield seed, PROFILE_ORDER[index % len(PROFILE_ORDER)], threads


async def _run_programs(dut, programs, spi=False):
    """``programs``: (program or maker, label, cycles). A maker is called with
    the avoid flags the RTL's CAPS asks for, and the build itself if it takes
    one, and returns the program."""
    _require_rtl(dut)
    probe = _Probe(dut)
    failures = []
    total_slots = total_cycles = 0
    started = time.time()
    for make, label, cycles in programs:
        caps = await _read_caps(dut)
        build = _Build(caps)
        if callable(make):
            prog = _make_program(make, build)
            label = "%s %s%s" % (label, build,
                                 " +host traffic" if prog.host else "")
        else:
            prog = make
            label = "%s %s" % (label, build)
        if probe.mem_words != prog.imem_words:
            raise AssertionError("the RTL instruction array has %d words, the "
                                 "program is for %d" % (probe.mem_words,
                                                        prog.imem_words))
        COVERAGE.note_build(build.features, build.fifo_depth)
        run = _Run(dut, probe, prog, label, cycles, build, spi=spi)
        try:
            spent = await run.run()
        except Divergence as exc:
            if not KEEP_GOING:
                raise
            failures.append(str(exc))
            dut._log.error(str(exc))
            continue
        COVERAGE.note_seed("%d/%s%s" % (prog.seed, prog.profile, "/spi" if spi else ""))
        total_slots += run.retired
        total_cycles += spent
        dut._log.info("seed %d (%s, run_mask %X): %d slots matched over %d "
                      "cycles, RUN from cycle %d%s"
                      % (prog.seed, label, prog.run_mask, run.retired, spent,
                         run.run_cycle,
                         ", %d host transactions" % run.txn_count
                         if run.txn_count else ""))
    dut._log.info("%d programs, %d cycles, %d retire records compared in %.1f s"
                  % (len(programs), total_cycles, total_slots,
                     time.time() - started))
    await ClockCycles(dut.clk, 2)
    if failures:
        raise AssertionError("%d of %d programs diverged:\n%s"
                             % (len(failures), len(programs), "\n".join(failures)))


@cocotb.test(skip=GATE_LEVEL)
async def test_cosim_random_programs(dut):
    """L2-RAND/L2-TRACE/L2-SLOT: random programs, compared on every cycle."""
    clock = _start_clock(dut)
    if REPLAY:
        with open(REPLAY, "r", encoding="utf-8") as handle:
            obj = json.load(handle)
        prog = GeneratedProgram.from_obj(obj)
        cycles = int(obj.get("failure", {}).get("cycles", CYCLES) or CYCLES)
        programs = [(prog, "replay of %s" % REPLAY, cycles)]
    else:
        programs = []
        for seed, profile, threads in _seed_plan(SEEDS, SEED_BASE):
            def make(avoid, build, seed=seed, profile=profile, threads=threads):
                return generate(seed=seed, threads=threads,
                                imem_words=build.imem_words, profile=profile,
                                cycles=CYCLES, avoid=avoid, isa=ISA,
                                features=build.features,
                                fifo_depth=build.fifo_depth)
            programs.append((make, "backdoor profile=%s threads=%d"
                             % (profile, threads), CYCLES))
    try:
        await _run_programs(dut, programs)
    finally:
        clock.stop()


@cocotb.test(skip=GATE_LEVEL or SPI_SEEDS <= 0 or bool(REPLAY))
async def test_cosim_over_the_host_port(dut):
    """The same comparison with the image, ``CTRL.RUN`` and the host's FIFO
    and interrupt traffic written over SPI."""
    clock = _start_clock(dut)
    programs = []
    for index in range(SPI_SEEDS):
        seed = SEED_BASE + 5000 + index
        profile = ("m2", "timing", "pins", "mixed")[index % 4]
        def make(avoid, build, seed=seed, profile=profile):
            return generate(seed=seed, threads=2, imem_words=build.imem_words,
                            profile=profile, cycles=SPI_CYCLES, avoid=avoid,
                            isa=ISA, features=build.features,
                            fifo_depth=build.fifo_depth,
                            host_traffic="FIFO" in build.features)
        programs.append((make, "spi profile=%s threads=2" % profile, SPI_CYCLES))
    try:
        await _run_programs(dut, programs, spi=True)
    finally:
        clock.stop()


@cocotb.test(skip=GATE_LEVEL)
async def test_cosim_coverage_report(dut):
    """L2-COV: print the functional coverage table, write cosim_coverage.json."""
    COVERAGE.write(COVERAGE_PATH)
    dut._log.info(COVERAGE.table())
    holes = COVERAGE.holes()
    if holes:
        dut._log.warning("L2-COV: %d bins stayed empty (listed above): %s"
                         % (len(holes), ", ".join("%s/%s" % h for h in holes)))
    await Timer(1, "ns")
    assert COVERAGE.slots > 0, "no retire records were compared"
