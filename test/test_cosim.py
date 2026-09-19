# SPDX-License-Identifier: Apache-2.0
"""L2-RAND, L2-TRACE, L2-SLOT, L2-COV: the RTL and the golden model in lockstep.

``tools/loomgen`` generates a constrained-random program and a pad stimulus
plan; this module runs them on ``src/`` (Icarus) and on ``tools/loomsim`` one
clock at a time and compares, on **every cycle**:

* the retire record of ``docs/SEMANTICS.md`` section 8 (``tr_*``, reached
  hierarchically in ``loom_top``) against the model's ``RetireRecord`` for the
  slot in W that cycle (``rd``/``val`` only when ``we`` is set);
* the pad outputs ``uo_out[5:0]``, ``uio_out`` and ``uio_oe`` (``uo_out[7:6]``
  are the host MISO and IRQ pins and are masked);
* as guards, ``ph`` (and that it equals ``k mod 4``), ``RUN``, ``HALTED``,
  ``BADOP``, ``SFLAGS`` and ``OD_MASK``;
* L2-SLOT: a thread's ``PC`` register changes only at the edge that ends a W
  cycle of that thread, that W cycle has ``ph == (t + 3) mod 4``, and the new
  value is that slot's ``tr_next_pc``; a slot that does not complete is a
  wait-class instruction and re-issues its own ``PC``.

Every ``LOOM_COSIM_STATE_EVERY`` cycles and at the end of every seed the whole
architectural state of SEMANTICS 5 is read out of the RTL hierarchy and
compared with the model (registers, PC, flags, TD, DT, NOW, ACC, TICK_SEEN,
RS0/RS1/DEPTH, WAIT_ACTIVE, PREV_PINS, STEPS, TICK_INT/FRAC, OUTGRP/INGRP,
RUN/HALTED/STEP_REQ/BADOP/SFLAGS/SWIRQ, PIN_OUT/PIN_OE/OD_MASK).

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
every host-control pulse of ``loom_host_ctl`` (IMEM write, RUN write, and every
other one, which would be an error here) is one clock wide in the cycle before
its commit edge, so seeing it at the falling edge of cycle ``k`` and making the
same ``host_*`` call before the model steps cycle ``k`` commits both at edge
``k + 1``. The lockstep comparison runs through the whole load as well.

M2 features
-----------

At the start of every seed the harness resets the RTL and reads ``CTRL.CAPS``
through the SPI host port. If it reports an M2 feature (FIFOs, bit engine,
deadline-latched ``SETP``), the program is generated with the ``m2_built``
avoid flag of ``tools/loomgen``, which leaves those instructions, the
bit-engine CSRs and the ``D`` form of ``SETP`` out: the golden model may still
treat them as unbuilt (``NOP`` + ``BADOP``) while the RTL builds them.

Environment
-----------

``LOOM_COSIM_SEEDS`` (default 12) and ``LOOM_COSIM_CYCLES`` (default 4000, per
seed, counted from ``RUN``) size the random run; ``LOOM_COSIM_SEED_BASE``
(default 1) is the first seed; ``LOOM_COSIM_SPI_SEEDS`` (default 2) and
``LOOM_COSIM_SPI_CYCLES`` (default 1500) size the host-path variant;
``LOOM_COSIM_STATE_EVERY`` (default 500) sets how often the full state is
compared; ``LOOM_COSIM_KEEP_GOING=1`` records a failure and moves on to the
next seed instead of stopping; ``LOOM_COSIM_REPLAY=path.json`` runs one saved
program (a file from ``test/cosim_failures/`` or ``python -m tools.loomgen
-o``) instead of the random set.

A divergence fails with the seed, cycle, thread, PC, the disassembled
instruction, both records and the thread's last 12 retire records, and writes
``test/cosim_failures/seed_<n>.json`` (image, stimulus, failure) plus a
``.lst`` listing; ``python -m tools.loomgen --replay FILE --run N --trace T``
shows the model's side of it. The module skips itself on a gate-level netlist
(``GATES=yes``), which has none of the signals it reads. In an RTL run a
signal it cannot find fails the tests instead of skipping them (the
instruction array moved when the SRAM macro came in, D-020, and a silent skip
would have looked like a pass).
"""

from __future__ import annotations

import collections
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

from cosim_coverage import Coverage                              # noqa: E402
from spi_host import (CLK_NS, CTRL_CAPS, CTRL_RUN, SP_CTRL,       # noqa: E402
                      LoomHost)
from tools.loomasm.disasm import disassemble                     # noqa: E402
from tools.loomgen import (GeneratedProgram, LOAD_CYCLE, RUN_CYCLE,  # noqa: E402
                           generate, host_actions)
from tools.loomisa import load as load_isa                       # noqa: E402
from tools.loomsim import Machine                                # noqa: E402

THREADS = 4
#: Instruction memory size of the default build: the 512 x 16 SRAM macro
#: (D-020). The harness checks it against CAPS[15:12] and the RTL array.
IMEM_WORDS = 512


def _env_int(name: str, default: int) -> int:
    return int(os.environ.get(name, str(default)) or default)


SEEDS = _env_int("LOOM_COSIM_SEEDS", 12)
CYCLES = _env_int("LOOM_COSIM_CYCLES", 4000)
SEED_BASE = _env_int("LOOM_COSIM_SEED_BASE", 1)
SPI_SEEDS = _env_int("LOOM_COSIM_SPI_SEEDS", 2)
SPI_CYCLES = _env_int("LOOM_COSIM_SPI_CYCLES", 1500)
STATE_EVERY = _env_int("LOOM_COSIM_STATE_EVERY", 500)
KEEP_GOING = os.environ.get("LOOM_COSIM_KEEP_GOING", "") not in ("", "0")
REPLAY = os.environ.get("LOOM_COSIM_REPLAY", "")

PROFILE_ORDER = ("mixed", "timing", "pins", "alu")

#: Constructs ``docs/spec-questions/cosim.md`` has open. Neither side may be
#: changed for them until the director rules, so the random programs leave
#: them out and the rest of the run stays useful.
AVOID = ("csrw_pin_out_high_bits",)

#: CAPS bits of M2 features (SEMANTICS 5): [3] FIFOs, [4] bit engine (manual
#: mode), [7] deadline-latched ``SETP``. If the RTL reports any of them, the
#: programs are generated with ``m2_built`` as well (see the module notes).
CAPS_M2_FEATURES = (1 << 3) | (1 << 4) | (1 << 7)

HERE = pathlib.Path(__file__).resolve().parent
FAILURE_DIR = HERE / "cosim_failures"
COVERAGE_PATH = HERE / "cosim_coverage.json"

ISA = load_isa()
COVERAGE = Coverage(ISA)

#: SPI host pins in ui_in, idle: CS_n high, SCK low, MOSI low.
UI_CS = 1 << 4
UI_HOST_MASK = 0x70


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
        imem_array(dut.user_project.u_loom)
    except AttributeError:
        return False
    return True


def _require_rtl(dut):
    """Fail, rather than skip, an RTL run that lacks the signals read here."""
    assert _rtl_hierarchy_present(dut), (
        "RTL hierarchy not found: user_project.u_loom.u_core.ph and the "
        "instruction array of loom_imem (macro model memory or g_flops.mem) "
        "are needed; gate-level runs must set GATES=yes")


#: cocotb 2 has no run-time skip, so the flag is decided at import time.
#: The gate-level flows (test/Makefile, Tiny Tapeout's gl_test) set GATES=yes.
GATE_LEVEL = os.environ.get("GATES", "").lower() == "yes"


# ------------------------------------------------------------------ probe
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
        # Host-control pulses (loom_host_ctl outputs), for the SPI variant.
        self.host = {name: getattr(loom, name) for name in (
            "h_imem_req", "h_imem_we", "h_imem_addr", "h_imem_wdata",
            "h_run_we", "h_run", "h_reset", "h_step_we", "h_step",
            "h_rpc_we", "h_rpc_sel", "h_rpc", "h_sfset_we", "h_sfset",
            "h_sfclr_we", "h_sfclr", "h_badop_clr_we", "h_badop_clr",
            "h_badop_set15", "h_swirq_clr_we", "h_pout_we", "h_pout",
            "h_poe_we", "h_poe", "h_od_we", "h_od", "h_dbg_req")}


def _i(handle) -> int:
    """``int(handle.value)``, naming the signal when it holds X or Z."""
    try:
        return int(handle.value)
    except ValueError:
        raise AssertionError("signal %s is not resolvable: %s"
                             % (handle._path, handle.value)) from None


def _field(word: int, index: int, width: int) -> int:
    return (word >> (index * width)) & ((1 << width) - 1)


# --------------------------------------------------------------- records
FIELDS = ("thread", "pc", "ir", "done", "we", "rd", "val", "flags", "next_pc")


def _rtl_record(p: _Probe):
    return (_i(p.tr_thread), _i(p.tr_pc), _i(p.tr_ir), _i(p.tr_done),
            _i(p.tr_we), _i(p.tr_rd), _i(p.tr_val), _i(p.tr_flags),
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
def _state_pairs(p: _Probe, machine: Machine):
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
    return pairs


# ------------------------------------------------------------- the run
class Divergence(AssertionError):
    """The RTL and the model disagree; the message is the full report."""


class _Run:
    """One program, one reset, one lockstep comparison."""

    def __init__(self, dut, probe: _Probe, prog: GeneratedProgram, label: str,
                 cycles: int, spi: bool = False):
        self.dut = dut
        self.p = probe
        self.prog = prog
        self.label = label
        self.cycles = cycles
        self.spi = spi
        self.machine = Machine(imem_words=prog.imem_words, loopback=True, isa=ISA)
        self.history = [collections.deque(maxlen=12) for _ in range(THREADS)]
        self.host = LoomHost(dut) if spi else None
        self.host_log = []
        self.run_cycle = RUN_CYCLE if not spi else None
        self.retired = 0

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
                "spi": self.spi, "thread": thread,
                "diffs": [[n, g, w] for n, g, w in diffs],
                "rtl": _fmt(rtl) if rtl is not None else None,
                "model": _fmt(_model_tuple(model)) if model is not None else None,
                "history": list(self.history[thread]) if thread is not None else []}
        path = _dump(self.prog, info)
        raise Divergence("\n".join(lines) + "\n\n  replay: %s\n" % path)

    def check_state(self, cycle):
        bad = [(n, g, w) for n, g, w in _state_pairs(self.p, self.machine) if g != w]
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

    def mirror_host_pulses(self, cycle):
        """SPI variant: the host-control pulses visible in ``cycle`` commit at
        the edge that ends it; give the model the same actions now."""
        h, m = self.p.host, self.machine
        if _i(h["h_imem_req"]):
            addr = _i(h["h_imem_addr"])
            if _i(h["h_imem_we"]):
                m.host_write_imem(addr, _i(h["h_imem_wdata"]))
                self.host_log.append((cycle, "imem_write", addr))
            else:
                m.host_read_imem(addr)
        if _i(h["h_run_we"]):
            m.host_set_run(_i(h["h_run"]))
            self.host_log.append((cycle, "run", _i(h["h_run"])))
            if self.run_cycle is None:
                self.run_cycle = cycle + 1
        others = [n for n in ("h_reset", "h_step_we", "h_rpc_we", "h_sfset_we",
                              "h_sfclr_we", "h_badop_clr_we", "h_badop_set15",
                              "h_swirq_clr_we", "h_pout_we", "h_poe_we",
                              "h_od_we", "h_dbg_req") if _i(h[n])]
        if others:
            raise AssertionError("cycle %d: host action %s is not mirrored into "
                                 "the model by this harness" % (cycle, others))

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

    async def run(self):
        dut, p, m, plan = self.dut, self.p, self.machine, self.prog.stimulus
        await self.reset()
        loader = None
        if self.spi:
            self.host._cs, self.host._sck, self.host._mosi = 1, 0, 0

            async def load_and_run():
                await ClockCycles(dut.clk, 2)
                await self.host.load_program(self.prog.image, verify=False)
                await self.host.write(SP_CTRL, CTRL_RUN, self.prog.run_mask)

            loader = cocotb.start_soon(load_and_run())

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

            # ---- RTL during cycle k
            ph = _i(p.ph)
            regs_rtl = (_i(p.run_r), _i(p.halted_r), _i(p.badop_r),
                        _i(p.sflags_r), _i(p.od_mask))
            pads_rtl = (_i(p.uo_out) & 0x3F, _i(p.uio_out), _i(p.uio_oe))
            pc_now = _i(p.pc_all)
            rtl = _rtl_record(p) if _i(p.tr_valid) else None

            # ---- model during cycle k (before it steps)
            regs_model = (m.run, m.halted, m.badop, m.sflags, m.od_mask)
            pads_model = (m.uo_out, m.uio_out, m.uio_oe)
            guards = []
            if ph != m.ph or ph != k % 4:
                guards.append(("ph", ph, m.ph))
            for name, got, want in zip(("RUN", "HALTED", "BADOP", "SFLAGS", "OD_MASK"),
                                       regs_rtl, regs_model):
                if got != want:
                    guards.append((name, got, want))
            for name, got, want in zip(("uo_out[5:0]", "uio_out", "uio_oe"),
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
            context = (th.depth, th.outgrp, th.ingrp, list(th.regs))
            model = m.step_cycle()
            diffs = _diff_record(rtl, model)
            if diffs:
                self.fail(k, diffs, rtl, model)

            if model is not None:
                self.retired += 1
                t = model.thread
                # L2-SLOT: W of thread t only when ph == (t + 3) mod 4; a
                # running thread's slots are exactly 4 cycles apart and each
                # one fetches the previous one's next PC; only waits stall.
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
                    if decoded is None or decoded[0].timing != "wait" \
                            or model.next_pc != model.pc:
                        problems.append("a stall that is not a wait re-issue")
                if problems:
                    self.fail(k, [], rtl, model, kind="L2-SLOT",
                              note="; ".join(problems))
                last_w[t] = (k, model.next_pc)
                self.history[t].append("W %6d  %s" % (k, _fmt(_model_tuple(model))))
                depth, outgrp, ingrp, regs = context
                COVERAGE.note(model, depth=depth, outgrp=outgrp, ingrp=ingrp,
                              regs=regs, od_mask=prev_od, w_record=prev_model)
            prev_rtl, prev_model, prev_od = rtl, model, od_now

            if deposits:
                await RisingEdge(dut.clk)
                for handle, value in deposits:
                    handle.value = value
            k += 1

        # ---- end of seed: the whole state, both as visible in cycle k
        await FallingEdge(dut.clk)
        self.check_state(k)
        if loader is not None and not loader.done():
            loader.cancel()
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
    """The avoid flags for an RTL that reports ``caps``."""
    return AVOID + (("m2_built",) if caps & CAPS_M2_FEATURES else ())


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
    """(seed, profile, threads): profiles rotate, most seeds run 4 threads."""
    for index in range(count):
        seed = base + index
        threads = 4 if index % 6 else 1 + (index // 6) % 3
        yield seed, PROFILE_ORDER[index % len(PROFILE_ORDER)], threads


async def _run_programs(dut, programs, spi=False):
    """``programs``: (program or maker, label, cycles). A maker is called with
    the avoid flags that the RTL's CAPS, read at the start of the seed, asks
    for, and returns the program."""
    _require_rtl(dut)
    probe = _Probe(dut)
    failures = []
    total_slots = total_cycles = 0
    started = time.time()
    for make, label, cycles in programs:
        if callable(make):
            caps = await _read_caps(dut)
            prog = make(_avoid_for(caps))
            label = "%s caps=%04X%s" % (label, caps,
                                         " m2_built" if "m2_built" in prog.avoid else "")
            if 1 << (caps >> 12) != prog.imem_words:
                raise AssertionError("CAPS %04X reports %d instruction words, the "
                                     "program is for %d" % (caps, 1 << (caps >> 12),
                                                            prog.imem_words))
        else:
            prog = make
        if probe.mem_words != prog.imem_words:
            raise AssertionError("the RTL instruction array has %d words, the "
                                 "program is for %d" % (probe.mem_words, prog.imem_words))
        run = _Run(dut, probe, prog, label, cycles, spi=spi)
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
                      "cycles, RUN from cycle %d"
                      % (prog.seed, label, prog.run_mask, run.retired, spent,
                         run.run_cycle))
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
            def make(avoid, seed=seed, profile=profile, threads=threads):
                return generate(seed=seed, threads=threads, imem_words=IMEM_WORDS,
                                profile=profile, cycles=CYCLES, avoid=avoid, isa=ISA)
            programs.append((make, "backdoor profile=%s threads=%d"
                             % (profile, threads), CYCLES))
    try:
        await _run_programs(dut, programs)
    finally:
        clock.stop()


@cocotb.test(skip=GATE_LEVEL or SPI_SEEDS <= 0 or bool(REPLAY))
async def test_cosim_over_the_host_port(dut):
    """The same comparison with the image and CTRL.RUN written over SPI."""
    clock = _start_clock(dut)
    programs = []
    for index in range(SPI_SEEDS):
        seed = SEED_BASE + 5000 + index
        profile = PROFILE_ORDER[(index + 1) % len(PROFILE_ORDER)]
        def make(avoid, seed=seed, profile=profile):
            return generate(seed=seed, threads=2, imem_words=IMEM_WORDS,
                            profile=profile, cycles=SPI_CYCLES, avoid=avoid, isa=ISA)
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
        dut._log.warning("L2-COV: %d reachable bins not hit (listed above)"
                         % len(holes))
    await Timer(1, "ns")
    assert COVERAGE.slots > 0, "no retire records were compared"
