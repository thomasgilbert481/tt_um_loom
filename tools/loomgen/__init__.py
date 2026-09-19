"""Constrained-random program generator for Loom co-simulation (L2-RAND).

``tools/loomgen`` builds instruction-memory images that ``test/test_cosim.py``
runs on the RTL and on :mod:`tools.loomsim` at the same time, one clock apiece,
comparing the retire record of ``docs/SEMANTICS.md`` section 8 on every cycle.

Typical use::

    from tools.loomgen import generate

    prog = generate(seed=7, threads=4, imem_words=256, profile="timing")
    prog.image        # {address: word}, the shape the assembler emits
    prog.entries      # start address per thread (D-017 defaults)
    prog.run_mask     # what the harness writes to CTRL.RUN
    prog.stimulus     # what the input pads hold at every edge
    prog.features     # the build it is for: CAPS features and fifo_depth
    prog.host         # host traffic it relies on (SPI plan), or None
    print(prog.disassembly())

A program is generated for one build of the chip (``features``,
``fifo_depth``), the one ``CTRL.CAPS`` reports; instructions of a feature that
build does not have are generated as the ``NOP`` + ``BADOP`` words they are
(SEMANTICS 9). The default is the M2 chip: FIFOs of depth 4, the manual bit
engine and the deadline-latched ``SETP``.

Everything is derived from the arguments: the same call gives the same image
and the same stimulus, on any machine and in any process. ``prog.to_obj()``
and :meth:`GeneratedProgram.from_obj` round-trip a program through JSON, which
is how a failing seed is written to ``test/cosim_failures/`` and replayed.

Command line::

    python -m tools.loomgen --seed 3 --threads 4 --profile pins --listing
    python -m tools.loomgen --seed 3 --features "" --listing      # M1 build
    python -m tools.loomgen --replay test/cosim_failures/seed_3.json --trace 1
"""

from .generator import (AVOID_FLAGS, DEFAULT_FEATURES, DEFAULT_FIFO_DEPTH,
                        FEATURE_OF, FEATURES, FIFO_DEPTHS, M1_CSR_NAMES,
                        M1_MNEMONICS, M2_CSR_NAMES, M2_MNEMONICS, PROFILES,
                        THREADS, UNBUILT_CSR_NAMES, UNBUILT_MNEMONICS,
                        WAITB_BE_IDLE, WAITB_INQ_NE, WAITB_OUTQ_NF, WAITB_TICK,
                        GeneratedProgram, LoomgenError, check_program,
                        generate, instruction_built, normalise_build,
                        static_target, unbuilt_mnemonics)
from .hostplan import CTRL_WRITES, HostPlan, HostTxn, build_host_plan
from .runner import (LOAD_CYCLE, RUN_CYCLE, ModelHost, build_machine,
                     fifo_error, group_host_log, host_actions, peek_word,
                     pop_after_peek, replay_host_log, run_model, trace)
from .stimulus import (INPUT_PINS, UI_BIT, UI_MASK, PinWave, StimulusPlan,
                       build_plan)

__all__ = [
    "generate", "GeneratedProgram", "LoomgenError", "check_program",
    "static_target", "instruction_built", "unbuilt_mnemonics", "normalise_build",
    "PROFILES", "AVOID_FLAGS", "M1_MNEMONICS", "UNBUILT_MNEMONICS",
    "M1_CSR_NAMES", "UNBUILT_CSR_NAMES", "M2_MNEMONICS", "M2_CSR_NAMES",
    "FEATURES", "FEATURE_OF", "DEFAULT_FEATURES", "DEFAULT_FIFO_DEPTH",
    "FIFO_DEPTHS", "THREADS",
    "WAITB_BE_IDLE", "WAITB_OUTQ_NF", "WAITB_INQ_NE", "WAITB_TICK",
    "HostPlan", "HostTxn", "build_host_plan", "CTRL_WRITES",
    "LOAD_CYCLE", "RUN_CYCLE", "host_actions", "run_model", "trace",
    "build_machine", "ModelHost", "fifo_error", "peek_word", "pop_after_peek",
    "replay_host_log", "group_host_log",
    "StimulusPlan", "PinWave", "build_plan", "INPUT_PINS", "UI_BIT", "UI_MASK",
]
