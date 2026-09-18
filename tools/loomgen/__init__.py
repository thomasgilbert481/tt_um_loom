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
    print(prog.disassembly())

Everything is derived from the arguments: the same call gives the same image
and the same stimulus, on any machine and in any process. ``prog.to_obj()``
and :meth:`GeneratedProgram.from_obj` round-trip a program through JSON, which
is how a failing seed is written to ``test/cosim_failures/`` and replayed.

Command line::

    python -m tools.loomgen --seed 3 --threads 4 --profile pins --listing
    python -m tools.loomgen --replay test/cosim_failures/seed_3.json --trace 1
"""

from .generator import (AVOID_FLAGS, M1_CSR_NAMES, M1_MNEMONICS, PROFILES,
                        THREADS, UNBUILT_CSR_NAMES, UNBUILT_MNEMONICS,
                        GeneratedProgram, LoomgenError, check_program,
                        generate, static_target)
from .runner import LOAD_CYCLE, RUN_CYCLE, host_actions, run_model, trace
from .stimulus import (INPUT_PINS, UI_BIT, UI_MASK, PinWave, StimulusPlan,
                       build_plan)

__all__ = [
    "generate", "GeneratedProgram", "LoomgenError", "check_program",
    "static_target",
    "PROFILES", "AVOID_FLAGS", "M1_MNEMONICS", "UNBUILT_MNEMONICS",
    "M1_CSR_NAMES", "UNBUILT_CSR_NAMES", "THREADS",
    "LOAD_CYCLE", "RUN_CYCLE", "host_actions", "run_model", "trace",
    "StimulusPlan", "PinWave", "build_plan", "INPUT_PINS", "UI_BIT", "UI_MASK",
]
