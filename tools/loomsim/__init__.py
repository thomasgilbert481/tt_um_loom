"""Loom golden model: a cycle-exact Python reference for the Loom core.

``tools/loomsim`` implements ``docs/SEMANTICS.md`` and nothing else.  It is the
reference that RTL simulation, the FPGA and the silicon are all compared
against through the retire record of SEMANTICS section 8, and it is written
independently of ``src/`` (``docs/VERIFICATION.md``, METH-1).

Typical use::

    from tools.loomisa import load
    from tools.loomsim import Machine

    isa = load()
    image = {0: isa.encode("LDI", rd=0, imm=0x41),
             1: isa.encode("HALT")}
    m = Machine(image)
    m.host_set_run(0b0001)
    while not (m.halted & 1):
        record = m.step_cycle()
        if record is not None:
            print(record)

Everything in the model is driven one clock at a time by
:meth:`Machine.step_cycle`.  Host actions (``host_*`` methods) commit at the
edge that ends the cycle they are called in, exactly as a host write over SPI
would.  Pad inputs are set with :meth:`Machine.set_pad_inputs` before each
cycle and reach instructions two cycles later through the synchroniser model.

Only the ``[M1]`` feature set is built.  Bit-engine and data-memory
instructions execute as ``NOP`` with ``BADOP`` set; ``Machine(features={"FIFO"})``
additionally builds ``PUSH``, ``POP`` and ``WAITB`` conditions 1, 2 and 3.
"""

from .alu import (add16, branch_target, ldih16, mask16, neg16, next_pc, not16,
                  par16, reached, rev16, ror16, shl16, shr16, sign_extend,
                  sub16, swap16)
from .machine import (DEFAULT_FIFO_DEPTH, DEFAULT_IMEM_WORDS, LoomsimError,
                      Machine, RunResult, image_from_obj, load_image_file)
from .state import (Commit, CycleTrace, PadState, RetireRecord, ThreadState,
                    SLOT_CLOCKS, THREADS)

__all__ = [
    "Machine", "ThreadState", "RetireRecord", "CycleTrace", "PadState",
    "Commit", "RunResult", "LoomsimError",
    "load_image_file", "image_from_obj",
    "THREADS", "SLOT_CLOCKS", "DEFAULT_IMEM_WORDS", "DEFAULT_FIFO_DEPTH",
    "reached", "add16", "sub16", "shl16", "shr16", "ror16", "not16", "neg16",
    "rev16", "par16", "swap16", "ldih16", "mask16", "sign_extend",
    "branch_target", "next_pc",
]
