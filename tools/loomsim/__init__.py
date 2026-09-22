"""Loom golden model: a cycle-exact Python reference for the Loom core.

``tools/loomsim`` implements ``docs/SEMANTICS.md`` (and the host registers of
``docs/HOST_PROTOCOL.md``) and nothing else.  It is the reference that RTL
simulation, the FPGA and the silicon are all compared against through the
retire record of SEMANTICS section 8, and it is written independently of
``src/`` (``docs/VERIFICATION.md``, METH-1).

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

The host port is modelled register by register, not bit by bit: the CTRL space
(:meth:`Machine.host_read_ctrl` / :meth:`Machine.host_write_ctrl`: ``ID``,
``VERSION``, ``RUN``, ``RESET``, the interrupt registers ``IRQ_EN``,
``IRQ_STAT``, ``IRQ_STAT2``, ``SWIRQ``, ``IRQ_EN2``, ``BADOP`` ...), the DEBUG
space by name or number (:meth:`Machine.host_read_debug` /
:meth:`Machine.host_write_debug`, writes only while the thread is halted in
the sense of SEMANTICS 7), IMEM, STEP and the FIFO space.  The registered
``HOST_IRQ`` output (pad ``uo_out[6]``, SEMANTICS 6.8) is
:attr:`Machine.host_irq` and :attr:`CycleTrace.host_irq`; ``uo_out`` stays
``PIN_OUT[13:8]``.

Optional features are chosen at construction, ``Machine(features=...)``:

* ``"FIFO"``: ``PUSH``, ``POP``, ``WAITB`` and the host FIFO space
  (SEMANTICS 6.7), ``fifo_depth`` 2, 4 or 8.  The model's host pop is atomic;
  :meth:`Machine.host_fifo_peek` lets a transport build the port's
  peek-then-pop split on top of it.
* ``"BE"``: the bit engine in manual mode (6.9): ``SHO``, ``SHI``, ``LDSR``,
  ``STSR``, ``CRCI``, ``STCRC`` and the ``SR``, ``CNT``, ``CRC``, ``BE_*`` and
  ``CRC_*`` CSRs; with ``"FIFO"`` also ``WAITB 0``, which is always true
  until auto mode exists.
* ``"BEENC"`` (M3 slice A, 6.9.1, D-026; needs ``"BE"``): the ``ENC``,
  ``STUFF`` and ``DIFF`` fields of ``BE_CFG`` and the per-thread encoder
  state at debug 0x27, so ``SHO`` and ``SHI`` do NRZI and Manchester coding,
  USB and CAN stuffing and destuffing (with the ``T`` flag on a violation)
  and the differential output.  ``CTRL.VERSION`` reads 3 in such a build,
  unless slice B is in it too.
* ``"SETPD"``: the deadline-latched ``SETP pin, v, D`` (6.10).
* ``"DMEM"`` (M3 slice B, 6.11, D-027): ``LD`` and ``ST``, which read and
  write the instruction memory as data in two slots of their own thread, the
  ``MEM_PEND``, ``MEM_LD`` and ``MEM_RD`` state and debug 0x28.
  ``CTRL.VERSION`` reads 4 in such a build.

``Machine(features=())`` is the M1 build: every instruction of an unbuilt
feature is a ``NOP`` that sets ``BADOP``, CSRs of unbuilt features read 0, and
``SETP ... D`` is an ordinary ``SETP``.
"""

from .alu import (add16, be_count, be_out_bit, be_run_step, be_shift_in,
                  be_shift_out, be_stuff_value, branch_target, crc_step,
                  ldih16, mask16, neg16, next_pc, not16, par16, reached,
                  rev16, ror16, shl16, shr16, sign_extend, sub16, swap16)
from .hostmap import (BADOP_ACCESS, BADOP_FIFO, CLI_FEATURES, DEFAULT_VERSION,
                      DMEM_VERSION, ENC_VERSION, FEATURES, FIFO_DEPTHS,
                      ID_VALUE)
from .machine import (DEBUG_REGS, DEFAULT_FIFO_DEPTH, DEFAULT_IMEM_WORDS,
                      LoomsimError, Machine, RunResult, image_from_obj,
                      load_image_file)
from .state import (Commit, CycleTrace, PadState, RetireRecord, ThreadState,
                    SLOT_CLOCKS, THREADS)

__all__ = [
    "Machine", "ThreadState", "RetireRecord", "CycleTrace", "PadState",
    "Commit", "RunResult", "LoomsimError",
    "load_image_file", "image_from_obj",
    "THREADS", "SLOT_CLOCKS", "DEFAULT_IMEM_WORDS", "DEFAULT_FIFO_DEPTH",
    "DEBUG_REGS", "FEATURES", "CLI_FEATURES", "FIFO_DEPTHS", "ID_VALUE",
    "DEFAULT_VERSION", "ENC_VERSION", "DMEM_VERSION",
    "BADOP_FIFO", "BADOP_ACCESS",
    "reached", "add16", "sub16", "shl16", "shr16", "ror16", "not16", "neg16",
    "rev16", "par16", "swap16", "ldih16", "mask16", "sign_extend",
    "branch_target", "next_pc",
    "be_out_bit", "be_shift_out", "be_shift_in", "be_count", "crc_step",
    "be_run_step", "be_stuff_value",
]
