"""Host-visible register map of the Loom model (``docs/HOST_PROTOCOL.md`` 0.2).

Addresses and bit layouts of the CTRL, FIFO and DEBUG spaces, the ``BADOP``
and ``CAPS`` bits of ``docs/SEMANTICS.md`` section 5, and the optional
feature names a :class:`~tools.loomsim.Machine` can be built with.  Written
from the two documents only; ``tools/loomhost/protocol.py`` has its own copy
for the transport side, and the two are compared by the test suite through
``ModelTransport``, never shared.
"""

from __future__ import annotations

from typing import Dict

#: ``CTRL.ID``: "LM".
ID_VALUE = 0x4C4D
#: ``CTRL.VERSION`` = ``{major, minor}`` of the host protocol the model
#: implements (HOST_PROTOCOL 0.2).  The document does not say what the
#: register holds; see ``docs/spec-questions/loomsim.md`` (M2 item 1).
DEFAULT_VERSION = 0x0002

# ------------------------------------------------------------------ features
#: Optional features a Machine can be built with.  ``FIFO`` (SEMANTICS 6.7:
#: PUSH/POP/WAITB and the host FIFO space), ``BE`` (6.9: the bit engine in
#: manual mode), ``SETPD`` (6.10: deadline-latched SETP).  ``DMEM`` and
#: ``BOOTROM`` only set their CAPS bits; the model has no data memory, so
#: ``LD``/``ST`` in a ``DMEM`` build raise :class:`LoomsimError`.
FEATURES = frozenset({"FIFO", "BE", "SETPD", "DMEM", "BOOTROM"})
#: The features the command line offers.
CLI_FEATURES = ("FIFO", "BE", "SETPD")
#: Legal ``FIFO_DEPTH`` build parameters (SEMANTICS 6.7): the counts must fit
#: the 4-bit fields of the host status word.
FIFO_DEPTHS = (2, 4, 8)

# ---------------------------------------------------------------------- BADOP
BADOP_THREADS = 0x000F       # bit t: thread t ran a reserved or unbuilt instruction
BADOP_FIFO = 1 << 14         # host FIFO error (SEMANTICS 6.7)
BADOP_ACCESS = 1 << 15       # host access error (IMEM while not quiet)
BADOP_MASK = BADOP_THREADS | BADOP_FIFO | BADOP_ACCESS

# ----------------------------------------------------------------------- CAPS
CAPS_FIFO = 1 << 3
CAPS_BE = 1 << 4
CAPS_DMEM = 1 << 5
CAPS_BOOTROM = 1 << 6
CAPS_SETPD = 1 << 7
CAPS_BE_AUTO = 1 << 8        # M3; never set by this model

# ----------------------------------------------------------------- CTRL space
CTRL: Dict[str, int] = {
    "ID": 0x00,
    "VERSION": 0x01,
    "RUN": 0x02,
    "HALTED": 0x03,
    "RESET": 0x04,
    "RESET_PC0": 0x08,
    "RESET_PC1": 0x09,
    "RESET_PC2": 0x0A,
    "RESET_PC3": 0x0B,
    "IRQ_EN": 0x10,
    "IRQ_STAT": 0x11,
    "IRQ_STAT2": 0x12,
    "SFLAGS": 0x13,
    "SFLAGS_CLR": 0x14,
    "OD_MASK": 0x15,
    "PIN_OUT": 0x16,
    "PIN_OE": 0x17,
    "PIN_IN": 0x18,
    "CAPS": 0x19,
    "BADOP": 0x1A,
    "SWIRQ": 0x1B,
    "IRQ_EN2": 0x1C,
}
CTRL_NAMES = {addr: name for name, addr in CTRL.items()}
#: CTRL registers the host can only read.
CTRL_READ_ONLY = frozenset({"ID", "VERSION", "HALTED", "IRQ_STAT", "IRQ_STAT2",
                            "PIN_IN", "CAPS"})
#: CTRL registers the host can only write (they read 0).
CTRL_WRITE_ONLY = frozenset({"RESET", "SFLAGS_CLR"})
IRQ_EN2_MASK = 0x000F        # HOST_PROTOCOL: IRQ_EN2 keeps bits 3:0

# ----------------------------------------------------------------- FIFO space
FIFO_DATA = 0x0000           # + t: write pushes INQ[t], read pops OUTQ[t]
FIFO_STATUS = 0x0100         # + t: status word

# ---------------------------------------------------------------- DEBUG space
#: DEBUG-space register numbers (the low byte of the address; the thread is
#: in bits 9:8).  0x10..0x1F are the thread's CSRs 0x00..0x0F by number.
DEBUG: Dict[str, int] = {
    "r0": 0x00, "r1": 0x01, "r2": 0x02, "r3": 0x03,
    "r4": 0x04, "r5": 0x05, "r6": 0x06, "r7": 0x07,
    "PC": 0x08,
    "FLAGS": 0x09,
    "TD": 0x0A,
    "NOW": 0x0B,
    "SR": 0x0C,
    "CNT": 0x0D,
    "CRC": 0x0E,
    "RS0": 0x0F,
    "STEPS": 0x20,
    "RS1_DEPTH": 0x21,
    "WAIT_ACTIVE": 0x22,
    "DT": 0x23,
    "TICK_SEEN": 0x24,
    "LAT": 0x25,
    "FIFO_CNT": 0x26,
}
DEBUG_NAMES = {addr: name for name, addr in DEBUG.items()}
DEBUG_CSR_BASE = 0x10
DEBUG_CSR_COUNT = 16
#: The highest register number HOST_PROTOCOL defines; above it reads 0.
DEBUG_LAST = 0x26


def pack_fifo_status(inq: int, outq: int, depth: int) -> int:
    """FIFO status word: ``[11:8] OUTQ_CNT, [7:4] INQ_CNT, [3] OUTQ_EMPTY,
    [2] OUTQ_FULL, [1] INQ_EMPTY, [0] INQ_FULL`` (HOST_PROTOCOL space 3)."""
    return (((outq & 0xF) << 8) | ((inq & 0xF) << 4)
            | (int(outq == 0) << 3) | (int(outq >= depth) << 2)
            | (int(inq == 0) << 1) | int(inq >= depth))


def pack_irq_stat(sflags: int, inq_not_full: int, outq_not_empty: int) -> int:
    """``IRQ_STAT = {SFLAGS[7:0], INQ_NOT_FULL[3:0], OUTQ_NOT_EMPTY[3:0]}``."""
    return (((sflags & 0xFF) << 8) | ((inq_not_full & 0xF) << 4)
            | (outq_not_empty & 0xF))


def pack_rs1_depth(rs1: int, depth: int) -> int:
    """Debug 0x21: ``{4'b0, DEPTH[1:0], RS1[9:0]}``."""
    return ((depth & 3) << 10) | (rs1 & 0x3FF)


def pack_fifo_counts(inq: int, outq: int) -> int:
    """Debug 0x26: ``{INQ_CNT, OUTQ_CNT}`` as ``{byte, byte}``."""
    return ((inq & 0xFF) << 8) | (outq & 0xFF)


def pack_lat(valid: int, value: int, pin: int) -> int:
    """Debug 0x25: ``{LAT_VALID, LAT_VAL, LAT_PIN[4:0]}`` in bits 6:0."""
    return ((valid & 1) << 6) | ((value & 1) << 5) | (pin & 0x1F)


def unpack_lat(word: int):
    """``(valid, value, pin)`` from a debug 0x25 word."""
    return (word >> 6) & 1, (word >> 5) & 1, word & 0x1F
