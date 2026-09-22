"""Encode and decode Loom host-port transactions (``docs/HOST_PROTOCOL.md``).

Pure functions and constants only: no transport, no model. Everything that
knows a byte layout of the SPI host protocol lives here, so the host library,
the model transport and the tests share one definition.

A transaction is one chip-select low period::

    byte 0      CMD  = {RW, SPACE[2:0], 0000}     RW: 1 write, 0 read
    byte 1..2   ADDR = 16 bits, MSB first
    write:      DATA words, 16 bits MSB first, ADDR increments per word
    read:       one dummy byte, then DATA words on MISO, ADDR increments

In the FIFO space the address does not increment: every word of a multi-word
transaction goes to (or comes from) the same FIFO. Elsewhere the address
wraps at 16 bits ("the address wraps within the space").
"""

from __future__ import annotations

import dataclasses
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

# --------------------------------------------------------------------- spaces
SPACE_CTRL = 0
SPACE_IMEM = 1
SPACE_DMEM = 2
SPACE_FIFO = 3
SPACE_DEBUG = 4
SPACE_STEP = 5

SPACES = {"CTRL": SPACE_CTRL, "IMEM": SPACE_IMEM, "DMEM": SPACE_DMEM,
          "FIFO": SPACE_FIFO, "DEBUG": SPACE_DEBUG, "STEP": SPACE_STEP}
SPACE_NAMES = {value: name for name, value in SPACES.items()}

HEADER_BYTES = 3            # CMD + ADDR
READ_HEADER_BYTES = 4       # CMD + ADDR + dummy
WORD_MASK = 0xFFFF
THREADS = 4

# ----------------------------------------------------------- CTRL registers
CTRL = {
    "ID": 0x0000,
    "VERSION": 0x0001,
    "RUN": 0x0002,
    "HALTED": 0x0003,
    "RESET": 0x0004,
    "RESET_PC0": 0x0008,
    "RESET_PC1": 0x0009,
    "RESET_PC2": 0x000A,
    "RESET_PC3": 0x000B,
    "IRQ_EN": 0x0010,
    "IRQ_STAT": 0x0011,
    "IRQ_STAT2": 0x0012,
    "SFLAGS": 0x0013,
    "SFLAGS_CLR": 0x0014,
    "OD_MASK": 0x0015,
    "PIN_OUT": 0x0016,
    "PIN_OE": 0x0017,
    "PIN_IN": 0x0018,
    "CAPS": 0x0019,
    "BADOP": 0x001A,
    "SWIRQ": 0x001B,
    "IRQ_EN2": 0x001C,
}
CTRL_NAMES = {value: name for name, value in CTRL.items()}
ID_VALUE = 0x4C4D           # "LM"

#: BADOP bits besides the per-thread bits 3:0 (SEMANTICS 5).
BADOP_FIFO = 1 << 14        # host FIFO error (SEMANTICS 6.7)
BADOP_ACCESS = 1 << 15      # host access error (IMEM while running)

# ---------------------------------------------------------- DEBUG registers
DEBUG = {
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
DEBUG_NAMES = {value: name for name, value in DEBUG.items()}
DEBUG_CSR_BASE = 0x10       # 0x10..0x1F are CSR 0x00..0x0F of the thread
DEBUG_CSR_COUNT = 16
#: Debug registers the host may never write (HOST_PROTOCOL: NOW is read only).
DEBUG_READ_ONLY = frozenset({DEBUG["NOW"], DEBUG["FIFO_CNT"]})

# ------------------------------------------------------------ FIFO space
FIFO_DATA = 0x0000          # + thread: write pushes INQ, read pops OUTQ
FIFO_STATUS = 0x0100        # + thread: read the status word


def reg_name_key(name: str) -> str:
    """Canonical spelling of a debug register name (``R3`` -> ``r3``)."""
    text = str(name).strip()
    if len(text) == 2 and text[0] in "rR" and text[1].isdigit():
        return "r" + text[1]
    return text.upper()


# ------------------------------------------------------------------ bytes
def cmd_byte(space: int, write: bool) -> int:
    """The CMD byte ``{RW, SPACE[2:0], 0000}``."""
    if not 0 <= space <= 7:
        raise ValueError("space %r outside 0..7" % (space,))
    return (0x80 if write else 0x00) | (space << 4)


def parse_cmd(byte: int) -> Tuple[bool, int]:
    """``(write, space)`` from a CMD byte. ``CMD[3:0]`` is reserved, ignored."""
    byte &= 0xFF
    return bool(byte & 0x80), (byte >> 4) & 0x7


def debug_addr(thread: int, reg: int) -> int:
    """DEBUG-space address ``{thread[9:8], reg[7:0]}``."""
    _check_thread(thread)
    if not 0 <= reg <= 0xFF:
        raise ValueError("debug register %r outside 0..0xFF" % (reg,))
    return (thread << 8) | reg


def fifo_addr(thread: int, status: bool = False) -> int:
    """FIFO-space address of a thread's data port or its status word."""
    _check_thread(thread)
    return (FIFO_STATUS if status else FIFO_DATA) + thread


def step_addr(thread: int) -> int:
    _check_thread(thread)
    return thread


def next_addr(space: int, addr: int) -> int:
    """The address of the following word of a multi-word transaction."""
    if space == SPACE_FIFO:
        return addr & WORD_MASK
    return (addr + 1) & WORD_MASK


def word_addresses(space: int, addr: int, count: int) -> List[int]:
    out = []
    for _ in range(count):
        out.append(addr & WORD_MASK)
        addr = next_addr(space, addr)
    return out


def _check_thread(thread: int) -> None:
    if not isinstance(thread, int) or not 0 <= thread < THREADS:
        raise ValueError("thread %r outside 0..3" % (thread,))


def _check_word(word: int) -> int:
    if not isinstance(word, int) or not 0 <= word <= WORD_MASK:
        raise ValueError("word %r is not a 16-bit value" % (word,))
    return word


def encode_write(space: int, addr: int, words: Iterable[int]) -> bytes:
    """MOSI bytes of a write transaction."""
    if not 0 <= addr <= WORD_MASK:
        raise ValueError("address %r outside 0..0xFFFF" % (addr,))
    out = bytearray([cmd_byte(space, True), addr >> 8, addr & 0xFF])
    for word in words:
        word = _check_word(word)
        out += bytes([word >> 8, word & 0xFF])
    return bytes(out)


def encode_read(space: int, addr: int, count: int) -> bytes:
    """MOSI bytes of a read of ``count`` words (dummy and data bytes are 0)."""
    if not 0 <= addr <= WORD_MASK:
        raise ValueError("address %r outside 0..0xFFFF" % (addr,))
    if count < 0:
        raise ValueError("negative word count")
    return bytes([cmd_byte(space, False), addr >> 8, addr & 0xFF, 0x00]) \
        + bytes(2 * count)


def decode_read_response(rx: bytes, count: int) -> List[int]:
    """The data words of a read transaction's MISO bytes."""
    need = READ_HEADER_BYTES + 2 * count
    if len(rx) < need:
        raise ValueError("read response is %d bytes, expected %d" % (len(rx), need))
    body = rx[READ_HEADER_BYTES:need]
    return [(body[2 * i] << 8) | body[2 * i + 1] for i in range(count)]


# ------------------------------------------------ the chip's view of a frame
@dataclasses.dataclass
class Transaction:
    """One decoded chip-select frame, as the chip sees it on MOSI.

    ``words`` holds the written words (writes) and is empty for reads;
    ``count`` is the number of complete words transferred either way.
    ``partial`` is true when the frame ended in the middle of a word.
    """

    write: bool
    space: int
    addr: int
    words: List[int] = dataclasses.field(default_factory=list)
    count: int = 0
    partial: bool = False
    cmd: int = 0

    @property
    def space_name(self) -> str:
        return SPACE_NAMES.get(self.space, "SPACE%d" % self.space)

    @property
    def addresses(self) -> List[int]:
        return word_addresses(self.space, self.addr, self.count)


def decode_transaction(tx: bytes) -> Optional[Transaction]:
    """Decode MOSI bytes; ``None`` if the frame ended before ADDR completed."""
    if len(tx) < HEADER_BYTES:
        return None
    write, space = parse_cmd(tx[0])
    addr = (tx[1] << 8) | tx[2]
    if write:
        body = tx[HEADER_BYTES:]
        words = [(body[2 * i] << 8) | body[2 * i + 1] for i in range(len(body) // 2)]
        return Transaction(True, space, addr, words, len(words), len(body) % 2 == 1, tx[0])
    body = tx[READ_HEADER_BYTES:]
    return Transaction(False, space, addr, [], len(body) // 2,
                       len(tx) > READ_HEADER_BYTES and len(body) % 2 == 1, tx[0])


@dataclasses.dataclass
class Event:
    """A point inside a frame at which the chip acts.

    ``after_byte`` is the number of complete bytes before the event: a write
    word commits when its second byte completes, and a read word is fetched
    when the byte before its first data byte completes (the dummy byte for
    the first word). ``kind`` is ``"write"`` or ``"read"``; ``index`` is the
    word's position in the frame.
    """

    after_byte: int
    kind: str
    space: int
    addr: int
    index: int
    word: int = 0


def chip_events(tx: bytes) -> List[Event]:
    """Every word access of a frame, in order, with the byte it happens after.

    A word cut in half is not a write (no partial writes); a read word whose
    first byte started is fetched, so a FIFO pop there consumes the entry.
    """
    trans = decode_transaction(tx)
    if trans is None:
        return []
    events = []
    addr = trans.addr
    if trans.write:
        for index, word in enumerate(trans.words):
            events.append(Event(HEADER_BYTES + 2 * index + 2, "write",
                                trans.space, addr, index, word))
            addr = next_addr(trans.space, addr)
    else:
        started = (len(tx) - READ_HEADER_BYTES + 1) // 2 if len(tx) > READ_HEADER_BYTES else 0
        for index in range(started):
            events.append(Event(READ_HEADER_BYTES + 2 * index, "read",
                                trans.space, addr, index))
            addr = next_addr(trans.space, addr)
    return events


# -------------------------------------------------------- packed registers
def pack_fifo_status(inq: int, outq: int, depth: int) -> int:
    """FIFO status ``{OUTQ_COUNT[3:0], INQ_COUNT[3:0], OUTQ_EMPTY, OUTQ_FULL,
    INQ_EMPTY, INQ_FULL}`` with INQ_FULL in bit 0 (HOST_PROTOCOL space 3)."""
    return (((outq & 0xF) << 8) | ((inq & 0xF) << 4)
            | (int(outq == 0) << 3) | (int(outq >= depth) << 2)
            | (int(inq == 0) << 1) | int(inq >= depth))


def unpack_fifo_status(word: int) -> Dict[str, int]:
    return {
        "outq": (word >> 8) & 0xF,
        "inq": (word >> 4) & 0xF,
        "outq_empty": (word >> 3) & 1,
        "outq_full": (word >> 2) & 1,
        "inq_empty": (word >> 1) & 1,
        "inq_full": word & 1,
    }


def decode_caps(word: int) -> Dict[str, int]:
    """``CTRL.CAPS`` in the layout of SEMANTICS section 5."""
    imem_log2 = (word >> 12) & 0xF
    fifos = (word >> 3) & 1
    depth_log2 = word & 0x7
    return {
        "fifo_depth_log2": depth_log2,
        "fifo_depth": (1 << depth_log2) if fifos else 0,
        "fifos": fifos,
        "bit_engine": (word >> 4) & 1,
        "dmem": (word >> 5) & 1,
        "boot_rom": (word >> 6) & 1,
        "setp_deadline": (word >> 7) & 1,
        "bit_engine_auto": (word >> 8) & 1,
        "bit_engine_enc": (word >> 9) & 1,     # slice A: encoders, stuffing, DIFF
        "imem_words_log2": imem_log2,
        "imem_words": 1 << imem_log2,
        "raw": word & WORD_MASK,
    }


def encode_caps(imem_words: int, fifo_depth: int = 0, bit_engine: bool = False,
                dmem: bool = False, boot_rom: bool = False,
                setp_deadline: bool = False, bit_engine_auto: bool = False,
                bit_engine_enc: bool = False) -> int:
    """Inverse of :func:`decode_caps` (``fifo_depth`` 0 means no FIFOs)."""
    word = ((imem_words.bit_length() - 1) & 0xF) << 12
    if fifo_depth:
        word |= ((fifo_depth.bit_length() - 1) & 0x7) | (1 << 3)
    word |= (int(bit_engine) << 4) | (int(dmem) << 5) | (int(boot_rom) << 6)
    word |= (int(setp_deadline) << 7) | (int(bit_engine_auto) << 8)
    word |= int(bit_engine_enc) << 9
    return word


def pack_irq_stat(sflags: int, inq_not_full: int, outq_not_empty: int) -> int:
    """``IRQ_STAT = {SFLAGS[7:0], INQ_NOT_FULL[3:0], OUTQ_NOT_EMPTY[3:0]}``."""
    return ((sflags & 0xFF) << 8) | ((inq_not_full & 0xF) << 4) | (outq_not_empty & 0xF)


def unpack_irq_stat(word: int) -> Dict[str, int]:
    return {"sflags": (word >> 8) & 0xFF, "inq_not_full": (word >> 4) & 0xF,
            "outq_not_empty": word & 0xF}


def pack_rs1_depth(rs1: int, depth: int) -> int:
    """Debug 0x21: ``{4'b0, DEPTH[1:0], RS1[9:0]}``."""
    return ((depth & 3) << 10) | (rs1 & 0x3FF)


def unpack_rs1_depth(word: int) -> Tuple[int, int]:
    return word & 0x3FF, (word >> 10) & 3


def pack_lat(valid: int, value: int, pin: int) -> int:
    """Debug 0x25: ``{LAT_VALID, LAT_VAL, LAT_PIN[4:0]}`` in bits 6:0."""
    return ((valid & 1) << 6) | ((value & 1) << 5) | (pin & 0x1F)


def unpack_lat(word: int) -> Dict[str, int]:
    return {"valid": (word >> 6) & 1, "value": (word >> 5) & 1, "pin": word & 0x1F}


def pack_fifo_counts(inq: int, outq: int) -> int:
    """Debug 0x26: ``{INQ_CNT, OUTQ_CNT}`` as ``{byte, byte}``."""
    return ((inq & 0xFF) << 8) | (outq & 0xFF)


def unpack_fifo_counts(word: int) -> Tuple[int, int]:
    return (word >> 8) & 0xFF, word & 0xFF


def hexdump(data: Sequence[int]) -> str:
    return " ".join("%02X" % b for b in data)
