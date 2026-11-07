"""MDB/ICP reference models: nine-bit characters, blocks and their CHK, a
line transmitter and receiver, a timing checker, a reference controller
(VMC) and a reference cashless device, and the board wire.

Written from the specification, not from the firmware, for
``firmware/mdb_ctrl.loom`` and ``firmware/mdb_cashless.loom`` (D-035):
"Multi-Drop Bus / Internal Communication Protocol", MDB/ICP Version 4.3
(NAMA, July 2019; the cashless section dated September 2020), read on
2026-11-07 from NAMA's own copy. The brief named Version 4.2 (2011); no
copy of it could be fetched (QUESTIONS.md), and the 4.3 revision notes list
no change to sections 2 and 3.

=================================  ==========================================
figure                             source
=================================  ==========================================
9600 baud NRZ; 1 start, 8 data     2.1 "Byte Format"; 3.2: 9600 +1 %/-2 %
LSB first, 1 mode, 1 stop bit
master to peripheral: mode 1 on    2.1 "Mode Bit: Master-to-Peripheral";
the ADDRESS byte only; DATA bytes  ADDRESS bytes are read by every
are read only by the addressed     peripheral
peripheral
peripheral to master: mode 1 on    2.1 "Mode Bit: Peripheral-to-Master";
the last byte only (CHK, ACK or    2.2 (page 2-3)
NAK)
block: address byte, data, CHK;    2.2 "Block Format", both directions
at most 36 bytes
address in bits 7:3, the command   2.2, 2.3; cashless device #1 is 10H
in bits 2:0
CHK: the sum of the address and    2.2 "CHK Byte" (and its 12CH -> 2CH
data bytes, carry ignored; none    example)
on ACK, NAK, RET
ACK 00H, RET AAH (VMC only),       2.2 "Response Codes"
NAK FFH; t-response silence is a
NAK
the VMC answers a data block       2.2 (page 2-2 and 2-3), examples 2.5 B,
with ACK, NAK or RET; a NAKed or   D; a peripheral repeats a data block the
unanswered data block is repeated  master NAKs or does not answer
on silence the master may repeat   2.2, example 2.5 E
the command
t-inter-byte max 1.0 ms,           3.2 "Timing Specifications"
t-response max 5.0 ms, t-break
min 100 ms, t-setup min 200 ms
cashless commands RESET 10H,       7.3 table 1; 7.4.1, 7.4.4, 7.4.5
POLL 12H, VEND 13H; VEND REQUEST
is 13H 00H, price (2), item (2)
JUST RESET 00H (as 00H 00H*),      7.4.4 (the note that tells it from ACK)
VEND APPROVED 05H + amount (2),    7.4.5; multi-byte values high byte first
VEND DENIED 06H                    (2.4.2)
RESET: no data, Inactive state;    7.4.1
the next POLL gets JUST RESET
non-response time 5 s              7.5 (not modelled: the host's)
=================================  ==========================================

The line is the logic side of the bus interface (the optocouplers of
section 4): idle high, a start bit low, as a UART sees it. Section 4's
"active" level of the transmit line (the break) is not used here.

Times are core clocks on the bench (:data:`CLOCK`, 50 MHz). A bit is
``CLOCK / baud`` clocks, a fraction in general; edges a model drives are
placed at ``round(t)`` of exact fractional times. :class:`Timing` scales the
times of 3.2 to a test rate by bit times (5.0 ms is 48 bits at 9600 baud).
"""

from __future__ import annotations

import dataclasses
from collections import deque
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from .bench import Drive, Lines, Model, PinSpec, pad_of

CLOCK = 50e6
BAUD = 9600                          # 2.1, 3.2
CHAR_BITS = 11                       # 2.1
MODE = 0x100                         # the mode bit, as bit 8 of a character
ACK, RET, NAK = 0x00, 0xAA, 0xFF     # 2.2 "Response Codes"
MAX_BLOCK = 36                       # 2.2
T_INTER_BYTE_S = 1.0e-3              # 3.2, max
T_RESPONSE_S = 5.0e-3                # 3.2, max
T_BREAK_S = 100e-3                   # 3.2, min (not generated)
T_SETUP_S = 200e-3                   # 3.2, min
CASHLESS1 = 0x10                     # 2.3
RESET, SETUP, POLL, VEND = 0, 1, 2, 3            # table 1, bits 2:0
VEND_REQUEST = 0x00                  # 7.4.5
JUST_RESET, VEND_APPROVED, VEND_DENIED = 0x00, 0x05, 0x06   # 7.4.4, 7.4.5


# ---------------------------------------------------------------- blocks
def chk(data: Sequence[int]) -> int:
    """2.2: the sum of the bytes, carry ignored."""
    return sum(b & 0xFF for b in data) & 0xFF


def vmc_block(data: Sequence[int]) -> List[int]:
    """A master-to-peripheral block as nine-bit characters: the ADDRESS byte
    with the mode bit, the DATA bytes and the CHK without it (2.1, 2.2)."""
    body = [b & 0xFF for b in data]
    return [body[0] | MODE] + body[1:] + [chk(body)]


def peripheral_block(data: Sequence[int]) -> List[int]:
    """A peripheral's data block: the bytes, then the CHK with the mode bit
    (2.1, 2.2)."""
    body = [b & 0xFF for b in data]
    return body + [chk(body) | MODE]


def single(code: int) -> List[int]:
    """ACK or NAK from a peripheral: one byte, mode bit set, no CHK (2.2)."""
    return [(code & 0xFF) | MODE]


def block_ok(chars: Sequence[int]) -> bool:
    """A data block of two bytes or more whose last byte is its CHK."""
    return len(chars) >= 2 and chk(chars[:-1]) == chars[-1] & 0xFF


def word(value: int) -> List[int]:
    """A 16-bit field, high byte first (2.4.2)."""
    return [(value >> 8) & 0xFF, value & 0xFF]


def vend_request(price: int, item: int, address: int = CASHLESS1) -> List[int]:
    """The bytes of VEND REQUEST (7.4.5), without the CHK."""
    return [address | VEND, VEND_REQUEST] + word(price) + word(item)


def char_levels(char: int) -> List[int]:
    """The eleven line levels of one character (2.1)."""
    return [0] + [(char >> i) & 1 for i in range(9)] + [1]


class Timing:
    """The times of 3.2 for a bit of ``bit`` clocks, scaled by bit times
    from 9600 baud (so at 9600 baud they are the specification's own)."""

    def __init__(self, bit: float) -> None:
        self.bit = bit
        scale = bit * BAUD / CLOCK          # 1.0 at 9600 baud
        self.t_inter_byte = T_INTER_BYTE_S * CLOCK * scale
        self.t_response = T_RESPONSE_S * CLOCK * scale
        self.char = CHAR_BITS * bit


# ------------------------------------------------------------ line models
class LineTx(Model):
    """Drives nine-bit characters onto a pad (a model controller or
    peripheral on a chip input). :meth:`send` queues characters back to
    back from ``start`` (or as soon as the line is free) and returns the
    cycle the first start bit falls; ``gaps`` maps a character index to
    extra idle clocks after it; ``bit`` overrides the bit length (a rate
    error)."""

    def __init__(self, pin: PinSpec, bit: float) -> None:
        self.pad = pad_of(pin)
        self.bit = bit
        self.level = 1
        self.now = 0
        self._changes: deque = deque()
        self._free_at = 0.0

    def send(self, chars: Sequence[int], *, start: Optional[float] = None,
             gaps: Optional[Dict[int, float]] = None,
             bit: Optional[float] = None) -> int:
        bit = self.bit if bit is None else bit
        t = max(self._free_at, float(self.now + 2))
        if start is not None:
            t = max(t, float(start))
        t0 = t
        for i, char in enumerate(chars):
            for k, level in enumerate(char_levels(char)):
                self._changes.append((int(round(t + k * bit)), level))
            t += CHAR_BITS * bit
            if gaps and i in gaps:
                t += gaps[i]
        self._free_at = t
        return int(round(t0))

    @property
    def busy(self) -> bool:
        return bool(self._changes)

    @property
    def free_at(self) -> int:
        """The cycle the last queued character's stop bit ends."""
        return int(round(self._free_at))

    def drive(self, drive: Drive, cycle: int) -> None:
        self.now = cycle
        changes = self._changes
        while changes and changes[0][0] <= cycle:
            self.level = changes.popleft()[1]
        drive.set(self.pad, self.level)


@dataclasses.dataclass
class Char:
    value: int                      # nine bits: bit 8 is the mode bit
    start: int                      # cycle the start bit was first seen low
    end: float                      # start + 11 bits: the stop bit's end
    levels: List[int]               # the eleven sampled levels
    edges: List[int]                # cycles of every edge inside it

    @property
    def mode(self) -> int:
        return self.value >> 8

    @property
    def stop_ok(self) -> bool:
        return self.levels[0] == 0 and self.levels[10] == 1


class LineRx(Model):
    """Decodes and times the characters on a pad (a chip output). A
    character starts at the first cycle the line is seen low after high;
    its eleven bits are sampled at their middles with the nominal ``bit``.
    ``on_char(char, cycle)`` is called as each character completes."""

    def __init__(self, pin: PinSpec, bit: float,
                 on_char: Optional[Callable[[Char, int], None]] = None) -> None:
        self.pad = pad_of(pin)
        self.bit = bit
        self.on_char = on_char
        self.chars: List[Char] = []
        self._prev = 1
        self._cur: Optional[Char] = None

    def observe(self, lines: Lines) -> None:
        c = lines.cycle
        level = lines.get(self.pad)
        cur = self._cur
        if cur is None:
            if level == 0 and self._prev == 1:
                self._cur = Char(0, c, c + CHAR_BITS * self.bit, [], [c])
        else:
            if level != self._prev:
                cur.edges.append(c)
            k = len(cur.levels)
            if c >= cur.start + (k + 0.5) * self.bit:
                cur.levels.append(level)
                if len(cur.levels) == CHAR_BITS:
                    cur.value = sum(b << i for i, b in enumerate(cur.levels[1:10]))
                    self.chars.append(cur)
                    self._cur = None
                    if self.on_char is not None:
                        self.on_char(cur, c)
        self._prev = level

    def since(self, cycle: float) -> List[Char]:
        return [ch for ch in self.chars if ch.start >= cycle]


class Wire(Model):
    """A board wire from a chip output to a chip input, one clock late as
    a registered model is, idle high (the bus interface's level)."""

    def __init__(self, src: PinSpec, dst: PinSpec) -> None:
        self.src, self.dst = pad_of(src), pad_of(dst)
        self.level = 1

    def drive(self, drive: Drive, cycle: int) -> None:
        drive.set(self.dst, self.level)

    def observe(self, lines: Lines) -> None:
        self.level = lines.get(self.src)


# --------------------------------------------------------- timing checker
def split_blocks(chars: Sequence[Char], gap: float) -> List[List[Char]]:
    """Characters split wherever the line was idle ``gap`` clocks or more."""
    out: List[List[Char]] = []
    for ch in chars:
        if out and ch.start - out[-1][-1].end < gap:
            out[-1].append(ch)
        else:
            out.append([ch])
    return out


def violations(block: Sequence[Char], timing: Timing, sender: str,
               edge_tol: float) -> List[str]:
    """What 2.1, 2.2 and 3.2 forbid the sender of ``block`` ("vmc" or
    "peripheral"): a bad stop bit, an edge more than ``edge_tol`` clocks
    off its bit boundary, a gap over t-inter-byte, a block over 36 bytes,
    and the mode bit on the wrong bytes."""
    bad = []
    bit = timing.bit
    for i, ch in enumerate(block):
        if not ch.stop_ok:
            bad.append("char %d: start or stop bit wrong" % i)
        for e in ch.edges:
            off = (e - ch.start) / bit
            err = abs(off - round(off)) * bit
            if err > edge_tol:
                bad.append("char %d: edge at +%d is %.1f clocks off" % (i, e - ch.start, err))
        if i and ch.start - block[i - 1].end > timing.t_inter_byte:
            bad.append("char %d: gap %.0f over t-inter-byte %.0f"
                       % (i, ch.start - block[i - 1].end, timing.t_inter_byte))
    if len(block) > MAX_BLOCK:
        bad.append("block of %d bytes" % len(block))
    modes = [ch.mode for ch in block]
    want = ([1] + [0] * (len(block) - 1)) if sender == "vmc" else ([0] * (len(block) - 1) + [1])
    if sender == "vmc" and len(block) == 1:
        want = [0]                  # the VMC's ACK / NAK / RET: a DATA byte
    if modes != want:
        bad.append("mode bits %s, want %s" % (modes, want))
    return bad


def response_delay(request_end: float, answer: Sequence[Char]) -> float:
    """Clocks from the end of a block's last stop bit to the answer's first
    start bit; 3.2 wants at most t-response."""
    return answer[0].start - request_end


# ---------------------------------------------------- reference cashless
class ModelCashless(Model):
    """A reference cashless device on a chip's controller pins: a
    :class:`LineRx` on the controller's TX (``rx``), a :class:`LineTx` on
    its RX (``tx``). Unlike the firmware it ends a block by its command's
    length (table 1: RESET and POLL 2 bytes, VEND REQUEST 7), else by a
    silence of t-inter-byte. It answers ``delay`` clocks after the block:

    * a bad CHK: NAK (2.2 allows silence too; NAK is what the brief tests);
    * RESET: ACK, and JUST RESET becomes pending (7.4.1);
    * POLL: the pending data block (until the VMC ACKs it, 2.2), else ACK;
    * VEND REQUEST: ACK, ``decide(price, item)`` (an amount, or None for
      denied) and the result pending for the next POLL (7.3).

    Faults: ``silent`` blocks are not answered at all; ``corrupt`` data
    blocks go out with their CHK inverted.
    """

    def __init__(self, rx_pin: PinSpec, tx_pin: PinSpec, bit: float, *,
                 address: int = CASHLESS1, delay: Optional[float] = None,
                 decide: Callable[[int, int], Optional[int]] = lambda p, n: p) -> None:
        self.timing = Timing(bit)
        self.rx = LineRx(rx_pin, bit, on_char=self._char)
        self.tx = LineTx(tx_pin, bit)
        self.address = address
        self.delay = 2 * bit if delay is None else delay
        self.decide = decide
        self.silent = 0
        self.corrupt = 0
        self.pending: Optional[List[int]] = peripheral_block([JUST_RESET])
        self.blocks: List[List[int]] = []        # the blocks addressed to it
        self.vends: List[Tuple[int, int]] = []
        self.acked: List[List[int]] = []         # data blocks the VMC ACKed
        self._block: Optional[List[int]] = None
        self._sent_at: Optional[float] = None    # end of a data block sent
        self._now = 0

    def _expected(self, block: Sequence[int]) -> Optional[int]:
        cmd = block[0] & 7
        if cmd in (RESET, POLL):
            return 2
        if cmd == VEND and len(block) >= 2 and block[1] & 0xFF == VEND_REQUEST:
            return 7
        return None

    def _char(self, ch: Char, cycle: int) -> None:
        v = ch.value
        if self._sent_at is not None:           # the VMC's answer to our data
            if ch.start <= self._sent_at + self.timing.t_response and v == ACK:
                self.acked.append(self.pending)
                self.pending = None
            self._sent_at = None
            if not v & MODE:
                return
        if v & MODE:
            self._block = [v] if (v & 0xF8) == self.address else None
        elif self._block is not None:
            self._block.append(v)
        b = self._block
        if b is not None and self._expected(b) is not None and len(b) == self._expected(b):
            self._block = None
            self._handle(b, cycle)

    def drive(self, drive: Drive, cycle: int) -> None:
        self._now = cycle
        b = self._block
        if (b is not None and self.rx._cur is None and self.rx.chars
                and cycle > self.rx.chars[-1].end + self.timing.t_inter_byte):
            self._block = None                  # a command of unknown length
            self._handle(b, cycle)
        self.tx.drive(drive, cycle)

    def observe(self, lines: Lines) -> None:
        self.rx.observe(lines)

    def _handle(self, block: List[int], cycle: int) -> None:
        self.blocks.append(block)
        if self.silent:
            self.silent -= 1
            return
        start = cycle + self.delay
        if not block_ok(block):
            self.tx.send(single(NAK), start=start)
            return
        cmd = block[0] & 7
        answer = single(ACK)
        if cmd == RESET:
            self.pending = peripheral_block([JUST_RESET])
        elif cmd == POLL and self.pending is not None:
            answer = list(self.pending)
            if self.corrupt:
                self.corrupt -= 1
                answer[-1] ^= 0xFF
        elif cmd == VEND and block[1] == VEND_REQUEST and len(block) == 7:
            price = (block[2] << 8) | block[3]
            item = (block[4] << 8) | block[5]
            self.vends.append((price, item))
            amount = self.decide(price, item)
            self.pending = (peripheral_block([VEND_DENIED]) if amount is None else
                            peripheral_block([VEND_APPROVED] + word(amount)))
        self.tx.send(answer, start=start)
        if len(answer) > 1:
            self._sent_at = self.tx.free_at


# -------------------------------------------------- reference controller
@dataclasses.dataclass
class Exchange:
    data: List[int]                 # the block's bytes without the CHK
    attempts: int = 0
    answer: List[int] = dataclasses.field(default_factory=list)  # last attempt
    outcome: str = ""               # "ack", "data", "nak", "chk", "timeout", "ferr"
    sent_end: List[float] = dataclasses.field(default_factory=list)  # per attempt
    answer_start: List[int] = dataclasses.field(default_factory=list)
    done: bool = False


class ModelController(Model):
    """A reference VMC on a chip's peripheral pins: a :class:`LineTx` on its
    RX (``tx``) and a :class:`LineRx` on its TX (``rx``). :meth:`queue`
    adds a block; blocks go one at a time. After each it waits t-response
    for a start bit (2.2, 3.2), reads to the byte with the mode bit, ACKs
    a good data block, NAKs a bad one, and repeats the block after a NAK, a
    bad CHK or a silence, ``tries`` attempts in all (2.2, 2.5 D and E).
    ``corrupt`` blocks go out with their CHK inverted, and ``withhold``
    good data blocks get no ACK (silence, which 2.2 makes a NAK); both are
    error injection."""

    def __init__(self, tx_pin: PinSpec, rx_pin: PinSpec, bit: float, *,
                 tries: int = 3, timeout: Optional[float] = None) -> None:
        self.timing = Timing(bit)
        self.tx = LineTx(tx_pin, bit)
        self.rx = LineRx(rx_pin, bit, on_char=self._char)
        self.tries = tries
        self.timeout = self.timing.t_response if timeout is None else timeout
        self.corrupt = 0
        self.withhold = 0
        self.exchanges: List[Exchange] = []
        self._queue: deque = deque()
        self._cur: Optional[Exchange] = None
        self._got: List[int] = []
        self._now = 0

    def queue(self, data: Sequence[int]) -> Exchange:
        ex = Exchange(list(data))
        self._queue.append(ex)
        self.exchanges.append(ex)
        return ex

    @property
    def idle(self) -> bool:
        return self._cur is None and not self._queue and not self.tx.busy

    def _send(self, ex: Exchange, start: Optional[float] = None) -> None:
        chars = vmc_block(ex.data)
        if self.corrupt:
            self.corrupt -= 1
            chars[-1] ^= 0xFF
        ex.attempts += 1
        self.tx.send(chars, start=start)
        ex.sent_end.append(self.tx.free_at)
        self._got = []

    def _finish(self, ex: Exchange, outcome: str, cycle: int) -> None:
        ex.outcome = outcome
        ex.answer = list(self._got)
        if outcome in ("ack", "data") or ex.attempts >= self.tries:
            ex.done = True
            self._cur = None
        else:
            self._send(ex, start=cycle + 2 * self.timing.bit)

    def drive(self, drive: Drive, cycle: int) -> None:
        self._now = cycle
        ex = self._cur
        if ex is None and self._queue and not self.tx.busy:
            ex = self._cur = self._queue.popleft()
            self._send(ex)
        elif (ex is not None and not self._got and self.rx._cur is None
              and not self.tx.busy and cycle > ex.sent_end[-1] + self.timeout):
            self._finish(ex, "timeout", cycle)
        self.tx.drive(drive, cycle)

    def observe(self, lines: Lines) -> None:
        self.rx.observe(lines)

    def _char(self, ch: Char, cycle: int) -> None:
        ex = self._cur
        if ex is None or self.tx.busy:
            return
        if not self._got:
            ex.answer_start.append(ch.start)
        self._got.append(ch.value)
        if not ch.stop_ok:
            self._finish(ex, "ferr", cycle)
            return
        if not ch.value & MODE:
            return
        got = self._got
        if len(got) == 1:
            self._finish(ex, "ack" if got[0] & 0xFF == ACK else "nak", cycle)
        elif block_ok(got):
            if self.withhold:
                self.withhold -= 1
            else:
                self.tx.send([ACK], start=cycle + self.timing.bit)
            self._finish(ex, "data", cycle)
        else:
            self.tx.send([NAK], start=cycle + self.timing.bit)
            self._finish(ex, "chk", cycle)
