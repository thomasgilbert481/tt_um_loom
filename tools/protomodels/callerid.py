"""On-hook caller ID reference models: the message builder and parser, the
burst's bit stream (channel seizure, mark signal, 8N1 characters, trailing
marks), a reference decoder for a recorded FSK burst, and the words the
Loom programs exchange with the host.

Written from the specifications, not from ``firmware/callerid_*.loom``. The
FSK layer (tones, the phase-continuous source, the zero-crossing receiver)
is ``tools.protomodels.bell202``.

Sources (*secondary* marks a figure not read in the primary document,
*unchecked* one not read at all):

* ETSI EN 300 659-1 V1.3.1 (2001-01), "Subscriber line protocol over the
  local loop for display (and related) services; Part 1: On-hook data
  transmission", read 2026-11-07:
  - clause 5.2: the channel seizure signal is "a block of 300 continuous
    bits of alternating 0s and 1s", the first a 0 and the last a 1; the mark
    signal is "180 +- 25 mark bits or 80 +- 25 mark bits"; the message
    length is the number of presentation-layer octets (not the type, length
    or checksum); the checksum is "the two's complement of the modulo 256
    sum of all the octets in the message starting from the Message type
    octet up to the end of the message (excluding the Checksum itself)";
    a message with a wrong checksum "should be discarded by the TE";
  - clause 5.1: a parameter is a type octet, a length octet (the number of
    data octets that follow) and 1 to 253 data octets;
  - clause 5.3: each octet has a start bit (space) and a stop bit (mark),
    least significant bit first; "the data signal shall be continuous";
    "the Stop bit following the checksum shall be followed by an additional
    one to ten Mark bits"; the modulation stops right after the last bit;
    the physical layer is V.23 at 1200 baud;
  - clause 6.1.1: the burst starts 500 ms to 2 s after the end of the first
    ring and the second ring starts at least 200 ms after it.
* ETSI EN 300 659-3 V1.3.1 (2001-01), "Part 3: Data link message and
  parameter codings", read 2026-11-07: clause 5.2.1, the Call Set-up
  message type 0x80; clause 5.3 table 6, parameter types 0x01 Date and Time
  (length 8: month, day, hour, minute, two T.50 digits each, clause 5.4.1),
  0x02 Calling Line Identity (max 20), 0x04 Reason for Absence of CLI
  (length 1), 0x07 Calling Party Name (max 50), 0x08 Reason for Absence of
  Name (length 1); clause 5.4.4, the reasons 0x4F 'O' unavailable and 0x50
  'P' private.
* Telcordia GR-30-CORE (the North American on-hook format) is not free and
  was not read. That it uses Bell 202 tones (``bell202``: mark 1200 Hz,
  space 2200 Hz) with the same seizure, mark signal, MDMF type 0x80,
  parameter codes and checksum is *secondary* (decoder chip datasheets such
  as the Holtek HT9032 repeat it; the fetch of that datasheet failed), as
  are its shorter limits (number 10 digits, name 15 characters, *unchecked*),
  and the SDMF type 0x04 (*unchecked*).

The models take mark (1) for 1200 Hz, Bell 202, as the North American
format does; ETSI's V.23 tones are not modelled.
"""

from __future__ import annotations

import dataclasses
from typing import Iterable, List, Optional, Sequence, Tuple

from . import bell202 as F

SEIZURE_BITS = 300                  # EN 300 659-1 clause 5.2
MARK_BITS = 180                     # on-hook; 80 is the other option
MARK_TOL = 25                       # +- (clause 5.2)
TAIL_MIN, TAIL_MAX = 1, 10          # mark bits after the checksum's stop bit (5.3)

MDMF = 0x80                         # Call Set-up (EN 300 659-3 clause 5.2.1)
SDMF = 0x04                         # *secondary*, *unchecked*
DATE_TIME = 0x01                    # EN 300 659-3 table 6
NUMBER = 0x02
NO_NUMBER = 0x04
NAME = 0x07
NO_NAME = 0x08
UNAVAILABLE, PRIVATE = 0x4F, 0x50   # 'O', 'P' (clause 5.4.4)


def checksum(octets: Iterable[int]) -> int:
    """The two's complement of the mod-256 sum (clause 5.2)."""
    return (-sum(octets)) & 0xFF


def build_message(params: Sequence[Tuple[int, bytes]], msg_type: int = MDMF,
                  *, corrupt: int = 0) -> bytes:
    """Type, length, the parameters (type, length, data each) and the
    checksum. ``corrupt`` is XORed into the checksum (a bad message)."""
    body = b"".join(bytes([t, len(d)]) + bytes(d) for t, d in params)
    head = bytes([msg_type, len(body)]) + body
    return head + bytes([checksum(head) ^ corrupt])


@dataclasses.dataclass
class Message:
    msg_type: int
    params: List[Tuple[int, bytes]]
    checksum_ok: bool


def parse_message(octets: bytes) -> Message:
    """The inverse of :func:`build_message` (MDMF-style parameters)."""
    msg_type, length = octets[0], octets[1]
    body = octets[2:2 + length]
    if len(body) != length or len(octets) < length + 3:
        raise ValueError("short message")
    params, i = [], 0
    while i < length:
        t, n = body[i], body[i + 1]
        params.append((t, bytes(body[i + 2:i + 2 + n])))
        i += 2 + n
    if i != length:
        raise ValueError("parameter lengths do not add up to the message length")
    ok = sum(octets[:length + 3]) & 0xFF == 0
    return Message(msg_type, params, ok)


def date_time(month: int, day: int, hour: int, minute: int) -> bytes:
    """The Date and Time parameter's data, MMDDHHMM in T.50 digits (5.4.1)."""
    return b"%02d%02d%02d%02d" % (month, day, hour, minute)


# ------------------------------------------------------------ the burst
def seizure(bits: int = SEIZURE_BITS) -> List[int]:
    """Alternating bits, the first a 0 (clause 5.2)."""
    return [i & 1 for i in range(bits)]


def burst_bits(message: bytes, *, seizure_bits: int = SEIZURE_BITS,
               mark_bits: int = MARK_BITS, tail: int = 4,
               gaps: Optional[dict] = None) -> List[int]:
    """The whole burst: seizure, mark signal, each octet as 8N1, ``tail``
    mark bits. ``gaps`` maps an octet index to extra mark bits before it
    (a stream the standard does not allow: clause 5.3, continuous)."""
    bits = seizure(seizure_bits) + [1] * mark_bits
    for i, octet in enumerate(message):
        bits += [1] * (gaps or {}).get(i, 0)
        bits += F.char_bits(octet, F.NONE)
    return bits + [1] * tail


@dataclasses.dataclass
class Burst:
    """What :func:`decode_burst` found in a recorded burst."""

    seizure_bits: float     # from the carrier's first edge to the mark signal
    mark_bits: float        # from the last seizure stop bit to the type's start bit
    octets: bytes           # type to checksum
    continuous: bool        # every octet 10 bits after the one before (5.3)
    tail_s: float           # from the checksum's stop bit end to the last edge
    framing_errors: int


def decode_burst(times: Sequence[float]) -> Burst:
    """A reference decoder, independent of the firmware's method: the
    ``bell202`` zero-crossing receiver gives the 8N1 characters; the
    seizure is the run of 0x55 characters on a 10-bit grid; the message is
    the first character after it that starts later than that grid, and the
    octets that its length byte says follow it, plus the checksum."""
    # the silence before the carrier reads as mark, as a UART's idle line
    # does, so the seizure's first 0 is a start bit (a mark half-period
    # ending at the carrier's first edge stands for it)
    chars = F.demodulate([times[0] - 0.5 / F.MARK_HZ] + list(times), F.NONE)
    if not chars:
        raise ValueError("no characters")
    j = 0
    while (j + 1 < len(chars) and chars[j].byte == 0x55
           and abs(chars[j + 1].start_s - chars[j].start_s - 10 * F.BIT_S)
           < 0.5 * F.BIT_S):
        j += 1
    if chars[j].byte != 0x55:
        raise ValueError("no seizure")
    mark_start = chars[j].start_s + 10 * F.BIT_S
    msg = chars[j + 1:]
    if len(msg) < 3:
        raise ValueError("no message")
    n = msg[1].byte + 3
    msg = msg[:n]
    if len(msg) < n:
        raise ValueError("message cut short: %d of %d octets" % (len(msg), n))
    steps = [b.start_s - a.start_s for a, b in zip(msg, msg[1:])]
    end = msg[-1].start_s + 10 * F.BIT_S
    return Burst(
        seizure_bits=(mark_start - times[0]) / F.BIT_S,
        mark_bits=(msg[0].start_s - mark_start) / F.BIT_S,
        octets=bytes(c.byte for c in msg),
        continuous=all(abs(s - 10 * F.BIT_S) < 0.5 * F.BIT_S for s in steps),
        tail_s=times[-1] - end,
        framing_errors=sum(c.framing_error for c in msg))


# --------------------------------------------- the programs' host words
# These follow the host interfaces in the program headers (the headers,
# not the code, are the specification of these words).
def gen_config(seizure_blocks: int = SEIZURE_BITS // 10,
               mark_blocks: int = MARK_BITS // 10) -> int:
    return seizure_blocks | mark_blocks << 8


def gen_words(params: Sequence[Tuple[int, bytes]], msg_type: int = MDMF) -> List[int]:
    """callerid_gen.loom's INQ words for one message."""
    words: List[int] = []
    for t, data in params:
        words.append(0x200 | t)
        words += list(data)
    return words + [0x100 | msg_type]


def rx_config(smin: int = 10, mmin: int = 50) -> int:
    return smin | mmin << 8


TYPE, LENGTH, PTYPE, PLEN, PDATA, END = 0x100, 0x200, 0x300, 0x400, 0x500, 0x600
GOOD, BAD_CHECKSUM, FRAMING, CARRIER_LOST = 0, 1, 2, 3


def rx_words(octets: bytes, status: Optional[int] = None) -> List[int]:
    """callerid_rx.loom's OUTQ words for a received message: every octet
    from the type to the last parameter byte with its role, then the end
    word (``status`` defaults to the checksum's verdict)."""
    words = [TYPE | octets[0], LENGTH | octets[1]]
    i, end = 2, 2 + octets[1]
    while i < end:
        words.append(PTYPE | octets[i])
        n = octets[i + 1] if i + 1 < end else 0
        if i + 1 < end:
            words.append(PLEN | n)
        words += [PDATA | b for b in octets[i + 2:min(i + 2 + n, end)]]
        i += 2 + n
    if status is None:
        status = GOOD if sum(octets[:end + 1]) & 0xFF == 0 else BAD_CHECKSUM
    return words + [END | status]
