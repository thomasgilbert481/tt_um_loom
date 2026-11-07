"""NMEA 0183 reference model: sentences with their checksum, a GPS source
that sends them over 8N1, and a reference parser for the RMC sentence.

NMEA 0183 itself is sold by the NMEA and was not read. Written from two
secondary sources, never from the firmware:

* SiRF Technology, "NMEA Reference Manual", 1050-0042, Revision 2.1,
  December 2007. Table 2-1, note 3 (p.2-1): the checksum is "a binary
  exclusive OR [of] the lower 7 bits of each character after the $ and
  before the * symbols", sent as two hexadecimal characters, and every
  sentence ends in CR LF (0x0D 0x0A). Table 1-11, "RMC Data Format"
  (p.1-5): message ID ``$GPRMC``, then UTC time ``hhmmss.sss``, status
  (``A`` = data valid, ``V`` = data not valid), latitude ``ddmm.mmmm``,
  N/S, longitude ``dddmm.mmmm``, E/W, speed (knots), course (degrees),
  date ``ddmmyy``, magnetic variation and its E/W, mode (A, D, E), with
  the example ``$GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,
  120598, ,*10`` (:data:`SIRF_EXAMPLE`). As printed, with a space in the
  magnetic-variation field, its checksum is 0x30; 0x10 is the checksum of
  the sentence with that field null, so the space is taken for a
  typesetting artifact (QUESTIONS.md). Table 1-1 and the PSRF100 table:
  4800 to 115200 baud, 8 data bits, 1 stop bit, no parity.
* E. S. Raymond, "NMEA Revealed" (gpsd.gitlab.io/gpsd/NMEA.html, read
  2026-11-07), "NMEA Encoding Conventions": the first field is "a
  two-letter talker ID followed by a three-letter type code"; the checksum
  is "encoded as two hexadecimal characters (0-9, A-F), the
  most-significant-nibble being sent first"; "Maximum sentence length,
  including the $ and <CR><LF> is 82 bytes". "NMEA 0183 physical protocol
  layer": 4800 bps 8N1 by the standard, "9600 probably the most common".

The XOR of 7-bit characters (SiRF) and of 8-bit ones (gpsd) agree for the
ASCII a sentence is made of. Upper-case hex is what both sources print;
neither says lower case is forbidden (QUESTIONS.md).
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Optional

from .uart import UartTx

#: SiRF Table 1-11's example with the checksum the manual prints, its
#: magnetic-variation field null (the printed space does not checksum).
SIRF_EXAMPLE = (b"$GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,"
                b"120598,,*10\r\n")
MAX_LEN = 82                    # gpsd: including '$' and CR LF
RMC_FIELDS = ("time", "status", "lat", "ns", "lon", "ew", "speed", "course",
              "date", "magvar", "magvar_ew", "mode")


def checksum(body: bytes) -> int:
    """The XOR of every character between ``$`` and ``*`` (``body``)."""
    value = 0
    for ch in body:
        value ^= ch & 0x7F
    return value


def sentence(body, *, csum: Optional[int] = None) -> bytes:
    """``$body*hh`` CR LF; ``csum`` overrides the checksum (a bad one)."""
    if isinstance(body, str):
        body = body.encode("ascii")
    value = checksum(body) if csum is None else csum
    return b"$" + body + b"*%02X\r\n" % value


def rmc(time="123519", status="A", lat="4807.038", ns="N",
        lon="01131.000", ew="E", speed="", course="", date="230394",
        magvar="", magvar_ew="", *, talker="GP", mode=None,
        csum: Optional[int] = None) -> bytes:
    """An RMC sentence, fields in Table 1-11's order; empty fields are null
    fields. ``mode`` (NMEA 2.3 on) is left out when None."""
    fields = [time, status, lat, ns, lon, ew, speed, course, date, magvar,
              magvar_ew]
    if mode is not None:
        fields.append(mode)
    return sentence(",".join([talker + "RMC"] + fields), csum=csum)


def parse(line: bytes) -> Optional[Dict[str, object]]:
    """A received sentence checked as the sources describe it: ``$``,
    printable characters, ``*``, two hex digits (either case) equal to the
    checksum, CR LF. Returns ``{"talker", "type", "fields"}`` or None."""
    if not (line.startswith(b"$") and line.endswith(b"\r\n")):
        return None
    if len(line) > MAX_LEN:
        return None
    star = line.rfind(b"*")
    if star < 0 or star != len(line) - 5:
        return None
    body, digits = line[1:star], line[star + 1:star + 3]
    if any(not 0x20 <= ch < 0x7F or ch in b"$*" for ch in body):
        return None
    try:
        value = int(digits.decode("ascii"), 16)
    except ValueError:
        return None
    if value != checksum(body):
        return None
    parts = body.decode("ascii").split(",")
    address = parts[0]
    if len(address) != 5:
        return None
    return {"talker": address[:2], "type": address[2:], "fields": parts[1:]}


def parse_rmc(line: bytes) -> Optional[Dict[str, str]]:
    """The fields of a good RMC sentence by Table 1-11's names (missing ones
    as ""), or None for anything else."""
    got = parse(line)
    if got is None or got["type"] != "RMC" or len(got["fields"]) < 6:
        return None
    fields: List[str] = list(got["fields"]) + [""] * len(RMC_FIELDS)
    return dict(zip(RMC_FIELDS, fields))


class GpsSource(UartTx):
    """A GPS receiver's TX line: 8N1, idle high, sentences back to back.

    ``noise(width)`` models a burst on the line: a low pulse of ``width``
    clocks where the next frame would start. A UART takes it for a start
    bit and reads a garbage character (a pulse shorter than a frame) or a
    framing error (a longer one); the line then idles for a frame."""

    def __init__(self, pin, bit_clocks: float, **kwargs) -> None:
        super().__init__(pin, bit_clocks, **kwargs)
        self.sent: List[bytes] = []

    def send_sentences(self, lines: Iterable[bytes]) -> None:
        for line in lines:
            self.sent.append(line)
            self.send(line)

    def noise(self, width: float) -> int:
        t0 = max(self._free_at, float(self.now + 1))
        self._changes.append((int(round(t0)), 0))
        self._changes.append((int(round(t0 + width)), self.idle))
        self._free_at = t0 + width + 10 * self.bit_clocks
        return int(round(t0))
