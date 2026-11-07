"""IrDA SIR reference models, for firmware/irda_sir_tx.loom and irda_sir_rx.loom.

Written from the IrDA physical layer specification, not from the firmware.
IrPHY 1.4 itself (``IrPHY_1p4.pdf`` at irda.org) could not be fetched on
2026-11-05 (HTTP 522); its figures are taken from Vishay application note
82513, "IrDA Physical Layer" (rev. 1.5, 12 Aug 2021), which reproduces the
IrPHY 1.4 tables and cites them:

* SIR is the UART character, start bit, eight data bits LSB first and one
  stop bit, with each 0 sent as one light pulse and each 1 as no light
  (RZI, "Signaling Rate and Pulse Duration" and figure 2).
* Rates 2.4, 9.6 (mandatory), 19.2, 38.4, 57.6 and 115.2 kbit/s, each within
  +-0.87 % (table 2).
* Pulse duration (table 2): 1.41 us minimum at every rate; nominal 3/16 of
  a bit (19.53 us at 9.6 kbit/s, 1.63 us at 115.2 kbit/s); maximum 3/16 of a
  bit plus the larger of 2.5 % of a bit and 0.6 us (88.55, 22.13, 11.07,
  5.96, 4.34 and 2.23 us). "The specification allows both a 3/16 of bit
  duration pulse and a minimum pulse duration for the 115.2 kbit/s signal
  (1.63 us minus the 0.22 us tolerance)."
* Rise and fall time 600 ns at most, edge jitter +-6.5 % of the nominal
  pulse duration, at 115.2 kbit/s and below (table 3). Rise and fall are the
  transceiver's (an optical figure); the pins here switch in one clock.
* Where in its bit cell a pulse sits is not in the note (figure 2 draws it
  inside the cell): unchecked. The decoder here takes a pulse anywhere in
  the cell, as a UART takes a level; the checker measures positions against
  the frame's own start pulse, so it does not depend on that choice.
* Receivers (Vishay TFDU4101 and similar) output low for each pulse
  received; transmitters take high for LED on. Those are the bench pin
  polarities: :class:`SirRecorder` reads an active-high TX pin,
  :class:`SirSource` drives an active-low RX pin.

Time is in clocks (floats allowed in pulse lists, rounded by the bench
models). A pulse list is ``[(start, width), ...]``.

* :func:`frame_bits`, :func:`encode`: the encoder; :func:`decode`: the
  decoder; :func:`violations`: the timing checker (width, position, rate).
* :func:`drop_pulse`, :func:`move_pulse`, :func:`add_pulse`, :func:`stretch`:
  error injection (a missing pulse, a pulse out of place, a stray pulse, a
  rate error).
"""

from __future__ import annotations

import dataclasses
from typing import Iterable, List, Optional, Sequence, Tuple

from tools.protomodels.bench import Drive, Lines, Model, PinSpec, pad_of

RATES = (2400, 9600, 19200, 38400, 57600, 115200)   # bit/s, table 2
RATE_TOL = 0.0087                   # +-0.87 % of the rate, table 2
PULSE_MIN_US = 1.41                 # table 2, every rate
PULSE_115K_US = 1.63                # 3/16 of a 115.2 kbit/s bit, nominal
JITTER = 0.065                      # +-6.5 % of the nominal pulse, table 3
RISE_FALL_NS = 600                  # table 3 (optical; not modelled)

Pulses = List[Tuple[float, float]]


def bit_us(rate: float) -> float:
    return 1e6 / rate


def nominal_us(rate: float) -> float:
    """3/16 of a bit (table 2's nominal column)."""
    return bit_us(rate) * 3 / 16


#: Table 2's maximum pulse durations, as printed. The text's rule (3/16 of
#: a bit plus the larger of 2.5 % of a bit and 0.6 us) gives these at 2.4,
#: 9.6, 19.2 and 115.2 kbit/s, but 5.53 and 3.86 us at 38.4 and 57.6, where
#: the table prints 5.96 and 4.34 (nominal + 1.08 us): QUESTIONS.md. The
#: checker takes the stricter, the rule.
MAX_US_TABLE = {2400: 88.55, 9600: 22.13, 19200: 11.07, 38400: 5.96,
                57600: 4.34, 115200: 2.23}


def max_us(rate: float) -> float:
    """The text's maximum: 3/16 of a bit plus max(2.5 % of a bit, 0.6 us)."""
    return nominal_us(rate) + max(0.025 * bit_us(rate), 0.6)


def frame_bits(byte: int) -> List[int]:
    """Start bit, eight data bits LSB first, stop bit."""
    return [0] + [(byte >> i) & 1 for i in range(8)] + [1]


def encode(data: Iterable[int], bit: float, pulse: float, *,
           offset: float = 7 / 16, gap_bits: float = 0.0,
           t0: float = 0.0) -> Pulses:
    """The pulses of ``data`` sent back to back (``gap_bits`` of idle 1s
    between frames), ``bit`` clocks a bit, each pulse ``pulse`` clocks long
    and starting ``offset`` of a bit into its cell."""
    out: Pulses = []
    t = float(t0)
    for byte in data:
        for k, b in enumerate(frame_bits(byte)):
            if b == 0:
                out.append((t + k * bit + offset * bit, float(pulse)))
        t += (10 + gap_bits) * bit
    return out


@dataclasses.dataclass
class SirFrame:
    """One decoded character: ``start`` is its start pulse's start."""

    byte: int
    ferr: bool                      # a pulse in the stop bit's cell
    start: float
    pulses: List[Tuple[float, float]]   # the frame's pulses, start first


def decode(pulses: Sequence[Tuple[float, float]], bit: float) -> List[SirFrame]:
    """UART decoding of a pulse list: the first pulse not yet used is a start
    bit; data bit k (1..8) is 0 when a pulse starts within half a bit of
    ``start + k * bit``, else 1; a pulse around ``start + 9 * bit`` is a
    framing error. The next frame's start is looked for after that cell."""
    ps = sorted(pulses)
    frames: List[SirFrame] = []
    i = 0
    while i < len(ps):
        s = ps[i][0]
        mine = [ps[i]]
        i += 1
        zeros = set()
        while i < len(ps) and ps[i][0] < s + 9.5 * bit:
            k = int((ps[i][0] - s) / bit + 0.5)
            zeros.add(k)
            mine.append(ps[i])
            i += 1
        byte = sum(1 << (k - 1) for k in range(1, 9) if k not in zeros)
        frames.append(SirFrame(byte, 9 in zeros, s, mine))
    return frames


def violations(pulses: Sequence[Tuple[float, float]], rate: float,
               clock_hz: float) -> List[str]:
    """Everything in a transmitted pulse list that IrPHY 1.4 does not allow
    (tables 2 and 3, through the Vishay note): a width outside 1.41 us ..
    the maximum, and a pulse whose start is off its place, measured from the
    frame's start pulse, by more than the edge jitter (6.5 % of the nominal
    pulse) plus the rate tolerance (0.87 %) over the bits between."""
    us = 1e6 / clock_hz
    bitc = clock_hz / rate
    jit = JITTER * nominal_us(rate) / us
    bad: List[str] = []
    for f in decode(pulses, bitc):
        for start, width in f.pulses:
            w = width * us
            if not PULSE_MIN_US <= w <= max_us(rate):
                bad.append("pulse at %d: %.3f us, outside %.2f .. %.2f us"
                           % (start, w, PULSE_MIN_US, max_us(rate)))
            k = round((start - f.start) / bitc)
            err = start - f.start - k * bitc
            if abs(err) > jit + RATE_TOL * k * bitc:
                bad.append("pulse at %d: %.1f clocks off bit %d of the frame "
                           "at %d" % (start, err, k, f.start))
    return bad


# ----------------------------------------------------------- error injection
def drop_pulse(pulses: Sequence[Tuple[float, float]], index: int) -> Pulses:
    """A missing pulse: a 0 that the receiver sees as a 1."""
    return [p for j, p in enumerate(pulses) if j != index]


def move_pulse(pulses: Sequence[Tuple[float, float]], index: int,
               by: float) -> Pulses:
    """A pulse out of place, ``by`` clocks later (earlier if negative)."""
    return sorted((s + by, w) if j == index else (s, w)
                  for j, (s, w) in enumerate(pulses))


def add_pulse(pulses: Sequence[Tuple[float, float]], at: float,
              width: float) -> Pulses:
    """A stray pulse (a 1 seen as a 0, or one in a stop cell)."""
    return sorted(list(pulses) + [(float(at), float(width))])


def stretch(pulses: Sequence[Tuple[float, float]], factor: float, *,
            widths: bool = True, t0: float = 0.0) -> Pulses:
    """A rate error: every time from ``t0`` scaled by ``factor`` (1.0087 is
    a sender 0.87 % slow); widths too unless ``widths`` is False (a fixed
    1.63 us pulse does not scale with the bit)."""
    return [(t0 + (s - t0) * factor, w * factor if widths else w)
            for s, w in pulses]


# ------------------------------------------------------------- bench models
class SirRecorder(Model):
    """The pulses on an active-high transmitter pin (LED on while high):
    ``pulses`` holds ``(rise cycle, width in clocks)``, the rise cycle being
    the first cycle high; ``high_at_end`` says a pulse is still going."""

    def __init__(self, pin: PinSpec) -> None:
        self.pad = pad_of(pin)
        self.level = 0
        self.rise = 0
        self.pulses: List[Tuple[int, int]] = []

    def observe(self, lines: Lines) -> None:
        level = lines.get(self.pad)
        if level and not self.level:
            self.rise = lines.cycle
        elif self.level and not level:
            self.pulses.append((self.rise, lines.cycle - self.rise))
        self.level = level

    @property
    def high_at_end(self) -> bool:
        return bool(self.level)


class SirSource(Model):
    """Drives an active-low receiver pin, as a transceiver's RXD: low for
    each pulse of the lists :meth:`send` queues, high otherwise. Times are
    rounded to whole clocks from the start of each list, so they do not
    drift."""

    def __init__(self, pin: PinSpec) -> None:
        self.pad = pad_of(pin)
        self.level = 1
        self.free_at = 0
        self._changes: List[Tuple[int, int]] = []

    def send(self, pulses: Sequence[Tuple[float, float]], *,
             at: Optional[int] = None) -> int:
        """Queue ``pulses`` (times from 0) to start at cycle ``at``, or where
        the last list ended; return the start cycle."""
        start = self.free_at if at is None else int(at)
        end = start
        for s, w in pulses:
            a = start + int(round(s))
            b = start + int(round(s + w))
            self._changes += [(a, 0), (b, 1)]
            end = max(end, b)
        self._changes.sort()
        self.free_at = end
        return start

    @property
    def busy(self) -> bool:
        return bool(self._changes)

    def drive(self, drive: Drive, cycle: int) -> None:
        changes = self._changes
        while changes and changes[0][0] <= cycle:
            self.level = changes.pop(0)[1]
        drive.set(self.pad, self.level)
