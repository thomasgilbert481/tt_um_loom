"""Tick settings for a rate: what to write to TICK_INT and TICK_FRAC.

A thread's tick is ``TICK_INT + TICK_FRAC / 256`` clocks (``docs/SEMANTICS.md``
5), so a program that spends ``ticks_per_unit`` ticks on each period of a
rate needs ``clock_hz / (rate_hz * ticks_per_unit)`` clocks per tick.
:func:`tick` rounds that to the nearest 1/256 of a clock, the resolution of
TICK_FRAC, and :func:`tick_error` gives the mean rate error the rounding
leaves. Whether a program can run that fast is its own business: its
``.tick`` line says the shortest tick its schedule is proved for
(``firmware/README.md``).

Examples, at 50 MHz:

* ``uart_tx_fifo`` (one tick per bit) at 115200 baud: ``tick(50e6, 115200)``
  is ``(434, 7)``;
* ``uart_rx`` (eight ticks per bit) at 115200 baud: ``tick(50e6, 115200, 8)``
  is ``(54, 65)``.
"""

from typing import Tuple

#: TICK_INT is a 16-bit CSR, and a tick shorter than one clock is not one.
TICK_INT_MAX = 0xFFFF


def tick(clock_hz: float, rate_hz: float, ticks_per_unit: int = 1) -> Tuple[int, int]:
    """``(TICK_INT, TICK_FRAC)`` for ``ticks_per_unit`` ticks per period of
    ``rate_hz`` at ``clock_hz``, to the nearest 1/256 of a clock."""
    if clock_hz <= 0 or rate_hz <= 0 or ticks_per_unit < 1:
        raise ValueError("clock, rate and ticks per unit must be positive")
    units = round(clock_hz * 256 / (rate_hz * ticks_per_unit))
    whole, frac = divmod(units, 256)
    if not 1 <= whole <= TICK_INT_MAX:
        raise ValueError("%g Hz with %d ticks per period at a %g Hz clock needs a tick of "
                         "%.3f clocks, outside 1 .. %d"
                         % (rate_hz, ticks_per_unit, clock_hz, units / 256, TICK_INT_MAX))
    return whole, frac


def tick_error(clock_hz: float, rate_hz: float, ticks_per_unit: int = 1) -> float:
    """The mean rate error of :func:`tick`'s setting, as a fraction of
    ``rate_hz`` (positive: the line runs fast)."""
    whole, frac = tick(clock_hz, rate_hz, ticks_per_unit)
    actual = clock_hz / ((whole + frac / 256) * ticks_per_unit)
    return actual / rate_hz - 1
