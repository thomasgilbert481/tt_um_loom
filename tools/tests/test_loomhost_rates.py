"""tools.loomhost.rates: TICK_INT/TICK_FRAC for a rate."""

import pytest

from tools.loomhost.rates import TICK_INT_MAX, tick, tick_error

CLOCK = 50e6


def test_the_two_uart_examples():
    assert tick(CLOCK, 115200) == (434, 7)               # 434.0278 clocks per bit
    assert tick(CLOCK, 115200, 8) == (54, 65)            # 54.2535 clocks per tick


def test_rounding_is_to_the_nearest_256th_of_a_clock():
    for rate in (1200, 9600, 31250, 115200, 250000, 921600):
        for per in (1, 8, 16):
            whole, frac = tick(CLOCK, rate, per)
            exact = CLOCK / (rate * per)
            assert abs(whole + frac / 256 - exact) <= 1 / 512
            assert 0 <= frac < 256


def test_the_error_left_by_the_rounding_is_tiny_at_uart_rates():
    for rate in (1200, 9600, 115200, 921600):
        assert abs(tick_error(CLOCK, rate)) < 1e-4
        assert abs(tick_error(CLOCK, rate, 8)) < 1e-3


def test_exact_settings_have_no_error():
    assert tick(CLOCK, CLOCK / 32) == (32, 0)
    assert tick_error(CLOCK, CLOCK / 32) == 0


def test_a_tick_outside_the_csr_is_refused():
    with pytest.raises(ValueError):
        tick(CLOCK, CLOCK / (TICK_INT_MAX + 1))          # one clock too slow
    with pytest.raises(ValueError):
        tick(CLOCK, 2 * CLOCK)                           # under a clock
    assert tick(CLOCK, CLOCK / TICK_INT_MAX) == (TICK_INT_MAX, 0)
    with pytest.raises(ValueError):
        tick(CLOCK, 0)
