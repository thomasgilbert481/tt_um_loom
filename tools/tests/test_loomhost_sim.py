"""``tools.loomhost.SimTransport``: the host waveform on the SPI pads.

The transport is what the RTL firmware tests move their bytes through
(``test/rtl_bench.py``), and there it is checked by the design answering it.
Here it is checked on its own, without a simulator: a tiny bench clocks the
transport's pin model against a fake chip that decodes CS_n, SCK and MOSI by
the rules of ``docs/HOST_PROTOCOL.md`` and answers on MISO. What this pins
down is the waveform and the timing the transport promises: mode 0, SCK =
clk/8, MSB first, CS_n low four clocks before the first edge and four after
the last, and MISO sampled where a chip that moves it on the falling edge
has settled.
"""

import pytest

from tools.loomhost import SimTransport
from tools.loomhost.sim_transport import CS_N_PAD, MISO_PAD, MOSI_PAD, SCK_PAD
from tools.protomodels.bench import (CS_N_BIT, Drive, Lines, Model, resolve_pads)

BYTE_CLOCKS = 64
HALF = BYTE_CLOCKS // 16


class Chip(Model):
    """A fake host port: shifts MOSI in on the rising edge and a canned byte
    out on the falling edge, one cycle later, as a registered chip does."""

    def __init__(self, answers=b"", irq=False):
        self.answers = bytes(answers)
        self.irq = irq
        self.prev_sck = 0
        self.selected = False
        self.miso = 0
        self.bits_in = []
        self.bytes_in = bytearray()
        self.out_bits = []
        self.frames = 0
        self.sck_cycles = []            # cycle of every SCK rise, by transaction

    def uo(self) -> int:
        """What ``uo_out`` shows during the coming cycle."""
        return (self.miso << 7) | (int(self.irq) << 6)

    def observe(self, lines: Lines) -> None:
        cs, sck = lines.get(CS_N_PAD), lines.get(SCK_PAD)
        if not cs and not self.selected:
            self.selected = True
            self.bits_in = []
            self.out_bits = [(b >> (7 - i)) & 1
                             for b in self.answers for i in range(8)]
            self.miso = self.out_bits.pop(0) if self.out_bits else 0
        elif cs and self.selected:
            self.selected = False
            self.frames += 1
        if self.selected and sck and not self.prev_sck:
            self.sck_cycles.append(lines.cycle)
            self.bits_in.append(lines.get(MOSI_PAD))
            if len(self.bits_in) == 8:
                self.bytes_in.append(sum(b << (7 - i)
                                         for i, b in enumerate(self.bits_in)))
                self.bits_in = []
        if self.selected and not sck and self.prev_sck:
            self.miso = self.out_bits.pop(0) if self.out_bits else 0
        self.prev_sck = sck


class TinyBench:
    """The three bench methods a transport uses: ``add``, ``step``, ``cycle``,
    plus ``lines``. The chip's outputs are the ``uo`` of the next cycle."""

    def __init__(self, chip):
        self.chip = chip
        self.models = [chip]
        self.lines = Lines()
        self._drive = Drive()
        self._cycle = 0
        self.ui_history = []

    @property
    def cycle(self):
        return self._cycle

    def add(self, model):
        self.models.insert(0, model)     # before the chip, as a bench drives first
        return model

    def step(self, cycles=1):
        for _ in range(cycles):
            self._drive.clear()
            for model in self.models:
                model.drive(self._drive, self._cycle)
            ui, uio, conflict = resolve_pads(self._drive, 0, 0, 0, CS_N_BIT)
            assert not conflict
            self.lines.cycle = self._cycle
            self.lines.ui, self.lines.uio = ui, uio
            self.lines.uo = self.chip.uo()
            self.ui_history.append(ui)
            for model in self.models:
                model.observe(self.lines)
            self._cycle += 1


def test_a_transaction_has_the_shape_host_protocol_asks_for():
    chip = Chip(answers=b"\x4c\x4d\x00")
    bench = TinyBench(chip)
    transport = SimTransport(bench)
    rx = transport.transfer(b"\x00\x00\x00\x00\x00\x00")
    assert rx[:3] == b"\x4c\x4d\x00"                 # what the chip shifted out
    assert chip.bytes_in == bytearray(6) and chip.frames == 1
    # duration and framing: cs_setup + 64 per byte + cs_hold + cs_gap
    assert bench.cycle == 4 + 6 * BYTE_CLOCKS + 4 + 8
    low = [c for c, ui in enumerate(bench.ui_history) if not (ui >> 4) & 1]
    assert low[0] == 0 and len(low) == 4 + 6 * BYTE_CLOCKS + 4
    rises = chip.sck_cycles
    assert rises[0] - low[0] >= 4 and low[-1] - rises[-1] >= 4
    assert len(rises) == 48
    assert all(b - a == 2 * HALF for a, b in zip(rises, rises[1:]))


def test_mosi_carries_the_bytes_msb_first_and_miso_comes_back():
    chip = Chip(answers=b"\xa5\x3c")
    bench = TinyBench(chip)
    transport = SimTransport(bench)
    assert transport.transfer(b"\x80\x01") == b"\xa5\x3c"
    assert bytes(chip.bytes_in) == b"\x80\x01"
    assert transport.transactions == 1


def test_idle_delay_and_the_irq_pin():
    chip = Chip(irq=True)
    bench = TinyBench(chip)
    transport = SimTransport(bench, clk_hz=1_000_000)
    transport.idle(10)
    assert bench.cycle == 10
    transport.delay(1e-3)                            # 1 ms at 1 MHz
    assert bench.cycle == 1010
    assert transport.irq() is True                   # uo_out[6]
    assert bench.lines.get(MISO_PAD) == 0
    chip.irq = False
    transport.idle(1)
    assert transport.irq() is False


def test_the_transport_refuses_timings_the_port_cannot_take():
    bench = TinyBench(Chip())
    with pytest.raises(ValueError):
        SimTransport(bench, clocks_per_byte=32)      # SCK faster than clk/8
    with pytest.raises(ValueError):
        SimTransport(bench, clocks_per_byte=72)      # not a whole number of clocks
    with pytest.raises(ValueError):
        SimTransport(bench, cs_setup=2)              # HOST_PROTOCOL wants 4
