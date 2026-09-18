"""tools.protomodels.bench: pad naming, line resolution and the clock loop."""

import pytest

from tools.loomasm import assemble
from tools.loomisa import load as load_isa
from tools.loomsim import Machine
from tools.protomodels.bench import (UI, UIO, UO, Bench, Drive, Model, pad_of)

ISA = load_isa()


def test_pad_of_follows_the_pin_index_space():
    assert pad_of("BIDIR3") == (UIO, 3)
    assert pad_of("in0") == (UI, 0) and pad_of("IN3") == (UI, 3)
    assert pad_of("IN4") == (UI, 7)                       # ui_in[7]
    assert pad_of("OUT5") == (UO, 5) and pad_of(16) == (UO, 0)
    assert pad_of(("ui", 4)) == (UI, 4)                   # host CS_n: no pin index
    for bad in ("OUT9", 13, 22, ("uo", 8), ("xx", 0)):
        with pytest.raises(ValueError):
            pad_of(bad)


def test_drive_rejects_chip_outputs_and_open_drain_on_inputs():
    d = Drive()
    with pytest.raises(ValueError):
        d.set((UO, 0), 1)
    with pytest.raises(ValueError):
        d.pull_low((UI, 0))


class Hold(Model):
    """Drives one pad to a fixed level (push-pull) or pulls it low."""

    def __init__(self, pad, value=None, low=False):
        self.pad, self.value, self.low = pad, value, low
        self.seen = []

    def drive(self, drive, cycle):
        if self.low:
            drive.pull_low(self.pad)
        elif self.value is not None:
            drive.set(self.pad, self.value)

    def observe(self, lines):
        self.seen.append(lines.get(self.pad))


def test_uio_lines_are_wired_and_with_pull_ups():
    bench = Bench(pullups=0b0110)
    watch = [bench.add(Hold((UIO, i))) for i in range(4)]
    bench.add(Hold((UIO, 2), low=True))                   # a device pulls BIDIR2 low
    bench.step(3)
    assert [w.seen[-1] for w in watch] == [0, 1, 0, 0]    # no pull-up on 0 and 3
    # the chip drives BIDIR0 high and BIDIR1 low with OE: the pad loops back
    bench.machine.host_write_pin_out(0b0001)
    bench.machine.host_write_pin_oe(0b0011)
    bench.step(3)
    assert [w.seen[-1] for w in watch] == [1, 0, 0, 0]
    assert bench.contention_free()


def test_contention_is_recorded():
    bench = Bench(pullups=0b1)
    bench.add(Hold((UIO, 0), low=True))
    bench.machine.host_write_pin_out(0b1)
    bench.machine.host_write_pin_oe(0b1)                  # push-pull high against a low
    bench.step(4)
    assert bench.contentions and bench.contentions[-1][1] == 0b1


def test_models_drive_chip_inputs_through_the_synchroniser():
    program = assemble(".thread 0\nloop: IN r0\n JMP loop\n")
    bench = Bench(image=program.words)
    bench.add(Hold((UI, 7), value=1))                     # IN4
    bench.machine.host_write_debug(0, "INGRP", 12 | (1 << 5))
    bench.machine.host_set_run(1)
    bench.step(40)
    assert bench.machine.pin_in(12) == 1
    assert bench.machine.threads[0].regs[0] == 1
    assert bench.lines.ui & 0x10                         # host CS_n idles high


def test_observers_see_every_cycle_and_run_until_times_out():
    bench = Bench(Machine({0: ISA.encode("JMP", abs=0)}, isa=ISA))
    records = []
    bench.add_observer(records.append)
    bench.machine.host_set_run(1)
    bench.step(20)
    assert len(records) == 20
    assert sum(r is not None for r in records) >= 3
    assert bench.run_until(lambda: False, 10, every=3) is False
    assert bench.run_until(lambda: bench.cycle >= 40, 100, every=5) is True
