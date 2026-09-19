"""``resolve_pads`` is the pad arithmetic ``Bench.step`` does inline.

``test/rtl_bench.py`` resolves the pads of the RTL with
:func:`tools.protomodels.bench.resolve_pads` while ``Bench`` keeps its own
copy of the same expressions in its clock loop, for speed. This module is
what keeps the two honest: on a random mix of push-pull drives, open-drain
pulls, pull-ups and chip drive, the function must produce exactly the lines
the bench produced, and the same contention bits.
"""

import random

from tools.protomodels.bench import (UI, UIO, Bench, Drive, Model, resolve_pads)


class Noise(Model):
    """Drives a random mix of ui bits, uio bits and open-drain pulls."""

    def __init__(self, seed):
        self.rng = random.Random(seed)

    def drive(self, drive, cycle):
        for bit in range(8):
            roll = self.rng.random()
            if roll < 0.2:
                drive.set((UIO, bit), 1)
            elif roll < 0.4:
                drive.set((UIO, bit), 0)
            elif roll < 0.55:
                drive.pull_low((UIO, bit))
        for bit in (0, 1, 2, 3, 7):
            if self.rng.random() < 0.5:
                drive.set((UI, bit), self.rng.getrandbits(1))


class Check(Model):
    """Last in the list: snapshots the drive, checks the lines it produced."""

    def __init__(self, bench):
        self.bench = bench
        self.snap = Drive()
        self.conflicts = []
        self.checked = 0

    def drive(self, drive, cycle):
        for field in Drive.__slots__:
            setattr(self.snap, field, getattr(drive, field))

    def observe(self, lines):
        ui, uio, conflict = resolve_pads(self.snap, lines.uio_out, lines.uio_oe,
                                         self.bench.pullups, self.bench.ui_idle)
        assert (ui, uio) == (lines.ui, lines.uio), (lines.cycle, ui, uio)
        if conflict:
            self.conflicts.append((lines.cycle, conflict))
        self.checked += 1


def test_resolve_pads_matches_the_bench_clock_loop():
    bench = Bench(pullups=0b0110_0011)
    bench.add(Noise(7))
    check = bench.add(Check(bench))
    rng = random.Random(11)
    for _ in range(40):
        bench.machine.host_write_pin_out(rng.getrandbits(16))
        bench.machine.host_write_pin_oe(rng.getrandbits(8))
        bench.machine.host_write_od_mask(rng.getrandbits(8))
        bench.step(5)
    assert check.checked == 200
    assert check.conflicts == bench.contentions       # same cycles, same bits
    assert check.conflicts                            # the mix does collide sometimes


def test_resolve_pads_on_an_idle_bench():
    bench = Bench(pullups=0b11)
    check = bench.add(Check(bench))
    bench.step(4)
    assert check.conflicts == [] and bench.contention_free()
    assert bench.lines.uio == 0b11                    # only the pull-ups
    assert bench.lines.ui == 0x10                     # host CS_n idles high
