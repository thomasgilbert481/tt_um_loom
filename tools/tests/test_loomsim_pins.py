"""Pins: the index space, open drain, groups, the synchroniser and pad timing.

Check IDs: L1-PINS (index mapping for every index, OD mode, OE control, group
base and count, synchroniser latency), PIN-1 (an open-drain pin never drives a
one), and the pad-timing half of L2-SLOT (a pin write with X cycle x changes
the pad at edge x + 2).
"""

import pytest

from tools.loomisa import load
from tools.loomsim import Machine
from tools.loomsim.harness import PadTrace, assemble, run_thread

ISA = load()

CSR_OUTGRP = 0x02
CSR_INGRP = 0x03
CSR_OD_MASK = 0x10
CSR_PIN_OUT = 0x11
CSR_PIN_OE = 0x12
CSR_PIN_IN = 0x13


def group(base, count):
    """Pack an OUTGRP/INGRP value: base in bits 4:0, count in bits 9:5."""
    return (base & 0x1F) | ((count & 0x1F) << 5)


def run_pins(program, setup=None, **kwargs):
    return run_thread(ISA, program, setup=setup, **kwargs)


# ------------------------------------------------------------- index space
@pytest.mark.parametrize("pin,pin_out_bit", [
    (0, 0), (1, 1), (7, 7), (16, 8), (17, 9), (21, 13),
])
def test_writable_indices_reach_their_pin_out_bit(pin, pin_out_bit):
    machine, _ = run_pins([("SETP", dict(pin=pin, val=1)), ("HALT", {})])
    assert machine.pin_out == 1 << pin_out_bit
    if pin <= 7:
        assert machine.uio_out == 1 << pin
        assert machine.uo_out == 0
    else:
        assert machine.uo_out == 1 << (pin - 16)
        assert machine.uio_out == 0


@pytest.mark.parametrize("pin", [8, 9, 12, 13, 14, 15, 22, 31])
def test_writes_to_read_only_and_reserved_indices_are_ignored(pin):
    machine, records = run_pins([("SETP", dict(pin=pin, val=1)), ("HALT", {})])
    assert machine.pin_out == 0
    assert machine.pin_oe == 0
    assert records[0].done is True          # still a normal one-slot instruction
    assert machine.badop == 0


@pytest.mark.parametrize("pin", [13, 14, 15, 22, 31])
def test_reserved_indices_read_zero(pin):
    """Even with every pad high, a reserved index reads 0."""
    program = [("IN", dict(rd=1)), ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "INGRP", group(pin, 1))
        machine.set_pad_inputs(ui_in=0xFF, uio_in=0xFF)

    machine, records = run_pins(program, setup=setup)
    assert records[0].val == 0


def test_bidir_pins_read_the_pad_not_the_driven_value():
    """SEMANTICS 3: a BIDIR pin always reads through the synchroniser."""
    program = [("SETP", dict(pin=0, val=1)), ("NOP", {}), ("NOP", {}),
               ("IN", dict(rd=1)), ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "INGRP", group(0, 1))
        machine.set_pad_inputs(uio_in=0x00)          # the pad is held low

    machine, records = run_pins(program, setup=setup)
    assert machine.uio_out & 1 == 1
    assert records[3].val == 0                        # the pad wins


def test_output_pins_read_back_their_driven_value():
    program = [("SETP", dict(pin=18, val=1)), ("NOP", {}),
               ("IN", dict(rd=1)), ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "INGRP", group(18, 1))

    machine, records = run_pins(program, setup=setup)
    assert records[2].val == 1


# --------------------------------------------------------------- open drain
def test_open_drain_writing_one_releases_the_pin():
    program = [("SETP", dict(pin=2, val=1)), ("HALT", {})]

    def setup(machine):
        machine.host_write_od_mask(0xFF)
        machine.host_write_pin_oe(0xFF)               # start driving

    machine, _ = run_pins(program, setup=setup)
    assert machine.pin_out & (1 << 2) == 0
    assert machine.uio_oe & (1 << 2) == 0             # released


def test_open_drain_writing_zero_drives_low():
    program = [("SETP", dict(pin=2, val=0)), ("HALT", {})]

    def setup(machine):
        machine.host_write_od_mask(0xFF)

    machine, _ = run_pins(program, setup=setup)
    assert machine.pin_out & (1 << 2) == 0
    assert machine.uio_oe & (1 << 2) != 0             # driving a zero


def test_an_open_drain_pin_never_drives_a_one():
    """PIN-1, checked every cycle of a program that writes both levels."""
    program = [("SETP", dict(pin=0, val=1)), ("SETP", dict(pin=0, val=0)),
               ("SETP", dict(pin=0, val=1)), ("HALT", {})]
    trace = PadTrace()

    def setup(machine):
        machine.host_write_od_mask(0x01)

    run_pins(program, setup=setup, on_cycle=trace)
    for _, uio_out, uio_oe in trace.samples:
        assert not (uio_out & uio_oe & 0x01)


def test_out_applies_the_open_drain_rule_to_every_bit():
    program = [("OUT", dict(ra=1)), ("HALT", {})]

    def setup(machine):
        machine.host_write_od_mask(0xFF)
        machine.host_write_debug(0, "OUTGRP", group(0, 8))
        machine.host_write_debug(0, "r1", 0b1010_1010)

    machine, _ = run_pins(program, setup=setup)
    assert machine.uio_out == 0                      # open drain never drives one
    assert machine.uio_oe == 0b0101_0101             # zeros are driven low


def test_open_drain_is_per_pin():
    program = [("SETP", dict(pin=0, val=1)), ("SETP", dict(pin=1, val=1)),
               ("HALT", {})]

    def setup(machine):
        machine.host_write_od_mask(0x01)              # only pin 0 is open drain
        machine.host_write_pin_oe(0x03)

    machine, _ = run_pins(program, setup=setup)
    assert machine.uio_out == 0b10                    # pin 1 drove a one
    assert machine.uio_oe == 0b10                     # pin 0 released


# ---------------------------------------------------------------------- OEP
def test_oep_sets_and_clears_the_output_enable():
    machine, _ = run_pins([("OEP", dict(pin=5, val=1)), ("HALT", {})])
    assert machine.uio_oe == 1 << 5
    program = [("OEP", dict(pin=5, val=0)), ("HALT", {})]

    def setup(machine):
        machine.host_write_pin_oe(0xFF)

    machine, _ = run_pins(program, setup=setup)
    assert machine.uio_oe == 0xFF & ~(1 << 5)


@pytest.mark.parametrize("pin", [8, 12, 16, 21, 31])
def test_oep_outside_the_bidir_range_is_ignored(pin):
    machine, _ = run_pins([("OEP", dict(pin=pin, val=1)), ("HALT", {})])
    assert machine.uio_oe == 0


def test_oep_does_not_touch_pin_out():
    program = [("SETP", dict(pin=3, val=1)), ("OEP", dict(pin=3, val=1)),
               ("HALT", {})]
    machine, _ = run_pins(program)
    assert machine.uio_out == 1 << 3
    assert machine.uio_oe == 1 << 3


# ------------------------------------------------------------------ OUT / IN
@pytest.mark.parametrize("count,value,expected_uio", [
    (0, 0xFFFF, 0x00),
    (1, 0xFFFF, 0x01),
    (1, 0xFFFE, 0x00),
    (4, 0x000A, 0x0A),
    (8, 0x00A5, 0xA5),
    (16, 0xFFFF, 0xFF),        # indices 8..15 are not writable
    (31, 0xFFFF, 0xFF),        # count saturates at 16
])
def test_out_writes_count_pins_from_base(count, value, expected_uio):
    program = [("OUT", dict(ra=1)), ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "OUTGRP", group(0, count))
        machine.host_write_debug(0, "r1", value)

    machine, _ = run_pins(program, setup=setup)
    assert machine.uio_out == expected_uio


def test_out_wraps_the_index_modulo_thirty_two():
    program = [("OUT", dict(ra=1)), ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "OUTGRP", group(30, 4))   # 30, 31, 0, 1
        machine.host_write_debug(0, "r1", 0b1100)             # bits 2 and 3

    machine, _ = run_pins(program, setup=setup)
    assert machine.uio_out == 0b11                # only indices 0 and 1 exist


def test_out_to_the_output_pins():
    program = [("OUT", dict(ra=1)), ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "OUTGRP", group(16, 6))
        machine.host_write_debug(0, "r1", 0b101010)

    machine, _ = run_pins(program, setup=setup)
    assert machine.uo_out == 0b101010
    assert machine.uio_out == 0


def test_out_only_changes_the_bits_it_writes():
    """All pin commits are bit-masked (SEMANTICS 6.3)."""
    program = [("OUT", dict(ra=1)), ("HALT", {})]

    def setup(machine):
        machine.host_write_pin_out(0x00F0)
        machine.host_write_debug(0, "OUTGRP", group(0, 2))
        machine.host_write_debug(0, "r1", 0b11)

    machine, _ = run_pins(program, setup=setup)
    assert machine.uio_out == 0xF3


@pytest.mark.parametrize("count,ui_in,expected", [
    (0, 0xFF, 0x0000),
    (1, 0x01, 0x0001),
    (1, 0x00, 0x0000),
    (4, 0x0A, 0x000A),
    (5, 0x8F, 0x001F),          # index 12 is ui_in[7]
])
def test_in_reads_count_pins_from_base(count, ui_in, expected):
    program = [("IN", dict(rd=1)), ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "INGRP", group(8, count))
        machine.host_write_debug(0, "r1", 0xFFFF)
        machine.set_pad_inputs(ui_in=ui_in)

    machine, records = run_pins(program, setup=setup)
    assert records[0].val == expected


def test_in_of_sixteen_pins_wraps_and_zero_fills():
    program = [("IN", dict(rd=1)), ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "INGRP", group(16, 16))
        machine.host_write_pin_out(0x3F00)       # every OUT pin driven high
        machine.set_pad_inputs(ui_in=0xFF, uio_in=0xFF)

    machine, records = run_pins(program, setup=setup)
    assert records[0].val == 0x003F              # indices 22..31 read 0


def test_in_wraps_past_index_thirty_one_into_the_bidir_pins():
    program = [("IN", dict(rd=1)), ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "INGRP", group(30, 4))    # 30, 31, 0, 1
        machine.set_pad_inputs(uio_in=0b11)

    machine, records = run_pins(program, setup=setup)
    assert records[0].val == 0b1100


def test_groups_can_be_set_with_csrw():
    program = [("LDI", dict(rd=1, imm=group(16, 2) & 0xFF)),
               ("CSRW", dict(csr=CSR_OUTGRP, ra=1)),
               ("LDI", dict(rd=2, imm=0b11)),
               ("OUT", dict(ra=2)),
               ("HALT", {})]
    machine, _ = run_pins(program)
    assert machine.threads[0].outgrp == group(16, 2)
    assert machine.uo_out == 0b11


# ------------------------------------------------------- synchroniser timing
def test_the_input_synchroniser_takes_two_edges():
    """pin_in in cycle x is the pad value sampled at edge x - 1."""
    machine = Machine({}, isa=ISA)
    machine.run_cycles(4)
    assert machine.pin_in(8) == 0
    change_cycle = machine.cycle
    machine.set_pad_inputs(ui_in=0x01)             # held at edge change_cycle + 1
    seen = {}
    for _ in range(5):
        cycle = machine.cycle
        seen[cycle] = machine.pin_in(8)
        machine.step_cycle()
    assert seen[change_cycle] == 0
    assert seen[change_cycle + 1] == 0             # only FF1 has it
    assert seen[change_cycle + 2] == 1             # FF2, which the logic reads


def test_pin_in_csr_shows_the_synchronised_word():
    program = [("CSRR", dict(rd=1, csr=CSR_PIN_IN)), ("HALT", {})]

    def setup(machine):
        machine.set_pad_inputs(ui_in=0x8F, uio_in=0x5A)

    machine, records = run_pins(program, setup=setup)
    # bits 0..7 uio, bits 8..11 ui_in[3:0], bit 12 ui_in[7]
    assert records[0].val == 0x5A | (0x0F << 8) | (1 << 12)


def test_loopback_feeds_driven_bidir_outputs_back_in():
    program = [("OEP", dict(pin=0, val=1)),
               ("SETP", dict(pin=0, val=1)),
               ("NOP", {}), ("NOP", {}),
               ("IN", dict(rd=1)),
               ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "INGRP", group(0, 2))

    machine, records = run_pins(program, setup=setup, loopback=True)
    assert records[4].val == 0b01                  # pin 0 reads its own output
    assert machine.uio_oe == 1


# ------------------------------------------------------------- pad timing
def test_a_pin_write_changes_the_pad_at_edge_x_plus_two():
    program = [("SETP", dict(pin=16, val=1)), ("HALT", {})]
    trace = PadTrace()
    machine, records = run_pins(program, on_cycle=trace)
    x = records[0].x_cycle
    bits = trace.uo_bit(0)
    assert bits[x] == 0
    assert bits[x + 1] == 0                        # still low while in W
    assert bits[x + 2] == 1                        # the pad register took it
    assert trace.edges(bits) == [(x + 2, 1)]


def test_back_to_back_pin_writes_land_one_slot_apart():
    program = [("SETP", dict(pin=16, val=1)),
               ("SETP", dict(pin=16, val=0)),
               ("SETP", dict(pin=16, val=1)),
               ("HALT", {})]
    trace = PadTrace()
    machine, records = run_pins(program, on_cycle=trace)
    edges = trace.edges(trace.uo_bit(0))
    assert [cycle for cycle, _ in edges] == [r.x_cycle + 2 for r in records[:3]]
    assert [value for _, value in edges] == [1, 0, 1]


def test_two_threads_writing_different_pins_do_not_collide():
    image = {}
    image.update(assemble(ISA, [("SETP", dict(pin=16, val=1)), ("HALT", {})], 0x000))
    image.update(assemble(ISA, [("SETP", dict(pin=17, val=1)), ("HALT", {})], 0x100))
    machine = Machine(image, isa=ISA)
    machine.host_set_run(0b0011)
    machine.run_cycles(24)
    assert machine.uo_out == 0b11
