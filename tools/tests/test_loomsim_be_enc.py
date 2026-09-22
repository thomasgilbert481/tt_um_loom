"""The bit-engine encoders, stuffers and differential output on the golden
model: ``docs/SEMANTICS.md`` 6.9.1 (M3 slice A, D-026), feature ``"BEENC"``.

Slice A adds three ``BE_CFG`` fields, ``ENC`` (bits 4:3: 0 NRZ, 1 NRZI, 2
Manchester), ``STUFF`` (6:5: 0 none, 1 USB, 2 CAN) and ``DIFF`` (bit 10), and
six bits of per-thread state, ``LVL``, ``RUN[2:0]``, ``RVAL``, ``PEND``,
``HALF`` and ``FIRST``, which reset to 0, are cleared by every write to
``BE_CFG`` and by ``CTRL.RESET``, and are readable and writable at debug 0x27.
``SHO`` picks a bit (none for the second half of a Manchester bit, the stuff
bit when one is due, else the data bit of 6.9), does the run accounting,
encodes the bit into a line level and writes it to ``BE_PINS.out`` (plus the
complement on the next index with ``DIFF``); ``SHI`` samples, decodes, drops a
stuff bit or shifts a data bit into ``SR``, and does the run accounting.  A
stuffing or Manchester violation sets ``T``, which ``SHI`` never clears.

Every expected value here comes from a reference written from that text at the
top of the file, never from the model.  Timing reference, as in
``test_loomsim_be.py``: with ``RUN`` written in cycle 0 thread 0's slots have X
cycles 6, 10, 14, ...; a pin write with X cycle ``x`` changes the pad at edge
``x + 2``, and ``pin_in`` of an input pin in X cycle ``x`` is the pad value
held at edge ``x - 1``.
"""

import random

import pytest

from tools.loomisa import load
from tools.loomsim import LoomsimError, Machine
from tools.loomsim.harness import PadTrace, assemble

ISA = load()
CSR = ISA.csr_by_name

#: ``BE_CFG`` fields (SEMANTICS 6.9 and 6.9.1).
DIR = 1 << 1
INV = 1 << 7
CRC_EN = 1 << 9
DIFF = 1 << 10
ENC_SHIFT, STUFF_SHIFT = 3, 5
NRZ, NRZI, MANCHESTER, ENC_RESERVED = 0, 1, 2, 3
NO_STUFF, USB, CAN, STUFF_RESERVED = 0, 1, 2, 3
#: Every bit slice A builds, and every bit it does not (MODE, RXTX, AUTOPULL,
#: 12:11 and the bits above the 13-bit CSR).
BUILT_BITS = DIR | (3 << ENC_SHIFT) | (3 << STUFF_SHIFT) | INV | CRC_EN | DIFF
UNBUILT_BITS = 0xFFFF & ~BUILT_BITS

FEATURES = ("BE", "BEENC")
IN0 = 8                      # pin index of pad ui_in[0]
OUT0 = 16                    # pin index of pad uo_out[0]
FIRST_X = 6                  # X cycle of thread 0's first slot (RUN in cycle 0)
ENC_REG = 0x27               # HOST_PROTOCOL space 4: the encoder state


def cfg(enc=NRZ, stuff=NO_STUFF, dir_=0, inv=0, crc=0, diff=0):
    """A ``BE_CFG`` word from its fields."""
    return ((enc & 3) << ENC_SHIFT) | ((stuff & 3) << STUFF_SHIFT) \
        | (DIR if dir_ else 0) | (INV if inv else 0) \
        | (CRC_EN if crc else 0) | (DIFF if diff else 0)


def be_pins(out=OUT0, pin_in=IN0):
    """``BE_PINS = {in[9:5], out[4:0]}``."""
    return ((pin_in & 0x1F) << 5) | (out & 0x1F)


def be_machine(program, features=FEATURES, thread=0, csrs=None, **kwargs):
    """``program`` at ``thread``'s reset vector, CSRs preloaded by the host."""
    kwargs.setdefault("isa", ISA)
    machine = Machine(assemble(ISA, program, thread * 0x100), features=features,
                      **kwargs)
    for name, value in (csrs or {}).items():
        machine.host_write_debug(thread, name, value)
    return machine


def run_to_halt(machine, thread=0, pads=None, max_cycles=5000):
    """Write ``RUN`` for ``thread`` in this cycle and step until it halts."""
    machine.host_set_run(1 << thread)
    records = []
    for _ in range(max_cycles):
        if pads is not None:
            machine.set_pad_inputs(ui_in=pads(machine.cycle))
        record = machine.step_cycle()
        if record is not None and record.thread == thread:
            records.append(record)
        if (machine.halted >> thread) & 1:
            return records
    raise AssertionError("thread %d did not halt" % thread)


def done(records, name):
    return [r for r in records if r.mnemonic == name and r.done]


def pads_for(levels, first_x=FIRST_X):
    """``ui_in`` per cycle so the ``SHI`` with X cycle ``first_x + 4k`` sees
    ``levels[k]`` on IN0 (the pad value held at edge ``x - 1``)."""
    def pads(cycle):
        k = (cycle + 2 - first_x) // 4
        return levels[k] if 0 <= k < len(levels) else 0
    return pads


# ------------------------------------------------------------- references
# Written from the 6.9.1 text, not from the model.

def random_bits(seed, count):
    rng = random.Random(seed)
    return [rng.getrandbits(1) for _ in range(count)]


def sr_from_bits(bits, msb_first):
    """The ``SR`` value whose transmission order is ``bits`` (at most 16)."""
    if msb_first:
        return sum(bit << (15 - k) for k, bit in enumerate(bits))
    return sum(bit << k for k, bit in enumerate(bits))


def sr_after_shi(bits, msb_first):
    """``SR`` after shifting ``bits`` in from ``SR == 0`` (at most 16 bits)."""
    value = 0
    for bit in bits:
        value = ((value << 1) | bit) & 0xFFFF if msb_first \
            else (bit << 15) | (value >> 1)
    return value


def nrzi_levels(bits, lvl=0):
    """NRZI: a 0 toggles the line, a 1 holds it (6.9.1, USB's convention)."""
    out = []
    for bit in bits:
        if not bit:
            lvl ^= 1
        out.append(lvl)
    return out


def nrzi_decode(levels, lvl=0):
    """The inverse: a bit is 1 where the level did not change."""
    out = []
    for level in levels:
        out.append(int(level == lvl))
        lvl = level
    return out


def manchester_levels(bits):
    """IEEE 802.3: a 0 is high then low, a 1 low then high; two halves a bit."""
    return [half for bit in bits for half in (1 - bit, bit)]


def usb_stuffed(bits):
    """A 0 after six consecutive 1s (``STUFF = 1``)."""
    out, ones = [], 0
    for bit in bits:
        out.append(bit)
        ones = ones + 1 if bit else 0
        if ones == 6:
            out.append(0)               # the stuff bit restarts the count
            ones = 0
    return out


def can_stuffed(bits):
    """The complement after five equal bits (``STUFF = 2``).

    ``RUN`` and ``RVAL`` start at 0, and the stuff bit starts the next run.
    """
    out, run, rval = [], 0, 0
    for bit in bits:
        out.append(bit)
        if bit == rval:
            run = min(run + 1, 7)
        else:
            run, rval = 1, bit
        if run == 5:
            out.append(1 - bit)
            run, rval = 1, 1 - bit
    return out


def crc_reference(poly, init, bits):
    """A textbook serial CRC on the 16-bit register, fed in the order given."""
    crc = init & 0xFFFF
    for bit in bits:
        feedback = ((crc >> 15) & 1) ^ bit
        crc = (crc << 1) & 0xFFFF
        if feedback:
            crc ^= poly & 0xFFFF
    return crc


def test_the_references_agree_with_the_worked_examples_in_the_text():
    """Anchor the references before anything is compared against them."""
    assert nrzi_levels([1, 1, 0, 1, 0, 0]) == [0, 0, 1, 1, 0, 1]
    assert nrzi_decode(nrzi_levels(random_bits(7, 40))) == random_bits(7, 40)
    assert manchester_levels([0, 1]) == [1, 0, 0, 1]
    assert usb_stuffed([1] * 6) == [1] * 6 + [0]
    assert usb_stuffed([1] * 12) == [1] * 6 + [0] + [1] * 6 + [0]
    assert usb_stuffed([1] * 5 + [0, 1]) == [1] * 5 + [0, 1]
    assert can_stuffed([0] * 5) == [0] * 5 + [1]
    assert can_stuffed([1] * 5) == [1] * 5 + [0]
    # The stuff bit starts the next run: nine 0s take one stuff bit, not two.
    assert can_stuffed([0] * 9) == [0] * 5 + [1] + [0] * 4
    assert can_stuffed([0] * 5 + [1] * 4) == [0] * 5 + [1] + [1] * 4 + [0]


# --------------------------------------------------------------- the build
def test_version_reads_3_from_slice_a_on():
    """6.9.1: ``VERSION`` reads 3 from slice A on; the M2 build keeps 2."""
    assert Machine({}, features=FEATURES, isa=ISA).host_read_ctrl("VERSION") == 3
    assert Machine({}, features=("BE",), isa=ISA).host_read_ctrl("VERSION") == 2
    assert Machine({}, isa=ISA).host_read_ctrl("VERSION") == 2
    assert Machine({}, features=FEATURES, version=0x0102,
                   isa=ISA).host_read_ctrl("VERSION") == 0x0102


def test_caps_bit_9_reports_slice_a():
    """SEMANTICS 5 (ruling on question 1): ``CAPS[9]`` is the slice-A encoders,
    on top of ``[4]`` the engine; ``[8]`` (auto mode) stays 0."""
    machine = Machine({}, imem_words=256, features=FEATURES, isa=ISA)
    assert machine.caps == 0x8000 | (1 << 4) | (1 << 9)
    assert Machine({}, imem_words=256, features=("BE",), isa=ISA).caps == 0x8000 | (1 << 4)


def test_the_encoders_need_the_bit_engine():
    with pytest.raises(LoomsimError, match="BEENC"):
        Machine({}, features=("BEENC",), isa=ISA)


# ------------------------------------------------------------ BE_CFG bits
BE_CFG_WRITES = [1 << bit for bit in range(16)] + [
    0xFFFF, BUILT_BITS, UNBUILT_BITS, cfg(enc=ENC_RESERVED),
    cfg(stuff=STUFF_RESERVED), cfg(enc=ENC_RESERVED, stuff=STUFF_RESERVED),
    cfg(enc=MANCHESTER, stuff=CAN, dir_=1, inv=1, crc=1, diff=1)]


def stored(value):
    """``BE_CFG`` as 6.9.1 says it is stored: unbuilt bits drop, and the
    reserved value 3 of ``ENC`` and of ``STUFF`` is stored as 0."""
    value &= BUILT_BITS
    if (value >> ENC_SHIFT) & 3 == 3:
        value &= ~(3 << ENC_SHIFT)
    if (value >> STUFF_SHIFT) & 3 == 3:
        value &= ~(3 << STUFF_SHIFT)
    return value


def test_be_cfg_stores_the_slice_a_fields_and_drops_the_rest_on_csrw():
    program = []
    for value in BE_CFG_WRITES:
        program += [("LDI", dict(rd=1, imm=value & 0xFF)),
                    ("LDIH", dict(rd=1, imm=(value >> 8) & 0xFF)),
                    ("CSRW", dict(csr=CSR["BE_CFG"], ra=1)),
                    ("CSRR", dict(rd=2, csr=CSR["BE_CFG"]))]
    machine = be_machine(program + [("HALT", {})])
    records = run_to_halt(machine)
    reads = [r.val for r in records if r.mnemonic == "CSRR"]
    assert reads == [stored(value) for value in BE_CFG_WRITES]


def test_be_cfg_stores_the_slice_a_fields_and_drops_the_rest_on_a_host_write():
    machine = be_machine([("HALT", {})])
    for value in BE_CFG_WRITES:
        machine.host_write_debug(0, "BE_CFG", value)
        machine.step_cycle()
        assert machine.host_read_debug(0, 0x10 + CSR["BE_CFG"]) == stored(value)
        assert machine.host_read_debug(0, "BE_CFG") == stored(value)


@pytest.mark.parametrize("value", [cfg(enc=ENC_RESERVED), cfg(stuff=STUFF_RESERVED)])
def test_a_reserved_field_value_behaves_as_zero(value):
    """Stored as 0, so ``SHO`` is plain NRZ with no stuffing: the level is the
    data bit and no stuff bit is ever due."""
    bits = [1] * 8
    trace = PadTrace()
    machine = be_machine([("SHO", {})] * 8 + [("HALT", {})], on_cycle=trace,
                         csrs={"SR": sr_from_bits(bits, False), "BE_CFG": value,
                               "BE_PINS": be_pins(out=OUT0)})
    records = run_to_halt(machine)
    pad = trace.uo_bit(0)
    assert [pad[r.x_cycle + 2] for r in done(records, "SHO")] == bits
    assert machine.host_read_debug(0, ENC_REG) == 0


def test_the_bits_slice_a_does_not_build_read_zero_and_ignore_writes():
    """``MODE``, ``RXTX``, ``AUTOPULL`` and 12:11 wait for slice C."""
    machine = be_machine([("SHO", {})] * 2 + [("HALT", {})],
                         csrs={"BE_CFG": UNBUILT_BITS, "SR": 0x0001,
                               "BE_PINS": be_pins(out=OUT0)})
    run_to_halt(machine)
    assert machine.host_read_debug(0, "BE_CFG") == 0
    # NRZ, LSB first, no stuffing, no second pin: SR = 1 sends 1 then 0.
    assert machine.uo_out == 0
    assert machine.threads[0].sr == 0
    assert machine.host_read_debug(0, ENC_REG) == 0


def test_an_m2_build_ignores_the_slice_a_fields_entirely():
    """Without ``BEENC`` the new bits read 0 and ``SHO`` is 6.9 as it was."""
    machine = be_machine([("SHO", {}), ("HALT", {})], features=("BE",),
                         csrs={"BE_CFG": cfg(enc=NRZI, stuff=USB, diff=1),
                               "SR": 0x0000, "BE_PINS": be_pins(out=OUT0)})
    run_to_halt(machine)
    assert machine.host_read_debug(0, "BE_CFG") == 0
    assert machine.uo_out == 0              # NRZI would have toggled to 1
    assert machine.host_read_debug(0, ENC_REG) == 0
    assert machine.host_read_debug(0, "ENC_LVL") == 0


# ------------------------------------------------------------------- NRZI
@pytest.mark.parametrize("inv", [0, 1])
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_nrzi_transmit_holds_the_level_on_a_one_and_toggles_on_a_zero(seed, inv):
    bits = random_bits(seed, 16)
    trace = PadTrace()
    machine = be_machine([("SHO", {})] * 16 + [("HALT", {})], on_cycle=trace,
                         csrs={"SR": sr_from_bits(bits, False),
                               "BE_CFG": cfg(enc=NRZI, inv=inv),
                               "BE_PINS": be_pins(out=OUT0)})
    records = run_to_halt(machine)
    shos = done(records, "SHO")
    assert [r.x_cycle for r in shos] == [FIRST_X + 4 * k for k in range(16)]
    levels = nrzi_levels(bits)
    pad = trace.uo_bit(0)
    assert [pad[r.x_cycle + 2] for r in shos] == [level ^ inv for level in levels]
    # ``LVL`` is the level before ``INV``, which the pin write applies (6.9.1).
    assert machine.host_read_debug(0, "ENC_LVL") == levels[-1]
    assert machine.threads[0].sr == 0        # all sixteen bits shifted out


@pytest.mark.parametrize("inv", [0, 1])
@pytest.mark.parametrize("seed", [4, 5, 6])
def test_nrzi_receive_is_the_inverse_of_transmit(seed, inv):
    bits = random_bits(seed, 12)
    levels = nrzi_levels(bits)
    machine = be_machine([("SHI", {})] * 12 + [("HALT", {})],
                         csrs={"BE_CFG": cfg(enc=NRZI, inv=inv, dir_=1, crc=1),
                               "CRC_POLY": 0x8005, "CRC": 0x0000,
                               "BE_PINS": be_pins(pin_in=IN0)})
    records = run_to_halt(machine, pads=pads_for([level ^ inv for level in levels]))
    assert [r.x_cycle for r in done(records, "SHI")] == \
        [FIRST_X + 4 * k for k in range(12)]
    assert nrzi_decode(levels) == bits                    # the reference agrees
    assert machine.threads[0].sr == sr_after_shi(bits, True)
    assert machine.host_read_debug(0, "ENC_LVL") == levels[-1]
    assert machine.threads[0].t == 0                      # NRZI sets no T
    # The CRC sees the decoded bits, the ones that entered SR, not the levels.
    assert machine.threads[0].crc == crc_reference(0x8005, 0x0000, bits)


def test_nrzi_starts_from_the_level_the_host_wrote_at_debug_0x27():
    machine = be_machine([("SHO", {}), ("HALT", {})],
                         csrs={"SR": 0x0001, "BE_CFG": cfg(enc=NRZI),
                               "BE_PINS": be_pins(out=OUT0), ENC_REG: 1})
    run_to_halt(machine)
    assert machine.uo_out == 1              # a 1 held the level the host set
    assert machine.host_read_debug(0, "ENC_LVL") == 1


# ------------------------------------------------------------- Manchester
@pytest.mark.parametrize("inv", [0, 1])
def test_manchester_transmit_is_two_sho_per_bit_inverted_then_true(inv):
    bits = random_bits(11, 8)
    trace = PadTrace()
    machine = be_machine([("SHO", {})] * 16 + [("HALT", {})], on_cycle=trace,
                         csrs={"SR": sr_from_bits(bits, True), "CNT": 8,
                               "BE_CFG": cfg(enc=MANCHESTER, dir_=1, inv=inv),
                               "BE_PINS": be_pins(out=OUT0 + 2)})
    records = run_to_halt(machine)
    shos = done(records, "SHO")
    pad = trace.uo_bit(2)
    assert [pad[r.x_cycle + 2] for r in shos] == \
        [level ^ inv for level in manchester_levels(bits)]
    # Only the first half is a data bit: CNT counts bits, not halves, and Z
    # follows the CNT after the instruction whether or not it changed.
    assert [r.z for r in shos] == [0] * 14 + [1, 1]
    assert machine.threads[0].cnt == 0
    assert machine.host_read_debug(0, "ENC_HALF") == 0     # ended on a full bit
    assert machine.threads[0].t == 0


def test_manchester_transmit_leaves_half_set_between_the_two_sho():
    machine = be_machine([("SHO", {}), ("HALT", {})],
                         csrs={"SR": 0x0001, "CNT": 4,
                               "BE_CFG": cfg(enc=MANCHESTER),
                               "BE_PINS": be_pins(out=OUT0)})
    run_to_halt(machine)
    assert machine.uo_out == 0              # first half of a 1 is low
    assert (machine.host_read_debug(0, "ENC_HALF"),
            machine.host_read_debug(0, "ENC_FIRST")) == (1, 1)
    assert (machine.threads[0].sr, machine.threads[0].cnt) == (0x0000, 3)


def test_manchester_receive_takes_the_second_half_and_checks_the_transition():
    bits = random_bits(12, 8)
    machine = be_machine([("SHI", {})] * 16 + [("HALT", {})],
                         csrs={"BE_CFG": cfg(enc=MANCHESTER, dir_=1), "CNT": 8,
                               "BE_PINS": be_pins(pin_in=IN0)})
    records = run_to_halt(machine, pads=pads_for(manchester_levels(bits)))
    shis = done(records, "SHI")
    assert len(shis) == 16
    assert machine.threads[0].sr == sr_after_shi(bits, True)
    assert machine.threads[0].t == 0        # every bit had a mid-bit transition
    assert machine.host_read_debug(0, "ENC_HALF") == 0
    # CNT (and so Z) moves only on the second half, where the bit arrives.
    assert [r.z for r in shis] == [0] * 15 + [1]
    assert machine.threads[0].cnt == 0


def test_a_manchester_bit_without_a_mid_bit_transition_sets_t():
    """6.9.1: with ``HALF == 1``, ``T <= 1`` if ``FIRST == p``.  The bit is
    still received, and ``SHI`` never clears ``T`` again."""
    levels = [0, 1,      # a clean 1
              1, 1,      # no transition: a violation, s = 1
              1, 0]      # a clean 0
    machine = be_machine([("SHI", {})] * 6 + [("HALT", {})],
                         csrs={"BE_CFG": cfg(enc=MANCHESTER, dir_=1),
                               "BE_PINS": be_pins(pin_in=IN0), "FLAGS": 0b010})
    records = run_to_halt(machine, pads=pads_for(levels))
    shis = done(records, "SHI")
    assert [r.t for r in shis] == [0, 0, 0, 1, 1, 1]   # set there, never cleared
    assert [r.c for r in shis] == [1] * 6              # C is left alone
    assert machine.threads[0].sr == sr_after_shi([1, 1, 0], True)
    assert machine.threads[0].t == 1


def test_a_manchester_half_bit_leaves_sr_cnt_and_the_crc_alone():
    machine = be_machine([("SHI", {}), ("HALT", {})],
                         csrs={"BE_CFG": cfg(enc=MANCHESTER, crc=1), "CNT": 7,
                               "SR": 0xBEEF, "CRC": 0x1234, "CRC_POLY": 0x8005,
                               "BE_PINS": be_pins(pin_in=IN0)})
    records = run_to_halt(machine, pads=lambda cycle: 0xFF)
    th = machine.threads[0]
    assert (th.sr, th.cnt, th.crc) == (0xBEEF, 7, 0x1234)
    assert (th.enc_half, th.enc_first) == (1, 1)
    assert records[0].z == 0                # Z = (CNT == 0) after, CNT = 7


# ----------------------------------------------------------- USB stuffing
def test_usb_transmit_inserts_a_zero_after_six_ones():
    bits = [1] * 6 + [0, 1, 1, 1, 1, 1, 1, 1]
    sent = usb_stuffed(bits)
    trace = PadTrace()
    machine = be_machine([("SHO", {})] * len(sent) + [("HALT", {})], on_cycle=trace,
                         csrs={"SR": sr_from_bits(bits, False), "CNT": 31,
                               "BE_CFG": cfg(stuff=USB, crc=1),
                               "CRC_POLY": 0x8005, "CRC": 0x0000,
                               "BE_PINS": be_pins(out=OUT0)})
    records = run_to_halt(machine)
    shos = done(records, "SHO")
    pad = trace.uo_bit(0)
    assert [pad[r.x_cycle + 2] for r in shos] == sent
    th = machine.threads[0]
    # The stuff bits leave SR, CNT and the CRC alone: two were inserted.
    assert len(sent) == len(bits) + 2
    assert th.cnt == 31 - len(bits)
    assert th.sr == sr_from_bits(bits, False) >> len(bits)
    assert th.crc == crc_reference(0x8005, 0x0000, bits)
    assert (th.enc_pend, th.enc_run, th.enc_rval) == (0, 1, 1)


def test_the_usb_stuff_bit_leaves_cnt_and_z_where_they_were():
    """6.9.1: the stuff bit is not a data bit, and ``Z = (CNT == 0)`` after the
    instruction whether or not ``CNT`` changed."""
    machine = be_machine([("SHO", {})] * 7 + [("HALT", {})],
                         csrs={"SR": 0x003F, "CNT": 6,
                               "BE_CFG": cfg(stuff=USB),
                               "BE_PINS": be_pins(out=OUT0)})
    records = run_to_halt(machine)
    shos = done(records, "SHO")
    assert [r.z for r in shos] == [0, 0, 0, 0, 0, 1, 1]
    assert machine.threads[0].cnt == 0       # six data bits, one stuff bit


def test_usb_receive_drops_the_stuff_bit():
    bits = [1] * 6 + [0, 1, 0, 1, 1, 1]
    received = usb_stuffed(bits)
    machine = be_machine([("SHI", {})] * len(received) + [("HALT", {})],
                         csrs={"BE_CFG": cfg(stuff=USB, dir_=1, crc=1), "CNT": 31,
                               "CRC_POLY": 0x8005, "CRC": 0x0000,
                               "BE_PINS": be_pins(pin_in=IN0)})
    records = run_to_halt(machine, pads=pads_for(received))
    assert len(done(records, "SHI")) == len(received) == len(bits) + 1
    th = machine.threads[0]
    assert th.sr == sr_after_shi(bits, True)
    assert th.cnt == 31 - len(bits)
    assert th.t == 0                         # a correct stuff bit is no violation
    assert th.crc == crc_reference(0x8005, 0x0000, bits)    # not the stuff bit


def test_a_seventh_one_is_a_usb_stuffing_violation():
    """After six 1s a 0 is due; anything else sets ``T`` and is still dropped."""
    machine = be_machine([("SHI", {})] * 8 + [("HALT", {})],
                         csrs={"BE_CFG": cfg(stuff=USB, dir_=1), "CNT": 31,
                               "BE_PINS": be_pins(pin_in=IN0)})
    records = run_to_halt(machine, pads=pads_for([1] * 8))
    shis = done(records, "SHI")
    assert [r.t for r in shis] == [0] * 6 + [1, 1]
    th = machine.threads[0]
    assert th.sr == sr_after_shi([1] * 6 + [1], True)   # six, then the eighth
    assert th.cnt == 31 - 7
    assert th.t == 1


# ----------------------------------------------------------- CAN stuffing
@pytest.mark.parametrize("bits,expected", [
    ([0] * 5 + [1, 1], None),
    ([1] * 5 + [0, 0], None),
    ([0] * 9, None),                        # the stuff bit starts the next run
    ([1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1], None),
])
def test_can_transmit_inserts_the_complement_after_five_equal_bits(bits, expected):
    sent = can_stuffed(bits)
    assert expected is None or sent == expected
    trace = PadTrace()
    machine = be_machine([("SHO", {})] * len(sent) + [("HALT", {})], on_cycle=trace,
                         csrs={"SR": sr_from_bits(bits, True), "CNT": 31,
                               "BE_CFG": cfg(stuff=CAN, dir_=1),
                               "BE_PINS": be_pins(out=OUT0 + 4)})
    records = run_to_halt(machine)
    pad = trace.uo_bit(4)
    assert [pad[r.x_cycle + 2] for r in done(records, "SHO")] == sent
    assert machine.threads[0].cnt == 31 - len(bits)


def test_can_receive_drops_the_stuff_bit_and_flags_a_violation():
    bits = [1, 1, 1, 1, 1, 0, 0, 0, 0, 0]
    received = can_stuffed(bits)
    # Five 1s, the 0 that stuffs them, four more 0s, the 1 that stuffs those,
    # and the tenth data bit.
    assert received == [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0]
    machine = be_machine([("SHI", {})] * len(received) + [("HALT", {})],
                         csrs={"BE_CFG": cfg(stuff=CAN, dir_=1), "CNT": 31,
                               "BE_PINS": be_pins(pin_in=IN0)})
    records = run_to_halt(machine, pads=pads_for(received))
    th = machine.threads[0]
    assert th.sr == sr_after_shi(bits, True) and th.t == 0
    assert th.cnt == 31 - len(bits)
    assert len(done(records, "SHI")) == len(bits) + 2

    # The same stream with the second stuff bit sent as a copy of the run
    # instead of its complement: a violation, and the bit is still dropped.
    wrong = received[:10] + [0] + received[11:]
    machine = be_machine([("SHI", {})] * len(wrong) + [("HALT", {})],
                         csrs={"BE_CFG": cfg(stuff=CAN, dir_=1), "CNT": 31,
                               "BE_PINS": be_pins(pin_in=IN0)})
    records = run_to_halt(machine, pads=pads_for(wrong))
    assert [r.t for r in done(records, "SHI")] == [0] * 10 + [1, 1]
    th = machine.threads[0]
    assert th.t == 1 and th.sr == sr_after_shi(bits, True)
    assert th.cnt == 31 - len(bits)         # the bad stuff bit was dropped too


def test_the_can_stuff_bit_starts_the_next_run_on_receive():
    """Nine 0s carry one stuff bit; a model that kept counting would expect
    a second one and flag the ninth 0 as a violation."""
    bits = [0] * 9
    received = can_stuffed(bits)
    assert len(received) == 10
    machine = be_machine([("SHI", {})] * len(received) + [("HALT", {})],
                         csrs={"BE_CFG": cfg(stuff=CAN, dir_=1), "CNT": 31,
                               "BE_PINS": be_pins(pin_in=IN0)})
    run_to_halt(machine, pads=pads_for(received))
    th = machine.threads[0]
    assert (th.sr, th.t) == (sr_after_shi(bits, True), 0)
    assert (th.enc_run, th.enc_rval, th.enc_pend) == (4, 0, 0)


def test_the_run_state_is_not_touched_without_stuffing():
    """6.9.1: with ``STUFF == 0`` the run state is not updated and ``PEND``
    stays 0, however long the run of equal bits is."""
    machine = be_machine([("SHO", {})] * 8 + [("HALT", {})],
                         csrs={"SR": 0x00FF, "BE_CFG": cfg(),
                               "BE_PINS": be_pins(out=OUT0)})
    run_to_halt(machine)
    assert machine.host_read_debug(0, ENC_REG) == 0


# -------------------------------------------------------------------- DIFF
@pytest.mark.parametrize("inv", [0, 1])
def test_diff_writes_the_complement_to_the_next_pin_index(inv):
    bits = [1, 0, 0, 1, 1, 0]
    trace = PadTrace()
    machine = be_machine([("SHO", {})] * len(bits) + [("HALT", {})], on_cycle=trace,
                         csrs={"SR": sr_from_bits(bits, False),
                               "BE_CFG": cfg(diff=1, inv=inv),
                               "BE_PINS": be_pins(out=OUT0)})
    records = run_to_halt(machine)
    shos = done(records, "SHO")
    plus, minus = trace.uo_bit(0), trace.uo_bit(1)
    assert [plus[r.x_cycle + 2] for r in shos] == [bit ^ inv for bit in bits]
    assert [minus[r.x_cycle + 2] for r in shos] == [1 - (bit ^ inv) for bit in bits]
    # Both writes land at the same edge, as one OUT of a two-pin group would:
    # the pads never change anywhere but at a SHO's edge x + 2.
    changed = [cycle for cycle, _ in trace.changes()]
    assert changed and set(changed) <= {r.x_cycle + 2 for r in shos}


def test_diff_wraps_the_index_and_ignores_one_that_is_not_writable():
    # out = 31: the level is dropped (31 is not writable), the complement
    # wraps to index 0, which is.
    machine = be_machine([("SHO", {}), ("HALT", {})],
                         csrs={"SR": 0x0001, "BE_CFG": cfg(diff=1),
                               "BE_PINS": be_pins(out=31)})
    run_to_halt(machine)
    assert (machine.pin_out, machine.uo_out) == (0, 0)
    assert machine.uio_out == 0             # the complement of a 1 is a 0
    assert machine.threads[0].sr == 0       # the shift happened all the same

    # out = 21 (OUT5): the complement index 22 is not writable, so only the
    # level lands.
    machine = be_machine([("SHO", {}), ("HALT", {})],
                         csrs={"SR": 0x0001, "BE_CFG": cfg(diff=1),
                               "BE_PINS": be_pins(out=21)})
    run_to_halt(machine)
    assert machine.uo_out == 1 << 5
    assert machine.pin_out == 1 << 13


def test_diff_applies_the_open_drain_rule_to_both_pins():
    """6.3 on BIDIR1: ``PIN_OUT <= 0`` and ``PIN_OE <= ~b`` for the complement."""
    machine = be_machine([("SHO", {}), ("HALT", {})],
                         csrs={"SR": 0x0001, "BE_CFG": cfg(diff=1),
                               "BE_PINS": be_pins(out=0)})
    machine.host_write_od_mask(0b10)        # pin 1 open drain, pin 0 push-pull
    machine.host_write_pin_oe(0b11)
    run_to_halt(machine)
    # pin 0 takes the level 1; pin 1 takes the complement 0 through open drain.
    assert machine.pin_out == 0b01
    assert machine.pin_oe == 0b11           # ~0 = 1 on the open-drain pin
    assert machine.uio_out == 0b01


def test_a_diff_pair_carries_nrzi_and_stuffing_together():
    """The USB low-speed shape: stuffing, then NRZI, then D+/D-."""
    bits = [1] * 7 + [0, 1, 1]
    sent = usb_stuffed(bits)
    levels = nrzi_levels(sent)
    trace = PadTrace()
    machine = be_machine([("SHO", {})] * len(sent) + [("HALT", {})], on_cycle=trace,
                         csrs={"SR": sr_from_bits(bits, False), "CNT": len(bits),
                               "BE_CFG": cfg(enc=NRZI, stuff=USB, diff=1),
                               "BE_PINS": be_pins(out=OUT0)})
    records = run_to_halt(machine)
    shos = done(records, "SHO")
    dp, dm = trace.uo_bit(0), trace.uo_bit(1)
    assert [dp[r.x_cycle + 2] for r in shos] == levels
    assert [dm[r.x_cycle + 2] for r in shos] == [1 - level for level in levels]
    assert machine.threads[0].cnt == 0 and shos[-1].z == 1


# ------------------------------------------------- clearing the state
def test_a_csrw_of_be_cfg_clears_the_encoder_state():
    """6.9.1: the state is cleared by every write to ``BE_CFG``, so the NRZI
    level restarts at 0 and the run count at 0."""
    value = cfg(enc=NRZI, stuff=USB)
    program = [("SHO", {})] * 3 + [
        ("CSRR", dict(rd=3, csr=CSR["BE_CFG"])),        # the state so far
        ("LDI", dict(rd=1, imm=value & 0xFF)),
        ("LDIH", dict(rd=1, imm=(value >> 8) & 0xFF)),
        ("CSRW", dict(csr=CSR["BE_CFG"], ra=1)),        # same value, still a write
        ("SHO", {}), ("HALT", {})]
    trace = PadTrace()
    machine = be_machine(program, on_cycle=trace,
                         csrs={"SR": 0x000F, "BE_CFG": value,
                               "BE_PINS": be_pins(out=OUT0)})
    records = run_to_halt(machine)
    shos = done(records, "SHO")
    pad = trace.uo_bit(0)
    # Three 1s hold the level at 0; after the clear the fourth 1 holds 0 again
    # from a cleared LVL, and the run of 1s starts over.
    assert [pad[r.x_cycle + 2] for r in shos] == [0, 0, 0, 0]
    assert machine.host_read_debug(0, "ENC_RUN") == 1
    assert machine.host_read_debug(0, "BE_CFG") == value


def test_a_host_write_of_be_cfg_clears_the_encoder_state():
    machine = be_machine([("HALT", {})], csrs={ENC_REG: 0xFF})
    machine.step_cycle()
    assert machine.host_read_debug(0, ENC_REG) == 0xFF
    machine.host_write_debug(0, "BE_CFG", cfg(enc=MANCHESTER))
    machine.step_cycle()
    assert machine.host_read_debug(0, ENC_REG) == 0
    assert machine.host_read_debug(0, "BE_CFG") == cfg(enc=MANCHESTER)


def test_ctrl_reset_clears_the_encoder_state():
    """SEMANTICS 7: from M3 ``CTRL.RESET`` clears the encoder state too.

    (``BE_CFG`` is written first: two host writes in one cycle land at the
    same edge in the order they were made, and a ``BE_CFG`` write clears the
    state whoever made it.)
    """
    machine = be_machine([("HALT", {})], csrs={"BE_CFG": cfg(enc=NRZI), ENC_REG: 0xFF})
    machine.step_cycle()
    assert machine.host_read_debug(0, ENC_REG) == 0xFF
    machine.host_reset_thread(0)
    machine.step_cycle()
    assert machine.host_read_debug(0, ENC_REG) == 0
    assert machine.host_read_debug(0, "BE_CFG") == cfg(enc=NRZI)   # CSRs untouched


def test_other_be_csr_writes_leave_the_encoder_state_alone():
    machine = be_machine([("HALT", {})], csrs={ENC_REG: 0xFF})
    machine.step_cycle()
    for name, value in (("BE_PINS", 0x1234), ("SR", 0xFFFF), ("CNT", 4),
                        ("CRC", 0x55), ("CRC_POLY", 0x8005), ("CRC_INIT", 1),
                        ("BE_RELOAD", 3)):
        machine.host_write_debug(0, name, value)
        machine.step_cycle()
    assert machine.host_read_debug(0, ENC_REG) == 0xFF


# ------------------------------------------------------------ debug 0x27
@pytest.mark.parametrize("field,bit", [
    ("ENC_LVL", 0), ("ENC_RVAL", 4), ("ENC_PEND", 5), ("ENC_HALF", 6),
    ("ENC_FIRST", 7),
])
def test_debug_0x27_packs_each_field_where_host_protocol_says(field, bit):
    machine = be_machine([("HALT", {})])
    machine.host_write_debug(0, ENC_REG, 1 << bit)
    machine.step_cycle()
    assert machine.host_read_debug(0, ENC_REG) == 1 << bit
    assert machine.host_read_debug(0, field) == 1
    assert sum(machine.host_read_debug(0, name) for name in
               ("ENC_LVL", "ENC_RVAL", "ENC_PEND", "ENC_HALF", "ENC_FIRST")) == 1


@pytest.mark.parametrize("run", range(8))
def test_debug_0x27_carries_run_in_bits_3_to_1(run):
    machine = be_machine([("HALT", {})])
    machine.host_write_debug(0, ENC_REG, run << 1)
    machine.step_cycle()
    assert machine.host_read_debug(0, ENC_REG) == run << 1
    assert machine.host_read_debug(0, "ENC_RUN") == run


def test_debug_0x27_ignores_the_bits_above_seven_and_is_dropped_while_running():
    machine = be_machine([("NOP", {})] * 40 + [("HALT", {})])
    machine.host_write_debug(0, ENC_REG, 0xFFFF)
    machine.step_cycle()
    assert machine.host_read_debug(0, ENC_REG) == 0x00FF
    machine.host_set_run(0b0001)
    machine.run_cycles(8)
    machine.host_write_debug(0, ENC_REG, 0x0000)        # dropped: thread runs
    machine.run_cycles(2)
    assert machine.host_read_debug(0, ENC_REG) == 0x00FF


def test_debug_0x27_reads_zero_until_slice_a_is_built():
    for features in ((), ("BE",)):
        machine = be_machine([("HALT", {})], features=features)
        machine.host_write_debug(0, ENC_REG, 0xFF)
        machine.step_cycle()
        assert machine.host_read_debug(0, ENC_REG) == 0
        assert machine.host_read_debug(0, "ENC") == 0
        assert machine.host_read_debug(0, "ENC_RUN") == 0
        assert max(machine.dump_debug_space(0)) == 0x26
    machine = be_machine([("HALT", {})])
    assert max(machine.dump_debug_space(0)) == ENC_REG


def test_the_state_the_host_writes_is_what_the_next_sho_uses():
    """``PEND = 1`` with USB stuffing: the next ``SHO`` sends the stuff bit,
    not the data bit, and leaves ``SR`` and ``CNT`` alone."""
    machine = be_machine([("SHO", {}), ("HALT", {})],
                         csrs={"SR": 0x0001, "CNT": 4, "BE_CFG": cfg(stuff=USB),
                               "BE_PINS": be_pins(out=OUT0),
                               ENC_REG: (1 << 5) | (6 << 1) | (1 << 4)})
    run_to_halt(machine)
    th = machine.threads[0]
    assert machine.uo_out == 0              # the USB stuff bit is a 0
    assert (th.sr, th.cnt) == (0x0001, 4)
    assert (th.enc_pend, th.enc_run, th.enc_rval) == (0, 1, 0)


# ------------------------------------------------------------- the placing
def test_the_pin_writes_land_at_edge_x_plus_two_and_hold():
    """Section 2, as ``test_loomsim_be.py`` checks it for the M2 engine."""
    bits = [1, 0, 1, 1]
    trace = PadTrace()
    machine = be_machine([("SHO", {})] * len(bits) + [("HALT", {})], on_cycle=trace,
                         csrs={"SR": sr_from_bits(bits, False),
                               "BE_CFG": cfg(enc=NRZI, diff=1),
                               "BE_PINS": be_pins(out=OUT0)})
    records = run_to_halt(machine)
    shos = done(records, "SHO")
    levels = nrzi_levels(bits)
    dp, dm = trace.uo_bit(0), trace.uo_bit(1)
    assert dp[:FIRST_X + 2] == [0] * (FIRST_X + 2)
    assert dm[:FIRST_X + 2] == [0] * (FIRST_X + 2)
    for k, record in enumerate(shos):
        start = record.x_cycle + 2
        end = shos[k + 1].x_cycle + 2 if k + 1 < len(shos) else len(dp)
        assert dp[start:end] == [levels[k]] * (end - start), k
        assert dm[start:end] == [1 - levels[k]] * (end - start), k


# ----------------------------------------------------------- per thread
def test_each_thread_has_its_own_encoder_state():
    image = assemble(ISA, [("SHO", {})] * 2 + [("HALT", {})], 0x000)
    image.update(assemble(ISA, [("SHO", {})] * 2 + [("HALT", {})], 0x100))
    machine = Machine(image, features=FEATURES, isa=ISA)
    setups = [{"SR": 0x0000, "BE_CFG": cfg(enc=NRZI), "BE_PINS": be_pins(out=OUT0)},
              {"SR": 0x0003, "BE_CFG": cfg(stuff=CAN), "BE_PINS": be_pins(out=OUT0 + 1)}]
    for thread, csrs in enumerate(setups):
        for name, value in csrs.items():
            machine.host_write_debug(thread, name, value)
    machine.host_set_run(0b0011)
    assert machine.run_until(lambda m: m.halted == 0b0011, max_cycles=200).fired
    # Thread 0 sent two 0s (NRZI: toggle, toggle) and thread 1 two 1s (NRZ).
    assert machine.uo_out == 0b10
    assert [(th.enc_lvl, th.enc_run, th.enc_rval) for th in machine.threads] == \
        [(0, 0, 0), (0, 2, 1), (0, 0, 0), (0, 0, 0)]
