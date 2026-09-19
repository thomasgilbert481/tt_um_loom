"""The bit engine in manual mode, SEMANTICS 6.9, on the golden model (feature ``"BE"``).

``SHO`` sends ``b = DIR ? SR[15] : SR[0]`` as one pin write (6.3 rules, open
drain included) of ``b ^ INV`` to ``BE_PINS.out`` and shifts ``SR`` towards
the end it sent from; ``SHI`` shifts ``s = pin_in(BE_PINS.in) ^ INV`` in at
the other end, so after ``n`` LSB-first bits the value sits in
``SR[15:16-n]``.  Both count ``CNT`` down to a floor of 0, set ``Z`` from the
new ``CNT`` and leave ``C`` and ``T`` alone, and with ``CRC_EN`` they feed the
serial MSB-first CRC on the left-aligned register (``b`` for ``SHO``, ``s``
for ``SHI``).  At M2 ``BE_CFG`` has only ``DIR`` (bit 1), ``INV`` (bit 7) and
``CRC_EN`` (bit 9); its other bits read 0 and ignore writes.  Without the
feature every bit-engine instruction is a ``NOP`` that sets ``BADOP``
(section 9), and so is ``WAITB 0`` (6.4).

Every expected value here is computed from the text, never read back from the
model.  Timing reference: with ``RUN`` written in cycle 0, thread 0's slots
have X cycles 6, 10, 14, ...; a pin write with X cycle ``x`` changes the pad
at edge ``x + 2`` (section 2), and ``pin_in`` of an input pin in X cycle
``x`` is the pad value held at edge ``x - 1`` (section 3).
"""

import pytest

from tools.loomisa import load
from tools.loomsim import Machine
from tools.loomsim.harness import PadTrace, assemble

ISA = load()
CSR = ISA.csr_by_name

DIR = 1 << 1                 # BE_CFG.DIR: 0 LSB first, 1 MSB first
INV = 1 << 7                 # BE_CFG.INV: invert the pin level
CRC_EN = 1 << 9              # BE_CFG.CRC_EN
M2_CFG_BITS = DIR | INV | CRC_EN
IN0 = 8                      # pin index of pad ui_in[0]
OUT0 = 16                    # pin index of pad uo_out[0]
FIRST_X = 6                  # X cycle of thread 0's first slot (RUN in cycle 0)
SR_PATTERN = 0xB38D          # 1011 0011 1000 1101: asymmetric either way round


def be_pins(out=OUT0, pin_in=IN0):
    """``BE_PINS = {in[9:5], out[4:0]}``."""
    return ((pin_in & 0x1F) << 5) | (out & 0x1F)


def load16(rd, value):
    """``LDI rd, lo; LDIH rd, hi``: a 16-bit constant (the MOV16 expansion)."""
    return [("LDI", dict(rd=rd, imm=value & 0xFF)),
            ("LDIH", dict(rd=rd, imm=(value >> 8) & 0xFF))]


def be_machine(program, features=("BE",), thread=0, csrs=None, **kwargs):
    """``program`` at ``thread``'s reset vector, with debug registers (CSRs by
    name, ``FLAGS``, ``r0..r7`` ...) preloaded by the host before ``RUN``."""
    kwargs.setdefault("isa", ISA)
    machine = Machine(assemble(ISA, program, thread * 0x100), features=features,
                      **kwargs)
    for name, value in (csrs or {}).items():
        machine.host_write_debug(thread, name, value)
    return machine


def run_to_halt(machine, thread=0, pads=None, max_cycles=5000):
    """Write ``RUN`` for ``thread`` in this cycle and step until it halts.

    ``pads(cycle)``, if given, is the ``ui_in`` value held at the edge that
    ends ``cycle``.  Returns that thread's retire records.
    """
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
    """``ui_in`` per cycle such that the SHI with X cycle ``first_x + 4k`` sees
    ``levels[k]`` on IN0: the pad value held at edge ``x - 1``, which is the
    value set before stepping cycle ``x - 2``."""
    def pads(cycle):
        k = (cycle + 2 - first_x) // 4
        return levels[k] if 0 <= k < len(levels) else 0
    return pads


# ------------------------------------------------------------------- SHO
@pytest.mark.parametrize("cfg", [0, DIR, INV, DIR | INV])
def test_sho_sends_sr_in_the_configured_order_and_level(cfg):
    n = 11
    trace = PadTrace()
    machine = be_machine([("SHO", {})] * n + [("STSR", dict(rd=1)), ("HALT", {})],
                         csrs={"SR": SR_PATTERN, "BE_CFG": cfg,
                               "BE_PINS": be_pins(out=OUT0 + 3, pin_in=IN0 + 1)},
                         on_cycle=trace)
    records = run_to_halt(machine)
    msb_first, inv = bool(cfg & DIR), int(bool(cfg & INV))
    order = range(15, 15 - n, -1) if msb_first else range(n)
    levels = [((SR_PATTERN >> i) & 1) ^ inv for i in order]
    shos = done(records, "SHO")
    assert [r.x_cycle for r in shos] == [FIRST_X + 4 * k for k in range(n)]
    pad = trace.uo_bit(3)                            # BE_PINS.out = OUT3
    assert pad[:FIRST_X + 2] == [0] * (FIRST_X + 2)
    for k, record in enumerate(shos):
        # Each level is on the pad from edge x + 2 up to the next SHO's edge.
        start = record.x_cycle + 2
        end = shos[k + 1].x_cycle + 2 if k + 1 < n else len(pad)
        assert pad[start:end] == [levels[k]] * (end - start), k
    left = (SR_PATTERN << n) & 0xFFFF if msb_first else SR_PATTERN >> n
    assert records[-2].mnemonic == "STSR" and records[-2].val == left
    assert machine.threads[0].sr == left
    assert machine.pin_out == ((machine.pin_out >> 11) & 1) << 11   # only OUT3


@pytest.mark.parametrize("sr,cfg,od,level", [
    (1, 0, 0, 1), (0, 0, 0, 0), (1, INV, 0, 0), (0, INV, 0, 1),
    (1, 0, 1, 1), (0, 0, 1, 0), (1, INV, 1, 0), (0, INV, 1, 1),
])
def test_sho_is_one_pin_write_with_the_open_drain_rule(sr, cfg, od, level):
    """6.3 on BIDIR2 with ``b ^ INV`` as the value: a push-pull pin takes the
    level and keeps its OE; an open-drain pin gets ``PIN_OUT <= 0`` and
    ``PIN_OE <= ~level``.  Every other bit of both registers is untouched."""
    pin = 2
    expect_out = 0 if od else level
    expect_oe = 1 - level if od else 1
    before_oe = level if od else 1                   # the opposite, where it moves
    machine = be_machine([("SHO", {}), ("HALT", {})],
                         csrs={"SR": sr, "BE_CFG": cfg, "BE_PINS": be_pins(out=pin)})
    machine.host_write_od_mask(od << pin)
    machine.host_write_pin_out(0xA5A5 & ~(1 << pin) | ((1 - expect_out) << pin))
    machine.host_write_pin_oe(0x5A & ~(1 << pin) | (before_oe << pin))
    run_to_halt(machine)
    assert machine.pin_out == 0xA5A5 & ~(1 << pin) | (expect_out << pin)
    assert machine.pin_oe == 0x5A & ~(1 << pin) | (expect_oe << pin)
    assert machine.badop == 0


@pytest.mark.parametrize("pin,pin_out", [
    (0, 1 << 0), (7, 1 << 7), (16, 1 << 8), (21, 1 << 13),       # writable
    (8, 0), (12, 0), (13, 0), (15, 0), (22, 0), (31, 0),         # ignored
])
def test_sho_writes_only_writable_indices_but_always_shifts(pin, pin_out):
    machine = be_machine([("SHO", {}), ("HALT", {})],
                         csrs={"SR": 0x0003, "CNT": 4, "BE_PINS": be_pins(out=pin)})
    records = run_to_halt(machine)
    assert (machine.pin_out, machine.pin_oe) == (pin_out, 0)
    assert (machine.threads[0].sr, machine.threads[0].cnt) == (0x0001, 3)
    assert records[0].done and machine.badop == 0


# ------------------------------------------------------------------- SHI
SHI_LEVELS = [1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1]      # pad levels on IN0, one per SHI


@pytest.mark.parametrize("cfg", [0, DIR, INV, DIR | INV])
def test_shi_shifts_the_pin_level_in_at_the_configured_end(cfg):
    n = len(SHI_LEVELS)
    start = 0x8001                                   # old contents move along
    program = [("SHI", {})] * n + [("STSR", dict(rd=1)), ("MOV", dict(rd=2, ra=1)),
                                   ("SHRI", dict(rd=2, imm=16 - n)), ("HALT", {})]
    machine = be_machine(program, csrs={"SR": start, "BE_CFG": cfg,
                                        "BE_PINS": be_pins(out=OUT0 + 5, pin_in=IN0)})
    records = run_to_halt(machine, pads=pads_for(SHI_LEVELS))
    assert [r.x_cycle for r in done(records, "SHI")] == [FIRST_X + 4 * k for k in range(n)]
    s = [level ^ int(bool(cfg & INV)) for level in SHI_LEVELS]
    if cfg & DIR:                                    # {SR[14:0], s}
        value = sum(bit << (n - 1 - k) for k, bit in enumerate(s))
        expected = ((start << n) & 0xFFFF) | value
    else:                                            # {s, SR[15:1]}
        value = sum(bit << k for k, bit in enumerate(s))
        expected = (start >> n) | (value << (16 - n))
        # The value sits in SR[15:16-n]; STSR then SHRI 16-n right-aligns it.
        assert records[-2].mnemonic == "SHRI" and records[-2].val == value
    assert records[-4].mnemonic == "STSR" and records[-4].val == expected
    assert machine.threads[0].sr == expected
    assert machine.pin_out == 0                      # SHI writes no pin


@pytest.mark.parametrize("high_edge,seen", [
    (FIRST_X - 2, 0), (FIRST_X - 1, 1), (FIRST_X, 0),
])
def test_shi_sees_the_pad_level_held_at_edge_x_minus_1(high_edge, seen):
    """Section 3: ``pin_in(0..12)`` in X cycle ``x`` is the pad at edge ``x - 1``."""
    machine = be_machine([("SHI", {}), ("HALT", {})],
                         csrs={"BE_PINS": be_pins(pin_in=IN0), "BE_CFG": DIR})
    run_to_halt(machine, pads=lambda cycle: int(cycle + 1 == high_edge))
    assert machine.threads[0].sr == seen


@pytest.mark.parametrize("pin,cfg,seen", [
    (OUT0 + 1, DIR, 1),          # an OUT index reads its PIN_OUT bit as visible in X
    (13, DIR, 0),                # a reserved index reads 0 ...
    (13, DIR | INV, 1),          # ... and INV applies to that level like any other
])
def test_shi_takes_pin_in_of_any_index(pin, cfg, seen):
    machine = be_machine([("SETP", dict(pin=OUT0 + 1, val=1)), ("SHI", {}), ("HALT", {})],
                         csrs={"BE_PINS": be_pins(out=31, pin_in=pin), "BE_CFG": cfg})
    machine.set_pad_inputs(ui_in=0xFF, uio_in=0xFF)  # every real input pad high
    run_to_halt(machine)
    assert machine.threads[0].sr == seen


# --------------------------------------------------------------- CNT, Z
@pytest.mark.parametrize("op", ["SHO", "SHI"])
@pytest.mark.parametrize("cnt,flags,expected", [
    # (CNT after each op, {T, C, Z} after each op): CNT stops at 0, Z is
    # (new CNT == 0) whatever Z was, C and T keep their values.
    (2, 0b110, [(1, 0b110), (0, 0b111), (0, 0b111), (0, 0b111)]),
    (3, 0b001, [(2, 0b000), (1, 0b000), (0, 0b001), (0, 0b001)]),
    (0, 0b010, [(0, 0b011), (0, 0b011), (0, 0b011), (0, 0b011)]),
    (31, 0b101, [(30, 0b100), (29, 0b100), (28, 0b100), (27, 0b100)]),
])
def test_cnt_saturates_at_zero_and_z_comes_from_the_new_cnt(op, cnt, flags, expected):
    program = [(op, {}), ("CSRR", dict(rd=1, csr=CSR["CNT"]))] * 4 + [("HALT", {})]
    machine = be_machine(program, csrs={"CNT": cnt, "FLAGS": flags})
    records = run_to_halt(machine)
    ops = [r for r in records if r.mnemonic == op]
    reads = [r.val for r in records if r.mnemonic == "CSRR"]
    assert [(value, r.flags) for value, r in zip(reads, ops)] == expected
    assert machine.threads[0].flags == expected[-1][1]


def test_the_documented_transmit_loop_sends_cnt_bits_three_slots_apart():
    """6.9: ``LDSR r0; CSRW CNT, r1; bit: SHO; WAITD 1; BNZ bit``."""
    trace = PadTrace()
    program = load16(0, 0x00A5) + [
        ("LDI", dict(rd=1, imm=8)), ("LDSR", dict(ra=0)),
        ("CSRW", dict(csr=CSR["CNT"], ra=1)),
        ("SHO", {}), ("WAITD", dict(imm=1)), ("BNZ", dict(rel=-3)), ("HALT", {})]
    machine = be_machine(program, csrs={"BE_PINS": be_pins(out=OUT0)}, on_cycle=trace)
    records = run_to_halt(machine)
    shos = done(records, "SHO")
    assert len(shos) == 8                            # BNZ fell through on CNT == 0
    assert [b.x_cycle - a.x_cycle for a, b in zip(shos, shos[1:])] == [12] * 7
    pad = trace.uo_bit(0)
    assert [pad[r.x_cycle + 2] for r in shos] == [(0xA5 >> k) & 1 for k in range(8)]
    assert (machine.threads[0].cnt, machine.threads[0].z) == (0, 1)


# ------------------------------------------------ LDSR, STSR, CRCI, STCRC
@pytest.mark.parametrize("flags", [0b000, 0b111])
def test_ldsr_stsr_crci_stcrc_move_data_and_leave_the_flags(flags):
    program = load16(1, 0xC35A) + [
        ("LDSR", dict(ra=1)), ("STSR", dict(rd=2)), ("STCRC", dict(rd=3)),
        ("CRCI", {}), ("STCRC", dict(rd=4)), ("HALT", {})]
    machine = be_machine(program, csrs={"CRC": 0x1234, "CRC_INIT": 0xBEEF,
                                        "FLAGS": flags})
    records = run_to_halt(machine)
    got = [(r.mnemonic, r.we, r.rd, r.val) for r in records[2:-1]]
    assert got == [("LDSR", False, 0, 0), ("STSR", True, 2, 0xC35A),
                   ("STCRC", True, 3, 0x1234), ("CRCI", False, 0, 0),
                   ("STCRC", True, 4, 0xBEEF)]
    assert all(r.flags == flags for r in records)
    th = machine.threads[0]
    assert (th.sr, th.crc, th.regs[2], th.regs[3], th.regs[4]) == \
        (0xC35A, 0xBEEF, 0xC35A, 0x1234, 0xBEEF)


@pytest.mark.parametrize("name,readback", [
    ("BE_CFG", M2_CFG_BITS), ("BE_PINS", 0x03FF), ("BE_RELOAD", 0x001F),
    ("CRC_POLY", 0xFFFF), ("CRC_INIT", 0xFFFF), ("SR", 0xFFFF), ("CNT", 0x001F),
    ("CRC", 0xFFFF),
])
def test_csrw_and_csrr_reach_every_bit_engine_csr_at_its_width(name, readback):
    """6.6 and 6.9: the CSR takes ``ra`` truncated to its width (isa.yaml);
    ``BE_CFG`` keeps only bits 1, 7 and 9 at M2."""
    program = load16(1, 0xFFFF) + [("CSRW", dict(csr=CSR[name], ra=1)),
                                   ("CSRR", dict(rd=2, csr=CSR[name])), ("HALT", {})]
    machine = be_machine(program, csrs={"FLAGS": 0b101})
    records = run_to_halt(machine)
    assert records[-2].mnemonic == "CSRR" and records[-2].val == readback
    assert all(r.flags == 0b101 for r in records)
    assert machine.host_read_debug(0, 0x10 + CSR[name]) == readback
    debug = {"SR": 0x0C, "CNT": 0x0D, "CRC": 0x0E}  # HOST_PROTOCOL space 4
    if name in debug:
        assert machine.host_read_debug(0, debug[name]) == readback


BE_CFG_WRITES = [1 << bit for bit in range(16)] + [0xFFFF, 0xFFFF & ~M2_CFG_BITS]


def test_be_cfg_bits_other_than_1_7_9_read_0_and_ignore_csrw():
    program = []
    for value in BE_CFG_WRITES:
        program += load16(1, value) + [("CSRW", dict(csr=CSR["BE_CFG"], ra=1)),
                                       ("CSRR", dict(rd=2, csr=CSR["BE_CFG"]))]
    machine = be_machine(program + [("HALT", {})])
    records = run_to_halt(machine)
    reads = [r.val for r in records if r.mnemonic == "CSRR"]
    assert reads == [value & M2_CFG_BITS for value in BE_CFG_WRITES]


def test_be_cfg_bits_other_than_1_7_9_read_0_and_ignore_host_writes():
    machine = be_machine([("HALT", {})])
    for value in BE_CFG_WRITES:
        machine.host_write_debug(0, "BE_CFG", value)
        machine.step_cycle()
        assert machine.host_read_debug(0, 0x10 + CSR["BE_CFG"]) == value & M2_CFG_BITS


def test_the_ignored_be_cfg_bits_change_nothing():
    """All the other bits set, DIR, INV and CRC_EN clear: LSB first, the level
    as is, the CRC untouched."""
    machine = be_machine([("SHO", {}), ("HALT", {})],
                         csrs={"BE_CFG": 0xFFFF & ~M2_CFG_BITS, "SR": 0x8001,
                               "CRC": 0x1234, "CRC_POLY": 0x8005,
                               "BE_PINS": be_pins(out=OUT0)})
    run_to_halt(machine)
    assert machine.uo_out == 0b1
    assert (machine.threads[0].sr, machine.threads[0].crc) == (0x4000, 0x1234)


# -------------------------------------------------------------------- CRC
def crc_reference(width, poly, init, bits):
    """A textbook serial CRC on a plain ``width``-bit register, fed one bit at
    a time in the order given: no left alignment, no reflection, no final XOR."""
    mask = (1 << width) - 1
    crc = init & mask
    for bit in bits:
        feedback = ((crc >> (width - 1)) & 1) ^ bit
        crc = (crc << 1) & mask
        if feedback:
            crc ^= poly
    return crc


def reflect(value, width):
    return int(format(value, "0%db" % width)[::-1], 2)


def serial_bits(data, lsb_first):
    """The bits of ``data`` in transmission order."""
    order = range(8) if lsb_first else range(7, -1, -1)
    return [(byte >> i) & 1 for byte in data for i in order]


# The catalogue entries (Greg Cook's CRC catalogue, as used by reveng):
# (name, refin, refout, xorout, check over "123456789").
#
# Which conventions apply to the raw register: the bit engine takes the bits
# in transmission order (6.9), and a catalogue's ``refin`` says exactly that
# (USB sends each byte LSB first, CAN and SMBus MSB first), so the input
# reflection is done by feeding the bytes in that order and not by the
# register.  ``refout`` and ``xorout`` are output conventions the hardware
# does not apply: the raw register is compared with the raw reference, and
# the catalogue check value is reached by applying them to that raw value.
CATALOGUE = {
    "usb5": ("CRC-5/USB", True, True, 0x1F, 0x19),
    "usb16": ("CRC-16/USB", True, True, 0xFFFF, 0xB4C8),
    "can15": ("CRC-15/CAN", False, False, 0x0000, 0x059E),
    "smbus8": ("CRC-8/SMBUS", False, False, 0x00, 0xF4),
}
CHECK_STRING = b"123456789"


def catalogue_value(preset, raw):
    """A raw ``width``-bit CRC with the preset's output conventions applied."""
    _, _, refout, xorout, _ = CATALOGUE[preset]
    width = ISA.crc_presets[preset]["width"]
    return (reflect(raw, width) if refout else raw) ^ xorout


def test_the_isa_lists_exactly_the_presets_checked_here():
    assert sorted(ISA.crc_presets) == sorted(CATALOGUE)


@pytest.mark.parametrize("preset", sorted(CATALOGUE))
def test_the_reference_reproduces_the_catalogue_check_value(preset):
    p = ISA.crc_presets[preset]
    _, refin, _, _, check = CATALOGUE[preset]
    raw = crc_reference(p["width"], p["poly"], p["init"],
                        serial_bits(CHECK_STRING, lsb_first=refin))
    assert catalogue_value(preset, raw) == check


def crc_program_sho(data, msb_first):
    """One byte at a time through the 6.9 transmit loop, CNT = 8."""
    program = [("LDI", dict(rd=1, imm=8)), ("CRCI", {})]
    for byte in data:
        program += load16(0, byte << 8 if msb_first else byte)
        program += [("LDSR", dict(ra=0)), ("CSRW", dict(csr=CSR["CNT"], ra=1)),
                    ("SHO", {}), ("BNZ", dict(rel=-2))]
    return program + [("STCRC", dict(rd=2)), ("HALT", {})]


def crc_program_shi(bits):
    """Drive OUT0 with each bit inverted and read it back with ``SHI`` and
    ``INV`` set, so ``s`` is the data bit (pin_in of 16..21 is PIN_OUT)."""
    program = [("CRCI", {})]
    for bit in bits:
        program += [("SETP", dict(pin=OUT0, val=1 - bit)), ("SHI", {})]
    return program + [("STCRC", dict(rd=2)), ("HALT", {})]


@pytest.mark.parametrize("path", ["SHO", "SHI"])
@pytest.mark.parametrize("preset", sorted(CATALOGUE))
def test_the_crc_of_each_preset_matches_the_reference_and_the_catalogue(preset, path):
    p = ISA.crc_presets[preset]
    width = p["width"]
    align = 16 - width
    _, refin, _, _, check = CATALOGUE[preset]
    bits = serial_bits(CHECK_STRING, lsb_first=refin)
    if path == "SHO":
        program = crc_program_sho(CHECK_STRING, msb_first=not refin)
        cfg = CRC_EN | (0 if refin else DIR)
    else:
        program = crc_program_shi(bits)
        cfg = CRC_EN | INV
    machine = be_machine(program, csrs={
        "CRC_POLY": p["poly"] << align, "CRC_INIT": p["init"] << align,
        "CRC": 0x5555, "BE_CFG": cfg, "BE_PINS": be_pins(out=OUT0 + 2, pin_in=OUT0)})
    records = run_to_halt(machine, max_cycles=20000)
    assert len(done(records, path)) == 8 * len(CHECK_STRING)
    assert records[-2].mnemonic == "STCRC"
    register = records[-2].val
    assert register & ((1 << align) - 1) == 0        # left-aligned in CRC[15:16-n]
    raw = register >> align
    assert raw == crc_reference(width, p["poly"], p["init"], bits)
    assert catalogue_value(preset, raw) == check


@pytest.mark.parametrize("op,sr,level,cfg,crc", [
    # SHO feeds the CRC with b, the data bit, whatever INV does to the pin ...
    ("SHO", 1, 1, 0, 0x8005), ("SHO", 1, 0, INV, 0x8005), ("SHO", 0, 1, INV, 0x0000),
    # ... and SHI with s, the pin level after INV.
    ("SHI", 0, 1, 0, 0x8005), ("SHI", 0, 1, INV, 0x0000), ("SHI", 0, 0, INV, 0x8005),
])
def test_sho_feeds_the_crc_with_b_and_shi_with_s(op, sr, level, cfg, crc):
    """From CRC = 0 one step with bit ``x`` gives ``x ? CRC_POLY : 0``."""
    if op == "SHO":
        program = [("SHO", {}), ("HALT", {})]
    else:
        program = [("SETP", dict(pin=OUT0, val=level)), ("SHI", {}), ("HALT", {})]
    machine = be_machine(program, csrs={
        "SR": sr, "BE_CFG": cfg | CRC_EN, "CRC_POLY": 0x8005,
        "BE_PINS": be_pins(out=OUT0 + 1, pin_in=OUT0)})
    run_to_halt(machine)
    assert machine.threads[0].crc == crc
    if op == "SHO":
        assert (machine.uo_out >> 1) & 1 == level


@pytest.mark.parametrize("op", ["SHO", "SHI"])
def test_without_crc_en_the_crc_is_left_alone(op):
    machine = be_machine([(op, {})] * 3 + [("HALT", {})],
                         csrs={"CRC": 0x1234, "CRC_POLY": 0x8005, "SR": 0xFFFF,
                               "BE_CFG": DIR | INV, "BE_PINS": be_pins()})
    run_to_halt(machine, pads=lambda cycle: 0xFF)
    assert machine.threads[0].crc == 0x1234


# ----------------------------------------------------------- per thread
def test_each_thread_has_its_own_bit_engine():
    image = assemble(ISA, [("SHO", {}), ("HALT", {})], 0x000)
    image.update(assemble(ISA, [("SHO", {}), ("SHO", {}), ("HALT", {})], 0x100))
    machine = Machine(image, features={"BE"}, isa=ISA)
    setups = [{"SR": 0x0001, "CNT": 5, "BE_CFG": 0, "BE_PINS": be_pins(out=OUT0)},
              {"SR": 0xC000, "CNT": 0, "BE_CFG": DIR, "BE_PINS": be_pins(out=OUT0 + 1)}]
    for thread, csrs in enumerate(setups):
        for name, value in csrs.items():
            machine.host_write_debug(thread, name, value)
    machine.host_set_run(0b0011)
    assert machine.run_until(lambda m: m.halted == 0b0011, max_cycles=200).fired
    assert machine.uo_out == 0b11
    assert [(th.sr, th.cnt, th.be_cfg) for th in machine.threads] == [
        (0x0000, 4, 0), (0x0000, 0, DIR), (0, 0, 0), (0, 0, 0)]


# ----------------------------------------------------- WAITB 0 and M1 builds
@pytest.mark.parametrize("tmo", [0, 1])
@pytest.mark.parametrize("features,built", [
    (("FIFO", "BE"), True),
    (("FIFO",), False),              # condition 0 also needs the bit engine
    (("BE",), False),                # WAITB itself is built with the FIFOs (6.4)
])
def test_waitb_zero_is_true_with_the_bit_engine_and_a_badop_nop_without_it(
        features, built, tmo):
    """Built, "bit engine idle" is always true at M2 (6.7), so the wait ends
    in its first slot and a timed one clears ``T``; unbuilt it is a ``NOP``."""
    machine = be_machine([("WAITB", dict(cond=0, tmo=tmo)), ("HALT", {})],
                         features=features, csrs={"FLAGS": 0b100, "TD": 0x8000})
    records = run_to_halt(machine)
    assert len(records) == 2 and records[0].done and records[0].next_pc == 1
    assert records[0].t == (0 if built and tmo else 1)
    assert machine.badop == (0 if built else 1)


@pytest.mark.parametrize("features", [(), ("FIFO", "SETPD")])
@pytest.mark.parametrize("name,ops", [
    ("SHO", {}), ("SHI", {}), ("LDSR", dict(ra=1)), ("STSR", dict(rd=2)),
    ("CRCI", {}), ("STCRC", dict(rd=2)),
])
def test_bit_engine_instructions_are_badop_nops_without_the_feature(name, ops, features):
    thread = 1
    machine = be_machine([(name, ops), ("HALT", {})], features=features, thread=thread,
                         csrs={"SR": 0xFFFF, "CRC_INIT": 0xFFFF, "BE_CFG": INV,
                               "BE_PINS": be_pins(out=OUT0), "FLAGS": 0b010,
                               "r1": 0x1234, "r2": 0x5678})
    records = run_to_halt(machine, thread=thread, pads=lambda cycle: 0xFF)
    first = records[0]
    assert first.done and first.next_pc == first.pc + 1 and not first.we
    assert first.flags == 0b010                      # SHO/SHI would have set Z
    assert machine.badop == 1 << thread
    assert (machine.pin_out, machine.pin_oe) == (0, 0)   # INV would drive a 1
    assert machine.threads[thread].regs[2] == 0x5678
    for register in ("SR", "CNT", "CRC", "BE_CFG", "CRC_INIT"):
        assert machine.host_read_debug(thread, register) == 0, register
    assert machine.caps & (1 << 4) == 0
