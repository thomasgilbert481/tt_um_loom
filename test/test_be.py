# SPDX-License-Identifier: Apache-2.0
"""Bit engine, manual mode (docs/SEMANTICS.md 6.9).

Expected values come from the text: SHO sends b = DIR ? SR[15] : SR[0] as a
pin write of b ^ INV (6.3 rules, open drain included), shifts SR, counts CNT
down to 0 and sets Z = (new CNT == 0); SHI shifts s = pin_in ^ INV in; with
CRC_EN the serial CRC takes the bit (fb = CRC[15] ^ x; CRC = CRC << 1 ^
(fb ? POLY : 0)). The CRC presets are checked twice: against that serial
rule, and, through the left alignment of isa.yaml's canonical presets,
against the standard CRC definitions (reveng catalogue parameters and check
values over "123456789").

Timing on the pads: a marker SETP OUT1,1 right before each SHO puts that
SHO's commit edge at the marker's pad edge + 4, where OUT0 (or the pin the
engine drives) is sampled.
"""

import cocotb
from cocotb.triggers import ClockCycles

from spi_host import (
    LoomHost, PadMonitor, ISA, asm, run_snippet, SP_CTRL, CTRL_PIN_OUT,
    CTRL_OD_MASK, CTRL_PIN_OE, CTRL_BADOP, DBG_SR, DBG_CNT, DBG_CRC,
    DBG_FLAGS, CSR_BE_CFG, CSR_BE_PINS, CSR_BE_RELOAD, CSR_CRC_POLY,
    CSR_CRC_INIT, CSR_SR, CSR_CNT, CSR_CRC, CSR_FLAGS, CSR_OUTGRP,
    CSR_TICK_INT, FLAG_Z, FLAG_C, FLAG_T,
)

OUT0, OUT1, OUT2 = 16, 17, 18
DIR, INV, CRC_EN = 1 << 1, 1 << 7, 1 << 9


def be_pins(out, inp):
    return (inp << 5) | out


# ------------------------------------------------------------ references
def serial_crc(bits, crc, poly):
    """SEMANTICS 6.9, bit by bit, on the 16-bit left-aligned register."""
    for x in bits:
        fb = ((crc >> 15) & 1) ^ x
        crc = (crc << 1) & 0xFFFF
        if fb:
            crc ^= poly
    return crc


def reflect(value, width):
    out = 0
    for i in range(width):
        if (value >> i) & 1:
            out |= 1 << (width - 1 - i)
    return out


def rocksoft_crc(data, width, poly, init, refin, refout, xorout):
    """The standard parameterised CRC (Williams' model), byte-wise input."""
    top, mask = 1 << (width - 1), (1 << width) - 1
    crc = init
    for byte in data:
        if refin:
            byte = reflect(byte, 8)
        for i in range(7, -1, -1):
            bit = (byte >> i) & 1
            fb = ((crc & top) != 0) ^ bit
            crc = (crc << 1) & mask
            if fb:
                crc ^= poly
    if refout:
        crc = reflect(crc, width)
    return crc ^ xorout


#: reveng catalogue entries for the four presets of isa.yaml.
STANDARD = {
    "usb5":   dict(refin=True, refout=True, xorout=0x1F, check=0x19),
    "usb16":  dict(refin=True, refout=True, xorout=0xFFFF, check=0xB4C8),
    "can15":  dict(refin=False, refout=False, xorout=0x0000, check=0x059E),
    "smbus8": dict(refin=False, refout=False, xorout=0x00, check=0xF4),
}
CHECK = b"123456789"


def bits_of(data, lsb_first):
    out = []
    for byte in data:
        order = range(8) if lsb_first else range(7, -1, -1)
        out += [(byte >> i) & 1 for i in order]
    return out


# --------------------------------------------------------------- helpers
async def setup_be(host, t, cfg, pins, poly=0, init=0, cnt=0, sr=0, crc=0):
    await host.write_csr(t, CSR_BE_CFG, cfg)
    await host.write_csr(t, CSR_BE_PINS, pins)
    await host.write_csr(t, CSR_CRC_POLY, poly)
    await host.write_csr(t, CSR_CRC_INIT, init)
    await host.write_csr(t, CSR_CNT, cnt)
    await host.write_csr(t, CSR_SR, sr)
    await host.write_csr(t, CSR_CRC, crc)


async def run_at(host, prog, t, base, timeout_slots=4000):
    """Load, point thread t at `base` with CTRL.RESET, run to HALT."""
    await host.load_program(prog, verify=False)
    await host.set_reset_pc(t, base)
    await host.reset_thread(t)
    await host.run(1 << t)
    await host.wait_halted(1 << t, timeout_slots)


def sho_loop(base, marker=True):
    """bit: [SETP OUT1,1]; SHO; [SETP OUT1,0]; BNZ bit; HALT (Z from CNT)."""
    prog, a = {}, base
    if marker:
        prog[a] = asm("SETP", pin=OUT1, val=1)
        a += 1
    prog[a] = asm("SHO")
    a += 1
    if marker:
        prog[a] = asm("SETP", pin=OUT1, val=0)
        a += 1
    prog[a] = asm("BNZ", rel=base - (a + 1))
    prog[a + 1] = asm("HALT")
    return prog


# ------------------------------------------------------------------ tests
@cocotb.test()
async def test_be_csrs(dut):
    """BE CSRs through CSRW/CSRR and the debug space: widths, the three
    BE_CFG bits that exist (DIR 1, INV 7, CRC_EN 9; others read 0 and
    ignore writes), SR/CNT/CRC at debug 0x0C-0x0E and 0x1D-0x1F."""
    host = LoomHost(dut)
    await host.start()
    caps = await host.caps()
    assert caps & 0x10, "the build must report the bit engine (CAPS[4])"
    widths = {CSR_BE_CFG: DIR | INV | CRC_EN, CSR_BE_PINS: 0x3FF,
              CSR_BE_RELOAD: 0x1F, CSR_CRC_POLY: 0xFFFF, CSR_CRC_INIT: 0xFFFF,
              CSR_SR: 0xFFFF, CSR_CNT: 0x1F, CSR_CRC: 0xFFFF}
    t = 1
    for csr, mask in widths.items():
        assert await host.read_csr(t, csr) == 0, f"CSR {csr:#x} resets to 0"
        await host.write_csr(t, csr, 0xFFFF)
        assert await host.read_csr(t, csr) == mask, f"CSR {csr:#x} via debug"
        await host.write_csr(t, csr, 0)
    assert await host.read_debug(t, DBG_SR) == 0
    await host.write_debug(t, DBG_SR, 0x1234)
    await host.write_debug(t, DBG_CNT, 0xFFF7)
    await host.write_debug(t, DBG_CRC, 0xBEEF)
    assert await host.read_csr(t, CSR_SR) == 0x1234
    assert await host.read_csr(t, CSR_CNT) == 0x17
    assert await host.read_csr(t, CSR_CRC) == 0xBEEF
    assert await host.read_debug(t, DBG_CNT) == 0x17
    # The same through the thread: CSRW then CSRR of every BE CSR, in two
    # chunks so the results fit in r0..r5 (r7 holds the value written).
    got = {}
    items = list(widths.items())
    for chunk in (items[:6], items[6:]):
        code = []
        for i, (csr, _) in enumerate(chunk):
            code += [asm("CSRW", csr=csr, ra=7), asm("CSRR", rd=i, csr=csr)]
        await run_snippet(host, code, thread=t, start=0x40, regs={7: 0xFFFF})
        for i, (csr, _) in enumerate(chunk):
            got[csr] = await host.read_reg(t, i)
    assert got == widths, {hex(k): hex(v) for k, v in got.items()}
    # Other threads' engines are separate.
    assert await host.read_csr(0, CSR_CRC_POLY) == 0
    assert await host.badop() == 0


@cocotb.test()
async def test_sho_directions_inv_cnt_z(dut):
    """SHO LSB first and MSB first, with and without INV, drives the pin in
    order; CNT counts down to 0 and stays there; Z = (new CNT == 0); C and
    T are untouched; SR ends shifted."""
    host = LoomHost(dut)
    await host.start()
    mon = PadMonitor(dut).start()
    t = 0
    sr0 = 0xB2C5
    for cfg, n in ((0, 16), (DIR, 16), (INV, 7), (DIR | INV, 5)):
        await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
        await setup_be(host, t, cfg, be_pins(OUT0, 0), cnt=n, sr=sr0)
        start = mon.now
        await run_at(host, sho_loop(0), t, 0)
        marks = [c for c, v in mon.changes(mon.uo, 1, start) if v]
        assert len(marks) == n, f"cfg {cfg:#x}: {len(marks)} SHOs, CNT was {n}"
        sent = [(mon.uo[m + 4] >> 0) & 1 for m in marks]
        if cfg & DIR:
            want = [(sr0 >> (15 - i)) & 1 for i in range(n)]
            sr_end = (sr0 << n) & 0xFFFF
        else:
            want = [(sr0 >> i) & 1 for i in range(n)]
            sr_end = sr0 >> n
        if cfg & INV:
            want = [b ^ 1 for b in want]
        assert sent == want, f"cfg {cfg:#x}: sent {sent}, want {want}"
        # The pin moves only at an SHO commit edge (marker + 4), like SETP.
        moved = [c for c, _ in mon.changes(mon.uo, 0, start)]
        assert all(c - 4 in marks for c in moved), f"cfg {cfg:#x}: OUT0 at {moved}"
        assert await host.read_debug(t, DBG_SR) == sr_end
        assert await host.read_debug(t, DBG_CNT) == 0
        assert await host.read_debug(t, DBG_FLAGS) & FLAG_Z

    # CNT == 0 stays 0 and Z = 1; C and T keep their values; Z = 0 while
    # CNT > 0 after the decrement.
    await setup_be(host, t, 0, be_pins(OUT0, 0), cnt=0, sr=1)
    await run_snippet(host, [asm("CSRW", csr=CSR_FLAGS, ra=1), asm("SHO")],
                      thread=t, start=0x20, regs={1: FLAG_C | FLAG_T})
    assert await host.read_debug(t, DBG_CNT) == 0
    assert await host.read_debug(t, DBG_FLAGS) == FLAG_C | FLAG_T | FLAG_Z
    await setup_be(host, t, 0, be_pins(OUT0, 0), cnt=2, sr=1)
    await run_snippet(host, [asm("CSRW", csr=CSR_FLAGS, ra=1), asm("SHO")],
                      thread=t, start=0x20, regs={1: FLAG_Z})
    assert await host.read_debug(t, DBG_CNT) == 1
    assert await host.read_debug(t, DBG_FLAGS) == 0
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_sho_pin_write_rules(dut):
    """SHO's pin write follows 6.3 like SETP: open drain on a BIDIR pin with
    OD_MASK set (PIN_OUT <- 0, PIN_OE <- ~b), and a read-only index is
    ignored while SR still shifts."""
    host = LoomHost(dut)
    await host.start()
    mon = PadMonitor(dut).start()
    t = 2
    await host.write(SP_CTRL, CTRL_OD_MASK, 0x08)          # BIDIR3 open drain
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0x0008)        # would drive 1
    await host.write(SP_CTRL, CTRL_PIN_OE, 0x00)
    sr0 = 0b1011001
    await setup_be(host, t, 0, be_pins(3, 0), cnt=7, sr=sr0)
    start = mon.now
    await run_at(host, sho_loop(0x80), t, 0x80)
    marks = [c for c, v in mon.changes(mon.uo, 1, start) if v]
    assert len(marks) == 7
    for i, m in enumerate(marks):
        b = (sr0 >> i) & 1
        assert (mon.uio[m + 4] >> 3) & 1 == 0, "open drain: PIN_OUT[3] = 0"
        assert (mon.oe[m + 4] >> 3) & 1 == b ^ 1, f"bit {i}: OE = ~b"
    # Read-only IN0 (index 8): no pin changes, SR shifts anyway.
    await host.write(SP_CTRL, CTRL_OD_MASK, 0)
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await host.write(SP_CTRL, CTRL_PIN_OE, 0)
    await setup_be(host, t, 0, be_pins(8, 0), cnt=4, sr=0xFFFF)
    start = mon.now
    await run_at(host, sho_loop(0x80), t, 0x80)
    assert mon.changes(mon.uo, 0, start) == []
    assert all(v == 0 for v in mon.uio[start:]) and all(v == 0 for v in mon.oe[start:])
    assert await host.read_debug(t, DBG_SR) == 0x0FFF
    mon.stop()
    await ClockCycles(dut.clk, 2)


def shi_program(base, bits, pin_setp):
    """For every bit: SETP <pin>, b; SHI. Then HALT."""
    prog, a = {}, base
    for b in bits:
        prog[a] = asm("SETP", pin=pin_setp, val=b)
        prog[a + 1] = asm("SHI")
        a += 2
    prog[a] = asm("HALT")
    return prog


@cocotb.test()
async def test_shi_directions_and_inputs(dut):
    """SHI shifts pin_in(BE_PINS.in) ^ INV into SR, LSB first (into SR[15],
    moving right) or MSB first (into SR[0], moving left), from an OUT pin
    (PIN_OUT view), a looped-back BIDIR pin (through the synchroniser) and
    a testbench-driven input; CNT and Z as for SHO."""
    host = LoomHost(dut)
    await host.start()
    t = 3
    bits = [1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1]
    n = len(bits)
    for pin, cfg in ((OUT2, 0), (OUT2, DIR), (5, INV), (5, DIR | INV)):
        await host.write(SP_CTRL, CTRL_PIN_OE, 1 << 5)     # BIDIR5 driven
        await setup_be(host, t, cfg, be_pins(0, pin), cnt=n + 1, sr=0)
        await run_at(host, shi_program(0xC0, bits, pin), t, 0xC0)
        s = [b ^ (1 if cfg & INV else 0) for b in bits]
        sr = 0
        for b in s:
            sr = ((sr << 1) | b) & 0xFFFF if cfg & DIR else (b << 15) | (sr >> 1)
        assert await host.read_debug(t, DBG_SR) == sr, \
            f"pin {pin} cfg {cfg:#x}: SR {await host.read_debug(t, DBG_SR):#06x} != {sr:#06x}"
        assert await host.read_debug(t, DBG_CNT) == 1
        assert not await host.read_debug(t, DBG_FLAGS) & FLAG_Z
        if not cfg & DIR:
            # 6.9: after n LSB-first bits the value sits in SR[15:16-n], so
            # SHRI by 16 - n right-aligns it (first bit received in bit 0).
            assert sr >> (16 - n) == sum(b << i for i, b in enumerate(s))

    # A testbench-driven input, IN3 (index 11), held while SHI runs.
    await host.write(SP_CTRL, CTRL_PIN_OE, 0)
    for level in (1, 0):
        host.set_in(11, level)
        await ClockCycles(dut.clk, 4)
        await setup_be(host, t, 0, be_pins(0, 11), cnt=3, sr=0x5555)
        await run_snippet(host, [asm("SHI"), asm("SHI"), asm("SHI")],
                          thread=t, start=0xC0)
        want = 0x5555
        for _ in range(3):
            want = (level << 15) | (want >> 1)
        assert await host.read_debug(t, DBG_SR) == want
        assert await host.read_debug(t, DBG_FLAGS) & FLAG_Z, "CNT 3 -> 0 sets Z"
    host.set_in(11, 0)
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_ldsr_stsr_crci_stcrc(dut):
    """LDSR/STSR/CRCI/STCRC move SR and CRC and change no flags; WAITB 0
    (bit engine idle) is always true in manual mode and is not BADOP."""
    host = LoomHost(dut)
    await host.start()
    t = 1
    await setup_be(host, t, 0, 0, init=0xACE1, crc=0x1111)
    for flags in (0, FLAG_Z | FLAG_C | FLAG_T):
        await run_snippet(host, [
            asm("CSRW", csr=CSR_FLAGS, ra=6),
            asm("LDSR", ra=1), asm("STSR", rd=2),
            asm("STCRC", rd=3), asm("CRCI"), asm("STCRC", rd=4),
            asm("WAITB", cond=0),
        ], thread=t, start=0x40, regs={1: 0x9A7E, 6: flags})
        assert await host.read_reg(t, 2) == 0x9A7E
        assert await host.read_debug(t, DBG_SR) == 0x9A7E
        assert await host.read_reg(t, 4) == 0xACE1
        assert await host.read_debug(t, DBG_FLAGS) == flags
        await host.write_csr(t, CSR_CRC, 0x1111)
    assert await host.read_reg(t, 3) == 0x1111
    assert await host.badop() == 0


def crc_program(base, words):
    """LDSR each 16-bit word, then SHO its bits (CNT from r1), CRC_EN on;
    `words` are (value, nbits). Ends with STCRC r5 and HALT."""
    prog, a = {}, base
    for value, nbits in words:
        for w in (asm("LDI", rd=2, imm=value & 0xFF),
                  asm("LDIH", rd=2, imm=value >> 8),
                  asm("LDSR", ra=2),
                  asm("LDI", rd=1, imm=nbits),
                  asm("CSRW", csr=CSR_CNT, ra=1),
                  asm("SHO"),
                  asm("BNZ", rel=-2)):
            prog[a] = w
            a += 1
    prog[a] = asm("STCRC", rd=5)
    prog[a + 1] = asm("HALT")
    return prog


def pack(data, lsb_first):
    """Bytes -> [(sr, nbits)] so that SHO sends them in transmission order:
    LSB first per byte with DIR = 0, MSB first with DIR = 1."""
    out = []
    for i in range(0, len(data), 2):
        pair = data[i:i + 2]
        if lsb_first:
            value = pair[0] | ((pair[1] << 8) if len(pair) > 1 else 0)
        else:
            value = (pair[0] << 8) | (pair[1] if len(pair) > 1 else 0)
        out.append((value, 8 * len(pair)))
    return out


@cocotb.test()
async def test_crc_presets(dut):
    """USB CRC5, USB CRC16, CAN CRC15 and SMBus CRC-8 over "123456789" and
    over random data, left-aligned per the isa.yaml presets: the RTL register
    equals the serial rule of SEMANTICS 6.9, and its top n bits give the
    standard CRC (reflected and inverted where the standard says so)."""
    host = LoomHost(dut)
    await host.start()
    import random
    rng = random.Random(6)
    t = 0
    for name, std in STANDARD.items():
        preset = ISA.crc_presets[name]
        width, poly, init = preset["width"], preset["poly"], preset["init"]
        poly_l, init_l = (poly << (16 - width)) & 0xFFFF, (init << (16 - width)) & 0xFFFF
        lsb_first = std["refin"]
        assert rocksoft_crc(CHECK, width, poly, init, std["refin"], std["refout"],
                            std["xorout"]) == std["check"], f"{name}: Python reference"
        for data in (CHECK, bytes(rng.randrange(256) for _ in range(5))):
            cfg = CRC_EN | (0 if lsb_first else DIR)
            await setup_be(host, t, cfg, be_pins(OUT0, 0), poly=poly_l, init=init_l)
            prog = crc_program(0, pack(data, lsb_first))
            prog = {0: asm("CRCI"), **{a + 1: w for a, w in prog.items()}}
            await run_at(host, prog, t, 0, timeout_slots=8000)
            reg = await host.read_reg(t, 5)
            want_reg = serial_crc(bits_of(data, lsb_first), init_l, poly_l)
            assert reg == want_reg, f"{name} {data!r}: CRC {reg:#06x} != serial {want_reg:#06x}"
            value = reg >> (16 - width)
            if std["refout"]:
                value = reflect(value, width)
            value ^= std["xorout"]
            want = rocksoft_crc(data, width, poly, init, std["refin"], std["refout"],
                                std["xorout"])
            assert value == want, f"{name} {data!r}: {value:#x} != standard {want:#x}"
            assert reg & ((1 << (16 - width)) - 1) == 0, "low bits stay 0"
            dut._log.info("%s over %r: register %04X -> CRC %X" % (name, data, reg, value))


@cocotb.test()
async def test_shi_crc(dut):
    """SHI with CRC_EN updates the CRC with the bit it shifts in (after INV):
    CRC-8/SMBus over two bytes arriving MSB first on OUT2 (PIN_OUT view),
    sent by the same thread with OUT one bit at a time."""
    host = LoomHost(dut)
    await host.start()
    t = 2
    data = b"\x31\xC7"
    preset = ISA.crc_presets["smbus8"]
    poly_l = preset["poly"] << 8
    for inv in (0, INV):
        await setup_be(host, t, CRC_EN | DIR | inv, be_pins(0, OUT2), poly=poly_l,
                       cnt=16)
        # r1 holds the 16 bits MSB first; OUT r1 writes r1[0] to OUT2, so
        # rotate left by 1 first (ROR by 15) and send bit 15 each time.
        prog = {0x80: asm("CSRW", csr=CSR_OUTGRP, ra=3),       # base 18, cnt 1
                0x81: asm("ROR", rd=4, ra=1, rb=6),             # r4 = r1 rotl 1
                0x82: asm("OUT", ra=4),
                0x83: asm("SHI"),
                0x84: asm("SHLI", rd=1, imm=1),
                0x85: asm("DJNZ", rd=2, rel=-5),
                0x86: asm("HALT")}
        await host.load_program(prog, verify=False)
        await host.set_reset_pc(t, 0x80)
        await host.reset_thread(t)
        await host.write_reg(t, 1, (data[0] << 8) | data[1])
        await host.write_reg(t, 2, 16)
        await host.write_reg(t, 3, (1 << 5) | OUT2)
        await host.write_reg(t, 6, 15)
        await host.run(1 << t)
        await host.wait_halted(1 << t)
        received = bits_of(data, lsb_first=False)
        seen = [b ^ (1 if inv else 0) for b in received]
        want = serial_crc(seen, 0, poly_l)
        assert await host.read_debug(t, DBG_CRC) == want
        sr = 0
        for b in seen:
            sr = ((sr << 1) | b) & 0xFFFF
        assert await host.read_debug(t, DBG_SR) == sr
        if not inv:
            assert want >> 8 == rocksoft_crc(data, 8, preset["poly"], 0, False, False, 0)
    await ClockCycles(dut.clk, 2)
