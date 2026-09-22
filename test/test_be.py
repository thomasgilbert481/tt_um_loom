# SPDX-License-Identifier: Apache-2.0
"""Bit engine, manual mode (docs/SEMANTICS.md 6.9) and the M3 slice A
encoders, stuffing and differential output (6.9.1).

Expected values come from the text: SHO sends b = DIR ? SR[15] : SR[0] as a
pin write of b ^ INV (6.3 rules, open drain included), shifts SR, counts CNT
down to 0 and sets Z = (new CNT == 0); SHI shifts s = pin_in ^ INV in; with
CRC_EN the serial CRC takes the bit (fb = CRC[15] ^ x; CRC = CRC << 1 ^
(fb ? POLY : 0)). The CRC presets are checked twice: against that serial
rule, and, through the left alignment of isa.yaml's canonical presets,
against the standard CRC definitions (reveng catalogue parameters and check
values over "123456789").

For 6.9.1 the expected values come from `EncRef` below, a transcription of
the ordered rules of that section into Python. It is written from the text
only; nothing here reads the RTL or the golden model (VERIFICATION.md
METH-1).

Timing on the pads: a marker SETP OUT1,1 right before each SHO puts that
SHO's commit edge at the marker's pad edge + 4, where OUT0 (or the pin the
engine drives) is sampled.
"""

import cocotb
from cocotb.triggers import ClockCycles

from spi_host import (
    LoomHost, PadMonitor, ISA, asm, run_snippet, SP_CTRL, CTRL_PIN_OUT,
    CTRL_OD_MASK, CTRL_PIN_OE, CTRL_BADOP, CTRL_VERSION, DBG_SR, DBG_CNT,
    DBG_CRC, DBG_FLAGS, DBG_ENC, CSR_BE_CFG, CSR_BE_PINS, CSR_BE_RELOAD,
    CSR_CRC_POLY, CSR_CRC_INIT, CSR_SR, CSR_CNT, CSR_CRC, CSR_FLAGS,
    CSR_OUTGRP, CSR_TICK_INT, FLAG_Z, FLAG_C, FLAG_T,
)

OUT0, OUT1, OUT2, OUT3, OUT4, OUT5 = 16, 17, 18, 19, 20, 21
DIR, INV, CRC_EN = 1 << 1, 1 << 7, 1 << 9
# BE_CFG fields added by slice A (ARCHITECTURE 8.1, SEMANTICS 6.9.1).
ENC_NRZ, ENC_NRZI, ENC_MANCH, ENC_RSVD = 0 << 3, 1 << 3, 2 << 3, 3 << 3
STUFF_OFF, STUFF_USB, STUFF_CAN, STUFF_RSVD = 0 << 5, 1 << 5, 2 << 5, 3 << 5
DIFF = 1 << 10
#: BE_CFG bits that slice A does not build: MODE (0), RXTX (2), AUTOPULL (8)
#: and 12:11, plus 15:13 which are not fields at all. They read 0 and ignore
#: writes.
NOT_BUILT = 0xF905


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


class EncRef:
    """SEMANTICS 6.9.1, transcribed rule by rule.

    Holds the whole per-thread engine: SR, CNT, CRC, T and the encoder state
    LVL, RUN, RVAL, PEND, HALF, FIRST. `sho()` returns the level put on
    BE_PINS.out *before* INV; `shi(level)` takes the level seen on
    BE_PINS.in, again before INV, so a transmit sequence can be fed straight
    back into a receiver. `data_bits` counts the bits that entered or left
    SR, which is what CNT and the CRC follow.
    """

    def __init__(self, cfg, sr=0, cnt=0, crc=0, poly=0):
        self.enc = (cfg >> 3) & 3
        self.stuff = (cfg >> 5) & 3
        if self.enc == 3:                 # reserved, stored as 0
            self.enc = 0
        if self.stuff == 3:
            self.stuff = 0
        self.dir_msb = bool(cfg & DIR)
        self.crc_en = bool(cfg & CRC_EN)
        self.poly = poly
        self.sr, self.cnt, self.crc, self.t = sr, cnt, crc, 0
        self.lvl = self.rval = self.pend = self.half = self.first = 0
        self.run = 0
        self.data_bits = 0

    # -- the debug 0x27 view (docs/HOST_PROTOCOL.md SPACE 4)
    @property
    def enc_word(self):
        return (self.first << 7) | (self.half << 6) | (self.pend << 5) \
            | (self.rval << 4) | (self.run << 1) | self.lvl

    @property
    def stuff_value(self):
        """0 for USB, ~RVAL for CAN."""
        return (~self.rval) & 1 if self.stuff == 2 else 0

    def _crc_step(self, x):
        fb = ((self.crc >> 15) & 1) ^ x
        self.crc = (self.crc << 1) & 0xFFFF
        if fb:
            self.crc ^= self.poly

    def _data_bit_out(self):
        x = (self.sr >> 15) & 1 if self.dir_msb else self.sr & 1
        self.sr = ((self.sr << 1) & 0xFFFF) if self.dir_msb else (self.sr >> 1)
        self._count(x)
        return x

    def _data_bit_in(self, s):
        self.sr = (((self.sr << 1) | s) & 0xFFFF) if self.dir_msb \
            else ((s << 15) | (self.sr >> 1))
        self._count(s)

    def _count(self, x):
        self.cnt = 0 if self.cnt == 0 else self.cnt - 1
        self.data_bits += 1
        if self.crc_en:
            self._crc_step(x)

    def _run_account(self, x):
        """Applied after a bit has been sent or received, data or stuff."""
        if self.stuff == 0:
            return
        if x == self.rval:
            self.run = min(self.run + 1, 7)
        else:
            self.run, self.rval = 1, x
        if self.stuff == 1:
            if self.run == 6 and self.rval == 1:
                self.pend = 1
        elif self.run == 5:
            self.pend = 1

    def sho(self):
        # 1. the bit sent
        x = None
        if self.enc == 2 and self.half == 1:
            pass                                   # the second half of a bit
        elif self.stuff != 0 and self.pend == 1:
            x = self.stuff_value                   # SR, CNT, CRC unchanged
            self.pend = 0
        else:
            x = self._data_bit_out()
        # 2. run accounting
        if x is not None:
            self._run_account(x)
        # 3. encoding into the level
        if self.enc == 1:
            level = self.lvl if x else (~self.lvl) & 1
            self.lvl = level
        elif self.enc == 2:
            if self.half == 0:
                self.first, level, self.half = x, (~x) & 1, 1
            else:
                level, self.half = self.first, 0
        else:
            level = x
        return level

    def shi(self, level):
        # 1 and 2: the sample and the bit received
        p = level
        if self.enc == 1:
            s = 1 if p == self.lvl else 0
            self.lvl = p
        elif self.enc == 2:
            if self.half == 0:
                self.first, self.half = p, 1
                return                             # steps 3 and 4 skipped
            s, self.half = p, 0
            if self.first == p:
                self.t = 1                         # no mid-bit transition
        else:
            s = p
        # 3. a pending stuff bit is dropped
        if self.stuff != 0 and self.pend == 1:
            self.pend = 0
            if s != self.stuff_value:
                self.t = 1
        else:
            self._data_bit_in(s)
        # 4. run accounting
        self._run_account(s)

    def send(self, n_data):
        """SHO until `n_data` data bits have left SR; return the levels."""
        levels = []
        while self.data_bits < n_data:
            levels.append(self.sho())
        return levels


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


def sho_marked(base, n):
    """n x (SETP OUT1,1; SHO; SETP OUT1,0), then HALT.

    A straight line, not the CNT loop of `sho_loop`: with stuffing or
    Manchester a SHO need not touch CNT, so the number of SHOs is decided
    here and CNT is left to say what the text says it says.
    """
    prog, a = {}, base
    for _ in range(n):
        prog[a] = asm("SETP", pin=OUT1, val=1)
        prog[a + 1] = asm("SHO")
        prog[a + 2] = asm("SETP", pin=OUT1, val=0)
        a += 3
    prog[a] = asm("HALT")
    return prog


async def run_sho_marks(host, mon, t, n, base=0x40, enc0=None):
    """Run n marked SHOs and return the marker cycle of each.

    The SHO's commit edge is marker + 4 (the marker SETP is the slot before
    it), so the level it drove is `mon.uo[m + 4]` and the like. CTRL.RESET
    clears the encoder state (6.9.1), so an `enc0` preset of debug 0x27 is
    written after the reset and before the run.
    """
    await host.load_program(sho_marked(base, n), verify=False)
    await host.set_reset_pc(t, base)
    await host.reset_thread(t)
    if enc0 is not None:
        await host.write_debug(t, DBG_ENC, enc0)
    start = mon.now
    await host.run(1 << t)
    await host.wait_halted(1 << t)
    marks = [c for c, v in mon.changes(mon.uo, 1, start) if v]
    assert len(marks) == n, f"saw {len(marks)} SHO markers, expected {n}"
    return marks


async def run_shi_levels(host, t, levels, drive_pin, base=0xC0, enc0=None):
    """SETP drive_pin, level; SHI -- once per level, then HALT."""
    await host.load_program(shi_program(base, levels, drive_pin), verify=False)
    await host.set_reset_pc(t, base)
    await host.reset_thread(t)
    if enc0 is not None:
        await host.write_debug(t, DBG_ENC, enc0)
    await host.run(1 << t)
    await host.wait_halted(1 << t)


async def check_engine(host, t, ref, what):
    """Every architectural value the reference predicts, after a run."""
    got = (await host.read_debug(t, DBG_SR), await host.read_debug(t, DBG_CNT),
           await host.read_debug(t, DBG_CRC), await host.read_debug(t, DBG_ENC),
           bool(await host.read_debug(t, DBG_FLAGS) & FLAG_T))
    want = (ref.sr, ref.cnt, ref.crc, ref.enc_word, bool(ref.t))
    assert got == want, (
        f"{what}: SR/CNT/CRC/0x27/T = "
        f"{got[0]:#06x},{got[1]},{got[2]:#06x},{got[3]:#04x},{got[4]} != "
        f"{want[0]:#06x},{want[1]},{want[2]:#06x},{want[3]:#04x},{want[4]}")
    # Z is (CNT == 0) after the instruction, whether or not CNT changed.
    z = bool(await host.read_debug(t, DBG_FLAGS) & FLAG_Z)
    assert z == (ref.cnt == 0), f"{what}: Z {z} for CNT {ref.cnt}"


# ------------------------------------------------------------------ tests
@cocotb.test()
async def test_be_csrs(dut):
    """BE CSRs through CSRW/CSRR and the debug space: widths, the BE_CFG
    bits that exist (DIR 1, ENC 4:3, STUFF 6:5, INV 7, CRC_EN 9, DIFF 10;
    others read 0 and ignore writes), SR/CNT/CRC at debug 0x0C-0x0E and
    0x1D-0x1F.

    A write of 0xFFFF puts 3 in ENC and in STUFF, and 3 is the reserved
    value of both, which 6.9.1 stores as 0; so the readback of an all-ones
    write is DIR | INV | CRC_EN | DIFF.
    """
    host = LoomHost(dut)
    await host.start()
    caps = await host.caps()
    assert caps & 0x10, "the build must report the bit engine (CAPS[4])"
    widths = {CSR_BE_CFG: DIR | INV | CRC_EN | DIFF, CSR_BE_PINS: 0x3FF,
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


@cocotb.test()
async def test_be_cfg_fields_readback(dut):
    """CSRR BE_CFG reports each stored field in its own bits.

    `test_be_csrs` writes 0xFFFF, which sets several fields at once, so a
    read that reported one field's bit in another field's position would
    still read back the same word there. Each field is therefore written
    alone and in pairs, always together with every bit slice A does *not*
    build (NOT_BUILT), which must be dropped.
    """
    host = LoomHost(dut)
    await host.start()
    t = 2
    for cfg in (CRC_EN, CRC_EN | DIR, INV, DIR, INV | DIR, CRC_EN | INV, 0,
                ENC_NRZI, ENC_MANCH, STUFF_USB, STUFF_CAN, DIFF,
                ENC_NRZI | STUFF_USB, ENC_MANCH | STUFF_CAN | DIFF,
                ENC_MANCH | INV | DIFF, DIR | ENC_NRZI | STUFF_CAN | CRC_EN):
        # The bits slice A does not build must be dropped.
        await run_snippet(host, [asm("CSRW", csr=CSR_BE_CFG, ra=1),
                                 asm("CSRR", rd=2, csr=CSR_BE_CFG)],
                          thread=t, start=0x60,
                          regs={1: cfg | NOT_BUILT, 2: 0xFFFF})
        got = await host.read_reg(t, 2)
        assert got == cfg, f"CSRW BE_CFG {cfg:#06x} reads back {got:#06x}"
        assert await host.read_csr(t, CSR_BE_CFG) == cfg, \
            f"BE_CFG {cfg:#06x} through the debug space"
        assert await host.read_csr(0, CSR_BE_CFG) == 0, "thread 0 is untouched"
    assert await host.badop() == 0


# ================================================ M3 slice A (SEMANTICS 6.9.1)
def becfg_stored(value):
    """What 6.9.1 says BE_CFG reads back after a write of `value`: the six
    stored fields, with the reserved value 3 of ENC and of STUFF as 0."""
    out = value & (DIR | INV | CRC_EN | DIFF)
    if (value >> 3) & 3 != 3:
        out |= value & ENC_RSVD            # ENC_RSVD is the mask of bits 4:3
    if (value >> 5) & 3 != 3:
        out |= value & STUFF_RSVD          # and STUFF_RSVD of bits 6:5
    return out


@cocotb.test()
async def test_version_and_reserved_cfg(dut):
    """VERSION reads 3 from slice A on; the reserved value 3 of ENC and of
    STUFF is stored as 0 and behaves as 0; MODE, RXTX, AUTOPULL and 12:11
    still read 0 and ignore writes, and CAPS[8] (auto mode) stays 0."""
    host = LoomHost(dut)
    await host.start()
    mon = PadMonitor(dut).start()
    assert await host.read1(SP_CTRL, CTRL_VERSION) == 3, "VERSION is 3 (6.9.1)"
    assert (await host.caps()) & 0x100 == 0, "auto mode (slice C) is not built"
    t = 1
    for cfg in (ENC_RSVD, STUFF_RSVD, ENC_RSVD | STUFF_RSVD,
                ENC_RSVD | STUFF_CAN, ENC_NRZI | STUFF_RSVD, NOT_BUILT,
                NOT_BUILT | ENC_RSVD | STUFF_RSVD | DIFF | DIR | INV | CRC_EN):
        await host.write_csr(t, CSR_BE_CFG, cfg)
        want = becfg_stored(cfg)
        got = await host.read_csr(t, CSR_BE_CFG)
        assert got == want, f"BE_CFG {cfg:#06x} reads {got:#06x}, want {want:#06x}"

    # A stored 0 in both fields means NRZ and no stuffing: sixteen 1s go out
    # unchanged, every SHO is a data bit and the run state stays 0.
    t = 0
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    cfg = ENC_RSVD | STUFF_RSVD
    await setup_be(host, t, cfg, be_pins(OUT0, 0), cnt=16, sr=0xFFFF)
    marks = await run_sho_marks(host, mon, t, 8)
    assert [(mon.uo[m + 4] >> 0) & 1 for m in marks] == [1] * 8
    assert await host.read_debug(t, DBG_CNT) == 8, "every SHO was a data bit"
    assert await host.read_debug(t, DBG_ENC) == 0, "no run accounting"
    assert await host.badop() == 0
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_nrzi_transmit_receive(dut):
    """NRZI (6.9.1 step 3): a 0 toggles the line, a 1 holds it. Random bit
    strings go out through SHO and come back through SHI, with and without
    INV; the levels themselves are checked against the reference, so a
    round trip cannot pass on a symmetric mistake."""
    host = LoomHost(dut)
    await host.start()
    mon = PadMonitor(dut).start()
    import random
    rng = random.Random(261)
    tx, rx, n = 0, 3, 13
    for inv in (0, INV):
        for trial in range(3):
            bits = [rng.randrange(2) for _ in range(n)]
            sr0 = sum(b << i for i, b in enumerate(bits))
            cfg = ENC_NRZI | inv
            ref = EncRef(cfg, sr=sr0, cnt=n)
            levels = ref.send(n)
            pads = [lv ^ (1 if inv else 0) for lv in levels]
            # -- transmit
            await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
            await setup_be(host, tx, cfg, be_pins(OUT0, 0), cnt=n, sr=sr0)
            marks = await run_sho_marks(host, mon, tx, n)
            sent = [(mon.uo[m + 4] >> 0) & 1 for m in marks]
            assert sent == pads, \
                f"NRZI inv={inv} trial {trial}: {sent} != {pads} for {bits}"
            await check_engine(host, tx, ref, f"NRZI TX inv={inv}")
            # -- receive the very levels that went out
            rref = EncRef(cfg, cnt=n)
            for lv in levels:
                rref.shi(lv)
            await setup_be(host, rx, cfg, be_pins(0, OUT2), cnt=n)
            await run_shi_levels(host, rx, pads, OUT2)
            await check_engine(host, rx, rref, f"NRZI RX inv={inv}")
            # n bits LSB first sit in SR[15:16-n] (6.9), so the round trip
            # gives back exactly the bits that were sent.
            assert rref.sr >> (16 - n) == sr0, f"round trip {bits}"
    assert await host.badop() == 0
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_manchester_transmit_receive(dut):
    """Manchester (IEEE 802.3): a 0 is high then low, a 1 low then high, so
    one bit is two SHO driving ~x then x and only the first touches CNT.
    Receive is two SHI, the second carrying the bit; equal halves are a
    violation and set T without clearing it."""
    host = LoomHost(dut)
    await host.start()
    mon = PadMonitor(dut).start()
    tx, rx = 2, 1
    bits = [1, 0, 1, 1, 0, 0, 1]
    n = len(bits)
    sr0 = sum(b << i for i, b in enumerate(bits))
    cfg = ENC_MANCH
    ref = EncRef(cfg, sr=sr0, cnt=10)
    levels = [ref.sho() for _ in range(2 * n)]
    assert levels == [h for b in bits for h in ((~b) & 1, b)], \
        "the reference itself: two halves per bit, ~x then x"
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await setup_be(host, tx, cfg, be_pins(OUT0, 0), cnt=10, sr=sr0)
    marks = await run_sho_marks(host, mon, tx, 2 * n)
    assert [(mon.uo[m + 4] >> 0) & 1 for m in marks] == levels
    # Only the first half of each bit is a data bit: CNT fell by n, not 2n.
    assert await host.read_debug(tx, DBG_CNT) == 10 - n
    await check_engine(host, tx, ref, "Manchester TX")

    # Receive the same halves: seven bits arrive, no violation.
    rref = EncRef(cfg, cnt=9)
    for lv in levels:
        rref.shi(lv)
    assert rref.t == 0 and rref.cnt == 9 - n
    await setup_be(host, rx, cfg, be_pins(0, OUT2), cnt=9)
    await run_shi_levels(host, rx, levels, OUT2)
    await check_engine(host, rx, rref, "Manchester RX")
    assert rref.sr >> (16 - n) == sr0, "round trip"

    # A mid-bit violation: two equal halves. The bit is still received
    # (s = p on the second half), and T stays set through a later good bit.
    for half in (0, 1):
        bad = [half, half, 0, 1]          # one flat bit, then a good 1
        vref = EncRef(cfg, cnt=4)
        for lv in bad:
            vref.shi(lv)
        assert vref.t == 1, "the reference: equal halves are a violation"
        await setup_be(host, rx, cfg, be_pins(0, OUT2), cnt=4)
        await run_shi_levels(host, rx, bad, OUT2)
        assert await host.read_debug(rx, DBG_FLAGS) & FLAG_T, \
            f"flat half-bit {half} must set T"
        await check_engine(host, rx, vref, f"Manchester violation {half}")
    assert await host.badop() == 0
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_usb_stuffing(dut):
    """USB stuffing (STUFF = 1): a 0 after six 1s. On transmit the stuff bit
    leaves SR, CNT and the CRC alone; on receive it is dropped, and a
    seventh 1 where the stuff bit was due sets T."""
    host = LoomHost(dut)
    await host.start()
    mon = PadMonitor(dut).start()
    tx, rx = 0, 2
    preset = ISA.crc_presets["usb16"]
    poly = (preset["poly"] << (16 - preset["width"])) & 0xFFFF
    bits = [1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1]
    n = len(bits)
    sr0 = sum(b << i for i, b in enumerate(bits))
    cfg = STUFF_USB | CRC_EN
    ref = EncRef(cfg, sr=sr0, cnt=n, crc=0xFFFF, poly=poly)
    levels = ref.send(n)
    assert len(levels) == n + 2, "two stuff bits in this pattern"
    # Six 1s are bits 0..5 and bits 8..13 of the data, so the stuff bits
    # land at positions 6 and 15 on the line.
    assert levels[6] == 0 and levels[15] == 0, "a 0 after six 1s"

    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await setup_be(host, tx, cfg, be_pins(OUT0, 0), poly=poly, cnt=n,
                   sr=sr0, crc=0xFFFF)
    marks = await run_sho_marks(host, mon, tx, len(levels))
    assert [(mon.uo[m + 4] >> 0) & 1 for m in marks] == levels
    # The stuff bits touched neither CNT nor the CRC: 18 SHOs, 16 decrements
    # and a CRC over the 16 data bits only.
    assert await host.read_debug(tx, DBG_CNT) == 0
    assert await host.read_debug(tx, DBG_CRC) == serial_crc(bits, 0xFFFF, poly)
    await check_engine(host, tx, ref, "USB TX")

    # Receive the same line: the stuff bits are dropped, the data is back.
    rref = EncRef(cfg, cnt=n, crc=0xFFFF, poly=poly)
    for lv in levels:
        rref.shi(lv)
    assert rref.t == 0 and rref.cnt == 0
    await setup_be(host, rx, cfg, be_pins(0, OUT2), poly=poly, cnt=n,
                   crc=0xFFFF)
    await run_shi_levels(host, rx, levels, OUT2)
    await check_engine(host, rx, rref, "USB RX")
    assert rref.sr >> (16 - n) == sr0, "round trip"
    assert rref.crc == serial_crc(bits, 0xFFFF, poly), "CRC over data bits"

    # A seventh 1: the bit due is the stuff bit, it is not the stuff value,
    # so T is set and the bit is dropped (SR and CNT unchanged).
    seven = [1] * 7
    vref = EncRef(STUFF_USB, cnt=8)
    for lv in seven:
        vref.shi(lv)
    assert vref.t == 1 and vref.cnt == 2, "six data bits, the seventh dropped"
    await setup_be(host, rx, STUFF_USB, be_pins(0, OUT2), cnt=8)
    await run_shi_levels(host, rx, seven, OUT2)
    assert await host.read_debug(rx, DBG_FLAGS) & FLAG_T, "a seventh 1 sets T"
    await check_engine(host, rx, vref, "USB seventh 1")

    # Past six: RUN saturates at 7 and "the new run is six 1s" stays false,
    # so a stuck-high line costs one T, not one per bit
    # (docs/spec-questions/rtl-m3a.md 4).
    ten = [1] * 10
    sref = EncRef(STUFF_USB, cnt=12)
    for lv in ten:
        sref.shi(lv)
    assert sref.t == 1 and sref.cnt == 12 - 9, "nine data bits, one dropped"
    assert sref.enc_word == (1 << 4) | (7 << 1), "RUN saturates, PEND stays 0"
    await setup_be(host, rx, STUFF_USB, be_pins(0, OUT2), cnt=12)
    await run_shi_levels(host, rx, ten, OUT2)
    await check_engine(host, rx, sref, "USB run saturation")
    assert await host.badop() == 0
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_can_stuffing(dut):
    """CAN stuffing (STUFF = 2): the complement after five equal bits, and
    the stuff bit starts the next run. Both directions, and a receive whose
    sixth bit repeats instead of complementing sets T."""
    host = LoomHost(dut)
    await host.start()
    mon = PadMonitor(dut).start()
    tx, rx = 3, 1

    # Five 1s then zeros: the sixth bit on the line is the complement, it
    # leaves CNT alone, and the run restarts at 1 with the stuff value.
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await setup_be(host, tx, STUFF_CAN, be_pins(OUT0, 0), cnt=16, sr=0x001F)
    marks = await run_sho_marks(host, mon, tx, 6)
    assert [(mon.uo[m + 4] >> 0) & 1 for m in marks] == [1, 1, 1, 1, 1, 0]
    assert await host.read_debug(tx, DBG_CNT) == 11, "five data bits, one stuff"
    # {FIRST, HALF, PEND, RVAL, RUN, LVL} = run of one, value 0, nothing due.
    assert await host.read_debug(tx, DBG_ENC) == (1 << 1), \
        "the stuff bit starts the next run"

    # A longer pattern both ways, against the reference.
    bits = [1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1]
    n = len(bits)
    sr0 = sum(b << i for i, b in enumerate(bits))
    ref = EncRef(STUFF_CAN, sr=sr0, cnt=n)
    levels = ref.send(n)
    assert len(levels) > n, "the pattern must stuff at least once"
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await setup_be(host, tx, STUFF_CAN, be_pins(OUT0, 0), cnt=n, sr=sr0)
    marks = await run_sho_marks(host, mon, tx, len(levels))
    assert [(mon.uo[m + 4] >> 0) & 1 for m in marks] == levels
    await check_engine(host, tx, ref, "CAN TX")

    rref = EncRef(STUFF_CAN, cnt=n)
    for lv in levels:
        rref.shi(lv)
    assert rref.t == 0
    await setup_be(host, rx, STUFF_CAN, be_pins(0, OUT2), cnt=n)
    await run_shi_levels(host, rx, levels, OUT2)
    await check_engine(host, rx, rref, "CAN RX")
    assert rref.sr >> (16 - n) == sr0, "round trip"

    # Six equal bits on the wire: the sixth is the stuff bit and is not the
    # complement, so T is set.
    six = [1] * 6
    vref = EncRef(STUFF_CAN, cnt=8)
    for lv in six:
        vref.shi(lv)
    assert vref.t == 1 and vref.cnt == 3, "five data bits, the sixth dropped"
    await setup_be(host, rx, STUFF_CAN, be_pins(0, OUT2), cnt=8)
    await run_shi_levels(host, rx, six, OUT2)
    assert await host.read_debug(rx, DBG_FLAGS) & FLAG_T
    await check_engine(host, rx, vref, "CAN sixth equal bit")
    assert await host.badop() == 0
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_diff_output(dut):
    """DIFF (6.9.1 step 4): the same SHO also writes the complement to
    (BE_PINS.out + 1) mod 32 at the same edge. A non-writable index is
    ignored as in 6.3, and each of the two pins takes the open-drain rule
    on its own."""
    host = LoomHost(dut)
    await host.start()
    mon = PadMonitor(dut).start()
    t = 0
    bits = [1, 0, 0, 1, 1, 0]
    sr0 = sum(b << i for i, b in enumerate(bits))

    # OUT2 and OUT3: complementary, and INV inverts both sides together.
    for inv in (0, INV):
        await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
        await setup_be(host, t, DIFF | inv, be_pins(OUT2, 0), cnt=len(bits),
                       sr=sr0)
        marks = await run_sho_marks(host, mon, t, len(bits))
        for i, m in enumerate(marks):
            v = bits[i] ^ (1 if inv else 0)
            assert (mon.uo[m + 4] >> 2) & 1 == v, f"inv {inv} bit {i}: OUT2"
            assert (mon.uo[m + 4] >> 3) & 1 == v ^ 1, f"inv {inv} bit {i}: OUT3"

    # OUT5 + 1 is index 22, which is not writable: the complement is
    # dropped and only OUT5 moves. (OUT1 is the marker.)
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await setup_be(host, t, DIFF, be_pins(OUT5, 0), cnt=len(bits), sr=sr0)
    marks = await run_sho_marks(host, mon, t, len(bits))
    for i, m in enumerate(marks):
        assert (mon.uo[m + 4] >> 5) & 1 == bits[i], f"bit {i}: OUT5"
        assert mon.uo[m + 4] & 0b011101 == 0, \
            f"bit {i}: only OUT5 and the marker moved ({mon.uo[m + 4]:#04x})"

    # The other way round: index 15 is not writable but 15 + 1 = 16 (OUT0)
    # is, and 6.3 judges the two indices one at a time, so the complement
    # still lands (docs/spec-questions/rtl-m3a.md 5).
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await setup_be(host, t, DIFF, be_pins(15, 0), cnt=len(bits), sr=sr0)
    marks = await run_sho_marks(host, mon, t, len(bits))
    for i, m in enumerate(marks):
        assert (mon.uo[m + 4] >> 0) & 1 == bits[i] ^ 1, \
            f"bit {i}: OUT0 takes the complement of an unwritable out"
        assert mon.uo[m + 4] & 0b111100 == 0, f"bit {i}: nothing else moved"
        assert mon.uio[m + 4] == 0 and mon.oe[m + 4] == 0

    # BIDIR7 + 1 is index 8 (IN0), read only: the same, on the other side.
    await host.write(SP_CTRL, CTRL_PIN_OE, 0x80)
    await setup_be(host, t, DIFF, be_pins(7, 0), cnt=len(bits), sr=sr0)
    marks = await run_sho_marks(host, mon, t, len(bits))
    for i, m in enumerate(marks):
        assert (mon.uio[m + 4] >> 7) & 1 == bits[i], f"bit {i}: BIDIR7"
        assert mon.uio[m + 4] & 0x7F == 0, f"bit {i}: nothing else moved"
    assert all(v == 0x80 for v in mon.oe[marks[0]:]), "PIN_OE untouched"

    # Open drain, per pin: OD_MASK on BIDIR3 only, with BE_PINS.out = 2. The
    # plain pin takes PIN_OUT <- v; the open-drain pin takes PIN_OUT <- 0,
    # PIN_OE <- ~(~v) = v, so at the pad it pulls low exactly when v is 0.
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await host.write(SP_CTRL, CTRL_PIN_OE, 1 << 2)
    await host.write(SP_CTRL, CTRL_OD_MASK, 1 << 3)
    await setup_be(host, t, DIFF, be_pins(2, 0), cnt=len(bits), sr=sr0)
    marks = await run_sho_marks(host, mon, t, len(bits))
    for i, m in enumerate(marks):
        v = bits[i]
        assert (mon.uio[m + 4] >> 2) & 1 == v, f"bit {i}: BIDIR2 drives v"
        assert (mon.oe[m + 4] >> 2) & 1 == 1, f"bit {i}: BIDIR2 OE untouched"
        assert (mon.uio[m + 4] >> 3) & 1 == 0, f"bit {i}: BIDIR3 never high"
        assert (mon.oe[m + 4] >> 3) & 1 == v, f"bit {i}: BIDIR3 OE is v"
    await host.write(SP_CTRL, CTRL_OD_MASK, 0)
    await host.write(SP_CTRL, CTRL_PIN_OE, 0)
    assert await host.badop() == 0
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_encoder_state_cleared(dut):
    """The encoder state is cleared by every write to BE_CFG, from the
    thread or from the host, and by CTRL.RESET (6.9.1, SEMANTICS 7)."""
    host = LoomHost(dut)
    await host.start()
    mon = PadMonitor(dut).start()
    t = 1
    cfg = ENC_NRZI | STUFF_USB
    # A 0 then six 1s: LVL toggles to 1, the run reaches six 1s and a stuff
    # bit falls due. Every bit of the state except HALF and FIRST is set.
    ref = EncRef(cfg, sr=0x7E, cnt=16)
    ref.send(7)
    built = ref.enc_word
    assert built == 0x3D, f"the reference itself: {built:#04x}"
    await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
    await setup_be(host, t, cfg, be_pins(OUT0, 0), cnt=16, sr=0x7E)
    await run_sho_marks(host, mon, t, 7)
    assert await host.read_debug(t, DBG_ENC) == built, \
        "seven SHOs build the encoder state"

    # The thread's own CSRW BE_CFG, with the state preset through 0x27 so
    # the clearing write is the only thing the program does.
    prog = {0x100: asm("CSRW", csr=CSR_BE_CFG, ra=1), 0x101: asm("HALT")}
    for value in (cfg, cfg | DIR, 0):
        await host.load_program(prog, verify=False)
        await host.set_reset_pc(t, 0x100)
        await host.reset_thread(t)
        await host.write_reg(t, 1, value)
        await host.write_debug(t, DBG_ENC, built)
        assert await host.read_debug(t, DBG_ENC) == built
        await host.run(1 << t)
        await host.wait_halted(1 << t)
        assert await host.read_debug(t, DBG_ENC) == 0, \
            f"CSRW BE_CFG {value:#06x} clears the encoder state"

    # A host write of BE_CFG (debug 0x14), including one of the same value.
    for value in (cfg, cfg, 0):
        await host.write_debug(t, DBG_ENC, built)
        await host.write_csr(t, CSR_BE_CFG, value)
        assert await host.read_debug(t, DBG_ENC) == 0, \
            f"a host BE_CFG write of {value:#06x} clears the encoder state"

    # CTRL.RESET.
    await host.write_debug(t, DBG_ENC, built)
    assert await host.read_debug(t, DBG_ENC) == built
    await host.reset_thread(t)
    assert await host.read_debug(t, DBG_ENC) == 0, "CTRL.RESET clears it"
    # Only that thread's.
    await host.write_debug(t, DBG_ENC, built)
    await host.write_debug(0, DBG_ENC, 0x12)
    await host.reset_thread(0)
    assert await host.read_debug(t, DBG_ENC) == built, "thread 1 is untouched"
    assert await host.read_debug(0, DBG_ENC) == 0
    await host.write_debug(t, DBG_ENC, 0)
    assert await host.badop() == 0
    mon.stop()
    await ClockCycles(dut.clk, 2)


@cocotb.test()
async def test_debug_enc_word(dut):
    """Debug 0x27 while halted: it resets to 0, keeps eight bits, and every
    field sits where HOST_PROTOCOL SPACE 4 puts it -- proved by what the
    next SHO does, not by reading the word back."""
    host = LoomHost(dut)
    await host.start()
    mon = PadMonitor(dut).start()
    t = 2
    assert await host.read_debug(t, DBG_ENC) == 0, "reset value"
    await host.write_debug(t, DBG_ENC, 0xFFFF)
    assert await host.read_debug(t, DBG_ENC) == 0x00FF, "eight bits"
    for value in (0x5A, 0xA5, 0x00):
        await host.write_debug(t, DBG_ENC, value)
        assert await host.read_debug(t, DBG_ENC) == value
    assert await host.read_debug(0, DBG_ENC) == 0, "per thread"

    async def probe(cfg, enc0, n, sr, cnt):
        await host.write(SP_CTRL, CTRL_PIN_OUT, 0)
        await setup_be(host, t, cfg, be_pins(OUT0, 0), cnt=cnt, sr=sr)
        marks = await run_sho_marks(host, mon, t, n, enc0=enc0)
        return ([(mon.uo[m + 4] >> 0) & 1 for m in marks],
                await host.read_debug(t, DBG_CNT))

    # LVL, bit 0: NRZI sending a 1 holds the line at LVL.
    assert await probe(ENC_NRZI, 0x01, 1, sr=1, cnt=4) == ([1], 3)
    assert await probe(ENC_NRZI, 0x00, 1, sr=1, cnt=4) == ([0], 3)
    # PEND, bit 5: the next SHO sends the USB stuff value 0 instead of the
    # data 1, and leaves CNT alone.
    assert await probe(STUFF_USB, 0x20, 1, sr=0xFFFF, cnt=4) == ([0], 4)
    assert await probe(STUFF_USB, 0x00, 1, sr=0xFFFF, cnt=4) == ([1], 3)
    # RUN bits 3:1 and RVAL bit 4: a run of five 1s plus one more data 1 is
    # six 1s, so the second SHO is a stuff bit. With RVAL 0 the same RUN
    # value starts a fresh run instead.
    assert await probe(STUFF_USB, (5 << 1) | (1 << 4), 2, sr=0xFFFF, cnt=8) \
        == ([1, 0], 7)
    assert await probe(STUFF_USB, (5 << 1), 2, sr=0xFFFF, cnt=8) == ([1, 1], 6)
    assert await probe(STUFF_USB, 0x00, 2, sr=0xFFFF, cnt=8) == ([1, 1], 6)
    # HALF bit 6 and FIRST bit 7: with HALF set the next SHO is a second
    # half, which drives FIRST and consumes no data bit.
    assert await probe(ENC_MANCH, 0xC0, 2, sr=0x0000, cnt=4) == ([1, 1], 3)
    assert await probe(ENC_MANCH, 0x40, 2, sr=0x0000, cnt=4) == ([0, 1], 3)
    # With HALF clear the first SHO is a first half (one data bit, l = ~x)
    # and the second is its partner (no data bit), so CNT falls only once.
    assert await probe(ENC_MANCH, 0x00, 2, sr=0x0000, cnt=4) == ([1, 0], 3)
    assert await host.badop() == 0
    mon.stop()
    await ClockCycles(dut.clk, 2)
