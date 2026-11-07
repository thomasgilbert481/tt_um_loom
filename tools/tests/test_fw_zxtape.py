"""L3 for the ZX Spectrum tape programs, on the model and the RTL.

``firmware/zxtape_save.loom`` (MIC = OUT0) writes a block as the ROM's
SA-BYTES does; ``firmware/zxtape_load.loom`` (EAR = IN0) reads one as
LD-BYTES does. The references are ``tools.protomodels.zxtape``: its encoder
makes the pulses the loader is fed (``TapeSource``, with speed error and
jitter), its decoder reads the saver's pin, and ``Wire`` joins the two
programs when they run together (saver on thread 0, loader on thread 1).

Expected values come from the format's sources (the model's docstring:
the TZX specification and the ROM disassembly) and the program headers'
stated windows, never from what the model or the RTL did:

* saver: every pulse on the pin exactly the table's length in clocks (the
  T-state length at 3.5 MHz; the nearest whole clock at 50 MHz), the pilot
  count the ROM's rule gives, sync 1 and sync 2, two equal pulses per bit,
  MSB first, the checksum the XOR of flag and data, the line held at least
  1 ms after the last edge, then low;
* loader: every byte of a block whose pulses are 5 % slow or fast with
  jitter, the end-of-block word, and the checksum and fault words.

Two boards. The saver's tick is one clock and its table holds clocks, so at
50 MHz a block is about 300 k clocks a byte; the RTL runs both programs on a
board clocked at 3.5 MHz, the Spectrum's own clock, where one tick is one
clock is one T-state: 14 times fewer clocks. Every scenario also shortens
the pilot (the table's pilot counts, and the loader's NPIL) and the
saver's hold (to 14 x 255 = 3570 T, 1.02 ms): a ROM header's 8063 pilot
pulses are 17.5 M clocks even there. The 50 MHz bodies are ``model_only``.

Every body here is named ``test_zxtape_*``: test/test_fw.py collects the
bodies of all modules into one namespace.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import zxtape as Z
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512


class Board:
    """A clock and the ticks and tables the program headers give for it."""

    def __init__(self, name, clock_hz, load_tick, save_table, hold):
        self.name, self.clock_hz = name, float(clock_hz)
        self.save_tick = (1, 0)                 # zxtape_save: one clock
        self.load_tick = load_tick
        self.save_table = save_table            # PILOT SYNC1 SYNC2 ZERO ONE
        self.hold = hold                        # 255-tick steps, >= 1 ms

    def cycles(self, us):
        return int(round(us * self.clock_hz / 1e6))

    def t_clocks(self, t_states):
        return Z.clocks_of(t_states, self.clock_hz)

    def __repr__(self):
        return self.name


_R = Z.ROM
_PULSES = [_R.pilot, _R.sync1, _R.sync2, _R.zero, _R.one]
B35 = Board("3500kHz", 3_500_000, (1, 0), _PULSES, 14)
B50 = Board("50MHz", 50_000_000, (14, 73),
            [Z.clocks_of(t, 50e6) for t in _PULSES], 200)
#: the loader's windows (program header), in T-states
LOAD_TABLE = [1300, 3000, 512, 300, 1100, 2500, 2400, 5000]


def program(name, thread=0):
    """A program from ``firmware/``, moved to ``thread`` if asked."""
    path = FIRMWARE / (name + ".loom")
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, name + ".loom", isa=ISA, imem_words=IMEM, strict=True)


def save_image(board, thread=0, phdr=12, pdata=8):
    """The saver with its table for ``board`` and shortened pilots."""
    prog = program("zxtape_save", thread)
    words, tbl = dict(prog.words), prog.symbols["tbl"]
    for i, v in enumerate(board.save_table + [phdr, pdata, board.hold]):
        words[tbl + i] = v
    return words


def load_image(thread=0, npil=12):
    """The loader with its windows, NPIL shortened."""
    prog = program("zxtape_load", thread)
    words, tbl = dict(prog.words), prog.symbols["tbl"]
    table = list(LOAD_TABLE)
    table[2] = npil
    for i, v in enumerate(table):
        words[tbl + i] = v
    return words


def zx_bench(backend):
    """FIFOs, the latch, the data memory, 512 words; the RTL has them all
    and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, loads):
    """Load ``[(words, thread, (TICK_INT, TICK_FRAC))]`` and run the threads."""
    loom = backend.loom(bench, isa=ISA)
    words = {}
    for w, _, _ in loads:
        words.update(w)
    loom.load(words)
    for _, thread, (tint, tfrac) in loads:
        loom.write_csr(thread, "TICK_INT", tint)
        loom.write_csr(thread, "TICK_FRAC", tfrac)
    loom.run([thread for _, thread, _ in loads])
    return loom


# -------------------------------------------------------------------- saver
def save_setup(backend, board, **kw):
    bench = zx_bench(backend)
    rec = bench.add(Z.EdgeRecorder("OUT0"))
    loom = start(backend, bench, [(save_image(board, **kw), 0, board.save_tick)])
    return bench, loom, rec


def save_length(board, flag, data, pilot):
    """Clocks from the push to the report: the lead-in, every pulse, the
    hold, and 1 ms to spare."""
    pulses = Z.block_pulses(flag, data, pilot=pilot)
    return (board.t_clocks(Z.ROM.pilot + sum(pulses)) + board.hold * 255
            + board.cycles(1000))


def check_save(rec, board, flag, data, pilot):
    """The pin against the format: every pulse exactly its table length in
    clocks, the bytes read back by the model's decoder, and the hold."""
    pulses = Z.pulses_of(rec.cycles())
    want = [board.t_clocks(p) for p in Z.block_pulses(flag, data, pilot=pilot)]
    assert pulses[:len(want)] == want, [
        (i, a, b) for i, (a, b) in enumerate(zip(pulses, want)) if a != b][:6]
    got = Z.decode_block(pulses[:len(want)], unit=board.clock_hz / Z.T_HZ, tol=0.01)
    assert got.ok and got.pilot == pilot
    assert got.bytes == Z.block_bytes(flag, data)
    high_after = (len(want) + 1) % 2              # low before the first edge
    tail = pulses[len(want):]
    assert len(tail) == high_after                # then low: nothing more
    if high_after:
        assert tail[0] >= board.cycles(1000)      # held 1 ms first [TZX]
        assert rec.edges[-1][1] == 0
    return pulses


def test_zxtape_save_block(backend):
    """The PHY-GL-SDF candidate (about 130 k clocks with the load, at the
    3.5 MHz board): a data block (flag 0xFF, so the data pilot count) of
    two bytes, judged on the pin, and the checksum word in OUTQ."""
    bench, loom, rec = save_setup(backend, B35)
    flag, data = 0xFF, [0x3C, 0x81]
    loom.push(0, [flag, len(data)] + data)
    bench.step(save_length(B35, flag, data, 8))
    check_save(rec, B35, flag, data, 8)
    assert loom.pop_available(0) == [Z.checksum(flag, data)] == [0x42]
    assert loom.badop() == 0


@model_only("two blocks, 0.2 M clocks; the one-block scenario runs on the RTL")
def test_zxtape_save_pilot_rule(backend):
    """The ROM's rule [TZX ID 10]: a flag under 128 gets the header pilot
    count, 128 or more the data one (here 12 and 8 pulses); a block of no
    data bytes is flag and checksum only."""
    bench, loom, rec = save_setup(backend, B35, phdr=12, pdata=8)
    for flag, data, pilot in ((0x00, [0x13, 0x00, 0x7F], 12), (0x80, [], 8)):
        mark = len(rec.edges)
        loom.push(0, [flag, len(data)] + data)
        bench.step(save_length(B35, flag, data, pilot))
        assert loom.pop_available(0) == [Z.checksum(flag, data)]
        rec.edges[:] = rec.edges[mark:]
        check_save(rec, B35, flag, data, pilot)
    assert loom.badop() == 0


@model_only("0.6 M clocks at 50 MHz, 25 s on the model and 95 s of RTL; the "
            "3.5 MHz twin runs there")
def test_zxtape_save_exact_at_50mhz(backend):
    """At 50 MHz every pulse is the whole number of clocks nearest its
    T-state length: pilot 30971, sync 9529 and 10500, a 0 12214 and a 1
    24429 clocks (program header), with no drift from pulse to pulse."""
    assert B50.save_table == [30971, 9529, 10500, 12214, 24429]
    bench, loom, rec = save_setup(backend, B50, pdata=4)
    flag, data = 0x80, []                       # 0x80 twice: a 1 and seven 0s each
    loom.push(0, [flag, 0])
    bench.step(save_length(B50, flag, data, 4))
    pulses = check_save(rec, B50, flag, data, 4)
    assert set(pulses[:-1]) == {30971, 9529, 10500, 12214, 24429}
    assert loom.pop_available(0) == [0x80]      # 0x80 xor nothing
    assert loom.badop() == 0


@model_only("an error path, 0.15 M clocks; the RTL runs the good blocks")
def test_zxtape_save_underrun(backend):
    """A data byte not queued when its turn comes: the line goes low at
    once, 0x8000 is reported, the block's late data words are dropped as
    they come, and the next block goes out whole."""
    bench, loom, rec = save_setup(backend, B35)
    loom.push(0, [0xFF, 3, 0x11])               # three bytes promised, one sent
    bench.step(save_length(B35, 0xFF, [0x11], 8))
    assert loom.pop_available(0) == [0x8000]
    assert rec.edges[-1][1] == 0
    pulses = Z.pulses_of(rec.cycles())
    good = Z.block_pulses(0xFF, [0x11, 0x00, 0x00], pilot=8)[:8 + 2 + 32]
    assert pulses[:len(good)] == good           # flag and the one byte whole
    loom.push(0, [0x22, 0x33])                  # the late two: dropped
    bench.step(1000)
    rec.edges[:] = []
    flag, data = 0x00, [0x5A]
    loom.push(0, [flag, len(data)] + data)
    bench.step(save_length(B35, flag, data, 12))
    check_save(rec, B35, flag, data, 12)
    assert loom.pop_available(0) == [Z.checksum(flag, data)]


# ------------------------------------------------------------------- loader
OK, CKS, TRUNC = 0x4000, 0x8000, 0xC000         # end words (program header)


def load_setup(backend, board, npil=12):
    bench = zx_bench(backend)
    src = bench.add(Z.TapeSource("IN0"))
    loom = start(backend, bench, [(load_image(npil=npil), 0, board.load_tick)])
    return bench, loom, src


def play(bench, src, board, pulses, lead_us=500.0):
    """Play a pulse list (T-states) from ``lead_us`` on, then run 1.5 ms of
    silence past its last edge (the loader's end of block is 2500 T)."""
    edges = Z.edges_of(pulses, bench.cycle + board.cycles(lead_us), board.clock_hz)
    src.play(edges)
    bench.step(edges[-1] - bench.cycle + board.cycles(1500))


def ended(block, word=OK):
    """What the loader reports for a block read whole: its bytes, then the
    end word with their count."""
    return list(block) + [word | len(block)]


def bit_flipped(pulses, pilot, index):
    """``pulses`` with bit ``index`` of the block (0: the flag's MSB) read
    the other way: both its pulses swapped between a 0's and a 1's."""
    out = list(pulses)
    i = pilot + 2 + 2 * index
    swap = {Z.ROM.zero: Z.ROM.one, Z.ROM.one: Z.ROM.zero}
    out[i], out[i + 1] = swap[out[i]], swap[out[i + 1]]
    return out


def test_zxtape_load_block(backend):
    """A header-flagged block from the model's encoder at nominal speed: the
    loader reports the flag, the data byte and the checksum, then 0x4003."""
    bench, loom, src = load_setup(backend, B35)
    flag, data = 0x00, [0xA5]
    play(bench, src, B35, Z.block_pulses(flag, data, pilot=20))
    assert loom.pop_available(0) == ended(Z.block_bytes(flag, data))
    assert loom.badop() == 0


def test_zxtape_load_bad_checksum(backend):
    """One bit of the data byte read the other way on the tape: the bytes as
    they came, then 0x8003, the checksum fault."""
    bench, loom, src = load_setup(backend, B35)
    flag, data = 0xFF, [0x18]
    pulses = bit_flipped(Z.block_pulses(flag, data, pilot=20), 20, 8 + 3)
    play(bench, src, B35, pulses)
    assert loom.pop_available(0) == ended([0xFF, 0x08, 0xE7], CKS)
    assert loom.badop() == 0


@pytest.mark.parametrize("k", [0.95, 1.05])
def test_zxtape_load_tape_speed(backend, k):
    """The tape 5 % fast (every pulse 0.95 of nominal) and 5 % slow (1.05),
    each pulse with up to 40 T of jitter of its own: read whole."""
    bench, loom, src = load_setup(backend, B35)
    flag, data = 0xFF, [0x6E]
    play(bench, src, B35, Z.vary(Z.block_pulses(flag, data, pilot=20), k, 40.0))
    assert loom.pop_available(0) == ended(Z.block_bytes(flag, data))
    assert loom.badop() == 0


@model_only("the window edges, 0.25 M clocks; the 5 % cases run on the RTL")
@pytest.mark.parametrize("k", [0.72, 1.36])
def test_zxtape_load_speed_limits(backend, k):
    """Inside the stated tolerance (0.71 .. 1.38, program header) at both
    ends: every bit value, flag 0xFF and data 0x00 0x55, read whole (five
    report words: the fifth waits for room in OUTQ's four)."""
    bench, loom, src = load_setup(backend, B35)
    flag, data = 0xFF, [0x00, 0x55]
    play(bench, src, B35, Z.vary(Z.block_pulses(flag, data, pilot=20), k))
    got = loom.pop_available(0)
    bench.step(1000)
    assert got + loom.pop_available(0) == ended(Z.block_bytes(flag, data))


@model_only("fault paths, 0.3 M clocks")
def test_zxtape_load_faults(backend):
    """A block cut off after 12 bits: the flag, then 0xC001; too little
    pilot (8 of the 12 asked for): nothing; then a good block is read."""
    bench, loom, src = load_setup(backend, B35)
    pulses = Z.block_pulses(0xFF, [0x42], pilot=20)
    play(bench, src, B35, pulses[:20 + 2 + 24])
    assert loom.pop_available(0) == [0xFF, TRUNC | 1]
    play(bench, src, B35, Z.block_pulses(0x00, [0x42], pilot=8))
    assert loom.pop_available(0) == []          # (no run of 12 pilot-long pulses in it)
    play(bench, src, B35, pulses)
    assert loom.pop_available(0) == ended([0xFF, 0x42, 0xBD])


@model_only("0.9 M clocks at 50 MHz, 37 s on the model; the 3.5 MHz board "
            "runs on the RTL")
def test_zxtape_load_at_50mhz(backend):
    """The loader's 50 MHz tick (14 + 73/256 clocks, a T-state less 39 ppm)
    with a block 3 % slow."""
    bench, loom, src = load_setup(backend, B50)
    flag = 0x80
    play(bench, src, B50, Z.vary(Z.block_pulses(flag, [], pilot=14), 1.03, 20.0))
    assert loom.pop_available(0) == ended(Z.block_bytes(flag, []))


# ----------------------------------------------------------------- together
def test_zxtape_save_and_load_together(backend):
    """The saver (thread 0, OUT0) wired to the loader (thread 1, IN0): the
    loader reports every byte the saver sent and 0x4003; the saver reports
    its checksum."""
    bench = zx_bench(backend)
    bench.add(Z.Wire("OUT0", "IN0"))
    loom = start(backend, bench, [(save_image(B35, pdata=16), 0, B35.save_tick),
                                  (load_image(thread=1), 1, B35.load_tick)])
    flag, data = 0xFF, [0xC3]
    loom.push(0, [flag, len(data)] + data)
    bench.step(save_length(B35, flag, data, 16))
    assert loom.pop_available(0) == [Z.checksum(flag, data)]
    assert loom.pop_available(1) == ended(Z.block_bytes(flag, data))
    assert loom.badop() == 0
