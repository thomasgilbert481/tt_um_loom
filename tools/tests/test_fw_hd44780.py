"""L3-HD44780 on the model and the RTL: firmware/hd44780_4bit.loom, an
HD44780U character LCD on its 4-bit parallel bus (D-035).

The program (thread 0: D4..D7 = BIDIR0..3, RS = OUT0, R/W = OUT1, E = OUT2)
is fed through ``tools.loomhost.Loom`` and drives
``tools.protomodels.hd44780.Hd44780``, which checks every E pulse against
the datasheet's bus timing, follows the initialization flowchart, rejects
a byte sent while it is busy and renders its DDRAM as text.

What is expected comes from the HD44780U datasheet (ADE-207-272(Z), Rev.
0.0) as ``protomodels.hd44780`` cites it: the 2.7 to 4.5 V bus timing of
p.49 (Figures 25 and 26), Figure 24's initialization with its waits, the
instruction codes and execution times of Table 6, the 2-line DDRAM map of
Figure 4; and from the program header (the host words, the result codes),
never from what the model or the RTL did.

Most bodies run a 25 MHz build (``US = 25``, the program's tick still
1 us) and an LCD that has been powered for longer than Figure 24's 40 ms,
with the initialize operation's "supply already up" bit: the 41 ms
power-on wait alone would be a million clocks. The cold start runs on the
model only. Every body takes a ``backend`` (``tools/tests/fw_backend.py``):
pytest runs them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import functools

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.hd44780 import (EXEC_LONG_NS, EXEC_NS, FOSC, FOSC_TABLE,
                                       INIT_WAIT_2_NS, INIT_WAIT_3_NS,
                                       POWER_ON_NS, TIMING, Hd44780)
from tools.protomodels.uart import UartTx
from tools.tests.fw_backend import model_only

ISA = load()
SOURCE = REPO / "firmware" / "hd44780_4bit.loom"
US_LINE = ".equ    US = 50 "
STARTUP = 400                     # clocks for the program's set-up after RUN
OP_INIT, OP_INST, OP_DATA, OP_PRINT, OP_READ = 0, 1, 2, 3, 4
TIMED, WARM, TWO_LINES = 1, 2, 8  # the initialize operation's bits
ERROR, E_BUSY, E_OP = 0x8000, 0x0100, 0x0F00
V3 = TIMING["2.7-4.5V"]
INIT_NIBBLES = [0x30, 0x30, 0x30, 0x20]           # Figure 24, one nibble each
INIT_BYTES = [0x28, 0x08, 0x01, 0x06]             # 2 lines, off, clear, entry
POLLS = 200_000


@functools.lru_cache(maxsize=None)
def program(us=25, patch=None):
    """The program assembled with ``--strict``, its tick ``US`` clocks (the
    clock in MHz), ``patch``: ``(old, new)`` replaced once in the source."""
    text = SOURCE.read_text(encoding="utf-8")
    assert text.count(US_LINE) == 1
    text = text.replace(US_LINE, ".equ    US = %d " % us)
    if patch is not None:
        old, new = patch
        assert text.count(old) == 1, old
        text = text.replace(old, new)
    return assemble_text(text, SOURCE.name, isa=ISA, strict=True)


def setup(backend, *, us=25, fosc=FOSC["max"], rw_wired=True, powered=True,
          patch=None, lcd=True, pullups=0):
    """The program on thread 0 and a model LCD on its pins. ``powered``: the
    LCD's supply came up 50 ms before the bench started."""
    clock = us * 1_000_000
    model = Hd44780(clock=clock, fosc=fosc, rw_wired=rw_wired,
                    powered_at=-(clock // 20) if powered else 0)
    kwargs = dict(pullups=pullups)
    if backend.name == "model":
        kwargs.update(features=("FIFO", "DMEM"), imem_words=512)
    bench = backend.bench(**kwargs)
    if lcd:
        bench.add(model)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(us, patch))
    loom.run(0)
    bench.step(STARTUP)
    return bench, model, loom


def do(loom, words):
    """One operation: its words pushed, its result word popped."""
    loom.push(0, words, max_polls=POLLS)
    return loom.pop(0, 1, max_polls=POLLS)[0]


def op_print(address, text):
    """Print ``text`` from DDRAM ``address``: the operation and its words."""
    data = list(text.encode("ascii"))
    words = [data[i] | (data[i + 1] << 8 if i + 1 < len(data) else 0)
             for i in range(0, len(data), 2)]
    return [OP_PRINT << 12 | len(data) << 7 | address] + words


def init(loom, bits=WARM | TWO_LINES):
    assert do(loom, [OP_INIT << 12 | bits]) == OP_INIT << 12


def clean(bench, lcd):
    """No timing, protocol or busy violation, nothing rejected, no line
    driven both ways: and every measured interval inside p.49's minimums."""
    assert lcd.violations == []
    assert lcd.rejected == []
    assert bench.contention_free()
    for name, values in lcd.measured.items():
        assert all(v >= V3[name][0] for v in values), (name, min(values))


def writes(lcd, kind):
    return [(rs, b) for _, k, rs, b in lcd.events if k == kind]


# ------------------------------------------------------------- the program
@pytest.mark.parametrize("us", [50, 25, 12])
def test_hd44780_assembles_strict_with_no_diagnostic(us):
    """L2-DEADLINE: every pair proved at the 1 us tick the program sets
    (TICK_INT = US), from the 50 MHz build down to 12 clocks a tick; the
    main loop in thread 0's quarter and the subroutines in thread 3's."""
    prog = program(us)
    assert prog.diagnostics == []
    report = prog.deadlines[0]
    assert report.period == us
    assert report.pairs and not report.infeasible and not report.unbounded
    assert report.worst_slack >= 0
    assert list(prog.threads) == [0] and len(prog.words) == 181
    assert all(a < 0x080 or 0x180 <= a < 0x200 for a in prog.words)


def test_hd44780_tick_below_12_fails_the_checker():
    """The header's claim, from the other side: at 11 clocks a tick the
    busy-flag read (WAITD, IN, SETP, WAITD) no longer fits."""
    with pytest.raises(Exception):
        program(11)


# ------------------------------------------------------------- init, text
def test_init_hello_and_a_second_line(backend):
    """Figure 24's 4-bit initialization, then "HELLO" at row 0 and a second
    string at row 1, column 3 (DDRAM 0x43, Figure 4), read back from the
    model's DDRAM; the address counter read back over the bus after each
    (5, then 0x43 + 8). On the bus: the four single nibbles 3, 3, 3, 2 with
    more than 4.1 ms and more than 100 us after the first two (p.46), the
    function set 0x28 (two lines), display off, clear, entry mode 0x06,
    every E pulse inside p.49's 2.7 V column, every byte sent only once the
    busy flag is clear (nothing rejected), and nothing driven both ways
    while the LCD drives the busy flag."""
    bench, lcd, loom = setup(backend)
    init(loom)
    assert writes(lcd, "nibble8") == [(0, b) for b in INIT_NIBBLES]
    assert writes(lcd, "byte") == [(0, b) for b in INIT_BYTES]
    assert lcd.initialized and lcd.lines == 2 and lcd.font == 0
    starts = [c for c, k, _, _ in lcd.events if k == "nibble8"]
    gaps = [lcd.ns(b - a) for a, b in zip(starts, starts[1:])]
    assert gaps[0] > INIT_WAIT_2_NS and gaps[1] > INIT_WAIT_3_NS
    assert do(loom, [OP_INST << 12 | 0x0C]) == OP_INST << 12     # display on
    assert lcd.display and not lcd.cursor and not lcd.blink
    assert do(loom, op_print(0x00, "HELLO")) == OP_PRINT << 12
    assert do(loom, [OP_READ << 12]) == OP_READ << 12 | 5
    assert do(loom, op_print(0x43, "LOOM 054")) == OP_PRINT << 12
    assert do(loom, [OP_READ << 12]) == OP_READ << 12 | 0x4B
    assert lcd.render() == ["HELLO" + " " * 11, "   LOOM 054" + " " * 5]
    reads = [b for _, k, _, b in lcd.events if k == "read"]
    assert any(b & 0x80 for b in reads)          # the program saw BF = 1
    clean(bench, lcd)
    assert loom.badop() == 0


@model_only("the slowest LCD (190 kHz) only lengthens the polls the RTL "
            "already runs in test_init_hello_and_a_second_line")
def test_busy_flag_path_waits_out_clear_on_the_slowest_lcd(backend):
    """An LCD at p.49's slowest oscillator, 190 kHz: Clear display busy for
    1.52 ms x 270 / 190 = 2.16 ms (Table 6 prints no time for Clear; the
    model takes Return home's). The program polls BF (two nibbles a poll,
    Figure 9) until it clears: the entry mode set after it comes later
    than that, and so does every character after a 37 us write."""
    bench, lcd, loom = setup(backend, fosc=FOSC["min"])
    init(loom)
    cycles = {b: c for c, k, _, b in lcd.events if k == "byte"}
    long_ns = EXEC_LONG_NS * FOSC_TABLE / FOSC["min"]
    assert lcd.ns(cycles[0x06] - cycles[0x01]) > long_ns
    assert do(loom, op_print(0x00, "BUSY")) == OP_PRINT << 12
    data = [c for c, k, rs, _ in lcd.events if k == "byte" and rs]
    short_ns = EXEC_NS * FOSC_TABLE / FOSC["min"]
    assert all(lcd.ns(b - a) > short_ns for a, b in zip(data, data[1:]))
    busy = [b for _, k, _, b in lcd.events if k == "read" and b & 0x80]
    assert len(busy) > 2160 // 7            # a poll is about 6.5 us
    assert lcd.line(0).startswith("BUSY")
    clean(bench, lcd)


@model_only("240,000 clocks of timed waits; the RTL runs the busy-flag path, "
            "the same E strobe, in test_init_hello_and_a_second_line")
def test_timed_path_without_rw(backend):
    """R/W tied low on the board (the model sees 0 whatever the pin does):
    the initialize operation's timed bit makes the program wait the
    execution times instead, Table 6's scaled to the slowest oscillator of
    p.49, so an LCD at 190 kHz rejects nothing. No busy-flag read is made,
    and the read operation is refused (0xF: not on the timed path)."""
    bench, lcd, loom = setup(backend, fosc=FOSC["min"], rw_wired=False)
    init(loom, TIMED | WARM | TWO_LINES)
    assert writes(lcd, "byte") == [(0, b) for b in INIT_BYTES]
    assert do(loom, [OP_INST << 12 | 0x0C]) == OP_INST << 12
    assert do(loom, op_print(0x40, "TIMED")) == OP_PRINT << 12
    assert do(loom, [OP_READ << 12]) == ERROR | OP_READ << 12 | E_OP
    assert lcd.render() == [" " * 16, "TIMED" + " " * 11]
    assert writes(lcd, "read") == []
    clean(bench, lcd)


# ------------------------------------------------------- injected faults
def test_checker_catches_a_short_enable_pulse(backend):
    """A fault put in the program on purpose: the wait that holds E high
    for a tick is taken out, so E is high for two slots (8 clocks, 320 ns
    at 25 MHz) against p.49's PW_EH >= 450 ns. The model flags the first
    nibble."""
    patch = ("WAITD   1                       ; PW_EH", "NOP")
    bench, lcd, loom = setup(backend, patch=patch)
    loom.push(0, [OP_INIT << 12 | WARM | TWO_LINES])
    bench.step(3000)
    assert writes(lcd, "nibble8")[:1] == [(0, 0x30)]
    names = {name for _, name, _ in lcd.violations}
    assert "PW_EH" in names
    pw = [v for v in lcd.measured["PW_EH"]]
    assert pw and max(pw) < V3["PW_EH"][0]


def test_checker_catches_an_init_too_soon_after_power_on(backend):
    """The host claims the supply has been up for 40 ms (the warm bit) when
    the LCD was powered as the bench started: Figure 24 needs more than
    40 ms after VCC reaches 2.7 V before the first 0x3."""
    bench, lcd, loom = setup(backend, powered=False)
    loom.push(0, [OP_INIT << 12 | WARM | TWO_LINES])
    bench.step(3000)
    assert writes(lcd, "nibble8")[:1] == [(0, 0x30)]
    assert any(name == "init" and "more than 40000000" in text
               for _, name, text in lcd.violations)


@model_only("an out-of-spec LCD on the timed path; seven milliseconds of "
            "timed waits (about 180,000 clocks) for a fault the model alone injects")
def test_checker_catches_a_byte_sent_while_busy(backend):
    """The timed path assumes p.49's slowest oscillator, 190 kHz. An LCD
    below the datasheet, at 175 kHz, takes 37 x 270 / 175 = 57 us for a
    byte, inside the 61 us wait, but is still busy with Clear display
    (1.52 ms x 270 / 175 = 2.35 ms) when the 2.2 ms wait ends: the model
    rejects the entry mode set that follows, and only that, and says so."""
    bench, lcd, loom = setup(backend, fosc=175_000, rw_wired=False)
    init(loom, TIMED | WARM | TWO_LINES)
    assert [b for _, _, b in lcd.rejected] == [0x06]
    assert [name for _, name, _ in lcd.violations] == ["busy"]


# ------------------------------------------------------------------ errors
def test_operations_refused_before_init_and_unknown(backend):
    """Before the first initialize every other operation is refused (bit
    15, 0xF) and nothing reaches the bus; operation 5 is no operation."""
    bench, lcd, loom = setup(backend)
    assert do(loom, [OP_DATA << 12 | 0x41]) == ERROR | OP_DATA << 12 | E_OP
    assert do(loom, op_print(0, "")) == ERROR | OP_PRINT << 12 | E_OP
    assert do(loom, [0x5123]) == ERROR | 0x5000 | E_OP
    assert lcd.events == [] and loom.badop() == 0


@model_only("two 5 ms busy-flag timeouts: 250,000 clocks of polling")
def test_no_lcd_times_out_and_keeps_the_host_words_in_step(backend):
    """No LCD, D4..D7 pulled up: BF reads 1 for ever. The initialize gives
    up after BFMAX polls on its first two-nibble byte and reports 0x1; a
    print then fails on its address byte, still takes its three data words
    from INQ, and the next operation is read as one."""
    bench, lcd, loom = setup(backend, lcd=False, pullups=0x0F)
    assert do(loom, [OP_INIT << 12 | WARM | TWO_LINES]) == ERROR | E_BUSY
    assert do(loom, op_print(0, "HELLO")) == ERROR | OP_PRINT << 12 | E_BUSY
    assert loom.read_reg(0, "INQ_CNT") == 0
    assert do(loom, [0x7000]) == ERROR | 0x7000 | E_OP
    assert bench.contention_free()


@model_only("the 41 ms power-on wait is 500,000 clocks even at 12 MHz")
def test_cold_start_waits_out_power_on(backend):
    """From power on (the LCD powered as the bench starts, no warm bit):
    the first 0x3 comes more than 40 ms after VCC reached 2.7 V (Figure
    24), and the model accepts the whole sequence. On the build for a 12 MHz
    clock (US 12, the shortest tick the pairs are proved for), which also
    halves the clocks the 41 ms take."""
    bench, lcd, loom = setup(backend, us=12, powered=False)
    init(loom, TWO_LINES)
    first = [c for c, k, _, _ in lcd.events if k == "nibble8"][0]
    assert lcd.ns(first) > POWER_ON_NS["2.7-4.5V"]
    assert lcd.initialized
    clean(bench, lcd)


@model_only("the instruction set beyond the RTL scenarios; same E strobe")
def test_instructions_and_ddram_addressing(backend):
    """Prints of 0, 1, 3 and 16 characters (an odd count leaves the last
    word's high byte unused), the line end of Figure 4 (0x27 runs on to
    0x40), a data write on its own, Return home (1.52 ms, polled) and a
    cursor shift (Table 6), all against the model's DDRAM."""
    bench, lcd, loom = setup(backend)
    init(loom)
    assert do(loom, op_print(0x05, "")) == OP_PRINT << 12
    assert do(loom, [OP_READ << 12]) == OP_READ << 12 | 0x05
    assert do(loom, op_print(0x00, "0123456789ABCDEF")) == OP_PRINT << 12
    assert do(loom, op_print(0x26, "xyz")) == OP_PRINT << 12
    assert [lcd.ddram[a] for a in (0x26, 0x27, 0x40)] == list(b"xyz")
    assert do(loom, [OP_INST << 12 | 0x02]) == OP_INST << 12    # home
    assert do(loom, [OP_DATA << 12 | ord("#")]) == OP_DATA << 12
    assert do(loom, [OP_INST << 12 | 0x14]) == OP_INST << 12    # cursor right
    assert do(loom, [OP_READ << 12]) == OP_READ << 12 | 0x02
    assert do(loom, op_print(0x42, "!")) == OP_PRINT << 12
    assert do(loom, [OP_READ << 12]) == OP_READ << 12 | 0x43
    assert lcd.render() == ["#123456789ABCDEF", "z !" + " " * 13]
    clean(bench, lcd)


# ------------------------------------------------------------- the bridge
BRIDGE = REPO / "firmware" / "bridge_uart_lcd.loom"
BOOT_LINE = ".equ    BOOT = 0x0008 "


@functools.lru_cache(maxsize=None)
def bridge(us=25, warm=True):
    """bridge_uart_lcd.loom with ``--strict``; ``warm``: the boot
    initialization takes the supply as already up (the model LCD was
    powered 50 ms before the bench started), which saves 41 ms."""
    text = BRIDGE.read_text(encoding="utf-8")
    assert text.count(US_LINE) == 1 and text.count(BOOT_LINE) == 1
    text = text.replace(US_LINE, ".equ    US = %d " % us)
    if warm:
        text = text.replace(BOOT_LINE, ".equ    BOOT = 0x000A ")
    return assemble_text(text, BRIDGE.name, isa=ISA, strict=True)


def test_bridge_assembles_strict_with_no_diagnostic():
    """Both threads' pairs proved: thread 0 at its 1 us tick (US 50), thread
    1 (the receiver) for ticks of 16 clocks or more; thread 0 in quarters 0
    and 3 (code, then HEAD, TAIL and the 32-byte ring at 0x1DE), thread 1
    in quarter 1; threads 2 and 3 not startable from this image."""
    prog = bridge(50, warm=False)
    assert prog.diagnostics == []
    for thread, period in ((0, 50), (1, 16)):
        report = prog.deadlines[thread]
        assert report.period == period
        assert report.pairs and not report.infeasible and not report.unbounded
        assert report.worst_slack >= 0
    assert {t: prog.threads[t].size for t in prog.threads} == {0: 212, 1: 32}
    assert all(a < 0x0A0 or 0x180 <= a for a in prog.words)
    assert all(prog.words[a] == 0 for a in range(0x1DE, 0x200))


def bridge_setup(backend, *, us=25, fosc=FOSC["max"], baud=115_200):
    """The bridge on threads 0 and 1, the LCD on thread 0's pins, a UART
    sender on IN0; thread 1's tick one eighth of a bit."""
    clock = us * 1_000_000
    lcd = Hd44780(clock=clock, fosc=fosc, powered_at=-(clock // 20))
    kwargs = {}
    if backend.name == "model":
        kwargs.update(features=("FIFO", "DMEM"), imem_words=512)
    bench = backend.bench(**kwargs)
    bench.add(lcd)
    bit = clock / baud
    uart = bench.add(UartTx("IN0", bit))
    loom = backend.loom(bench, isa=ISA)
    loom.load(bridge(us))
    tick = bit / 8
    loom.write_csr(1, "TICK_INT", int(tick))
    loom.write_csr(1, "TICK_FRAC", int(round((tick - int(tick)) * 256)))
    loom.run([0, 1], exclusive=True)
    return bench, lcd, uart


def settle(bench, lcd, uart, cycles):
    """Run until the sender is done and the LCD has been idle a while."""
    assert bench.run_until(lambda: not uart.busy, 10 ** 7, every=1000)
    bench.step(cycles)


def test_bridge_shows_two_lines_from_the_uart(backend):
    """"HELLO", LF, "WORLD" at 115200 baud: the boot initialization (Figure
    24, then the display on), the two words on the two lines; then FF
    (Clear display, 1.52 ms, polled) and "AB", CR, "C": CR returns to the
    line's start, so C overwrites A. Every E pulse in p.49's window,
    nothing sent while busy, nothing driven both ways."""
    bench, lcd, uart = bridge_setup(backend)
    assert bench.run_until(lambda: lcd.display, 400_000, every=2000)
    assert writes(lcd, "nibble8") == [(0, b) for b in INIT_NIBBLES]
    assert writes(lcd, "byte")[:5] == [(0, b) for b in INIT_BYTES + [0x0C]]
    uart.send(b"HELLO\nWORLD")
    settle(bench, lcd, uart, 5_000)
    assert lcd.render() == ["HELLO" + " " * 11, "WORLD" + " " * 11]
    uart.send(b"\x0cAB\rC")
    settle(bench, lcd, uart, 60_000)
    assert lcd.render() == ["CB" + " " * 14, " " * 16]
    clean(bench, lcd)


@model_only("a 4 ms stream on the slowest LCD; the RTL runs the short one")
def test_bridge_wraps_and_buffers_a_clear_on_the_slowest_lcd(backend):
    """Bytes back to back at 115200 baud (one every 86.8 us) into an LCD at
    190 kHz: 20 characters wrap after the 16th to the second line; then FF
    and 20 more at once, which arrive while Clear display keeps the LCD
    busy for 2.16 ms (Table 6 scaled) and wait in the ring (24 bytes in
    2.16 ms, of 31); a frame with a low stop bit is dropped. Nothing is
    lost and nothing is sent while the LCD is busy."""
    bench, lcd, uart = bridge_setup(backend, fosc=FOSC["min"])
    assert bench.run_until(lambda: lcd.display, 600_000, every=2000)
    uart.send(b"ABCDEFGHIJKLMNOPQRST")
    uart.send_frame(ord("X"), stop=0)
    settle(bench, lcd, uart, 5_000)
    assert lcd.render() == ["ABCDEFGHIJKLMNOP", "QRST" + " " * 12]
    uart.send(b"\x0c0123456789abcdefghij")
    settle(bench, lcd, uart, 80_000)        # the ring drains, 60 us a byte
    assert lcd.render() == ["0123456789abcdef", "ghij" + " " * 12]
    clean(bench, lcd)
