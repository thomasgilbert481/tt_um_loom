"""L3-SDI12 on the model and the RTL: firmware/sdi12_recorder.loom and sdi12_sensor.loom.

The recorder (thread 0: TX = OUT0, DIR = OUT1, RX = IN0) sends the
commands the host gives it and reports the responses; the sensor (thread
0 alone, or thread 1 beside the recorder with OUT2, OUT3, IN1) answers a!,
?!, aI!, aM! and aD0! with strings the host loads.
``tools.protomodels.sdi12.Sdi12Line`` is the one data line: each chip port
drives it while its DIR pin is 1, the model devices drive it too, and every
RX pin sees it.

Expected values come from SDI-12 version 1.4 through the reference model
(the 7E1 frame, the command and response formats, section 7's timing and
7.2's retry rule) and from the program headers (the host formats), never
from what the model or the RTL did. Timing is measured on the line with
``check_transaction``: the break, the marking after it, the release, the
response window, the sensor's own marking, the gaps between characters.

Clocks. At 1200 baud a bit is 41,667 clocks at 50 MHz, too slow for the
RTL; the tests run the chip on a slow clock of 230.4 kHz instead, where a
tick of 24 clocks is an eighth of a bit and every figure in milliseconds
is the standard's. One scenario runs at 50 MHz on the model.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.sdi12 import (MS, RETRY_MAX, RETRY_MIN, WAKE_MAX,
                                     Sdi12Line, Sdi12Recorder, Sdi12Sensor,
                                     check_transaction, text)
from tools.tests.fw_backend import model_only

ISA = load()
RECORDER = REPO / "firmware" / "sdi12_recorder.loom"
SENSOR = REPO / "firmware" / "sdi12_sensor.loom"
IMEM = 512

SLOW = 230_400                      # the test clock: 8 ticks of 24 clocks a bit
TICKS = {SLOW: (24, 0), 50_000_000: (5208, 85), 2_000_000: (208, 85)}

# The recorder's end-of-try word (program header).
END, LAST = 1 << 15, 1 << 14
TIMEOUT, PARITY, FRAMING, FORMAT, OVERRUN = (1 << 8, 1 << 9, 1 << 10, 1 << 11,
                                             1 << 12)
BREAK = 1 << 8                      # the first command word: force a break

# What the sensors answer (section 4.4, Tables 6, 7, 9, 11).
IDENT = "14LOOMTT  SDI12S001"       # ll, vendor (8), model (6), version (3)
MEAS = "0001"                       # ttt = 000, n = 1
DATA0 = "+21.5"
RESPONSES = {"!": "", "I!": IDENT, "M!": MEAS, "D0!": DATA0}


def sdi_bench(backend):
    """FIFOs, the bit engine, LD/ST and the 512-word memory on the model;
    the RTL has all of them and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "DMEM"), imem_words=IMEM)
    return backend.bench()


def start(loom, path, clock, threads=(0,)):
    loom.load(assemble_file(path, isa=ISA, strict=True).words)
    ti, tf = TICKS[clock]
    for t in threads:
        loom.write_csr(t, "TICK_INT", ti)
        loom.write_csr(t, "TICK_FRAC", tf)
    loom.run(list(threads))


def bits(clock, n):
    return int(n * clock / 1200)


# ------------------------------------------------------------------- build
#: program -> (proved tick, code words at its entry, all words) (headers)
BUILD = {RECORDER: (16, 126, 208), SENSOR: (24, 126, 175)}


@pytest.mark.parametrize("path", sorted(BUILD), ids=lambda p: p.stem)
def test_programs_assemble_with_no_diagnostic(path):
    """--strict and no diagnostic at all: every deadline pair proved at the
    tick the header gives, nothing unbounded, the recorder's one
    .bounded PUSH, and the word counts the headers give."""
    prog = assemble_file(path, isa=ISA, strict=True)
    tick, main, total = BUILD[path]
    assert prog.diagnostics == []
    report = prog.deadlines[0]
    assert report.period == tick
    assert report.worst_slack is not None and report.worst_slack >= 0
    entry = prog.threads[0].entry
    assert sum(1 for a in prog.words if entry <= a < entry + 128) == main
    assert len(prog.words) == total


def test_programs_fit_together():
    """Recorder and sensor in one 512-word image without overlap; thread 1
    runs the sensor from 0x080."""
    words = together_image()
    assert len(words) == sum(t for _, _, t in BUILD.values())


# ---------------------------------------------------------------- recorder
def recorder_setup(backend, clock=SLOW, **sensor):
    """The chip recorder with a model sensor at address '0' on the line;
    returns ``(bench, loom, line, sensor)``."""
    bench = sdi_bench(backend)
    dev = Sdi12Sensor(sensor.pop("address", "0"), sensor.pop("responses", RESPONSES),
                      clock=clock, **sensor)
    line = bench.add(Sdi12Line([dev], clock=clock))
    loom = backend.loom(bench, isa=ISA)
    start(loom, RECORDER, clock)
    bench.step(64)
    return bench, loom, line, dev


def ask(bench, loom, command, *, force_break=False, max_bits=1400):
    """The host sends ``command`` to the chip recorder and reads OUTQ[0]
    until the last word for it: returns ``[(characters, end word)]``, one
    per try (program header)."""
    words = [ord(c) for c in command]
    if force_break:
        words[0] |= BREAK
    loom.push(0, words)
    tries, chars = [], []
    T = bench.models[0].T
    limit = bench.cycle + int(max_bits * T)
    while bench.cycle < limit:
        for w in loom.pop_available(0):
            if w & END:
                tries.append((text(chars), w))
                chars = []
                if w & LAST:
                    return tries
            else:
                chars.append(w & 0x7F)
        bench.step(int(3 * T))
    raise AssertionError("no last word after %d bits: %r %r" % (max_bits, tries, chars))


def assert_valid(tries, response, count=1):
    """``count`` tries, the last one valid with ``response``."""
    assert len(tries) == count, tries
    got, word = tries[-1]
    assert word == END | LAST | count, (hex(word), tries)
    assert got == response, (got, response)


def assert_on_time(timing, brk):
    """Section 7 on the line: no violation, and a break exactly when one
    was due."""
    assert timing.violations == [], (timing.violations, timing)
    assert (timing.break_ms is not None) == brk, timing


def test_recorder_acknowledge(backend):
    """The quick one: 0! after a break, the model sensor's 0<CR><LF> in one
    try, the break, the marking and the release within section 7."""
    bench, loom, line, sensor = recorder_setup(backend)
    tries = ask(bench, loom, "0!")
    assert_valid(tries, "0\r\n")
    timing = check_transaction(line, "chip0", "sensor")
    assert_on_time(timing, True)
    assert timing.command == "0!" and timing.response == "0\r\n"
    assert 12.0 <= timing.break_ms <= 13.0, timing.break_ms         # 15 bits
    assert 8.33 <= timing.mark_ms <= 9.6, timing.mark_ms            # 11 bits
    assert timing.release_ms <= 0.1, timing.release_ms
    assert timing.gaps_ms and max(timing.gaps_ms) <= 0.1, timing.gaps_ms
    assert line.contentions == [] and loom.badop() == 0


def test_recorder_identify_measure_and_data(backend):
    """0I!, 0M!, 0D0! one after another: the first after a break, the
    others without one (the same sensor, the line marking less than 87
    ms); every response whole and valid, every transaction on time."""
    bench, loom, line, sensor = recorder_setup(backend)
    for i, (cmd, body) in enumerate((("0I!", IDENT), ("0M!", MEAS), ("0D0!", DATA0))):
        since = bench.cycle
        assert_valid(ask(bench, loom, cmd), "0" + body + "\r\n")
        assert_on_time(check_transaction(line, "chip0", "sensor", since), i == 0)
    assert [c.text for c in sensor.commands] == ["0I!", "0M!", "0D0!"]
    assert line.contentions == [] and loom.badop() == 0


@model_only("the break rule's other two cases; the RTL runs the first-command break")
def test_recorder_breaks_when_due(backend):
    """A break precedes a command to another sensor and one after 87 ms of
    marking (7.0, 7.1); after 40 ms of marking it is not needed. The
    recorder sends one after 80 ms (its header), and when the host sets
    bit 8."""
    bench, loom, line, s0 = recorder_setup(backend)
    s1 = Sdi12Sensor("1", RESPONSES, clock=SLOW, name="sensor1")
    line.add(s1)
    cases = [("0!", 0, True, s0), ("0!", 40, False, s0), ("1!", 0, True, s1),
             ("1!", 82, True, s1), ("1!", 0, False, s1)]
    for cmd, idle_ms, brk, dev in cases:
        bench.step(int(idle_ms * MS * SLOW))
        since = bench.cycle
        assert_valid(ask(bench, loom, cmd), cmd[0] + "\r\n")
        timing = check_transaction(line, "chip0", dev.name, since)
        assert_on_time(timing, brk)
    since = bench.cycle
    assert_valid(ask(bench, loom, "1!", force_break=True), "1\r\n")
    assert_on_time(check_transaction(line, "chip0", "sensor1", since), True)


def retry_starts(line, since):
    """``(cycle, break)`` of every command the chip recorder started after
    ``since``: the first start bit of each, and whether a break came
    before it."""
    out, prev = [], None
    for ch in line.chars_by("chip0"):
        if ch.start < since:
            continue
        if prev is None or prev.value == ord("!"):
            brk = any(prev_end <= b.start and b.end <= ch.start
                      for b in line.decoder.breaks
                      for prev_end in [prev.end if prev else since])
            out.append((ch.start, brk))
        prev = ch
    return out


@model_only("the wake test runs two retries without a break on the RTL")
def test_recorder_retries_a_silent_sensor(backend):
    """The sensor leaves the first command unanswered: after 17.5 ms
    (16.67 to 87 ms, 7.2) the recorder sends it again without a break and
    reports the timeout of try 1 and the response of try 2."""
    bench, loom, line, sensor = recorder_setup(backend, silent=1)
    since = bench.cycle
    tries = ask(bench, loom, "0!")
    assert tries[0] == ("", END | TIMEOUT | 1), tries
    assert_valid(tries, "0\r\n", 2)
    (first, b1), (second, b2) = retry_starts(line, since)
    cmd_end = sensor.commands[0].end
    wait = (second - cmd_end) / SLOW
    assert b1 and not b2
    assert RETRY_MIN <= wait - 1 / 1200 <= RETRY_MAX, wait      # less its marking bit


def test_recorder_retries_after_a_parity_error(backend):
    """A parity error in the response's second character: try 1 is
    reported with its characters and the parity flag, and once the
    response has ended (7.2) try 2 gets a good one."""
    bench, loom, line, sensor = recorder_setup(backend, parity_error_at=1)
    tries = ask(bench, loom, "0M!")
    assert len(tries) == 2, tries
    assert tries[0][1] == END | PARITY | 1, hex(tries[0][1])
    assert len(tries[0][0]) == len("0" + MEAS + "\r\n")
    assert_valid(tries, "0" + MEAS + "\r\n", 2)
    (first, _), (second, brk) = retry_starts(line, 0)
    assert not brk and second > sensor.commands[0].end + 6 * 10 * SLOW / 1200


@model_only("one more invalid-response case; the parity one runs on the RTL")
def test_recorder_rejects_a_wrong_address(backend):
    """A response that starts with another address is invalid (format);
    the sensor does it every time, so after nine tries the recorder gives
    up and says so."""
    bench, loom, line, sensor = recorder_setup(backend, answer_as="5")
    tries = ask(bench, loom, "0!", max_bits=3000)
    assert len(tries) == 9
    for k, (got, word) in enumerate(tries, 1):
        assert got == "5\r\n" and word == END | FORMAT | k | (LAST if k == 9 else 0)


def test_recorder_waits_for_a_sensor_to_wake(backend):
    """A sensor may take 100 ms to wake after a break (7.0); the model one
    answers nothing that ends sooner. The first two tries go unanswered
    and the third, which 7.2 puts more than 100 ms after the break (after
    its end, so after its fall too), is answered."""
    bench, loom, line, sensor = recorder_setup(backend, wake_ms=100.0)
    tries = ask(bench, loom, "0!")
    assert [w for _, w in tries[:2]] == [END | TIMEOUT | 1, END | TIMEOUT | 2]
    assert_valid(tries, "0\r\n", 3)
    starts = retry_starts(line, 0)
    assert [b for _, b in starts] == [True, False, False]
    brk = line.decoder.breaks[0]
    assert (starts[2][0] - brk.end) / SLOW > WAKE_MAX      # so after its fall too


@model_only("nine tries, about 800 ms of line time")
def test_recorder_gives_up_after_three_sequences(backend):
    """A sensor that never answers: nine tries in three sequences, each led
    by a break (7.2), the retries 16.67 to 87 ms after the command before
    them, and the third of each sequence over 100 ms after its break's end."""
    bench, loom, line, sensor = recorder_setup(backend, silent=9)
    tries = ask(bench, loom, "0!", max_bits=3000)
    assert [w for _, w in tries] == [END | TIMEOUT | k | (LAST if k == 9 else 0)
                                     for k in range(1, 10)]
    starts = retry_starts(line, 0)
    assert [b for _, b in starts] == [True, False, False] * 3
    ends = [c.end for c in sensor.commands]
    breaks = line.decoder.breaks
    assert len(ends) == 9 and len(breaks) == 3
    for k in range(1, 9):
        if k % 3:
            wait = (starts[k][0] - ends[k - 1]) / SLOW - 1 / 1200
            assert RETRY_MIN <= wait <= RETRY_MAX, (k, wait)
    for s in range(3):
        assert (starts[3 * s + 2][0] - breaks[s].end) / SLOW > WAKE_MAX


@model_only("a check of the checker, with the recorder as the bench")
def test_checker_sees_a_late_response(backend):
    """A sensor that starts its response 16 ms after the command, 0.6 ms
    over 15 ms + 0.40 (7.0): the recorder's 17.5 ms window still takes it,
    and check_transaction reports the late start."""
    bench, loom, line, sensor = recorder_setup(backend, start_ms=16.0)
    assert_valid(ask(bench, loom, "0!"), "0\r\n")
    timing = check_transaction(line, "chip0", "sensor")
    assert timing.violations == ["response started at 16.00 ms > 15 ms"], timing


# ------------------------------------------------------------------ sensor
def sensor_words(address="0", ident=IDENT, meas=MEAS, data=DATA0):
    """The sensor's set-up words: the address, then the aI!, aM! and aD0!
    strings, each ending with a 0 word (program header)."""
    words = [ord(address)]
    for s in (ident, meas, data):
        words += [ord(c) for c in s] + [0]
    return words


def sensor_setup(backend, clock=SLOW, address="0"):
    """The chip sensor at ``address`` with a model recorder on the line;
    returns ``(bench, loom, line, recorder)``."""
    bench = sdi_bench(backend)
    rec = Sdi12Recorder(clock=clock)
    line = bench.add(Sdi12Line([rec], clock=clock))
    loom = backend.loom(bench, isa=ISA)
    start(loom, SENSOR, clock)
    loom.push(0, sensor_words(address))
    bench.step(bits(clock, 4))
    return bench, loom, line, rec


def transact(bench, rec, cmd, **kwargs):
    """The model recorder sends ``cmd`` and waits for the answer, giving up
    60 ms after the command or the last character; returns the
    :class:`Command` record."""
    c = rec.command(cmd, **kwargs)
    clock = rec.clock_hz
    assert bench.run_until(lambda: not rec.busy, int(clock), every=64)
    seen = {"n": 0, "at": c.end}

    def over():
        if len(c.response) != seen["n"]:
            seen["n"], seen["at"] = len(c.response), bench.cycle
        return c.answered or bench.cycle >= seen["at"] + 0.060 * clock

    bench.run_until(over, int(clock), every=64)
    if c.answered:
        bench.step(bits(clock, 1))      # the release after the <LF>
    return c


def test_sensor_acknowledge(backend):
    """The quick one: a break, 0!, and the chip's 0<CR><LF>: its own 8.33 ms
    of marking, the first start bit within 15 ms of the command, the
    characters back to back and the release, all within section 7."""
    bench, loom, line, rec = sensor_setup(backend)
    c = transact(bench, rec, "0!")
    assert text(c.response) == "0\r\n"
    timing = check_transaction(line, "recorder", "chip0")
    assert_on_time(timing, True)
    assert 9.9 <= timing.start_ms <= 10.2, timing.start_ms       # 12 bits
    assert 8.33 <= timing.sensor_mark_ms <= 8.5, timing.sensor_mark_ms
    assert timing.sensor_release_ms <= 0.1, timing.sensor_release_ms
    assert line.contentions == [] and loom.badop() == 0


@model_only("about 160,000 clocks; the together test runs aD0! on the RTL, the recorder's aI! test the long frame")
def test_sensor_identify_measure_and_data(backend):
    """aI!, aM! and aD0! after one break: the strings the host loaded,
    behind the address, each response on time."""
    bench, loom, line, rec = sensor_setup(backend, address="7")
    for i, (cmd, body) in enumerate((("7I!", IDENT), ("7M!", MEAS), ("7D0!", DATA0))):
        since = bench.cycle
        c = transact(bench, rec, cmd, break_ms=12.5 if i == 0 else None)
        assert text(c.response) == "7" + body + "\r\n"
        assert_on_time(check_transaction(line, "recorder", "chip0", since), i == 0)
    assert line.contentions == [] and loom.badop() == 0


@model_only("the wildcard and the ignored commands; the RTL runs the plain ones")
def test_sensor_wildcard_and_ignored_commands(backend):
    """?! is answered with the sensor's own address (4.4.3); ?I!, 3V! (a
    command it does not have) and 3D1! get no response, and the sensor
    stays awake: the next 3! without a break is answered."""
    bench, loom, line, rec = sensor_setup(backend, address="3")
    assert text(transact(bench, rec, "?!").response) == "3\r\n"
    for cmd in ("?I!", "3V!", "3D1!", "3!"):
        c = transact(bench, rec, cmd, break_ms=None)
        assert text(c.response) == ("3\r\n" if cmd == "3!" else ""), cmd


@model_only("two breaks, 160,000 clocks; every RTL sensor scenario starts with a 12.5 ms one")
@pytest.mark.parametrize("break_ms, seen", [(6.0, False), (12.0, True)])
def test_sensor_break_threshold(backend, break_ms, seen):
    """A sensor never takes spacing under 6.5 ms for a break and always
    takes spacing over 12 ms for one (7.0): a 6 ms break leaves it asleep,
    one of 12 ms wakes it (its threshold is 8.33 ms, its header)."""
    bench, loom, line, rec = sensor_setup(backend)
    c = transact(bench, rec, "0!", break_ms=break_ms)
    assert text(c.response) == ("0\r\n" if seen else ""), break_ms


@model_only("three short commands; the threshold case runs the break on the RTL")
def test_sensor_needs_marking_after_the_break(backend):
    """After a break the sensor needs 8.33 ms of marking before it looks
    for an address (7.0): a command 5 ms after the break is not answered,
    one 8.5 ms after it is."""
    bench, loom, line, rec = sensor_setup(backend)
    assert text(transact(bench, rec, "0!", mark_ms=5.0).response) == ""
    assert text(transact(bench, rec, "0!", mark_ms=8.5).response) == "0\r\n"


@model_only("standby rules: four commands, 300 ms of line time")
def test_sensor_standby_rules(backend):
    """A sensor sleeps after a wrong address and after 100 ms of marking
    (7.0): 1! is not answered and neither is the 0! after it without a
    break; a break wakes it; 0! 50 ms after the last response is answered
    without a break, 0! 110 ms after one is not."""
    bench, loom, line, rec = sensor_setup(backend)
    assert text(transact(bench, rec, "1!").response) == ""
    assert text(transact(bench, rec, "0!", break_ms=None).response) == ""
    assert text(transact(bench, rec, "0!").response) == "0\r\n"
    for idle_ms, answer in ((50, "0\r\n"), (110, "")):
        bench.step(int(idle_ms * MS * SLOW))
        c = transact(bench, rec, "0!", break_ms=None)
        assert text(c.response) == answer, idle_ms


@model_only("about 105,000 clocks; a parity error is caught on the RTL by the recorder, whose frame code is the same")
def test_sensor_ignores_a_parity_error(backend):
    """A command with a parity error in its command letter is ignored, no
    response; the sensor stays awake, and the recorder's retry without a
    break (7.2) is answered."""
    bench, loom, line, rec = sensor_setup(backend)
    assert text(transact(bench, rec, "0M!", parity_error_at=1).response) == ""
    c = transact(bench, rec, "0M!", break_ms=None)
    assert text(c.response) == "0" + MEAS + "\r\n"
    assert_on_time(check_transaction(line, "recorder", "chip0", c.start), False)


# ---------------------------------------------------------------- together
def together_image():
    """The recorder on thread 0 (OUT0, OUT1, IN0) and the sensor on thread
    1 (OUT2, OUT3, IN1): the sensor's file with only its ``.thread`` and
    ``.pins`` lines changed, both assembled for the 512-word memory, as
    one image."""
    words = {}
    for path, edits in ((RECORDER, ()),
                        (SENSOR, ((".thread 0", ".thread 1"),
                                  (".pins   TX = OUT0, DIR = OUT1, RX = IN0",
                                   ".pins   TX = OUT2, DIR = OUT3, RX = IN1")))):
        src = path.read_text(encoding="utf-8")
        for old, new in edits:
            assert src.count(old) == 1, old
            src = src.replace(old, new)
        prog = assemble_text(src, path.name, isa=ISA, imem_words=IMEM, strict=True)
        assert prog.diagnostics == []
        overlap = set(words) & set(prog.words)
        assert not overlap, "the programs overlap at %s" % sorted(overlap)[:4]
        words.update(prog.words)
    return words


def together(backend, clock=SLOW):
    bench = sdi_bench(backend)
    line = bench.add(Sdi12Line([], clock=clock, ports=(("OUT0", "OUT1"), ("OUT2", "OUT3")),
                               rx=("IN0", "IN1")))
    loom = backend.loom(bench, isa=ISA)
    loom.load(together_image())
    ti, tf = TICKS[clock]
    for t in (0, 1):
        loom.write_csr(t, "TICK_INT", ti)
        loom.write_csr(t, "TICK_FRAC", tf)
    loom.run([0, 1])
    loom.push(1, sensor_words("4"))
    bench.step(bits(clock, 4))
    return bench, loom, line


def test_recorder_and_sensor_together(backend):
    """The recorder on thread 0 and the sensor on thread 1, each with its
    own TX, DIR and RX pins, the bench wiring both drivers onto one line
    seen by both RX pins: 4! after a break, then 4D0! without one, both
    reports valid and both transactions within section 7 on the line,
    each side measured against its own rules."""
    bench, loom, line = together(backend)
    for cmd, body, brk in (("4!", "", True), ("4D0!", DATA0, False)):
        since = bench.cycle
        assert_valid(ask(bench, loom, cmd), "4" + body + "\r\n")
        assert_on_time(check_transaction(line, "chip0", "chip1", since), brk)
    assert line.contentions == [] and loom.badop() == 0


@model_only("2 MHz: 1,667 clocks a bit, about 200,000 clocks")
def test_together_on_a_fractional_tick(backend):
    """The same pair on a 2 MHz clock, where 1200 baud needs a fractional
    tick, TICK_INT 208 and TICK_FRAC 85 (208.33 clocks), the fraction of
    the headers' 50 MHz setting (5208 + 85/256): 4! after a break, every
    figure of section 7 met on the line. (At 50 MHz itself the run is 5
    million clocks, over three minutes on the model.)"""
    clock = 2_000_000
    bench, loom, line = together(backend, clock)
    assert_valid(ask(bench, loom, "4!"), "4\r\n")
    timing = check_transaction(line, "chip0", "chip1")
    assert_on_time(timing, True)
    assert 12.0 <= timing.break_ms <= 12.6 and 9.9 <= timing.start_ms <= 10.1, timing
