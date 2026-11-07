"""L3 for the International Morse code programs, on the model and the RTL.

``firmware/morse_tx.loom`` (thread 0, KEY = OUT0, TONE = OUT1) keys ASCII
from the host; ``firmware/morse_rx.loom`` (RX = IN0, low while the key is
down) decodes a keyed line, learning the sender's speed, and sends ASCII
to the host. The references are ``tools.protomodels.morse``, written from
ITU-R M.1677-1: its encoder (with speed and jitter) drives the receiver
through ``IrSource`` (low for a mark), and its decoder and timing checkers
hold the transmitter to the code and to the ratios of section 2.

Expected values come from M.1677-1 (a dash three dots, spaces of one,
three and seven dots), from PARIS (a dot is 1.2 s / WPM) and from the
program headers' stated tick and dot rules, never from what the model or
the RTL did. The transmitter's key edges are latched (SEMANTICS 6.10), so
every mark and space must be its nominal number of dots to the clock; the
receiver must give back the text.

Scale. Morse is slow: at 20 WPM a dot is 60 ms, 3 M clocks at 50 MHz, and
the word PARIS 150 M clocks, beyond either backend. Both programs take
their time base from the host, so the scenarios run a fast "board": the
transmitter at TICK_INT 32 (a sidetone cycle of 64 clocks) with a dot of
D = 4 cycles, 256 clocks; the receiver at TICK_INT 16 (a dot of 16 of its
ticks). The program logic is the same at any speed; only the counts
change. The 50 MHz figures (a 700 Hz sidetone, the WPM dot) are checked
by arithmetic, without a simulation (``test_morse_tx_tick_arithmetic_at_50mhz``).
RTL budget, stated: the nine scenarios that run on the RTL total 743 k
clocks (counted on the model; loading the program and the host's FIFO
polling are most of each), inside the 770 k the batch allows; the
whole-table, host-word and large-step bodies are ``model_only``.

Every body here is named ``test_morse_*``: test/test_fw.py collects the
bodies of all modules into one namespace.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import morse as M
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512

TX_TICK = (32, 0)                   # clocks a tick: half a sidetone cycle
TX_D = 4                            # sidetone cycles a dot
DOT = 2 * TX_TICK[0] * TX_D         # 256 clocks
RX_TICK = (16, 0)
RX_DOT0 = DOT // RX_TICK[0]         # the receiver's starting estimate, ticks


def program(name, thread=0):
    """A program from ``firmware/``, moved to ``thread`` if asked."""
    path = FIRMWARE / (name + ".loom")
    if thread == 0:
        return assemble_file(path, isa=ISA, strict=True)
    text = path.read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, name + ".loom", isa=ISA, imem_words=IMEM, strict=True)


def morse_bench(backend):
    """The FIFOs, the deadline latch, the data memory and 512 words; the
    RTL has them all and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "DMEM"), imem_words=IMEM)
    return backend.bench()


def start(backend, bench, loads):
    """Load ``[(program, thread, (TICK_INT, TICK_FRAC))]`` and run the threads."""
    loom = backend.loom(bench, isa=ISA)
    words = {}
    for prog, _, _ in loads:
        words.update(prog.words)
    loom.load(words)
    for _, thread, (tint, tfrac) in loads:
        loom.write_csr(thread, "TICK_INT", tint)
        loom.write_csr(thread, "TICK_FRAC", tfrac)
    loom.run([thread for _, thread, _ in loads])
    return loom


# ------------------------------------------------------------ transmitter
RAW, WPM, SIDE = 0x8000, 0x4000, 0xC000     # host word kinds (program header)


def chars(text):
    return [ord(c) for c in text]


def tx_setup(backend, d=TX_D, tone=False, tick=TX_TICK):
    bench = morse_bench(backend)
    key = bench.add(M.EdgeRecorder("OUT0"))
    tone_rec = bench.add(M.EdgeRecorder("OUT1"))
    loom = start(backend, bench, [(program("morse_tx"), 0, tick)])
    loom.push(0, [RAW | d] + ([SIDE | 1] if tone else []))
    return bench, loom, key, tone_rec


def key_pulses(edges):
    """``[(level, clocks)]`` from the key's edges, a mark first."""
    out = []
    for (c0, v0), (c1, _) in zip(edges, edges[1:]):
        out.append((M.MARK if v0 else M.SPACE, c1 - c0))
    return out


def feed_run(bench, loom, words, done, budget, chunk=2048):
    """Keep INQ[0] topped up from ``words`` while running until ``done``
    holds with every word sent, or ``budget`` clocks pass."""
    words, end = list(words), bench.cycle + budget
    while True:
        if words:
            st = loom.fifo_status(0)
            free = st["depth"] - st["inq"]
            if free > 0:
                loom.push(0, words[:free])
                words = words[free:]
        if not words and done():
            return True
        if bench.cycle >= end:
            return False
        bench.run_until(done if not words else (lambda: False),
                        max(1, min(chunk, end - bench.cycle)), every=64)


def tx_send(bench, loom, rec, words, marks, dot, budget):
    """Send ``words``; run until ``marks`` marks have ended and the key has
    been up 10 dots."""
    done = lambda: (len(rec.edges) >= 2 * marks
                    and bench.cycle - rec.edges[-1][0] > 10 * dot)
    assert feed_run(bench, loom, words, done, budget), (
        "%d of %d key edges" % (len(rec.edges), 2 * marks))


def marks_of(text):
    return sum(1 for lev, _ in M.encode_units(text)[0] if lev == M.MARK)


def check_tx(rec, text, dot):
    """Every mark and space exact (latched edges: 0 clocks of tolerance)
    against M.1677-1 section 2, and the text decoded back."""
    got = key_pulses(rec.edges)
    nominal, _ = M.encode_units(text)
    bad = M.check_timing(got, nominal, dot, 0)
    assert not bad, "pulses off nominal (index, clocks, nominal): %r" % bad[:6]
    assert M.text_of(M.decode_us(got, dot)) == text.upper().strip()
    return got


def test_morse_tx_paris(backend):
    """PARIS twice, no sidetone: every mark and space exactly 1, 3 or 7
    dots (M.1677-1 2.1 .. 2.4) to the clock, PARIS 50 dots with its word
    space, the text decoded by the reference decoder."""
    bench, loom, key, tone = tx_setup(backend)
    tx_send(bench, loom, key, chars("PARIS PARIS"), marks_of("PARIS PARIS"), DOT,
            150 * DOT)
    got = check_tx(key, "PARIS PARIS", DOT)
    first_word = sum(c for _, c in got[:got.index((M.SPACE, 7 * DOT)) + 1])
    assert first_word == M.PARIS_UNITS * DOT
    assert tone.edges == []                     # sidetone off
    assert loom.pop_available(0) == []
    assert loom.badop() == 0


def test_morse_tx_sidetone(backend):
    """SOS with the sidetone on: TONE is a square wave of two ticks (64
    clocks, high one tick) only while the key is down, D cycles in a dot
    and 3 D in a dash, and low whenever the key is up."""
    bench, loom, key, tone = tx_setup(backend, tone=True)
    tx_send(bench, loom, key, chars("SOS"), marks_of("SOS"), DOT, 60 * DOT)
    got = check_tx(key, "SOS", DOT)
    rises = [c for c, v in tone.edges if v]
    falls = [c for c, v in tone.edges if not v]
    assert len(rises) == len(falls)
    marks = [(key.edges[k][0], key.edges[k + 1][0]) for k in range(0, len(key.edges), 2)]
    per_mark = [sum(1 for r in rises if a <= r < b) for a, b in marks]
    assert per_mark == [TX_D * (1 if lev_c == DOT else 3)
                        for lev_c in [c for lev, c in got if lev == M.MARK]]
    assert {f - r for r, f in zip(rises, falls)} == {TX_TICK[0]}
    for a, b in marks:
        inside = [r for r in rises if a <= r < b]
        assert {y - x for x, y in zip(inside, inside[1:])} <= {2 * TX_TICK[0]}
    for (a, b), r in zip(marks, [rises[sum(per_mark[:k])] for k in range(len(marks))]):
        assert 0 < r - a <= 8                   # one slot after the key edge
    assert all(any(a < f <= b + 8 for a, b in marks) for f in falls)
    assert loom.badop() == 0


@model_only("the whole table, 0.17 M clocks, half a minute of RTL; PARIS and "
            "SOS run there")
def test_morse_tx_every_character(backend):
    """Every character of M.1677-1 1.1.1 .. 1.1.3 that has an ASCII code,
    keyed and decoded, in one message: letters, figures, punctuation."""
    text = "".join(M.LETTERS) + " " + "".join(M.FIGURES) + " " + "".join(M.PUNCTUATION)
    bench, loom, key, _ = tx_setup(backend)
    tx_send(bench, loom, key, chars(text), marks_of(text), DOT, 900 * DOT)
    check_tx(key, text, DOT)
    assert loom.pop_available(0) == []


# ------------------------------------------------------------- receiver
def rx_setup(backend, e0=RX_DOT0):
    bench = morse_bench(backend)
    src = bench.add(M.IrSource("IN0", 1e6))         # 1e6 Hz: durations in clocks
    loom = start(backend, bench, [(program("morse_rx"), 0, RX_TICK)])
    loom.push(0, [e0])
    return bench, loom, src


def tokens(words):
    """The receiver's words as the reference decoder's tokens (headers)."""
    out = []
    for w in words:
        if w < 0x100:
            out.append(chr(w))
        elif w >> 8 == 0x80:
            out.append(("pattern", M.code_from_byte(w & 0xFF)))
        elif w == 0x8100:
            out.append(("long",))
        else:
            assert w == 0x8200, hex(w)
            out.append(("error",))
    return out


def expected(pulses_clocks, dot):
    """The reference decoder's reading at the nominal dot, then the word
    space the idle line after the message makes."""
    out = []
    for t in M.decode_us(pulses_clocks, dot):
        out.append(("error",) if isinstance(t, tuple) and t[0] == "error" else t)
    return out + [" "]


def rx_run(bench, loom, src, pulses, tail_dots=12, dot=DOT, thread=0):
    """Send ``pulses`` (clocks) 4 dots from now; run past their end by
    ``tail_dots`` dots; return the receiver's tokens."""
    src.send(pulses, at=bench.cycle + 4 * dot)
    end = src.free_at + tail_dots * dot
    words = []
    while bench.cycle < end:                    # the host drains OUTQ as it goes
        step = min(8 * dot, end - bench.cycle)
        bench.run_until(lambda: False, step, every=256)
        words += loom.pop_available(thread)
    return tokens(words + loom.pop_available(thread))


@pytest.mark.parametrize("dot", [DOT, 7 * DOT // 4])
def test_morse_rx_two_speeds(backend, dot):
    """PARIS PARIS from the reference encoder at two speeds, the receiver
    starting from a 16-tick estimate both times: at 256 clocks it is right,
    at 448 (1.75 times slower) it learns the speed from the first marks."""
    bench, loom, src = rx_setup(backend)
    pulses = M.encode_us("PARIS PARIS", dot)
    assert rx_run(bench, loom, src, pulses, dot=dot) == list("PARIS PARIS ")
    assert loom.badop() == 0


def test_morse_rx_jitter(backend):
    """Every mark and space of the encoder off by up to +-20 % at random
    (a hand key), the receiver still reads the text."""
    bench, loom, src = rx_setup(backend)
    text = "CQ DE LOOM"
    pulses = M.encode_us(text, DOT, jitter=0.20, seed=7)
    assert expected(pulses, DOT) == list(text + " ")   # the reference reads it too
    assert rx_run(bench, loom, src, pulses) == list(text + " ")
    assert loom.badop() == 0


@pytest.mark.parametrize("factor", [(4, 3), (2, 3)], ids=["faster", "slower"])
def test_morse_rx_speed_change(backend, factor):
    """A sender who changes speed between words, 4/3 times faster or 1.5
    times slower: every character after the change is read (M.1677-1 sets
    no speed; the receiver follows the dot it measures)."""
    num, den = factor
    dot2 = DOT * den // num
    bench, loom, src = rx_setup(backend)
    pulses = M.speed_change("PARIS PARIS", DOT, "TEST 73", dot2)
    assert rx_run(bench, loom, src, pulses, dot=max(DOT, dot2)) == list("PARIS PARIS TEST 73 ")
    assert loom.badop() == 0


def test_morse_rx_signs_and_errors(backend):
    """A sign with no character (understood, ...-.), seven dashes, the error
    signal (eight dots) and a letter after them: patterns reported by code,
    seven or more elements as too long, eight dots as the error signal."""
    bench, loom, src = rx_setup(backend)
    units = []
    for code in [M.SIGNS["understood"], "-------", M.ERROR_SIGNAL, "-.-"]:
        for k, e in enumerate(code):
            units += [(M.MARK, 3 if e == "-" else 1), (M.SPACE, 1 if k < len(code) - 1 else 3)]
    pulses = M.units_to_us(units[:-1], DOT)
    got = rx_run(bench, loom, src, pulses)
    assert got == [("pattern", "...-."), ("long",), ("error",), "K", " "]
    assert loom.badop() == 0


# ------------------------------------------------------------- together
def test_morse_together_with_a_speed_change(backend):
    """The keyer on thread 0 and the decoder on thread 1, the board's
    keying transistor between OUT0 and IN0 (``KeyLine``), the decoder
    starting from the right estimate. Mid-message the host speeds the
    keyer up from a dot of 4 sidetone cycles to 3 (4/3 times faster): the
    decoder follows, and gives back the whole text."""
    bench = morse_bench(backend)
    key = bench.add(M.EdgeRecorder("OUT0"))
    bench.add(M.KeyLine("OUT0", "IN0"))
    loom = start(backend, bench, [(program("morse_tx"), 0, TX_TICK),
                                  (program("morse_rx", thread=1), 1, RX_TICK)])
    loom.push(1, [RX_DOT0])
    words = [RAW | TX_D] + chars("PARIS ") + [RAW | 3] + chars("TEST 73")
    marks = marks_of("PARIS TEST 73")
    got = []
    done = lambda: (len(key.edges) >= 2 * marks
                    and bench.cycle - key.edges[-1][0] > 12 * DOT)
    end = bench.cycle + 200 * DOT
    while not done() and bench.cycle < end:      # the host keeps INQ[0] fed
        st = loom.fifo_status(0)                   # and drains OUTQ[1]
        free = st["depth"] - st["inq"]
        if words and free > 0:
            loom.push(0, words[:free])
            words = words[free:]
        bench.run_until(lambda: False, 2 * DOT, every=256)
        got += loom.pop_available(1)
    got += loom.pop_available(1)
    assert done()
    assert tokens(got) == list("PARIS TEST 73 ")
    assert loom.pop_available(0) == []
    assert loom.badop() == 0


# ------------------------------------------------------------- host words
@model_only("63 k clocks, most of them loading; keeps the RTL set inside its budget, "
            "and PARIS, SOS and the together scenario key the same paths there")
def test_morse_tx_host_words(backend):
    """Lower case keyed as upper case; a character with no code skipped and
    reported (0xC0cc), never shortening the space it sits in; a speed word
    of 0 WPM and a dot of 1 cycle refused (0xC1nn); a WPM word giving
    round(840 / w) cycles a dot (1.2 s / w at 700 Hz, PARIS)."""
    bench, loom, key, _ = tx_setup(backend)
    words = chars("a#b") + [WPM | 0, RAW | 1, WPM | 168] + chars("E")
    tx_send(bench, loom, key, words, marks_of("ABE"), 2 * DOT, 80 * DOT)
    assert loom.pop_available(0) == [0xC023, 0xC100, 0xC101]
    got = key_pulses(key.edges)
    assert M.decode_us(got[:11], DOT) == ["A", "B"]
    assert got[3][0] == M.SPACE and got[3][1] >= 3 * DOT    # the space holding the #
    assert got[3][1] <= 3 * DOT + 2 * 8 * TX_TICK[0]
    d168 = round(840 / 168)                                            # 5 cycles
    assert got[-1] == (M.MARK, d168 * 2 * TX_TICK[0])                  # E at 168 WPM
    assert got[-2][1] >= 3 * DOT
    assert loom.badop() == 0


def test_morse_tx_tick_arithmetic_at_50mhz():
    """The header's 50 MHz figures, from PARIS and the tick rule alone (no
    simulation: at 50 MHz a dot is millions of clocks): TICK_INT 35714 +
    73/256 gives a 700 Hz sidetone within 1 ppm, and round(840 / w) cycles
    give a dot within 4 % of 1.2 s / w for every w from 5 to 60 WPM."""
    tick = 35714 + 73 / 256.0
    assert abs(50e6 / (2 * tick) / 700.0 - 1) < 1e-6
    for w in range(5, 61):
        dot_s = round(840 / w) * 2 * tick / 50e6
        assert abs(dot_s / (M.dot_us(w) / 1e6) - 1) < 0.04, w
    assert round(840 / 20) * 2 * tick / 50e6 == pytest.approx(0.060, rel=1e-6)


# ------------------------------------------------------------- build, model
def test_morse_programs_assemble_strict_with_no_diagnostic():
    """Both programs assemble with --strict and no diagnostic, fit their
    quarters, and their tables sit apart in thread 3's quarter, so they
    load together (the transmitter on thread 0, the receiver on 1)."""
    tx, rx = program("morse_tx"), program("morse_rx", thread=1)
    for prog in (tx, rx):
        assert not prog.diagnostics, [str(d) for d in prog.diagnostics]
    tx_code = [a for a in tx.words if a < 384]
    rx_code = [a for a in rx.words if a < 384]
    assert max(tx_code) < 128 and 128 <= min(rx_code) and max(rx_code) < 256
    assert sorted(a for a in tx.words if a >= 384) == list(range(448, 496))
    assert sorted(a for a in rx.words if a >= 384) == list(range(384, 448))


def test_morse_model_table_is_m1677():
    """The model's table against M.1677-1 1.1 (counts and a few codes read
    from the recommendation), and the marker form round trip."""
    assert len(M.LETTERS) == 26 and len(M.FIGURES) == 10 and len(M.PUNCTUATION) == 13
    assert M.TABLE["?"] == "..--.." and M.TABLE["@"] == ".--.-." and M.TABLE["0"] == "-----"
    assert M.SIGNS["starting signal"] == "-.-.-" and M.ERROR_SIGNAL == "." * 8
    assert len(set(M.TABLE.values())) == len(M.TABLE)          # no two alike
    for code in M.TABLE.values():
        assert M.code_from_byte(M.code_byte(code)) == code
        assert M.code_byte(code) < 128                          # six elements at most
    assert M.code_byte(".-") == 6


def test_morse_model_encoder_and_decoder():
    """PARIS is 50 units with its word space; spaces of 1, 3 and 7; jittered
    pulses decode; two spaces make 14 units."""
    p, skipped = M.encode_units("PARIS")
    assert sum(u for _, u in p) + M.WORD_GAP == M.PARIS_UNITS and skipped == []
    assert {u for lev, u in p if lev == M.SPACE} == {1, 3}
    p, skipped = M.encode_units("E  E#")
    assert p == [(M.MARK, 1), (M.SPACE, 14), (M.MARK, 1)] and skipped == ["#"]
    assert M.text_of(M.decode_us(M.encode_us("HELLO WORLD", 1000.0, jitter=0.2), 1000.0)) == "HELLO WORLD"
    assert M.dot_us(20) == pytest.approx(60_000.0)


def test_morse_model_timing_checkers():
    """check_timing finds a pulse off nominal; check_ratios finds the dot
    of a list of unknown speed and flags a dash of 2.4 dots."""
    nominal, _ = M.encode_units("SOS")
    good = M.units_to_us(nominal, 100.0)
    assert M.check_timing(good, nominal, 100.0, 0) == []
    bad = list(good)
    bad[6] = (M.MARK, 240.0)
    assert [k for k, _, _ in M.check_timing(bad, nominal, 100.0, 1.0)] == [6]
    dot, flagged = M.check_ratios(M.encode_us("SOS SOS", 80.0, jitter=0.05), 0.1)
    assert dot == pytest.approx(80.0, rel=0.05) and flagged == []
    assert [k for k, _, _ in M.check_ratios(bad, 0.1)[1]] == [6]


@model_only("two runs of four words, 0.2 M clocks; the in-range steps run on the RTL")
@pytest.mark.parametrize("dot2", [DOT // 2, DOT * 5 // 2], ids=["2x_faster", "2.5x_slower"])
def test_morse_rx_large_speed_step(backend, dot2):
    """A step beyond the stated range (header: up to 1.5 times faster or 2
    times slower is read from its first character): the receiver may
    misread the first word after it, and has caught up by the second."""
    bench, loom, src = rx_setup(backend)
    pulses = M.speed_change("PARIS", DOT, "PARIS PARIS PARIS", dot2)
    got = "".join(t for t in rx_run(bench, loom, src, pulses, dot=max(DOT, dot2))
                  if isinstance(t, str))
    assert got.startswith("PARIS") and got.endswith(" PARIS PARIS ")
    assert loom.badop() == 0
