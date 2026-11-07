"""L3-POCSAG: the POCSAG pager transmitter and receiver, on the model and the RTL.

``firmware/pocsag_tx.loom`` (thread 0; data on OUT0, the transmitter key on
OUT1) and ``firmware/pocsag_rx.loom`` (thread 2; a receiver's data slicer
on IN0) are assembled, loaded and driven through ``tools.loomhost.Loom``.
``tools.protomodels.pocsag`` supplies the reference: :class:`~pocsag.Line`
records the transmitter's pins, reads the bits at mid-bit and checks every
edge against the bit grid and the rate against M.539-3 4.3 (+-1e-5);
:func:`~pocsag.decode` and :func:`~pocsag.pages` are a pager written from
M.584-2 Annex 1; :class:`~pocsag.Player` plays a transmission from
:func:`~pocsag.transmission` (with bit errors where asked) into the
receiver; :class:`~pocsag.Wire` joins OUT0 to IN0 for the two together.

Expected numbers come from the standards and ``docs/SEMANTICS.md``: the
transmitter's tick is half a bit and SEMANTICS 6.10 lands every edge on
the exact clock of its tick, so with an integer tick every bit is exactly
two ticks and every edge sits on the bit grid with no error at all; with a
fractional tick within one clock of it.

Clocks. POCSAG is slow (a bit is 97 656 clocks at 50 MHz and 512 bit/s),
so the scenarios run on slow board clocks that keep the rate exact: 512
bit/s on a 131.072 kHz clock (256 clocks a bit, 4 x the 32.768 kHz watch
crystal pagers use) and 1200 bit/s on 307.2 kHz. The RTL scenarios shorten
the preamble (M.584-2 1.1 asks for at least 576 bits) to 32 bits to save
simulated time; the model runs one full-length transmission.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import pocsag as P
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
TX, RX = 0, 2                                   # threads
RIC = 1234567                                   # frame 7: the address is codeword 14
RIC_F2 = 0x0ABCD2                               # frame 2: codeword 4
SHORT = 32                                      # preamble bits on the RTL

_PROGRAMS = {}


def program(name):
    if name not in _PROGRAMS:
        _PROGRAMS[name] = assemble_file(FIRMWARE / (name + ".loom"), isa=ISA,
                                        strict=True)
    return _PROGRAMS[name]


def make_bench(backend):
    """FIFOs, the deadline latch, the bit engine (CRC), data memory, the
    512-word memory; the RTL has them all."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "SETPD", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM)
    return backend.bench()


def tick(clocks):
    """``(TICK_INT, TICK_FRAC)`` for a tick of ``clocks`` (SEMANTICS 4)."""
    p = int(round(clocks * 256))
    return p >> 8, p & 0xFF


def boot(backend, bench, *, tx_bit=None, rx_bit=None, preamble=None, rx_ric=None):
    """Load the programs asked for: the transmitter's tick is half a bit of
    ``tx_bit`` clocks, the receiver's a sixteenth of ``rx_bit``; patch the
    preamble length and the receiver's RIC; start them."""
    loom = backend.loom(bench, isa=ISA)
    threads = []
    if tx_bit:
        prog = program("pocsag_tx")
        words = dict(prog.words)
        if preamble is not None:
            words[prog.symbols["cfg"]] = preamble // 2      # bit pairs
        loom.load(words)
        ti, tf = tick(tx_bit / 2)
        loom.write_csr(TX, "TICK_INT", ti)
        loom.write_csr(TX, "TICK_FRAC", tf)
        threads.append(TX)
    if rx_bit:
        prog = program("pocsag_rx")
        words = dict(prog.words)
        if rx_ric is not None:
            words[prog.symbols["cfg"]] = rx_ric & 0xFFFF
            words[prog.symbols["cfg"] + 1] = rx_ric >> 16
        loom.load(words)
        ti, tf = tick(rx_bit / 16)
        loom.write_csr(RX, "TICK_INT", ti)
        loom.write_csr(RX, "TICK_FRAC", tf)
        threads.append(RX)
    loom.run(threads)
    return loom


def page_bits(ric, text, preamble):
    return P.transmission(ric, P.FUNC_NUMERIC, P.numeric_fields(P.numeric_codes(text)),
                          preamble=preamble)


# ------------------------------------------------------- the transmitter
def tx_page(backend, bit, ric, text, preamble, func=P.FUNC_NUMERIC):
    """Send one page; return the line once the key is down again."""
    bench = make_bench(backend)
    line = bench.add(P.Line("OUT0", "OUT1"))
    loom = boot(backend, bench, tx_bit=bit, preamble=preamble)
    codes = P.numeric_codes(text) if isinstance(text, str) else list(text)
    loom.push(TX, P.tx_host_words(ric, func, codes))
    nbits = len(P.transmission(ric, func, P.numeric_fields(codes), preamble=preamble))
    budget = int((nbits + 8) * bit) + 200_000      # the host load and push first
    assert bench.run_until(lambda: line.key_span(bit) is not None, budget, every=512)
    return bench, line, loom, nbits


def check_tx(line, bit, ric, text, preamble, nbits, tol=0.0):
    """Bit-exact against the model encoder, decoded by the model pager,
    every edge on the grid, the key up before the first bit and down at
    the end of the last."""
    fields = P.numeric_fields(P.numeric_codes(text))
    ti, tf = tick(bit / 2)
    assert line.check(2 * (ti + tf / 256), tol_clocks=tol, nominal=bit) == []
    assert line.bits(bit) == page_bits(ric, text, preamble)
    assert line.key_span(bit) == pytest.approx(nbits, abs=(tol + 1) / bit)
    received = P.decode(line.bits(bit))
    assert all(r.errors == 0 for r in received)
    assert P.pages(received, ric) == [(P.FUNC_NUMERIC, fields)]


def test_pocsag_tx_to_model_decoder(backend):
    """512 bit/s on a 131.072 kHz clock (256 clocks a bit, tick 128): a
    7-digit page to a pager in frame 2, one batch: preamble, sync, idle
    in frames 0 and 1, the address at codeword 4, two message codewords,
    idle to the end. Bit-exact, every edge exactly on the 256-clock grid,
    the rate exact (well inside M.539-3's 1e-5), the key line one bit
    ahead of the first bit and down at the end of the last."""
    bit, text = 256, "5551234"
    bench, line, loom, nbits = tx_page(backend, bit, RIC_F2, text, SHORT)
    check_tx(line, bit, RIC_F2, text, SHORT, nbits)
    assert line.key_edges[0][0] == line.t0 - bit
    assert loom.badop() == 0


@model_only("a full 576-bit preamble and two batches: 288 k clocks; the RTL run covers the code")
def test_pocsag_tx_full_preamble_across_batches(backend):
    """The standard's 576-bit preamble; a pager in frame 7, so the address
    is codeword 14 and a 12-character message (3 codewords) runs into the
    second batch past its sync codeword (M.584-2 1.2), then idle to the
    end of that batch."""
    bit, text = 256, "0123-456 789"
    bench, line, loom, nbits = tx_page(backend, bit, RIC, text, P.PREAMBLE_BITS)
    assert nbits == 576 + 2 * 544
    check_tx(line, bit, RIC, text, P.PREAMBLE_BITS, nbits)


@model_only("fractional ticks at 1 MHz: 0.6 to 1.1 M clocks; the integer-tick RTL run covers the code")
@pytest.mark.parametrize("rate", [512.0, 1200.0])
def test_pocsag_tx_rate_fractional_tick(backend, rate):
    """1 MHz board: 512 bit/s is 1953.125 clocks a bit (tick 976 + 144/256,
    exact) and 1200 bit/s 833.33 (tick 416 + 171/256, 3e-6 slow): every
    edge within one clock of the ideal grid and the rate over the
    transmission within M.539-3 4.3's +-1e-5."""
    bit = 1e6 / rate
    text = "999"
    bench, line, loom, nbits = tx_page(backend, bit, RIC_F2, text, SHORT)
    check_tx(line, bit, RIC_F2, text, SHORT, nbits, tol=1.0)


@model_only("a second page shape; the RTL run covers the code")
def test_pocsag_tx_tone_only_and_long_message(backend):
    """No characters: the address alone, then idle (a tone-only page). Then
    a 45-character message: the first 40 go out (8 message codewords), the
    rest are popped and dropped."""
    bit = 256
    bench, line, loom, nbits = tx_page(backend, bit, RIC_F2, "", SHORT)
    received = P.decode(line.bits(bit))
    assert P.pages(received, RIC_F2) == [(P.FUNC_NUMERIC, [])]
    text = "0123456789" * 4 + "UUUUU"
    nbits = len(page_bits(RIC_F2, text[:40], SHORT))
    bench2 = make_bench(backend)
    line2 = bench2.add(P.Line("OUT0", "OUT1"))
    loom2 = boot(backend, bench2, tx_bit=bit, preamble=SHORT)
    loom2.push(TX, P.tx_host_words(RIC_F2, 0, P.numeric_codes(text)))
    assert bench2.run_until(lambda: line2.key_span(bit) is not None, int((nbits + 8) * bit) + 200_000, every=512)
    assert line2.bits(bit) == page_bits(RIC_F2, text[:40], SHORT)
    assert loom2.fifo_status(TX)["inq"] == 0


# ------------------------------------------------------- the receiver
RXB = 512                                       # clocks a bit at the receiver: tick 32


def cw_bit(index, k, preamble=SHORT):
    """Index in a transmission of bit ``k`` (1..32) of codeword ``index``
    of the first batch."""
    return preamble + 32 + 32 * index + k - 1


def collect(bench, loom, until, budget, chunk=4096):
    """Step, popping OUTQ[RX] as it fills, until ``until(words)``."""
    words = []
    spent = 0
    while spent < budget and not until(words):
        bench.step(chunk)
        spent += chunk
        words += loom.pop_available(RX)
    return words


def rx_play(backend, bits, *, flips=(), bit=RXB, rate_scale=1.0, nreports=None,
            ric=RIC_F2, pop=True):
    """Play ``bits`` (with ``flips``) into the receiver; return its reports
    and the model pager's for the same bits."""
    bench = make_bench(backend)
    player = bench.add(P.Player("IN0"))
    loom = boot(backend, bench, rx_bit=bit, rx_ric=ric)
    player.play(bits, bit * rate_scale, flips=flips)
    sent = list(bits)
    for k in flips:
        sent[k] ^= 1
    want = P.pager_reports(P.decode(sent), ric)
    n = len(want) if nreports is None else nreports
    words = collect(bench, loom, lambda w: len(w) >= 2 * n,
                    int((len(bits) + 40) * bit * rate_scale))
    return P.parse_reports(words), want, loom


def test_pocsag_rx_from_model_encoder(backend):
    """The model encoder's page (frame 2, 7 digits) into the receiver, with
    one error in a message bit of codeword 5 and one in the parity bit of
    codeword 6: the address and both message codewords reported, the two
    marked corrected, the message bits as sent."""
    bits = page_bits(RIC_F2, "5551234", SHORT)
    flips = (cw_bit(5, 7), cw_bit(6, 32))
    got, want, loom = rx_play(backend, bits[:cw_bit(8, 1)], flips=flips)
    fields = P.numeric_fields(P.numeric_codes("5551234"))
    assert [(r.kind, r.index, r.corrected) for r in want] == \
        [(1, 4, False), (2, 5, True), (2, 6, True)]
    assert got == want
    assert [r.data for r in got[1:]] == fields
    assert got[0].data == ((RIC_F2 >> 3) << 2) | P.FUNC_NUMERIC
    assert loom.badop() == 0


def test_pocsag_tx_and_rx_together(backend):
    """The transmitter (thread 0) wired to the receiver (thread 2) on one
    chip, 512 clocks a bit for both: the receiver reports the address and
    the message the transmitter was given, nothing corrected."""
    bench = make_bench(backend)
    bench.add(P.Wire("OUT0", "IN0"))
    loom = boot(backend, bench, tx_bit=RXB, rx_bit=RXB, preamble=SHORT, rx_ric=RIC_F2)
    loom.push(TX, P.tx_host_words(RIC_F2, P.FUNC_NUMERIC, P.numeric_codes("5551234")))
    want = P.pager_reports(P.decode(page_bits(RIC_F2, "5551234", SHORT)), RIC_F2)
    words = collect(bench, loom, lambda w: len(w) >= 2 * len(want), 200_000 + 330 * RXB)
    assert P.parse_reports(words) == want
    assert [r.data for r in want[1:]] == P.numeric_fields(P.numeric_codes("5551234"))
    assert not any(r.corrected for r in want)
    assert loom.badop() == 0


@model_only("four 480-bit runs at 512 clocks a bit; the RTL scenario covers the search code")
@pytest.mark.parametrize("group", [0, 1, 2, 3])
def test_pocsag_rx_corrects_every_bit_position(backend, group):
    """A 40-character page: 8 message codewords, each with one bit error,
    at positions 8g + 1 .. 8g + 8: over the four runs every one of the 32
    positions (flag, message, check bits, parity) is corrected."""
    text = "0123456789" * 4
    bits = page_bits(RIC_F2, text, SHORT)
    flips = [cw_bit(5 + j, 8 * group + j + 1) for j in range(8)]
    got, want, loom = rx_play(backend, bits[:cw_bit(14, 1)], flips=flips)
    assert len(want) == 9 and all(r.corrected for r in want[1:])
    assert got == want
    assert [r.data for r in got[1:]] == P.numeric_fields(P.numeric_codes(text))


@model_only("one hand-made batch; the RTL scenario covers the paths")
def test_pocsag_rx_two_errors_other_pagers_and_frames(backend):
    """One batch: another pager's page in frame 1 (ignored), ours in frame
    2 with a message whose middle codeword has two bit errors (reported as
    indecipherable, kind 3, the message goes on), an idle that ends it, a
    stray message codeword after it (ignored), and our address in the
    wrong frame (ignored)."""
    other = (RIC_F2 ^ 0x1F8) & ~7 | 1                   # frame 1, other bits
    m = [P.message_codeword(f) for f in P.numeric_fields(P.numeric_codes("12345678901234"))]
    cws = [P.IDLE, P.IDLE, P.address_codeword(other), m[0],
           P.address_codeword(RIC_F2), m[0], m[1], m[2], P.IDLE, m[1],
           P.address_codeword(RIC_F2), P.IDLE, P.IDLE, P.IDLE, P.IDLE, P.IDLE]
    bits = [1 - (k & 1) for k in range(SHORT)] + P.word_bits(P.SYNC)
    for cw in cws:
        bits += P.word_bits(cw)
    flips = (cw_bit(6, 3), cw_bit(6, 20))
    got, want, loom = rx_play(backend, bits + P.word_bits(P.SYNC), flips=flips)
    assert [(r.kind, r.index) for r in want] == [(1, 4), (2, 5), (3, 6), (2, 7)]
    assert got == want


@model_only("two batches and the standard's 576-bit preamble: 640 k clocks")
def test_pocsag_rx_full_preamble_across_batches(backend):
    """The 576-bit preamble; a pager in frame 7, the address in codeword
    14, the message on past the second sync codeword: all four reported,
    the last two with batch positions 0 and 1."""
    text = "0123-456 789"
    bits = page_bits(RIC, text, P.PREAMBLE_BITS)
    got, want, loom = rx_play(backend, bits, ric=RIC)
    assert [(r.kind, r.index) for r in want] == [(1, 14), (2, 15), (2, 0), (2, 1)]
    assert got == want


@model_only("bit-rate offsets far outside the standard: robustness of the clock recovery")
@pytest.mark.parametrize("offset", [-0.005, 0.005])
def test_pocsag_rx_rate_offset(backend, offset):
    """The transmitter 0.5 % slow or fast (500 times M.539-3's 1e-5): the
    edge-by-edge clock recovery still reads every codeword."""
    bits = page_bits(RIC_F2, "5551234", SHORT)
    got, want, loom = rx_play(backend, bits[:cw_bit(8, 1)], rate_scale=1 + offset)
    assert got == want and len(got) == 3


@model_only("a host that stops reading; the RTL scenarios cover the push path")
def test_pocsag_rx_overflow_is_reported(backend):
    """A 9-report page and a host that reads nothing until codeword 10 has
    gone by: the reports OUTQ holds (two at the default depth of 4) arrive
    whole, the next ones are dropped, and the first report after the host
    reads again says so with its overflow bit; every report that arrives
    is one of the pager's, whole."""
    text = "0123456789" * 4
    bits = page_bits(RIC_F2, text, SHORT)
    bench = make_bench(backend)
    player = bench.add(P.Player("IN0"))
    loom = boot(backend, bench, rx_bit=RXB, rx_ric=RIC_F2)
    player.play(bits[:cw_bit(14, 1)], RXB)
    want = P.pager_reports(P.decode(bits), RIC_F2)
    start = bench.cycle
    bench.run_until(lambda: bench.cycle - start > cw_bit(10, 1) * RXB, 400_000, every=1024)
    words = collect(bench, loom, lambda w: False, 6 * 32 * RXB)
    got = P.parse_reports(words)
    whole = loom.fifo_status(RX)["depth"] // 2       # reports OUTQ holds
    assert got[:whole] == want[:whole]
    assert got[whole].overflow and not any(r.overflow for r in got[whole + 1:])
    plain = {(r.kind, r.index, r.data) for r in want}
    assert all((r.kind, r.index, r.data) in plain for r in got)
    assert got[-1] == want[-1]
