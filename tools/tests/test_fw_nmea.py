"""L3-NMEA on the model and the RTL: firmware/bridge_nmea_lcd.loom, NMEA 0183
RMC sentences from a GPS receiver to an HD44780U LCD, with no host in the
data path (D-035).

Thread 1 receives the GPS's 8N1 stream on IN0 and parses it; thread 0
drives ``tools.protomodels.hd44780.Hd44780`` on its 4-bit bus (D4..D7 =
BIDIR0..3, RS = OUT0, R/W = OUT1, E = OUT2). The sentences come from
``tools.protomodels.nmea`` (written from the SiRF NMEA Reference Manual
and gpsd's "NMEA Revealed"); what the LCD must show comes from
``nmea.parse_rmc`` and the layout in the program header, never from what
the model or the RTL did.

Rates: a GPS sends at 4800 or 9600 baud. Most bodies run a 25 MHz build
(``US = 25``) at 57600 baud (434 clocks a bit) so that a sentence costs
about 230,000 clocks instead of 1.4 million. It is the hardest rate, not
an easy one: the parser's longest path (the first comma, 220 clocks)
overruns the half bit (217 clocks) by less than a slot, which the
receiver absorbs (the program header). One body runs 9600 baud on the
model. The LCD has been powered 50 ms
before the bench starts and the boot takes the supply as already up
(``BOOT = 0x000A``), as in ``test_fw_hd44780.py``.
"""

import functools

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import nmea
from tools.protomodels.hd44780 import FOSC, TIMING, Hd44780
from tools.tests.fw_backend import model_only

ISA = load()
SOURCE = REPO / "firmware" / "bridge_nmea_lcd.loom"
US_LINE = ".equ    US = 50 "
BOOT_LINE = ".equ    BOOT = 0x0008 "
V3 = TIMING["2.7-4.5V"]
FAST = 57_600                   # the tests' rate (see the module docstring)
BLANK = " " * 16


@functools.lru_cache(maxsize=None)
def program(us=25, warm=True):
    """The program with ``--strict``, its LCD tick ``US`` clocks a us."""
    text = SOURCE.read_text(encoding="utf-8")
    assert text.count(US_LINE) == 1 and text.count(BOOT_LINE) == 1
    text = text.replace(US_LINE, ".equ    US = %d " % us)
    if warm:
        text = text.replace(BOOT_LINE, ".equ    BOOT = 0x000A ")
    return assemble_text(text, SOURCE.name, isa=ISA, strict=True)


def shown(line):
    """The two rows the program header promises for a good RMC ``line``."""
    f = nmea.parse_rmc(line)
    assert f is not None, line
    fix = f["status"] == "A"
    row0 = f["time"][:6].ljust(6) + " "
    row0 += (f["lat"][:8].ljust(8) + f["ns"][:1].ljust(1)) if fix else " " * 9
    row1 = f["status"][:1].ljust(1) + " " * 5
    row1 += (f["lon"][:9].ljust(9) + f["ew"][:1].ljust(1)) if fix else " " * 10
    return [row0, row1]


def setup(backend, *, us=25, baud=FAST, fosc=FOSC["max"]):
    """The bridge on threads 0 and 1, the LCD on thread 0's pins, a GPS on
    IN0; thread 1's tick one eighth of a bit. Returns after the boot
    initialization has turned the display on."""
    clock = us * 1_000_000
    lcd = Hd44780(clock=clock, fosc=fosc, powered_at=-(clock // 20))
    kwargs = {}
    if backend.name == "model":
        kwargs.update(features=("FIFO", "DMEM"), imem_words=512)
    bench = backend.bench(**kwargs)
    bench.add(lcd)
    bit = clock / baud
    gps = bench.add(nmea.GpsSource("IN0", bit))
    loom = backend.loom(bench, isa=ISA)
    loom.load(program(us))
    tick = bit / 8
    loom.write_csr(1, "TICK_INT", int(tick))
    loom.write_csr(1, "TICK_FRAC", int(round((tick - int(tick)) * 256)))
    loom.run([0, 1], exclusive=True)
    assert bench.run_until(lambda: lcd.display, 400_000, every=2000)
    assert lcd.render() == [BLANK, BLANK]
    return bench, lcd, gps


def send(bench, gps, lines, idle=40_000):
    """Send ``lines`` back to back and run until the LCD has had ``idle``
    clocks to draw (32 characters at about 60 us each at 25 MHz)."""
    gps.send_sentences(lines)
    assert bench.run_until(lambda: not gps.busy, 10 ** 8, every=1000)
    bench.step(idle)


def clean(bench, lcd):
    """No timing, protocol or busy violation, nothing rejected, no line
    driven both ways, every measured interval inside p.49's minimums."""
    assert lcd.violations == []
    assert lcd.rejected == []
    assert bench.contention_free()
    for name, values in lcd.measured.items():
        assert all(v >= V3[name][0] for v in values), (name, min(values))


def data_bytes(lcd):
    return [b for _, kind, rs, b in lcd.events if kind == "byte" and rs == 1]


# ------------------------------------------------------- the model and image
def test_nmea_model_checksum_matches_the_sirf_example():
    """The reference model against its sources: Table 1-11's example has the
    checksum the manual prints (0x10), and the parser takes it; a changed
    digit, a lost CR LF or a non-RMC sentence is refused by parse_rmc."""
    assert nmea.checksum(nmea.SIRF_EXAMPLE[1:-5]) == 0x10
    assert nmea.sentence(nmea.SIRF_EXAMPLE[1:-5]) == nmea.SIRF_EXAMPLE
    f = nmea.parse_rmc(nmea.SIRF_EXAMPLE)
    assert (f["time"], f["status"], f["lat"], f["ew"]) == (
        "161229.487", "A", "3723.2475", "W")
    assert nmea.parse_rmc(nmea.SIRF_EXAMPLE.replace(b"*10", b"*11")) is None
    assert nmea.parse_rmc(nmea.SIRF_EXAMPLE[:-2]) is None
    assert nmea.parse_rmc(nmea.sentence("GPGGA,123519,4807.038,N")) is None
    assert len(nmea.rmc()) <= nmea.MAX_LEN


def test_nmea_assembles_strict_with_no_diagnostic():
    """Both threads' pairs proved: thread 0 at its 1 us tick (US 50), thread
    1 (the receiver) for ticks of 16 clocks or more. Thread 0 in quarters
    0 and 3, thread 1 in quarters 1 and 2 (the spill and the three
    buffers), every data word loaded."""
    prog = program(50, warm=False)
    assert prog.diagnostics == []
    for thread, period in ((0, 50), (1, 16)):
        report = prog.deadlines[thread]
        assert report.period == period
        assert report.pairs and not report.infeasible and not report.unbounded
        assert report.worst_slack >= 0
    assert set(prog.threads) == {0, 1}
    assert len(prog.words) == 423
    quarter = [sum(1 for a in prog.words if a // 128 == q) for q in range(4)]
    assert quarter == [101, 94, 122, 106]
    assert prog.words[prog.symbols["PUB"]] == prog.symbols["buf0"]


# --------------------------------------------------------------- the bridge
def test_nmea_rmc_shows_and_a_bad_checksum_is_ignored(backend):
    """A good RMC at 57600 baud: the time, the latitude on row 0, the status
    and the longitude on row 1. Then an RMC with other values and a wrong
    checksum: not one byte more reaches the LCD, the display is unchanged."""
    bench, lcd, gps = setup(backend)
    good = nmea.rmc(speed="", course="")
    send(bench, gps, [good])
    assert lcd.render() == shown(good) == ["123519 4807.038N",
                                           "A     01131.000E"]
    count = len(lcd.events)
    bad = nmea.rmc(time="235959", lat="1111.111", csum=0x00)
    assert nmea.parse(bad) is None
    send(bench, gps, [bad], idle=20_000)
    assert len(lcd.events) == count
    assert lcd.render() == shown(good)
    clean(bench, lcd)


@model_only("a second 230,000-clock sentence; the RTL scenario already "
            "draws a fix through the same layout")
def test_nmea_void_fix_shows_the_time_and_no_position(backend):
    """A good fix, then status V ("data not valid", Table 1-11) with a
    position still in its fields: the time and the V are shown, the
    position is blanked. A V sentence with null position fields (what a
    receiver sends before its first fix) shows the same."""
    bench, lcd, gps = setup(backend)
    send(bench, gps, [nmea.rmc()])
    void = nmea.rmc(time="000102", status="V")
    send(bench, gps, [void])
    assert lcd.render() == shown(void) == ["000102" + " " * 10,
                                           "V" + " " * 15]
    empty = nmea.rmc(time="000103", status="V", lat="", ns="", lon="", ew="",
                     date="")
    send(bench, gps, [empty])
    assert lcd.render() == ["000103" + " " * 10, "V" + " " * 15]
    clean(bench, lcd)


@model_only("four sentences, about a million clocks; the checksum path it "
            "ends in is the RTL scenario's")
def test_nmea_sentence_split_by_line_noise_is_dropped(backend):
    """After a good fix: a burst of noise in the middle of a sentence that
    the UART reads as one more character (the checksum then fails); a
    longer burst, a framing error; a sentence cut short and followed by a
    new '$' (it resynchronizes). None of the three changes the display;
    the good sentence after them is shown."""
    bench, lcd, gps = setup(backend)
    first = nmea.rmc()
    send(bench, gps, [first])
    bit = gps.bit_clocks
    for width in (1.2 * bit, 12 * bit):         # a garbage byte, a break
        line = nmea.rmc(time="010203", lat="0102.030")
        gps.send(line[:20])
        gps.noise(width)
        gps.send(line[20:])
    gps.send(nmea.rmc(time="040506")[:30])      # cut short, then a new '$'
    last = nmea.rmc(time="070809", lat="5100.500", ns="S", lon="00012.250",
                    ew="W")
    count = len(data_bytes(lcd))
    gps.send(last[:1])
    assert bench.run_until(lambda: not gps.busy, 10 ** 8, every=1000)
    assert lcd.render() == shown(first)
    assert len(data_bytes(lcd)) == count        # nothing drawn meanwhile
    send(bench, gps, [last[1:]])
    assert lcd.render() == shown(last) == ["070809 5100.500S",
                                           "A     00012.250W"]
    clean(bench, lcd)


@model_only("five sentences back to back, about 1.2 million clocks")
def test_nmea_other_sentences_ignored_back_to_back(backend):
    """A receiver's burst with no gaps: GGA, GSV and an RMB (good checksums,
    not RMC) are ignored; SiRF Table 1-11's example and a u-blox-style GNRMC
    (more decimals, the mode field) are both shown, the last one stays.
    Fields longer than their slots are cut, not rounded."""
    bench, lcd, gps = setup(backend)
    gga = nmea.sentence("GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,"
                        "545.4,M,46.9,M,,")
    gsv = nmea.sentence("GPGSV,1,1,01,07,79,048,42")
    rmb = nmea.sentence("GPRMB,A,0.66,L,003,004,4917.24,N,12309.57,W,"
                        "001.3,052.5,000.5,V")
    send(bench, gps, [gga, gsv, nmea.SIRF_EXAMPLE, rmb])
    assert lcd.render() == shown(nmea.SIRF_EXAMPLE) == ["161229 3723.247N",
                                                        "A     12158.341W"]
    ublox = nmea.rmc(time="083559.00", lat="4717.11437", lon="00833.91522",
                     speed="0.004", date="091202", talker="GN", mode="A")
    send(bench, gps, [ublox, gga])
    assert lcd.render() == shown(ublox) == ["083559 4717.114N",
                                            "A     00833.915E"]
    clean(bench, lcd)


@model_only("9600 baud is 1,250 clocks a bit even in a 12 MHz build: "
            "800,000 clocks a sentence")
def test_nmea_at_9600_baud(backend):
    """The rate of a GPS module (gpsd: 9600 the most common), in a 12 MHz
    build (US = 12, the slowest the LCD thread is proved for) on the
    slowest LCD (190 kHz): one sentence, shown."""
    bench, lcd, gps = setup(backend, us=12, baud=9600, fosc=FOSC["min"])
    line = nmea.rmc(time="225446", lat="4916.45", lon="12311.12", ew="W",
                    speed="000.5", course="054.7", date="191194",
                    magvar="020.3", magvar_ew="E")
    send(bench, gps, [line])
    assert lcd.render() == shown(line) == ["225446 4916.45 N",
                                           "A     12311.12 W"]
    clean(bench, lcd)
