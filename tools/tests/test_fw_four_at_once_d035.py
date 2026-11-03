"""Four at once, built only from programs written after the hardware was done.

``test_fw_four_at_once.py`` runs UART, SPI, I2C and WS2812 side by side:
protocols Loom was designed for. Here the four threads run four D-035
programs (DECISIONS D-032, D-035), protocols nobody designed Loom for,
each the shipped file in ``firmware/`` in its own quarter of the 512-word
memory and on its own pins, on one 50 MHz board:

    thread 0  dmx_fixture     RX = IN0 (a receiver)    TICK 25
    thread 1  dshot_tx        DOUT = OUT0              TICK 4 + 43/256
    thread 2  ir_nec_tx       LED = OUT1 (moved)       TICK 438 + 153/256
    thread 3  wiegand_reader  D0 = BIDIR0, D1 = BIDIR1 TICK 250

127 + 65 + 71 + 107 = 370 words; the fixture's two window words at
0x0FE..0x0FF sit in thread 1's quarter, above dshot_tx's last (0x0C0).
Each program is moved only by its ``.thread`` line (changed in memory, as
the original demo does) and, for the NEC LED, by the assembler's ``pins=``
(``--pin LED=OUT1``). The ticks are the README rows' 50 MHz values.

The bench models are the ones each program's own module uses, and each
device is checked as that module checks it: ``dmx.DmxTx`` feeds the
fixture, whose window and status words must be the header's; ``DshotRx``
times every DShot600 pulse and frame start; ``ir_nec.CarrierAnalysis``
judges the NEC carrier, duty and envelope and decodes the frame;
``WiegandPanel`` checks SIA AC-01's windows and every pulse to the clock.

Then the claim the original demo makes: every run loads the same image and
makes the same host writes, and only the RUN mask differs, so each
program's pin edges with all four running must fall on exactly the clocks
they fall on when it runs alone (SEMANTICS 2: each thread has its own
slot). The receiver drives no pin of its own, so its timing is taken on
HOST_IRQ (``uo_out[6]``, SEMANTICS 6.8) with ``IRQ_EN`` = OUTQ_NOT_EMPTY[0]
alone: the pad rises on the clock the fixture pushes its first report,
which follows from where it sampled the DMX line.

One compromise, for RTL time only. A NEC frame is 67.5 ms, 3.4 M clocks at
50 MHz: minutes of Icarus per run. So the RTL body runs ir_nec_tx at TICK
16, the tick its own module's RTL bodies use (the 1.824 MHz board of
``test_fw_ir_nec.py``): the same program and the same waveform counted in
clocks, so on this 50 MHz board the NEC frame goes 27.4 times fast and the
NEC model judges it as on a 1.824 MHz clock. The other three run at their
real 50 MHz rates. ``test_four_d035_at_50mhz_ticks``, on the golden model
only, runs NEC at its real tick too and judges it as its module's 50 MHz
body does, over the leader, its space and four bits.

The PHY-GL-SDF candidate is the RTL body itself (450 k clocks over its
five runs).
"""

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import dshot as D
from tools.protomodels import ir_nec as N
from tools.protomodels import wiegand as W
from tools.protomodels.bench import Model, pad_of
from tools.protomodels.dmx import DmxTx
from tools.tests.fw_backend import model_only
# Each device is judged by its own module's checks, imported, not rewritten.
from tools.tests.test_fw_dshot import B600, check_frames, check_starts
from tools.tests.test_fw_ir_nec import B50 as NEC_B50
from tools.tests.test_fw_ir_nec import B1824, CARRIER_TOL, TX_TOL, check_tx, messages
from tools.tests.test_fw_wiegand import B50 as WIEGAND_B50
from tools.tests.test_fw_wiegand import K_FRAME, K_PERIOD, K_WIDTH, check_exact

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512                                   # the macro build: quarters of 128 words
CLOCK_HZ = 50_000_000                        # the board
HOST_IRQ = ("uo", 6)

#: (protocol, program, thread, pin overrides)
LAYOUT = (
    ("dmx", "dmx_fixture", 0, {}),
    ("dshot", "dshot_tx", 1, {}),
    ("nec", "ir_nec_tx", 2, {"LED": "OUT1"}),
    ("wiegand", "wiegand_reader", 3, {}),
)
THREAD = {name: thread for name, _, thread, _ in LAYOUT}
NAMES = [name for name, _, _, _ in LAYOUT]
#: The pads whose edges are each program's timing: its outputs, or, for the
#: receiver, HOST_IRQ.
PINS = {"dmx": (HOST_IRQ,), "dshot": ("OUT0",), "nec": ("OUT1",),
        "wiegand": ("BIDIR0", "BIDIR1")}

# Ticks (TICK_INT, TICK_FRAC) at 50 MHz, from firmware/README.md's rows.
TICKS = {"dmx": (25, 0), "dshot": (4, 43), "nec": (438, 153), "wiegand": (250, 0)}
NEC_FAST = (16, 0)                           # ir_nec_tx's RTL tick (1.824 MHz board)

# The host commands, from each program's header.
WINDOW = [1, 2]                              # dmx_fixture: slots 1 .. 2
S1, S2 = 0x8000, 0x4000                      # its status words
DMX_SLOTS = [0x11, 0x22, 0x33]
DMX_AT = 20_000                              # clocks after RUN: the first break
DSHOT_IV = 40                                # dshot_tx: a frame every 40 bits
DSHOT_WORDS = [0x8000 | DSHOT_IV, D.packet_of(1046, True)]
NEC_ADDR, NEC_CMD = 0x5A, 0xC3
NEC_WORDS = [NEC_CMD & 0xFF, NEC_ADDR]       # ir_nec_tx: one standard frame
PW, IV = 8, 40                               # 40 us pulses, 200 us apart (SIA's shortest)
KEY = 0xA5                                   # an 8-bit frame
WIEGAND_WORDS = [K_WIDTH | PW, K_PERIOD | IV, K_FRAME | 8, KEY]

#: Clocks after RUN each program needs alone; the together run takes the
#: longest. The NEC figure depends on its tick.
SPAN = {"dmx": 60_000, "dshot": 30_000, "wiegand": 100_000}
#: At 50 MHz, NEC is judged as its module's 50 MHz body judges it: the
#: leader, its space and four bits (20.8 ms), not the whole 67.5 ms frame.
NEC_SPAN = {NEC_FAST: 130_000, TICKS["nec"]: 1_080_000}


def program(source, thread, pins):
    """The shipped source with only its ``.thread`` line changed."""
    text = (FIRMWARE / (source + ".loom")).read_text(encoding="utf-8")
    assert text.count(".thread 0") == 1, "%s: '.thread 0' is not in the source once" % source
    text = text.replace(".thread 0", ".thread %d" % thread)
    return assemble_text(text, source + ".loom", isa=ISA, imem_words=IMEM, strict=True,
                         pins=pins)


def image():
    """The four programs, each in its own thread's quarter, as one image."""
    words = {}
    for _, source, thread, pins in LAYOUT:
        prog = program(source, thread, pins)
        assert prog.diagnostics == [], prog.diagnostics
        base, size = thread * IMEM // 4, IMEM // 4
        assert all(base <= a < base + size for a in prog.words), \
            "%s does not fit thread %d's quarter" % (source, thread)
        assert not set(words) & set(prog.words)
        words.update(prog.words)
    assert not {0x0FE, 0x0FF} & set(words), "the fixture's window words are taken"
    return words


IMAGE = image()


class Edges(Model):
    """Every change of the watched pads, as (cycle, pad, level).

    ``RtlBench``'s ``lines.uo`` has all eight ``uo_out`` bits, HOST_IRQ
    among them. The golden model's ``Bench`` puts only ``PIN_OUT[13:8]`` on
    ``lines.uo``, and ``IRQ_EN`` lives in ``ModelTransport``'s shim, not in
    the machine (so ``Machine.host_irq`` stays 0). On the model, then,
    ``irq_level`` is the shim's ``irq()``: the level computed from the
    cycle's state, which the pad takes at the edge that ends the cycle
    (SEMANTICS 6.8). It is read in ``drive``, before the edge, and logged
    one cycle later, which is the level the pad shows in that cycle, as
    ``lines`` gives every other pad."""

    def __init__(self, pads):
        self.pads = [pad_of(p) for p in pads]
        self.irq_level = None                # set on the model only
        self.irq = self.irq_next = 0
        self.prev = {}
        self.log = []

    def drive(self, drive, cycle):
        if self.irq_level is not None:
            self.irq, self.irq_next = self.irq_next, int(self.irq_level())

    def observe(self, lines):
        for pad in self.pads:
            if pad == HOST_IRQ and self.irq_level is not None:
                level = self.irq
            else:
                level = lines.get(pad)
            if pad in self.prev and self.prev[pad] != level:
                self.log.append((lines.cycle, pad, level))
            self.prev[pad] = level

    def of(self, pins, before=None):
        pads = {pad_of(p) for p in pins}
        return [e for e in self.log if e[1] in pads and (before is None or e[0] < before)]


def run(backend, threads, nec_tick, clocks):
    """One run: the same host sequence every time, then RUN for ``threads``
    and ``clocks`` clocks. Returns the bench, the host, the devices, the
    edge log and the cycle RUN's write completed on."""
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "SETPD", "BE", "BEENC", "DMEM"),
                              imem_words=IMEM, pullups=0b11)
    else:
        bench = backend.bench(pullups=0b11)
    devices = {
        "dmx": bench.add(DmxTx("IN0")),
        "dshot": bench.add(D.DshotRx("OUT0", clock_mhz=CLOCK_HZ / 1e6, rate=600000)),
        "nec": bench.add(N.EdgeRecorder("OUT1")),
        "wiegand": bench.add(W.WiegandPanel("BIDIR0", "BIDIR1", clock_hz=CLOCK_HZ)),
    }
    edges = bench.add(Edges([p for pins in PINS.values() for p in pins] + ["IN0"]))
    loom = backend.loom(bench, isa=ISA)
    if backend.name == "model":
        edges.irq_level = loom.transport.irq
    # No read-back: on the RTL it costs as many host-port clocks as the
    # load (about 50 k), five times over, and the four devices' checks
    # would fail on a bad word anyway.
    loom.load(IMAGE, verify=False)
    loom.push(THREAD["dmx"], WINDOW)
    loom.push(THREAD["dshot"], DSHOT_WORDS)
    loom.push(THREAD["nec"], NEC_WORDS)
    loom.push(THREAD["wiegand"], WIEGAND_WORDS)
    ticks = dict(TICKS, nec=nec_tick)
    for name in NAMES:
        loom.write_csr(THREAD[name], "TICK_INT", ticks[name][0])
        loom.write_csr(THREAD[name], "TICK_FRAC", ticks[name][1])
    loom.irq_enable(1 << THREAD["dmx"])          # OUTQ_NOT_EMPTY[0] only
    loom.run([THREAD[name] for name in threads], exclusive=True)
    t0 = bench.cycle
    devices["dmx"].send_packet(0x00, DMX_SLOTS, start=t0 + DMX_AT)
    devices["dmx"].send_packet(0x00, [])         # its break ends the first packet
    bench.step(clocks)
    devices["wiegand"].flush()
    return bench, loom, devices, edges, t0


def span(name, nec_tick):
    return NEC_SPAN[nec_tick] if name == "nec" else SPAN[name]


def check_device(name, loom, devices, bench, nec_tick):
    """One device, checked as its own module checks it."""
    if name == "dmx":                    # test_fixture_keeps_its_window_...
        got = loom.pop(THREAD["dmx"], 4, max_polls=100)
        assert got == DMX_SLOTS[:2] + [S1 | 0x00, S2 | len(DMX_SLOTS)], [hex(w) for w in got]
        assert loom.pop_available(THREAD["dmx"]) == []
    elif name == "dshot":                # test_tx_repeat_stream_keeps_the_interval
        rx = devices["dshot"]
        frames = rx.frames
        assert len(frames) >= 8, rx.report()
        check_frames(rx, B600)
        check_starts(frames, DSHOT_IV, B600)
        assert frames[0].value == 1046 and frames[0].telemetry
        assert all(f.value == 1046 and not f.telemetry for f in frames[1:])
    elif name == "nec" and nec_tick == NEC_FAST:     # test_tx_frame_and_repeat
        ca = check_tx(devices["nec"], B1824, [N.frame_pulses(NEC_ADDR, NEC_CMD)])
        got = N.decode_pulses(messages(ca)[0], tol=TX_TOL)[0]
        assert (got.kind, got.address, got.command, got.extended) == \
            ("frame", NEC_ADDR | ((~NEC_ADDR & 0xFF) << 8), NEC_CMD, False)
        assert {h for b in ca.bursts for h in b.highs} == {16}
        assert {p for b in ca.bursts for p in b.periods} == {48}
    elif name == "nec":                  # test_tx_carrier_at_50mhz
        edges = 2 * (342 + 5 * 22)       # leader, four bits, bit 4's burst
        rec = devices["nec"]
        assert len(rec.edges) >= edges, len(rec.edges)
        ca = N.CarrierAnalysis(rec.edges[:edges], NEC_B50.clock_hz)
        assert [b.cycles for b in ca.bursts] == [342] + [22] * 5
        for b in ca.bursts:
            assert abs(ca.frequency(b) / N.CARRIER_HZ - 1) <= CARRIER_TOL
            assert abs(ca.duty(b) - 1 / 3.0) <= 0.01
        nominal = N.frame_pulses(NEC_ADDR, NEC_CMD)[:len(ca.pulses())]
        for (lv, us), (nlv, nus) in zip(ca.pulses(), nominal):
            assert lv == nlv and abs(us / nus - 1) <= TX_TOL, (us, nus)
        assert {h for b in ca.bursts for h in b.highs} == {438, 439}
        assert {p for b in ca.bursts for p in b.periods} == {1315, 1316}
    elif name == "wiegand":              # test_reader_sends_long_frames, 8 bits
        mon = devices["wiegand"]
        assert len(mon.frames) == 1 and mon.frames[0]["n"] == 8
        assert mon.frames[0]["value"] == KEY
        check_exact(mon, WIEGAND_B50, PW, IV, 8)
        assert bench.contention_free()


def four_at_once(backend, nec_tick):
    """All four together, each device checked; the four overlap in time;
    then each alone, and its edges compared clock for clock."""
    longest = max(span(name, nec_tick) for name in NAMES)
    bench, loom, devices, together, t0 = run(backend, NAMES, nec_tick, longest)
    for name in NAMES:
        check_device(name, loom, devices, bench, nec_tick)
    assert loom.badop() == 0

    # The receiver's activity is its line; the others', their pins.
    active = dict(PINS, dmx=("IN0", HOST_IRQ))
    spans = {}
    for name in NAMES:
        log = together.of(active[name])
        assert log, "%s: no edges" % name
        spans[name] = (log[0][0] - t0, log[-1][0] - t0)
    assert max(s[0] for s in spans.values()) < min(s[1] for s in spans.values()), \
        "the four do not overlap: %s" % spans

    measured = {}
    for name in NAMES:
        end = t0 + span(name, nec_tick)
        solo_bench, solo_loom, solo_devices, alone, solo_t0 = run(
            backend, [name], nec_tick, span(name, nec_tick))
        assert solo_t0 == t0, "RUN landed on %d alone, %d together" % (solo_t0, t0)
        check_device(name, solo_loom, solo_devices, solo_bench, nec_tick)
        a, b = alone.of(PINS[name], end), together.of(PINS[name], end)
        assert a, "%s: no edges alone" % name
        assert b == a, "%s: %d edges alone, %d with the others; first difference at %s" % (
            name, len(a), len(b),
            next((x, y) for x, y in zip(a + [None], b + [None]) if x != y))
        measured[name] = len(a)
    return spans, measured


def test_four_d035_programs_at_once_land_on_the_same_clocks_as_alone(backend):
    """The RTL scenario (NEC at its 16-clock RTL tick, the other three at
    their real 50 MHz rates): every device gets what its own module
    expects with all four threads running, the four overlap, and each
    program's every pin edge (the fixture's: HOST_IRQ) is on the same clock
    as when its thread runs alone."""
    four_at_once(backend, NEC_FAST)


@model_only("NEC at its real 50 MHz tick: 1.1 M clocks for each of the two "
            "runs that need it, about six minutes of RTL")
def test_four_d035_at_50mhz_ticks(backend):
    """The same, with ir_nec_tx at its real 50 MHz tick (438 + 153/256 clocks,
    a 37,999.9 Hz carrier), so every tick on the board is a README row's.
    NEC is judged as ``test_tx_carrier_at_50mhz`` judges it, over the
    leader, its space and four bits: every carrier high 438 or 439 clocks,
    every period 1315 or 1316."""
    four_at_once(backend, TICKS["nec"])
