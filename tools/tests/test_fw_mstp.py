"""L3-MSTP on the model and the RTL: firmware/mstp_master.loom, a BACnet
MS/TP master node that threads 0 and 2 run as two masters.

The bench wires each running master's TX and DE (master 0: OUT0, OUT1;
master 1: OUT2, OUT3) and every model station onto one ``MstpBus``, the
RS-485 line, whose level goes back to the masters' RX pins (IN0, IN2).
Every octet on the line is decoded by the bus, frames by the Receive Frame
state machine of ``tools.protomodels.mstp``, and the tests check

* the frames' sequence (type, source, destination), derived by hand from
  the Master Node state machine of the BACnet stack (mstp.c, cited in the
  model) for the settings each test gives;
* the line rules on every frame a Loom master sends: bit edges on the bit
  grid, stop bits, Tframe_gap, the driver on before the first start bit and
  off within Tpostdrive of the last stop bit, Tturnaround after the frame
  before (``mstp.frame_violations``);
* the token rules: only the token holder sends a Token, a Poll For Master
  or a data frame, and a Reply To Poll For Master answers a poll
  (``mstp.token_violations``); and no two drivers at once.

Expected values come from the model (CRCs, frame format, the state
machine) and the program header (settings, report words), never from what
the model or the RTL did. Rates: the RTL runs 192 clocks a bit (a tick of
24 clocks, the shortest the program is proved for); the model also runs
115200 baud on a fractional tick. The millisecond timers are scaled to bit
times (Tusage_timeout 60, Tno_token 150, Tslot 64), the bit-time rules are
the stack's. Every body takes a ``backend`` (``tools/tests/fw_backend.py``).
"""

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import mstp
from tools.protomodels.mstp import (DNER, PFM, RPFM, TOKEN, MstpBus, ModelMaster,
                                    NodeTx, bad_header_crc, build_frame, decode)
from tools.tests.fw_backend import model_only

ISA = load()
SOURCE = REPO / "firmware" / "mstp_master.loom"
IMEM = 512
OVS = 8                             # ticks a bit (program header)
CFG = {0: 0x1A0, 2: 0x1D0}          # each master's data (program header)
PINS = {0: ("IN0", "OUT0", "OUT1"), 2: ("IN2", "OUT2", "OUT3")}
NAME = {0: "loom0", 2: "loom2"}
SENT, STATUS, BADCRC = 0x4000, 0x8000, 0x4000   # report words (program header)

# Scaled timers, in bit times.
T_USAGE, T_NO_TOKEN, T_SLOT, T_ABORT = 60, 150, 64, 60


class Rate:
    def __init__(self, tick_int, tick_frac=0):
        self.tick_int, self.tick_frac = tick_int, tick_frac
        self.tick = tick_int + tick_frac / 256
        self.bit = OVS * self.tick
        # Edges land on the 4-clock slot grid, 0 to 3 clocks late (SEMANTICS
        # 2), and a fractional tick rounds: 3 clocks and 1 % of a bit.
        self.edge_tol = 3 + 0.01 * self.bit


FAST = Rate(24)                     # 192 clocks a bit, 260 kbaud
B115200 = Rate(54, 64)              # 434.0 clocks a bit (434.03 nominal)


def settings(ts, nmax, npoll=50, no_token=T_NO_TOKEN):
    """The seven setting words of the program header, ticks from bits."""
    return [ts, nmax, T_USAGE * OVS, (no_token + T_SLOT * ts) * OVS, 1,
            T_ABORT * OVS, npoll]


_PROG = None


def program():
    global _PROG
    if _PROG is None:
        _PROG = assemble_text(SOURCE.read_text(encoding="utf-8"), SOURCE.name,
                              isa=ISA, imem_words=IMEM, strict=True)
        assert _PROG.diagnostics == []
    return _PROG


class Rig:
    """Loom masters on ``masters`` ({thread: (TS, setting overrides)}),
    model masters added with :meth:`model`, all on one bus."""

    def __init__(self, backend, rate, masters, nmax):
        self.rate, self.nmax = rate, nmax
        if backend.name == "model":
            self.bench = backend.bench(features=("FIFO", "BE", "DMEM"), imem_words=IMEM)
        else:
            self.bench = backend.bench()
        self.bus = self.bench.add(MstpBus(
            rate.bit, {NAME[t]: PINS[t][1:] for t in masters},
            [PINS[t][0] for t in masters]))
        self.station = {}
        words = dict(program().words)
        for t, (ts, extra) in masters.items():
            for i, v in enumerate(settings(ts, nmax, **extra)):
                words[CFG[t] + i] = v
            self.station[NAME[t]] = ts
        self.masters = masters
        self.loom = backend.loom(self.bench, isa=ISA)
        # Not read back: the image is checked by the scenarios themselves,
        # and the read-back costs about 45 k clocks of RTL time a scenario.
        self.loom.load(words, verify=False)
        for t in masters:
            self.loom.write_csr(t, "TICK_INT", rate.tick_int)
            self.loom.write_csr(t, "TICK_FRAC", rate.tick_frac)
        self.reports = {t: [] for t in masters}
        self.models = []

    def model(self, name, ts, **kw):
        b = self.rate.bit
        m = ModelMaster(name, self.bus, ts, self.nmax, b, t_usage=T_USAGE * b,
                        t_no_token=T_NO_TOKEN * b, t_slot=T_SLOT * b, **kw)
        self.station[name] = ts
        self.models.append(m)
        m.alive = False             # until the Loom masters run
        return m

    def run(self):
        """Start the Loom masters; the model stations' silence starts with them."""
        self.loom.run(sorted(self.masters))
        for m in self.models:
            m.alive = True
            m.last = float(self.bench.cycle)

    def frames(self):
        return decode(self.bus.rx.octets, self.rate.bit)

    def until(self, done, max_bits, chunk_bits=20):
        """Step until ``done(frames)``, popping the reports as it goes."""
        spent = 0
        while True:
            for t in self.reports:
                self.reports[t] += self.loom.pop_available(t)
            frames = self.frames()
            if done(frames):
                return frames
            assert spent < max_bits, "only %d frames: %s" % (len(frames), seq(frames))
            self.bench.step(int(chunk_bits * self.rate.bit))
            spent += chunk_bits

    def check_line(self, frames):
        """The line rules on every frame a Loom master sent, and the token
        rules and the driver count over them all."""
        assert all(f.valid for f in frames), [(seq([f]), f.why) for f in frames if not f.valid]
        for i, f in enumerate(frames):
            if not set(f.sender) & set(NAME.values()):
                continue
            prev = frames[i - 1].end if i and frames[i - 1].sender != f.sender else None
            bad = mstp.frame_violations(f, self.bus.de_edges[f.sender[0]], self.rate.bit,
                                        self.rate.edge_tol, prev)
            assert not bad, (i, seq([f]), bad)
        bad = mstp.token_violations(frames, self.station,
                                    lost_silence=T_NO_TOKEN * self.rate.bit,
                                    usage_silence=T_USAGE * self.rate.bit)
        assert not bad, bad
        assert not self.bus.contentions, self.bus.contentions[:3]
        assert self.loom.badop() == 0


def seq(frames):
    """``[(type, SA, DA)]`` of ``frames``."""
    return [(f.ftype, f.sa, f.da) for f in frames]


def count(frames, ftype, sa, da):
    return sum(1 for f in frames if (f.ftype, f.sa, f.da) == (ftype, sa, da))


def add_probe(bus, name, bit):
    """A scripted station: a bare :class:`NodeTx` on the bus."""
    tx = NodeTx(bit)
    bus.models[name] = tx
    bus.de_edges[name] = []
    bus._de[name] = 0
    return tx


def frames_from(frames, sender):
    return [f for f in frames if sender in f.sender]


# ----------------------------------------------------------- two Loom masters
RING_TWO = [(PFM, 0, 1), (RPFM, 1, 0), (TOKEN, 0, 1), (DNER, 1, 0), (PFM, 1, 0),
            (RPFM, 0, 1), (TOKEN, 1, 0), (TOKEN, 0, 1), (TOKEN, 1, 0)]


def test_mstp_two_loom_masters(backend):
    """Masters 0 (thread 0) and 1 (thread 2), Nmax_master 1, master 1 given a
    host frame. Silence: master 0's slot comes first (Tno_token), it polls
    1, which answers; it passes the token to 1, which sends the host frame
    while it holds it, finds no successor known and polls 0, which answers;
    then the token goes round both ways. Master 0 reports the data frame,
    master 1 that it went out."""
    rig = Rig(backend, FAST, {0: (0, {}), 2: (1, {})}, nmax=1)
    data = [0x11, 0x22, 0x33]
    rig.loom.push(2, [0 | (len(data) << 8)] + data)
    rig.run()
    frames = rig.until(lambda fr: len(fr) >= len(RING_TWO), max_bits=2400)
    assert seq(frames[:len(RING_TWO)]) == RING_TWO
    assert frames[3].data == data
    rig.check_line(frames)
    rig.until(lambda fr: len(rig.reports[0]) >= 4 and rig.reports[2], max_bits=200)
    assert rig.reports[0][:4] == data + [STATUS | (DNER << 8) | 1]
    assert rig.reports[2][:1] == [SENT | 0]


# ------------------------------------------------ a Loom master, a model master
RING_MODEL = [(PFM, 0, 1), (RPFM, 1, 0), (TOKEN, 0, 1), (PFM, 1, 0), (RPFM, 0, 1),
              (TOKEN, 1, 0), (DNER, 0, 1), (TOKEN, 0, 1), (TOKEN, 1, 0)]


def loom_and_model(backend, rate):
    """A model master 0 and the Loom master 1 (thread 0), Nmax_master 1.
    The model's slot comes first: it polls 1, the Loom answers, takes the
    token, polls 0 (no successor known), passes the token to it; the model
    sends its data frame with the token and passes it straight back
    (back to back, one bit of idle), and the token goes round."""
    rig = Rig(backend, rate, {0: (1, {})}, nmax=1)
    peer = rig.model("peer", 0)
    data = [0xA5, 0x5A]
    peer.queue.append((1, data))
    rig.run()
    frames = rig.until(lambda fr: len(fr) >= len(RING_MODEL), max_bits=2400)
    assert seq(frames[:len(RING_MODEL)]) == RING_MODEL
    rig.check_line(frames)
    rig.until(lambda fr: len(rig.reports[0]) >= 3, max_bits=200)
    assert rig.reports[0][:3] == data + [STATUS | (DNER << 8) | 0]


def test_mstp_loom_and_model_master(backend):
    loom_and_model(backend, FAST)


@model_only("115200 baud is 434 clocks a bit: the same scenario as the fast one, 2.3 times longer")
def test_mstp_loom_and_model_master_115200(backend):
    loom_and_model(backend, B115200)


# --------------------------------------------------- bad header CRC, host frame
def test_mstp_bad_header_crc_ignored_and_host_frame_waits_for_token(backend):
    """The Loom master 1 holds a host frame from the start and its own
    lost-token timer is long. A Token for it with a bad header CRC is
    ignored: nothing is sent. A good Token: the host frame goes out first,
    Tturnaround after it, then (no successor known) a Poll For Master."""
    rig = Rig(backend, FAST, {0: (1, {"no_token": 3000})}, nmax=1)
    probe = add_probe(rig.bus, "probe", FAST.bit)
    rig.station["probe"] = 0
    rig.loom.push(0, [0 | (1 << 8), 0x77])
    rig.run()
    bit = FAST.bit
    probe.send(bad_header_crc(build_frame(TOKEN, 1, 0)), rig.bench.cycle + 20 * bit)
    rig.bench.step(int(150 * bit))
    frames = rig.frames()
    assert len(frames) == 1 and not frames[0].valid and frames[0].why == "header CRC"
    assert not rig.bus.de_edges["loom0"]
    probe.send(build_frame(TOKEN, 1, 0), rig.bench.cycle + 10 * bit)
    frames = rig.until(lambda fr: len(fr) >= 4, max_bits=600)
    assert seq(frames[1:4]) == [(TOKEN, 0, 1), (DNER, 1, 0), (PFM, 1, 0)]
    assert frames[2].data == [0x77]
    for i in (2, 3):
        prev = frames[i - 1].end if i == 2 else None
        bad = mstp.frame_violations(frames[i], rig.bus.de_edges["loom0"], bit,
                                    FAST.edge_tol, prev)
        assert not bad, (i, bad)
    assert not rig.bus.contentions
    rig.until(lambda fr: rig.reports[0], max_bits=100)
    assert rig.reports[0][:1] == [SENT | 0]


# ------------------------------------------------- the successor goes away
@model_only("about 3000 bit times (580 k clocks) of passes, retries and polls; "
            "the ring and the timing rules it uses run on the RTL in the scenarios above")
def test_mstp_successor_gone_retry_then_poll(backend):
    """Loom master 0 and model master 1 in a ring of Nmax_master 3; then the
    model goes silent. The Loom passes the token, waits Tusage_timeout,
    passes it once more (Nretry_token 1), gives the successor up and polls
    from NS + 1 (2, 3), finds nobody, and as the sole master polls the ring
    again from TS + 1."""
    rig = Rig(backend, FAST, {0: (0, {})}, nmax=3)
    peer = rig.model("peer", 1)
    rig.run()
    ring = [(PFM, 0, 1), (RPFM, 1, 0), (TOKEN, 0, 1), (PFM, 1, 2), (PFM, 1, 3),
            (PFM, 1, 0), (RPFM, 0, 1), (TOKEN, 1, 0), (TOKEN, 0, 1), (TOKEN, 1, 0)]
    frames = rig.until(lambda fr: len(fr) >= len(ring), max_bits=3000, chunk_bits=8)
    peer.alive = False
    assert seq(frames[:len(ring)]) == ring
    after = [(TOKEN, 0, 1), (TOKEN, 0, 1), (PFM, 0, 2), (PFM, 0, 3), (PFM, 0, 1)]
    n = len(ring) + len(after)
    frames = rig.until(lambda fr: len(fr) >= n, max_bits=3000)
    assert seq(frames[len(ring):n]) == after
    a, b = frames[len(ring)], frames[len(ring) + 1]
    assert b.start - a.end >= mstp.T_TURNAROUND * FAST.bit + T_USAGE * FAST.bit
    rig.check_line(frames[:n])


# ------------------------------------------------------ maintenance polls
@model_only("about 2700 bit times (520 k clocks); the poll and pass code it "
            "uses runs on the RTL in the scenarios above")
def test_mstp_npoll_maintenance_poll(backend):
    """Loom master 0 with Npoll 3 and model master 2, Nmax_master 3. Once
    the ring is up, every third token the Loom polls the one address
    between it and its successor (1, which does not answer) before passing
    the token, and the poll cycle starts again after it (mstp.c
    DONE_WITH_TOKEN)."""
    rig = Rig(backend, FAST, {0: (0, {"npoll": 3})}, nmax=3)
    rig.model("peer", 2)
    rig.run()
    want = [(PFM, 0, 1), (PFM, 0, 2), (RPFM, 2, 0), (TOKEN, 0, 2), (PFM, 2, 3),
            (PFM, 2, 0), (RPFM, 0, 2), (TOKEN, 2, 0),
            (TOKEN, 0, 2), (TOKEN, 2, 0), (TOKEN, 0, 2), (TOKEN, 2, 0),
            (PFM, 0, 1), (TOKEN, 0, 2), (TOKEN, 2, 0),
            (TOKEN, 0, 2), (TOKEN, 2, 0), (TOKEN, 0, 2), (TOKEN, 2, 0),
            (PFM, 0, 1), (TOKEN, 0, 2)]
    frames = rig.until(lambda fr: len(fr) >= len(want), max_bits=4000)
    assert seq(frames[:len(want)]) == want
    rig.check_line(frames[:len(want)])


# ------------------------------------------------------- the model itself
def test_mstp_crcs_are_the_named_polynomials():
    """crc.c's two functions are the reflected CRCs of x^8 + x^7 + 1 and
    x^16 + x^12 + x^5 + 1 (preset all ones), the arithmetic the firmware
    runs on the CRC unit."""
    import random
    rnd = random.Random(99)
    for _ in range(200):
        data = [rnd.randrange(256) for _ in range(rnd.randrange(1, 12))]
        reg8, reg16 = 0xFF, 0xFFFF
        for b in data:
            reg8, reg16 = mstp.crc8_step(b, reg8), mstp.crc16_step(b, reg16)
        assert mstp.lfsr_reflected(data, 0x81, 8, 0xFF) == reg8
        assert mstp.lfsr_reflected(data, 0x1021, 16, 0xFFFF) == reg16


def test_mstp_frames_and_residues():
    """A Token from 5 to 0x10 (header CRC octet 0x8C by crc.c's code), and
    the residues a receiver checks: 0x55 over a good header with its CRC,
    0xF0B8 over good data with its CRC; bit-reversed (the firmware's CRC
    unit runs MSB first) they are 0xAA and 0x1D0F."""
    assert build_frame(TOKEN, 0x10, 0x05) == [0x55, 0xFF, 0, 0x10, 0x05, 0, 0, 0x8C]
    f = build_frame(DNER, 3, 4, [1, 0x22, 0x30])
    assert mstp.residue8(f[2:8]) == 0x55
    assert mstp.residue16(f[8:]) == 0xF0B8
    assert int(format(0x55, "08b")[::-1], 2) == 0xAA
    assert int(format(0xF0B8, "016b")[::-1], 2) == 0x1D0F
    assert mstp.residue8(bad_header_crc(f)[2:8]) != 0x55


def test_mstp_model_masters_form_a_ring():
    """Two model masters alone (no chip): the ring forms as the Master Node
    state machine says and the token goes round."""
    from tools.protomodels.bench import Bench
    bit = 100.0
    bench = Bench(image={}, features=("FIFO",))
    bus = bench.add(MstpBus(bit, {}, []))
    kw = dict(t_usage=T_USAGE * bit, t_no_token=T_NO_TOKEN * bit, t_slot=T_SLOT * bit)
    ModelMaster("a", bus, 0, 1, bit, **kw)
    ModelMaster("b", bus, 1, 1, bit, **kw)
    bench.step(int(1400 * bit))
    frames = decode(bus.rx.octets, bit)
    assert seq(frames[:8]) == [(PFM, 0, 1), (RPFM, 1, 0), (TOKEN, 0, 1), (PFM, 1, 0),
                               (RPFM, 0, 1), (TOKEN, 1, 0), (TOKEN, 0, 1), (TOKEN, 1, 0)]
    assert not mstp.token_violations(frames, {"a": 0, "b": 1})
    assert not bus.contentions
