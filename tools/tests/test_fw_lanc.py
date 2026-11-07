"""L3-LANC on the model and the RTL: firmware/lanc_ctrl.loom and lanc_cam.loom.

The controller runs on thread 0 with LANC = BIDIR0, the camera on thread 1
with LANC = BIDIR1, both open drain; ``tools.protomodels.lanc.LancWire``
joins the two pads and the model nodes into one wired-AND line (the board
wire), with the bench's pull-ups on both pads (PORTING 4). Both programs
are loaded as one image in every scenario and only the threads a scenario
needs are run.

Expected values come from the LANC frame as ``protomodels.lanc`` cites it
(Boehmel's 104 us bit, 8 bytes a frame with the camera's start bits, 1200
to 1400 us start to start, 20 / 16.6 ms fields, "valid after 3...4
telegrams"; levels and bit order from memory, unchecked) and from the
program headers (host words, report formats, the 1302 us byte slot), never
from what the model or the RTL did. Pin timing is measured on the line and
on each chip pin: the camera's start-to-start window, every edge of the
controller's pin within a quarter bit of its byte's grid (``check_pin``),
the chip camera's field period to the clock.

Rate: the programs count ticks of one eighth of a bit, set by the host. At
9600 baud and 50 MHz (TICK 651 + 11/256) one field is about a million
clocks, which runs on the model only (``test_lanc_real_rate_together``).
The other bodies run at a test tick of 16 clocks (a 128-clock bit, a field
of about 20500 clocks) with every time of the models scaled with it
(``lanc.scale_for``); the programs count in ticks, so only the speed
changes.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import lanc as L
from tools.tests.fw_backend import model_only

ISA = load()
CTRL = REPO / "firmware" / "lanc_ctrl.loom"
CAM = REPO / "firmware" / "lanc_cam.loom"
IMEM = 512

FAST = (16, 0)                          # the test tick: a 128-clock bit
REAL = L.tick_for(50.0)                 # 9600 baud at 50 MHz: (651, 11)
NTSC_TICKS = 1281                       # lanc_cam.loom header: 16.68 ms
PAL_TICKS = 1536                        # 20 ms
STATUS = {2: 0x00, 3: 0x00, 4: 0x02, 5: 0x04, 6: 0x12, 7: 0x34}
PLAY = (L.NORMAL, L.PLAY)
STOP = (L.NORMAL, L.STOP)
RUN_AT = 400                            # clocks for the programs' set-up

_IMAGE = None


def image():
    """Both programs, assembled with --strict, as one image."""
    global _IMAGE
    if _IMAGE is None:
        words = {}
        for path in (CTRL, CAM):
            prog = assemble_file(path, isa=ISA, strict=True)
            assert prog.diagnostics == []
            assert not set(words) & set(prog.words)
            words.update(prog.words)
        _IMAGE = words
    return _IMAGE


def us_of(rate):
    return L.scale_for(rate[0] + rate[1] / 256)


def field(rate, ticks=NTSC_TICKS):
    return ticks * (rate[0] + rate[1] / 256)


def word(pair):
    return pair[0] | pair[1] << 8


def setup(backend, rate, nodes=(), ctrl=False, cam=False, repeat=5,
          period=NTSC_TICKS, status=STATUS, threads_first=None):
    """The chip at ``rate`` on a wire with model ``nodes``; thread 0 runs
    the controller (N = ``repeat``), thread 1 the camera (field ``period``
    ticks, ``status`` bytes), as asked. Returns ``(bench, loom, wire)``."""
    wire = L.LancWire(nodes, pins=("BIDIR0", "BIDIR1"))
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "BE", "SETPD", "DMEM"),
                              imem_words=IMEM, pullups=wire.pullups)
    else:
        bench = backend.bench(pullups=wire.pullups)
    bench.add(wire)
    loom = backend.loom(bench, isa=ISA)
    loom.load(image())
    threads = ([0] if ctrl else []) + ([1] if cam else [])
    for t in threads:
        loom.write_csr(t, "TICK_INT", rate[0])
        loom.write_csr(t, "TICK_FRAC", rate[1])
    if ctrl:
        loom.push(0, [repeat])
    words = [(k << 8) | v for k, v in sorted(status.items())]
    if cam:
        loom.push(1, [period] + words[:3])
    if threads_first is not None:
        threads_first(bench, loom, threads)
    else:
        loom.run(threads)
    if cam:
        loom.push(1, words[3:])            # taken in the next gaps
    return bench, loom, wire


def frames_of(wire, bench, rate, fast=1.0):
    return L.decode(wire.edges, us_of(rate), rate=fast, end=bench.cycle)


def runs(received, command):
    """The lengths of the runs of ``command`` in a list of frames."""
    out, n = [], 0
    for c in list(received) + [None]:
        if c == command:
            n += 1
        elif n:
            out.append(n)
            n = 0
    return out


def polls(clocks):
    return int(clocks) // 40 + 2000


def run_to(bench, cycle):
    if cycle > bench.cycle:
        bench.step(int(cycle - bench.cycle))


def status_words(status):
    return [status[4] | status[5] << 8, status[6] | status[7] << 8]


def model_camera(bench, wire, tick, delay=RUN_AT, **kwargs):
    kwargs.setdefault("status", dict(STATUS))
    return wire.add(L.Camera(us_of(tick), field="NTSC", start=bench.cycle + delay, **kwargs))


def check_ctrl_pin(wire, frames, us):
    assert L.check_pin(wire.lows_of("BIDIR0"), frames, us, allowed=L.CONTROLLER_BYTES) == []


# ------------------------------------------------------------ the controller
def ctrl_against_camera(backend, fast):
    """The body of the two controller scenarios: a model camera ``fast``
    times fast (NTSC fields); the chip controller writes PLAY for N = 5
    frames, reports the status bytes 4..7 of the first frame and again
    after byte 5 changes, and nothing while they stay the same."""
    us = us_of(FAST)
    bench, loom, wire = setup(backend, FAST, ctrl=True)
    cam = model_camera(bench, wire, FAST, rate=fast)
    loom.push(0, [word(PLAY)])
    assert loom.pop(0, 2, max_polls=polls(3 * cam.field)) == status_words(STATUS)
    cam.status[5] = 0x00                         # byte 5: battery no longer low
    changed = {**STATUS, 5: 0x00}
    assert loom.pop(0, 2, max_polls=polls(3 * cam.field)) == status_words(changed)
    run_to(bench, cam.start + 7.2 * cam.field)
    assert loom.pop_available(0) == []           # no change, no report
    assert runs(cam.received, PLAY) == [5], cam.received
    assert set(cam.received) <= {(0, 0), PLAY}
    frames = frames_of(wire, bench, FAST, fast)
    assert L.check_frames(frames, us, rate=fast) == []
    check_ctrl_pin(wire, frames, us)
    whole = [f for f in frames if f.complete()]
    assert cam.received[:len(whole)] == [f.command() for f in whole]
    assert loom.badop() == 0
    return cam, frames


def test_lanc_ctrl_to_model_camera(backend):
    """The chip controller against a model camera at the nominal rate:
    PLAY (0x18 0x34) in exactly five frames in a row and nowhere else, two
    status reports (first frame, then byte 5's change), every controller
    edge within a quarter bit of its byte's grid, start to start inside
    Boehmel's 1200..1400 us."""
    ctrl_against_camera(backend, 1.0)


def test_lanc_camera_two_percent_fast(backend):
    """The same against a camera 2 % fast (every time of it 1/1.02 as
    long): the controller restarts its tick on every start bit, so the
    camera still reads PLAY in five frames, and the controller's edges stay
    inside the window measured against its own nominal bit."""
    ctrl_against_camera(backend, 1.02)


# ------------------------------------------------------------ the camera
def test_lanc_model_controller_to_cam(backend):
    """A model controller sends START/STOP (0x18 0x33) for five frames to
    the chip camera (NTSC, 1281 ticks): the camera reports the command with
    a count of 5; the controller reads the host's status bytes 4..7 in
    every frame once they are set; every start bit is 100 ticks after the
    last, every field exactly 1281 ticks, every edge of the camera's pin on
    its byte's grid."""
    us = us_of(FAST)
    ctl = L.Controller(us, repeat=5)
    ctl.send(L.NORMAL, L.START_STOP)
    bench, loom, wire = setup(backend, FAST, [ctl], cam=True)
    period = field(FAST)
    assert loom.pop(1, 2, max_polls=polls(9 * period)) == [word((L.NORMAL, L.START_STOP)), 5]
    assert ctl.written == [(L.NORMAL, L.START_STOP)] * 5
    assert ctl.statuses[-1] == tuple(STATUS[k] for k in L.STATUS_BYTES)
    frames = frames_of(wire, bench, FAST)
    assert L.check_frames(frames, us) == []
    for f in frames:
        assert all(b - a == 100 * FAST[0] for a, b in zip(f.starts, f.starts[1:]))
    firsts = [f.starts[0] for f in frames]
    assert all(b - a == period for a, b in zip(firsts, firsts[1:]))
    assert L.check_pin(wire.lows_of("BIDIR1"), frames, us, allowed=range(8), first=0) == []
    assert [f.data[2:] for f in frames if f.complete()][-1] == [STATUS[k] for k in range(2, 8)]
    assert loom.badop() == 0


# ------------------------------------------------------------ together
def check_both_pins(wire, frames, us):
    check_ctrl_pin(wire, frames, us)
    assert L.check_pin(wire.lows_of("BIDIR1"), frames, us, allowed=range(8), first=0) == []


def last_report(loom):
    words = loom.pop_available(0)
    assert words and len(words) % 2 == 0, words
    return words[-2:]


def test_lanc_ctrl_and_cam_together(backend):
    """Thread 0's controller writes PLAY for five frames into the fields of
    thread 1's camera across the bench's wire: the camera reports PLAY with
    a count of 5, the controller's last report is the host's status bytes
    4..7, and both pins keep their byte grids."""
    us = us_of(FAST)
    bench, loom, wire = setup(backend, FAST, ctrl=True, cam=True)
    loom.push(0, [word(PLAY)])
    assert loom.pop(1, 2, max_polls=polls(9 * field(FAST))) == [word(PLAY), 5]
    assert last_report(loom) == status_words(STATUS)
    frames = frames_of(wire, bench, FAST)
    assert L.check_frames(frames, us) == []
    assert runs([f.command() for f in frames], PLAY) == [5]
    check_both_pins(wire, frames, us)
    assert loom.badop() == 0


@model_only("ten fields more than the RTL scenarios, through the same paths")
def test_lanc_command_repeated_five_frames(backend):
    """PLAY then STOP queued at once: each is written in exactly five
    fields (N = 5, more than Boehmel's "valid after 3...4 telegrams"), the
    second straight after the first, and the camera reports each with its
    count of 5."""
    us = us_of(FAST)
    bench, loom, wire = setup(backend, FAST, ctrl=True, cam=True)
    loom.push(0, [word(PLAY), word(STOP)])
    assert loom.pop(1, 4, max_polls=polls(14 * field(FAST))) == [word(PLAY), 5, word(STOP), 5]
    frames = frames_of(wire, bench, FAST)
    cmds = [f.command() for f in frames]
    assert runs(cmds, PLAY) == [5] and runs(cmds, STOP) == [5]
    first = cmds.index(PLAY)
    assert cmds[first:first + 10] == [PLAY] * 5 + [STOP] * 5
    assert 5 >= max(L.VALID_AFTER)
    check_both_pins(wire, frames, us)


def _phase(p, order):
    def start(bench, loom, threads):
        bench.step(p)
        if order == "one_run":
            loom.run([0, 1])
        else:
            first, second = (0, 1) if order == "ctrl_first" else (1, 0)
            loom.run(first)
            bench.step(p)
            loom.run(second)
    return start


@model_only("BUGS 14 start phases: six starts of two fields each")
@pytest.mark.parametrize("p, order", [(0, "one_run"), (1, "one_run"), (2, "one_run"),
                                      (3, "one_run"), (1, "ctrl_first"), (2, "cam_first")])
def test_lanc_together_at_every_start_phase(backend, p, order):
    """Both programs set their OD_MASK bit first (BUGS 14). Under one RUN at
    each of the four phases of the slot grid, and started apart in both
    orders, both bits survive and a one-frame command (N = 1) arrives."""
    bench, loom, wire = setup(backend, FAST, ctrl=True, cam=True, repeat=1,
                              threads_first=_phase(p, order))
    loom.push(0, [word(PLAY)])
    assert loom.pop(1, 2, max_polls=polls(5 * field(FAST))) == [word(PLAY), 1]
    assert bench.machine.od_mask & 0b11 == 0b11


@model_only("9600 baud at 50 MHz: half a million clocks")
def test_lanc_real_rate_ctrl_to_model_camera(backend):
    """At the real rate (TICK 651 + 11/256 at 50 MHz, a 5208-clock bit) the
    controller writes PLAY into the first field of a model camera (N = 1)
    and reports its status bytes; its edges are within a quarter bit of
    the camera's grid, measured in microseconds."""
    us = us_of(REAL)
    assert abs(us - 50.0) < 0.01
    bench, loom, wire = setup(backend, REAL, ctrl=True, repeat=1)
    cam = model_camera(bench, wire, REAL, delay=2 * L.GAP_US * us, frames=1)
    loom.push(0, [word(PLAY)])
    assert loom.pop(0, 2, max_polls=polls(cam.field)) == status_words(STATUS)
    assert cam.received == [PLAY]
    frames = frames_of(wire, bench, REAL)
    assert L.check_frames(frames, us) == []
    check_ctrl_pin(wire, frames, us)


# ------------------------------------------------------------ the programs
@pytest.mark.parametrize("path, thread, words", [(CTRL, 0, 97), (CAM, 1, 104)])
def test_lanc_programs_assemble_strict(path, thread, words):
    """L2-DEADLINE: no diagnostic with --strict, every pair proved at the
    test tick (.tick 16), nothing unbounded, no ``.bounded``; each program
    in its thread's quarter, so the two load together (the camera's count
    includes its six-word status table)."""
    prog = assemble_file(path, isa=ISA, strict=True)
    assert prog.diagnostics == []
    report = prog.deadlines[thread]
    assert report.period == 16
    assert report.pairs and not report.infeasible and not report.unbounded
    assert report.worst_slack >= 0
    assert list(prog.threads) == [thread] and len(prog.words) == words
    assert all(thread * 128 <= a < (thread + 1) * 128 for a in prog.words)


# ------------------------------------------------- the reference model alone
def test_lanc_model_controller_and_model_camera():
    """The two models on a bench of their own (no chip program running):
    the controller finds byte 0 from the gap, the camera reads the command
    in five frames, the controller reads the status, and the line keeps
    the windows; tick_for gives 651 + 11/256 at 50 MHz."""
    from tools.protomodels.bench import Bench
    assert L.tick_for(50.0) == (651, 11)
    us = us_of(FAST)
    ctl = L.Controller(us, repeat=5, delay=10)
    cam = L.Camera(us, field="PAL", status=STATUS, start=3000, frames=7)
    wire = L.LancWire([ctl, cam], pins=("BIDIR0",))
    ctl.send(*PLAY)
    bench = Bench(imem_words=IMEM, pullups=wire.pullups)
    bench.add(wire)
    bench.step(int(3000 + 7 * cam.field))
    assert runs(cam.received, PLAY) == [5] and ctl.written == [PLAY] * 5
    assert set(ctl.statuses) == {tuple(STATUS[k] for k in L.STATUS_BYTES)}
    frames = L.decode(wire.edges, us)
    assert len(frames) == 7 and L.check_frames(frames, us) == []
    assert [f.command() for f in frames] == cam.received
