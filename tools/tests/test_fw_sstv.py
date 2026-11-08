"""L3 for the SSTV Martin M1 transmitter, on the model and the RTL.

``firmware/sstv_m1.loom`` (TX = OUT0) sends the VIS header and N lines of
EBU 100/75 colour bars as a phase-continuous square wave. The reference is
``tools.protomodels.sstv``: it fits the pin's edge index against time per
segment, so it reads each segment's frequency and each boundary to a few
microseconds, decodes the VIS and rebuilds the bars.

Expected values come from the mode specifications (the module docstring of
``tools.protomodels.sstv``: [W] secondary, [B] from memory, unchecked) and
the program's interface (S, T, N; 2860 >> S clocks a sample at 50 MHz),
never from what the model or the RTL did:

* the VIS reads 44 (Martin M1) with even parity, each bit within 1 % of
  1100 / 1300 / 1200 Hz ([W]); the leader within 1 % of 1900 Hz;
* the scans start at the [B] offsets after the sync: G to B and B to R
  each 146.432 + 0.572 ms, line to line 446.446 ms, the sync 4.862 ms,
  all times the compression 2^-T; the firmware's own error is zero
  samples at T = 0 (every element a whole number of samples), and at
  T > 0 each element is truncated to a sample, so the tolerance is one
  sample per element plus the receiver's 20 us;
* every bar's brightness within 4 levels (of 255) of the EBU 100/75 value,
  2 levels at T = 0 (the program's frequency error is under 0.6 Hz, 0.2
  of a level; the rest is the receiver's over a short bar);
* every pin edge on the sample grid (WAITD deadlines are absolute,
  SEMANTICS 2 and 6.4; TICK a multiple of 4 makes the lateness constant).

Board: the program is proved for a tick of 76 clocks, so the tests run a
board of 76 * fs clocks a second (1.33 MHz at S = 0); the stream is sample
for sample the one a 50 MHz board sends with TICK_INT = 2860 >> S. Every
body is named ``test_sstv_*``.
"""

import os

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import sstv as V
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
TICK = 76                               # the proved minimum (.tick)
FS0 = 50e6 / 2860                       # the header: samples a second at S = 0
FIT_TOL_S = 20e-6                       # the receiver's boundary accuracy
# EBU 100/75 bars [G, B, R] by bar: white 100 %, then 75 % (191)
EBU = [(255, 255, 255), (191, 0, 191), (191, 191, 0), (191, 0, 0),
       (0, 191, 191), (0, 0, 191), (0, 191, 0), (0, 0, 0)]


def clock_of(s):
    return TICK * FS0 * (1 << s)


def start(backend, s, t, n):
    """Load and start the transmitter, then queue S, T and N: (bench, loom,
    recorder, cycle of the first word)."""
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "DMEM"), imem_words=512)
    else:
        bench = backend.bench()
    rec = bench.add(V.EdgeRecorder("OUT0"))
    loom = backend.loom(bench, isa=ISA)
    prog = assemble_file(FIRMWARE / "sstv_m1.loom", isa=ISA, strict=True)
    loom.load(prog.words)
    loom.write_csr(0, "TICK_INT", TICK)
    loom.run([0])
    bench.step(2000)
    assert rec.edges == [], "the pin must be quiet before the host words"
    c0 = bench.cycle
    loom.push(0, [s, t, n])
    return bench, loom, rec, c0


def vis_seconds(scale):
    return (2 * V.LEADER_S + V.BREAK_S + 10 * V.BIT_S) * scale


def check_vis(vis):
    assert vis.code == V.VIS_MARTIN_M1, vis
    assert vis.parity_ok, vis
    assert abs(vis.leader_hz / V.LEADER_HZ - 1) <= 0.01, vis
    want = [V.START_HZ] + [V.ONE_HZ if (V.VIS_MARTIN_M1 >> i) & 1 else V.ZERO_HZ
                           for i in range(7)]
    want += [V.ONE_HZ if vis.parity else V.ZERO_HZ, V.STOP_HZ]
    for got, w in zip(vis.freqs, want):
        assert abs(got / w - 1) <= 0.01, (vis.freqs, want)


def check_bars(line, tol):
    for c in range(3):
        for j in range(8):
            assert abs(line.bars[c][j] - EBU[j][c]) <= tol, (c, j, line.bars)


def check_grid(rec, tick):
    cycles = [c for c, _ in rec.edges]
    assert cycles and all((c - cycles[0]) % tick == 0 for c in cycles)


def check_scan_starts(line, scale, samples_tol, fs):
    """G to B and B to R: a scan and a separator each."""
    want = (V.SCAN_S + V.SEP_S) * scale
    tol = samples_tol / fs + FIT_TOL_S
    for a, b in zip(line.scan_start_s, line.scan_start_s[1:]):
        assert abs((b - a) - want) <= tol, (line.scan_start_s, want, tol)


def run_capture(bench, rec, c0, s, t, n, extra_s=0.002):
    scale = 2.0 ** -t
    clock = clock_of(s)
    total = vis_seconds(scale) + n * V.LINE_S * scale + extra_s
    bench.step(int(total * clock))
    return V.edge_times(rec.edges, clock, c0), scale


# ------------------------------------------------------------------ bodies
def test_sstv_vis_and_first_line(backend):
    """S = 0, T = 2 (every duration a quarter), one line: the VIS reads
    Martin M1 with even parity, the bars' colours within 6 levels, the G,
    B and R scans start a scan and a separator apart (within one truncated
    sample a scan, 9 elements), the pin low after the line, every edge on
    the sample grid. The PHY-GL-SDF candidate."""
    bench, loom, rec, c0 = start(backend, 0, 2, 1)
    times, scale = run_capture(bench, rec, c0, 0, 2, 1)
    vis = V.decode_vis(times, scale)
    check_vis(vis)
    line = V.decode_line(times, vis.stop_end_s, scale=scale, first=True)
    check_bars(line, 6)
    check_scan_starts(line, scale, 9, FS0)
    assert rec.level == 0, "the pin goes low after the last line"
    check_grid(rec, TICK)
    assert loom.badop() == 0


@model_only("VIS and two lines at full length, 2.4 M clocks, 6 min of RTL; "
            "the RTL runs the same schedule a quarter long")
def test_sstv_full_length_timing(backend):
    """S = 0, T = 0, two lines, Martin M1 at its real figures: the VIS,
    the bars within 2 levels, scan to scan 147.004 ms, line to line
    446.446 ms, the second line's sync 4.862 ms, each within one sample
    (57.2 us) plus the receiver's 20 us, and the first sync where the VIS
    says (within the VIS's ten truncated samples). Writes the rebuilt picture as a PPM when LOOM_SSTV_PPM names a
    file."""
    bench, loom, rec, c0 = start(backend, 0, 0, 2)
    times, scale = run_capture(bench, rec, c0, 0, 0, 2)
    vis = V.decode_vis(times, scale)
    check_vis(vis)
    tol = 1.0 / FS0 + FIT_TOL_S
    l0 = V.decode_line(times, vis.stop_end_s, first=True)
    l1 = V.decode_line(times, vis.stop_end_s + V.LINE_S)
    for ln in (l0, l1):
        check_bars(ln, 2)
        check_scan_starts(ln, 1.0, 1, FS0)
    assert abs(l1.scan_start_s[0] - l0.scan_start_s[0] - V.LINE_S) <= tol
    sync = l1.scan_start_s[0] - V.PORCH_S - l1.sync_start_s
    assert abs(sync - V.SYNC_S) <= tol, sync
    first_g = vis.stop_end_s + V.SYNC_S + V.PORCH_S
    # the VIS bits are truncated to 524 samples (29.97 ms): ten of them
    assert abs(l0.scan_start_s[0] - first_g) <= tol + 10 / FS0, (l0, first_g)
    path = os.environ.get("LOOM_SSTV_PPM")
    if path:
        img = V.rebuild(times, [l0.scan_start_s[0], l1.scan_start_s[0]])
        V.write_ppm(path, img)
    assert loom.badop() == 0


@model_only("S = 2 sends four times the samples, 0.9 M clocks")
def test_sstv_rate_s2_same_tones(backend):
    """S = 2 (TICK_INT 715 at 50 MHz), T = 3: the same VIS and the same
    bar colours as at S = 0, the phase steps shifted, not re-rounded."""
    bench, loom, rec, c0 = start(backend, 2, 3, 1)
    times, scale = run_capture(bench, rec, c0, 2, 3, 1)
    vis = V.decode_vis(times, scale)
    check_vis(vis)
    line = V.decode_line(times, vis.stop_end_s, scale=scale, first=True)
    check_bars(line, 6)
    check_scan_starts(line, scale, 9, 4 * FS0)
    check_grid(rec, TICK)
    assert loom.badop() == 0


@model_only("a second VIS-only transmission, 0.6 M clocks")
def test_sstv_vis_only_then_again(backend):
    """N = 0 sends the VIS only and the pin goes low and stays low; three
    more words start a second transmission with its own VIS."""
    bench, loom, rec, c0 = start(backend, 0, 2, 0)
    times, scale = run_capture(bench, rec, c0, 0, 2, 0)
    check_vis(V.decode_vis(times, scale))
    assert rec.level == 0
    quiet = len(rec.edges)
    bench.step(int(0.010 * clock_of(0)))
    assert len(rec.edges) == quiet, "the pin must stay low between transmissions"
    c1 = bench.cycle
    loom.push(0, [0, 2, 0])
    times, scale = run_capture(bench, rec, c1, 0, 2, 0)
    check_vis(V.decode_vis(times, scale))
    assert loom.badop() == 0
