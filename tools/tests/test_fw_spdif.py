"""L3 for the S/PDIF transmitter, on the model and the RTL.

``firmware/spdif_tx.loom`` (TX = OUT0) sends IEC 60958 consumer frames at
one half-cell (UI) a tick: thread 0 shifts 16-half-cell words out of the
bit engine's NRZI encoder, thread 1 builds them into a ring in data memory.
The reference is ``tools.protomodels.spdif``, written from the public
descriptions of IEC 60958 / AES3 (EBU Tech 3250, the Wikipedia articles):
``receive`` holds the recorded pin to the UI grid, checks every interval
(1, 2 or 3 UI), the edge jitter against the transmitter's 0.025 UI, the
preambles, the bit-boundary transitions, the parity and the block, and
returns the samples and the channel status; ``encode_stream`` is the
half-cell sequence the standard asks for, compared cell by cell.

Board clocks. The program is for 32 kHz on a 49.152 MHz clock: TICK_INT
12, so a UI is exactly 12 clocks and a frame 1536. The 50 MHz case (TICK
12 + 53/256) runs on the model only, to measure the jitter that makes it
out of specification.

Expected values come from the standard and the program's stated signal
(header): left a sawtooth from 0 rising 0x0400 a frame, right +0x4000 for
frames 0..31 of every 64 and -0x4000 for 32..63, channel status bytes
0x04, 0, 0, 0x03, 0x02 and zeros.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import spdif as S
from tools.tests.fw_backend import model_only

ISA = load()
FIRMWARE = REPO / "firmware"
IMEM = 512
UI = 12                                     # clocks at 49.152 MHz, 32 kHz
FRAME = S.UI_FRAME * UI
STATUS = [0x04, 0x00, 0x00, 0x03, 0x02]


def signal(nframes):
    """The program's samples for frames 0 .. nframes-1, signed 16-bit."""
    out = []
    for f in range(nframes):
        left = (f * 0x0400) & 0xFFFF
        left = left - 0x10000 if left & 0x8000 else left
        out.append((left, 0x4000 if (f % 192) & 32 == 0 else -0x4000))
    return out


def spdif_bench(backend):
    """FIFOs, the bit engine with its encoders, the data memory, 512 words."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM)
    return backend.bench()


def tx_run(backend, nframes, tick=(UI, 0)):
    """Run the transmitter for ``nframes`` frames past its first edge;
    returns the recorded edges."""
    bench = spdif_bench(backend)
    rec = bench.add(S.EdgeRecorder("OUT0"))
    loom = backend.loom(bench, isa=ISA)
    prog = assemble_file(FIRMWARE / "spdif_tx.loom", isa=ISA, strict=True)
    loom.load(prog.words)
    loom.run([0, 1])
    loom.push(0, list(tick))
    assert bench.run_until(lambda: rec.edges, 20000, every=256), "no first edge"
    ui = tick[0] + tick[1] / 256
    bench.step(int(nframes * S.UI_FRAME * ui) + 2 * int(ui) - (bench.cycle - rec.edges[0][0]))
    assert loom.badop() == 0
    assert rec.edges[0][1] == 1                     # from idle low, a rise
    return rec.edges


def test_spdif_tx_first_frames(backend):
    """49.152 MHz, 32 kHz: the first three frames from the B preamble,
    every edge exactly on the 12-clock UI grid, the half-cells exactly as
    the standard encodes the stated samples and C bits, both channels."""
    edges = tx_run(backend, 3)
    cells = S.cells_from_edges(edges, UI, tol=0.0)
    assert cells.jitter_pp == 0
    assert set(cells.widths) == {1, 2, 3}
    want = S.encode_stream(signal(3), STATUS)
    assert cells.levels[:len(want) - 1] == want[:len(want) - 1]
    stream = S.receive(edges, UI)
    assert stream.block_starts == [0]
    assert stream.samples[:3] == signal(3)
    assert [l.c for l, _ in stream.frames[:3]] == [0, 0, 1]   # byte 0 bit 2


@model_only("a block and two frames, 298,000 clocks; three frames run on the RTL")
def test_spdif_tx_block(backend):
    """49.152 MHz, 32 kHz: a whole block and the next B. Every cell width
    checked (1, 2 or 3 UI, 3 only in preambles), no edge off the grid, B
    every 192 frames, parity, the samples, the channel status block."""
    n = S.FRAMES + 2
    edges = tx_run(backend, n)
    cells = S.cells_from_edges(edges, UI, tol=0.0)
    assert cells.jitter_pp == 0
    want = S.encode_stream(signal(n), STATUS)
    assert cells.levels[:len(want) - 1] == want[:len(want) - 1]
    stream = S.receive(edges, UI)
    assert stream.block_starts == [0, S.FRAMES]
    assert stream.samples == signal(len(stream.frames))
    status = stream.status(0)
    assert status == STATUS + [0] * 19
    fields = S.consumer_status(status)
    assert fields["professional"] == 0 and fields["non_pcm"] == 0
    assert fields["fs"] == 32000 and fields["copy_permitted"] == 1
    assert all(l.v == 0 and l.u == 0 and r.v == 0 and r.u == 0
               for l, r in stream.frames)


@model_only("measures the 50 MHz case the program does not claim")
def test_spdif_tx_50mhz_jitter(backend):
    """50 MHz, TICK 12 + 53/256: 32 kHz on average and decodable, but SHO
    writes on the 4-clock slot grid, so an edge lands 0 to 3 clocks after
    its tick, and the fixed 3-slot spacing of half-cells 14, 15, 0 and 1
    drifts up to 3 x 53/256 clocks early against the 12.207-clock UI: about
    0.36 UI peak to peak, more than the 0.025 UI peak a transmitter may
    have (EBU Tech 3250 6.2.5.1) and than a receiver's 0.25 UI p-p
    (unchecked). An ideal receiver, holding edges to 0.25 UI, still reads it."""
    ui = 12 + 53 / 256
    edges = tx_run(backend, 4, tick=(12, 53))
    with pytest.raises(AssertionError, match="edge jitter"):
        S.cells_from_edges(edges, ui)
    with pytest.raises(AssertionError, match="edge jitter"):
        S.cells_from_edges(edges, ui, tol=S.JITTER_RX_PP / 2)
    cells = S.cells_from_edges(edges, ui, tol=0.25)
    assert S.JITTER_RX_PP < cells.jitter_pp <= (4 + 3 * 53 / 256) / ui
    stream = S.receive(edges, ui, tol=0.25)
    assert stream.samples[:4] == signal(4)
    span = edges[-1][0] - edges[0][0]
    assert abs(span / ui - sum(cells.widths)) < 0.5   # no drift: the rate is exact


def test_spdif_model_round_trip():
    """The model alone: an encoded block decodes to its samples and status,
    in either starting polarity."""
    for last in (0, 1):
        sig = signal(S.FRAMES + 1)
        cells = S.encode_stream(sig, STATUS, last=last)
        edges, level = [], last
        for k, c in enumerate(cells):
            if c != level:
                edges.append((10 + UI * k, c))
                level = c
        stream = S.receive(edges, UI)
        assert stream.samples == sig[:len(stream.frames)]
        assert stream.status(0) == STATUS + [0] * 19
