"""L3-LIN on the model and the RTL: firmware/lin_master.loom and lin_slave.loom.

The master (thread 0, TX = OUT0, RX = IN0) sends the headers the host asks
for and either its own response or reads a slave's; the slave (thread 0
alone, or thread 1 beside the master) measures the master's bit rate from
the sync byte and publishes or subscribes according to its table.
``tools.protomodels.lin.LinBus`` makes the bus the wired AND of the chip's
TX pins and the model nodes, seen by the chip's RX pins.

Expected values come from the LIN 2.2A specification through the reference
model (PID parity, checksums, frame layout, timing limits) and from the
program headers (command and result formats), never from what the model or
the RTL did. Pin timing is measured on the bus: the master's break and
delimiter against their minimum lengths and its bit edges against its bit
time (the master's clock tolerance is +-0.5 %), the slave's response edges
against the master's bit time (F_TOL_SYNC, +-2 % after synchronisation).

Rates at 50 MHz: 19200, 10417 and 9600 bit/s, and for the RTL runs, where
a LIN frame at 19200 bit/s would be 150,000 clocks or more, faster test
rates (the programs' deadline proofs do not depend on the rate).

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import pytest

from tools.loomasm import assemble_file, assemble_text
from tools.loomisa import REPO, load
from tools.protomodels.lin import (CLOCK, F_TOL_SYNC, LinBus, LinMasterNode,
                                   LinSlaveNode, LinSniffer, checksum,
                                   frame_checksum, pid)
from tools.tests.fw_backend import model_only

ISA = load()
MASTER = REPO / "firmware" / "lin_master.loom"
SLAVE = REPO / "firmware" / "lin_slave.loom"
IMEM = 512

# The master's tick is a sixteenth of a bit (program header).
RATES = [19200, 10417, 9600]
MASTER_TICKS = {19200: (162, 195), 10417: (300, 0), 9600: (325, 133)}
FAST = CLOCK / 256                  # the RTL test rate: TICK 16, 195 kbit/s
MASTER_TICKS[FAST] = (16, 0)

# The master's command and status words (program header).
PUB = 1 << 12
CKERR, NORESP, BITERR, FRAMING = 1, 2, 4, 8
MASTER_TOL = 0.005                  # F_TOL_RES_MASTER
SLOT = 4


def command(ident, length, publish):
    return ident | (PUB if publish else 0) | ((length - 1) << 13)


def lin_bench(backend):
    """FIFOs, LD/ST (slice B) and the 512-word memory on the model; the RTL
    has all of them and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "DMEM"), imem_words=IMEM)
    return backend.bench()


def bit_time(rate):
    """The chip master's bit time in clocks: sixteen of its ticks."""
    ti, tf = MASTER_TICKS[rate]
    return 16 * (ti + tf / 256)


# ------------------------------------------------------------------ master
def master_setup(backend, rate, *, slave_error=0.0, response_space=0.0):
    """The master at ``rate`` with a model slave on the bus; returns
    ``(bench, loom, bus, slave)``."""
    bench = lin_bench(backend)
    slave = LinSlaveNode(CLOCK / bit_time(rate), clock_error=slave_error,
                         response_space=response_space)
    bus = bench.add(LinBus([slave]))
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(MASTER, isa=ISA, strict=True).words)
    ti, tf = MASTER_TICKS[rate]
    loom.write_csr(0, "TICK_INT", ti)
    loom.write_csr(0, "TICK_FRAC", tf)
    loom.run(0)
    bench.step(64)
    return bench, loom, bus, slave


def frame_clocks(rate, length):
    """A frame slot: the header, TResponse_max and a margin, in clocks."""
    return int((34 * 1.4 + 14 * (length + 1) + 8) * bit_time(rate))


def master_frame(bench, loom, rate, ident, data=None, length=None):
    """Has the chip master run one frame: publish ``data`` or read
    ``length`` bytes; returns ``(status, bytes)`` from OUTQ[0]."""
    publish = data is not None
    length = len(data) if publish else length
    loom.push(0, [command(ident, length, publish)] + (list(data) if publish else []))
    status = loom.pop(0, 1, max_polls=frame_clocks(rate, length) // 200)[0]
    n = (status >> 4) & 15
    return status, loom.pop(0, n) if n else []


def assert_master_timing(frame, T, count=None):
    """The chip master's header and bytes on the bus against LIN 2.2A: a
    break of at least 13 bit times, a delimiter of at least one, and every
    edge of every byte it sent within its clock tolerance of the bit grid
    that byte's start edge sets, plus the 4-clock slot grid (SEMANTICS 2);
    ``count`` limits that to the master's own bytes."""
    assert frame.break_clocks >= 13 * T, (frame.break_clocks, 13 * T)
    assert frame.break_clocks <= 13.25 * T + 2 * SLOT, (frame.break_clocks, T)
    assert frame.delimiter_clocks >= T, (frame.delimiter_clocks, T)
    for byte in frame.bytes[:count]:
        for cycle, _ in byte.edges:
            k = round((cycle - byte.start) / T)
            off = cycle - byte.start - k * T
            assert abs(off) <= MASTER_TOL * k * T + SLOT, (byte.start, k, off)


def test_lin_master_publishes_two_bytes_fast(backend):
    """The quick one, at the RTL test rate: a frame the master publishes
    reaches the model slave whole and on time."""
    bench, loom, bus, slave = master_setup(backend, FAST)
    slave.subscribe(0x12, 2)
    status, got = master_frame(bench, loom, FAST, 0x12, data=[0xA5, 0x3C])
    ck = frame_checksum(0x12, [0xA5, 0x3C])
    assert status == 3 << 4, hex(status)
    assert got == [0xA5, 0x3C, ck]
    rec = slave.received()
    assert len(rec) == 1 and rec[0].data == [0xA5, 0x3C] and rec[0].checksum_ok
    frame = slave.decoder.frames[-1]
    assert frame.pid == pid(0x12) and frame.response == [0xA5, 0x3C, ck]
    assert_master_timing(frame, bit_time(FAST))
    assert loom.badop() == 0


def publish_and_check(bench, loom, slave, rate, ident, data):
    """The master publishes ``data``; the model slave must receive it with
    a good checksum and the master report it as sent."""
    slave.subscribe(ident, len(data))
    count = len(slave.received())
    status, got = master_frame(bench, loom, rate, ident, data=data)
    ck = frame_checksum(ident, data)
    assert status == (len(data) + 1) << 4, hex(status)
    assert got == list(data) + [ck]
    rec = slave.received()
    assert len(rec) == count + 1 and rec[-1].data == list(data) and rec[-1].checksum_ok
    frame = slave.decoder.frames[-1]
    assert frame.pid == pid(ident) and frame.response == list(data) + [ck]
    assert_master_timing(frame, bit_time(rate))


def request_and_check(bench, loom, slave, rate, ident, data):
    """The model slave publishes ``data``; the master must read it."""
    slave.publish(ident, data)
    status, got = master_frame(bench, loom, rate, ident, length=len(data))
    assert status == (len(data) + 1) << 4, hex(status)
    assert got == list(data) + [frame_checksum(ident, data)]
    frame = slave.decoder.frames[-1]
    assert frame.pid == pid(ident)
    assert_master_timing(frame, bit_time(rate), 2)   # its header


@model_only("16 frames at the RTL test rate, about 450,000 clocks")
def test_lin_master_every_length_both_ways_fast(backend):
    """Lengths 1 to 8, the master's own response and a slave's, with the
    enhanced checksum (identifiers other than 0x3C and 0x3D)."""
    bench, loom, bus, slave = master_setup(backend, FAST)
    for length in range(1, 9):
        data = [(0x11 * length + 7 * i) & 0xFF for i in range(length)]
        publish_and_check(bench, loom, slave, FAST, length, data)
        request_and_check(bench, loom, slave, FAST, 0x20 + length,
                          [b ^ 0xFF for b in data])
    assert loom.badop() == 0


@model_only("LIN rates: 150,000 to 600,000 clocks a frame")
@pytest.mark.parametrize("rate", RATES)
def test_lin_master_at_lin_rates(backend, rate):
    """At 19200, 10417 and 9600 bit/s: a frame each way, timed against the
    specification on the bus."""
    bench, loom, bus, slave = master_setup(backend, rate)
    publish_and_check(bench, loom, slave, rate, 0x01, [0x55, 0xAA])
    request_and_check(bench, loom, slave, rate, 0x31, [0x80, 0x01, 0xFE])


@model_only("two 8-byte frames; the RTL runs the classic checksum in the together test")
def test_lin_master_classic_checksum_for_diagnostic_frames(backend):
    """0x3C (master request) and 0x3D (slave response) use the classic
    checksum, over the data alone, whatever their length."""
    bench, loom, bus, slave = master_setup(backend, FAST)
    request = [0x7F, 0x06, 0xB2, 0x00, 0xFF, 0x7F, 0xFF, 0xFF]
    publish_and_check(bench, loom, slave, FAST, 0x3C, request)
    assert slave.received()[-1].data == request
    assert frame_checksum(0x3C, request) == checksum(request)
    reply = [0x7F, 0x06, 0xF2, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF]
    request_and_check(bench, loom, slave, FAST, 0x3D, reply)


def test_lin_master_no_response(backend):
    """Nobody publishes the identifier: no byte at all, so n = 0 and the
    no-response flag, after TResponse_max."""
    bench, loom, bus, slave = master_setup(backend, FAST)
    status, got = master_frame(bench, loom, FAST, 0x2A, length=4)
    assert status == NORESP and got == [], hex(status)
    frame = slave.decoder.frames[-1]
    assert frame.pid == pid(0x2A) and frame.response == []


def test_lin_master_checksum_error_from_the_slave(backend):
    """The model slave spoils its checksum: the master reports the bytes as
    they were on the bus and the checksum error."""
    bench, loom, bus, slave = master_setup(backend, FAST)
    data = [0x10, 0x20, 0x30]
    slave.publish(0x07, data, checksum_error=True)
    status, got = master_frame(bench, loom, FAST, 0x07, length=3)
    assert status == CKERR | (4 << 4), hex(status)
    assert got == data + [frame_checksum(0x07, data) ^ 0x01]


def test_lin_master_bit_error(backend):
    """Another node holds the bus dominant across the middle of the sync
    byte's first data bit, a 1: the master reads back a 0 and reports a
    bit error (the frame itself goes out to the end)."""
    bench, loom, bus, slave = master_setup(backend, FAST)
    T = bit_time(FAST)
    slave.subscribe(0x15, 1)
    seen = len(bus.edges)
    loom.push(0, [command(0x15, 1, True), 0x42])
    assert bench.run_until(lambda: len(bus.edges) >= seen + 2, int(40 * T), every=8)
    rise = bus.edges[seen + 1][0]                       # the break's end
    slave.force_dominant(rise + 2.25 * T, 0.5 * T)      # sync bit 0, mid-bit
    status = loom.pop(0, 1, max_polls=frame_clocks(FAST, 1) // 200)[0]
    assert status == BITERR | (2 << 4), hex(status)
    assert loom.pop(0, 2) == [0x42, frame_checksum(0x15, [0x42])]
    assert not slave.received()                         # the sync was spoiled


@model_only("the deadline arithmetic the no-response case runs on the RTL, at its edge")
@pytest.mark.parametrize("space, late", [(7.5, False), (8.5, True)])
def test_lin_master_response_window(backend, space, late):
    """TResponse_max for one data byte is 1.4 x 20 = 28 bit times, so the
    response may start up to 8 bit times after the header: 7.5 is read,
    8.5 is too late and reported as no response, nothing read."""
    bench, loom, bus, slave = master_setup(backend, FAST, response_space=space)
    slave.publish(0x0B, [0x5A])
    status, got = master_frame(bench, loom, FAST, 0x0B, length=1)
    if late:
        assert status == NORESP and got == [], hex(status)
    else:
        assert status == 2 << 4 and got == [0x5A, frame_checksum(0x0B, [0x5A])]


@model_only("two more frames that prove what the +-0 % ones prove")
@pytest.mark.parametrize("error", [-F_TOL_SYNC, F_TOL_SYNC])
def test_lin_master_reads_a_slave_two_per_cent_off(backend, error):
    """A synchronised slave may be 2 % off the master's bit rate: every bit
    of its eight bytes and checksum is still read."""
    bench, loom, bus, slave = master_setup(backend, FAST, slave_error=error)
    request_and_check(bench, loom, slave, FAST, 0x22,
                      [0xFF, 0x00, 0xF0, 0x0F, 0xAA, 0x55, 0x01, 0x80])


@model_only("the time-out path the no-response case runs on the RTL, after three bytes")
def test_lin_master_slave_stops_early(backend):
    """The slave sends two bytes and its checksum where the master asked
    for four: n = 3 and no response, the three bytes reported."""
    bench, loom, bus, slave = master_setup(backend, FAST)
    slave.publish(0x0C, [0x01, 0x02])
    status, got = master_frame(bench, loom, FAST, 0x0C, length=4)
    assert status == NORESP | (3 << 4), hex(status)
    assert got == [0x01, 0x02, frame_checksum(0x0C, [0x01, 0x02])]


# ------------------------------------------------------------------- slave
SLAVE_PUB, SLAVE_CKERR, SLAVE_PARITY = 0x10, 0x20, 0x40     # status bits 15:8
SLAVE_FAST = 100_000                # the slave's RTL test rate: 500 clocks a bit


def slave_table(entries):
    """The INQ words of the slave's table: c, then each entry's header and,
    for a published frame, its data (program header)."""
    words = [len(entries)]
    for ident, length, data in entries:
        words.append(command(ident, length, data is not None))
        words += list(data or [])
    return words


def slave_setup(backend, rate, entries, *, clock_error=0.0, break_bits=13,
                delimiter_bits=1):
    """The slave (thread 0, its own tick) with a model master at ``rate``
    (times ``1 + clock_error``) on the bus, the table loaded; returns
    ``(bench, loom, bus, master)``."""
    bench = lin_bench(backend)
    master = LinMasterNode(rate, clock_error=clock_error, break_bits=break_bits,
                           delimiter_bits=delimiter_bits)
    bus = bench.add(LinBus([master]))
    loom = backend.loom(bench, isa=ISA)
    loom.load(assemble_file(SLAVE, isa=ISA, strict=True).words)
    loom.run(0)
    loom.push(0, slave_table(entries))
    bench.step(2000)
    return bench, loom, bus, master


def run_frame(bench, master, transfer):
    """Step until the master node's frame slot is over."""
    assert bench.run_until(lambda: not master.busy, 400 * int(master.T), every=256)
    return master.frame_of(transfer)


def slave_report(loom):
    """``(status, bit time, bytes)`` from OUTQ[0]."""
    status, bit = loom.pop(0, 2)
    n = (status >> 8) & 15
    return status, bit, loom.pop(0, n) if n else []


def assert_slave_timing(frame, T):
    """Every edge of the slave's response within F_TOL_SYNC (2 %) of the
    master's bit time T, counted from the start edge of its byte, plus the
    4-clock slot grid."""
    assert len(frame.bytes) > 2
    for byte in frame.bytes[2:]:
        for cycle, _ in byte.edges:
            k = round((cycle - byte.start) / T)
            off = cycle - byte.start - k * T
            assert abs(off) <= F_TOL_SYNC * k * T + SLOT, (byte.start, k, off, T)


@model_only("the together test runs the slave both ways on the RTL, timed the same way")
def test_lin_slave_publishes_and_subscribes_fast(backend):
    """At the RTL test rate: a frame the slave publishes, on time against
    the master's bit time, and one it subscribes to, reported whole."""
    bench, loom, bus, master = slave_setup(
        backend, SLAVE_FAST, [(0x21, 2, [0x12, 0xEF]), (0x05, 3, None)])
    tr = master.request(0x21, 2)
    frame = run_frame(bench, master, tr)
    assert frame.response == [0x12, 0xEF, frame_checksum(0x21, [0x12, 0xEF])]
    assert_slave_timing(frame, master.T)
    status, bit, data = slave_report(loom)
    assert status == (SLAVE_PUB << 8) | pid(0x21), hex(status)
    assert abs(bit - master.T) <= 8, (bit, master.T)
    tr = master.publish(0x05, [0x01, 0x80, 0x7F])
    run_frame(bench, master, tr)
    status, bit, data = slave_report(loom)
    assert status == (4 << 8) | pid(0x05), hex(status)
    assert data == [0x01, 0x80, 0x7F, frame_checksum(0x05, [0x01, 0x80, 0x7F])]
    assert loom.badop() == 0


def slave_publishes(bench, loom, master, ident, data):
    """The model master asks for ``ident``; the slave's response must be
    ``data`` with its checksum, on time, and reported."""
    tr = master.request(ident, len(data))
    frame = run_frame(bench, master, tr)
    assert frame.response == list(data) + [frame_checksum(ident, data)], frame.response
    assert_slave_timing(frame, master.T)
    status, bit, got = slave_report(loom)
    assert status == (SLAVE_PUB << 8) | pid(ident) and got == [], hex(status)
    return bit


@model_only("nine frames at LIN rates, 150,000 to 450,000 clocks each")
@pytest.mark.parametrize("rate", RATES)
@pytest.mark.parametrize("error", [-0.14, 0.0, 0.14])
def test_lin_slave_follows_any_master_rate(backend, rate, error):
    """The model master at 9600, 10417 and 19200 bit/s with its clock
    14 % slow, exact, or 14 % fast (F_TOL_UNSYNC): the slave measures the
    sync byte, answers with every edge within 2 % of the master's bit time
    (F_TOL_SYNC), and reports the bit time it measured."""
    bench, loom, bus, master = slave_setup(backend, rate, [(0x10, 1, [0xC3])],
                                           clock_error=error)
    bit = slave_publishes(bench, loom, master, 0x10, [0xC3])
    assert abs(bit - master.T) <= 0.002 * master.T + 8, (bit, master.T)


def test_lin_slave_ignores_bad_parity(backend):
    """A header whose PID parity is wrong is not answered even though the
    identifier is in the table; the parity error is reported with the PID
    as received, and the next good header is answered."""
    bench, loom, bus, master = slave_setup(backend, SLAVE_FAST, [(0x22, 1, [0x99])])
    tr = master.request(0x22, 1, parity_error=True)
    frame = run_frame(bench, master, tr)
    assert frame.response == []
    status, bit, got = slave_report(loom)
    assert status == (SLAVE_PARITY << 8) | (pid(0x22) ^ 0x40), hex(status)
    slave_publishes(bench, loom, master, 0x22, [0x99])


@model_only("three frames; classic is in the together test, the checksum check in the master's")
def test_lin_slave_classic_checksum_and_checksum_error(backend):
    """0x3C (master request, subscribed) and 0x3D (slave response,
    published) use the classic checksum; a spoiled checksum from the
    master is reported with the bytes received."""
    request = [0x7F, 0x06, 0xB2, 0x00, 0xFF, 0x7F, 0xFF, 0xFF]
    reply = [0x7F, 0x06, 0xF2, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF]
    bench, loom, bus, master = slave_setup(
        backend, SLAVE_FAST, [(0x3C, 8, None), (0x3D, 8, reply), (0x08, 2, None)])
    run_frame(bench, master, master.publish(0x3C, request))
    status, bit, got = slave_report(loom)
    assert status == (9 << 8) | pid(0x3C), hex(status)
    assert got == request + [checksum(request)]
    slave_publishes(bench, loom, master, 0x3D, reply)
    run_frame(bench, master, master.publish(0x08, [0x01, 0x02], checksum_error=True))
    status, bit, got = slave_report(loom)
    assert status == ((SLAVE_CKERR | 3) << 8) | pid(0x08), hex(status)
    assert got == [0x01, 0x02, frame_checksum(0x08, [0x01, 0x02]) ^ 0x01]


@model_only("the break check every header runs on the RTL, at its threshold")
@pytest.mark.parametrize("bits, seen", [(10, False), (11.5, True)])
def test_lin_slave_break_threshold(backend, bits, seen):
    """A dominant run of 10 bit times is not a break; one of 11.5 is (the
    threshold is 11 nominal bit times, the measured bit time being the
    sync byte's)."""
    bench, loom, bus, master = slave_setup(backend, SLAVE_FAST, [(0x2C, 1, [0x3E])],
                                           break_bits=bits)
    tr = master.request(0x2C, 1)
    frame = run_frame(bench, master, tr)
    if seen:
        assert frame.response == [0x3E, frame_checksum(0x2C, [0x3E])]
        assert slave_report(loom)[0] == (SLAVE_PUB << 8) | pid(0x2C)
    else:                               # (the model's decoder ignores it too)
        assert bus.edges[-1][0] <= tr.header_end + 2, "the slave answered"
        assert loom.pop_available(0) == []


@model_only("two long frames of zero bytes, about 250,000 clocks")
def test_lin_slave_is_not_fooled_by_zero_bytes(backend):
    """Bytes of 0x00 are nine dominant bits, the longest run data can make:
    a frame of them for another node, then a header for this slave, which
    must answer it and report nothing else; a long delimiter is fine too."""
    bench, loom, bus, master = slave_setup(backend, SLAVE_FAST, [(0x1E, 2, [0x00, 0x00])],
                                           delimiter_bits=4)
    run_frame(bench, master, master.publish(0x31, [0x00] * 8))
    assert loom.pop_available(0) == []
    slave_publishes(bench, loom, master, 0x1E, [0x00, 0x00])
    assert loom.pop_available(0) == []


# ---------------------------------------------------------------- together
TOGETHER_RATE = 100_000                     # master TICK 31.25: 500 clocks a bit
MASTER_TICKS[TOGETHER_RATE] = (31, 64)


def together_image():
    """The master on thread 0 (OUT0, IN0) and the slave on thread 1 (OUT1,
    IN1): each file with only its ``.thread`` and ``.pins`` lines changed,
    assembled for the 512-word memory, as one image."""
    words = {}
    for path, thread, pins in ((MASTER, 0, None),
                               (SLAVE, 1, ".pins   TX = OUT1, RX = IN1")):
        text = path.read_text(encoding="utf-8")
        if thread:
            for old, new in ((".thread 0", ".thread 1"),
                             (".pins   TX = OUT0, RX = IN0", pins)):
                assert text.count(old) == 1, old
                text = text.replace(old, new)
        prog = assemble_text(text, path.name, isa=ISA, imem_words=IMEM, strict=True)
        assert prog.diagnostics == []
        overlap = set(words) & set(prog.words)
        assert not overlap, "the programs overlap at %s" % sorted(overlap)[:4]
        words.update(prog.words)
    return words


def test_lin_master_and_slave_together(backend):
    """The master on thread 0 and the slave on thread 1, each with its own
    TX and RX pins, the bench wiring both TX pins into one wired-AND bus
    seen by both RX pins. The slave publishes 0x2D (enhanced checksum) and
    subscribes to 0x3C, the master request (classic): both programs'
    reports agree with the specification, and a listener on the bus holds
    the master's bytes to its bit time and the slave's to 2 % of it."""
    bench = lin_bench(backend)
    T = CLOCK / TOGETHER_RATE
    ear = LinSniffer(T)
    bench.add(LinBus([ear], tx=("OUT0", "OUT1"), rx=("IN0", "IN1")))
    loom = backend.loom(bench, isa=ISA)
    loom.load(together_image())
    ti, tf = MASTER_TICKS[TOGETHER_RATE]
    loom.write_csr(0, "TICK_INT", ti)
    loom.write_csr(0, "TICK_FRAC", tf)
    loom.run([0, 1])
    loom.push(1, slave_table([(0x2D, 2, [0xBE, 0xEF]), (0x3C, 8, None)]))
    status, got = master_frame(bench, loom, TOGETHER_RATE, 0x2D, length=2)
    assert status == 3 << 4 and got == [0xBE, 0xEF, frame_checksum(0x2D, [0xBE, 0xEF])]
    sstatus, bit, _ = slave_report_of(loom, 1)
    assert sstatus == (SLAVE_PUB << 8) | pid(0x2D), hex(sstatus)
    assert abs(bit - T) <= 8, bit
    request = [0x7F, 0x06, 0xB2, 0x00, 0xFF, 0x7F, 0xFF, 0xFF]
    status, got = master_frame(bench, loom, TOGETHER_RATE, 0x3C, data=request)
    assert status == 9 << 4 and got == request + [checksum(request)]
    sstatus, bit, sgot = slave_report_of(loom, 1)
    assert sstatus == (9 << 8) | pid(0x3C) and sgot == request + [checksum(request)]
    first, second = ear.decoder.frames[-2:]
    assert_master_timing(first, T, 2)
    assert_slave_timing(first, T)
    assert_master_timing(second, T)
    assert loom.badop() == 0


def slave_report_of(loom, thread):
    status, bit = loom.pop(thread, 2)
    n = (status >> 8) & 15
    return status, bit, loom.pop(thread, n) if n else []
