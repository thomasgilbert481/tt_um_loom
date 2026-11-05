"""L3-IOLINK on the model and the RTL: firmware/iolink_master.loom and iolink_device.loom.

The master (thread 0: TX = OUT0, TXEN = OUT1, RX = IN0) runs the wake-up
request and the M-sequences the host asks for; the device (thread 2: TX =
OUT2, TXEN = OUT3, RX = IN1) wakes on the request, serves Direct Parameter
page 1 from the image and exchanges process data. ``tools.protomodels.
iolink.IoLinkLine`` is the C/Q line at the PHYs' logic level, with each
PHY's delay a parameter, and the model master and device are the other
ends.

Expected values come from the IO-Link Interface and System Specification
v1.1.3 through the reference model (checksum, M-sequence layout, page
addresses, the timing windows of Table 9, Table 10, Table 42 and Annex
A.3) and from the program headers (command and result formats), never from
what the model or the RTL did. Pin timing is measured on the line: the
wake-up pulse against TWU, the master's bit edges against its bit time,
t1 and t2 between characters, tA, and the cycle time.

COM2 at 50 MHz is TICK 81 + 97/256 (1302.06 clocks a bit). An M-sequence
there is 60,000 clocks or more, so the RTL runs use a test rate of TICK 10
(160 clocks a bit, 312.5 kbit/s); TWU, TDMT and the device's wake-up window
are then the same number of bit times as at COM2, which is what the
programs count in. The programs' deadline proofs do not depend on the rate.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import pytest

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.iolink import (CH_DIAGNOSIS, CH_PAGE, CLOCK, COM2, DEVICE_ID,
                                      MIN_CYCLE_TIME, READ, TA_BITS, TINITCYC_BITS,
                                      TWU_US, TYPE_0, TYPE_2, VENDOR_ID, WRITE,
                                      DeviceNode, IoLinkLine, MasterNode,
                                      bit_clocks, characters_after, check_ok,
                                      device_reply, edge_offsets, gaps_bits,
                                      master_message, mc)
from tools.tests.fw_backend import model_only

ISA = load()
MASTER = REPO / "firmware" / "iolink_master.loom"
DEVICE = REPO / "firmware" / "iolink_device.loom"
IMEM = 512
SLOT = 4

# Sixteen ticks a bit (program headers).
TICKS = {"COM2": (81, 97), "FAST": (10, 0)}
TBIT_COM2 = bit_clocks(COM2)
W_TICKS = 49                        # the wake-up: 49 ticks, 79.8 us at COM2

# Master command and status (iolink_master.loom header).
OP_WAKE, OP_CYCLE = 1 << 14, 2 << 14
TYPE2, CYC = 1 << 12, 1 << 13
CKERR, NORESP, PARERR, FRAMING = 1, 2, 4, 8

# Device configuration and status (iolink_device.loom header).
D_CKERR, D_PARERR, D_FRAMING, D_SHORT, D_BADTYPE, D_REPLIED = (1 << b for b in range(8, 14))
WMIN, WMAX = 40, 64                 # ticks: 65 to 104 us at COM2

# Direct Parameter page 1 (Table B.1): MinCycleTime 2.3 ms (B.1.3),
# RevisionID 0x11, ProcessDataIn 8 bits, VendorID 0x0123, DeviceID 0x456789.
PAGE = [0x00, 0x00, 0x17, 0x00, 0x11, 0x08, 0x00, 0x01,
        0x23, 0x45, 0x67, 0x89, 0x00, 0x00, 0x00, 0x00]


def tbit(rate):
    ti, tf = TICKS[rate]
    return 16 * (ti + tf / 256)


def tick(rate):
    return tbit(rate) / 16


def twu_window(T):
    """TWU (Table 10) in clocks, as bit times of COM2 scaled to ``T``."""
    return tuple(us * 1e-6 * CLOCK / TBIT_COM2 * T for us in TWU_US)


def bench_for(backend):
    """FIFOs, the bit engine, LD/ST and the 512-word memory on the model;
    the RTL has all of them and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC", "DMEM"), imem_words=IMEM)
    return backend.bench()


def start_thread(loom, thread, rate):
    ti, tf = TICKS[rate]
    loom.write_csr(thread, "TICK_INT", ti)
    loom.write_csr(thread, "TICK_FRAC", tf)
    loom.run(thread)


def master_image():
    return assemble_file(MASTER, isa=ISA, strict=True)


def device_image(page=PAGE):
    """The device's image with ``page`` in Direct Parameter page 1."""
    prog = assemble_file(DEVICE, isa=ISA, strict=True)
    words = dict(prog.words)
    base = prog.symbols["PAGE"]
    for i, octet in enumerate(page):
        words[base + i] = octet
    return words


def mseq(mc_octet, *, n_in=0, n_out=0, type2=False, cyc=False):
    return mc_octet | (n_in << 8) | (n_out << 10) | (TYPE2 if type2 else 0) | (CYC if cyc else 0)


# ------------------------------------------------------------------ master
def master_setup(backend, rate, *, phy_delay=0, **device):
    """The chip master at ``rate`` with a model device on the line; returns
    ``(bench, loom, line, dev)``."""
    T = tbit(rate)
    bench = bench_for(backend)
    dev = DeviceNode(T, page=PAGE, wake_window=twu_window(T), **device)
    line = bench.add(IoLinkLine(T, [dev], ports=[("OUT0", "OUT1", "IN0")],
                                phy_delay=phy_delay))
    loom = backend.loom(bench, isa=ISA)
    loom.load(master_image().words)
    start_thread(loom, 0, rate)
    bench.step(64)
    return bench, loom, line, dev


def master_run(loom, cmd, words=()):
    """One M-sequence: returns ``(status, octets)`` from OUTQ[0]."""
    loom.push(0, [cmd] + list(words))
    status = loom.pop(0, 1, max_polls=20000)[0]
    n = (status >> 4) & 15
    return status, (loom.pop(0, n, max_polls=200) if n else [])


def assert_master_chars(line, since, T, expect):
    """The chip master's characters on the line from ``since``: the octets
    ``expect``, good parity and stop bits, every edge on the bit grid of its
    start edge (+-0.1 % of the bit time, Table 9, plus the 4-clock slot
    grid, SEMANTICS 2) and t1 <= 1 bit time (A.3.3). Returns them."""
    chars = characters_after(line.rx["chip0"], since)[:len(expect)]
    assert [c.value for c in chars] == list(expect), [hex(c.value) for c in chars]
    assert all(c.ok for c in chars)
    for c in chars:
        assert all(abs(off) <= 0.001 * 11 * T + SLOT for off in edge_offsets(c, T)), edge_offsets(c, T)
    assert all(-SLOT / T <= g <= 1 for g in gaps_bits(chars, T)), gaps_bits(chars, T)
    return chars


def assert_wakeup(line, dev, T, rate):
    """The wake-up pulse on the line against TWU (Table 10), the device
    model woken by it, and the first message after TDMT (Table 42)."""
    assert len(dev.wakeups) == 1
    start, width = line.low_runs["chip0"][0]
    lo, hi = twu_window(T)
    assert lo <= width <= hi, (width, lo, hi)
    if rate == "COM2":
        assert 75 <= width * 1e6 / CLOCK <= 85
    first = characters_after(line.rx["chip0"], start + width)[0]
    tdmt = (first.start - (start + width)) / T
    assert 27 <= tdmt <= 37, tdmt
    if rate == "COM2":
        assert (first.start - (start + width)) * 1e6 / CLOCK >= 500     # TREN


def test_iolink_master_wakes_and_reads_the_page_fast(backend):
    """The quick one, at the test rate: a wake-up, the TYPE_0 read of
    MinCycleTime that 7.3.2.2 puts first, then VendorID 1 and 2 with
    Tinitcyc (100 bit times, Table A.7) between the starts."""
    bench, loom, line, dev = master_setup(backend, "FAST")
    T = tbit("FAST")
    loom.push(0, [OP_CYCLE, TINITCYC_BITS * 16])
    loom.push(0, [OP_WAKE | W_TICKS])
    status, got = master_run(loom, mc(READ, CH_PAGE, MIN_CYCLE_TIME))
    assert status & 0xFF == 2 << 4, hex(status)
    assert got == device_reply(PAGE[MIN_CYCLE_TIME])
    assert abs((status >> 8) - 2 * 16) <= 2, status >> 8     # tA: the model's 2 bits
    assert_wakeup(line, dev, T, "FAST")
    wake_end = sum(line.low_runs["chip0"][0])
    first = assert_master_chars(line, wake_end, T, master_message(0xA2, TYPE_0))
    assert first[0].value == 0xA2 and first[1].value == 0x00      # A.1.6 by hand
    starts = [first[0].start]
    for addr in (VENDOR_ID, VENDOR_ID + 1):
        status, got = master_run(loom, mseq(mc(READ, CH_PAGE, addr), cyc=True))
        assert status & 0xFF == 2 << 4 and got == device_reply(PAGE[addr])
        sent = characters_after(line.rx["chip0"], starts[-1] + 30 * T)
        starts.append(sent[0].start)
    for a, b in zip(starts, starts[1:]):
        assert abs((b - a) - TINITCYC_BITS * T) <= SLOT, (b - a)
    assert loom.badop() == 0


def woken(backend, rate, **device):
    bench, loom, line, dev = master_setup(backend, rate, **device)
    loom.push(0, [OP_WAKE | W_TICKS])
    status, got = master_run(loom, mc(READ, CH_PAGE, MIN_CYCLE_TIME))
    assert status & 0xFF == 2 << 4
    return bench, loom, line, dev


@model_only("119,000 clocks; the together test runs TYPE_2_5 cycles on the RTL")
def test_iolink_master_cyclic_type_2_1_fast(backend):
    """TYPE_2_1 (one PD in octet, Table A.10) every 2.3 ms scaled to the
    test rate (88 bit times): the cycle on the line exact to the slot grid,
    the device's PD in reported, each reply's checksum good."""
    bench, loom, line, dev = woken(backend, "FAST", n_in=1)
    T = tbit("FAST")
    cmd = mseq(mc(READ, CH_DIAGNOSIS, 0), n_in=1, type2=True)
    dev.pd_in = [0x5A]
    # The host queues the commands ahead of their cycles (INQ holds four):
    # a command that arrives after its cycle is due starts late.
    loom.push(0, [OP_CYCLE, 88 * 16, cmd, cmd | CYC])
    loom.push(0, [cmd | CYC])
    results = []
    for i in range(3):
        status = loom.pop(0, 1, max_polls=20000)[0]
        results.append((status, loom.pop(0, (status >> 4) & 15, max_polls=200)))
    for status, got in results:
        assert status & 0xFF == 3 << 4, hex(status)
        assert got == device_reply(0, [0x5A])
    starts = [m.chars[0].start for m in dev.messages[-3:]]
    for a, b in zip(starts, starts[1:]):
        assert abs((b - a) - 88 * T) <= SLOT, (b - a)
    assert loom.badop() == 0


@model_only("the PD out and write forms; the together test runs TYPE_2_5 on the RTL")
def test_iolink_master_pd_out_and_writes_fast(backend):
    """TYPE_2_3 and TYPE_2_4 carry PD out (one and two octets), TYPE_2_5
    one each way; a TYPE_0 write of MasterCycleTime goes into the model
    device's page and gets a reply of CKS alone."""
    for n_out, n_in, pd in ((1, 0, 0x3C00), (2, 0, 0xBEEF), (1, 1, 0x7100)):
        bench, loom, line, dev = woken(backend, "FAST", n_in=n_in, n_out=n_out)
        dev.pd_in = [0x42] * n_in
        status, got = master_run(loom, mseq(mc(READ, CH_DIAGNOSIS, 1), n_in=n_in,
                                            n_out=n_out, type2=True), [pd])
        octets = [pd >> 8, pd & 0xFF][:n_out]
        assert dev.pd_out_log == [octets]
        assert status & 0xFF == (2 + n_in) << 4 and got == device_reply(0, [0x42] * n_in)
        assert dev.messages[-1].octets == master_message(0x80 | 0x40 | 1, TYPE_2, octets)
    status, got = master_run(loom, mc(WRITE, CH_PAGE, 0x01), [0x17])
    assert status & 0xFF == 1 << 4 and got == device_reply()
    assert dev.page[0x01] == 0x17
    assert dev.messages[-1].octets == master_message(0x21, TYPE_0, od=0x17)


@model_only("error injection, each a full M-sequence; the RTL runs the good paths")
def test_iolink_master_reports_errors_fast(backend):
    """A bad CKS, a device that stays silent, one that replies after 12 bit
    times (tA <= 10, A.3.5, plus the master's one-bit allowance for the
    PHYs), a parity error, and a t2 of 5 bit times (t2 <= 3, A.3.4)."""
    bench, loom, line, dev = woken(backend, "FAST", n_in=1)
    cmd = mseq(mc(READ, CH_PAGE, VENDOR_ID), n_in=1, type2=True)
    dev.spoil = 1
    status, got = master_run(loom, cmd)
    assert status & 0xF == CKERR and len(got) == 3
    dev.silent = True
    status, got = master_run(loom, cmd)
    assert status & 0xFFFF == NORESP and got == []
    dev.silent, dev.ta_bits = False, 12
    status, got = master_run(loom, cmd)
    assert status & 0xFF == NORESP and got == []
    bench.step(int(60 * tbit("FAST")))  # the late reply goes by
    dev.ta_bits, dev.parity_error = 2.0, (1,)
    status, got = master_run(loom, cmd)
    assert status & 0xF == PARERR and len(got) == 3
    dev.t2_bits = 5
    status, got = master_run(loom, cmd)
    assert status & 0xFF == (1 << 4) | NORESP and got == [PAGE[VENDOR_ID]]
    bench.step(int(60 * tbit("FAST")))
    dev.t2_bits = 3                     # the limit: still read
    status, got = master_run(loom, cmd)
    assert status & 0xFF == 3 << 4


@model_only("COM2: about 60,000 clocks an M-sequence")
@pytest.mark.parametrize("phy_delay", [0, 50])
def test_iolink_master_at_com2(backend, phy_delay):
    """At COM2 (38.4 kbit/s at 50 MHz), with no PHY delay and with 1 us
    each way: TWU 75 to 85 us, TREN and TDMT, the bit edges within the
    master's +-0.1 % (Table 9), MinCycleTime, VendorID and DeviceID read
    at Tinitcyc, then TYPE_2_1 at the 2.3 ms cycle of Table A.11."""
    bench, loom, line, dev = master_setup(backend, "COM2", phy_delay=phy_delay, n_in=1)
    T = tbit("COM2")
    assert abs(T - TBIT_COM2) / TBIT_COM2 < 0.001
    loom.push(0, [OP_CYCLE, TINITCYC_BITS * 16, OP_WAKE | W_TICKS])
    status, got = master_run(loom, mc(READ, CH_PAGE, MIN_CYCLE_TIME))
    assert status & 0xFF == 2 << 4 and got == device_reply(PAGE[MIN_CYCLE_TIME])
    assert_wakeup(line, dev, T, "COM2")
    wake_end = sum(line.low_runs["chip0"][0])
    assert_master_chars(line, wake_end, T, [0xA2, 0x00])
    ids = []
    for addr in range(VENDOR_ID, DEVICE_ID + 3):
        status, got = master_run(loom, mseq(mc(READ, CH_PAGE, addr), cyc=True))
        assert status & 0xFF == 2 << 4
        ids.append(got[0])
    assert ids == PAGE[VENDOR_ID:DEVICE_ID + 3]
    cycle = round(2.3e-3 * CLOCK / tick("COM2"))       # 2.3 ms in ticks
    cmd = mseq(mc(READ, CH_DIAGNOSIS, 0), n_in=1, type2=True)
    dev.pd_in = [0x81]
    loom.push(0, [OP_CYCLE, cycle, cmd, cmd | CYC])
    for _ in range(2):
        status = loom.pop(0, 1, max_polls=40000)[0]
        assert status & 0xFF == 3 << 4
        assert loom.pop(0, 3, max_polls=200) == device_reply(0, [0x81])
    starts = [m.chars[0].start for m in dev.messages[-2:]]
    assert abs((starts[1] - starts[0]) * 1e3 / CLOCK - 2.3) < 0.002
    assert loom.badop() == 0


# ------------------------------------------------------------------ device
def device_setup(backend, rate, *, n_in=0, n_out=0, phy_delay=0, page=PAGE):
    """The chip device on thread 2 at ``rate`` with a model master on the
    line, configured and asleep; returns ``(bench, loom, line, master)``."""
    T = tbit(rate)
    bench = bench_for(backend)
    master = MasterNode(T)
    line = bench.add(IoLinkLine(T, [master], ports=[("OUT2", "OUT3", "IN1")],
                                phy_delay=phy_delay))
    loom = backend.loom(bench, isa=ISA)
    loom.load(device_image(page))
    loom.push(2, [n_in | (n_out << 2), WMIN, WMAX])
    start_thread(loom, 2, rate)
    bench.step(64)
    return bench, loom, line, master


def wake_device(bench, loom, master, rate, ticks=W_TICKS):
    """The model master's wake-up of ``ticks`` ticks; returns the device's
    report word, then waits TDMT (32 bit times)."""
    master.wake(bench.cycle + 8, ticks * tick(rate))
    word = loom.pop(2, 1, max_polls=2000)[0]
    bench.step(int(32 * tbit(rate)))
    return word


def exchange(bench, master, rate, mc_octet, mtype=TYPE_0, pd_out=(), od=None, n_in=0):
    """One M-sequence from the model master; waits for the whole reply."""
    T = tbit(rate)
    ex = master.send(bench.cycle + 8, mc_octet, mtype, pd_out, od, n_in)
    bench.run_until(lambda: ex.complete, int(70 * T), every=16)
    bench.step(int(4 * T))
    return ex


def assert_device_reply(line, ex, T, expect):
    """The chip device's reply on the line against Annex A.3: the octets
    ``expect``, good parity and stop bits, its edges on its bit grid (the
    slot grid only: the chip's tick is exact), t2 <= 3 bit times and tA
    within 1 to 10 bit times. The program's tA is 5.5 bit times from its
    own sample of the last stop bit, which lags the line by the latency of
    its start-bit detection: up to 64 clocks after a CKT (its header)."""
    assert ex.octets == list(expect), [hex(o) for o in ex.octets]
    assert ex.ok
    for c in ex.reply:
        assert all(abs(off) <= SLOT for off in edge_offsets(c, T)), edge_offsets(c, T)
    assert all(-SLOT / T <= g <= TA_BITS[0] for g in gaps_bits(ex.reply, T))
    assert TA_BITS[0] * T <= ex.ta_clocks <= TA_BITS[1] * T, ex.ta_clocks / T
    assert 5.5 * T - 2 * SLOT <= ex.ta_clocks <= 5.5 * T + 64, ex.ta_clocks / T


def test_iolink_device_wakes_and_serves_the_page_fast(backend):
    """The quick one: a wake-up of 49 ticks; MinCycleTime, VendorID and
    DeviceID read from the page the image holds; a write of
    MasterCycleTime read back; every reply timed on the line."""
    bench, loom, line, master = device_setup(backend, "FAST")
    T = tbit("FAST")
    word = wake_device(bench, loom, master, "FAST")
    assert word >> 15 == 1 and abs((word & 0x7FFF) - W_TICKS) <= 1, hex(word)
    for addr in (MIN_CYCLE_TIME, VENDOR_ID, VENDOR_ID + 1, DEVICE_ID, DEVICE_ID + 2):
        ex = exchange(bench, master, "FAST", mc(READ, CH_PAGE, addr))
        assert_device_reply(line, ex, T, device_reply(PAGE[addr]))
        assert loom.pop(2, 1) == [D_REPLIED | mc(READ, CH_PAGE, addr)]
    ex = exchange(bench, master, "FAST", mc(WRITE, CH_PAGE, 0x01), od=0x2A)
    assert_device_reply(line, ex, T, device_reply())
    assert loom.pop(2, 2) == [D_REPLIED | 0x21, 0x2A]
    ex = exchange(bench, master, "FAST", mc(READ, CH_PAGE, 0x01))
    assert_device_reply(line, ex, T, device_reply(0x2A))
    loom.pop(2, 1)
    ex = exchange(bench, master, "FAST", mc(READ, CH_PAGE, 0x12))
    assert_device_reply(line, ex, T, device_reply(0))     # page 2: not implemented
    loom.pop(2, 1)
    assert loom.badop() == 0


def test_iolink_device_type_2_5_process_data_fast(backend):
    """TYPE_2_5 (one PD octet each way, Table A.10): the PD out octet the
    master sends goes to OUTQ, the PD in words the host queues go out one
    per cycle, and the last one repeats when none is queued."""
    bench, loom, line, master = device_setup(backend, "FAST", n_in=1, n_out=1)
    T = tbit("FAST")
    wake_device(bench, loom, master, "FAST")
    loom.push(2, [0x6600, 0x9900])
    cmd = mc(READ, CH_DIAGNOSIS, 0)
    for pd_out, pd_in in ((0x11, 0x66), (0x22, 0x99), (0x33, 0x99)):
        ex = exchange(bench, master, "FAST", cmd, TYPE_2, [pd_out], n_in=1)
        assert_device_reply(line, ex, T, device_reply(0, [pd_in]))
        assert loom.pop(2, 2) == [D_REPLIED | cmd, pd_out << 8]
    assert loom.badop() == 0


@model_only("TYPE_2_2 and 2_4, the two-octet forms; the RTL runs TYPE_2_5")
def test_iolink_device_two_octet_process_data_fast(backend):
    """TYPE_2_2 (two PD in octets) and TYPE_2_4 (two PD out), PD0 first on
    the line, as the host's word has it in bits 15:8."""
    bench, loom, line, master = device_setup(backend, "FAST", n_in=2)
    T = tbit("FAST")
    wake_device(bench, loom, master, "FAST")
    loom.push(2, [0xC0DE])
    ex = exchange(bench, master, "FAST", mc(READ, CH_PAGE, VENDOR_ID), TYPE_2, n_in=2)
    assert_device_reply(line, ex, T, device_reply(PAGE[VENDOR_ID], [0xC0, 0xDE]))
    assert loom.pop(2, 1) == [D_REPLIED | mc(READ, CH_PAGE, VENDOR_ID)]
    bench, loom, line, master = device_setup(backend, "FAST", n_out=2)
    wake_device(bench, loom, master, "FAST")
    ex = exchange(bench, master, "FAST", mc(WRITE, CH_PAGE, 0x00), TYPE_2, [0xAB, 0xCD], od=0x99)
    assert_device_reply(line, ex, T, device_reply())
    assert loom.pop(2, 3) == [D_REPLIED | 0x20, 0xABCD, 0x99]


@model_only("error injection, one M-sequence each")
def test_iolink_device_rejects_bad_messages_fast(backend):
    """No reply, and the error in the status word, for a bad checksum, a
    parity error, a message cut short, an M-sequence type of 1; a 0 too
    short or too long for a wake-up is not one (wmin, wmax)."""
    bench, loom, line, master = device_setup(backend, "FAST")
    T = tbit("FAST")
    for ticks in (WMIN - 4, WMAX + 8):
        master.wake(bench.cycle + 8, ticks * tick("FAST"))
        bench.step(int((ticks + 40) * tick("FAST")))
        assert loom.pop_available(2) == []
    wake_device(bench, loom, master, "FAST")
    cmd = mc(READ, CH_PAGE, VENDOR_ID)
    ex = master.send(bench.cycle + 8, cmd, TYPE_0, spoil=True)
    bench.step(int(40 * T))
    assert ex.reply == [] and loom.pop(2, 1) == [D_CKERR | cmd]
    sent = master_message(cmd, TYPE_0)
    end = master.driver.send_chars(bench.cycle + 8, T, sent, parity_error=(1,))
    bench.step(int(end - bench.cycle + 30 * T))
    assert loom.pop(2, 1) == [D_PARERR | cmd]
    master.driver.send_chars(bench.cycle + 8, T, sent[:1])
    bench.step(int(40 * T))
    assert loom.pop(2, 1) == [D_SHORT | cmd]
    master.driver.send_chars(bench.cycle + 8, T, master_message(cmd, 1))
    bench.step(int(40 * T))
    assert loom.pop(2, 1)[0] & D_BADTYPE
    ex = exchange(bench, master, "FAST", cmd)       # and it still answers
    assert_device_reply(line, ex, T, device_reply(PAGE[VENDOR_ID]))


# ---------------------------------------------------------------- together
def test_iolink_master_and_device_together_fast(backend):
    """Both programs in one image, the master on thread 0 and the device on
    thread 2, each with its own pins and the line between them: the
    wake-up, the startup reads of MinCycleTime and VendorID at Tinitcyc,
    then TYPE_2_5 cycles with PD both ways every 80 bit times (the
    M-sequence takes about 62) and the cycle time on the line."""
    master, device = master_image().words, device_image()
    assert not set(master) & set(device)            # they load together
    T = tbit("FAST")
    bench = bench_for(backend)
    line = bench.add(IoLinkLine(T, [], ports=[("OUT0", "OUT1", "IN0"),
                                              ("OUT2", "OUT3", "IN1")], phy_delay=3))
    loom = backend.loom(bench, isa=ISA)
    loom.load({**master, **device})
    loom.push(2, [1 | (1 << 2), WMIN, WMAX, 0x4400])
    start_thread(loom, 2, "FAST")
    start_thread(loom, 0, "FAST")
    bench.step(64)
    loom.push(0, [OP_CYCLE, TINITCYC_BITS * 16, OP_WAKE | W_TICKS])
    status, got = master_run(loom, mc(READ, CH_PAGE, MIN_CYCLE_TIME))
    assert status & 0xFF == 2 << 4 and got == device_reply(PAGE[MIN_CYCLE_TIME])
    assert TA_BITS[0] * 16 <= status >> 8 <= TA_BITS[1] * 16, status >> 8
    word = loom.pop(2, 1)[0]
    assert word >> 15 == 1 and abs((word & 0x7FFF) - W_TICKS) <= 1
    assert loom.pop(2, 1) == [D_REPLIED | 0xA2]
    status, got = master_run(loom, mseq(mc(READ, CH_PAGE, VENDOR_ID), cyc=True))
    assert status & 0xFF == 2 << 4 and got == device_reply(PAGE[VENDOR_ID])
    assert loom.pop(2, 1) == [D_REPLIED | mc(READ, CH_PAGE, VENDOR_ID)]
    cmd = mseq(mc(READ, CH_DIAGNOSIS, 0), n_in=1, n_out=1, type2=True)
    loom.push(0, [OP_CYCLE, 80 * 16, cmd, 0x1100, cmd | CYC])
    loom.push(0, [0x2200])
    loom.push(2, [0x5500])
    for pd_out, pd_in in ((0x11, 0x44), (0x22, 0x55)):
        status = loom.pop(0, 1, max_polls=20000)[0]
        assert status & 0xFF == 3 << 4, hex(status)
        assert loom.pop(0, 3) == device_reply(0, [pd_in])
        assert loom.pop(2, 2) == [D_REPLIED | 0xC0, pd_out << 8]
    msgs = [c for c in line.rx["chip0"].chars if c.value == 0xC0]
    assert abs((msgs[1].start - msgs[0].start) - 80 * T) <= SLOT
    assert not line.contentions
    assert loom.badop() == 0
