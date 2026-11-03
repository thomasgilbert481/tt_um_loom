"""L3-MODBUS on the model and the RTL: firmware/modbus_server.loom and modbus_client.loom.

The server (thread 0: RX = IN0, TX = OUT0, DE = OUT1) answers requests that
``tools.protomodels.modbus.RtuTx`` sends; ``RtuRx`` decodes and times its
responses and DE, and ``ReferenceServer`` says what each response must be.
The client (thread 3: RX = IN1, TX = OUT2, DE = OUT3) sends what the host
gives it to a ``ModelServer``. Together, client and server share an
``Rs485Bus`` the bench wires between their pins.

Expected values come from the Modbus specifications through the reference
model (CRC, framing, the silences, the function codes and exceptions) and
from the program headers (settings, report and status formats), never from
what the model or the RTL did.

Rates: the RTL runs a test rate of 64 clocks a bit (a tick of 8 clocks,
the fastest the programs are proved for, 781250 baud at 50 MHz), with the
silences of SL 2.5.1.1's character rule (1.5 and 3.5 characters), not its
fixed 750 us and 1750 us for rates above 19200: those are about 600 and
1400 characters at this rate and would only make the scenarios longer. The
model also runs 9600 and 19200 baud, where the character rule is SL's own.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import modbus as mb
from tools.protomodels.modbus import (EVEN, NONE, ODD, ModelServer, ReferenceServer,
                                      Rs485Bus, RtuRx, RtuTx, bad_crc, encode_frame)
from tools.tests.fw_backend import model_only

ISA = load()
SERVER = REPO / "firmware" / "modbus_server.loom"
CLIENT = REPO / "firmware" / "modbus_client.loom"
IMEM = 512
OVS = 8                             # ticks a bit (both headers)

# The program headers: settings and data memory.
MASK = {EVEN: 0x0FF, ODD: 0x1FF, NONE: 0x100}
S_REGS, S_CFG, C_CFG = 0x118, 0x151, 0x155
# Server report bits.
R_BCAST, R_CRC, R_CHAR, R_GAP, R_OTHER = 1 << 11, 1 << 12, 1 << 13, 1 << 14, 1 << 15
# Client status bits.
C_STATUS, C_TMO, C_CRC, C_FERR, C_GAP, C_BCAST = (1 << 15, 1 << 8, 1 << 9,
                                                 1 << 10, 1 << 11, 1 << 13)


class Rate:
    """A bit rate: TICK_INT/TICK_FRAC for eight ticks a bit, and the
    silences in clocks and in the ticks the settings take."""

    def __init__(self, baud, fixed_timers=True):
        self.baud = baud
        self.bit = mb.bit_clocks(baud)
        tick = self.bit / OVS
        self.tick_int = int(tick)
        self.tick_frac = int(round((tick - self.tick_int) * 256))
        self.tick = self.tick_int + self.tick_frac / 256
        if fixed_timers:
            self.t15, self.t35 = mb.silences(baud)
        else:                       # the character rule at any rate
            char = mb.CHAR_BITS * self.bit
            self.t15, self.t35 = 1.5 * char, 3.5 * char
        self.T15 = int(round(self.t15 / self.tick))
        self.T35 = int(round(self.t35 / self.tick))
        self.char = mb.CHAR_BITS * self.bit
        # Edges land on the 4-clock slot grid, 0 to 3 clocks late (SEMANTICS
        # 2), and fractional ticks round: 3 clocks and 1 % of a bit.
        self.edge_tol = 3 + 0.01 * self.bit


FAST = Rate(50e6 / 64, fixed_timers=False)     # the RTL test rate
B19200 = Rate(19200)
B9600 = Rate(9600)


def mb_bench(backend):
    """The FIFOs, the bit engine, LD/ST and the 512-word memory on the
    model; the RTL has all of them and ``RtlBench`` takes neither keyword."""
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC", "DMEM"),
                             imem_words=IMEM)
    return backend.bench()


def program(path):
    prog = assemble_text(path.read_text(encoding="utf-8"), path.name, isa=ISA,
                         imem_words=IMEM, strict=True)
    assert prog.diagnostics == []
    return prog


def server_data(rate, parity, address, registers):
    data = {S_CFG: MASK[parity], S_CFG + 1: rate.T15, S_CFG + 2: rate.T35,
            S_CFG + 3: address}
    data.update({S_REGS + i: v for i, v in enumerate(registers)})
    return data


def client_data(rate, parity, timeout_ticks):
    return {C_CFG: MASK[parity], C_CFG + 1: rate.T15, C_CFG + 2: rate.T35,
            C_CFG + 3: timeout_ticks}


def boot(backend, bench, rate, threads, data):
    """Load the programs of ``threads`` ({thread: program}) and ``data``,
    set each thread's tick to an eighth of a bit, run them."""
    words = {}
    for prog in threads.values():
        assert not set(words) & set(prog.words), "the programs overlap"
        words.update(prog.words)
    assert not set(words) & set(data), "data on code"
    words.update(data)
    loom = backend.loom(bench, isa=ISA)
    loom.load(words)
    for t in threads:
        loom.write_csr(t, "TICK_INT", rate.tick_int)
        loom.write_csr(t, "TICK_FRAC", rate.tick_frac)
    loom.run(sorted(threads))
    return loom


# ------------------------------------------------------------------ server
ADDR = 17
TABLE = [(0x1111 * i + 0x0102) & 0xFFFF for i in range(16)]


class ServerRig:
    """The server on thread 0, a model client on IN0, a decoder on OUT0
    with DE on OUT1, and the reference server beside it."""

    def __init__(self, backend, rate=FAST, parity=EVEN, address=ADDR,
                 registers=TABLE, client_bit=None):
        self.rate, self.parity = rate, parity
        self.bench = mb_bench(backend)
        self.client = self.bench.add(RtuTx("IN0", client_bit or rate.bit, parity))
        self.rx = self.bench.add(RtuRx("OUT0", rate.bit, de="OUT1"))
        self.ref = ReferenceServer(address, registers=list(registers))
        self.got = []
        self.loom = boot(backend, self.bench, rate, {0: program(SERVER)},
                         server_data(rate, parity, address, registers))
        self.bench.step(int(rate.t35 + 4 * rate.bit))   # SL figure 14's power-up silence

    def transact(self, frame, expected="reference", **inject):
        """Send ``frame`` and run until any response is over. Returns the
        response frames decoded on OUT0 (a list) and the request's end."""
        self.client.send(frame, **inject)
        end = self.client.free_at
        if expected == "reference":
            expected = self.ref.handle(frame)
        tail = self.rate.t35 + 2000 + (len(expected or []) + 1) * self.rate.char
        self.bench.step(int(end - self.bench.cycle + tail))
        self.got += self.loom.pop_available(0)     # OUTQ holds four
        frames = self.rx.frames(self.rate.t35, since=end)
        if expected is None:
            assert frames == [], [f.data for f in frames]
            return None
        (reply,) = frames
        assert reply.data == expected, (reply.data, expected)
        self.check_timing(reply, end)
        return reply

    def check_timing(self, reply, request_end):
        """SL 2.5.1 and 2.5.1.1 on the response, the header's DE window."""
        r = self.rate
        assert self.rx.violations(reply, self.parity, r.t15, r.t35, r.edge_tol) == []
        assert reply.gap_max <= r.edge_tol             # back to back (header)
        assert reply.start - request_end >= r.t35      # the request's frame end first
        rise, fall = self.rx.de_window(reply)
        assert abs((reply.start - rise) - r.bit) <= r.edge_tol, (rise, reply.start)
        assert abs(fall - reply.end) <= r.edge_tol, (fall, reply.end)

    def reports(self, count):
        """Every report so far, ``count`` of them."""
        more = count - len(self.got)
        got, self.got = self.got + (self.loom.pop(0, more, max_polls=2000) if more > 0 else []), []
        return got


def req(pdu, address=ADDR):
    return encode_frame(address, pdu)


def report(fc, exc=0, flags=0):
    return fc | (exc << 8) | flags


def test_server_answers_read_write_and_write_multiple(backend):
    """03, 06, 16 and 03 again, each response what AP says and timed to SL;
    the table read back from data memory at the end."""
    rig = ServerRig(backend)
    rig.transact(req(mb.read_holding(0, 3)))
    rig.transact(req(mb.write_single(5, 0xBEEF)))
    rig.transact(req(mb.write_multiple(9, [0x0A0B, 0x0C0D, 0xFFFF])))
    rig.transact(req(mb.read_holding(4, 9)))
    assert rig.reports(4) == [report(3), report(6), report(16), report(3)]
    rig.loom.halt([0])
    assert rig.loom.read_imem(S_REGS, 16) == rig.ref.registers
    assert rig.loom.badop() == 0


@model_only("seven requests, 189 k clocks; the RTL time goes to the gap, error and bus scenarios")
def test_server_exceptions_broadcast_and_other_addresses(backend):
    """AP 7's exceptions 01, 02 and 03; a broadcast write executed and not
    answered (SL 2.1); a request for another server ignored."""
    rig = ServerRig(backend)
    rig.transact(req([0x04, 0, 0, 0, 1]))                     # 01: not supported
    rig.transact(req(mb.read_holding(14, 3)))                 # 02: past the table
    rig.transact(req(mb.read_holding(0, 0)))                  # 03: quantity 0
    rig.transact(req(mb.write_multiple(0, [1, 2], byte_count=3)))  # 03: byte count
    rig.transact(req(mb.write_single(7, 0x1234), address=0))  # broadcast
    rig.transact(req(mb.write_single(8, 0x5678), address=ADDR + 1))
    rig.transact(req(mb.read_holding(7, 2)))                  # 7 written, 8 not
    assert rig.reports(7) == [report(4, 1), report(3, 2), report(3, 3), report(16, 3),
                              report(6, 0, R_BCAST), report(6, 0, R_OTHER), report(3)]
    assert rig.loom.badop() == 0


def test_server_drops_a_frame_with_a_gap_inside(backend):
    """A silence of two characters after the third (over t1.5, under t3.5):
    SL figure 14 makes the frame NOK, so nothing is answered; the next
    request, a whole t3.5 later, is."""
    rig = ServerRig(backend)
    rig.transact(req(mb.read_holding(0, 1)), expected=None, gaps={2: 2 * rig.rate.char})
    rig.transact(req(mb.read_holding(1, 1)))
    assert rig.reports(2) == [report(0, 0, R_GAP), report(3)]


@model_only("the boundary on both sides; the two-character gap runs on the RTL")
@pytest.mark.parametrize("side", [-2, +2])
def test_server_t15_boundary(backend, side):
    """A silence two ticks under t1.5 inside a frame is fine, two ticks
    over it spoils the frame (the header: right to within a tick)."""
    rig = ServerRig(backend)
    gap = rig.rate.t15 + side * rig.rate.tick
    if side < 0:
        rig.transact(req(mb.read_holding(0, 1)), gaps={4: gap})
        assert rig.reports(1) == [report(3)]
    else:
        rig.transact(req(mb.read_holding(0, 1)), expected=None, gaps={4: gap})
        assert rig.reports(1) == [report(0, 0, R_GAP)]


def test_server_drops_crc_and_parity_errors(backend):
    """A frame with a bad CRC, and one whose fourth character has a wrong
    parity bit: dropped and reported; then a good one answered."""
    rig = ServerRig(backend)
    rig.transact(bad_crc(req(mb.read_holding(0, 1))), expected=None)
    rig.transact(req(mb.read_holding(0, 1)), expected=None, bad_parity=[3])
    rig.transact(req(mb.read_holding(2, 1)))
    assert rig.reports(3) == [report(0, 0, R_CRC), report(0, 0, R_CHAR), report(3)]


@model_only("9600 and 19200 baud are 5208 and 2604 clocks a bit; the RTL runs 64")
@pytest.mark.parametrize("baud, parity", [(9600, EVEN), (19200, EVEN), (19200, ODD),
                                          (19200, NONE)])
def test_server_at_9600_and_19200_baud(backend, baud, parity):
    """SL's own rates and its three character formats, with the silences of
    SL 2.5.1.1's character rule (132 and 308 ticks)."""
    rate = {9600: B9600, 19200: B19200}[baud]
    assert (rate.T15, rate.T35) == (132, 308)
    rig = ServerRig(backend, rate=rate, parity=parity)
    rig.transact(req(mb.read_holding(6, 2)))
    assert rig.reports(1) == [report(3)]


@model_only("rate error corners; the nominal rate runs on the RTL")
@pytest.mark.parametrize("error", [-0.03, +0.03])
def test_server_takes_a_client_three_percent_off(backend, error):
    """A client 3 % fast or slow at 19200 baud, back to back: understood
    (the header's +-4 % claim, with margin) and answered at the nominal rate."""
    rig = ServerRig(backend, rate=B19200, client_bit=B19200.bit * (1 + error))
    rig.transact(req(mb.read_holding(3, 4)))
    assert rig.reports(1) == [report(3)]


@model_only("long frames: a 41-byte request is 29 k clocks at the test rate")
def test_server_limits_of_the_table_and_the_quantities(backend):
    """All 16 registers written and read in one request each; 17 is past the
    table (02), and AP's quantity limits, 123 for 16 and 125 for 03, give
    03 one past them; a frame too short for its function gives 03."""
    rig = ServerRig(backend)
    values = [(0x0F0F * i) & 0xFFFF for i in range(16)]
    rig.transact(req(mb.write_multiple(0, values)))
    rig.transact(req(mb.read_holding(0, 16)))
    rig.transact(req(mb.read_holding(0, 17)))                     # 02
    rig.transact(req(mb.read_holding(0, 126)))                    # 03
    rig.transact(req(mb.write_multiple(15, [1], quantity=124, byte_count=248)))  # 03
    rig.transact(req(mb.write_multiple(15, [1], quantity=123, byte_count=246)))  # 03: length
    rig.transact(req([0x03, 0x00, 0x00]))                         # 03: 6 bytes
    assert rig.reports(7) == [report(16), report(3), report(3, 2), report(3, 3),
                              report(16, 3), report(16, 3), report(3, 3)]
    assert rig.ref.registers[:16] == values


# ------------------------------------------------------------------ client
def collect(bench, loom, thread, step, max_cycles):
    """Pop OUTQ[thread] as the bytes come (it holds four) until a status
    word (bit 15) arrives; ``(bytes, status, cycle)``."""
    words, spent = [], 0
    while spent < max_cycles:
        words += loom.pop_available(thread)
        if any(w & C_STATUS for w in words):
            break
        bench.step(step)
        spent += step
    status = [w for w in words if w & C_STATUS]
    assert len(status) == 1 and words[-1] == status[0], [hex(w) for w in words]
    return words[:-1], status[0], bench.cycle


class ClientRig:
    """The client on thread 3, a ModelServer on its pins, and a decoder
    that times the requests on OUT2 with DE on OUT3."""

    def __init__(self, backend, rate=FAST, parity=EVEN, timeout=None):
        self.rate, self.parity = rate, parity
        self.bench = mb_bench(backend)
        self.wire = self.bench.add(RtuRx("OUT2", rate.bit, de="OUT3"))
        self.server = self.bench.add(ModelServer("OUT2", "IN1", rate.bit,
                                                 ReferenceServer(ADDR, registers=TABLE),
                                                 rate.t35, parity=parity))
        self.timeout = timeout or 2 * rate.T35
        self.loom = boot(backend, self.bench, rate, {3: program(CLIENT)},
                         client_data(rate, parity, self.timeout))
        self.bench.step(200)

    def ask(self, body):
        """Send ``body`` (address, function, data) through the client; the
        response bytes, the status word, the request frame on the wire."""
        start = self.bench.cycle
        self.loom.push(3, [len(body)] + list(body))
        limit = int((len(body) + 45) * self.rate.char + 2 * self.rate.t35
                    + self.timeout * self.rate.tick)
        data, status, _ = collect(self.bench, self.loom, 3, int(self.rate.char / 2), limit)
        (frame,) = self.wire.frames(self.rate.t35, since=start)
        r = self.rate
        assert frame.data == encode_frame(body[0], body[1:])      # the CRC appended
        assert self.wire.violations(frame, self.parity, r.t15, r.t35, r.edge_tol) == []
        assert frame.gap_max <= r.edge_tol
        rise, fall = self.wire.de_window(frame)
        assert abs((frame.start - rise) - r.bit) <= r.edge_tol
        assert abs(fall - frame.end) <= r.edge_tol
        return data, status, frame


def good(reply):
    return C_STATUS | len(reply)


def test_client_reads_and_writes_through_a_model_server(backend):
    """03 and 16 through a server model: each request framed and timed to
    SL, each response handed over byte by byte with a good status."""
    rig = ClientRig(backend)
    ref = ReferenceServer(ADDR, registers=TABLE)
    for body in ([ADDR] + mb.read_holding(2, 3),
                 [ADDR] + mb.write_multiple(0, [0x0123, 0x4567]),
                 [ADDR] + mb.read_holding(0, 2)):
        expected = ref.handle(encode_frame(body[0], body[1:]))
        data, status, _ = rig.ask(body)
        assert (data, status) == (expected, good(expected))
    assert rig.loom.badop() == 0


def test_client_timeout_crc_error_and_broadcast(backend):
    """No response (the timeout, not sooner), a response with a bad CRC,
    then a broadcast, which is sent and not waited for."""
    rig = ClientRig(backend)
    rig.server.fault = "silent"
    data, status, frame = rig.ask([ADDR] + mb.read_holding(0, 1))
    assert (data, status) == ([], C_STATUS | C_TMO)
    assert rig.bench.cycle >= frame.end + rig.timeout * rig.rate.tick
    rig.bench.step(int(rig.rate.t35))         # SL: t3.5 after a timeout (the host's job)
    rig.server.fault = "bad_crc"
    data, status, _ = rig.ask([ADDR] + mb.read_holding(0, 1))
    assert status == C_STATUS | C_CRC | 7 and len(data) == 7
    rig.server.fault = None
    data, status, _ = rig.ask([0] + mb.write_single(4, 0xAAAA))
    assert (data, status) == ([], C_STATUS | C_BCAST)
    rig.bench.step(int(rig.rate.t35 + 2 * rig.rate.char))
    assert rig.server.server.registers[4] == 0xAAAA
    assert len(rig.server.requests) == 3


@model_only("two more faults; the timeout and CRC scenario runs on the RTL")
def test_client_flags_a_gap_and_ignores_parity(backend):
    """A response with a silence of two characters inside it (bit 11); one
    with a wrong parity bit, which the header says the client does not
    check: good, since its CRC is."""
    rig = ClientRig(backend)
    rig.server.fault, rig.server.gap = "gap", 2 * rig.rate.char
    data, status, _ = rig.ask([ADDR] + mb.read_holding(0, 1))
    assert status == C_STATUS | C_GAP | 7
    rig.server.fault = "bad_parity"
    expected = ReferenceServer(ADDR, registers=TABLE).handle(req(mb.read_holding(1, 2)))
    data, status, _ = rig.ask([ADDR] + mb.read_holding(1, 2))
    assert (data, status) == (expected, good(expected))


# ---------------------------------------------------------------- together
def test_client_and_server_on_one_bus(backend):
    """The server on thread 0 and the client on thread 3, an RS-485 bus
    between their pins (SL 3.3): the client reads, writes and reads back;
    every frame on the bus is timed, the drivers never overlap, and the
    server reports what it was asked."""
    rate = FAST
    bench = mb_bench(backend)
    bus = bench.add(Rs485Bus([("OUT0", "OUT1"), ("OUT2", "OUT3")], ["IN0", "IN1"]))
    on_bus = bench.add(RtuRx(bit=rate.bit, bus=bus))
    server_side = bench.add(RtuRx("OUT0", rate.bit, de="OUT1"))
    client_side = bench.add(RtuRx("OUT2", rate.bit, de="OUT3"))
    data = server_data(rate, EVEN, ADDR, TABLE)
    data.update(client_data(rate, EVEN, 2 * rate.T35))
    loom = boot(backend, bench, rate, {0: program(SERVER), 3: program(CLIENT)}, data)
    bench.step(int(rate.t35 + 4 * rate.bit))
    ref = ReferenceServer(ADDR, registers=TABLE)
    bodies = ([ADDR] + mb.read_holding(1, 2),
              [ADDR] + mb.write_single(2, 0xCAFE),
              [ADDR] + mb.read_holding(1, 2))
    start = bench.cycle
    for body in bodies:
        expected = ref.handle(encode_frame(body[0], body[1:]))
        loom.push(3, [len(body)] + body)
        got, status, _ = collect(bench, loom, 3, int(rate.char / 2), 60 * int(rate.char))
        assert (got, status) == (expected, good(expected))
    assert loom.pop(0, 3, max_polls=2000) == [report(3), report(6), report(3)]
    frames = on_bus.frames(rate.t35, since=start)
    assert len(frames) == 6
    for frame in frames:
        assert on_bus.violations(frame, EVEN, rate.t15, rate.t35, rate.edge_tol) == []
    for side in (server_side, client_side):
        for frame in side.frames(rate.t35, since=start):
            assert side.violations(frame, EVEN, rate.t15, rate.t35, rate.edge_tol) == []
    assert bus.contentions == []
    assert loom.badop() == 0
