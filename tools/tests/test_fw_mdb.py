"""L3-MDB on the model and the RTL: firmware/mdb_ctrl.loom and mdb_cashless.loom.

The controller (thread 0: RX = IN0, TX = OUT0) sends what the host gives it
to ``tools.protomodels.mdb.ModelCashless`` on its pins. The cashless device
(thread 2: RX = IN1, TX = OUT1) answers ``ModelController``. Together, the
bench wires OUT0 to IN1 and OUT1 to IN0, the controller's transmit line to
the peripheral's receive line and back, as MDB's two separate lines are.
``LineRx`` decoders on the chip outputs time every block against MDB/ICP
3.2 (t-inter-byte, t-response) and check the mode-bit rules of 2.1.

Expected values come from the specification through the reference model
(CHK, mode bits, ACK/NAK, the cashless responses, the timing) and from the
program headers (settings, status and event formats), never from what the
model or the RTL did.

Rates: the RTL runs 64 clocks a bit (TICK 8, the fastest the programs are
proved for), with every time of 3.2 scaled by bit times (t-response 5.0 ms
is 48 bits at 9600 baud, t-inter-byte 1.0 ms is 9.6 bits). The model also
runs 9600 baud itself (TICK 651 + 11/256).

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

import pytest

from tools.loomasm import assemble_text
from tools.loomisa import REPO, load
from tools.protomodels import mdb
from tools.protomodels.mdb import (ACK, MODE, NAK, LineRx, ModelCashless,
                                   ModelController, Timing, Wire, split_blocks,
                                   violations)
from tools.tests.fw_backend import model_only

ISA = load()
CTRL = REPO / "firmware" / "mdb_ctrl.loom"
DEV = REPO / "firmware" / "mdb_cashless.loom"
IMEM = 512
OVS = 8                             # ticks a bit (both headers)
C_CFG, D_CFG = 0x0C0, 0x1D0         # the settings (headers)
# Controller status (mdb_ctrl header).
S_MARK, S_TMO, S_CHK, S_NAK, S_FERR = 1 << 15, 1 << 8, 1 << 9, 1 << 10, 1 << 11
# Cashless events (mdb_cashless header).
E_RESET, E_VEND = 0x0010, 0x0013
DENY = 0x8000                       # the host's VEND DENIED decision


class Rate:
    """A bit rate: TICK_INT/TICK_FRAC for eight ticks a bit, and the times
    of 3.2 in clocks and in the ticks the settings take."""

    def __init__(self, bit):
        self.bit = bit
        tick = bit / OVS
        self.tick_int = int(tick)
        self.tick_frac = int(round((tick - self.tick_int) * 256))
        self.tick = self.tick_int + self.tick_frac / 256
        self.timing = Timing(bit)
        self.TRESP = int(round(self.timing.t_response / self.tick))
        self.TIB = int(round(self.timing.t_inter_byte / self.tick))
        self.GAP = 10 * OVS                  # 10 bits: over t-inter-byte (9.6 bits)
        # Edges land on the 4-clock slot grid, 0 to 3 clocks late (SEMANTICS
        # 2), and fractional ticks round: 3 clocks and 1 % of a bit.
        self.edge_tol = 3 + 0.01 * bit


FAST = Rate(64.0)                    # the RTL test rate
B9600 = Rate(mdb.CLOCK / mdb.BAUD)


def mdb_bench(backend):
    if backend.name == "model":
        return backend.bench(features=("FIFO", "BE", "BEENC", "DMEM"), imem_words=IMEM)
    return backend.bench()


def program(path):
    prog = assemble_text(path.read_text(encoding="utf-8"), path.name, isa=ISA,
                         imem_words=IMEM, strict=True)
    assert prog.diagnostics == []
    return prog


def ctrl_data(rate, tries=3):
    return {C_CFG: rate.TRESP, C_CFG + 1: rate.TIB, C_CFG + 2: tries}


def dev_data(rate, address=mdb.CASHLESS1):
    return {D_CFG: rate.GAP, D_CFG + 1: rate.TRESP, D_CFG + 2: address}


def boot(backend, bench, rate, threads, data):
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


def command(loom, bench, data, limit):
    """Give the controller one block; return (answer words, status)."""
    loom.push(0, [len(data)] + list(data))
    got = []
    spent = 0
    while not (got and got[-1] & S_MARK):
        assert spent < limit, ("no status", got)
        bench.step(256)
        spent += 256
        got += loom.pop_available(0)
    return got[:-1], got[-1]


def status(count, attempts, flags=0):
    return S_MARK | attempts << 12 | flags | count


def blocks_of(rx, rate, since=0, vmc=True):
    """Characters on a line, split at silences of t-inter-byte and, on the
    controller's line, before every ADDRESS byte (2.1)."""
    out = []
    for blk in split_blocks(rx.since(since), rate.timing.t_inter_byte):
        for ch in blk:
            if not out or out[-1][-1] is not blk[blk.index(ch) - 1] or (vmc and ch.mode):
                out.append([ch])
            else:
                out[-1].append(ch)
    return out


# -------------------------------------------------------------- controller
class CtrlRig:
    """mdb_ctrl on thread 0, a ModelCashless on its pins, and decoders on
    both lines: ``line`` the controller's TX (OUT0), ``ans`` the model's
    answers on its RX (IN0)."""

    def __init__(self, backend, rate=FAST, tries=3, dev_bit=None, **device):
        self.rate = rate
        self.bench = mdb_bench(backend)
        self.dev = self.bench.add(ModelCashless("OUT0", "IN0", dev_bit or rate.bit, **device))
        self.ans = self.bench.add(LineRx("IN0", rate.bit))
        self.line = self.dev.rx
        self.loom = boot(backend, self.bench, rate, {0: program(CTRL)},
                         ctrl_data(rate, tries))
        # TX is low from reset until the program sets it idle, which the
        # models read as a broken character: let them see 12 idle bits (a
        # scaled-down t-setup, 3.2) before the first block.
        self.bench.step(int(12 * rate.bit))
        self.t0 = self.bench.cycle
        self.limit = int(8 * tries * rate.timing.t_response)

    def do(self, data):
        answer, st = command(self.loom, self.bench, data, self.limit)
        self.bench.step(int(2 * self.rate.timing.char))   # a trailing ACK or NAK
        return answer, st

    def sent(self, since=None):
        """The controller's blocks since ``since``, each checked against
        2.1, 2.2 and 3.2 (stop bits, bit edges, gaps, mode bits)."""
        r = self.rate
        out = blocks_of(self.line, r, self.t0 if since is None else since)
        for blk in out:
            assert violations(blk, r.timing, "vmc", r.edge_tol) == [], [c.value for c in blk]
        return out

    def acks_in_time(self, since=None):
        """Every ACK or NAK the controller sends answers a data block that
        ended at most t-response before it (2.2, 3.2)."""
        for blk in self.sent(since):
            if len(blk) != 1:
                continue
            before = [c for c in self.ans.chars if c.end <= blk[0].start]
            assert before and before[-1].mode == 1
            assert blk[0].start - before[-1].end <= self.rate.timing.t_response


def test_mdb_ctrl_reset_poll_vend(backend):
    """RESET, POLL (JUST RESET, ACKed), VEND REQUEST, POLL (VEND APPROVED)
    against the reference device, every byte checked against 2.1, 2.2, 3.2."""
    rig = CtrlRig(backend)
    assert rig.do([0x10]) == ([MODE | ACK], status(1, 1))             # RESET
    jr = mdb.peripheral_block([mdb.JUST_RESET])
    assert rig.do([0x12]) == (jr, status(2, 1))                       # POLL
    assert rig.dev.acked == [jr]                                      # ACKed (2.2)
    req = mdb.vend_request(0x0123, 0x0007)
    assert rig.do(req) == ([MODE | ACK], status(1, 1))
    assert rig.dev.vends == [(0x0123, 0x0007)]
    ok = mdb.peripheral_block([mdb.VEND_APPROVED, 0x01, 0x23])
    assert rig.do([0x12]) == (ok, status(4, 1))
    assert rig.dev.acked == [jr, ok]
    assert [[c.value for c in b] for b in rig.sent()] == [
        mdb.vmc_block([0x10]), mdb.vmc_block([0x12]), [ACK],
        mdb.vmc_block(req), mdb.vmc_block([0x12]), [ACK]]
    rig.acks_in_time()


def test_mdb_ctrl_bad_chk_nak_retry(backend):
    """A JUST RESET whose CHK is wrong: the controller NAKs it (2.5 D) and
    repeats the POLL; the device repeats the block (2.2), which is ACKed."""
    rig = CtrlRig(backend)
    rig.dev.corrupt = 1
    jr = mdb.peripheral_block([mdb.JUST_RESET])
    answer, st = rig.do([0x12])
    assert answer == [jr[0], jr[1] ^ 0xFF] + jr and st == status(2, 2)
    assert [[c.value for c in b] for b in rig.sent()] == [
        mdb.vmc_block([0x12]), [NAK], mdb.vmc_block([0x12]), [ACK]]
    assert rig.dev.acked == [jr]
    rig.acks_in_time()


def test_mdb_ctrl_silence_retry(backend):
    """One POLL unanswered: the controller waits t-response from the CHK's
    end (2.2, 3.2), then repeats the block, which is answered."""
    rig = CtrlRig(backend)
    rig.dev.silent = 1
    answer, st = rig.do([0x10])
    assert answer == [MODE | ACK] and st == status(1, 2)
    first, second = rig.sent()
    assert [c.value for c in first] == [c.value for c in second] == mdb.vmc_block([0x10])
    wait = second[0].start - first[-1].end
    assert rig.rate.timing.t_response <= wait <= rig.rate.timing.t_response + 2 * rig.rate.bit


def test_mdb_ctrl_peripheral_too_slow(backend):
    """A peripheral that answers later than t-response: every answer is
    taken as a NAK (2.2), each of the three attempts starts after a full
    t-response of silence, and the host gets TMO."""
    rig = CtrlRig(backend, delay=FAST.timing.t_response + 3 * FAST.bit)
    answer, st = rig.do([0x12])
    assert answer == [] and st == status(0, 3, S_TMO)
    blocks = rig.sent()
    assert len(blocks) == 3
    for a, b in zip(blocks, blocks[1:]):
        assert b[0].start - a[-1].end >= rig.rate.timing.t_response
    assert len(rig.dev.blocks) == 3           # the device did hear them all


@model_only("a rate sweep; the RTL runs the nominal rate in the other scenarios")
@pytest.mark.parametrize("error", [+0.01, -0.02])
def test_mdb_ctrl_rate_error(backend, error):
    """A peripheral at the edges of 3.2's 9600 +1 %/-2 %, at the fastest
    tick: a four-byte VEND APPROVED back to back is still read and ACKed."""
    rig = CtrlRig(backend, dev_bit=FAST.bit / (1 + error))
    ok = mdb.peripheral_block([mdb.VEND_APPROVED, 0xAB, 0xCD])
    rig.dev.pending = ok
    assert rig.do([0x12]) == (ok, status(4, 1))
    assert rig.dev.acked == [ok]


@model_only("9600 baud is 5208 clocks a bit: about 0.4 M clocks for this exchange")
def test_mdb_ctrl_9600_baud(backend):
    """POLL and VEND REQUEST at the specification's own rate, 3.2's times
    unscaled (TRESP 384 ticks: 5.0 ms; TIB 77: 1.0 ms)."""
    rig = CtrlRig(backend, rate=B9600)
    assert (rig.rate.TRESP, rig.rate.TIB) == (384, 77)
    jr = mdb.peripheral_block([mdb.JUST_RESET])
    assert rig.do([0x12]) == (jr, status(2, 1))
    assert rig.do(mdb.vend_request(250, 3)) == ([MODE | ACK], status(1, 1))
    assert len(rig.sent()) == 3
    rig.acks_in_time()


# --------------------------------------------------------------- cashless
class DevRig:
    """mdb_cashless on thread 2 and a ModelController on its pins (the
    controller's TX on IN1, the device's TX decoded on OUT1)."""

    def __init__(self, backend, rate=FAST, tries=3, vmc_bit=None):
        self.rate = rate
        self.bench = mdb_bench(backend)
        self.vmc = self.bench.add(ModelController("IN1", "OUT1", vmc_bit or rate.bit,
                                                  tries=tries))
        self.line = self.vmc.rx
        self.loom = boot(backend, self.bench, rate, {2: program(DEV)}, dev_data(rate))
        self.bench.step(int(12 * rate.bit))       # see CtrlRig
        self.t0 = self.bench.cycle
        self.events = []

    def do(self, data):
        """One block from the model controller, to its end; returns the
        Exchange (outcome, attempts, the last answer)."""
        ex = self.vmc.queue(data)
        limit = int(8 * self.vmc.tries * self.rate.timing.t_response)
        assert self.bench.run_until(lambda: ex.done and not self.vmc.tx.busy, limit, every=64)
        self.bench.step(int(self.rate.timing.char))
        self.events += self.loom.pop_available(2)
        return ex

    def answers(self):
        """The device's blocks, each checked against 2.1, 2.2 and 3.2, and
        each starting within t-response of the end of the block it answers."""
        r = self.rate
        out = blocks_of(self.line, r, self.t0, vmc=False)
        ends = [e for ex in self.vmc.exchanges for e in ex.sent_end]
        for blk in out:
            assert violations(blk, r.timing, "peripheral", r.edge_tol) == [], [c.value for c in blk]
            before = [e for e in ends if e <= blk[0].start]
            assert before and blk[0].start - before[-1] <= r.timing.t_response
        return [[c.value for c in b] for b in out]


def test_mdb_cashless_reset_poll_vend(backend):
    """JUST RESET from power-up and after RESET (7.4.1), a VEND REQUEST
    ACKed and reported, POLL ACKed until the host decides, then VEND
    APPROVED with the host's amount (7.4.5); timing against 3.2."""
    rig = DevRig(backend)
    jr = mdb.peripheral_block([mdb.JUST_RESET])
    ex = rig.do([0x12])
    assert (ex.outcome, ex.answer) == ("data", jr)
    ex = rig.do([0x10])
    assert (ex.outcome, ex.answer) == ("ack", [MODE | ACK])
    assert rig.do([0x12]).answer == jr
    ex = rig.do(mdb.vend_request(0x0150, 0x0012))
    assert (ex.outcome, ex.answer) == ("ack", [MODE | ACK])
    assert rig.do([0x12]).outcome == "ack"            # not decided yet (7.4.4)
    rig.loom.push(2, [0x0140])
    ok = mdb.peripheral_block([mdb.VEND_APPROVED, 0x01, 0x40])
    ex = rig.do([0x12])
    assert (ex.outcome, ex.attempts, ex.answer) == ("data", 1, ok)
    assert rig.do([0x12]).outcome == "ack"            # delivered: nothing pending
    assert rig.events == [mdb.JUST_RESET, E_RESET, mdb.JUST_RESET,
                          E_VEND, 0x0150, 0x0012, mdb.VEND_APPROVED]
    assert rig.answers() == [jr, [MODE], jr, [MODE], [MODE], ok, [MODE]]


def test_mdb_cashless_denied_repeated_until_acked(backend):
    """VEND DENIED from the host; the controller withholds its ACK, so the
    device sends the block again at the next POLL (2.2) until it is ACKed."""
    rig = DevRig(backend)
    rig.do([0x12])                                    # JUST RESET, ACKed
    rig.do(mdb.vend_request(0x0099, 0xFFFF))
    rig.loom.push(2, [DENY])
    rig.vmc.withhold = 1
    no = mdb.peripheral_block([mdb.VEND_DENIED])
    assert rig.do([0x12]).answer == no
    assert rig.do([0x12]).answer == no
    assert rig.do([0x12]).outcome == "ack"
    assert rig.events == [mdb.JUST_RESET, E_VEND, 0x0099, 0xFFFF, mdb.VEND_DENIED]
    rig.answers()


def test_mdb_cashless_bad_chk_nak(backend):
    """A RESET with a wrong CHK is NAKed (2.2) and not acted on; the
    controller's repeat is ACKed."""
    rig = DevRig(backend)
    rig.vmc.corrupt = 1
    ex = rig.do([0x10])
    assert (ex.outcome, ex.attempts) == ("ack", 2)
    assert rig.events == [E_RESET]
    assert rig.answers() == [[MODE | NAK], [MODE]]


def test_mdb_cashless_other_address_ignored(backend):
    """A POLL to the changer (08H) is not answered (2.1: DATA bytes are for
    the peripheral addressed); one to 10H right after it is."""
    rig = DevRig(backend, tries=1)
    assert rig.do([0x0A]).outcome == "timeout"
    assert rig.do([0x12]).answer == mdb.peripheral_block([mdb.JUST_RESET])
    assert len(rig.answers()) == 1



@model_only("a rate sweep; the RTL runs the nominal rate in the other scenarios")
@pytest.mark.parametrize("error", [+0.01, -0.02])
def test_mdb_cashless_rate_error(backend, error):
    """A controller at the edges of 3.2's 9600 +1 %/-2 %: the device still
    reads its blocks and answers in time."""
    rig = DevRig(backend, vmc_bit=FAST.bit / (1 + error))
    ex = rig.do([0x12])
    assert (ex.outcome, ex.answer) == ("data", mdb.peripheral_block([mdb.JUST_RESET]))
    ex = rig.do(mdb.vend_request(0x1234, 0x0001))
    assert (ex.outcome, ex.attempts) == ("ack", 1)
    assert rig.events == [mdb.JUST_RESET, E_VEND, 0x1234, 0x0001]


# --------------------------------------------------------------- together
def test_mdb_together(backend):
    """mdb_ctrl and mdb_cashless in one image, the bench wiring the
    controller's TX to the device's RX and back: the reset sequence of
    7.4.1 (RESET, POLL for JUST RESET), a VEND REQUEST, a POLL before the
    host's decision and one after it; both lines checked against 2.1, 2.2
    and 3.2."""
    rate = FAST
    bench = mdb_bench(backend)
    bench.add(Wire("OUT0", "IN1"))
    bench.add(Wire("OUT1", "IN0"))
    vmc_line = bench.add(LineRx("OUT0", rate.bit))
    dev_line = bench.add(LineRx("OUT1", rate.bit))
    data = dict(ctrl_data(rate))
    data.update(dev_data(rate))
    loom = boot(backend, bench, rate, {0: program(CTRL), 2: program(DEV)}, data)
    bench.step(int(12 * rate.bit))
    t0 = bench.cycle
    limit = int(24 * rate.timing.t_response)
    jr = mdb.peripheral_block([mdb.JUST_RESET])
    events = []

    def do(data):
        out = command(loom, bench, data, limit)
        events.extend(loom.pop_available(2))      # OUTQ[2] holds four
        return out
    assert do([0x10]) == ([MODE | ACK], status(1, 1))
    assert do([0x12]) == (jr, status(2, 1))
    req = mdb.vend_request(0x0064, 0x0021)
    assert do(req) == ([MODE | ACK], status(1, 1))
    assert do([0x12]) == ([MODE | ACK], status(1, 1))
    loom.push(2, [0x005A])                           # approved, 90 (a discount)
    ok = mdb.peripheral_block([mdb.VEND_APPROVED, 0x00, 0x5A])
    assert do([0x12]) == (ok, status(4, 1))
    bench.step(int(2 * rate.timing.char))
    events += loom.pop_available(2)
    # JUST RESET from power-up was never polled: the RESET replaced it.
    assert events == [E_RESET, mdb.JUST_RESET, E_VEND, 0x0064, 0x0021, mdb.VEND_APPROVED]
    sent = blocks_of(vmc_line, rate, t0)
    got = blocks_of(dev_line, rate, t0, vmc=False)
    assert [[c.value for c in b] for b in sent] == [
        mdb.vmc_block([0x10]), mdb.vmc_block([0x12]), [ACK], mdb.vmc_block(req),
        mdb.vmc_block([0x12]), mdb.vmc_block([0x12]), [ACK]]
    assert [[c.value for c in b] for b in got] == [[MODE], jr, [MODE], [MODE], ok]
    for blk in sent:
        assert violations(blk, rate.timing, "vmc", rate.edge_tol) == []
    for blk in got:
        assert violations(blk, rate.timing, "peripheral", rate.edge_tol) == []
    # Every answer within t-response of its block, every ACK within
    # t-response of the data it answers (3.2).
    requests = [b for b in sent if len(b) > 1]
    for req_blk, ans in zip(requests, got):
        assert 0 < ans[0].start - req_blk[-1].end <= rate.timing.t_response
    for ack in (sent[2], sent[6]):
        data_end = max(c.end for c in dev_line.chars if c.end <= ack[0].start)
        assert ack[0].start - data_end <= rate.timing.t_response
