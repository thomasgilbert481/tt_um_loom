"""L3-RDM on the model and the RTL: firmware/rdm_responder.loom.

The responder (thread 0: RX = IN0, TX = OUT0, DE = OUT1, IDENT = OUT2)
sits on ``tools.protomodels.rdm.RdmLine``, the RS-485 pair with a model
controller on it: the controller drives the line while it sends, the
chip's TX while DE is 1, the bias resistors otherwise. ``Controller``
sends requests, waits for the response as E1.20's Table 3-2 says, and
decodes it; ``response_violations`` checks the responder delay, the
response's break, mark after break and slots, and DE against the timing
table the model cites.

Expected values come from E1.20 through the reference model (the packet
layout, the checksum, the encoded UID, the timing table) and from the
program header (the report format, DEVICE_INFO's contents), never from
what the model or the RTL did.

Rate: 250 kbit/s, 200 clocks a bit at 50 MHz, a tick of 25 clocks (eight
a bit). A request is 1.1 .. 1.7 ms, a response up to 2 ms, so one
exchange is about 150 k clocks: a whole discovery and the parameter
sweeps are ``model_only``; the RTL runs one exchange per scenario. A
controller listens 2.1 ms for silence: past Table 3-4's 2 ms maximum
responder delay, after which a response would be a violation anyway.

Every body takes a ``backend`` (``tools/tests/fw_backend.py``): pytest runs
them on the golden model, ``test/test_fw.py`` on the RTL.
"""

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels.bench import pad_of
from tools.protomodels.rdm import (
    ACK, BROADCAST, DEVICE_INFO, DISC_MUTE, DISC_UN_MUTE, DISC_UNIQUE_BRANCH,
    DISCOVERY, DISCOVERY_RESPONSE, GET, GET_RESPONSE, IDENTIFY_DEVICE, MAX_UID,
    NACK_REASON, NR_DATA_OUT_OF_RANGE, NR_FORMAT_ERROR, NR_SUB_DEVICE_OUT_OF_RANGE,
    NR_UNKNOWN_PID, NR_UNSUPPORTED_COMMAND_CLASS, RESPONDER_DELAY, SET,
    SET_RESPONSE, US, Controller, RdmLine, decode_dub, dub_request,
    dub_response, parse, response_violations)
from tools.tests.fw_backend import model_only

ISA = load()
PROGRAM = REPO / "firmware" / "rdm_responder.loom"
IMEM = 512
TICK = 25                           # eight ticks a bit (program header)
UID = 0x4C4F_1234_5678              # the responder under test
SILENCE_US = RESPONDER_DELAY[1] + 100.0
SLACK = 8                           # clocks: chip edges land a few after a tick

# The report (program header): word 1 = command class << 8 | flags.
SENT, DUBF, NACKF, BCASTF, IDF, MUTEF = 0x01, 0x02, 0x04, 0x08, 0x10, 0x20
BAD = 0x0080

# DEVICE_INFO as the program header lists it, in E1.20's field order:
# protocol 1.0, model 0x0001, category 0x0000, software 0x00000001,
# footprint 0, personality 1 of 1, start address 0xFFFF, 0 sub-devices,
# 0 sensors: 19 bytes.
DEVICE_INFO_PD = [0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
                  0x00, 0x00, 0x01, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0x00]


def rig(backend, uid=UID):
    """The responder booted with ``uid``, a line and a controller."""
    if backend.name == "model":
        bench = backend.bench(features=("FIFO", "BE", "BEENC", "DMEM"),
                              imem_words=IMEM)
    else:
        bench = backend.bench()
    line = bench.add(RdmLine())
    prog = assemble_file(PROGRAM, isa=ISA, imem_words=IMEM, strict=True)
    assert prog.diagnostics == []
    loom = backend.loom(bench, isa=ISA)
    loom.load(prog.words, verify=False)
    loom.write_csr(0, "TICK_INT", TICK)
    loom.write_csr(0, "TICK_FRAC", 0)
    loom.run([0])
    loom.push(0, [uid >> 32, (uid >> 16) & 0xFFFF, uid & 0xFFFF])
    bench.step(400)
    return bench, loom, line, Controller(bench.step, line, wait_us=SILENCE_US)


def assert_timing(ex):
    """The response against the timing table, and no fight on the line."""
    assert ex.response is not None
    assert response_violations(ex.end, ex.response, ex.de_edges, slack=SLACK) == []


def report(loom, n):
    return loom.pop(0, n, max_polls=200)


def answer(ex):
    """The response packet, parsed (checksum checked by ``parse``)."""
    assert ex.response is not None and ex.response.break_clocks is not None
    reply = parse(ex.response.data)
    assert reply is not None, ex.response.data
    return reply


def test_rdm_dub_answer_and_turnaround(backend):
    """DISC_UNIQUE_BRANCH over the whole range: the encoded UID with no
    break, its first edge 176 us .. 2 ms after the request ends (Table
    3-4), DE up before it and down at its end, nothing driving against
    the controller."""
    bench, loom, line, ctrl = rig(backend)
    ex = ctrl.transact(dub_request(0, MAX_UID, src=ctrl.uid, tn=1))
    resp = ex.response
    assert resp is not None and resp.break_clocks is None
    assert resp.data == dub_response(UID)
    assert decode_dub(resp.data) == UID
    assert_timing(ex)
    delay = (resp.first_fall - ex.end) / US
    assert RESPONDER_DELAY[0] <= delay <= RESPONDER_DELAY[1], delay
    assert line.contention == []
    assert report(loom, 2) == [DISCOVERY << 8 | DUBF | BCASTF, DISC_UNIQUE_BRANCH]
    assert loom.badop() == 0


def test_rdm_branch_above_the_uid_gets_no_reply(backend):
    """A branch that starts one past the UID: silence, DE never rises."""
    bench, loom, line, ctrl = rig(backend)
    ex = ctrl.transact(dub_request(UID + 1, MAX_UID, src=ctrl.uid, tn=2))
    assert ex.response is None and ex.de_edges == []
    assert report(loom, 2) == [DISCOVERY << 8 | BCASTF, DISC_UNIQUE_BRANCH]
    assert loom.badop() == 0


def test_rdm_get_device_info(backend):
    """GET DEVICE_INFO to this UID: GET_COMMAND_RESPONSE, ACK, the
    transaction number and PID echoed, the 19 bytes, a good checksum,
    timed to the table."""
    bench, loom, line, ctrl = rig(backend)
    ex = ctrl.transact(ctrl.request(GET, DEVICE_INFO, dst=UID))
    reply = answer(ex)
    assert (reply.dst, reply.src, reply.tn) == (ctrl.uid, UID, ctrl.tn)
    assert (reply.cc, reply.pid, reply.port) == (GET_RESPONSE, DEVICE_INFO, ACK)
    assert reply.pd == DEVICE_INFO_PD and reply.msg_count == 0
    assert_timing(ex)
    assert report(loom, 2) == [GET << 8 | SENT, DEVICE_INFO]
    assert line.contention == [] and loom.badop() == 0


def test_rdm_bad_checksum_gets_no_reply(backend):
    """GET DEVICE_INFO with its checksum off by one: no response, and the
    host hears of the bad packet."""
    bench, loom, line, ctrl = rig(backend)
    packet = ctrl.request(GET, DEVICE_INFO, dst=UID)
    ex = ctrl.transact(packet.encode(bad_checksum=True))
    assert ex.response is None and ex.de_edges == []
    assert report(loom, 1) == [BAD]
    assert loom.badop() == 0


# ------------------------------------------------------ model only: sweeps
@model_only("a whole discovery is about 12 ms, 600 k clocks")
def test_rdm_discovery_finds_the_responder(backend):
    """The controller's binary search (un-mute all, DUB, mute, DUB again)
    finds exactly this UID; every response is timed to the table; once
    muted the responder is silent to the full-range branch."""
    bench, loom, line, ctrl = rig(backend)
    assert ctrl.discover() == [UID]
    answered = [ex for ex in ctrl.log if ex.response is not None]
    assert len(answered) == 2               # the DUB and the mute
    for ex in answered:
        assert_timing(ex)
    assert ctrl.log[-1].response is None    # muted: the last DUB is silent
    assert line.contention == []
    # OUTQ holds four words and nobody popped: the first two reports.
    assert loom.pop_available(0) == [DISCOVERY << 8 | BCASTF, DISC_UN_MUTE,
                                     DISCOVERY << 8 | DUBF | BCASTF,
                                     DISC_UNIQUE_BRANCH]


@model_only("four exchanges, about 600 k clocks")
def test_rdm_branches_that_exclude_the_uid(backend):
    """Silence for a branch that ends one below the UID; an answer for
    the branch that is exactly the UID; silence once muted; an answer
    again after DISC_UN_MUTE to this UID, which is itself answered with
    a control field of 0x0000."""
    bench, loom, line, ctrl = rig(backend)
    assert ctrl.dub(0, UID - 1) is None
    assert ctrl.dub(UID, UID) == UID
    assert ctrl.mute(UID)
    assert ctrl.dub(0, MAX_UID) is None
    ex = ctrl.transact(ctrl.request(DISCOVERY, DISC_UN_MUTE, dst=UID))
    reply = answer(ex)
    assert (reply.cc, reply.pid, reply.pd) == (DISCOVERY_RESPONSE, DISC_UN_MUTE, [0, 0])
    assert_timing(ex)
    assert ctrl.dub(0, MAX_UID) == UID
    assert line.contention == []


@model_only("three exchanges")
def test_rdm_identify_device(backend):
    """SET IDENTIFY_DEVICE 1 turns IDENT on and is ACKed with no data;
    GET reads it back; a value of 2 is NACKed NR_DATA_OUT_OF_RANGE and
    changes nothing."""
    bench, loom, line, ctrl = rig(backend)
    ident = pad_of("OUT2")
    assert bench.lines.get(ident) == 0
    reply = answer(ctrl.transact(ctrl.request(SET, IDENTIFY_DEVICE, [1], dst=UID)))
    assert (reply.cc, reply.port, reply.pd) == (SET_RESPONSE, ACK, [])
    assert bench.lines.get(ident) == 1
    reply = answer(ctrl.transact(ctrl.request(GET, IDENTIFY_DEVICE, dst=UID)))
    assert (reply.cc, reply.port, reply.pd) == (GET_RESPONSE, ACK, [1])
    reply = answer(ctrl.transact(ctrl.request(SET, IDENTIFY_DEVICE, [2], dst=UID)))
    assert (reply.port, reply.pd) == (NACK_REASON, [0, NR_DATA_OUT_OF_RANGE])
    assert bench.lines.get(ident) == 1
    words = loom.pop_available(0)
    assert words[:2] == [SET << 8 | SENT | IDF, IDENTIFY_DEVICE]


@model_only("five exchanges")
def test_rdm_nacks(backend):
    """NR_UNKNOWN_PID, NR_FORMAT_ERROR, NR_UNSUPPORTED_COMMAND_CLASS and
    NR_SUB_DEVICE_OUT_OF_RANGE, each a NACK_REASON response with the
    reason high byte first."""
    bench, loom, line, ctrl = rig(backend)
    cases = [(ctrl.request(GET, 0x00F0, dst=UID), NR_UNKNOWN_PID),
             (ctrl.request(GET, DEVICE_INFO, [0], dst=UID), NR_FORMAT_ERROR),
             (ctrl.request(SET, DEVICE_INFO, [0], dst=UID),
              NR_UNSUPPORTED_COMMAND_CLASS),
             (ctrl.request(GET, DEVICE_INFO, dst=UID, sub_device=1),
              NR_SUB_DEVICE_OUT_OF_RANGE),
             (ctrl.request(SET, IDENTIFY_DEVICE, [1, 0], dst=UID), NR_FORMAT_ERROR)]
    for packet, reason in cases:
        ex = ctrl.transact(packet)
        reply = answer(ex)
        assert (reply.port, reply.pd) == (NACK_REASON, [reason >> 8, reason & 0xFF])
        assert reply.cc == packet.cc + 1 and reply.pid == packet.pid
        assert_timing(ex)
        loom.pop_available(0)               # keep OUTQ from filling
    assert bench.lines.get(pad_of("OUT2")) == 0


@model_only("broadcasts and foreign packets, four exchanges")
def test_rdm_ignores_what_is_not_for_it(backend):
    """A broadcast SET IDENTIFY_DEVICE is acted on and not answered; a
    GET to another UID and a DMX512 null start code packet get nothing
    and no report; a manufacturer broadcast DISC_MUTE mutes silently."""
    bench, loom, line, ctrl = rig(backend)
    ex = ctrl.transact(ctrl.request(SET, IDENTIFY_DEVICE, [1], dst=BROADCAST))
    assert ex.response is None and ex.de_edges == []
    assert bench.lines.get(pad_of("OUT2")) == 1
    assert report(loom, 2) == [SET << 8 | BCASTF | IDF, IDENTIFY_DEVICE]
    ex = ctrl.transact(ctrl.request(GET, DEVICE_INFO, dst=UID + 1))
    assert ex.response is None
    ex = ctrl.transact([0x00] + list(range(1, 25)))     # dimmer data
    assert ex.response is None
    assert loom.pop_available(0) == []
    ex = ctrl.transact(ctrl.request(DISCOVERY, DISC_MUTE,
                                    dst=(UID >> 32) << 32 | 0xFFFFFFFF))
    assert ex.response is None
    assert report(loom, 2) == [DISCOVERY << 8 | BCASTF | IDF | MUTEF, DISC_MUTE]
    assert ctrl.dub(0, MAX_UID) is None
    assert line.contention == [] and loom.badop() == 0
