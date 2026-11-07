"""L3 for the IEEE 488 (GPIB) device program, model and RTL.

``firmware/gpib_device.loom`` is a talker and listener at a primary address
the host sets, with serial poll and SRQ: DIO1..8 on BIDIR0..7 (open drain),
receivers ATN, DAV, NRFD, NDAC, EOI on IN0..IN4, drivers DAV, NRFD, NDAC,
EOI, SRQ on OUT0..OUT4 through the board's open-collector inverters. The
references are ``tools.protomodels.gpib``: ``GpibBus`` is the board,
``Controller`` the controller-in-charge (address 0), ``Listener`` a second
device, ``HandshakeChecker`` the handshake rules. Expected values come from
the sources in that module's docstring (command bytes, the handshake order,
T1 = 2 us, the serial poll byte's bit 6) and the program header's stated
windows, never from what the model or the RTL did.

All bodies are cheap (a byte is a few hundred clocks) and run on both
backends. The PHY-GL-SDF candidate is
``test_gpib_controller_reads_from_device_with_eoi``: the device as source,
T1 measured on the pins, a byte cut short by ATN and offered again.
"""

from tools.loomasm import assemble_file
from tools.loomisa import REPO, load
from tools.protomodels import gpib as G

ISA = load()
DEVICE = REPO / "firmware" / "gpib_device.loom"
IMEM = 512
CLOCK_HZ = 50e6
ADDR = 5                        # the device's primary address in the tests
WORDS = 204                     # the program header and FW_README.md
OTHER = 7                       # the second (model) listener
STARTUP = 200                   # clocks for the program's set-up after RUN
T1 = G.ns_to_clocks(G.T1_NS, CLOCK_HZ)          # 100 clocks
#: ATN asserted to the device's NDAC asserted, from idle or talking with
#: nothing on the bus: at most 62 clocks after the receiver's pad changes
#: (the program header's "Timing", derived from SEMANTICS 2 and 3: a host
#: word taken just as ATN comes), plus one clock of board wire.
ATN_NDAC_MAX = 63
#: ATN to DAV, EOI and DIO all released while the device sources a byte:
#: at most 44 clocks (same derivation), plus the board wire.
ATN_RELEASE_MAX = 45


def program():
    return assemble_file(DEVICE, isa=ISA, strict=True)


def setup(backend, t1_units=0, parties=()):
    """The device at ADDR, a controller, the checker; returns
    ``(bench, loom, bus, ctl, chk)`` with the program running."""
    if backend.name == "model":
        bench = backend.bench(features=("FIFO",), imem_words=IMEM, pullups=0xFF)
    else:
        bench = backend.bench(pullups=0xFF)
    bus = bench.add(G.GpibBus())
    ctl = bus.add(G.Controller(0, clock_hz=CLOCK_HZ))
    for p in parties:
        bus.add(p)
    chk = G.HandshakeChecker()
    bus.observers.append(chk)
    loom = backend.loom(bench, isa=ISA)
    loom.load(program())
    loom.run(0)
    loom.push(0, [ADDR | (t1_units << 8)])
    bench.step(STARTUP)
    return bench, loom, bus, ctl, chk


def settle(bench, ctl, limit=60000):
    assert bench.run_until(lambda: not ctl.busy, limit, every=8), "controller stuck"


def clean(chk, ctl, *others):
    assert chk.violations == []
    assert ctl.errors == []
    for o in others:
        assert o.errors == []


def data_words(data):
    """The device's OUTQ words for ``data`` received with EOI on the last."""
    return [b | (0x100 if i == len(data) - 1 else 0) for i, b in enumerate(data)]


# ------------------------------------------------------------- assembly
def test_gpib_device_assembles_strict_with_no_diagnostic():
    """L2-DEADLINE: no diagnostic; no deadline pair by design (the program
    follows the handshake, only T1 is a counted loop); WORDS words from 0x000,
    spilling into thread 1's quarter (the header says why that costs
    nothing: the program takes every BIDIR and IN pin)."""
    prog = program()
    assert prog.diagnostics == [] and list(prog.threads) == [0]
    assert prog.deadlines[0].pairs == []
    assert len(prog.words) == WORDS
    assert sorted(prog.words) == list(range(WORDS))


def atn_answers_ok(chk):
    """Every ATN answer of the chip came within T1 (the controller waits T1
    after ATN before DAV, so this is what the bus needs) and within the
    program header's worst cases."""
    assert all(n <= ATN_NDAC_MAX for n in chk.atn_ndac), chk.atn_ndac
    assert all(n <= ATN_RELEASE_MAX for n in chk.atn_release), chk.atn_release
    assert ATN_NDAC_MAX < T1 and ATN_RELEASE_MAX < T1


# ------------------------------------------------------------ scenarios
def test_gpib_controller_writes_to_device(backend):
    """UNL, MLA 5, MTA 0, six data bytes with EOI on the last, UNL: the
    host gets the six bytes with the EOI bit on the last; the handshake
    rules hold on every byte and the device answered ATN in time."""
    bench, loom, bus, ctl, chk = setup(backend)
    data = list(b"VOLT 1")
    ctl.command([G.UNL, G.mla(ADDR), G.mta(0)])
    ctl.write(data)
    ctl.command([G.UNL])
    ctl.release()
    # OUTQ holds four words: the device holds NRFD (the bus waits) until
    # the host pops, so the host reads while the controller writes.
    assert loom.pop(0, len(data)) == data_words(data)
    settle(bench, ctl)
    assert loom.pop_available(0) == []
    clean(chk, ctl)
    moved = [t for t in chk.transfers if not t.atn]
    assert [t.byte for t in moved] == data and [t.eoi for t in moved][-1]
    assert all(t.done is not None for t in chk.transfers)
    assert chk.atn_ndac, "the device never answered ATN"
    atn_answers_ok(chk)


def test_gpib_controller_reads_from_device_with_eoi(backend):
    """The host queues "1.25E+0" with EOI on the last byte; MTA 5 and the
    controller reads until EOI. Every byte the device sources waits T1
    (2 us) from DIO to DAV and asserts DAV only with NRFD released. Then a
    read of two bytes of three: ATN takes the bus while the third waits for
    NRFD, the device lets go of DAV, EOI and DIO, and offers the byte again
    at the next read."""
    bench, loom, bus, ctl, chk = setup(backend)
    text = list(b"1.25E+0")
    ctl.command([G.UNL, G.mla(0), G.mta(ADDR)])
    ctl.read()
    ctl.command([G.UNT])
    loom.push(0, data_words(text))      # more than INQ holds: the bus runs meanwhile
    settle(bench, ctl)
    assert ctl.reads[0] == [(b, i == len(text) - 1) for i, b in enumerate(text)]
    loom.push(0, [0x41, 0x42, 0x143])
    ctl.command([G.mta(ADDR)])
    ctl.read(limit=2, until_eoi=False)
    ctl.pause(400)                      # the third byte goes on DIO, T1, NRFD
    ctl.command([G.UNT, G.mta(ADDR)])
    ctl.read()
    ctl.command([G.UNT])
    ctl.release()
    settle(bench, ctl)
    assert ctl.reads[1] == [(0x41, False), (0x42, False)]
    assert ctl.reads[2] == [(0x43, True)]
    mine = [t for t in chk.transfers if t.by_chip and t.done is not None]
    assert len(mine) == len(text) + 3
    assert all(t.t1 >= T1 for t in mine), [t.t1 for t in mine]
    # Mid-byte, the release comes through a JP ATN loop and t_atn: at least
    # 24 clocks after the receiver's pad (the header), 25 on the wire. The
    # shorter ones are the device's own release after an accepted byte.
    assert any(n >= 25 for n in chk.atn_release), chk.atn_release
    clean(chk, ctl)
    atn_answers_ok(chk)
    assert loom.pop_available(0) == []


def test_gpib_slow_second_listener_paces_the_bus(backend):
    """A second listener at address 7 takes 300 clocks (6 us) from DAV to
    each byte. Controller to both: every byte stays on the bus until the
    slow one has it, so DAV to NDAC high is at least 300 clocks, and both
    get the bytes. Device to both: the device waits for the slow one too."""
    slow = G.Listener(OTHER, delay=300, recover=50)
    bench, loom, bus, ctl, chk = setup(backend, parties=[slow])
    data = [0x00, 0xFF, 0x5A]
    ctl.command([G.UNL, G.mla(ADDR), G.mla(OTHER), G.mta(0)])
    ctl.write(data)
    settle(bench, ctl)
    assert loom.pop(0, 3) == data_words(data)
    assert slow.received == [(0x00, False), (0xFF, False), (0x5A, True)]
    loom.push(0, [0x31, 0x132])
    ctl.command([G.UNL, G.mla(0), G.mla(OTHER), G.mta(ADDR)])
    ctl.read()
    ctl.command([G.UNT, G.UNL])
    ctl.release()
    settle(bench, ctl)
    assert ctl.reads == [[(0x31, False), (0x32, True)]]
    assert slow.received[3:] == [(0x31, False), (0x32, True)]
    assert all(t.done - t.dav >= 300 for t in chk.transfers), \
        [(t.byte, t.done - t.dav) for t in chk.transfers]
    clean(chk, ctl, slow)
    atn_answers_ok(chk)


def test_gpib_serial_poll_after_srq(backend):
    """The host sets the status byte 0x41 (bit 6, rsv, set): the device
    asserts SRQ. The controller waits for SRQ and serial polls address 5:
    the poll byte is 0x41, SRQ is released after it, the host is told
    (0x8141). A second poll reads 0x01 (bit 6 cleared) with SRQ released.
    A status byte without bit 6 never asserts SRQ."""
    bench, loom, bus, ctl, chk = setup(backend)
    loom.push(0, [0x8001])
    bench.step(400)
    assert not bus.state["SRQ"]
    loom.push(0, [0x8041])
    ctl.wait_srq()
    ctl.serial_poll(ADDR)
    settle(bench, ctl)
    assert ctl.polls == [0x41] and ctl.polls[0] & G.RQS
    assert not bus.state["SRQ"]
    ctl.serial_poll(ADDR)
    ctl.release()
    settle(bench, ctl)
    assert ctl.polls == [0x41, 0x01]
    assert not bus.state["SRQ"]
    assert loom.pop(0, 2) == [0x8141, 0x8101]
    clean(chk, ctl)
    atn_answers_ok(chk)


def test_gpib_device_ignores_another_address(backend):
    """The controller talks to the listener at 7 (MLA 7) and addresses 7 to
    talk (MTA 7): the device at 5 takes none of the bytes and never drives
    DIO, DAV or EOI; SDC to listener 7 is not the device's either. DCL is
    universal (0x8014); MLA 5 then SDC and GET are reported (0x8004,
    0x8008); UNL ends it."""
    other = G.Listener(OTHER)
    bench, loom, bus, ctl, chk = setup(backend, parties=[other])
    drove = []
    bus.observers.append(lambda s: drove.append(s.chip_dio or s.chip["DAV"]
                                                or s.chip["EOI"]))
    ctl.command([G.UNL, G.mla(OTHER), G.mta(0)])
    ctl.write(list(b"*RST"))
    ctl.command([G.SDC, G.UNL, G.mta(OTHER), G.mla(0)])
    ctl.command([G.UNT, G.UNL])
    settle(bench, ctl)
    assert other.received == [(b, i == 3) for i, b in enumerate(b"*RST")]
    assert loom.pop_available(0) == []
    assert not any(drove)
    ctl.command([G.DCL, G.mla(ADDR), G.SDC, G.GET, G.UNL])
    ctl.release()
    settle(bench, ctl)
    assert loom.pop(0, 3) == [0x8014, 0x8004, 0x8008]
    clean(chk, ctl, other)
    atn_answers_ok(chk)
