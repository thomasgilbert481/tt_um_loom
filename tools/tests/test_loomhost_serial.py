"""PicoTransport and TTBoardTransport against fake serial ports.

The far end of each fake is a ModelTransport, so the whole Loom API runs
through the Pico line protocol and through the TT board's raw REPL exactly as
it would over USB, with the golden model standing in for the chip. The
MicroPython programs themselves cannot run here; they are compiled to check
their syntax.
"""

import sys

import pytest

from tools.loomasm import assemble
from tools.loomhost import (Loom, ModelTransport, PicoTransport, TTBoardTransport,
                            TransportError)
from tools.loomhost.serial_transport import MICROPYTHON_DIR

ECHO = ".thread 0\nloop: POP r0\n XORI r0, 0x3F\n PUSH r0\n JMP loop\n"


class FakePico:
    """The Pico bridge's line protocol, answered from a ModelTransport."""

    def __init__(self, model=None, hello="LOOMBRIDGE 1"):
        self.model = model or ModelTransport()
        self.hello = hello
        self.lines = []
        self.out = []
        self.sck = None
        self.closed = False
        self.override = None

    def write(self, data):
        for line in data.decode("ascii").splitlines():
            self.lines.append(line)
            self.out.append(self.answer(line) + "\n")

    def answer(self, line):
        if self.override is not None:
            return self.override
        op, _, arg = line.partition(" ")
        if op == "H":
            return self.hello
        if op == "X":
            return "R " + self.model.transfer(bytes.fromhex(arg)).hex()
        if op == "I":
            return "Q %d" % self.model.irq()
        if op == "S":
            self.sck = int(arg)
            return "OK"
        return "E unknown command"

    def readline(self):
        return self.out.pop(0).encode("ascii") if self.out else b""

    def close(self):
        self.closed = True


class FakeRepl:
    """tt-micropython-firmware's raw REPL, running the helper's calls on a model."""

    def __init__(self, model=None):
        self.model = model or ModelTransport()
        self.out = bytearray()
        self.code = bytearray()
        self.raw = False
        self.executed = []
        self.setup = None
        self.closed = False

    def reset_input_buffer(self):
        self.out.clear()

    def write(self, data):
        for byte in data:
            if not self.raw:
                if byte == 0x01:
                    self.raw = True
                    self.out += b"raw REPL; CTRL-B to exit\r\n>"
                continue
            if byte == 0x04:
                self.run(self.code.decode("utf-8"))
                self.code.clear()
            elif byte == 0x02:
                self.raw = False
            else:
                self.code.append(byte)

    def run(self, code):
        self.executed.append(code)
        out, err = "", ""
        if code.startswith("_lsetup("):
            self.setup = code
        elif code.startswith("_lx("):
            tx = bytes.fromhex(code[len("_lx('"):-2])
            out = self.model.transfer(tx).hex() + "\r\n"
        elif code.startswith("_lirq()"):
            out = "%d\r\n" % self.model.irq()
        elif code.startswith("boom"):
            err = "Traceback (most recent call last):\r\nNameError: boom\r\n"
        self.out += b"OK" + out.encode() + b"\x04" + err.encode() + b"\x04>"

    def read(self, n):
        chunk, self.out[:n] = bytes(self.out[:n]), b""
        return chunk

    def close(self):
        self.closed = True


def exercise(loom, model):
    """The same API session over any transport."""
    assert loom.id() == 0x4C4D
    program = assemble(ECHO)
    loom.load(program)
    loom.run(0)
    loom.push(0, [0x00, 0x3F, 0x1234])
    assert loom.pop(0, 3) == [0x3F, 0x00, 0x120B]
    loom.halt(0)
    assert loom.dump(0)["PC"] in range(4)
    assert model.machine.imem[1] == program.words[1]


# ------------------------------------------------------------------- Pico
def test_pico_transport_runs_the_whole_api_through_the_line_protocol():
    fake = FakePico()
    t = PicoTransport(serial=fake, sck_hz=2_000_000)
    assert t.bridge_version == 1 and fake.sck == 2_000_000
    exercise(Loom(t), fake.model)
    assert fake.lines[0] == "H" and fake.lines[2].startswith("X 00000000")


def test_pico_irq_is_read_from_the_bridge():
    fake = FakePico()
    t = PicoTransport(serial=fake)
    loom = Loom(t)
    assert t.irq() is False
    loom.set_sflags(1)
    loom.irq_enable(0x0100)
    assert t.irq() is True and loom.irq_pending()


@pytest.mark.parametrize("hello, message", [("HELLO", "no Loom bridge"),
                                            ("LOOMBRIDGE 2", "protocol 2")])
def test_pico_hello_is_checked(hello, message):
    with pytest.raises(TransportError, match=message):
        PicoTransport(serial=FakePico(hello=hello))


def test_pico_errors_become_transport_errors():
    fake = FakePico()
    t = PicoTransport(serial=fake)
    fake.override = "E spi fault"
    with pytest.raises(TransportError, match="spi fault"):
        t.transfer(b"\x00\x00\x00\x00\x00\x00")
    fake.override = "R 00"
    with pytest.raises(TransportError, match="1 bytes for 6"):
        t.transfer(b"\x00\x00\x00\x00\x00\x00")
    fake.override = "Z"
    with pytest.raises(TransportError, match="unexpected"):
        t.transfer(b"\x00\x00\x00")
    fake.override = None
    fake.readline = lambda: b""
    with pytest.raises(TransportError, match="timed out"):
        t.irq()
    t.close()
    assert fake.closed


def test_pico_needs_a_port_or_a_serial_object():
    with pytest.raises(TransportError):
        PicoTransport()


def test_pyserial_is_only_imported_to_open_a_port(monkeypatch):
    monkeypatch.setitem(sys.modules, "serial", None)    # make "import serial" fail
    with pytest.raises(TransportError, match="pyserial"):
        PicoTransport("COM99")
    with pytest.raises(TransportError, match="pyserial"):
        TTBoardTransport("COM99")


# -------------------------------------------------------------- TT board
def test_tt_board_pastes_the_helper_and_runs_the_whole_api():
    fake = FakeRepl()
    t = TTBoardTransport(serial=fake, project="tt_um_loom", clock_hz=50_000_000,
                         sck_hz=1_000_000)
    helper = (MICROPYTHON_DIR / "tt_helper.py").read_text(encoding="utf-8")
    assert fake.executed[0] == helper
    assert fake.setup == "_lsetup('tt_um_loom', 50000000, 1000000)"
    exercise(Loom(t), fake.model)
    assert fake.executed[-1].startswith("_lx('")
    assert t.irq() is False
    t.close()
    assert fake.closed and not fake.raw


def test_tt_board_exceptions_become_transport_errors():
    t = TTBoardTransport(serial=FakeRepl(), setup=False)
    with pytest.raises(TransportError, match="NameError: boom"):
        t.exec("boom()")


def test_tt_board_timeout_is_reported():
    fake = FakeRepl()
    t = TTBoardTransport(serial=fake, setup=False)
    fake.run = lambda code: None                       # the board never answers
    with pytest.raises(TransportError, match="timed out"):
        t.transfer(b"\x00\x00\x00\x00\x00\x00")


# ---------------------------------------------------------- MicroPython
@pytest.mark.parametrize("name", ["pico_bridge.py", "tt_helper.py"])
def test_micropython_sources_compile(name):
    source = (MICROPYTHON_DIR / name).read_text(encoding="utf-8")
    compile(source, name, "exec")
