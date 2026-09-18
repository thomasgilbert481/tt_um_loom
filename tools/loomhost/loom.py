"""High-level host API for a Loom chip, over any transport.

::

    from tools.loomhost import Loom, ModelTransport
    loom = Loom(ModelTransport(image=...))       # or PicoTransport("COM7")
    loom.load("firmware/build/uart_tx_fifo.json")  # IMEM while halted, verified
    loom.write_csr(thread=0, csr="TICK_INT", value=434)
    loom.run(0)
    loom.push(0, b"Hi")
    state = loom.dump(0)

Every method is a handful of transactions from ``tools.loomhost.protocol``;
the class holds no chip state of its own except the cached ``CAPS`` word.
Misuse (loading or writing registers while threads run, stepping a running
thread, FIFOs on a build without them) raises :class:`LoomStateError` before
any transaction is sent, so nothing is silently dropped by the chip.
"""

from __future__ import annotations

import json
import pathlib
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

from tools.loomisa import Isa
from tools.loomisa import load as load_isa

from . import protocol as P
from .transport import Transport

Threads = Union[None, int, Iterable[int]]


class LoomError(RuntimeError):
    """Base class of every host-library error."""


class LoomStateError(LoomError):
    """The chip is not in a state that allows the request."""


class LoomVerifyError(LoomError):
    """Instruction-memory readback differs from what was written."""


class LoomTimeout(LoomError):
    """Polling gave up."""


def thread_mask(threads: Threads) -> int:
    """``None`` -> all four; an int -> that thread; an iterable -> those threads."""
    if threads is None:
        return 0xF
    if isinstance(threads, int):
        threads = [threads]
    mask = 0
    for t in threads:
        if not isinstance(t, int) or not 0 <= t < P.THREADS:
            raise ValueError("thread %r outside 0..3" % (t,))
        mask |= 1 << t
    return mask


def load_image(source) -> Tuple[Dict[int, int], Optional[int]]:
    """Normalise an instruction image; returns ``(words, imem_words or None)``.

    Accepts a path to a JSON image or to ``.loom`` source (assembled here), an
    assembler ``Program``, a parsed JSON image (``{"words": ...}``), a plain
    ``{address: word}`` mapping, or a sequence of words from address 0.
    """
    imem_words = None
    if isinstance(source, (str, pathlib.Path)):
        path = pathlib.Path(source)
        if path.suffix == ".loom":
            from tools.loomasm import assemble_file
            source = assemble_file(path)
        else:
            source = json.loads(path.read_text(encoding="utf-8"))
    if hasattr(source, "words") and hasattr(source, "imem_words"):
        return {int(a): int(w) & 0xFFFF for a, w in source.words.items()}, source.imem_words
    if isinstance(source, Mapping):
        if "words" in source:
            imem_words = source.get("imem_words")
            source = source["words"]
        words = {}
        for key, value in source.items():
            addr = key if isinstance(key, int) else int(str(key), 0)
            words[addr] = int(value) & 0xFFFF
        return words, imem_words
    return {addr: int(w) & 0xFFFF for addr, w in enumerate(source)}, None


def runs(words: Mapping[int, int], limit: int = 256) -> List[Tuple[int, List[int]]]:
    """Group an image into ``(start, [words])`` runs of consecutive addresses."""
    out: List[Tuple[int, List[int]]] = []
    for addr in sorted(words):
        if out and out[-1][0] + len(out[-1][1]) == addr and len(out[-1][1]) < limit:
            out[-1][1].append(words[addr])
        else:
            out.append((addr, [words[addr]]))
    return out


class Loom:
    """One Loom chip behind a :class:`~tools.loomhost.transport.Transport`."""

    #: Debug registers read by :meth:`dump`, in address order 0x00..0x26.
    DUMP_WORDS = P.DEBUG["FIFO_CNT"] + 1

    def __init__(self, transport: Transport, *, isa: Optional[Isa] = None,
                 max_polls: int = 10000) -> None:
        self.transport = transport
        self.isa = isa or load_isa()
        self.max_polls = max_polls
        self._caps: Optional[Dict[str, int]] = None
        self._csr_numbers = {v["name"].upper(): k for k, v in self.isa.csrs.items()}

    # --------------------------------------------------------- raw access
    def read(self, space: int, addr: int, count: int = 1) -> List[int]:
        rx = self.transport.transfer(P.encode_read(space, addr, count))
        return P.decode_read_response(rx, count)

    def write(self, space: int, addr: int, words: Iterable[int]) -> None:
        self.transport.transfer(P.encode_write(space, addr, list(words)))

    def read_ctrl(self, name: str) -> int:
        return self.read(P.SPACE_CTRL, P.CTRL[name.upper()])[0]

    def write_ctrl(self, name: str, value: int) -> None:
        self.write(P.SPACE_CTRL, P.CTRL[name.upper()], [value & 0xFFFF])

    # ------------------------------------------------------------ identity
    def id(self) -> int:
        return self.read_ctrl("ID")

    def version(self) -> Tuple[int, int]:
        word = self.read_ctrl("VERSION")
        return word >> 8, word & 0xFF

    def caps(self, refresh: bool = False) -> Dict[str, int]:
        """``CTRL.CAPS`` decoded as SEMANTICS section 5 lays it out."""
        if self._caps is None or refresh:
            self._caps = P.decode_caps(self.read_ctrl("CAPS"))
        return dict(self._caps)

    def check_id(self) -> None:
        got = self.id()
        if got != P.ID_VALUE:
            raise LoomError("no Loom on this transport: ID reads 0x%04X, expected 0x%04X"
                            % (got, P.ID_VALUE))

    # ---------------------------------------------------------- run control
    def running(self) -> int:
        return self.read_ctrl("RUN") & 0xF

    def halted(self) -> int:
        """``HALTED``: threads that executed ``HALT`` since their last start."""
        return self.read_ctrl("HALTED") & 0xF

    def run(self, threads: Threads = None, *, exclusive: bool = False) -> None:
        """Start threads at their current PC.

        ``RUN`` has no set/clear aliases, so this is read-modify-write and
        can restart a thread that halted itself in between; ``exclusive``
        writes the mask as given instead (stopping every other thread).
        """
        mask = thread_mask(threads)
        self.write_ctrl("RUN", mask if exclusive else (self.running() | mask))

    def halt(self, threads: Threads = None) -> None:
        """Stop threads after their current instruction."""
        self.write_ctrl("RUN", self.running() & ~thread_mask(threads))

    def _require_halted(self, mask: int, what: str) -> None:
        busy = self.running() & mask
        if busy:
            raise LoomStateError("cannot %s: thread(s) %s running (RUN=0x%X); halt first"
                                 % (what, _names(busy), busy))

    def step(self, thread: int, count: int = 1, *, check: bool = True) -> None:
        """Execute exactly ``count`` slots of a halted thread, one STEP each."""
        mask = thread_mask(thread)
        if check:
            self._require_halted(mask, "step")
        for _ in range(count):
            self.write(P.SPACE_STEP, P.step_addr(thread), [1])

    def reset(self, threads: Threads = None, *, check: bool = True) -> None:
        """``CTRL.RESET``: PC to RESET_PC, flags 0, TD := NOW, stack and FIFOs empty."""
        mask = thread_mask(threads)
        if check:
            self._require_halted(mask, "reset")
        self.write_ctrl("RESET", mask)

    def set_reset_pc(self, thread: int, pc: int) -> None:
        self.write_ctrl("RESET_PC%d" % _thread(thread), pc & 0x3FF)

    def reset_pc(self, thread: int) -> int:
        return self.read_ctrl("RESET_PC%d" % _thread(thread))

    def wait_halted(self, thread: int, max_polls: Optional[int] = None) -> None:
        """Poll ``HALTED`` until the thread has executed ``HALT``."""
        bit = thread_mask(thread)
        for _ in range(max_polls or self.max_polls):
            if self.halted() & bit:
                return
        raise LoomTimeout("thread %d did not halt" % thread)

    # ----------------------------------------------------------- program
    def load(self, image, *, verify: bool = True, chunk: int = 256) -> Dict[int, int]:
        """Write an instruction image while every thread is halted.

        Raises :class:`LoomStateError` if any thread runs, :class:`LoomError`
        if the image does not fit this build or the chip refused the writes
        (``BADOP[15]``), and :class:`LoomVerifyError` if the readback differs.
        Returns the ``{address: word}`` mapping that was written.
        """
        words, laid_out_for = load_image(image)
        self._require_halted(0xF, "load instruction memory")
        size = self.caps()["imem_words"]
        # Thread t's section starts at t * words / 4, so an image laid out for
        # another size is only portable if it stays inside thread 0's quarter.
        quarter = min(laid_out_for or size, size) // 4
        if laid_out_for not in (None, size) and max(words, default=0) >= quarter:
            raise LoomError("image is laid out for %d words but this build has %d; "
                            "reassemble with --imem-words %d" % (laid_out_for, size, size))
        too_high = [a for a in words if not 0 <= a < size]
        if too_high:
            raise LoomError("image address 0x%X is outside the %d-word memory"
                            % (min(too_high), size))
        if self.badop(clear=False) & P.BADOP_ACCESS:
            self.clear_badop(P.BADOP_ACCESS)
        for start, block in runs(words, chunk):
            self.write(P.SPACE_IMEM, start, block)
        if self.badop(clear=False) & P.BADOP_ACCESS:
            self.clear_badop(P.BADOP_ACCESS)
            raise LoomError("the chip refused the IMEM writes (BADOP[15]): "
                            "a thread started or a step was in flight")
        if verify:
            bad = self.verify(words, chunk=chunk)
            if bad:
                first = ", ".join("0x%03X: wrote %04X read %04X" % item for item in bad[:4])
                raise LoomVerifyError("IMEM readback differs at %d address(es): %s"
                                      % (len(bad), first))
        return words

    def verify(self, image, *, chunk: int = 256) -> List[Tuple[int, int, int]]:
        """``(address, expected, read)`` for every word that differs."""
        words, _ = load_image(image)
        self._require_halted(0xF, "read instruction memory")
        bad = []
        for start, block in runs(words, chunk):
            got = self.read(P.SPACE_IMEM, start, len(block))
            bad += [(start + i, w, g) for i, (w, g) in enumerate(zip(block, got)) if w != g]
        return bad

    def read_imem(self, addr: int, count: int = 1) -> List[int]:
        self._require_halted(0xF, "read instruction memory")
        return self.read(P.SPACE_IMEM, addr, count)

    # ------------------------------------------------------- debug space
    def _debug_reg(self, name: Union[str, int]) -> int:
        if isinstance(name, int):
            return name
        key = P.reg_name_key(name)
        if key in P.DEBUG:
            return P.DEBUG[key]
        number = self._csr_numbers.get(key)
        if number is not None and number < P.DEBUG_CSR_COUNT:
            return P.DEBUG_CSR_BASE + number
        raise LoomError("no debug register %r" % (name,))

    def read_reg(self, thread: int, name: Union[str, int]) -> int:
        """One DEBUG-space register by name (``r3``, ``PC``, ``TD``, ``RS1`` ...).

        ``r0..r7`` read 0 while the thread runs (HOST_PROTOCOL space 4).
        """
        key = P.reg_name_key(name) if isinstance(name, str) else name
        if key in ("RS1", "DEPTH"):
            rs1, depth = P.unpack_rs1_depth(self.read_reg(thread, "RS1_DEPTH"))
            return rs1 if key == "RS1" else depth
        if key in ("INQ_CNT", "OUTQ_CNT"):
            inq, outq = P.unpack_fifo_counts(self.read_reg(thread, "FIFO_CNT"))
            return inq if key == "INQ_CNT" else outq
        return self.read(P.SPACE_DEBUG, P.debug_addr(_thread(thread), self._debug_reg(key)))[0]

    def write_reg(self, thread: int, name: Union[str, int], value: int) -> None:
        """Write one DEBUG-space register; the thread must be halted."""
        self._require_halted(thread_mask(thread), "write thread %d registers" % thread)
        key = P.reg_name_key(name) if isinstance(name, str) else name
        if key in ("RS1", "DEPTH"):
            rs1, depth = P.unpack_rs1_depth(self.read_reg(thread, "RS1_DEPTH"))
            rs1, depth = (value, depth) if key == "RS1" else (rs1, value)
            key, value = "RS1_DEPTH", P.pack_rs1_depth(rs1, depth)
        reg = self._debug_reg(key)
        if reg in P.DEBUG_READ_ONLY or reg in (P.DEBUG_CSR_BASE + self._csr_numbers["NOW"],
                                                P.DEBUG_CSR_BASE + self._csr_numbers["TID"]):
            raise LoomError("debug register %r is read-only" % (name,))
        self.write(P.SPACE_DEBUG, P.debug_addr(thread, reg), [value & 0xFFFF])

    def dump(self, thread: int) -> Dict[str, int]:
        """The whole DEBUG space of one thread plus its RUN/HALTED/BADOP bits."""
        words = self.read(P.SPACE_DEBUG, P.debug_addr(_thread(thread), 0), self.DUMP_WORDS)
        out: Dict[str, int] = {}
        for name, reg in P.DEBUG.items():
            if reg < self.DUMP_WORDS:
                out[name] = words[reg]
        for number, csr in sorted(self.isa.csrs.items()):
            if number < P.DEBUG_CSR_COUNT and csr["name"] not in out:
                out[csr["name"]] = words[P.DEBUG_CSR_BASE + number]
        out["RS1"], out["DEPTH"] = P.unpack_rs1_depth(out.pop("RS1_DEPTH"))
        lat = P.unpack_lat(out.pop("LAT"))
        out["LAT_VALID"], out["LAT_VAL"], out["LAT_PIN"] = lat["valid"], lat["value"], lat["pin"]
        out["INQ_CNT"], out["OUTQ_CNT"] = P.unpack_fifo_counts(out.pop("FIFO_CNT"))
        run, halted = self.read(P.SPACE_CTRL, P.CTRL["RUN"], 2)
        badop = self.read_ctrl("BADOP")
        out["RUN"] = (run >> thread) & 1
        out["HALTED"] = (halted >> thread) & 1
        out["BADOP"] = (badop >> thread) & 1
        return out

    # ---------------------------------------------------------------- CSRs
    def _csr_number(self, csr: Union[str, int]) -> int:
        if isinstance(csr, int):
            return csr
        try:
            return self._csr_numbers[csr.upper()]
        except KeyError:
            raise LoomError("no CSR named %r" % (csr,)) from None

    _GLOBAL_CSRS = {"OD_MASK": "OD_MASK", "PIN_OUT": "PIN_OUT", "PIN_OE": "PIN_OE",
                    "PIN_IN": "PIN_IN", "SFLAGS": "SFLAGS"}

    def read_csr(self, thread: int, csr: Union[str, int]) -> int:
        """A CSR by name or number: per-thread CSRs through the DEBUG window,
        global ones (``OD_MASK``, ``PIN_*``, ``SFLAGS``) through CTRL."""
        number = self._csr_number(csr)
        if number < P.DEBUG_CSR_COUNT:
            return self.read(P.SPACE_DEBUG,
                             P.debug_addr(_thread(thread), P.DEBUG_CSR_BASE + number))[0]
        name = self.isa.csrs.get(number, {}).get("name")
        if name in self._GLOBAL_CSRS:
            return self.read_ctrl(self._GLOBAL_CSRS[name])
        return 0                              # write-only and reserved CSRs read 0

    def write_csr(self, thread: int, csr: Union[str, int], value: int) -> None:
        number = self._csr_number(csr)
        if number < P.DEBUG_CSR_COUNT:
            self.write_reg(thread, P.DEBUG_CSR_BASE + number, value)
            return
        name = self.isa.csrs.get(number, {}).get("name")
        if name in ("OD_MASK", "PIN_OUT", "PIN_OE", "SFLAGS"):
            self.write_ctrl(name, value)      # SFLAGS: sets the bits, as CSRW does
        else:
            raise LoomError("CSR %r cannot be written by the host" % (csr,))

    # --------------------------------------------------------------- FIFOs
    def _fifo_depth(self) -> int:
        caps = self.caps()
        if not caps["fifos"]:
            raise LoomStateError("this build has no FIFOs (CAPS=0x%04X)" % caps["raw"])
        return caps["fifo_depth"]

    def fifo_status(self, thread: int) -> Dict[str, int]:
        self._fifo_depth()
        status = P.unpack_fifo_status(self.read(P.SPACE_FIFO, P.fifo_addr(thread, True))[0])
        status["depth"] = self._fifo_depth()
        return status

    def push(self, thread: int, words: Union[bytes, Iterable[int]], *,
             check: bool = True, max_polls: Optional[int] = None) -> None:
        """Push words (or the bytes of a ``bytes``) into ``INQ[thread]``.

        With ``check`` the status is read first and only as many words are
        sent as fit, polling until everything is in; without it the words
        go out in one transaction and a full INQ drops them (``BADOP[14]``).
        """
        words = list(words)
        depth = self._fifo_depth()
        if not check:
            self.write(P.SPACE_FIFO, P.fifo_addr(thread), words)
            return
        polls = 0
        while words:
            free = depth - self.fifo_status(thread)["inq"]
            if free > 0:
                self.write(P.SPACE_FIFO, P.fifo_addr(thread), words[:free])
                words = words[free:]
                polls = 0
                continue
            polls += 1
            if polls > (max_polls or self.max_polls):
                raise LoomTimeout("INQ[%d] stayed full; %d word(s) not sent"
                                  % (thread, len(words)))

    def pop(self, thread: int, count: int = 1, *, max_polls: Optional[int] = None) -> List[int]:
        """Pop exactly ``count`` words from ``OUTQ[thread]``, polling for them."""
        out: List[int] = []
        polls = 0
        while len(out) < count:
            ready = min(self.fifo_status(thread)["outq"], count - len(out))
            if ready > 0:
                out += self.read(P.SPACE_FIFO, P.fifo_addr(thread), ready)
                polls = 0
                continue
            polls += 1
            if polls > (max_polls or self.max_polls):
                raise LoomTimeout("OUTQ[%d] stayed empty; got %d of %d word(s)"
                                  % (thread, len(out), count))
        return out

    def pop_available(self, thread: int) -> List[int]:
        """Whatever ``OUTQ[thread]`` holds now (possibly nothing)."""
        ready = self.fifo_status(thread)["outq"]
        return self.read(P.SPACE_FIFO, P.fifo_addr(thread), ready) if ready else []

    def pop_raw(self, thread: int, count: int = 1) -> List[int]:
        """Pop without checking: an empty OUTQ reads 0 and sets ``BADOP[14]``."""
        self._fifo_depth()
        return self.read(P.SPACE_FIFO, P.fifo_addr(thread), count)

    # ------------------------------------------------------ flags, errors
    def sflags(self) -> int:
        return self.read_ctrl("SFLAGS") & 0xFF

    def set_sflags(self, mask: int) -> None:
        self.write_ctrl("SFLAGS", mask & 0xFF)

    def clear_sflags(self, mask: int) -> None:
        self.write_ctrl("SFLAGS_CLR", mask & 0xFF)

    def badop(self, clear: bool = True) -> int:
        """``BADOP``: bit t reserved opcode in thread t, bit 14 host FIFO error,
        bit 15 host access error. With ``clear`` the bits read are cleared."""
        value = self.read_ctrl("BADOP")
        if clear and value:
            self.clear_badop(value)
        return value

    def clear_badop(self, mask: int = 0xFFFF) -> None:
        self.write_ctrl("BADOP", mask)

    # ----------------------------------------------------------------- IRQ
    def irq_enable(self, mask: int) -> None:
        """``IRQ_EN`` over ``IRQ_STAT = {SFLAGS, INQ_NOT_FULL, OUTQ_NOT_EMPTY}``."""
        self.write_ctrl("IRQ_EN", mask)

    def irq_enable2(self, mask: int) -> None:
        """``IRQ_EN2`` over ``IRQ_STAT2 = HALTED``."""
        self.write_ctrl("IRQ_EN2", mask & 0xF)

    def irq_status(self) -> Dict[str, int]:
        stat = self.read_ctrl("IRQ_STAT")
        out = P.unpack_irq_stat(stat)
        out.update(raw=stat, halted=self.read_ctrl("IRQ_STAT2") & 0xF,
                   swirq=self.swirq(), en=self.read_ctrl("IRQ_EN"),
                   en2=self.read_ctrl("IRQ_EN2"))
        return out

    def swirq(self) -> int:
        return self.read_ctrl("SWIRQ") & 0xF

    def clear_swirq(self, mask: int = 0xF) -> None:
        self.write_ctrl("SWIRQ", mask & 0xF)

    def irq_pending(self) -> bool:
        """The HOST_IRQ level: from the pin if the transport sees it, else
        recomputed from the registers (SEMANTICS 6.8)."""
        level = self.transport.irq()
        if level is not None:
            return bool(level)
        s = self.irq_status()
        return bool((s["raw"] & s["en"]) or (s["halted"] & s["en2"]) or s["swirq"])

    # ---------------------------------------------------------------- pins
    def pin_in(self) -> int:
        return self.read_ctrl("PIN_IN")

    def pin_out(self) -> int:
        return self.read_ctrl("PIN_OUT")

    def set_pin_out(self, value: int) -> None:
        self.write_ctrl("PIN_OUT", value)

    def pin_oe(self) -> int:
        return self.read_ctrl("PIN_OE") & 0xFF

    def set_pin_oe(self, value: int) -> None:
        self.write_ctrl("PIN_OE", value & 0xFF)

    def od_mask(self) -> int:
        return self.read_ctrl("OD_MASK") & 0xFF

    def set_od_mask(self, value: int) -> None:
        self.write_ctrl("OD_MASK", value & 0xFF)

    # ---------------------------------------------------------------- time
    def delay(self, seconds: float) -> None:
        self.transport.delay(seconds)


def _thread(thread: int) -> int:
    if not isinstance(thread, int) or not 0 <= thread < P.THREADS:
        raise ValueError("thread %r outside 0..3" % (thread,))
    return thread


def _names(mask: int) -> str:
    return ",".join(str(t) for t in range(P.THREADS) if (mask >> t) & 1)
