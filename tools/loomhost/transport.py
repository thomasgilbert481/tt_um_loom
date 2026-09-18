"""Transports: the one line below which the host library differs per target.

A transport moves one SPI transaction: ``transfer(tx) -> rx`` clocks the
bytes ``tx`` out on MOSI with chip select low around the whole call and
returns the bytes that came back on MISO. Everything above that line
(``tools.loomhost.loom.Loom``) is shared by the golden model, the FPGA and the
chip.

* :class:`ModelTransport` runs the transaction against the golden model
  (``tools.loomsim``), advancing it a realistic number of clocks per byte so
  that firmware keeps running while the host talks.
* :class:`PicoTransport` and :class:`TTBoardTransport` (``serial_transport``)
  talk to real hardware over USB serial.
"""

from __future__ import annotations

from typing import Iterable, Optional

from tools.loomsim import Machine
from tools.protomodels.bench import Bench

from . import protocol as P


class TransportError(RuntimeError):
    """The transport could not complete a transaction."""


class Transport:
    """Base class. Subclasses implement :meth:`transfer`."""

    #: Core clock, used to convert :meth:`delay` seconds into model clocks.
    clk_hz = 50_000_000

    def transfer(self, tx: bytes) -> bytes:
        raise NotImplementedError

    def irq(self) -> Optional[bool]:
        """Level of the HOST_IRQ pin, or ``None`` if this transport cannot see it."""
        return None

    def delay(self, seconds: float) -> None:
        """Let time pass on the target (real transports sleep)."""
        import time
        time.sleep(max(0.0, seconds))

    def close(self) -> None:
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# ------------------------------------------------------------------ model
class ModelTransport(Transport):
    """Runs host transactions against the golden model, clock by clock.

    ``target`` is a :class:`tools.protomodels.bench.Bench` (use this when pin
    models are attached, so they keep running during host traffic), a bare
    :class:`tools.loomsim.Machine`, or ``None`` to build a machine from
    ``image``/``features``. Timing, all in core clocks:

    * ``clocks_per_byte`` (default 64: SCK = clk/8, the protocol's fastest);
    * ``cs_setup`` / ``cs_hold``: CS_n low before the first and after the last
      SCK edge (HOST_PROTOCOL: at least 4 each);
    * ``cs_gap``: CS_n high time after the transaction.

    A write word commits when its second byte completes (HOST_PROTOCOL: at
    the end of each complete word). A read word is fetched when the byte
    before its first data byte completes. A FIFO-space read follows the M2
    RTL's reading (``docs/spec-questions/rtl-m2.md`` 1): the word is a peek
    at the head when it is loaded, the pop commits when its last byte has
    gone out, a word cut short by CS_n removes nothing, and the word loaded
    while the previous one is being popped is the entry after the head. An
    empty peek reads 0 and sets ``BADOP[14]`` at the end of the word. Each
    host action is issued in the cycle after the byte's last clock and
    commits at the edge that ends that cycle, like a real host write
    (SEMANTICS 7).

    SHIM. The M2 golden model does not implement every register of
    ``docs/HOST_PROTOCOL.md`` (``docs/spec-questions/firmware.md``). This
    class supplies the missing ones itself, following HOST_PROTOCOL and
    SEMANTICS 6.7/6.8, without touching the model's code:

    * ``ID``, ``VERSION`` (constants, ``VERSION`` as the M1 RTL reports it);
    * ``IRQ_EN``, ``IRQ_EN2`` (plain registers here), ``IRQ_STAT`` and
      ``IRQ_STAT2`` (computed from the model's state);
    * ``SWIRQ``: set when a retire record shows a ``CSRW HOST_IRQ``, cleared
      by the host's write-1-to-clear;
    * ``BADOP[14]``: set on a host push to a full INQ or a pop from an empty
      OUTQ, cleared through ``BADOP`` like the model's own bits;
    * ``CTRL.RESET`` also empties both FIFOs of the thread (SEMANTICS 6.7):
      the model's public ``ThreadState`` FIFO lists are cleared at the reset
      edge (a thread may only be reset while halted, so nothing races it);
    * debug writes are dropped unless the thread is halted (the model only
      gates ``r0..r7``); ``STEPS`` writes are kept as an offset; the CSR
      window 0x10..0x1F is mapped by number onto the model's CSR names, and
      unbuilt CSRs (bit engine) read 0 and ignore writes;
    * the HOST_IRQ level (:meth:`irq`) is computed from the shim and the
      model, from the current cycle (the pin itself is one cycle later).
    """

    def __init__(self, target=None, *, image=None, features: Iterable[str] = ("FIFO",),
                 clocks_per_byte: int = 64, cs_setup: int = 4, cs_hold: int = 4,
                 cs_gap: int = 8, version: int = 0x0001, clk_hz: int = 50_000_000,
                 **machine_kwargs) -> None:
        if isinstance(target, Bench):
            self.bench = target
        elif isinstance(target, Machine) or target is None:
            self.bench = Bench(target, image=image, features=features, **machine_kwargs)
        else:
            raise TypeError("target must be a Bench, a Machine or None")
        if clocks_per_byte < 64:
            raise ValueError("an SPI byte takes at least 64 clocks (SCK <= clk/8)")
        self.machine = self.bench.machine
        self.isa = self.bench.isa
        self.clocks_per_byte = clocks_per_byte
        self.cs_setup = cs_setup
        self.cs_hold = cs_hold
        self.cs_gap = cs_gap
        self.version = version & 0xFFFF
        self.clk_hz = clk_hz
        self.transactions = 0
        # --- shim state (see the class docstring)
        self.irq_en = 0
        self.irq_en2 = 0
        self.swirq = 0
        self.badop_shim = 0
        self.steps_offset = [0] * P.THREADS
        self._fifo_reset = 0
        self._csrw = self.isa.by_name["CSRW"]
        self._csr_host_irq = self.isa.csr_by_name["HOST_IRQ"]
        self._csr_names = dict(self.isa.csrs)
        self.bench.add_observer(self._observe)

    # ------------------------------------------------------------- clock
    @property
    def cycle(self) -> int:
        return self.machine.cycle

    def idle(self, cycles: int) -> None:
        """Advance the model with chip select high."""
        self.bench.step(cycles)

    def delay(self, seconds: float) -> None:
        self.idle(max(0, int(round(seconds * self.clk_hz))))

    def _observe(self, record) -> None:
        if self._fifo_reset:
            # The CTRL.RESET commit landed at the edge that just passed.
            for t in range(P.THREADS):
                if (self._fifo_reset >> t) & 1:
                    self.machine.threads[t].inq.clear()
                    self.machine.threads[t].outq.clear()
            self._fifo_reset = 0
        if record is not None and record.mnemonic == "CSRW" and record.done:
            if self._csrw.decode_fields(record.ir)["csr"] == self._csr_host_irq:
                self.swirq |= 1 << record.thread

    @property
    def fifos_built(self) -> bool:
        return "FIFO" in self.machine.features

    # ---------------------------------------------------------- transfer
    def transfer(self, tx: bytes) -> bytes:
        tx = bytes(tx)
        events = P.chip_events(tx)
        rx = bytearray(len(tx))
        self.bench.step(self.cs_setup)
        k = 0
        finish = None                     # (byte count, thread, peeked) of a FIFO read
        for done_bytes in range(len(tx) + 1):
            popping = None
            if finish is not None and finish[0] == done_bytes:
                _, thread, peeked = finish
                finish = None
                if peeked:
                    self.machine.host_fifo_pop(thread)   # commits at this edge
                    popping = thread
                else:
                    self.badop_shim |= P.BADOP_FIFO      # SHIM: SEMANTICS 6.7
            while k < len(events) and events[k].after_byte == done_bytes:
                ev = events[k]
                k += 1
                if ev.kind == "write":
                    self._write(ev.space, ev.addr, ev.word)
                    continue
                if self._is_fifo_data(ev.space, ev.addr):
                    value, peeked = self._fifo_peek(ev.addr, ev.addr == popping)
                    finish = (done_bytes + 2, ev.addr, peeked)
                else:
                    value = self._read(ev.space, ev.addr) & 0xFFFF
                rx[done_bytes] = value >> 8
                if done_bytes + 1 < len(tx):
                    rx[done_bytes + 1] = value & 0xFF
            if done_bytes < len(tx):
                self.bench.step(self.clocks_per_byte)
        self.bench.step(self.cs_hold + self.cs_gap)
        self.transactions += 1
        return bytes(rx)

    def _is_fifo_data(self, space: int, addr: int) -> bool:
        return space == P.SPACE_FIFO and self.fifos_built and addr < P.THREADS

    def _fifo_peek(self, thread: int, popping: bool):
        """The word a FIFO read loads: the head, or the entry after it while
        the head is being popped at this same edge. ``(value, found)``."""
        queue = self.machine.threads[thread].outq
        index = 1 if popping else 0
        if len(queue) > index:
            return queue[index], True
        return 0, False

    # ------------------------------------------------------------ writes
    def _write(self, space: int, addr: int, word: int) -> None:
        m = self.machine
        if space == P.SPACE_CTRL:
            self._write_ctrl(addr, word)
        elif space == P.SPACE_IMEM:
            m.host_write_imem(addr, word)
        elif space == P.SPACE_FIFO:
            if not self.fifos_built:
                return                              # M1: ignored
            if addr < P.THREADS:
                if len(m.threads[addr].inq) < m.fifo_depth:
                    m.host_fifo_push(addr, word)
                else:
                    self.badop_shim |= P.BADOP_FIFO  # SHIM: SEMANTICS 6.7
        elif space == P.SPACE_DEBUG:
            self._write_debug((addr >> 8) & 3, addr & 0xFF, word)
        elif space == P.SPACE_STEP:
            m.host_step(addr & 3)
        # DMEM (not built) and unknown spaces ignore writes.

    def _write_ctrl(self, addr: int, word: int) -> None:
        m = self.machine
        name = P.CTRL_NAMES.get(addr)
        if name == "RUN":
            m.host_set_run(word & 0xF)
        elif name == "RESET":
            for t in range(P.THREADS):
                if (word >> t) & 1:
                    m.host_reset_thread(t)
            if self.fifos_built:
                self._fifo_reset |= word & 0xF       # SHIM: SEMANTICS 6.7
        elif name is not None and name.startswith("RESET_PC"):
            m.host_write_reset_pc(int(name[-1]), word)
        elif name == "IRQ_EN":
            self.irq_en = word                       # SHIM: SEMANTICS 6.8
        elif name == "IRQ_EN2":
            self.irq_en2 = word & 0xF                # SHIM
        elif name == "SFLAGS":
            m.host_write_sflags_set(word)
        elif name == "SFLAGS_CLR":
            m.host_write_sflags_clr(word)
        elif name == "OD_MASK":
            m.host_write_od_mask(word)
        elif name == "PIN_OUT":
            m.host_write_pin_out(word)
        elif name == "PIN_OE":
            m.host_write_pin_oe(word)
        elif name == "BADOP":
            m.host_clear_badop(word)
            self.badop_shim &= ~word                 # SHIM: bit 14
        elif name == "SWIRQ":
            self.swirq &= ~word                      # SHIM: write 1 to clear
        # ID, VERSION, HALTED, IRQ_STAT*, PIN_IN, CAPS are read-only.

    def _write_debug(self, t: int, reg: int, word: int) -> None:
        m = self.machine
        # SHIM: HOST_PROTOCOL space 4 - every debug register is writable only
        # while the thread is halted; the model gates r0..r7 only.
        if not m.thread_halted_for_debug(t):
            return
        name = self._debug_name(reg)
        if name is None or reg in P.DEBUG_READ_ONLY:
            return
        if name == "STEPS":
            self.steps_offset[t] = (word - m.threads[t].steps) & 0xFFFF   # SHIM
        elif name == "RS1_DEPTH":
            rs1, depth = P.unpack_rs1_depth(word)
            m.host_write_debug(t, "RS1", rs1)
            m.host_write_debug(t, "DEPTH", depth)
        elif name in _MODEL_WRITABLE:
            m.host_write_debug(t, name, word)
        # NOW, TID, unbuilt CSRs, TICK_SEEN, LAT and FIFO_CNT ignore writes.

    def _debug_name(self, reg: int) -> Optional[str]:
        """The model's name for a debug register, or None if the model has none."""
        if P.DEBUG_CSR_BASE <= reg < P.DEBUG_CSR_BASE + P.DEBUG_CSR_COUNT:
            csr = self._csr_names.get(reg - P.DEBUG_CSR_BASE)
            name = csr["name"] if csr else None
            return name if name in _MODEL_CSRS else None
        return P.DEBUG_NAMES.get(reg)

    # ------------------------------------------------------------- reads
    def _read(self, space: int, addr: int) -> int:
        m = self.machine
        if space == P.SPACE_CTRL:
            return self._read_ctrl(addr)
        if space == P.SPACE_IMEM:
            return m.host_read_imem(addr)
        if space == P.SPACE_FIFO:
            # Data reads (0x0000+t) are handled in transfer(): peek and pop.
            if not self.fifos_built:
                return 0
            if P.FIFO_STATUS <= addr < P.FIFO_STATUS + P.THREADS:
                th = m.threads[addr - P.FIFO_STATUS]
                return P.pack_fifo_status(len(th.inq), len(th.outq), m.fifo_depth)
            return 0
        if space == P.SPACE_DEBUG:
            return self._read_debug((addr >> 8) & 3, addr & 0xFF)
        return 0

    def _read_ctrl(self, addr: int) -> int:
        m = self.machine
        name = P.CTRL_NAMES.get(addr)
        if name == "ID":
            return P.ID_VALUE
        if name == "VERSION":
            return self.version
        if name == "RUN":
            return m.run
        if name in ("HALTED", "IRQ_STAT2"):
            return m.halted
        if name is not None and name.startswith("RESET_PC"):
            return m.reset_pc[int(name[-1])]
        if name == "IRQ_EN":
            return self.irq_en
        if name == "IRQ_EN2":
            return self.irq_en2
        if name == "IRQ_STAT":
            return self.irq_stat()
        if name == "SFLAGS":
            return m.sflags
        if name == "OD_MASK":
            return m.od_mask
        if name == "PIN_OUT":
            return m.pin_out
        if name == "PIN_OE":
            return m.pin_oe
        if name == "PIN_IN":
            return m.pin_in_word
        if name == "CAPS":
            return m.caps
        if name == "BADOP":
            return (m.badop | self.badop_shim) & 0xFFFF
        if name == "SWIRQ":
            return self.swirq
        return 0

    def _read_debug(self, t: int, reg: int) -> int:
        m = self.machine
        th = m.threads[t]
        if reg == P.DEBUG["STEPS"]:
            return (th.steps + self.steps_offset[t]) & 0xFFFF
        if reg == P.DEBUG["RS1_DEPTH"]:
            return P.pack_rs1_depth(th.rs1, th.depth)
        if reg == P.DEBUG["FIFO_CNT"]:
            return P.pack_fifo_counts(len(th.inq), len(th.outq)) if self.fifos_built else 0
        if reg == P.DEBUG["LAT"]:
            return 0                                 # SETP ... D is not built
        name = self._debug_name(reg)
        if name is None or name not in _MODEL_READABLE:
            return 0
        return m.host_read_debug(t, name)

    # --------------------------------------------------------------- IRQ
    def irq_stat(self) -> int:
        m = self.machine
        inq_nf = outq_ne = 0
        if self.fifos_built:
            for t, th in enumerate(m.threads):
                inq_nf |= int(len(th.inq) < m.fifo_depth) << t
                outq_ne |= int(len(th.outq) > 0) << t
        return P.pack_irq_stat(m.sflags, inq_nf, outq_ne)

    def irq(self) -> bool:
        return bool((self.irq_stat() & self.irq_en)
                    or (self.machine.halted & self.irq_en2) or self.swirq)


#: Debug-register names the model's host_read_debug / host_write_debug know.
_MODEL_READABLE = frozenset({
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "PC", "FLAGS", "TD", "NOW",
    "RS0", "WAIT_ACTIVE", "DT", "TICK_SEEN", "TICK_INT", "TICK_FRAC", "OUTGRP",
    "INGRP", "TID"})
_MODEL_WRITABLE = frozenset({
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "PC", "FLAGS", "TD", "RS0",
    "WAIT_ACTIVE", "DT", "TICK_INT", "TICK_FRAC", "OUTGRP", "INGRP"})
#: CSRs of the debug window that the M2 model builds (others read 0).
_MODEL_CSRS = frozenset({"TICK_INT", "TICK_FRAC", "OUTGRP", "INGRP", "NOW", "TD",
                         "FLAGS", "TID"})
