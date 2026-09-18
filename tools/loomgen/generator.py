"""Constrained-random Loom program generator (``docs/VERIFICATION.md`` L2-RAND).

The generator emits whole instruction-memory images that are safe to run for a
bounded number of cycles without supervision:

* **Control flow never leaves a thread's region.** Every ``JMP``/``CALL``
  target is an address inside the region, every ``Bcc``/``DJNZ``/``JP``
  offset resolves to one, and the last word of the region is a ``JMP`` back to
  the loop head, so a thread that falls through stays alive instead of
  wandering into its neighbour. ``RET`` can only return to an address a
  ``CALL`` in the same region pushed. :func:`check_program` proves all of this
  on every image before it is returned.
* **Branch targets are block heads.** A block is a short run of words that has
  to execute together, such as ``LDI r7, n`` followed by ``CSRW TICK_INT, r7``.
  Nothing ever jumps into the middle of one, so a ``CSRW`` of a divider or a
  deadline always sees the value its own ``LDI`` put there.
* **Loops are bounded.** ``DJNZ`` loops load a small count first; a free
  ``DJNZ``, ``BT``/``BNT`` and ``JP`` only branch forwards (a backward one
  whose condition cannot change would park the thread for good); other
  conditional branches go backwards only occasionally; nothing branches to
  itself except a ``JP`` poll on a pin the stimulus moves.
* **Waits finish.** ``WAITD``/``DLY``/``SETD`` immediates and ``TICK_INT`` are
  small, conditional waits are either timed or watch something that will
  happen (a pin the stimulus plan moves, a shared flag other threads use),
  and only a small minority are untimed waits that may never complete, which
  is a legitimate thing for the harness to see.

Everything an instruction encodes goes through :mod:`tools.loomisa`; there is
no instruction hex in this file.

Reproducibility: ``generate(seed, ...)`` is a pure function of its arguments.
Two calls with the same arguments give identical images and plans, in any
process (the RNG is seeded from a string, not from ``hash()``).
"""

from __future__ import annotations

import dataclasses
import random
from typing import Dict, FrozenSet, Iterable, List, Optional, Sequence, Tuple

from tools.loomisa import Isa
from tools.loomisa import load as load_isa

from .stimulus import StimulusPlan, build_plan

THREADS = 4
PC_MASK = 0x3FF

#: Every mnemonic the M1 build implements (SEMANTICS scope markers). The
#: generator emits all of them; ``tools/tests/test_loomgen_*`` checks the list
#: against ``isa/isa.yaml`` so a new instruction cannot be silently skipped.
M1_MNEMONICS: Tuple[str, ...] = (
    "ADD", "SUB", "AND", "OR", "XOR", "SHL", "SHR", "ROR",
    "ADDI", "SUBI", "ANDI", "ORI", "XORI", "SHLI", "SHRI", "CMPI",
    "LDI", "LDIH", "MOV", "NOT", "NEG", "CMP", "TEST", "REV", "PAR", "SWAP",
    "JMP", "CALL", "RET", "HALT",
    "BZ", "BNZ", "BC", "BNC", "BT", "BNT", "DJNZ", "JP",
    "SETP", "OEP", "OUT", "IN",
    "WAITD", "WAITP", "WAITE", "WAITS", "DLY", "SETD", "NOP",
    "CSRR", "CSRW", "CLR", "SIG",
)

#: Instructions that decode but whose feature M1 does not build: the FIFO
#: ops and WAITB (M2), the bit engine (M2/M3) and LD/ST (optional DMEM). They
#: execute as ``NOP`` and set ``BADOP`` (SEMANTICS 9).
UNBUILT_MNEMONICS: Tuple[str, ...] = (
    "PUSH", "POP", "WAITB", "SHO", "SHI", "LDSR", "STSR", "CRCI", "STCRC",
    "LD", "ST",
)

#: Instructions the M2 RTL builds (SEMANTICS 6.7, 6.9): FIFOs, WAITB and the
#: manual bit engine. With the ``m2_built`` avoid flag they leave the unbuilt
#: pool as well, so a program means the same thing to an M2 build and to a
#: golden model that still treats them as unbuilt.
M2_MNEMONICS: Tuple[str, ...] = (
    "PUSH", "POP", "WAITB", "SHO", "SHI", "LDSR", "STSR", "CRCI", "STCRC",
)

#: CSRs built at M1 (SEMANTICS 6.6: 0x00-0x03, 0x09-0x0C, 0x10-0x15), by name.
M1_CSR_NAMES: Tuple[str, ...] = (
    "TICK_INT", "TICK_FRAC", "OUTGRP", "INGRP", "NOW", "TD", "FLAGS", "TID",
    "OD_MASK", "PIN_OUT", "PIN_OE", "PIN_IN", "SFLAGS", "HOST_IRQ",
)
#: CSRs of unbuilt features: they read 0 and ignore writes, without BADOP.
UNBUILT_CSR_NAMES: Tuple[str, ...] = (
    "BE_CFG", "BE_PINS", "BE_RELOAD", "CRC_POLY", "CRC_INIT", "SR", "CNT", "CRC",
)
#: The bit-engine CSRs, which an M2 build implements (SEMANTICS 6.9). The
#: ``m2_built`` avoid flag takes them out of the "reads 0" pool.
M2_CSR_NAMES: Tuple[str, ...] = UNBUILT_CSR_NAMES

#: Register 7 is the generator's scratch: no random instruction writes it, so
#: the only values it ever holds are the ones a ``LDI``/``LDIH`` or ``CSRR
#: NOW`` in the same block put there.
SCRATCH = 7
VALUE_REGS = tuple(range(0, SCRATCH))           # r0..r6
ALL_REGS = tuple(range(0, 8))

ALU3 = ("ADD", "SUB", "AND", "OR", "XOR", "SHL", "SHR", "ROR")
ALUI = ("ADDI", "SUBI", "ANDI", "ORI", "XORI", "SHLI", "SHRI", "CMPI")
UNARY = ("MOV", "NOT", "NEG", "CMP", "TEST", "REV", "PAR", "SWAP")
#: Conditional branches on Z and C: backward targets allowed now and then.
ZC_BRANCHES = ("BZ", "BNZ", "BC", "BNC")
#: Branches on T: T only changes on timed waits and CSRW FLAGS, so a backward
#: BT/BNT is almost always an infinite loop. Forward only.
T_BRANCHES = ("BT", "BNT")

#: Known ``avoid`` flags. Each switches off one construct on which the RTL
#: and the model disagree where ``docs/SEMANTICS.md`` does not decide the
#: answer, until the director rules; see ``docs/spec-questions/cosim.md``.
#:
#: ``m2_built`` is different in kind: it marks a run against an RTL that
#: builds M2 features (``CAPS`` says so) while the golden model may not. It
#: removes :data:`M2_MNEMONICS` from every pool, keeps the bit-engine CSRs
#: out of the dead-CSR pool, and never sets the ``D`` (lat) bit of ``SETP``.
AVOID_FLAGS: FrozenSet[str] = frozenset((
    "csrw_pin_out_high_bits",
    "m2_built",
))


class LoomgenError(ValueError):
    """Bad generator arguments, or an image that breaks a generator rule."""


# --------------------------------------------------------------------- blocks
@dataclasses.dataclass
class _Word:
    """One instruction word, either final or waiting for a branch target.

    ``fix`` is ``""`` for a final word, ``"abs"`` (JMP/CALL to a block
    head), ``"loop"`` (the region's closing JMP), ``"self"`` (absolute target
    at a fixed ``offset`` from the word itself), ``"rel8"`` or ``"rel6"``
    (PC-relative to a block head). ``direction`` is ``"fwd"`` or ``"any"``.
    """

    word: Optional[int] = None
    fix: str = ""
    mnemonic: str = ""
    operands: Dict[str, int] = dataclasses.field(default_factory=dict)
    direction: str = "any"
    offset: int = 0


@dataclasses.dataclass
class _Block:
    """A run of words that must execute together; only its head is a target."""

    words: List[_Word]
    targetable: bool = True
    addr: int = 0

    def __len__(self) -> int:
        return len(self.words)


# ----------------------------------------------------------------- the result
@dataclasses.dataclass
class GeneratedProgram:
    """One random program plus everything needed to replay it exactly."""

    seed: int
    profile: str
    threads: int
    imem_words: int
    #: ``{address: word}`` for every word of every running thread's region.
    image: Dict[int, int]
    #: Reset vector of every thread, including the ones that are not started.
    entries: List[int]
    #: ``CTRL.RUN`` value the harness writes.
    run_mask: int
    stimulus: StimulusPlan
    avoid: Tuple[str, ...] = ()
    #: Addresses that are branch targets (block heads), for the listing.
    heads: FrozenSet[int] = frozenset()

    @property
    def stride(self) -> int:
        return max(self.imem_words // THREADS, 1)

    @property
    def default_entries(self) -> List[int]:
        return [t * self.stride for t in range(THREADS)]

    def region(self, thread: int) -> Tuple[int, int]:
        """``[start, end)`` of the thread's code: its entry plus one stride."""
        start = self.entries[thread]
        return start, start + self.stride

    def running(self) -> List[int]:
        return [t for t in range(THREADS) if (self.run_mask >> t) & 1]

    def to_obj(self) -> Dict:
        """The assembler's JSON image shape plus the co-simulation extras."""
        return {
            "generator": "tools.loomgen",
            "seed": self.seed,
            "profile": self.profile,
            "imem_words": self.imem_words,
            "words": {str(a): self.image[a] for a in sorted(self.image)},
            "symbols": {},
            "threads": {
                str(t): {"entry": self.entries[t], "size": self.stride,
                         "run": bool((self.run_mask >> t) & 1)}
                for t in range(THREADS)
            },
            "run_mask": self.run_mask,
            "avoid": list(self.avoid),
            "heads": sorted(self.heads),
            "stimulus": self.stimulus.to_obj(),
            "source": "generated by tools.loomgen seed %d profile %s"
                      % (self.seed, self.profile),
        }

    @staticmethod
    def from_obj(obj: Dict) -> "GeneratedProgram":
        image = {int(k, 0) if isinstance(k, str) else int(k): int(v)
                 for k, v in obj["words"].items()}
        threads = dict(obj["threads"])
        entries = [int(threads[str(t)]["entry"]) for t in range(THREADS)]
        run_mask = int(obj["run_mask"])
        return GeneratedProgram(
            seed=int(obj["seed"]), profile=str(obj["profile"]),
            threads=bin(run_mask).count("1"),
            imem_words=int(obj["imem_words"]), image=image, entries=entries,
            run_mask=run_mask,
            stimulus=StimulusPlan.from_obj(obj["stimulus"]),
            avoid=tuple(obj.get("avoid", ())),
            heads=frozenset(int(a) for a in obj.get("heads", ())))

    def disassembly(self, isa: Optional[Isa] = None) -> str:
        """An annotated listing of every word, for debugging a failing seed."""
        from tools.loomasm.disasm import disassemble

        isa = isa or load_isa()
        lines = ["; tools.loomgen seed=%d profile=%s threads=%d imem_words=%d"
                 % (self.seed, self.profile, self.threads, self.imem_words),
                 "; run_mask=0x%X avoid=%s" % (self.run_mask,
                                               ",".join(self.avoid) or "-"),
                 "; '>' marks a block head (a branch target)"]
        for t in range(THREADS):
            start, end = self.region(t)
            if not any(start <= a < end for a in self.image):
                continue
            lines.append("")
            lines.append("; ---- thread %d  region %03X..%03X  entry %03X  %s"
                         % (t, start, end - 1, self.entries[t],
                            "RUN" if (self.run_mask >> t) & 1 else "idle"))
            for addr in range(start, end):
                if addr not in self.image:
                    continue
                word = self.image[addr]
                mark = ">" if addr in self.heads else " "
                lines.append("  %03X %s %04X  %s"
                             % (addr, mark, word, disassemble(word, isa)))
        return "\n".join(lines) + "\n"


# ------------------------------------------------------------------- profiles
#: Relative weight of every block emitter, per profile. Names are the
#: ``_emit_*`` methods of :class:`_ThreadBuilder`.
PROFILES: Dict[str, Dict[str, int]] = {
    "mixed": {
        "alu3": 10, "alui": 10, "ldi": 5, "ldih": 3, "unary": 9,
        "branch": 9, "tbranch": 3, "djnz_loop": 3, "djnz_fwd": 2, "jp": 4,
        "jp_poll": 1, "jmp": 2, "call": 4, "call_chain": 1, "ret": 4,
        "setp": 8, "oep": 3, "out": 4, "in": 4, "grpcfg": 2,
        "waitd": 5, "dly": 3, "setd": 3, "nop": 2, "tdrel": 1, "retick": 1,
        "waitp": 4, "waite": 4, "waits": 4, "sig": 4, "clr": 3,
        "csrr": 6, "csrw": 6, "reserved": 2, "unbuilt": 3,
    },
    "alu": {
        "alu3": 26, "alui": 24, "ldi": 8, "ldih": 6, "unary": 22,
        "branch": 14, "tbranch": 3, "djnz_loop": 4, "djnz_fwd": 3, "jp": 2,
        "jp_poll": 1, "jmp": 2, "call": 4, "call_chain": 1, "ret": 4,
        "setp": 3, "oep": 1, "out": 2, "in": 2, "grpcfg": 1,
        "waitd": 2, "dly": 1, "setd": 2, "nop": 2, "tdrel": 1, "retick": 1,
        "waitp": 1, "waite": 1, "waits": 2, "sig": 2, "clr": 1,
        "csrr": 4, "csrw": 4, "reserved": 2, "unbuilt": 2,
    },
    "timing": {
        "alu3": 5, "alui": 5, "ldi": 3, "ldih": 2, "unary": 4,
        "branch": 6, "tbranch": 6, "djnz_loop": 3, "djnz_fwd": 1, "jp": 3,
        "jp_poll": 2, "jmp": 2, "call": 2, "call_chain": 1, "ret": 2,
        "setp": 6, "oep": 1, "out": 2, "in": 2, "grpcfg": 1,
        "waitd": 14, "dly": 9, "setd": 8, "nop": 2, "tdrel": 4, "retick": 4,
        "waitp": 8, "waite": 8, "waits": 8, "sig": 7, "clr": 4,
        "csrr": 5, "csrw": 6, "reserved": 2, "unbuilt": 2,
    },
    "pins": {
        "alu3": 4, "alui": 4, "ldi": 5, "ldih": 3, "unary": 4,
        "branch": 5, "tbranch": 2, "djnz_loop": 2, "djnz_fwd": 1, "jp": 9,
        "jp_poll": 3, "jmp": 2, "call": 2, "call_chain": 1, "ret": 2,
        "setp": 22, "oep": 9, "out": 12, "in": 12, "grpcfg": 7,
        "waitd": 3, "dly": 2, "setd": 3, "nop": 1, "tdrel": 1, "retick": 1,
        "waitp": 10, "waite": 12, "waits": 4, "sig": 3, "clr": 2,
        "csrr": 5, "csrw": 6, "reserved": 2, "unbuilt": 2,
    },
}

#: Probability that a thread's program contains one ``HALT``.
HALT_RATE = 0.12
#: Share of conditional waits that are untimed on something that may never
#: happen (the "gamble"); every other untimed wait watches a live condition.
GAMBLE_RATE = 1.0 / 14.0


# ------------------------------------------------------------- thread builder
class _ThreadBuilder:
    """Builds the blocks of one thread's region."""

    def __init__(self, rng: random.Random, isa: Isa, thread: int,
                 size: int, profile: str, plan: StimulusPlan,
                 others_running: bool, flag_pool: Sequence[int],
                 avoid: FrozenSet[str]):
        self.rng = rng
        self.isa = isa
        self.thread = thread
        self.size = size
        self.profile = profile
        self.others_running = others_running
        self.flag_pool = tuple(flag_pool)
        self.avoid = avoid
        self.blocks: List[_Block] = []
        self.loop_head_index = 0
        csr = isa.csr_by_name
        self.csr = csr
        # Pins that only the stimulus controls and that move: the safe choice
        # for an untimed wait. BIDIR pins may be driven by the chip itself
        # (PIN_OE), which can freeze what the core reads back.
        moving = plan.toggling
        self.live_in = tuple(sorted(p for p in moving if 8 <= p <= 12)) or (8,)
        self.live_any = tuple(sorted(moving)) or self.live_in
        pins = isa.pins
        self.bidir = tuple(sorted(p for p, d in pins.items() if d["dir"] == "bidir"))
        self.inputs = tuple(sorted(p for p, d in pins.items() if d["dir"] == "in"))
        self.outputs = tuple(sorted(p for p, d in pins.items() if d["dir"] == "out"))
        self.reserved_pins = tuple(p for p in range(32) if p not in pins)
        self.reserved_words = _reserved_words(isa, rng)
        self.m2_built = "m2_built" in avoid
        self.unbuilt_cycle = [n for n in UNBUILT_MNEMONICS
                              if not (self.m2_built and n in M2_MNEMONICS)]
        rng.shuffle(self.unbuilt_cycle)
        self.unbuilt_next = 0
        self.m1_csrs = [csr[n] for n in M1_CSR_NAMES]
        # Unbuilt CSRs plus the CSR numbers isa.yaml does not define at all:
        # both read 0 and ignore writes (SEMANTICS 6.6).
        self.dead_csrs = [csr[n] for n in UNBUILT_CSR_NAMES
                          if not (self.m2_built and n in M2_CSR_NAMES)] + \
            [n for n in range(32) if n not in isa.csrs]
        self.csr_cycle = list(self.m1_csrs)
        rng.shuffle(self.csr_cycle)
        self.csr_next = 0

    # ------------------------------------------------------------- utilities
    def enc(self, name: str, **ops: int) -> _Word:
        return _Word(word=self.isa.encode(name, **ops), mnemonic=name)

    def push(self, words: Sequence[_Word], targetable: bool = True) -> None:
        self.blocks.append(_Block(list(words), targetable))

    def one(self, name: str, **ops: int) -> None:
        self.push([self.enc(name, **ops)])

    @staticmethod
    def fixup(fix: str, name: str, direction: str = "any", offset: int = 0,
              **ops: int) -> _Word:
        return _Word(fix=fix, mnemonic=name, operands=dict(ops),
                     direction=direction, offset=offset)

    @property
    def used(self) -> int:
        return sum(len(b) for b in self.blocks)

    @property
    def room(self) -> int:
        """Words still free, keeping one for the closing ``JMP``."""
        return self.size - self.used - 1

    def reg(self) -> int:
        return self.rng.choice(VALUE_REGS)

    def any_reg(self) -> int:
        return self.rng.choice(ALL_REGS)

    def flag(self) -> int:
        if self.flag_pool and self.rng.random() < 0.8:
            return self.rng.choice(self.flag_pool)
        return self.rng.randrange(8)

    def small_imm(self) -> int:
        return self.rng.choice((0, 1, 1, 2, 2, 3, 3, 4, 5, 6, 8, 10, 12))

    def write_pin(self) -> int:
        """A pin index for ``SETP``, drawn from every index class."""
        roll = self.rng.random()
        if roll < 0.42:
            return self.rng.choice(self.bidir)
        if roll < 0.74:
            return self.rng.choice(self.outputs)
        if roll < 0.88:
            return self.rng.choice(self.inputs)            # read-only: ignored
        return self.rng.choice(self.reserved_pins)         # reserved: ignored

    def read_pin(self, live: bool) -> int:
        """A pin index for a wait or ``JP``; ``live`` asks for a moving one."""
        if live:
            if self.rng.random() < 0.75:
                return self.rng.choice(self.live_in)
            return self.rng.choice(self.live_any)
        roll = self.rng.random()
        if roll < 0.45:
            return self.rng.choice(self.inputs)
        if roll < 0.75:
            return self.rng.choice(self.bidir)
        if roll < 0.92:
            return self.rng.choice(self.outputs)
        return self.rng.choice(self.reserved_pins)

    def load_scratch(self, value: int) -> List[_Word]:
        """``LDI``, plus ``LDIH`` when the value does not fit in eight bits."""
        words = [self.enc("LDI", rd=SCRATCH, imm=value & 0xFF)]
        if value > 0xFF:
            words.append(self.enc("LDIH", rd=SCRATCH, imm=(value >> 8) & 0xFF))
        return words

    def csrw_scratch(self, name: str, value: int) -> List[_Word]:
        return self.load_scratch(value) + [
            self.enc("CSRW", csr=self.csr[name], ra=SCRATCH)]

    def tick_int_value(self) -> int:
        """TICK_INT for this profile: 1..40, both multiples of 4 and not."""
        rng = self.rng
        if self.profile == "timing" or rng.random() < 0.25:
            if rng.random() < 0.5:
                return rng.choice((4, 8, 12, 16, 20, 24, 28, 32, 36, 40))
            return rng.choice([n for n in range(1, 41) if n % 4])
        return rng.choice((1, 1, 2, 3, 4, 5, 6, 7, 8))

    # -------------------------------------------------------------- prologue
    def emit_prologue(self) -> None:
        """Set the tick divider, the pin groups and the deadline anchor.

        The prologue is not a branch target, so it runs exactly once: the
        closing ``JMP`` goes to the first block after it.
        """
        rng = self.rng
        self.push(self.csrw_scratch("TICK_INT", self.tick_int_value()),
                  targetable=False)
        if rng.random() < 0.45:
            self.push(self.csrw_scratch("TICK_FRAC", rng.randrange(1, 256)),
                      targetable=False)
        if rng.random() < 0.6:                       # open-drain mode, global
            od = rng.choice((0x00, 0x01, 0x03, 0x0F, 0x55, 0xAA, 0xF0, 0xFF))
            self.push(self.csrw_scratch("OD_MASK", od), targetable=False)
        self.push(self.group_block("OUTGRP"), targetable=False)
        self.push(self.group_block("INGRP"), targetable=False)
        # SEMANTICS 6.4: "Always SETD before the first deadline", otherwise the
        # first WAITD compares against TD = 0 and completes at once.
        self.push([self.enc("SETD", imm=self.small_imm())], targetable=False)
        self.loop_head_index = len(self.blocks)

    def group_block(self, name: str) -> List[_Word]:
        """``OUTGRP``/``INGRP`` = base[4:0] | cnt[9:5], counts 0 and 16 included."""
        base = self.rng.randrange(32)
        count = self.rng.choice((0, 1, 1, 2, 3, 4, 5, 6, 8, 12, 16, 16, 17, 31))
        return self.csrw_scratch(name, (base & 0x1F) | ((count & 0x1F) << 5))

    # ------------------------------------------------------- block emitters
    def _emit_alu3(self) -> None:
        self.one(self.rng.choice(ALU3), rd=self.reg(), ra=self.any_reg(),
                 rb=self.any_reg())

    def _emit_alui(self) -> None:
        self.one(self.rng.choice(ALUI), rd=self.reg(), imm=self.rng.randrange(64))

    def _emit_ldi(self) -> None:
        self.one("LDI", rd=self.reg(), imm=self.rng.randrange(256))

    def _emit_ldih(self) -> None:
        self.one("LDIH", rd=self.reg(), imm=self.rng.randrange(256))

    def _emit_unary(self) -> None:
        name = self.rng.choice(UNARY)
        # CMP and TEST write nothing, so they may name r7 as rd.
        rd = self.any_reg() if name in ("CMP", "TEST") else self.reg()
        self.one(name, rd=rd, ra=self.any_reg())

    def _emit_branch(self) -> None:
        direction = "any" if self.rng.random() < 0.25 else "fwd"
        self.push([self.fixup("rel8", self.rng.choice(ZC_BRANCHES), direction)])

    def _emit_tbranch(self) -> None:
        self.push([self.fixup("rel8", self.rng.choice(T_BRANCHES), "fwd")])

    def _emit_djnz_loop(self) -> None:
        """A bounded loop: load a small count, do one thing, decrement."""
        if self.room < 3:
            return self._emit_alui()
        rd = self.reg()
        body_rd = self.rng.choice([r for r in VALUE_REGS if r != rd])
        self.push([
            self.enc("LDI", rd=rd, imm=self.rng.randrange(1, 7)),
            self.enc(self.rng.choice(ALUI), rd=body_rd,
                     imm=self.rng.randrange(64)),
            # rel = body - (djnz_pc + 1) = -2, wherever the block lands.
            self.enc("DJNZ", rd=rd, rel=-2),
        ])

    def _emit_djnz_fwd(self) -> None:
        self.push([self.fixup("rel8", "DJNZ", "fwd", rd=self.reg())])

    def _emit_jp(self) -> None:
        self.push([self.fixup("rel6", "JP", "fwd", pin=self.read_pin(False),
                              val=self.rng.randrange(2))])

    def _emit_jp_poll(self) -> None:
        """``JP pin, v, -1``: spin while a moving input pin reads ``v``."""
        self.one("JP", pin=self.rng.choice(self.live_in),
                 val=self.rng.randrange(2), rel=-1)

    def _emit_jmp(self) -> None:
        direction = "fwd" if self.rng.random() < 0.6 else "any"
        self.push([self.fixup("abs", "JMP", direction)])

    def _emit_call(self) -> None:
        self.push([self.fixup("abs", "CALL", "any")])

    def _emit_call_chain(self) -> None:
        """``CALL`` the next word two or three times running: depth 2 and the
        overflow that drops the oldest entry (SEMANTICS 6.2), on purpose."""
        length = self.rng.choice((2, 3, 3))
        if self.room < length + 1:
            return self._emit_call()
        self.push([self.fixup("self", "CALL", offset=1) for _ in range(length)])

    def _emit_ret(self) -> None:
        self.one("RET")

    def _emit_halt(self) -> None:
        self.one("HALT")

    def _emit_setp(self) -> None:
        # lat (the D form, SEMANTICS 6.10) stays 0: a staged write is an M2
        # feature, and with ``m2_built`` the model may not have it.
        self.one("SETP", pin=self.write_pin(), val=self.rng.randrange(2), lat=0)

    def _emit_oep(self) -> None:
        pin = self.rng.choice(self.bidir) if self.rng.random() < 0.8 \
            else self.rng.randrange(32)
        self.one("OEP", pin=pin, val=self.rng.randrange(2))

    def _emit_out(self) -> None:
        self.one("OUT", ra=self.any_reg())

    def _emit_in(self) -> None:
        self.one("IN", rd=self.reg())

    def _emit_grpcfg(self) -> None:
        block = self.group_block(self.rng.choice(("OUTGRP", "INGRP")))
        if len(block) > self.room:
            return self._emit_nop()
        self.push(block)

    def _emit_waitd(self) -> None:
        words = []
        if self.rng.random() < 0.5:
            words.append(self.enc("SETD", imm=self.small_imm()))
        words.append(self.enc("WAITD", imm=self.small_imm()))
        self.push(words[-self.room:] if len(words) > self.room else words)

    def _emit_dly(self) -> None:
        self.one("DLY", imm=self.small_imm())

    def _emit_setd(self) -> None:
        self.one("SETD", imm=self.small_imm())

    def _emit_nop(self) -> None:
        self.one("NOP")

    def _emit_tdrel(self) -> None:
        """``TD = NOW + k`` the long way round: ``CSRR NOW``, ``ADDI``, ``CSRW TD``."""
        if self.room < 3:
            return self._emit_setd()
        self.push([self.enc("CSRR", rd=SCRATCH, csr=self.csr["NOW"]),
                   self.enc("ADDI", rd=SCRATCH, imm=self.rng.randrange(16)),
                   self.enc("CSRW", csr=self.csr["TD"], ra=SCRATCH)])

    def _emit_retick(self) -> None:
        """Change the tick rate mid-run (ACC is cleared at that edge)."""
        value = 0 if self.rng.random() < 0.1 else self.tick_int_value()
        block = self.csrw_scratch("TICK_INT", value)
        if len(block) > self.room:
            return self._emit_nop()
        self.push(block)

    def _cond_wait(self, name: str, live: bool, force_timed: bool = False,
                   **ops: int) -> None:
        """A conditional wait with the timeout and anchor rules applied.

        Untimed only when the condition will come true (``live``), or as a
        rare gamble that it may never complete.
        """
        gamble = self.rng.random() < GAMBLE_RATE
        if force_timed:
            timed = True
        elif live:
            timed = self.rng.random() < 0.5
        else:
            timed = not gamble
        words: List[_Word] = []
        if timed and self.rng.random() < 0.6:
            # Anchor the deadline in the future so the wait can end by its
            # condition; without this TD is often stale and the wait times out
            # at once, which is the other bin.
            words.append(self.enc("SETD", imm=self.small_imm()))
        words.append(self.enc(name, tmo=1 if timed else 0, **ops))
        self.push(words[-self.room:] if len(words) > self.room else words)

    def _emit_waitp(self) -> None:
        live = self.rng.random() < 0.55
        self._cond_wait("WAITP", live, pin=self.read_pin(live),
                        val=self.rng.randrange(2))

    def _emit_waite(self) -> None:
        live = self.rng.random() < 0.6
        edge = self.rng.choice((0, 1, 2, 2, 3))
        pin = self.read_pin(live)
        # "e == 3 and pin > 12 are never true" (SEMANTICS 6.4): time those out.
        never = edge == 3 or pin > 12
        self._cond_wait("WAITE", live and not never, force_timed=never,
                        pin=pin, edge=edge)

    def _emit_waits(self) -> None:
        # A pool flag is live when another thread runs: every thread signals
        # the pool flags at the top of its main loop (see build()). Even so a
        # third thread can consume the flag first, so fewer untimed WAITS.
        flag = self.flag()
        live = self.others_running and flag in self.flag_pool \
            and self.rng.random() < 0.6
        self._cond_wait("WAITS", live, flag=flag)

    def _emit_sig(self) -> None:
        self.one("SIG", flag=self.flag())

    def _emit_clr(self) -> None:
        self.one("CLR", flag=self.flag())

    def _next_csr(self) -> int:
        if self.rng.random() < 0.85:
            number = self.csr_cycle[self.csr_next % len(self.csr_cycle)]
            self.csr_next += 1
            return number
        return self.rng.choice(self.dead_csrs)

    def _emit_csrr(self) -> None:
        self.one("CSRR", rd=self.reg(), csr=self._next_csr())

    def _emit_csrw(self) -> None:
        number = self._next_csr()
        name = self.isa.csrs[number]["name"] if number in self.isa.csrs else ""
        rng = self.rng
        if name == "TICK_INT":
            value = 0 if rng.random() < 0.1 else self.tick_int_value()
            block = self.csrw_scratch(name, value)
        elif name == "TD":
            if rng.random() < 0.7 and self.room >= 3:
                block = [self.enc("CSRR", rd=SCRATCH, csr=self.csr["NOW"]),
                         self.enc("ADDI", rd=SCRATCH, imm=rng.randrange(24)),
                         self.enc("CSRW", csr=number, ra=SCRATCH)]
            else:
                block = self.csrw_scratch(name, rng.randrange(48))
        elif name in ("OUTGRP", "INGRP") and rng.random() < 0.7:
            block = self.group_block(name)
        elif name == "PIN_OUT" and "csrw_pin_out_high_bits" in self.avoid:
            # PIN_OUT[15:14] have no pin; see docs/spec-questions/cosim.md.
            block = self.csrw_scratch(name, rng.randrange(0x4000))
        else:
            block = [self.enc("CSRW", csr=number, ra=self.any_reg())]
        if len(block) > self.room:
            return self._emit_nop()
        self.push(block)

    def _emit_reserved(self) -> None:
        word = self.rng.choice(self.reserved_words)
        self.push([_Word(word=word, mnemonic="")])

    def _emit_unbuilt(self) -> None:
        name = self.unbuilt_cycle[self.unbuilt_next % len(self.unbuilt_cycle)]
        self.unbuilt_next += 1
        ops: Dict[str, int] = {}
        for operand in self.isa.by_name[name].ops:
            base = operand.rstrip("0123456789")
            if base in ("rd", "ra"):
                ops[base] = self.any_reg()
            elif base == "imm":
                ops["imm"] = self.rng.randrange(32)
            elif base == "cond":
                ops["cond"] = self.rng.randrange(4)
            elif base == "tmo":
                ops["tmo"] = self.rng.randrange(2)
        self.one(name, **ops)

    # ------------------------------------------------------------- assembly
    def build(self, weights: Dict[str, int], want_halt: bool) -> List[_Block]:
        self.emit_prologue()
        # The main loop starts by signalling the shared flags (most of the
        # time), so an untimed WAITS in another thread has something to wait
        # for on every lap.
        for flag in self.flag_pool:
            if self.rng.random() < 0.75:
                self.one("SIG", flag=flag)
        names = sorted(weights)
        totals = [weights[n] for n in names]
        halt_at = None
        if want_halt:
            halt_at = self.rng.randrange(3, max(4, self.room // 2))
        emitted = 0
        while self.room > 0:
            before = self.used
            if halt_at is not None and emitted == halt_at:
                self._emit_halt()
            else:
                getattr(self, "_emit_" + self.rng.choices(names, totals)[0])()
            if self.used == before:                  # an emitter declined
                self._emit_nop()
            emitted += 1
        # SEMANTICS has no fall-through protection: the region has to end with
        # a jump back, or the thread wanders into its neighbour's code.
        self.push([self.fixup("loop", "JMP")], targetable=False)
        return self.blocks


# ------------------------------------------------------------------ resolving
def _reserved_words(isa: Isa, rng: random.Random) -> List[int]:
    """A handful of 16-bit words that decode to no instruction at all.

    ``isa.decode`` is the only authority on which words those are, so the list
    follows ``isa/isa.yaml`` automatically.
    """
    out: List[int] = []
    tries = 0
    while len(out) < 24 and tries < 4000:
        tries += 1
        candidate = rng.randrange(1 << 16)
        if isa.decode(candidate) is None:
            out.append(candidate)
    if not out:
        raise LoomgenError("isa.yaml leaves no reserved encodings")
    return out


def _place(blocks: List[_Block], start: int) -> None:
    addr = start
    for block in blocks:
        block.addr = addr
        addr += len(block)


def _pick_rel(rng: random.Random, heads: Sequence[int], addr: int, span: int,
              direction: str) -> int:
    """A block head reachable with a ``span``-bit signed offset, never ``addr``
    itself. Forward means past the next word, so taken and not taken differ."""
    lo, hi = -(1 << (span - 1)), (1 << (span - 1)) - 1
    nxt = addr + 1
    reach = [h for h in heads if lo <= h - nxt <= hi and h != addr]
    if direction == "fwd":
        forward = [h for h in reach if h > nxt]
        if forward:
            return rng.choice(forward)
        return nxt                                  # rel 0: harmless
    if reach:
        return rng.choice(reach)
    return nxt


def _resolve(blocks: List[_Block], isa: Isa, rng: random.Random,
             loop_head: int) -> Dict[int, int]:
    """Turn placed blocks into ``{address: word}``, choosing branch targets."""
    heads = sorted(b.addr for b in blocks if b.targetable) or [loop_head]
    image: Dict[int, int] = {}
    for block in blocks:
        for offset, item in enumerate(block.words):
            addr = block.addr + offset
            if not item.fix:
                image[addr] = item.word
                continue
            ops = dict(item.operands)
            if item.fix == "loop":
                image[addr] = isa.encode(item.mnemonic, abs=loop_head)
            elif item.fix == "self":
                image[addr] = isa.encode(item.mnemonic, abs=addr + item.offset)
            elif item.fix == "abs":
                others = [h for h in heads if h != addr]
                forward = [h for h in others if h > addr + 1]
                pool = forward if (item.direction == "fwd" and forward) \
                    else (others or [loop_head])
                image[addr] = isa.encode(item.mnemonic, abs=rng.choice(pool),
                                         **ops)
            else:
                span = 8 if item.fix == "rel8" else 6
                target = _pick_rel(rng, heads, addr, span, item.direction)
                image[addr] = isa.encode(item.mnemonic, rel=target - (addr + 1),
                                         **ops)
    return image


def static_target(isa: Isa, addr: int, word: int) -> Optional[int]:
    """The branch or jump target a word names, or None if it names none."""
    decoded = isa.decode(word)
    if decoded is None:
        return None
    instr, fields = decoded
    if instr.name in ("JMP", "CALL"):
        return fields["abs"] & PC_MASK
    if instr.cls == "branch":
        return (addr + 1 + fields["rel"]) & PC_MASK
    return None


def check_program(prog: GeneratedProgram, isa: Optional[Isa] = None) -> None:
    """Raise :class:`LoomgenError` unless ``prog`` keeps every generator rule:
    words and targets inside their region, the region closed by a ``JMP`` to
    a head, no self-loop except a ``JP`` poll, no backward free ``DJNZ``,
    ``BT``/``BNT`` or ``JP``."""
    isa = isa or load_isa()
    for t in prog.running():
        start, end = prog.region(t)
        words = {a: w for a, w in prog.image.items() if start <= a < end}
        if sorted(words) != list(range(start, end)):
            raise LoomgenError("thread %d region %#x..%#x is not fully written"
                               % (t, start, end))
        last = isa.decode(words[end - 1])
        if last is None or last[0].name != "JMP":
            raise LoomgenError("thread %d region does not end with JMP" % t)
        for addr, word in words.items():
            target = static_target(isa, addr, word)
            if target is None:
                continue
            if not start <= target < end:
                raise LoomgenError("thread %d: %#x targets %#x outside %#x..%#x"
                                   % (t, addr, target, start, end))
            name = isa.decode(word)[0].name
            if target == addr and name != "JP":
                raise LoomgenError("thread %d: %s at %#x branches to itself"
                                   % (t, name, addr))
            if target <= addr and name in ("BT", "BNT"):
                raise LoomgenError("thread %d: backward %s at %#x" % (t, name, addr))
            if target < addr and name == "JP":
                raise LoomgenError("thread %d: backward JP at %#x" % (t, addr))
            if target < addr and name == "DJNZ" and target != addr - 1:
                raise LoomgenError("thread %d: unbounded DJNZ at %#x" % (t, addr))
    outside = [a for a in prog.image
               if not any(prog.region(t)[0] <= a < prog.region(t)[1]
                          for t in prog.running())]
    if outside:
        raise LoomgenError("words outside every running region: %s"
                           % ", ".join("%#x" % a for a in sorted(outside)[:8]))


# ---------------------------------------------------------------- public API
def generate(seed: int, threads: int = 1, imem_words: int = 256,
             profile: str = "mixed", cycles: int = 6000,
             entries: Optional[Sequence[int]] = None,
             avoid: Iterable[str] = (),
             run_mask: Optional[int] = None,
             isa: Optional[Isa] = None) -> GeneratedProgram:
    """Build one random program.

    Args:
        seed: everything is derived from this; the same arguments give the
            same image, entry points, running threads and stimulus plan.
        threads: how many threads get a program and are started, 1 to 4.
            Which ones is part of the random draw unless ``run_mask`` is given.
        imem_words: instruction-memory size, a power of two from 64 to 1024.
            Each thread's region is a quarter of it.
        profile: ``"alu"``, ``"timing"``, ``"pins"`` or ``"mixed"``.
        cycles: how long the run is expected to be (recorded in the plan).
        entries: per-thread start addresses; each thread's region is
            ``[entry, entry + imem_words / 4)``. The default is D-017's
            ``t * (IMEM_WORDS / 4)``, which is what both implementations
            reset to, so the harness only has to write ``RUN``.
        avoid: names from :data:`AVOID_FLAGS`; each switches off a construct
            that ``docs/spec-questions/cosim.md`` has open.
        run_mask: which threads run; overrides the random choice (and
            ``threads``).
    """
    if not 1 <= threads <= THREADS:
        raise LoomgenError("threads must be 1..4, got %r" % (threads,))
    if imem_words & (imem_words - 1) or not 64 <= imem_words <= 1024:
        raise LoomgenError("imem_words must be a power of two, 64..1024")
    if profile not in PROFILES:
        raise LoomgenError("unknown profile %r; have %s"
                           % (profile, ", ".join(sorted(PROFILES))))
    avoid = frozenset(str(a) for a in avoid)
    unknown = avoid - AVOID_FLAGS
    if unknown:
        raise LoomgenError("unknown avoid flags: %s" % ", ".join(sorted(unknown)))

    isa = isa or load_isa()
    rng = random.Random()
    rng.seed("loomgen|%d|%d|%d|%s|%s|%s|%s" % (
        seed, threads, imem_words, profile, ",".join(sorted(avoid)),
        ",".join(str(e) for e in entries) if entries is not None else "-",
        run_mask if run_mask is not None else "-"))

    stride = imem_words // THREADS
    entry_list = list(entries) if entries is not None \
        else [t * stride for t in range(THREADS)]
    if len(entry_list) != THREADS:
        raise LoomgenError("entries must have one address per thread")

    if run_mask is None:
        chosen = sorted(rng.sample(range(THREADS), threads))
        run_mask = sum(1 << t for t in chosen)
    run_mask &= (1 << THREADS) - 1
    if not run_mask:
        raise LoomgenError("run_mask selects no thread")
    running = [t for t in range(THREADS) if (run_mask >> t) & 1]
    spans = sorted((entry_list[t], entry_list[t] + stride) for t in running)
    for (s0, e0), (s1, _) in zip(spans, spans[1:]):
        if s1 < e0:
            raise LoomgenError("thread regions overlap at %#x" % s1)
    if spans[0][0] < 0 or spans[-1][1] > imem_words:
        raise LoomgenError("a thread region does not fit in %d words" % imem_words)

    plan = build_plan(rng, cycles, busy=(profile == "pins"))
    weights = PROFILES[profile]
    flag_pool = rng.sample(range(8), rng.choice((1, 2, 2, 3)))

    image: Dict[int, int] = {}
    heads: List[int] = []
    for thread in running:
        builder = _ThreadBuilder(rng, isa, thread, stride, profile, plan,
                                 len(running) > 1, flag_pool, avoid)
        blocks = builder.build(weights, rng.random() < HALT_RATE)
        _place(blocks, entry_list[thread])
        loop_head = blocks[builder.loop_head_index].addr
        image.update(_resolve(blocks, isa, rng, loop_head))
        heads += [b.addr for b in blocks if b.targetable]

    prog = GeneratedProgram(
        seed=seed, profile=profile, threads=len(running),
        imem_words=imem_words, image=image, entries=entry_list,
        run_mask=run_mask, stimulus=plan, avoid=tuple(sorted(avoid)),
        heads=frozenset(heads))
    check_program(prog, isa)
    return prog
