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

The build
---------

A program is generated for one build of the chip, the one ``CTRL.CAPS``
reports (``docs/SEMANTICS.md`` 5): ``features`` (any of ``"FIFO"``, ``"BE"``,
``"SETPD"``) and ``fifo_depth``. The default is the M2 chip, FIFOs of depth 4,
the manual bit engine and the deadline-latched ``SETP``. Instructions of a
feature that is built are generated as live code (below); those of a feature
that is not are ``NOP`` + ``BADOP`` (SEMANTICS 9) and come from the unbuilt
pool, as ``LD``/``ST`` always do. Both implementations are then run with the
same build, so a program means the same thing to both.

How each M2 construct stays live
--------------------------------

* ``PUSH ra`` (OUTQ, SEMANTICS 6.7) stalls while ``OUTQ[t]`` is full, and only
  the host pops it. Without host traffic every ``PUSH`` is guarded:
  ``[SETD k] WAITB OUTQ_NF, T; BT +1; PUSH ra``. The ``WAITB`` ends by its
  condition (``T <= 0``) or at the deadline (``T <= 1``) and ``BT`` skips the
  ``PUSH`` on a timeout. After a condition end the ``PUSH`` cannot stall:
  only thread ``t`` pushes ``OUTQ[t]`` and the host can only pop it, so it
  cannot fill up between the ``WAITB``'s X cycle and the ``PUSH``'s.
* ``POP rd`` (INQ) stalls while ``INQ[t]`` is empty, and only the host pushes
  it; the guard is the same with ``WAITB INQ_NE, T``. No thread ever pops
  what another thread pushes (there is no such path in the chip).
* With ``host_traffic=True`` the program comes with a :class:`HostPlan` that
  pushes every running thread's ``INQ`` and reads its ``OUTQ`` in every round
  (:mod:`tools.loomgen.hostplan`), and then raw ``PUSH``/``POP`` and untimed
  ``WAITB OUTQ_NF``/``INQ_NE`` are emitted too: the plan serves them.
* ``WAITB TICK`` (``TICK_SEEN``) and ``WAITB BE_IDLE`` (always true until auto
  mode) are live on their own; they are generated timed and untimed.
* Bit-engine loops count with ``CNT``: ``LDI r7, n; CSRW CNT, r7; SHO|SHI;
  [DLY|WAITD]; BNZ back``. ``SHO``/``SHI`` set ``CNT <= max(CNT - 1, 0)`` and
  ``Z = (CNT == 0)`` (SEMANTICS 6.9), nothing else in the loop touches ``Z``,
  so the loop runs ``max(n, 1)`` times, at most 31.
* ``SETP pin, v, D`` does not wait for anything: it stages a write that lands
  on the thread's next deadline (6.10). The generator follows it with a
  ``WAITD``, a ``SETD``, a ``CSRW TD`` or nothing, with deadlines both ahead
  of ``NOW`` (the write lands by rule 1, ``NOW`` ticking onto ``TD``) and
  already passed (rule 2, at the ``TD`` write).

There are no FIFO CSRs in ``isa/isa.yaml``: a thread sees its FIFOs only
through ``PUSH``, ``POP`` and ``WAITB``.

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

from .hostplan import HostPlan, build_host_plan
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

#: Instructions that decode but whose feature the M1 build does not have:
#: the FIFO ops and WAITB (M2), the bit engine (M2/M3) and LD/ST (optional
#: DMEM). In a build without their feature they execute as ``NOP`` and set
#: ``BADOP`` (SEMANTICS 9); :func:`unbuilt_mnemonics` gives the list for a
#: given build.
UNBUILT_MNEMONICS: Tuple[str, ...] = (
    "PUSH", "POP", "WAITB", "SHO", "SHI", "LDSR", "STSR", "CRCI", "STCRC",
    "LD", "ST",
)

#: Instructions of the M2 features (SEMANTICS 6.7, 6.9): FIFOs, WAITB and the
#: manual bit engine. (``SETP ... D`` is ``SETP`` with the ``lat`` bit.)
M2_MNEMONICS: Tuple[str, ...] = (
    "PUSH", "POP", "WAITB", "SHO", "SHI", "LDSR", "STSR", "CRCI", "STCRC",
)

#: The feature each M2 instruction needs. ``WAITB BE_IDLE`` needs ``"BE"``
#: as well (SEMANTICS 6.4); :func:`instruction_built` handles that operand.
FEATURE_OF: Dict[str, str] = {
    "PUSH": "FIFO", "POP": "FIFO", "WAITB": "FIFO",
    "SHO": "BE", "SHI": "BE", "LDSR": "BE", "STSR": "BE", "CRCI": "BE",
    "STCRC": "BE", "LD": "DMEM", "ST": "DMEM",
}

#: CSRs built at M1 (SEMANTICS 6.6: 0x00-0x03, 0x09-0x0C, 0x10-0x15), by name.
M1_CSR_NAMES: Tuple[str, ...] = (
    "TICK_INT", "TICK_FRAC", "OUTGRP", "INGRP", "NOW", "TD", "FLAGS", "TID",
    "OD_MASK", "PIN_OUT", "PIN_OE", "PIN_IN", "SFLAGS", "HOST_IRQ",
)
#: CSRs of the bit engine (SEMANTICS 6.9). In a build without ``"BE"`` they
#: read 0 and ignore writes, without BADOP (6.6).
UNBUILT_CSR_NAMES: Tuple[str, ...] = (
    "BE_CFG", "BE_PINS", "BE_RELOAD", "CRC_POLY", "CRC_INIT", "SR", "CNT", "CRC",
)
#: The same list under the name the M2 build uses for it: built with ``"BE"``.
M2_CSR_NAMES: Tuple[str, ...] = UNBUILT_CSR_NAMES

#: Features :func:`generate` can build for (the ones that change what an
#: instruction does; ``DMEM`` and ``BOOTROM`` are not modelled). ``BEENC``
#: (the slice-A encoders, SEMANTICS 6.9.1, CAPS[9]) is accepted so that a
#: slice-A build co-simulates; the generator drives its ``BE_CFG`` bits only
#: once :meth:`be_cfg_value` is taught them. ``DMEM`` (slice B, CAPS[5]) is
#: accepted the same way; ``LD``/``ST`` are emitted only through the unbuilt
#: cycle until the generator learns a data area to address (integration).
FEATURES: FrozenSet[str] = frozenset(("FIFO", "BE", "SETPD", "BEENC", "DMEM"))
#: The M2 chip: ``CAPS`` reports the FIFOs (depth 4), the manual bit engine
#: and the deadline-latched ``SETP``. What :func:`generate` assumes unless told.
DEFAULT_FEATURES: Tuple[str, ...] = ("BE", "FIFO", "SETPD")
DEFAULT_FIFO_DEPTH = 4
#: Legal FIFO depths (SEMANTICS 6.7): a power of two from 2 to 8.
FIFO_DEPTHS: Tuple[int, ...] = (2, 4, 8)

#: Register 7 is the generator's scratch: no random instruction writes it, so
#: the only values it ever holds are the ones a ``LDI``/``LDIH`` or ``CSRR
#: NOW`` in the same block put there.
SCRATCH = 7
#: Slice B (SEMANTICS 6.11, feature ``DMEM``): every running thread keeps the
#: top DATA_WORDS words of its region as a data window that LD/ST address
#: through the scratch register, so no store ever lands on code. The window
#: is part of the image (random words), so a load never reads an unwritten
#: word, which the RTL's memory would return as X.
DATA_WORDS = 8
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

#: ``WAITB`` conditions (``isa.yaml`` ``enums.cond``).
WAITB_BE_IDLE, WAITB_OUTQ_NF, WAITB_INQ_NE, WAITB_TICK = 0, 1, 2, 3

#: ``BE_CFG`` fields that exist at M2 (SEMANTICS 6.9).
BE_CFG_DIR, BE_CFG_INV, BE_CFG_CRC_EN = 1 << 1, 1 << 7, 1 << 9
#: Slice A (SEMANTICS 6.9.1, feature ``BEENC``): ``ENC`` at bits 4:3,
#: ``STUFF`` at 6:5, ``DIFF`` at bit 10. Value 3 of a field is reserved
#: and stored as 0, so it is worth writing now and then.
BE_CFG_ENC_SHIFT, BE_CFG_STUFF_SHIFT, BE_CFG_DIFF = 3, 5, 1 << 10

#: Known ``avoid`` flags. Each switches off one construct on which the RTL
#: and the model disagree where ``docs/SEMANTICS.md`` does not decide the
#: answer, until the director rules; see ``docs/spec-questions/cosim.md``.
AVOID_FLAGS: FrozenSet[str] = frozenset((
    "csrw_pin_out_high_bits",
))


class LoomgenError(ValueError):
    """Bad generator arguments, or an image that breaks a generator rule."""


def unbuilt_mnemonics(features: Iterable[str]) -> Tuple[str, ...]:
    """The instructions that are ``NOP`` + ``BADOP`` in a build with
    ``features`` (``WAITB`` counts as built with the FIFOs even though its
    ``BE_IDLE`` operand also needs the bit engine)."""
    built = frozenset(features)
    return tuple(n for n in UNBUILT_MNEMONICS if FEATURE_OF[n] not in built)


def instruction_built(name: str, fields: Dict[str, int],
                      features: Iterable[str]) -> bool:
    """Does this decoded instruction execute (rather than ``NOP`` + ``BADOP``)
    in a build with ``features``? SEMANTICS 6.4: ``WAITB BE_IDLE`` needs the
    bit engine as well as the FIFOs."""
    built = frozenset(features)
    feature = FEATURE_OF.get(name)
    if feature is None:
        return True
    if feature not in built:
        return False
    if name == "WAITB" and fields.get("cond") == WAITB_BE_IDLE:
        return "BE" in built
    return True


def normalise_build(features: Iterable[str], fifo_depth: int) -> Tuple[Tuple[str, ...], int]:
    """Check a build and return it in canonical form: sorted feature names
    and the FIFO depth (the default depth when the FIFOs are not built)."""
    names = tuple(sorted(frozenset(str(f).upper() for f in features)))
    unknown = [n for n in names if n not in FEATURES]
    if unknown:
        raise LoomgenError("cannot generate for feature(s) %s; know %s"
                           % (", ".join(unknown), ", ".join(sorted(FEATURES))))
    if "FIFO" in names:
        if fifo_depth not in FIFO_DEPTHS:
            raise LoomgenError("fifo_depth must be 2, 4 or 8, not %r" % (fifo_depth,))
    else:
        fifo_depth = DEFAULT_FIFO_DEPTH
    return names, int(fifo_depth)


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
    #: The build the program is for (SEMANTICS 5, ``CAPS``): sorted feature
    #: names and the FIFO depth. ``()`` is the M1 build.
    features: Tuple[str, ...] = ()
    fifo_depth: int = DEFAULT_FIFO_DEPTH
    #: Host traffic the program relies on (raw PUSH/POP), or None.
    host: Optional[HostPlan] = None

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
            "features": list(self.features),
            "fifo_depth": self.fifo_depth,
            "host": self.host.to_obj() if self.host is not None else None,
            "stimulus": self.stimulus.to_obj(),
            "source": "generated by tools.loomgen seed %d profile %s"
                      % (self.seed, self.profile),
        }

    @staticmethod
    def from_obj(obj: Dict) -> "GeneratedProgram":
        """Inverse of :meth:`to_obj`. A file written before the build was
        recorded (no ``features`` key) is an M1-build program."""
        image = {int(k, 0) if isinstance(k, str) else int(k): int(v)
                 for k, v in obj["words"].items()}
        threads = dict(obj["threads"])
        entries = [int(threads[str(t)]["entry"]) for t in range(THREADS)]
        run_mask = int(obj["run_mask"])
        host = obj.get("host")
        return GeneratedProgram(
            seed=int(obj["seed"]), profile=str(obj["profile"]),
            threads=bin(run_mask).count("1"),
            imem_words=int(obj["imem_words"]), image=image, entries=entries,
            run_mask=run_mask,
            stimulus=StimulusPlan.from_obj(obj["stimulus"]),
            avoid=tuple(a for a in obj.get("avoid", ()) if a != "m2_built"),
            heads=frozenset(int(a) for a in obj.get("heads", ())),
            features=tuple(sorted(obj.get("features", ()))),
            fifo_depth=int(obj.get("fifo_depth", DEFAULT_FIFO_DEPTH)),
            host=HostPlan.from_obj(host) if host else None)

    def disassembly(self, isa: Optional[Isa] = None) -> str:
        """An annotated listing of every word, for debugging a failing seed."""
        from tools.loomasm.disasm import disassemble

        isa = isa or load_isa()
        lines = ["; tools.loomgen seed=%d profile=%s threads=%d imem_words=%d"
                 % (self.seed, self.profile, self.threads, self.imem_words),
                 "; run_mask=0x%X avoid=%s" % (self.run_mask,
                                               ",".join(self.avoid) or "-"),
                 "; build: features=%s fifo_depth=%d host traffic=%s"
                 % (",".join(self.features) or "M1", self.fifo_depth,
                    "%d transactions" % len(self.host) if self.host else "none"),
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
        if self.host:
            lines.append("")
            lines.append("; ---- host plan (repeats until the run ends)")
            lines += ["  %3d  gap %-4d %s" % (i, txn.gap, txn)
                      for i, txn in enumerate(self.host.txns)]
        return "\n".join(lines) + "\n"


# ------------------------------------------------------------------- profiles
#: Relative weight of every block emitter, per profile. Names are the
#: ``_emit_*`` methods of :class:`_ThreadBuilder`. The M2 emitters
#: (``fifo_*``, ``waitb``, ``be_*``, ``crc``, ``setp_d``) fall back to the
#: unbuilt pool (or, for ``setp_d``, an ordinary ``SETP``) in a build without
#: their feature.
PROFILES: Dict[str, Dict[str, int]] = {
    "mixed": {
        "alu3": 10, "alui": 10, "ldi": 5, "ldih": 3, "unary": 9,
        "branch": 9, "tbranch": 3, "djnz_loop": 3, "djnz_fwd": 2, "jp": 4,
        "jp_poll": 1, "jmp": 2, "call": 4, "call_chain": 1, "ret": 4,
        "setp": 8, "oep": 3, "out": 4, "in": 4, "grpcfg": 2,
        "waitd": 5, "dly": 3, "setd": 3, "nop": 2, "tdrel": 1, "retick": 1,
        "waitp": 4, "waite": 4, "waits": 4, "sig": 4, "clr": 3,
        "csrr": 6, "csrw": 6, "reserved": 2, "unbuilt": 2,
        "fifo_push": 3, "fifo_pop": 3, "waitb": 2, "be_cfg": 1, "be_op": 3,
        "be_loop": 1, "crc": 1, "setp_d": 3, "mem": 2,
    },
    "alu": {
        "alu3": 26, "alui": 24, "ldi": 8, "ldih": 6, "unary": 22,
        "branch": 14, "tbranch": 3, "djnz_loop": 4, "djnz_fwd": 3, "jp": 2,
        "jp_poll": 1, "jmp": 2, "call": 4, "call_chain": 1, "ret": 4,
        "setp": 3, "oep": 1, "out": 2, "in": 2, "grpcfg": 1,
        "waitd": 2, "dly": 1, "setd": 2, "nop": 2, "tdrel": 1, "retick": 1,
        "waitp": 1, "waite": 1, "waits": 2, "sig": 2, "clr": 1,
        "csrr": 4, "csrw": 4, "reserved": 2, "unbuilt": 2,
        "fifo_push": 1, "fifo_pop": 1, "waitb": 1, "be_cfg": 1, "be_op": 3,
        "be_loop": 1, "crc": 1, "setp_d": 1, "mem": 2,
    },
    "timing": {
        "alu3": 5, "alui": 5, "ldi": 3, "ldih": 2, "unary": 4,
        "branch": 6, "tbranch": 6, "djnz_loop": 3, "djnz_fwd": 1, "jp": 3,
        "jp_poll": 2, "jmp": 2, "call": 2, "call_chain": 1, "ret": 2,
        "setp": 6, "oep": 1, "out": 2, "in": 2, "grpcfg": 1,
        "waitd": 14, "dly": 9, "setd": 8, "nop": 2, "tdrel": 4, "retick": 4,
        "waitp": 8, "waite": 8, "waits": 8, "sig": 7, "clr": 4,
        "csrr": 5, "csrw": 6, "reserved": 2, "unbuilt": 2,
        "fifo_push": 2, "fifo_pop": 2, "waitb": 5, "be_cfg": 1, "be_op": 2,
        "be_loop": 2, "crc": 1, "setp_d": 8, "mem": 2,
    },
    "pins": {
        "alu3": 4, "alui": 4, "ldi": 5, "ldih": 3, "unary": 4,
        "branch": 5, "tbranch": 2, "djnz_loop": 2, "djnz_fwd": 1, "jp": 9,
        "jp_poll": 3, "jmp": 2, "call": 2, "call_chain": 1, "ret": 2,
        "setp": 22, "oep": 9, "out": 12, "in": 12, "grpcfg": 7,
        "waitd": 3, "dly": 2, "setd": 3, "nop": 1, "tdrel": 1, "retick": 1,
        "waitp": 10, "waite": 12, "waits": 4, "sig": 3, "clr": 2,
        "csrr": 5, "csrw": 6, "reserved": 2, "unbuilt": 2,
        "fifo_push": 1, "fifo_pop": 1, "waitb": 2, "be_cfg": 4, "be_op": 8,
        "be_loop": 3, "crc": 1, "setp_d": 6, "mem": 2,
    },
    # The M2 features first: FIFOs, bit engine, deadline-latched SETP, with
    # enough of everything else around them to keep the slots varied.
    "m2": {
        "alu3": 4, "alui": 4, "ldi": 3, "ldih": 2, "unary": 3,
        "branch": 4, "tbranch": 2, "djnz_loop": 1, "djnz_fwd": 1, "jp": 1,
        "jp_poll": 1, "jmp": 2, "call": 2, "call_chain": 1, "ret": 2,
        "setp": 4, "oep": 2, "out": 2, "in": 2, "grpcfg": 1,
        "waitd": 5, "dly": 2, "setd": 3, "nop": 1, "tdrel": 2, "retick": 1,
        "waitp": 2, "waite": 2, "waits": 2, "sig": 2, "clr": 1,
        "csrr": 4, "csrw": 4, "reserved": 1, "unbuilt": 1,
        "fifo_push": 8, "fifo_pop": 7, "waitb": 5, "be_cfg": 3, "be_op": 9,
        "be_loop": 4, "crc": 2, "setp_d": 9, "mem": 2,
    },
}

#: Probability that a thread's program contains one ``HALT``.
HALT_RATE = 0.12
#: Share of conditional waits that are untimed on something that may never
#: happen (the "gamble"); every other untimed wait watches a live condition.
#: The M2 constructs never gamble: they are timed unless live.
GAMBLE_RATE = 1.0 / 14.0

#: Deadline offsets for the latched ``SETP``: short ones are usually already
#: passed when the ``TD`` write commits (rule 2), long ones land by rule 1.
LATCH_IMMS = (0, 1, 2, 3, 4, 6, 8, 12, 16, 24)


# ------------------------------------------------------------- thread builder
class _ThreadBuilder:
    """Builds the blocks of one thread's region."""

    def __init__(self, rng: random.Random, isa: Isa, thread: int,
                 size: int, profile: str, plan: StimulusPlan,
                 others_running: bool, flag_pool: Sequence[int],
                 avoid: FrozenSet[str], features: Sequence[str] = (),
                 fifo_depth: int = DEFAULT_FIFO_DEPTH,
                 host_traffic: bool = False, entry: int = 0):
        self.rng = rng
        self.isa = isa
        self.thread = thread
        self.size = size
        self.profile = profile
        self.others_running = others_running
        self.flag_pool = tuple(flag_pool)
        self.avoid = avoid
        self.features = frozenset(features)
        self.fifo = "FIFO" in self.features
        self.be_enc = "BEENC" in self.features
        self.be = "BE" in self.features
        self.setpd = "SETPD" in self.features
        # Slice B: the top DATA_WORDS words of the region are data, never code.
        self.dmem = "DMEM" in self.features
        self.data_base = entry + size - DATA_WORDS
        if self.dmem:
            self.size = size - DATA_WORDS
        self.fifo_depth = fifo_depth
        # The host pushes INQ[t] and pops OUTQ[t] of every running thread in
        # every round of its plan (tools.loomgen.hostplan), which is what
        # makes a raw PUSH/POP or an untimed FIFO WAITB live. A round is a
        # few thousand clocks, so a region gets at most one raw PUSH and one
        # raw POP: more would leave the thread waiting for the host most of
        # the run instead of executing the rest of its program.
        self.host_traffic = host_traffic and self.fifo
        # PUSH is released by any host read of this thread's OUTQ and the
        # plan makes one every round, so two raw pushes are affordable; a raw
        # POP waits for a push into its INQ, which is the longer wait.
        self.raw_left = {"PUSH": 2 if self.host_traffic else 0,
                         "POP": 1 if self.host_traffic else 0,
                         "WAITB": 1 if self.host_traffic else 0}
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
        self.unbuilt_cycle = list(unbuilt_mnemonics(self.features))
        rng.shuffle(self.unbuilt_cycle)
        self.unbuilt_next = 0
        built_csrs = list(M1_CSR_NAMES) + (list(M2_CSR_NAMES) if self.be else [])
        self.m1_csrs = [csr[n] for n in built_csrs]
        # Unbuilt CSRs plus the CSR numbers isa.yaml does not define at all:
        # both read 0 and ignore writes (SEMANTICS 6.6).
        self.dead_csrs = [csr[n] for n in UNBUILT_CSR_NAMES if not self.be] + \
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

    def fit(self, words: List[_Word], droppable: int = 0) -> bool:
        """Push ``words`` as one block if they fit, dropping up to
        ``droppable`` optional words from the front first. False (and
        nothing pushed) if they still do not fit."""
        while len(words) > self.room and droppable > 0:
            words = words[1:]
            droppable -= 1
        if len(words) > self.room:
            return False
        self.push(words)
        return True

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

    def be_cfg_value(self, crc: Optional[bool] = None) -> int:
        """A ``BE_CFG`` value: usually just ``DIR``/``INV``/``CRC_EN``, now and
        then all sixteen bits random (the other bits must be ignored)."""
        rng = self.rng
        if crc is None and rng.random() < 0.3:
            return rng.randrange(1 << 16)
        value = (BE_CFG_DIR if rng.random() < 0.5 else 0) | \
            (BE_CFG_INV if rng.random() < 0.3 else 0)
        if crc if crc is not None else rng.random() < 0.4:
            value |= BE_CFG_CRC_EN
        if self.be_enc:
            # Slice A: an encoder, a stuffer and DIFF, each field with its
            # reserved value 3 now and then (stored as 0, SEMANTICS 6.9.1).
            value |= rng.choice((0, 0, 0, 0, 1, 1, 1, 2, 2, 3)) << BE_CFG_ENC_SHIFT
            value |= rng.choice((0, 0, 0, 0, 1, 1, 1, 2, 2, 3)) << BE_CFG_STUFF_SHIFT
            if rng.random() < 0.25:
                value |= BE_CFG_DIFF
        return value

    def be_pins_value(self) -> int:
        """``BE_PINS = {in[9:5], out[4:0]}``: every index class for both.

        ``SHO`` writes its bit to ``out`` with the rules of 6.3 (a read-only
        or reserved index is ignored) and ``SHI`` reads ``in`` as an
        instruction sees it (a pad, a ``PIN_OUT`` bit, or 0 for anything
        else, SEMANTICS 3), so both index spaces are worth covering.
        """
        roll = self.rng.random()
        if roll < 0.40:
            out = self.rng.choice(self.bidir)
        elif roll < 0.70:
            out = self.rng.choice(self.outputs)
        elif roll < 0.85:
            out = self.rng.choice(self.inputs)           # read-only: ignored
        else:
            out = self.rng.choice(self.reserved_pins)    # reserved: ignored
        roll = self.rng.random()
        if roll < 0.45:
            pin_in = self.rng.choice(self.live_any)
        elif roll < 0.7:
            pin_in = self.rng.choice(self.outputs)        # reads PIN_OUT back
        elif roll < 0.85:
            pin_in = self.rng.choice(self.reserved_pins)  # reads 0
        else:
            pin_in = self.rng.randrange(32)
        return ((pin_in & 0x1F) << 5) | (out & 0x1F)

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
        if self.be and rng.random() < 0.7:
            # Give SHO/SHI real pins most of the time (reset is index 0 both).
            self.push(self.csrw_scratch("BE_PINS", self.be_pins_value()),
                      targetable=False)
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
        # The ordinary form; the D form (SEMANTICS 6.10) is _emit_setp_d.
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
        self.push(self.tdrel_words(self.rng.randrange(16)))

    def tdrel_words(self, offset: int) -> List[_Word]:
        return [self.enc("CSRR", rd=SCRATCH, csr=self.csr["NOW"]),
                self.enc("ADDI", rd=SCRATCH, imm=offset & 0x3F),
                self.enc("CSRW", csr=self.csr["TD"], ra=SCRATCH)]

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
                block = self.tdrel_words(rng.randrange(24))
            else:
                block = self.csrw_scratch(name, rng.randrange(48))
        elif name in ("OUTGRP", "INGRP") and rng.random() < 0.7:
            block = self.group_block(name)
        elif name == "PIN_OUT" and "csrw_pin_out_high_bits" in self.avoid:
            # PIN_OUT[15:14] have no pin; see docs/spec-questions/cosim.md.
            block = self.csrw_scratch(name, rng.randrange(0x4000))
        elif name == "BE_PINS" and self.be and rng.random() < 0.6:
            block = self.csrw_scratch(name, self.be_pins_value())
        elif name == "BE_CFG" and self.be and rng.random() < 0.6:
            block = self.csrw_scratch(name, self.be_cfg_value())
        else:
            block = [self.enc("CSRW", csr=number, ra=self.any_reg())]
        if len(block) > self.room:
            return self._emit_nop()
        self.push(block)

    def _emit_reserved(self) -> None:
        word = self.rng.choice(self.reserved_words)
        self.push([_Word(word=word, mnemonic="")])

    def _emit_unbuilt(self) -> None:
        if not self.unbuilt_cycle:
            return self._emit_reserved()
        name = self.unbuilt_cycle[self.unbuilt_next % len(self.unbuilt_cycle)]
        self.unbuilt_next += 1
        self._emit_unbuilt_named(name)

    def _emit_unbuilt_named(self, name: str) -> None:
        """One instruction of an unbuilt feature, random operands."""
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

    # ------------------------------------------------------ M2: FIFOs (6.7)
    def _guarded_fifo(self, cond: int, name: str, **ops: int) -> None:
        """``[SETD k] WAITB cond, T; BT +1; PUSH|POP``: the FIFO op runs only
        when the ``WAITB`` ended by its condition, and then cannot stall (see
        the module docstring); on a timeout ``BT`` skips it."""
        words: List[_Word] = []
        optional = 0
        if self.rng.random() < 0.6:
            words.append(self.enc("SETD", imm=self.small_imm()))
            optional = 1                  # only the SETD may be dropped
        words += [self.enc("WAITB", cond=cond, tmo=1),
                  self.enc("BT", rel=1),              # to the word after the op
                  self.enc(name, **ops)]
        if not self.fit(words, droppable=optional):
            self._emit_nop()

    def _raw_fifo(self, name: str) -> bool:
        """May this region hold one more raw (blocking) ``PUSH``/``POP``?"""
        if not self.raw_left.get(name) or self.rng.random() < 0.5:
            return False
        self.raw_left[name] -= 1
        return True

    def _emit_fifo_push(self) -> None:
        if not self.fifo:
            return self._emit_unbuilt_named("PUSH")
        ra = self.any_reg()
        if self._raw_fifo("PUSH"):
            self.one("PUSH", ra=ra)                   # the host plan pops OUTQ[t]
        else:
            self._guarded_fifo(WAITB_OUTQ_NF, "PUSH", ra=ra)

    def _emit_fifo_pop(self) -> None:
        if not self.fifo:
            return self._emit_unbuilt_named("POP")
        rd = self.reg()
        if self._raw_fifo("POP"):
            self.one("POP", rd=rd)                    # the host plan pushes INQ[t]
        else:
            self._guarded_fifo(WAITB_INQ_NE, "POP", rd=rd)

    def _emit_waitb(self) -> None:
        if not self.fifo:
            return self._emit_unbuilt_named("WAITB")
        cond = self.rng.choice((WAITB_BE_IDLE, WAITB_OUTQ_NF, WAITB_OUTQ_NF,
                                WAITB_INQ_NE, WAITB_INQ_NE, WAITB_TICK,
                                WAITB_TICK))
        if cond == WAITB_TICK:
            live = True                               # ticks never stop
        elif cond == WAITB_BE_IDLE:
            live = True                               # true until auto mode (or BADOP)
        else:
            # Only the host can empty OUTQ or fill INQ, and it takes a round
            # of its plan to get to this thread: one untimed FIFO WAITB per
            # region, like the raw PUSH/POP.
            live = self._raw_fifo("WAITB")
        self._cond_wait("WAITB", live, force_timed=not live, cond=cond)

    # ------------------------------------------------- M2: bit engine (6.9)
    # -------------------------------------------- slice B: LD/ST (6.11)
    def _emit_mem(self) -> None:
        """One ``LD`` or ``ST`` into the thread's data window (SEMANTICS 6.11):
        ``LDI r7, lo; LDIH r7, hi; LD|ST rX, r7, imm5`` with ``imm5`` inside
        the window, so the access is real and never touches code. Two slots
        on both sides, the completion slot decoding nothing. In a build
        without the feature it is an unbuilt instruction as before."""
        if not self.dmem:
            return self._emit_unbuilt_named(self.rng.choice(("LD", "ST")))
        if self.room < 3:
            return self._emit_nop()
        name = self.rng.choice(("LD", "ST", "LD"))
        block = self.load_scratch(self.data_base) + [
            self.enc(name, rd=self.any_reg(), ra=SCRATCH,
                     imm=self.rng.randrange(DATA_WORDS))]
        self.push(block)

    def _emit_be_cfg(self) -> None:
        """``BE_CFG`` and/or ``BE_PINS`` from the scratch register."""
        if not self.be:
            return self._emit_unbuilt()
        rng = self.rng
        words: List[_Word] = []
        if rng.random() < 0.7:
            words += self.csrw_scratch("BE_CFG", self.be_cfg_value())
        if not words or rng.random() < 0.5:
            words += self.csrw_scratch("BE_PINS", self.be_pins_value())
        if not self.fit(words):
            self._emit_nop()

    def _emit_be_op(self) -> None:
        """One bit-engine instruction."""
        rng = self.rng
        name = rng.choice(("SHO", "SHO", "SHO", "SHI", "SHI", "SHI", "LDSR",
                           "LDSR", "STSR", "STSR", "CRCI", "STCRC", "STCRC"))
        if not self.be:
            return self._emit_unbuilt_named(name)
        if name == "LDSR":
            self.one(name, ra=self.any_reg())
        elif name in ("STSR", "STCRC"):
            self.one(name, rd=self.reg())
        else:
            self.one(name)

    def _emit_be_loop(self) -> None:
        """A transmit or receive loop counted by ``CNT`` (bounded, see the
        module docstring)."""
        if not self.be:
            return self._emit_unbuilt_named(self.rng.choice(("SHO", "SHI")))
        rng = self.rng
        n = rng.choice((0, 1, 2, 3, 4, 5, 7, 8, 12, 16))
        pace: List[_Word] = []
        roll = rng.random()
        if roll < 0.3:
            pace = [self.enc("DLY", imm=rng.choice((0, 1, 2, 3)))]
        elif roll < 0.5:
            pace = [self.enc("WAITD", imm=rng.choice((1, 1, 2, 4)))]
        head = self.load_scratch(n) + [self.enc("CSRW", csr=self.csr["CNT"], ra=SCRATCH)]
        back = -(2 + len(pace))              # BNZ -> the SHO/SHI
        if rng.random() < 0.5:
            words = [self.enc("LDSR", ra=self.any_reg())] + head + \
                [self.enc("SHO")] + pace + [self.enc("BNZ", rel=back)]
        else:
            rd = self.reg()
            words = head + [self.enc("SHI")] + pace + [self.enc("BNZ", rel=back),
                                                       self.enc("STSR", rd=rd)]
            if 0 < n < 16 and rng.random() < 0.5:
                words.append(self.enc("SHRI", rd=rd, imm=16 - n))
        if not self.fit(words):
            self._emit_be_op()

    def _emit_crc(self) -> None:
        """A ``.crc`` preset loaded left-aligned (SEMANTICS 6.9), ``CRCI``, and
        ``CRC_EN`` switched on; the loops and single shifts then update it."""
        if not self.be:
            return self._emit_unbuilt_named(self.rng.choice(("CRCI", "STCRC")))
        rng = self.rng
        presets = self.isa.crc_presets
        preset = presets[rng.choice(sorted(presets))]
        width = int(preset["width"])
        poly = (int(preset["poly"]) << (16 - width)) & 0xFFFF
        init = (int(preset["init"]) << (16 - width)) & 0xFFFF
        words = self.csrw_scratch("CRC_POLY", poly) + \
            self.csrw_scratch("CRC_INIT", init) + [self.enc("CRCI")]
        if rng.random() < 0.8:
            words += self.csrw_scratch("BE_CFG", self.be_cfg_value(crc=True))
        if not self.fit(words):
            self._emit_be_op()

    # ------------------------------------------ M2: latched SETP (6.10)
    def _emit_setp_d(self) -> None:
        """``SETP pin, v, D`` and the deadline it lands on.

        Without ``"SETPD"`` the ``D`` bit is ignored and this is an ordinary
        ``SETP`` on both sides (SEMANTICS 5, ISA note)."""
        rng = self.rng
        words: List[_Word] = []
        optional = 0
        if rng.random() < 0.4:
            words.append(self.enc("SETD", imm=rng.choice(LATCH_IMMS)))
            optional = 1
        words.append(self.enc("SETP", pin=self.write_pin(),
                              val=rng.randrange(2), lat=1))
        roll = rng.random()
        if roll < 0.55:
            words.append(self.enc("WAITD", imm=rng.choice(LATCH_IMMS)))
        elif roll < 0.7:
            words.append(self.enc("SETD", imm=rng.choice(LATCH_IMMS)))
        elif roll < 0.8:
            words += self.tdrel_words(rng.choice(LATCH_IMMS))
        # else: left staged; it lands when NOW ticks onto TD (rule 1) or at
        # the next TD write of the thread (rule 2), or is replaced first.
        if not self.fit(words, droppable=optional):
            self._emit_setp()

    # ------------------------------------------------------------- assembly
    def build(self, weights: Dict[str, int], want_halt: bool) -> List[_Block]:
        self.emit_prologue()
        # The main loop starts by signalling the shared flags (most of the
        # time), so an untimed WAITS in another thread has something to wait
        # for on every lap.
        for flag in self.flag_pool:
            if self.rng.random() < 0.75:
                self.one("SIG", flag=flag)
        names = sorted(n for n in weights if weights[n] > 0)
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


def _check_fifo_liveness(prog: GeneratedProgram, isa: Isa, t: int,
                         words: Dict[int, int], targets: FrozenSet[int]) -> None:
    """Without host traffic, every ``PUSH``/``POP`` is guarded (module
    docstring) and no untimed ``WAITB OUTQ_NF``/``INQ_NE`` exists: the INQ
    is never pushed and the OUTQ never popped, so either would block for
    good. The guard is ``WAITB cond, T`` then ``BT +1`` right before the op,
    and nothing branches to the ``BT`` or the op."""
    guard_cond = {"PUSH": WAITB_OUTQ_NF, "POP": WAITB_INQ_NE}
    for addr, word in words.items():
        decoded = isa.decode(word)
        if decoded is None:
            continue
        name, fields = decoded[0].name, decoded[1]
        if name == "WAITB" and fields["cond"] in (WAITB_OUTQ_NF, WAITB_INQ_NE) \
                and not fields["tmo"]:
            raise LoomgenError("thread %d: untimed %s at %#x with no host traffic"
                               % (t, "WAITB", addr))
        if name not in guard_cond:
            continue
        bt = isa.decode(words.get(addr - 1, 0))
        wait = isa.decode(words.get(addr - 2, 0))
        ok = (bt is not None and bt[0].name == "BT" and bt[1]["rel"] == 1
              and wait is not None and wait[0].name == "WAITB"
              and wait[1]["cond"] == guard_cond[name] and wait[1]["tmo"] == 1
              and addr not in targets and addr - 1 not in targets)
        if not ok:
            raise LoomgenError("thread %d: %s at %#x is not guarded by WAITB %d, T; "
                               "BT +1 and there is no host traffic"
                               % (t, name, addr, guard_cond[name]))


def check_program(prog: GeneratedProgram, isa: Optional[Isa] = None) -> None:
    """Raise :class:`LoomgenError` unless ``prog`` keeps every generator rule:
    words and targets inside their region, the region closed by a ``JMP`` to
    a head, no self-loop except a ``JP`` poll, no backward free ``DJNZ``,
    ``BT``/``BNT`` or ``JP``, and (FIFO build, no host traffic) every
    ``PUSH``/``POP`` guarded and no untimed FIFO ``WAITB``."""
    isa = isa or load_isa()
    fifo_guards = "FIFO" in prog.features and prog.host is None
    for t in prog.running():
        start, end = prog.region(t)
        words = {a: w for a, w in prog.image.items() if start <= a < end}
        if sorted(words) != list(range(start, end)):
            raise LoomgenError("thread %d region %#x..%#x is not fully written"
                               % (t, start, end))
        # Slice B: the top DATA_WORDS words of a DMEM build's region are the
        # data window, written but never code (SEMANTICS 6.11).
        if "DMEM" in prog.features:
            end -= DATA_WORDS
            words = {a: w for a, w in words.items() if a < end}
        last = isa.decode(words[end - 1])
        if last is None or last[0].name != "JMP":
            raise LoomgenError("thread %d region does not end with JMP" % t)
        targets = set()
        for addr, word in words.items():
            target = static_target(isa, addr, word)
            if target is None:
                continue
            targets.add(target)
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
        if fifo_guards:
            # A guard's own BT +1 targets the word after the op; only other
            # branches into the guard matter.
            _check_fifo_liveness(prog, isa, t, words, frozenset(targets))
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
             isa: Optional[Isa] = None,
             features: Iterable[str] = DEFAULT_FEATURES,
             fifo_depth: int = DEFAULT_FIFO_DEPTH,
             host_traffic: bool = False,
             weights: Optional[Dict[str, int]] = None) -> GeneratedProgram:
    """Build one random program.

    Args:
        seed: everything is derived from this; the same arguments give the
            same image, entry points, running threads and stimulus plan.
        threads: how many threads get a program and are started, 1 to 4.
            Which ones is part of the random draw unless ``run_mask`` is given.
        imem_words: instruction-memory size, a power of two from 64 to 1024.
            Each thread's region is a quarter of it.
        profile: ``"alu"``, ``"timing"``, ``"pins"``, ``"mixed"`` or ``"m2"``.
        cycles: how long the run is expected to be (recorded in the plan).
        entries: per-thread start addresses; each thread's region is
            ``[entry, entry + imem_words / 4)``. The default is D-017's
            ``t * (IMEM_WORDS / 4)``, which is what both implementations
            reset to, so the harness only has to write ``RUN``.
        avoid: names from :data:`AVOID_FLAGS`; each switches off a construct
            that ``docs/spec-questions/cosim.md`` has open.
        run_mask: which threads run; overrides the random choice (and
            ``threads``).
        features, fifo_depth: the build the program is for (``CAPS``); see
            the module docstring. ``features=()`` is the M1 build.
        host_traffic: the program comes with a :class:`HostPlan` of SPI
            traffic and may block on it (raw ``PUSH``/``POP``); needs FIFOs.
        weights: emitter weights to use instead of the profile's (tests).
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
    feature_names, fifo_depth = normalise_build(features, fifo_depth)
    if host_traffic and "FIFO" not in feature_names:
        raise LoomgenError("host traffic needs a build with FIFOs")
    if weights is not None:
        bad = [n for n in weights if not hasattr(_ThreadBuilder, "_emit_" + n)]
        if bad or not any(v > 0 for v in weights.values()):
            raise LoomgenError("bad weights: %s" % (", ".join(bad) or "all zero"))

    isa = isa or load_isa()
    rng = random.Random()
    rng.seed("loomgen|%d|%d|%d|%s|%s|%s|%s|%s|%d|%d|%s" % (
        seed, threads, imem_words, profile, ",".join(sorted(avoid)),
        ",".join(str(e) for e in entries) if entries is not None else "-",
        run_mask if run_mask is not None else "-",
        ",".join(feature_names), fifo_depth, int(bool(host_traffic)),
        ",".join("%s=%d" % kv for kv in sorted(weights.items())) if weights else "-"))

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
    chosen_weights = dict(weights) if weights is not None else PROFILES[profile]
    flag_pool = rng.sample(range(8), rng.choice((1, 2, 2, 3)))

    image: Dict[int, int] = {}
    heads: List[int] = []
    for thread in running:
        builder = _ThreadBuilder(rng, isa, thread, stride, profile, plan,
                                 len(running) > 1, flag_pool, avoid,
                                 features=feature_names, fifo_depth=fifo_depth,
                                 host_traffic=host_traffic,
                                 entry=entry_list[thread])
        blocks = builder.build(chosen_weights, rng.random() < HALT_RATE)
        _place(blocks, entry_list[thread])
        loop_head = blocks[builder.loop_head_index].addr
        image.update(_resolve(blocks, isa, rng, loop_head))
        heads += [b.addr for b in blocks if b.targetable]
        if "DMEM" in feature_names:
            # The data window is in the image, so a load reads a known word.
            base = entry_list[thread] + stride - DATA_WORDS
            for i in range(DATA_WORDS):
                image.setdefault(base + i, rng.randrange(1 << 16))

    host = build_host_plan(rng, run_mask, fifo_depth) if host_traffic else None
    prog = GeneratedProgram(
        seed=seed, profile=profile, threads=len(running),
        imem_words=imem_words, image=image, entries=entry_list,
        run_mask=run_mask, stimulus=plan, avoid=tuple(sorted(avoid)),
        heads=frozenset(heads), features=feature_names, fifo_depth=fifo_depth,
        host=host)
    check_program(prog, isa)
    return prog
