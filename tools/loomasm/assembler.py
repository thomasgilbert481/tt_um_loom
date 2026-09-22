"""Two-pass assembler for the Loom instruction set.

Pass 1 walks the statements to place every word: it applies ``.thread`` and
``.org``, defines labels and ``.equ`` symbols, sizes each statement, and checks
that thread sections do not overlap. Pass 2 encodes, through
``tools.loomisa`` only, and reports operand errors with file and line.

Every diagnostic in a file is collected; nothing stops at the first error.
"""

from __future__ import annotations

import dataclasses
import json
import os
import pathlib
from typing import Dict, List, Optional, Sequence, Tuple

from tools.loomisa import Isa, IsaError, Instr, load, operand_base

from . import deadline as dl
from .diag import (DEADLINE, ERROR, LAYOUT, PIN, RANGE, SYMBOL, SYNTAX,
                   WARNING, AsmError, Diagnostic)
from .disasm import disassemble
from .expr import ExprError, UnresolvedSymbol, evaluate, symbols_in
from .lexer import NAME, STRING, Token
from .names import (FLAG_TOKENS, TIMEOUT_TOKEN, csr_table, enum_tables, pin_table,
                    register_number)
from .parser import Stmt, parse_source

DIRECTIVES = frozenset({
    ".thread", ".org", ".equ", ".pins", ".word", ".csr", ".tick",
    ".deadline_check", ".imem", ".bounded",
})
PSEUDO_OPS = frozenset({"MOV16", "BRA", "INC", "DEC"})

DEFAULT_SCRATCH = 7                    # r7, the .csr / MOV16 scratch register

#: Instruction-memory size the assembler lays out for when nothing says
#: otherwise. Thread t's section starts at ``t * (IMEM_WORDS / threads)``
#: (``docs/DECISIONS.md`` D-017, ``docs/SEMANTICS.md`` section 5), so the
#: default keeps the historical 0x100 spacing.
DEFAULT_IMEM_WORDS = 1024
MIN_IMEM_WORDS = 64


def imem_choices(pc_bits: int = 10) -> "List[int]":
    """The instruction-memory sizes the assembler accepts: powers of two from
    ``MIN_IMEM_WORDS`` up to what the program counter can address."""
    sizes, size = [], MIN_IMEM_WORDS
    while size <= (1 << pc_bits):
        sizes.append(size)
        size <<= 1
    return sizes


class _Bad(Exception):
    """Internal: one statement-level problem, turned into a Diagnostic."""

    def __init__(self, message: str, token: "Optional[Token]" = None,
                 kind: str = SYNTAX):
        super().__init__(message)
        self.message = message
        self.token = token
        self.kind = kind


@dataclasses.dataclass(frozen=True)
class WordInfo:
    addr: int
    word: int
    thread: int
    line: int
    timing: str
    text: str
    #: the reason of a ``.bounded`` declaration on this word, or None
    bounded: Optional[str] = None


@dataclasses.dataclass(frozen=True)
class ThreadInfo:
    thread: int
    entry: int
    size: int


@dataclasses.dataclass
class Program:
    """The result of assembling one source file."""

    isa_version: str
    source: str
    imem_words: int = DEFAULT_IMEM_WORDS
    words: Dict[int, int] = dataclasses.field(default_factory=dict)
    symbols: Dict[str, int] = dataclasses.field(default_factory=dict)
    threads: Dict[int, ThreadInfo] = dataclasses.field(default_factory=dict)
    listing: List[str] = dataclasses.field(default_factory=list)
    diagnostics: List[Diagnostic] = dataclasses.field(default_factory=list)
    deadlines: Dict[int, "dl.ThreadDeadlines"] = dataclasses.field(default_factory=dict)
    word_info: List[WordInfo] = dataclasses.field(default_factory=list)
    #: address -> the reason of the ``.bounded`` declaration on it
    bounded: Dict[int, str] = dataclasses.field(default_factory=dict)

    @property
    def errors(self) -> List[Diagnostic]:
        return [d for d in self.diagnostics if d.severity == ERROR]

    @property
    def warnings(self) -> List[Diagnostic]:
        return [d for d in self.diagnostics if d.severity == WARNING]

    @property
    def deadline_errors(self) -> List[Diagnostic]:
        return [d for d in self.errors if d.kind == DEADLINE]

    @property
    def ok(self) -> bool:
        return not self.errors

    def to_image(self) -> dict:
        """The JSON image the golden model and the host library load."""
        return {
            "isa": self.isa_version,
            "imem_words": self.imem_words,
            "words": {str(a): self.words[a] for a in sorted(self.words)},
            "symbols": {k: self.symbols[k] for k in sorted(self.symbols)},
            "threads": {
                str(t): {"entry": self.threads[t].entry,
                         "size": self.threads[t].size}
                for t in sorted(self.threads)
            },
            "source": self.source,
        }

    def image_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_image(), indent=indent) + "\n"

    def write_image(self, path) -> pathlib.Path:
        path = pathlib.Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.image_json(), encoding="utf-8", newline="\n")
        return path

    def listing_text(self) -> str:
        return "\n".join(self.listing) + "\n"


class _Assembler:
    def __init__(self, isa: Isa, filename: str,
                 imem_words: "Optional[int]" = None):
        self.isa = isa
        self.filename = filename
        self.threads = int(isa.meta["threads"])
        self.pc_bits = int(isa.meta["pc_bits"])
        self.slot_clocks = int(isa.meta["slot_clocks"])
        self.pc_mask = (1 << self.pc_bits) - 1      # PC wraps at 2^pc_bits
        #: True when the caller fixed the size, so a ``.imem`` only warns.
        self.imem_forced = imem_words is not None
        self.imem_words = (imem_words if imem_words is not None
                           else DEFAULT_IMEM_WORDS)
        if self.imem_words not in imem_choices(self.pc_bits):
            raise ValueError(
                "imem_words=%r is not a power of two from %d to %d"
                % (imem_words, MIN_IMEM_WORDS, 1 << self.pc_bits))
        #: Set once an address has been committed, after which ``.imem`` would
        #: move code that is already placed.
        self.layout_locked = False
        self.pins = pin_table(isa)
        self.csrs = csr_table(isa)
        self.enums = enum_tables(isa)          # operand base -> {NAME: value}
        self.td_csr = isa.csr_by_name.get("TD")

        self.labels: Dict[str, int] = {}
        self.equs: Dict[str, int] = {}
        self.pin_aliases: Dict[str, int] = {}
        self.words: Dict[int, int] = {}
        self.owner: Dict[int, Tuple[int, int]] = {}
        self.diagnostics: List[Diagnostic] = []
        self.word_info: List[WordInfo] = []
        self.nodes: Dict[int, List["dl.Node"]] = {t: [] for t in range(self.threads)}
        self.tick: Dict[int, Optional[int]] = {t: None for t in range(self.threads)}
        self.check_enabled: Dict[int, bool] = {t: True for t in range(self.threads)}
        self.first_addr: Dict[int, int] = {}
        self.count: Dict[int, int] = {t: 0 for t in range(self.threads)}
        #: address -> the reason text of the ``.bounded`` declaration on it
        self.bounded: Dict[int, str] = {}
        #: a ``.bounded`` seen but not yet attached to an instruction
        self.pending_bounded: "Optional[Tuple[str, Stmt]]" = None

        self.thread = 0
        self.pcs: Dict[int, int] = {t: self.thread_origin(t)
                                    for t in range(self.threads)}

    def thread_origin(self, thread: int) -> int:
        """Thread t's reset vector: ``t * (IMEM_WORDS / threads)`` (D-017)."""
        return thread * (self.imem_words // self.threads)

    # ------------------------------------------------------------ diagnostics
    def error(self, stmt: Stmt, message: str, token: "Optional[Token]" = None,
              kind: str = SYNTAX) -> None:
        self.diagnostics.append(Diagnostic(
            ERROR, kind, self.filename, stmt.line, message,
            token.col if token is not None else 0))

    def warn(self, stmt: Stmt, message: str, kind: str = SYNTAX) -> None:
        self.diagnostics.append(Diagnostic(
            WARNING, kind, self.filename, stmt.line, message))

    # ---------------------------------------------------------------- symbols
    def resolve(self, name: str) -> int:
        if name in self.equs:
            return self.equs[name]
        if name in self.labels:
            return self.labels[name]
        raise UnresolvedSymbol(name)

    def eval_tokens(self, tokens: Sequence[Token], what: str) -> int:
        if not tokens:
            raise _Bad("missing %s" % what)
        try:
            return evaluate(list(tokens), self.resolve)
        except UnresolvedSymbol as exc:
            raise _Bad(exc.message, exc.token or tokens[0], SYMBOL) from None
        except ExprError as exc:
            raise _Bad("%s in %s" % (exc.message, what), exc.token or tokens[0]) from None

    def define(self, name: str, value: int, stmt: Stmt, what: str) -> None:
        if register_number(name) is not None:
            self.error(stmt, "'%s' is a register name and cannot be a %s"
                       % (name, what), kind=SYMBOL)
            return
        if name in self.labels or name in self.equs:
            self.error(stmt, "'%s' is already defined" % name, kind=SYMBOL)
            return
        if what == "label":
            self.labels[name] = value
        else:
            self.equs[name] = value

    # ------------------------------------------------------------------ pass 1
    def size_of(self, stmt: Stmt) -> int:
        """Words this statement emits; also records the MOV16 form chosen."""
        upper, lower = stmt.upper, stmt.lower
        if stmt.head is None:
            return 0
        if lower == ".word":
            if not stmt.args or any(not g for g in stmt.args):
                raise _Bad(".word needs at least one value", stmt.head)
            return len(stmt.args)
        if lower == ".csr":
            if len(stmt.args) not in (2, 3):
                raise _Bad(".csr takes NAME, value [, rN]", stmt.head)
            stmt.short = self.mov16_is_short(stmt.args[1])
            return (1 if stmt.short else 2) + 1
        if upper == "MOV16":
            if len(stmt.args) != 2:
                raise _Bad("MOV16 takes rd, imm16", stmt.head)
            stmt.short = self.mov16_is_short(stmt.args[1])
            return 1 if stmt.short else 2
        if upper in PSEUDO_OPS:
            return 1
        if stmt.is_directive:
            return 0
        if upper not in self.isa.by_name:
            raise _Bad("unknown instruction '%s'" % stmt.name, stmt.head)
        return 1

    def mov16_is_short(self, tokens: Sequence[Token]) -> bool:
        """One word (LDI only) iff the value is known now and fits in 8 bits."""
        try:
            value = evaluate(list(tokens), self.resolve)
        except (ExprError, IndexError):
            return False
        return 0 <= value <= 0xFF

    def pass1(self, stmts: List[Stmt]) -> None:
        for stmt in stmts:
            if stmt.broken:
                continue
            lower = stmt.lower
            try:
                if lower == ".thread":
                    self.do_thread(stmt)
                elif lower == ".org":
                    self.do_org(stmt)
            except _Bad as bad:
                self.error(stmt, bad.message, bad.token, bad.kind)

            stmt.thread = self.thread
            stmt.addr = self.pcs[self.thread]
            for label in stmt.labels:
                # A label fixes an address, so the memory size can no longer move.
                self.layout_locked = True
                self.define(label, stmt.addr, stmt, "label")

            if stmt.head is None or lower in (".thread", ".org"):
                continue
            try:
                if lower == ".imem":
                    self.do_imem(stmt)
                    stmt.thread = self.thread
                    stmt.addr = self.pcs[self.thread]
                elif lower == ".equ":
                    self.do_equ(stmt)
                elif lower == ".pins":
                    self.do_pins(stmt)
                elif lower == ".tick":
                    self.do_tick(stmt)
                elif lower == ".deadline_check":
                    self.do_deadline_check(stmt)
                elif lower == ".bounded":
                    self.do_bounded(stmt)
                elif stmt.is_directive and lower not in DIRECTIVES:
                    raise _Bad("unknown directive '%s'" % stmt.name, stmt.head)
                if lower == ".csr":
                    self.note_tick_csr(stmt)
                stmt.size = self.size_of(stmt)
                stmt.is_data = lower == ".word"
            except _Bad as bad:
                self.error(stmt, bad.message, bad.token, bad.kind)
                stmt.size = 0
                continue
            if lower != ".bounded":
                self.attach_bounded(stmt)
            self.reserve(stmt)
        if self.pending_bounded is not None:
            _, where = self.pending_bounded
            self.pending_bounded = None
            self.error(where, ".bounded applies to the instruction after it, "
                              "and none follows", where.head)

    def reserve(self, stmt: Stmt) -> None:
        for offset in range(stmt.size):
            addr = stmt.addr + offset
            if addr >= self.imem_words:
                self.error(stmt, "address 0x%03X is past the end of instruction "
                                 "memory (%d words)" % (addr, self.imem_words),
                           stmt.head, LAYOUT)
                stmt.size = 0
                return
            if addr in self.owner:
                other_thread, other_line = self.owner[addr]
                self.error(stmt, "address 0x%03X is already used by thread %d "
                                 "(line %d): thread sections overlap"
                           % (addr, other_thread, other_line), stmt.head, LAYOUT)
                stmt.size = 0
                return
            self.owner[addr] = (stmt.thread, stmt.line)
        if stmt.size:
            self.layout_locked = True
            self.first_addr.setdefault(stmt.thread, stmt.addr)
            self.count[stmt.thread] += stmt.size
        self.pcs[self.thread] = stmt.addr + stmt.size

    # ------------------------------------------------------------- directives
    def one_arg(self, stmt: Stmt, what: str) -> List[Token]:
        if len(stmt.args) != 1 or not stmt.args[0]:
            raise _Bad("%s takes exactly one %s" % (stmt.name, what), stmt.head)
        return stmt.args[0]

    def do_thread(self, stmt: Stmt) -> None:
        value = self.eval_tokens(self.one_arg(stmt, "thread number"),
                                 "thread number")
        if not 0 <= value < self.threads:
            raise _Bad("thread %d does not exist (0..%d)"
                       % (value, self.threads - 1), stmt.head, RANGE)
        self.thread = value

    def do_org(self, stmt: Stmt) -> None:
        value = self.eval_tokens(self.one_arg(stmt, "address"), "address")
        if not 0 <= value < self.imem_words:
            raise _Bad(".org 0x%X is outside instruction memory (0..0x%X)"
                       % (value, self.imem_words - 1), stmt.head, RANGE)
        self.layout_locked = True
        self.pcs[self.thread] = value

    def do_imem(self, stmt: Stmt) -> None:
        """``.imem W``: the instruction-memory size this program is laid out for.

        It must come before any code, because it moves every thread's default
        origin. An ``--imem-words`` on the command line wins over it.
        """
        value = self.eval_tokens(self.one_arg(stmt, "word count"),
                                 "the memory size")
        choices = imem_choices(self.pc_bits)
        if value not in choices:
            raise _Bad(".imem %d is not a valid memory size (a power of two "
                       "from %d to %d)" % (value, choices[0], choices[-1]),
                       stmt.head, RANGE)
        if self.imem_forced:
            if value != self.imem_words:
                self.warn(stmt, ".imem %d ignored: --imem-words %d was given on "
                                "the command line" % (value, self.imem_words),
                          LAYOUT)
            return
        if self.layout_locked:
            raise _Bad(".imem must come before any code: it moves every "
                       "thread's default origin", stmt.head, LAYOUT)
        self.imem_words = value
        self.pcs = {t: self.thread_origin(t) for t in range(self.threads)}

    def do_equ(self, stmt: Stmt) -> None:
        if len(stmt.args) != 1:
            raise _Bad(".equ takes NAME = expression", stmt.head)
        name_tokens, value_tokens = self.split_assignment(stmt.args[0], ".equ")
        value = self.eval_tokens(value_tokens, "the .equ value")
        self.define(name_tokens.text, value, stmt, "symbol")

    def do_pins(self, stmt: Stmt) -> None:
        if not stmt.args or any(not g for g in stmt.args):
            raise _Bad(".pins takes NAME = PIN [, NAME = PIN ...]", stmt.head)
        for group in stmt.args:
            name_token, value_tokens = self.split_assignment(group, ".pins")
            value = self.pin_value(value_tokens)
            key = name_token.text.upper()
            if key in self.pins:
                raise _Bad("'%s' is a pin name in isa.yaml and cannot be "
                           "redefined" % name_token.text, name_token, SYMBOL)
            if key in self.pin_aliases and self.pin_aliases[key] != value:
                raise _Bad("pin alias '%s' is already defined as %d"
                           % (name_token.text, self.pin_aliases[key]),
                           name_token, SYMBOL)
            self.pin_aliases[key] = value

    def split_assignment(self, group: List[Token],
                         who: str) -> Tuple[Token, List[Token]]:
        if len(group) < 3 or group[0].kind != NAME or not group[1].is_punct("="):
            raise _Bad("%s expects NAME = value" % who,
                       group[0] if group else None)
        return group[0], group[2:]

    def do_tick(self, stmt: Stmt) -> None:
        value = self.eval_tokens(self.one_arg(stmt, "clock count"),
                                 "the tick period")
        if value < 1:
            raise _Bad(".tick needs at least 1 clock, got %d" % value,
                       stmt.head, RANGE)
        self.tick[self.thread] = value

    def do_bounded(self, stmt: Stmt) -> None:
        """``.bounded "<reason>"``: the next ``PUSH``/``POP`` cannot stall.

        The reason is the author's argument, which the checker takes on trust
        and never verifies (``tools/loomasm/README.md``, "Deadline checker").
        It is mandatory so that the assumption is written down where a reader
        of the source meets it.
        """
        group = self.one_arg(stmt, "quoted reason")
        if len(group) != 1 or group[0].kind != STRING:
            raise _Bad('.bounded takes one quoted reason, as .bounded "why '
                       'this cannot stall"', group[0] if group else stmt.head)
        reason = group[0].text.strip()
        if not reason:
            raise _Bad(".bounded needs a reason: it is the argument that the "
                       "instruction cannot stall, and the checker cannot make "
                       "it for you", group[0])
        if self.pending_bounded is not None:
            raise _Bad(".bounded from line %d has not been used yet: one "
                       "declaration covers one instruction"
                       % self.pending_bounded[1].line, stmt.head)
        self.pending_bounded = (reason, stmt)

    def attach_bounded(self, stmt: Stmt) -> None:
        """Give a pending ``.bounded`` to the next statement that emits words."""
        if self.pending_bounded is None or stmt.size == 0:
            return
        reason, where = self.pending_bounded
        self.pending_bounded = None
        what = "the data of a .word" if stmt.is_data else stmt.upper
        if stmt.is_data or stmt.upper not in dl.BOUNDABLE:
            self.error(where, ".bounded applies to %s, but the next "
                              "instruction (line %d) is %s"
                       % (" or ".join(sorted(dl.BOUNDABLE)), stmt.line, what),
                       where.head)
            return
        self.bounded[stmt.addr] = reason

    def do_deadline_check(self, stmt: Stmt) -> None:
        group = self.one_arg(stmt, "setting")
        if len(group) != 1 or group[0].kind != NAME or \
                group[0].text.lower() not in ("on", "off"):
            raise _Bad(".deadline_check takes 'on' or 'off'", stmt.head)
        self.check_enabled[self.thread] = group[0].text.lower() == "on"

    def note_tick_csr(self, stmt: Stmt) -> None:
        """``.csr TICK_INT, <constant>`` also declares the tick period."""
        if len(stmt.args) < 2 or len(stmt.args[0]) != 1:
            return
        if stmt.args[0][0].kind != NAME or stmt.args[0][0].text.upper() != "TICK_INT":
            return
        try:
            value = evaluate(list(stmt.args[1]), self.resolve)
        except ExprError:
            return
        self.tick[self.thread] = max(int(value), 1)

    # ------------------------------------------------------------------ pass 2
    def pass2(self, stmts: List[Stmt]) -> None:
        for stmt in stmts:
            if stmt.broken or stmt.head is None or stmt.size == 0:
                continue
            upper = stmt.upper
            try:
                if stmt.lower == ".word":
                    words = [self.data_word(group) for group in stmt.args]
                elif stmt.lower == ".csr":
                    words = self.encode_csr_directive(stmt)
                elif upper in PSEUDO_OPS:
                    words = self.encode_pseudo(stmt)
                else:
                    words = [self.encode_instruction(
                        self.isa.by_name[upper], stmt, stmt.args, stmt.addr)]
            except _Bad as bad:
                self.error(stmt, bad.message, bad.token, bad.kind)
                continue
            except IsaError as exc:
                self.error(stmt, str(exc), stmt.head, RANGE)
                continue
            if len(words) != stmt.size:
                self.error(stmt, "internal error: sized %d words, encoded %d"
                           % (stmt.size, len(words)), stmt.head)
                continue
            self.emit(stmt, words)

    def emit(self, stmt: Stmt, words: Sequence[int]) -> None:
        for offset, word in enumerate(words):
            addr = stmt.addr + offset
            word &= 0xFFFF
            self.words[addr] = word
            decoded = self.isa.decode(word)
            if stmt.is_data or decoded is None:
                timing = "data" if stmt.is_data else "reserved"
                text = ".word 0x%04X" % word
            else:
                instr, fields = decoded
                timing = instr.timing
                text = disassemble(word, self.isa)
                self.nodes[stmt.thread].append(dl.Node(
                    addr=addr, name=instr.name, fields=fields, line=stmt.line,
                    timing=instr.timing, bounded=self.bounded.get(addr)))
            self.word_info.append(WordInfo(
                addr=addr, word=word, thread=stmt.thread, line=stmt.line,
                timing=timing, text=text, bounded=self.bounded.get(addr)))

    def data_word(self, group: Sequence[Token]) -> int:
        value = self.eval_tokens(group, "a .word value")
        if not -0x8000 <= value <= 0xFFFF:
            raise _Bad(".word value %d does not fit in 16 bits" % value,
                       group[0], RANGE)
        return value & 0xFFFF

    # ------------------------------------------------------------- pseudo-ops
    def encode_pseudo(self, stmt: Stmt) -> List[int]:
        upper = stmt.upper
        if upper == "BRA":
            if len(stmt.args) != 1:
                raise _Bad("BRA takes one target", stmt.head)
            return [self.encode_instruction(
                self.isa.by_name["JMP"], stmt, stmt.args, stmt.addr)]
        if upper in ("INC", "DEC"):
            if len(stmt.args) != 1:
                raise _Bad("%s takes one register" % stmt.name, stmt.head)
            real = self.isa.by_name["ADDI" if upper == "INC" else "SUBI"]
            return [real.encode(rd=self.register(stmt.args[0]), imm=1)]
        if upper == "MOV16":
            return self.encode_mov16(stmt, self.register(stmt.args[0]),
                                     stmt.args[1], stmt.short)
        raise _Bad("unknown pseudo-op '%s'" % stmt.name, stmt.head)

    def encode_mov16(self, stmt: Stmt, rd: int, tokens: Sequence[Token],
                     short: bool) -> List[int]:
        value = self.eval_tokens(tokens, "a 16-bit value")
        if not -0x8000 <= value <= 0xFFFF:
            raise _Bad("%d does not fit in 16 bits" % value,
                       tokens[0] if tokens else stmt.head, RANGE)
        value &= 0xFFFF
        words = [self.isa.by_name["LDI"].encode(rd=rd, imm=value & 0xFF)]
        if not short:
            words.append(self.isa.by_name["LDIH"].encode(rd=rd, imm=value >> 8))
        return words

    def encode_csr_directive(self, stmt: Stmt) -> List[int]:
        group = stmt.args[0]
        if len(group) != 1 or group[0].kind != NAME:
            number = self.eval_tokens(group, "a CSR number")
            if not 0 <= number <= 31:
                raise _Bad("CSR number %d is outside 0..31" % number,
                           group[0], RANGE)
        else:
            number = self.csr_number(group[0])
        scratch = DEFAULT_SCRATCH
        if len(stmt.args) == 3:
            scratch = self.register(stmt.args[2])
        words = self.encode_mov16(stmt, scratch, stmt.args[1], stmt.short)
        words.append(self.isa.by_name["CSRW"].encode(csr=number, ra=scratch))
        return words

    # ----------------------------------------------------------- instructions
    def encode_instruction(self, instr: Instr, stmt: Stmt,
                           args: List[List[Token]], addr: int) -> int:
        ops = list(instr.ops)
        given: List[Optional[List[Token]]] = list(args)
        if len(given) == 1 and not given[0]:
            given = []
        if ops and operand_base(ops[-1]) in FLAG_TOKENS and len(given) == len(ops) - 1:
            given.append(None)
        if len(given) != len(ops):
            raise _Bad("%s takes %d operand%s (%s), %d given" % (
                instr.name, len(ops), "" if len(ops) == 1 else "s",
                ", ".join(ops) or "none", len(args)), stmt.head)
        values = {}
        for op, group in zip(ops, given):
            base = operand_base(op)
            if group is not None and not group:
                raise _Bad("missing operand '%s' of %s" % (op, instr.name),
                           stmt.head)
            values[base] = self.operand_value(instr, op, base, group, addr, stmt)
        return instr.encode(**values)

    def operand_value(self, instr: Instr, op: str, base: str,
                      group: "Optional[List[Token]]", addr: int,
                      stmt: "Optional[Stmt]" = None) -> int:
        if base in FLAG_TOKENS:
            token = FLAG_TOKENS[base]
            if group is None:
                return 0
            if len(group) == 1 and group[0].kind == NAME and \
                    group[0].text.upper() == token:
                return 1
            value = self.eval_tokens(group, "the %s operand" % base)
            if value not in (0, 1):
                raise _Bad("the %s operand is the bare token %s (or 0/1), "
                           "not %d" % (base, token, value), group[0], RANGE)
            return value
        assert group is not None
        if base in ("rd", "ra", "rb"):
            return self.register(group)
        if base == "pin":
            return self.pin_value(group, stmt)
        if base == "csr":
            if len(group) == 1 and group[0].kind == NAME:
                return self.csr_number(group[0])
            value = self.eval_tokens(group, "a CSR number")
            if not 0 <= value <= 31:
                raise _Bad("CSR number %d is outside 0..31" % value,
                           group[0], RANGE)
            return value
        if base in self.enums:                       # edge, cond, ... from enums:
            return self.enum_value(group, base)
        if base == "val":
            value = self.eval_tokens(group, "a pin level")
            if value not in (0, 1):
                raise _Bad("a pin level is 0 or 1, not %d" % value,
                           group[0], RANGE)
            return value
        if base == "rel":
            return self.rel_value(instr, op, group, addr)
        return self.eval_tokens(group, "operand '%s'" % op)

    def register(self, group: Sequence[Token]) -> int:
        if len(group) != 1 or group[0].kind != NAME:
            raise _Bad("expected a register r0..r7",
                       group[0] if group else None)
        number = register_number(group[0].text)
        if number is None:
            raise _Bad("'%s' is not a register (r0..r7)" % group[0].text,
                       group[0])
        return number

    def pin_value(self, group: Sequence[Token],
                  stmt: "Optional[Stmt]" = None) -> int:
        """Resolve a pin operand.

        ``stmt`` is the statement to hang the "index has no name" warning on;
        pass None (as ``.pins`` does) to resolve without warning, so that a
        declaration warns once at each use rather than twice.
        """
        if len(group) == 1 and group[0].kind == NAME:
            key = group[0].text.upper()
            if key in self.pin_aliases:
                return self.warn_unnamed_pin(self.pin_aliases[key], stmt)
            if key in self.pins:
                return self.pins[key]               # a named pin never warns
        value = self.eval_tokens(group, "a pin")
        if not 0 <= value <= 31:
            raise _Bad("pin index %d is outside 0..31" % value, group[0], RANGE)
        return self.warn_unnamed_pin(value, stmt)

    def warn_unnamed_pin(self, value: int, stmt: "Optional[Stmt]") -> int:
        """Warn about an index isa.yaml does not name, however it was written."""
        if stmt is not None and value not in self.isa.pins:
            self.warn(stmt, "pin index %d has no name in isa.yaml, so it is "
                            "not a pad: writes to it are ignored and reads of "
                            "it return 0" % value, PIN)
        return value

    def csr_number(self, token: Token) -> int:
        key = token.text.upper()
        if key in self.csrs:
            return self.csrs[key]
        try:
            return self.resolve(token.text)
        except UnresolvedSymbol:
            raise _Bad("unknown CSR '%s'" % token.text, token, SYMBOL) from None

    def enum_value(self, group: Sequence[Token], base: str) -> int:
        """An operand listed in ``isa.yaml`` ``enums``: a name or a number.

        The accepted numeric range is 0..max(value in the enum), not the field
        width, so an encoding the ISA does not define stays unreachable: the
        ``edge`` field is two bits but ``isa.yaml`` names only 0..2, and
        ``SEMANTICS.md`` 6.4 says ``e == 3`` is never true.
        """
        table = self.enums[base]
        high = max(table.values()) if table else 0
        if len(group) == 1 and group[0].kind == NAME:
            key = group[0].text.upper()
            if key in table:
                return table[key]
            if group[0].text not in self.equs and group[0].text not in self.labels:
                raise _Bad("unknown %s '%s' (use %s or 0..%d)" % (
                    base, group[0].text, ", ".join(sorted(table)), high),
                    group[0])
        value = self.eval_tokens(group, "an %s operand" % base)
        if not 0 <= value <= high:
            raise _Bad("%s %d is outside 0..%d" % (base, value, high),
                       group[0], RANGE)
        return value

    def rel_value(self, instr: Instr, op: str, group: Sequence[Token],
                  addr: int) -> int:
        width = instr.field_for(op).width
        low, high = -(1 << (width - 1)), (1 << (width - 1)) - 1
        if symbols_in(list(group)):
            target = self.eval_tokens(group, "a branch target")
            if not 0 <= target < self.imem_words:
                raise _Bad("branch target 0x%X is outside instruction memory"
                           % target, group[0], RANGE)
            offset = target - ((addr + 1) & self.pc_mask)   # PC wraps, not IMEM
            if not low <= offset <= high:
                raise _Bad(
                    "%s cannot reach 0x%03X from 0x%03X: offset %d, %s "
                    "holds %d..%d" % (instr.name, target, addr, offset, op,
                                      low, high), group[0], RANGE)
            return offset
        offset = self.eval_tokens(group, "a branch offset")
        if not low <= offset <= high:
            raise _Bad("%s offset %d does not fit %s (%d..%d)"
                       % (instr.name, offset, op, low, high), group[0], RANGE)
        return offset

    # ------------------------------------------------------------------ result
    def finish(self, stmts: List[Stmt], source: str,
               run_deadline_check: bool) -> Program:
        program = Program(isa_version=self.isa.version, source=source,
                          imem_words=self.imem_words)
        program.words = dict(self.words)
        program.symbols = dict(self.equs)
        program.symbols.update(self.labels)
        program.word_info = list(self.word_info)
        program.bounded = dict(self.bounded)
        for thread in range(self.threads):
            if self.count[thread]:
                program.threads[thread] = ThreadInfo(
                    thread=thread, entry=self.first_addr[thread],
                    size=self.count[thread])
        # A file that did not assemble has holes in its control-flow graph, so
        # its deadline numbers would be noise on top of the real errors.
        if run_deadline_check and not any(d.fatal for d in self.diagnostics):
            for thread in range(self.threads):
                if not self.nodes[thread] and thread not in program.threads:
                    continue
                report = dl.analyse_thread(
                    thread, self.nodes[thread], self.tick[thread],
                    self.pc_bits, self.check_enabled[thread],
                    self.slot_clocks, self.td_csr)
                program.deadlines[thread] = report
                self.diagnostics.extend(dl.diagnostics_for(report, self.filename))
        program.diagnostics = sorted(
            self.diagnostics, key=lambda d: (d.line, d.column, d.message))
        return program


# ---------------------------------------------------------------- entry points
def assemble_text(text: str, filename: str = "<text>", *, isa: "Optional[Isa]" = None,
                  strict: bool = False, deadline_check: bool = True,
                  imem_words: "Optional[int]" = None) -> Program:
    """Assemble source text. See :func:`assemble` for the argument meanings."""
    from .listing import build_listing

    isa = isa if isa is not None else load()
    stmts, diagnostics = parse_source(text, filename)
    asm = _Assembler(isa, filename, imem_words)
    asm.diagnostics.extend(diagnostics)
    asm.pass1(stmts)
    # Pass 2 always runs: statements that failed to size emit nothing, so every
    # operand error in the file is reported alongside the layout errors.
    asm.pass2(stmts)
    program = asm.finish(stmts, filename, deadline_check)
    program.listing = build_listing(program, stmts, isa)
    fatal = [d for d in program.diagnostics if d.fatal]
    if fatal or (strict and program.errors):
        raise AsmError(program.diagnostics)
    return program


def assemble_file(path, *, isa: "Optional[Isa]" = None, strict: bool = False,
                  deadline_check: bool = True,
                  imem_words: "Optional[int]" = None) -> Program:
    path = pathlib.Path(path)
    text = path.read_text(encoding="utf-8")
    return assemble_text(text, str(path).replace(os.sep, "/"), isa=isa,
                         strict=strict, deadline_check=deadline_check,
                         imem_words=imem_words)


def assemble(text_or_path, *, filename: "Optional[str]" = None,
             isa: "Optional[Isa]" = None, strict: bool = False,
             deadline_check: bool = True,
             imem_words: "Optional[int]" = None) -> Program:
    """Assemble a file or a block of source text.

    A :class:`pathlib.Path` is always read from disk. A ``str`` is treated as
    source text, unless it holds no newline and names a file that exists.
    ``strict`` also turns deadline errors into an :class:`AsmError`.

    ``imem_words`` is the instruction-memory size the program is laid out for:
    thread ``t``'s section starts at ``t * (imem_words / 4)`` (D-017). It
    defaults to 1024 and overrides a ``.imem`` in the source.
    """
    kwargs = dict(isa=isa, strict=strict, deadline_check=deadline_check,
                  imem_words=imem_words)
    if isinstance(text_or_path, pathlib.Path):
        return assemble_file(text_or_path, **kwargs)
    text = str(text_or_path)
    if "\n" not in text and text.strip() and len(text) < 4096:
        try:
            if os.path.isfile(text):
                return assemble_file(text, **kwargs)
        except OSError:
            pass
    return assemble_text(text, filename or "<text>", **kwargs)
