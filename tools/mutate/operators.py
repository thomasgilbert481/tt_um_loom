"""
tools.mutate.operators: a line-oriented mutator for the Loom Verilog subset.

VERIFICATION.md L7 (MUT-RUN) wants one-line faults injected into `src/*.v`.
This module produces them. It deliberately parses no more Verilog than it must:

  * a lexical pass masks out comments, string literals, compiler directives and
    text disabled by `ifdef`, so an operator never fires inside one;
  * a token pass over what is left tracks just enough context to tell a
    nonblocking assignment from a comparison, a bit-select from a declared
    range, and a one-bit signal from a vector;
  * each operator then rewrites exactly one token (or wraps one `if`
    condition), producing a mutant that differs from the original in one line.

Nothing here reads or writes `src/`: `mutations_for_text` works on a string and
`Mutation.apply` returns new text. The runner owns all file handling.

Operators (`OPERATORS`):

  arith     `+` <-> `-`
  relop     `<` <-> `<=`, `>` <-> `>=`   (never a nonblocking assignment)
  eqop      `==` <-> `!=`
  bitop     `&` <-> `|`, `&&` <-> `||`
  cond_inv  `if (X)` -> `if (!(X))`
  const     a numeric literal outside a bit-select, +/- 1 in its own width
  index     a constant inside a bit-select or part-select, +/- 1
  stuck     a one-bit control signal inside an `if` condition -> `1'b0`/`1'b1`

A mutant is identified by file, line, column, operator and a one-line
description, and two mutants that produce the same text are the same mutant.
"""

from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass
from typing import Dict, Iterable, Iterator, List, Optional, Sequence, Set, Tuple

OPERATORS: Tuple[str, ...] = (
    "arith",
    "relop",
    "eqop",
    "bitop",
    "cond_inv",
    "const",
    "index",
    "stuck",
)

# Keywords that open a declaration. A `[` inside a declaration and before its
# `=` is a width or array range, not a select, so `index` leaves it alone.
_DECL_KW = frozenset(
    (
        "input",
        "output",
        "inout",
        "wire",
        "reg",
        "logic",
        "tri",
        "signed",
        "unsigned",
        "integer",
        "genvar",
        "parameter",
        "localparam",
        "real",
        "time",
    )
)
# Of those, the ones that actually declare a signal whose width we care about.
_SIGNAL_KW = frozenset(("input", "output", "inout", "wire", "reg", "logic", "tri"))
# Statement boundaries: `seen_assign` and the declaration state reset here.
_STMT_KW = frozenset(
    ("begin", "end", "else", "always", "assign", "if", "case", "endcase", "initial")
)

# Longest first: the tokenizer tries these in order.
_PUNCT = (
    "<<<",
    ">>>",
    "===",
    "!==",
    "+:",
    "-:",
    "<<",
    ">>",
    "<=",
    ">=",
    "==",
    "!=",
    "&&",
    "||",
    "~&",
    "~|",
    "~^",
    "^~",
    "->",
    "+",
    "-",
    "*",
    "/",
    "%",
    "<",
    ">",
    "&",
    "|",
    "^",
    "~",
    "!",
    "?",
    ":",
    "=",
    ";",
    ",",
    "(",
    ")",
    "[",
    "]",
    "{",
    "}",
    ".",
    "#",
    "@",
    "$",
    "'",
)

_NUM_RE = re.compile(
    r"(?:\d[\d_]*)?'[sS]?[bBoOdDhH][0-9a-fA-FxXzZ?_]+"  # sized / based
    r"|\d[\d_]*"  # plain decimal
)
_IDENT_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_$]*")


# --------------------------------------------------------------------------
# Mutation
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Mutation:
    """One one-line change to one file."""

    path: str  # repo-relative, e.g. "src/loom_fifo.v"
    line: int  # 1-based
    col: int  # 1-based, first changed column
    operator: str
    description: str
    original: str  # the original line, no newline
    mutated: str  # the mutated line, no newline

    @property
    def module(self) -> str:
        """`loom_fifo` for `src/loom_fifo.v`."""
        name = self.path.replace("\\", "/").rsplit("/", 1)[-1]
        return name[:-2] if name.endswith(".v") else name

    @property
    def ident(self) -> str:
        """A short, stable, filesystem-safe, unique name for this mutant.

        The digest is part of it because one operator can fire twice at one
        column (a constant goes both up and down), and the runner uses the
        name as a directory.
        """
        return "%s_L%04dC%03d_%s_%s" % (
            self.module,
            self.line,
            self.col,
            self.operator,
            self.digest[:4],
        )

    @property
    def digest(self) -> str:
        """Identity of the *result*: two operators reaching the same text agree."""
        h = hashlib.sha1()
        h.update(("%s\n%d\n%s" % (self.path, self.line, self.mutated)).encode())
        return h.hexdigest()[:12]

    def apply(self, text: str) -> str:
        """Return `text` with this mutation applied. Raises if it does not fit."""
        lines = text.split("\n")
        idx = self.line - 1
        if idx >= len(lines) or lines[idx] != self.original:
            raise ValueError(
                "%s:%d does not hold the expected line (stale mutant?)"
                % (self.path, self.line)
            )
        lines[idx] = self.mutated
        return "\n".join(lines)

    def as_dict(self) -> Dict[str, object]:
        return {
            "id": self.ident,
            "path": self.path,
            "module": self.module,
            "line": self.line,
            "col": self.col,
            "operator": self.operator,
            "description": self.description,
            "original": self.original,
            "mutated": self.mutated,
            "digest": self.digest,
        }

    @staticmethod
    def from_dict(d: Dict[str, object]) -> "Mutation":
        return Mutation(
            path=str(d["path"]),
            line=int(d["line"]),
            col=int(d["col"]),
            operator=str(d["operator"]),
            description=str(d["description"]),
            original=str(d["original"]),
            mutated=str(d["mutated"]),
        )


# --------------------------------------------------------------------------
# Lexical pass: which columns are live code
# --------------------------------------------------------------------------


def code_mask(text: str, defines: Iterable[str] = ()) -> List[str]:
    """Blank out everything that is not live code, keeping every column.

    Comments, string literals, compiler-directive lines and text disabled by
    `ifdef`/`ifndef`/`else`/`elsif`/`endif` become spaces; the result has the
    same lines and the same column numbers as `text`.
    """
    defined: Set[str] = set(defines)
    lines = text.split("\n")
    out: List[str] = []
    in_block = False
    # Each frame: (enabled_here, any_branch_taken, parent_enabled)
    stack: List[List[object]] = []

    def live() -> bool:
        return all(bool(f[0]) for f in stack)

    for raw in lines:
        buf = [" "] * len(raw)
        stripped = raw.lstrip()
        if not in_block and stripped.startswith("`"):
            parts = stripped[1:].replace("(", " ").split()
            name = parts[0] if parts else ""
            arg = parts[1] if len(parts) > 1 else ""
            if name in ("ifdef", "ifndef"):
                hit = (arg in defined) if name == "ifdef" else (arg not in defined)
                parent = live()
                stack.append([parent and hit, hit, parent])
            elif name == "elsif" and stack:
                f = stack[-1]
                hit = arg in defined
                f[0] = bool(f[2]) and (not f[1]) and hit
                f[1] = bool(f[1]) or hit
            elif name == "else" and stack:
                f = stack[-1]
                f[0] = bool(f[2]) and not bool(f[1])
                f[1] = True
            elif name == "endif" and stack:
                stack.pop()
            # A directive line is never mutated.
            out.append("".join(buf))
            continue

        j = 0
        n = len(raw)
        while j < n:
            if in_block:
                if raw.startswith("*/", j):
                    in_block = False
                    j += 2
                    continue
                j += 1
                continue
            if raw.startswith("//", j):
                break
            if raw.startswith("/*", j):
                in_block = True
                j += 2
                continue
            ch = raw[j]
            if ch == '"':
                j += 1
                while j < n:
                    if raw[j] == "\\":
                        j += 2
                        continue
                    if raw[j] == '"':
                        j += 1
                        break
                    j += 1
                continue
            if live():
                buf[j] = ch
            j += 1
        out.append("".join(buf))
    return out


# --------------------------------------------------------------------------
# Token pass
# --------------------------------------------------------------------------


@dataclass
class _Tok:
    line: int  # 0-based
    col: int  # 0-based
    text: str
    kind: str  # "ident" | "num" | "punct"
    depth: int  # paren depth before this token
    bstack: Tuple[str, ...]  # bracket kinds enclosing this token
    seen_assign: bool  # an `=`/`<=` has already been seen in this statement
    in_decl: bool  # this statement started with a declaration keyword

    @property
    def end(self) -> int:
        return self.col + len(self.text)


def _tokenize(masked: Sequence[str]) -> List[_Tok]:
    """Tokenize masked code, carrying statement context across lines."""
    toks: List[_Tok] = []
    depth = 0
    bstack: List[str] = []
    seen_assign = False
    in_decl = False
    prev: Optional[_Tok] = None
    for li, line in enumerate(masked):
        j = 0
        n = len(line)
        while j < n:
            if line[j] == " ":
                j += 1
                continue
            m = _NUM_RE.match(line, j)
            if m and not (
                j and (line[j - 1].isalnum() or line[j - 1] == "_")
            ):  # not the tail of an identifier
                tok = _Tok(
                    li, j, m.group(0), "num", depth, tuple(bstack), seen_assign, in_decl
                )
                toks.append(tok)
                prev = tok
                j = m.end()
                continue
            m = _IDENT_RE.match(line, j)
            if m:
                word = m.group(0)
                if word in _DECL_KW:
                    if not seen_assign:
                        in_decl = True
                elif word in _STMT_KW:
                    seen_assign = False
                    in_decl = False
                tok = _Tok(
                    li, j, word, "ident", depth, tuple(bstack), seen_assign, in_decl
                )
                toks.append(tok)
                prev = tok
                j = m.end()
                continue
            text = None
            for p in _PUNCT:
                if line.startswith(p, j):
                    text = p
                    break
            if text is None:
                j += 1
                continue
            tok = _Tok(li, j, text, "punct", depth, tuple(bstack), seen_assign, in_decl)
            toks.append(tok)
            j += len(text)
            if text == "(":
                depth += 1
            elif text == ")":
                depth = max(0, depth - 1)
            elif text == "[":
                # A select is a `[` that binds to a name; a declaration range is
                # a `[` that appears in a declaration before its `=`.
                if in_decl and not seen_assign:
                    kind = "decl"
                elif prev is not None and (
                    prev.kind == "ident" or prev.text in (")", "]")
                ):
                    kind = "select"
                else:
                    kind = "other"
                bstack.append(kind)
            elif text == "]":
                if bstack:
                    bstack.pop()
            elif text in ("=", "<=") and depth == 0 and not seen_assign:
                seen_assign = True
            elif text == ";":
                seen_assign = False
                in_decl = False
            prev = tok
    return toks


def _is_nonblocking(tok: _Tok, prev: Optional[_Tok]) -> bool:
    """True if this `<=` is a nonblocking assignment rather than a comparison."""
    if tok.text != "<=":
        return False
    if tok.depth > 0 or tok.seen_assign:
        return False
    return prev is not None and (prev.kind == "ident" or prev.text in ("]", ")", "}"))


def one_bit_signals(masked: Sequence[str]) -> Tuple[Set[str], Set[str]]:
    """Names declared one bit wide, and names declared as vectors."""
    ones: Set[str] = set()
    vectors: Set[str] = set()
    has_range = False
    in_decl = False
    is_signal = False
    depth_at_start = 0
    for tok in _tokenize(masked):
        if tok.kind == "ident" and tok.text in _DECL_KW:
            if tok.text in _SIGNAL_KW:
                in_decl = True
                is_signal = True
                has_range = False
                depth_at_start = tok.depth
            elif tok.text in ("parameter", "localparam", "integer", "genvar"):
                in_decl = True
                is_signal = False
            continue
        if not in_decl:
            continue
        if tok.kind == "punct":
            if tok.text == "[" and not tok.seen_assign:
                has_range = True
            elif tok.text in (";", "="):
                in_decl = False
                is_signal = False
            elif tok.text == ")" and tok.depth <= depth_at_start:
                in_decl = False
                is_signal = False
            continue
        if tok.kind == "ident" and is_signal and not tok.seen_assign:
            if tok.text in _DECL_KW or tok.text in _STMT_KW:
                continue
            (ones if not has_range else vectors).add(tok.text)
    return ones, vectors


# --------------------------------------------------------------------------
# Literal arithmetic
# --------------------------------------------------------------------------

_SIZED_RE = re.compile(r"^(\d[\d_]*)?'([sS]?)([bBoOdDhH])([0-9a-fA-FxXzZ?_]+)$")
_BASE_RADIX = {"b": 2, "o": 8, "d": 10, "h": 16}


def bump_literal(text: str, delta: int) -> Optional[str]:
    """`4'd7` + 1 -> `4'd8`. Returns None when the literal cannot be bumped."""
    m = _SIZED_RE.match(text)
    if not m:
        if not text or not text[0].isdigit():
            return None
        try:
            value = int(text.replace("_", ""))
        except ValueError:
            return None
        if value + delta < 0:
            return None
        return str(value + delta)
    width_s, sign, base_c, digits = m.groups()
    base = _BASE_RADIX[base_c.lower()]
    clean = digits.replace("_", "")
    if any(c in "xXzZ?" for c in clean):
        return None
    try:
        value = int(clean, base)
    except ValueError:
        return None
    width = int(width_s.replace("_", "")) if width_s else None
    new = value + delta
    if width:
        new &= (1 << width) - 1
    elif new < 0:
        return None
    if base == 10:
        body = str(new)
    elif base == 2:
        body = format(new, "0%db" % (width or 1))
    elif base == 8:
        body = format(new, "0%do" % (((width or 3) + 2) // 3))
    else:
        digs = ((width or 4) + 3) // 4
        upper = any(c.isupper() for c in clean if c.isalpha())
        body = format(new, "0%dX" % digs) if upper else format(new, "0%dx" % digs)
    return "%s'%s%s%s" % (width_s or "", sign, base_c, body)


# --------------------------------------------------------------------------
# Mutation generation
# --------------------------------------------------------------------------

_SWAP_SIMPLE = {
    "arith": {"+": "-", "-": "+"},
    "eqop": {"==": "!=", "!=": "=="},
    "bitop": {"&": "|", "|": "&", "&&": "||", "||": "&&"},
}
_SWAP_REL = {"<": "<=", "<=": "<", ">": ">=", ">=": ">"}


def _replace(line: str, col: int, old: str, new: str) -> str:
    # A real check, not an assert: `python -O` would drop an assert, and a
    # mis-spliced mutant is a silently wrong result rather than a crash.
    if line[col : col + len(old)] != old:
        raise ValueError("token %r is not at column %d of %r" % (old, col, line))
    return line[:col] + new + line[col + len(old) :]


def mutations_for_text(
    text: str,
    path: str = "<text>",
    defines: Iterable[str] = (),
    operators: Optional[Sequence[str]] = None,
) -> List[Mutation]:
    """Every mutant of `text`, in file order. One change each, deduplicated."""
    wanted = set(operators or OPERATORS)
    unknown = wanted - set(OPERATORS)
    if unknown:
        raise ValueError("unknown operator(s): %s" % ", ".join(sorted(unknown)))
    raw_lines = text.split("\n")
    masked = code_mask(text, defines)
    toks = _tokenize(masked)
    ones, vectors = one_bit_signals(masked)
    out: List[Mutation] = []

    def emit(tok: _Tok, old: str, new: str, operator: str, why: str) -> None:
        line = raw_lines[tok.line]
        mutated = _replace(line, tok.col, old, new)
        if mutated == line:
            return
        out.append(
            Mutation(
                path=path,
                line=tok.line + 1,
                col=tok.col + 1,
                operator=operator,
                description=why,
                original=line,
                mutated=mutated,
            )
        )

    prev: Optional[_Tok] = None
    for i, tok in enumerate(toks):
        if tok.kind == "punct":
            t = tok.text
            if "arith" in wanted and t in _SWAP_SIMPLE["arith"]:
                emit(tok, t, _SWAP_SIMPLE["arith"][t], "arith", "`%s` -> `%s`" % (t, _SWAP_SIMPLE["arith"][t]))
            elif "eqop" in wanted and t in _SWAP_SIMPLE["eqop"]:
                emit(tok, t, _SWAP_SIMPLE["eqop"][t], "eqop", "`%s` -> `%s`" % (t, _SWAP_SIMPLE["eqop"][t]))
            elif "bitop" in wanted and t in _SWAP_SIMPLE["bitop"]:
                emit(tok, t, _SWAP_SIMPLE["bitop"][t], "bitop", "`%s` -> `%s`" % (t, _SWAP_SIMPLE["bitop"][t]))
            if "relop" in wanted and t in _SWAP_REL and not _is_nonblocking(tok, prev):
                emit(tok, t, _SWAP_REL[t], "relop", "`%s` -> `%s`" % (t, _SWAP_REL[t]))
        elif tok.kind == "num":
            inner = tok.bstack[-1] if tok.bstack else None
            # A declared width or array bound is not an index: changing it is a
            # different fault class (a resized register) and every one of them
            # dies in the linter, which would only flatter the kill rate.
            op = None if inner == "decl" else ("index" if inner == "select" else "const")
            if op in wanted:
                for delta, word in ((1, "+1"), (-1, "-1")):
                    new = bump_literal(tok.text, delta)
                    if new is not None and new != tok.text:
                        emit(
                            tok,
                            tok.text,
                            new,
                            op,
                            "%s `%s` %s -> `%s`"
                            % (
                                "index" if op == "index" else "constant",
                                tok.text,
                                word,
                                new,
                            ),
                        )
        elif tok.kind == "ident" and tok.text == "if":
            nxt = toks[i + 1] if i + 1 < len(toks) else None
            if nxt is not None and nxt.text == "(" and nxt.line == tok.line:
                close = _matching_paren(toks, i + 1)
                if close is not None and toks[close].line == tok.line:
                    open_tok, close_tok = nxt, toks[close]
                    line = raw_lines[tok.line]
                    inner_txt = line[open_tok.end : close_tok.col]
                    if "cond_inv" in wanted and inner_txt.strip():
                        mutated = (
                            line[: open_tok.end]
                            + "!("
                            + inner_txt
                            + ")"
                            + line[close_tok.col :]
                        )
                        out.append(
                            Mutation(
                                path=path,
                                line=tok.line + 1,
                                col=open_tok.end + 1,
                                operator="cond_inv",
                                description="invert the `if` condition `%s`"
                                % inner_txt.strip(),
                                original=line,
                                mutated=mutated,
                            )
                        )
                    if "stuck" in wanted:
                        out.extend(
                            _stuck_in_condition(
                                raw_lines, toks, i + 2, close, path, ones, vectors
                            )
                        )
        prev = tok

    return _dedupe(out)


def _matching_paren(toks: Sequence[_Tok], open_idx: int) -> Optional[int]:
    depth = 0
    for k in range(open_idx, len(toks)):
        if toks[k].text == "(":
            depth += 1
        elif toks[k].text == ")":
            depth -= 1
            if depth == 0:
                return k
    return None


def _stuck_in_condition(
    raw_lines: Sequence[str],
    toks: Sequence[_Tok],
    start: int,
    stop: int,
    path: str,
    ones: Set[str],
    vectors: Set[str],
) -> List[Mutation]:
    """Force one control bit of an `if` condition to a constant."""
    out: List[Mutation] = []
    if start >= stop:
        return out
    span = stop - start
    k = start
    while k < stop:
        tok = toks[k]
        if tok.kind != "ident" or tok.line != toks[start].line:
            k += 1
            continue
        name = tok.text
        end_col = tok.end
        if name in vectors:
            # A single-bit select of a vector is a control bit too.
            if (
                k + 3 < stop
                and toks[k + 1].text == "["
                and toks[k + 3].text == "]"
                and toks[k + 1].col == tok.end
            ):
                end_col = toks[k + 3].end
                k += 3
            else:
                k += 1
                continue
        elif name not in ones:
            k += 1
            continue
        if span == 1:
            # The condition is exactly this signal: `cond_inv` already covers
            # both constants between them, so do not duplicate the work.
            k += 1
            continue
        line = raw_lines[tok.line]
        expr = line[tok.col : end_col]
        for value in ("1'b0", "1'b1"):
            out.append(
                Mutation(
                    path=path,
                    line=tok.line + 1,
                    col=tok.col + 1,
                    operator="stuck",
                    description="stuck-at: `%s` -> `%s` in the `if` condition"
                    % (expr, value),
                    original=line,
                    mutated=line[: tok.col] + value + line[end_col:],
                )
            )
        k += 1
    return out


def _dedupe(muts: Iterable[Mutation]) -> List[Mutation]:
    """Drop mutants that produce text another mutant already produces."""
    seen: Set[str] = set()
    out: List[Mutation] = []
    for m in muts:
        if m.mutated == m.original:
            continue
        key = m.digest
        if key in seen:
            continue
        seen.add(key)
        out.append(m)
    out.sort(key=lambda m: (m.path, m.line, m.col, m.operator, m.description))
    return out


def mutations_for_file(
    path: str,
    rel: Optional[str] = None,
    defines: Iterable[str] = (),
    operators: Optional[Sequence[str]] = None,
) -> List[Mutation]:
    """Every mutant of one file on disk. `rel` is the name recorded in results."""
    with open(path, "r", encoding="utf-8", newline="") as fh:
        text = fh.read()
    return mutations_for_text(
        text.replace("\r\n", "\n"),
        path=rel or path,
        defines=defines,
        operators=operators,
    )


def iter_operator_counts(muts: Iterable[Mutation]) -> Dict[str, int]:
    counts: Dict[str, int] = {op: 0 for op in OPERATORS}
    for m in muts:
        counts[m.operator] = counts.get(m.operator, 0) + 1
    return counts
