"""Loom ISA loader and generator.

This package is the only code that parses ``isa/isa.yaml``. The assembler, the
golden model, the RTL header, the ISA reference page and the tests all go
through it, so an encoding exists in exactly one place.

Usage::

    from tools.loomisa import load
    isa = load()
    word = isa.encode("ADD", rd=1, ra=2, rb=3)
    instr, fields = isa.decode(word)

Command line (from the repository root)::

    python -m tools.loomisa check          # overlap and coverage report
    python -m tools.loomisa gen            # write src/loom_isa.vh and docs/ISA.md
    python -m tools.loomisa gen --check    # exit 1 if generated files are stale
"""

from __future__ import annotations

import dataclasses
import pathlib
import re
from typing import Dict, List, Optional, Tuple

import yaml

REPO = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_YAML = REPO / "isa" / "isa.yaml"
VH_PATH = REPO / "src" / "loom_isa.vh"
MD_PATH = REPO / "docs" / "ISA.md"

WORD_BITS = 16

# Operand base name (trailing digits stripped) -> field letter in the format.
OPERAND_LETTER = {
    "rd": "d", "ra": "a", "rb": "b", "imm": "i", "rel": "r", "abs": "A",
    "pin": "p", "val": "v", "edge": "e", "flag": "n", "tmo": "T", "cond": "c",
    "csr": "C",
}
SIGNED_LETTERS = {"r"}          # PC-relative offsets are two's complement
OPTIONAL_OPERANDS = {"tmo": 0}  # may be omitted when encoding; default value


class IsaError(ValueError):
    """Raised for malformed ISA descriptions and out-of-range operands."""


def operand_base(name: str) -> str:
    return re.sub(r"\d+$", "", name)


@dataclasses.dataclass(frozen=True)
class Field:
    letter: str
    hi: int
    lo: int

    @property
    def width(self) -> int:
        return self.hi - self.lo + 1

    @property
    def mask(self) -> int:
        return ((1 << self.width) - 1) << self.lo


@dataclasses.dataclass(frozen=True)
class Instr:
    name: str
    fmt: str
    mask: int                    # bits the decoder must compare
    match: int                   # value of those bits
    fields: Dict[str, Field]     # by letter, fixed fields included
    fixed: Dict[str, int]        # letter -> value
    ops: Tuple[str, ...]         # operand names as written in isa.yaml
    sem: str
    flags: Tuple[str, ...]
    timing: str
    cls: str
    optional: Optional[str]

    def field_for(self, operand: str) -> Field:
        letter = OPERAND_LETTER.get(operand_base(operand))
        if letter is None or letter not in self.fields:
            raise IsaError(f"{self.name}: no field for operand '{operand}'")
        return self.fields[letter]

    def encode(self, **operands: int) -> int:
        """Encode with operands given by name (``rd``, ``imm``, ``rel`` ...).

        Operand names may be given with or without their width suffix
        (``imm`` or ``imm8``). Optional operands default to 0.
        """
        given = {operand_base(k): v for k, v in operands.items()}
        word = self.match
        seen = set()
        for op in self.ops:
            base = operand_base(op)
            if base in given:
                value = given[base]
            elif base in OPTIONAL_OPERANDS:
                value = OPTIONAL_OPERANDS[base]
            else:
                raise IsaError(f"{self.name}: missing operand '{op}'")
            seen.add(base)
            fld = self.field_for(op)
            if fld.letter in SIGNED_LETTERS:
                lo, hi = -(1 << (fld.width - 1)), (1 << (fld.width - 1)) - 1
            else:
                lo, hi = 0, (1 << fld.width) - 1
            if not isinstance(value, int) or not lo <= value <= hi:
                raise IsaError(
                    f"{self.name}: operand {op}={value!r} outside {lo}..{hi}")
            word |= (value & ((1 << fld.width) - 1)) << fld.lo
        extra = set(given) - seen
        if extra:
            raise IsaError(f"{self.name}: unexpected operands {sorted(extra)}")
        return word

    def decode_fields(self, word: int) -> Dict[str, int]:
        """Operand values by base name; signed fields are sign-extended."""
        out = {}
        for op in self.ops:
            fld = self.field_for(op)
            value = (word >> fld.lo) & ((1 << fld.width) - 1)
            if fld.letter in SIGNED_LETTERS and value >> (fld.width - 1):
                value -= 1 << fld.width
            out[operand_base(op)] = value
        return out

    def matches(self, word: int) -> bool:
        return (word & self.mask) == self.match


def _parse_format(name: str, fmt: str, fixed: Dict[str, int]):
    bits = re.sub(r"[ _]", "", fmt)
    if len(bits) != WORD_BITS:
        raise IsaError(f"{name}: format '{fmt}' is {len(bits)} bits, not {WORD_BITS}")
    mask = match = 0
    runs: Dict[str, List[int]] = {}
    for i, ch in enumerate(bits):
        pos = WORD_BITS - 1 - i
        if ch in "01":
            mask |= 1 << pos
            match |= int(ch) << pos
        elif ch == "-":
            continue
        elif ch.isalpha():
            runs.setdefault(ch, []).append(pos)
        else:
            raise IsaError(f"{name}: bad character '{ch}' in format")
    fields = {}
    for letter, positions in runs.items():
        hi, lo = max(positions), min(positions)
        if positions != list(range(hi, lo - 1, -1)):
            raise IsaError(f"{name}: field '{letter}' is not contiguous")
        fields[letter] = Field(letter, hi, lo)
    for letter, value in fixed.items():
        if letter not in fields:
            raise IsaError(f"{name}: fixed field '{letter}' not in format")
        fld = fields[letter]
        if not 0 <= value < (1 << fld.width):
            raise IsaError(f"{name}: fixed {letter}={value} does not fit")
        mask |= fld.mask
        match |= value << fld.lo
    return mask, match, fields


class Isa:
    def __init__(self, raw: dict, source: pathlib.Path):
        self.raw = raw
        self.source = source
        self.meta = raw["meta"]
        self.version = str(self.meta["version"])
        self.csrs: Dict[int, dict] = {int(k): v for k, v in raw["csrs"].items()}
        self.pins: Dict[int, dict] = {int(k): v for k, v in raw["pins"].items()}
        self.crc_presets = raw.get("crc_presets", {})
        self.pseudo_ops = raw.get("pseudo_ops", [])
        # Symbolic operand values by operand base name, e.g.
        # enums["edge"] == {"RISE": 0, "FALL": 1, "ANY": 2}
        self.enums: Dict[str, Dict[str, int]] = {
            str(k): {str(n): int(v) for n, v in table.items()}
            for k, table in raw.get("enums", {}).items()}
        self.instructions: List[Instr] = []
        self.by_name: Dict[str, Instr] = {}
        for entry in raw["instructions"]:
            name = entry["name"]
            fixed = dict(entry.get("fixed", {}))
            mask, match, fields = _parse_format(name, entry["fmt"], fixed)
            instr = Instr(
                name=name, fmt=re.sub(r"[ _]", "", entry["fmt"]), mask=mask,
                match=match, fields=fields, fixed=fixed,
                ops=tuple(entry.get("ops", [])), sem=entry.get("sem", ""),
                flags=tuple(entry.get("flags", [])),
                timing=entry.get("timing", "one_slot"),
                cls=entry.get("class", ""), optional=entry.get("optional"))
            for op in instr.ops:
                instr.field_for(op)          # validates operand names
            if name in self.by_name:
                raise IsaError(f"duplicate instruction {name}")
            self.instructions.append(instr)
            self.by_name[name] = instr
        self.csr_by_name = {v["name"]: k for k, v in self.csrs.items()}
        self.pin_by_name = {v["name"]: k for k, v in self.pins.items()}

    # ---------------------------------------------------------------- coding
    def encode(self, name: str, **operands: int) -> int:
        try:
            instr = self.by_name[name.upper()]
        except KeyError:
            raise IsaError(f"unknown instruction '{name}'") from None
        return instr.encode(**operands)

    def decode(self, word: int) -> Optional[Tuple[Instr, Dict[str, int]]]:
        """Return (instruction, operands) or None for a reserved word."""
        word &= (1 << WORD_BITS) - 1
        for instr in self.instructions:
            if instr.matches(word):
                return instr, instr.decode_fields(word)
        return None

    # ---------------------------------------------------------------- checks
    def overlaps(self) -> List[Tuple[str, str]]:
        bad = []
        ins = self.instructions
        for i, a in enumerate(ins):
            for b in ins[i + 1:]:
                if ((a.match ^ b.match) & (a.mask & b.mask)) == 0:
                    bad.append((a.name, b.name))
        return bad

    def coverage(self) -> Tuple[int, int]:
        """(words that decode to an instruction, total words)."""
        used = sum(1 for w in range(1 << WORD_BITS) if self.decode(w) is not None)
        return used, 1 << WORD_BITS

    def check(self) -> List[str]:
        problems = [f"encodings overlap: {a} and {b}" for a, b in self.overlaps()]
        for number, csr in self.csrs.items():
            if not 0 <= number < 32:
                problems.append(f"CSR {csr['name']} number {number} does not fit 5 bits")
        for number in self.pins:
            if not 0 <= number < 32:
                problems.append(f"pin index {number} does not fit 5 bits")
        return problems


def load(path: Optional[pathlib.Path] = None) -> Isa:
    source = pathlib.Path(path) if path else DEFAULT_YAML
    with open(source, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    return Isa(raw, source)


# ------------------------------------------------------------------ generators
_HEADER = "GENERATED by tools/loomisa from isa/isa.yaml. DO NOT EDIT."


def gen_vh(isa: Isa) -> str:
    lines = [
        f"// {_HEADER}",
        f"// ISA version {isa.version}. Include inside a module body.",
        "// Decode with: (ir & ISA_<NAME>_MASK) == ISA_<NAME>_MATCH",
        "/* verilator lint_off UNUSEDPARAM */",
        "",
        f"localparam integer ISA_THREADS  = {isa.meta['threads']};",
        f"localparam integer ISA_PC_BITS  = {isa.meta['pc_bits']};",
        f"localparam integer ISA_REG_BITS = {isa.meta['register_bits']};",
        "",
    ]
    for instr in isa.instructions:
        lines.append(f"// {instr.name}: {instr.fmt}  ({instr.timing})")
        lines.append(f"localparam [15:0] ISA_{instr.name}_MASK  = 16'h{instr.mask:04X};")
        lines.append(f"localparam [15:0] ISA_{instr.name}_MATCH = 16'h{instr.match:04X};")
        for op in instr.ops:
            fld = instr.field_for(op)
            base = operand_base(op).upper()
            lines.append(f"localparam integer ISA_{instr.name}_{base}_HI = {fld.hi};")
            lines.append(f"localparam integer ISA_{instr.name}_{base}_LO = {fld.lo};")
        lines.append("")
    lines.append("// CSR numbers")
    for number in sorted(isa.csrs):
        lines.append(f"localparam [4:0] CSR_{isa.csrs[number]['name']} = 5'h{number:02X};")
    lines.append("")
    lines.append("// Pin indices")
    for number in sorted(isa.pins):
        lines.append(f"localparam [4:0] PIN_{isa.pins[number]['name']} = 5'd{number};")
    lines += ["", "/* verilator lint_on UNUSEDPARAM */", ""]
    return "\n".join(lines)


def _pretty_fmt(fmt: str) -> str:
    return " ".join(fmt[i:i + 4] for i in range(0, WORD_BITS, 4))


def gen_md(isa: Isa) -> str:
    out = [
        f"<!-- {_HEADER} -->",
        "",
        f"# Loom instruction set reference (ISA {isa.version})",
        "",
        "Generated from `isa/isa.yaml`. Encoding strings are MSB first; digits are",
        "fixed bits, letters are fields, `-` is ignored by the decoder. Cycle-exact",
        "behaviour is defined in `docs/SEMANTICS.md`.",
        "",
        "## Instructions",
        "",
        "| Mnemonic | Operands | Encoding | Mask / match | Flags | Timing | Semantics |",
        "|---|---|---|---|---|---|---|",
    ]
    for instr in isa.instructions:
        note = f" (needs {instr.optional})" if instr.optional else ""
        sem = instr.sem.replace("|", "\\|")
        out.append(
            f"| `{instr.name}` | {', '.join(instr.ops) or '-'} | `{_pretty_fmt(instr.fmt)}` | "
            f"`{instr.mask:04X}` / `{instr.match:04X}` | {' '.join(instr.flags) or '-'} | "
            f"{instr.timing} | {sem}{note} |")
    used, total = isa.coverage()
    out += [
        "",
        f"{len(isa.instructions)} instructions. {used} of {total} words decode to an",
        "instruction; every other word is reserved and executes as `NOP` with `BADOP` set.",
        "",
        "## CSRs",
        "",
        "| Number | Name | Access | Bits | Meaning |",
        "|---|---|---|---|---|",
    ]
    for number in sorted(isa.csrs):
        csr = isa.csrs[number]
        out.append(f"| 0x{number:02X} | `{csr['name']}` | {csr['rw']} | {csr['bits']} | {csr['doc']} |")
    out += ["", "## Pin indices", "", "| Index | Name | Direction | Pad |", "|---|---|---|---|"]
    for number in sorted(isa.pins):
        pin = isa.pins[number]
        out.append(f"| {number} | `{pin['name']}` | {pin['dir']} | {pin['pad']} |")
    if isa.enums:
        out += ["", "## Symbolic operand values", "", "| Operand | Name | Value |", "|---|---|---|"]
        for operand, table in isa.enums.items():
            for name, value in table.items():
                out.append(f"| {operand} | `{name}` | {value} |")
    if isa.pseudo_ops:
        out += ["", "## Pseudo-ops (assembler)", "", "| Name | Expands to | Note |", "|---|---|---|"]
        for pseudo in isa.pseudo_ops:
            out.append(f"| `{pseudo['name']}` | `{pseudo['expands']}` | {pseudo.get('doc', '')} |")
        out += ["", "Directives and the full assembly language are defined in",
                "`tools/loomasm/README.md`."]
    out += ["", "## CRC presets", "", "| Name | Polynomial | Init | Width | Note |", "|---|---|---|---|---|"]
    for name, preset in isa.crc_presets.items():
        out.append(f"| `{name}` | 0x{preset['poly']:04X} | 0x{preset['init']:04X} | "
                   f"{preset['width']} | {preset['doc']} |")
    out.append("")
    return "\n".join(out)


# Normalised operand outputs of the generated decoder: base name -> width.
# "rel" is sign-extended to the PC width, "imm" is zero-extended to 16 bits.
_DECODE_OUTPUTS = [
    ("rd", 3), ("ra", 3), ("rb", 3), ("imm", 16), ("rel", None), ("abs", None),
    ("pin", 5), ("val", 1), ("edge", 2), ("flag", 3), ("tmo", 1), ("cond", 2),
    ("csr", 5),
]


def gen_decode_v(isa: Isa) -> str:
    """The instruction decoder as a synthesisable module, generated so that
    hand-written RTL never contains an encoding."""
    pc_bits = int(isa.meta["pc_bits"])
    widths = {name: (pc_bits if w is None else w) for name, w in _DECODE_OUTPUTS}
    classes = sorted({i.cls for i in isa.instructions if i.cls})
    lines = [
        f"// {_HEADER}",
        f"// ISA version {isa.version}. Purely combinational.",
        "//",
        "// is_<name>    one-hot instruction strobes; all 0 for a reserved word",
        "// is_reserved  no instruction matches (executes as NOP, sets BADOP)",
        "// grp_<class>  OR of the strobes in a class of isa.yaml",
        "// tmg_*        timing class of the matched instruction",
        "// f_<operand>  operand fields moved to a fixed place: imm zero-extended,",
        "//              rel sign-extended to the PC width, 0 when not present",
        "// f_funct      bits of the 'f' sub-operation field (ALU classes)",
        "// csr_<name>   f_csr selects that CSR (qualified by CSRR or CSRW)",
        "",
        "`default_nettype none",
        "",
        "module loom_decode (",
        "    input  wire [15:0] ir,",
    ]
    for instr in isa.instructions:
        lines.append(f"    output wire        is_{instr.name.lower()},")
    lines.append("    output wire        is_reserved,")
    for cls in classes:
        lines.append(f"    output wire        grp_{cls},")
    lines.append("    output wire        tmg_wait,")
    lines.append("    output wire        tmg_blocking,")
    lines.append("    output wire [2:0]  f_funct,")
    for name, _ in _DECODE_OUTPUTS:
        w = widths[name]
        rng = f"[{w - 1}:0]" if w > 1 else "      "
        lines.append(f"    output wire {rng:<6} f_{name},")
    csr_numbers = sorted(isa.csrs)
    for idx, number in enumerate(csr_numbers):
        comma = "," if idx < len(csr_numbers) - 1 else ""
        lines.append(f"    output wire        csr_{isa.csrs[number]['name'].lower()}{comma}")
    lines.append(");")
    lines.append("")

    for instr in isa.instructions:
        lines.append(f"  assign is_{instr.name.lower():<6} = (ir & 16'h{instr.mask:04X}) == 16'h{instr.match:04X};"
                     f"  // {instr.fmt}")
    lines.append("")
    all_is = " | ".join(f"is_{i.name.lower()}" for i in isa.instructions)
    lines.append(f"  assign is_reserved = ~({all_is});")
    lines.append("")
    for cls in classes:
        members = " | ".join(f"is_{i.name.lower()}" for i in isa.instructions if i.cls == cls)
        lines.append(f"  assign grp_{cls} = {members};")
    for timing in ("wait", "blocking"):
        members = " | ".join(f"is_{i.name.lower()}" for i in isa.instructions if i.timing == timing)
        expr = members if members else "1'b0"
        lines.append(f"  assign tmg_{timing} = {expr};")
    lines.append("")

    # sub-operation field
    f_positions = {(i.fields["f"].hi, i.fields["f"].lo) for i in isa.instructions if "f" in i.fields}
    if len(f_positions) != 1:
        raise IsaError("field 'f' must sit at one position in every format that has it")
    (f_hi, f_lo), = f_positions
    f_users = " | ".join(f"is_{i.name.lower()}" for i in isa.instructions if "f" in i.fields)
    lines.append(f"  assign f_funct = ({f_users}) ? ir[{f_hi}:{f_lo}] : 3'b000;")
    lines.append("")

    # normalised operands, grouped by bit position
    for name, _ in _DECODE_OUTPUTS:
        width = widths[name]
        by_pos: Dict[Tuple[int, int], List[str]] = {}
        for instr in isa.instructions:
            for op in instr.ops:
                if operand_base(op) == name:
                    fld = instr.field_for(op)
                    by_pos.setdefault((fld.hi, fld.lo), []).append(f"is_{instr.name.lower()}")
        if not by_pos:
            lines.append(f"  assign f_{name} = {width}'d0;")
            continue
        terms = []
        for (hi, lo), users in sorted(by_pos.items(), reverse=True):
            fw = hi - lo + 1
            if fw > width:
                raise IsaError(f"operand {name} is {fw} bits wide, decoder output is {width}")
            sel = " | ".join(users)
            src = f"ir[{hi}:{lo}]" if fw > 1 else f"ir[{hi}]"
            if fw < width:
                pad = width - fw
                fill = f"{{{pad}{{ir[{hi}]}}}}" if name == "rel" else f"{pad}'d0"
                src = f"{{{fill}, {src}}}"
            zero = f"{width}'d0"
            terms.append(f"(({sel}) ? {src} : {zero})")
        joiner = "\n" + " " * (len(f"  assign f_{name} = ")) + "| "
        lines.append(f"  assign f_{name} = {joiner.join(terms)};")
    lines.append("")

    csr_users = " | ".join(f"is_{i.name.lower()}" for i in isa.instructions
                           if any(operand_base(op) == "csr" for op in i.ops))
    lines.append(f"  wire csr_access = {csr_users};")
    for number in csr_numbers:
        lines.append(f"  assign csr_{isa.csrs[number]['name'].lower()} = csr_access & (f_csr == 5'h{number:02X});")
    lines += ["", "endmodule", ""]
    return "\n".join(lines)


DECODE_PATH = REPO / "src" / "loom_decode.v"


def generated_files(isa: Isa) -> Dict[pathlib.Path, str]:
    return {VH_PATH: gen_vh(isa), DECODE_PATH: gen_decode_v(isa), MD_PATH: gen_md(isa)}
