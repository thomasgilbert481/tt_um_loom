"""Assembler tests: operands, labels, directives, errors, image, listing."""

import json
import pathlib
import re

import pytest

from tools.loomasm import AsmError, assemble, assemble_text, disassemble
from tools.loomisa import load, operand_base

ISA = load()


def src(*lines):
    """Build a source file whose line numbers are the list index plus one."""
    return "\n".join(lines) + "\n"


def asm(text, **kwargs):
    return assemble_text(text, "t.loom", isa=ISA, **kwargs)


def words_of(text):
    """Assemble and return the emitted words in address order."""
    program = asm(text)
    return [program.words[a] for a in sorted(program.words)]


def errors_of(text):
    with pytest.raises(AsmError) as info:
        asm(text)
    return info.value.diagnostics


# --------------------------------------------------------------- operands
def test_register_operands():
    assert words_of(src(".thread 0", "ADD r1, r2, r3")) == [
        ISA.encode("ADD", rd=1, ra=2, rb=3)]
    assert words_of(src(".thread 0", "add R1, r2, R3")) == [
        ISA.encode("ADD", rd=1, ra=2, rb=3)]


def test_immediate_operands_decimal_hex_binary_and_char():
    assert words_of(src(".thread 0",
                        "LDI r0, 65",
                        "LDI r1, 0x41",
                        "LDI r2, 0b0100_0001",
                        "LDI r3, 'A'")) == [
        ISA.encode("LDI", rd=r, imm=0x41) for r in range(4)]


def test_immediate_expression_with_symbols():
    assert words_of(src(".thread 0",
                        ".equ BASE = 0x40",
                        "LDI r0, BASE + 1",
                        "LDI r1, lo8(0x1234)",
                        "LDI r2, hi8(0x1234)")) == [
        ISA.encode("LDI", rd=0, imm=0x41),
        ISA.encode("LDI", rd=1, imm=0x34),
        ISA.encode("LDI", rd=2, imm=0x12)]


def test_pin_operands_by_isa_name_alias_and_number():
    program = asm(src(".thread 0",
                      ".pins TX = OUT0, SDA = BIDIR2",
                      "SETP OUT1, 1",
                      "SETP TX, 0",
                      "SETP sda, 1",
                      "SETP 21, 1"))
    assert [program.words[a] for a in sorted(program.words)] == [
        ISA.encode("SETP", pin=17, val=1),
        ISA.encode("SETP", pin=16, val=0),
        ISA.encode("SETP", pin=2, val=1),
        ISA.encode("SETP", pin=21, val=1)]


def test_edge_operands():
    assert words_of(src(".thread 0",
                        "WAITE IN0, RISE",
                        "WAITE IN0, fall",
                        "WAITE IN0, ANY",
                        "WAITE IN0, 1")) == [
        ISA.encode("WAITE", pin=8, edge=e, tmo=0) for e in (0, 1, 2, 1)]


def test_cond_operands_for_waitb():
    assert words_of(src(".thread 0",
                        "WAITB BE_IDLE",
                        "WAITB OUTQ_NF",
                        "WAITB inq_ne",
                        "WAITB TICK",
                        "WAITB 3")) == [
        ISA.encode("WAITB", cond=c, tmo=0) for c in (0, 1, 2, 3, 3)]


def test_flag_and_val_operands():
    assert words_of(src(".thread 0", "SIG 5", "CLR 0", "SETP OUT0, 1")) == [
        ISA.encode("SIG", flag=5), ISA.encode("CLR", flag=0),
        ISA.encode("SETP", pin=16, val=1)]


def test_csr_operands_by_name_and_number():
    assert words_of(src(".thread 0",
                        "CSRR r3, NOW",
                        "CSRR r3, now",
                        "CSRR r3, 9",
                        "CSRW TD, r1")) == [
        ISA.encode("CSRR", rd=3, csr=9), ISA.encode("CSRR", rd=3, csr=9),
        ISA.encode("CSRR", rd=3, csr=9), ISA.encode("CSRW", csr=0x0A, ra=1)]


def test_timeout_operand_is_the_bare_token_t():
    assert words_of(src(".thread 0",
                        "WAITP IN0, 1, T",
                        "WAITP IN0, 1",
                        "WAITS 3, t",
                        "WAITS 3",
                        "WAITE IN0, FALL, T",
                        "WAITB INQ_NE, T")) == [
        ISA.encode("WAITP", pin=8, val=1, tmo=1),
        ISA.encode("WAITP", pin=8, val=1, tmo=0),
        ISA.encode("WAITS", flag=3, tmo=1),
        ISA.encode("WAITS", flag=3, tmo=0),
        ISA.encode("WAITE", pin=8, edge=1, tmo=1),
        ISA.encode("WAITB", cond=2, tmo=1)]


def test_abs_operand_takes_a_label_or_a_number():
    program = asm(src(".thread 0", "here: JMP here", "CALL 0x40"))
    assert program.words[0] == ISA.encode("JMP", abs=0)
    assert program.words[1] == ISA.encode("CALL", abs=0x40)


def test_three_operand_and_memory_forms():
    assert words_of(src(".thread 0", "LD r1, r2, 5", "ST r3, r4, 0")) == [
        ISA.encode("LD", rd=1, ra=2, imm=5),
        ISA.encode("ST", rd=3, ra=4, imm=0)]


# ----------------------------------------------------------------- labels
def test_backward_and_forward_label_references():
    program = asm(src(".thread 0",
                      "back:  NOP",
                      "       BZ back",
                      "       BNZ fwd",
                      "       NOP",
                      "fwd:   NOP"))
    assert program.symbols["back"] == 0
    assert program.symbols["fwd"] == 4
    assert program.words[1] == ISA.encode("BZ", rel=-2)      # 0 - (1 + 1)
    assert program.words[2] == ISA.encode("BNZ", rel=1)      # 4 - (2 + 1)


def test_label_alone_on_its_line_and_several_labels_on_one_line():
    program = asm(src(".thread 0", "one:", "two: three: NOP"))
    assert program.symbols == {"one": 0, "two": 0, "three": 0}


def test_labels_are_case_sensitive():
    program = asm(src(".thread 0", "Loop: NOP", "loop: NOP", "JMP Loop"))
    assert program.symbols["Loop"] == 0 and program.symbols["loop"] == 1


def test_numeric_rel_operand_is_a_raw_offset():
    assert words_of(src(".thread 0", "NOP", "BZ -2", "BZ 0")) == [
        ISA.encode("NOP"), ISA.encode("BZ", rel=-2), ISA.encode("BZ", rel=0)]


def test_rel_out_of_range_names_the_field():
    diagnostics = errors_of(src(".thread 0", "BZ far", ".org 0x180", "far: NOP"))
    assert len(diagnostics) == 1
    assert diagnostics[0].line == 2
    assert "cannot reach" in diagnostics[0].message
    assert "rel8 holds -128..127" in diagnostics[0].message


def test_raw_rel_offset_out_of_range():
    diagnostics = errors_of(src(".thread 0", "JP IN0, 1, 40"))
    assert "does not fit rel6" in diagnostics[0].message


def test_redefining_a_symbol_is_an_error():
    diagnostics = errors_of(src(".thread 0", "dup: NOP", "dup: NOP"))
    assert "already defined" in diagnostics[0].message


def test_a_register_name_cannot_be_a_label():
    diagnostics = errors_of(src(".thread 0", "r0: NOP"))
    assert "register name" in diagnostics[0].message


# ------------------------------------------------------------- directives
def test_thread_sections_start_at_the_reset_vector():
    program = asm(src(".thread 0", "NOP",
                      ".thread 2", "NOP",
                      ".thread 0", "NOP"))
    assert sorted(program.words) == [0, 1, 0x200]
    assert program.threads[0].entry == 0 and program.threads[0].size == 2
    assert program.threads[2].entry == 0x200 and program.threads[2].size == 1


def test_org_moves_the_current_thread_only():
    program = asm(src(".thread 1", ".org 0x120", "here: NOP"))
    assert program.symbols["here"] == 0x120
    assert program.threads[1].entry == 0x120


def test_overlapping_thread_sections_are_an_error():
    diagnostics = errors_of(src(".thread 0", "NOP",
                                ".thread 1", ".org 0", "NOP"))
    assert diagnostics[0].line == 5
    assert "already used by thread 0" in diagnostics[0].message
    assert "overlap" in diagnostics[0].message


def test_org_past_the_end_of_memory_is_an_error():
    diagnostics = errors_of(src(".thread 0", ".org 0x400"))
    assert "outside instruction memory" in diagnostics[0].message


def test_unknown_thread_number_is_an_error():
    assert "does not exist" in errors_of(src(".thread 4"))[0].message


def test_equ_defines_a_constant_and_appears_in_the_symbols():
    program = asm(src(".thread 0", ".equ EIGHT = 4 + 4", "LDI r1, EIGHT"))
    assert program.symbols["EIGHT"] == 8
    assert program.words[0] == ISA.encode("LDI", rd=1, imm=8)


def test_equ_cannot_use_a_forward_reference():
    diagnostics = errors_of(src(".thread 0", ".equ X = later", "later: NOP"))
    assert "unknown symbol 'later'" in diagnostics[0].message


def test_pins_rejects_redefining_an_isa_pin_name():
    diagnostics = errors_of(src(".thread 0", ".pins OUT0 = BIDIR1"))
    assert "pin name in isa.yaml" in diagnostics[0].message


def test_pins_alias_may_refer_to_an_earlier_alias():
    program = asm(src(".thread 0", ".pins A = OUT3", ".pins B = A", "SETP B, 1"))
    assert program.words[0] == ISA.encode("SETP", pin=19, val=1)


def test_word_directive_emits_raw_words():
    program = asm(src(".thread 0", ".word 0x1234, 'A', -1, 0"))
    assert [program.words[a] for a in sorted(program.words)] == [
        0x1234, 0x41, 0xFFFF, 0]


def test_word_out_of_range_is_an_error():
    assert "does not fit in 16 bits" in errors_of(
        src(".thread 0", ".word 0x10000"))[0].message


def test_tick_directive_sets_the_period_without_emitting_a_word():
    program = asm(src(".thread 0", ".tick 100", "SETD 0", "WAITD 1"))
    assert len(program.words) == 2
    assert program.deadlines[0].period == 100


def test_unknown_directive_is_an_error():
    assert "unknown directive" in errors_of(
        src(".thread 0", ".nope 1"))[0].message


# --------------------------------------------------------- .csr and MOV16
def test_csr_directive_expands_to_mov16_then_csrw():
    program = asm(src(".thread 0", ".csr TICK_INT, 434"))
    assert [program.words[a] for a in sorted(program.words)] == [
        ISA.encode("LDI", rd=7, imm=434 & 0xFF),
        ISA.encode("LDIH", rd=7, imm=434 >> 8),
        ISA.encode("CSRW", csr=0x00, ra=7)]


def test_csr_directive_short_form_and_explicit_scratch_register():
    program = asm(src(".thread 0", ".csr TICK_FRAC, 0, r6"))
    assert [program.words[a] for a in sorted(program.words)] == [
        ISA.encode("LDI", rd=6, imm=0),
        ISA.encode("CSRW", csr=0x01, ra=6)]


def test_csr_directive_accepts_a_csr_number():
    program = asm(src(".thread 0", ".csr 0x0A, 1"))
    assert program.words[1] == ISA.encode("CSRW", csr=0x0A, ra=7)


def test_mov16_short_form_when_the_value_fits_in_eight_bits():
    program = asm(src(".thread 0", "MOV16 r1, 0x12", "NOP"))
    assert program.words[0] == ISA.encode("LDI", rd=1, imm=0x12)
    assert program.words[1] == ISA.encode("NOP")


def test_mov16_long_form():
    program = asm(src(".thread 0", "MOV16 r1, 0x1234", "NOP"))
    assert [program.words[a] for a in sorted(program.words)] == [
        ISA.encode("LDI", rd=1, imm=0x34),
        ISA.encode("LDIH", rd=1, imm=0x12),
        ISA.encode("NOP")]


def test_mov16_uses_the_long_form_for_a_forward_reference():
    program = asm(src(".thread 0", "MOV16 r1, later", "later: NOP"))
    assert program.symbols["later"] == 2
    assert [program.words[a] for a in sorted(program.words)] == [
        ISA.encode("LDI", rd=1, imm=2),
        ISA.encode("LDIH", rd=1, imm=0),
        ISA.encode("NOP")]


def test_other_pseudo_ops():
    assert words_of(src(".thread 0",
                        "target: BRA target", "INC r2", "DEC r3")) == [
        ISA.encode("JMP", abs=0),
        ISA.encode("ADDI", rd=2, imm=1),
        ISA.encode("SUBI", rd=3, imm=1)]


# ----------------------------------------------------------- diagnostics
def test_every_error_in_a_file_is_reported_with_its_line():
    diagnostics = errors_of(src(
        ".thread 0",                 # 1
        "        LDI r0, 300",       # 2  immediate out of range
        "        FOO r1",            # 3  unknown instruction
        "        BZ nowhere",        # 4  unknown symbol
        "        SETP r9, 1",        # 5  not a pin and not a symbol
        "        ADD r1, r2",        # 6  wrong operand count
        "        NOP @",             # 7  lex error
    ))
    assert [d.line for d in diagnostics] == [2, 3, 4, 5, 6, 7]
    assert "outside 0..255" in diagnostics[0].message
    assert "unknown instruction 'FOO'" in diagnostics[1].message
    assert "unknown symbol 'nowhere'" in diagnostics[2].message
    assert "unknown symbol 'r9'" in diagnostics[3].message
    assert "takes 3 operands" in diagnostics[4].message
    assert "unexpected character" in diagnostics[5].message
    assert all(d.file == "t.loom" for d in diagnostics)


def test_diagnostic_string_carries_file_and_line():
    diagnostics = errors_of(src(".thread 0", "FOO"))
    assert str(diagnostics[0]).startswith("t.loom:2:")
    assert "error" in str(diagnostics[0])


def test_asmerror_message_counts_the_other_errors():
    with pytest.raises(AsmError) as info:
        asm(src(".thread 0", "FOO", "BAR", "BAZ"))
    assert "+2 more errors" in str(info.value)
    assert len(info.value.errors) == 3


def test_not_a_register_is_reported_clearly():
    assert "is not a register" in errors_of(
        src(".thread 0", "ADD r8, r1, r2"))[0].message


def test_unknown_edge_name_lists_the_choices():
    message = errors_of(src(".thread 0", "WAITE IN0, SIDEWAYS"))[0].message
    assert "unknown edge 'SIDEWAYS'" in message
    assert "ANY, FALL, RISE" in message


def test_unknown_csr_name():
    assert "unknown CSR 'NOPE'" in errors_of(
        src(".thread 0", "CSRR r1, NOPE"))[0].message


# --------------------------------------------------------- image, listing
def test_json_image_shape():
    program = asm(src(".thread 0", "start: NOP", ".thread 1", "NOP"))
    image = program.to_image()
    assert set(image) == {"isa", "words", "symbols", "threads", "source"}
    assert image["isa"] == ISA.version
    assert image["source"] == "t.loom"
    assert image["words"] == {"0": ISA.encode("NOP"), "256": ISA.encode("NOP")}
    assert image["symbols"] == {"start": 0}
    assert image["threads"] == {"0": {"entry": 0, "size": 1},
                                "1": {"entry": 256, "size": 1}}
    assert all(isinstance(k, str) for k in image["words"])
    assert all(isinstance(v, int) for v in image["words"].values())
    round_tripped = json.loads(program.image_json())
    assert round_tripped == image


def test_write_image_creates_the_directory(tmp_path):
    program = asm(src(".thread 0", "NOP"))
    path = program.write_image(tmp_path / "build" / "x.json")
    assert json.loads(path.read_text(encoding="utf-8")) == program.to_image()


def test_listing_has_a_header_a_row_and_a_deadline_summary():
    program = asm(src(".thread 0", ".tick 50", "SETD 0", "NOP", "WAITD 1"))
    text = program.listing_text()
    assert "ADDR  WORD  TH  TIMING     LINE  SOURCE" in text
    assert "one_slot" in text and "wait" in text
    assert "thread 0 deadline analysis (tick period 50 clocks)" in text
    assert "slack" in text
    assert "0 errors" in text
    # the source text of every line survives into the listing
    assert "SETD 0" in text and "WAITD 1" in text


def test_listing_shows_the_extra_words_of_a_multi_word_statement():
    program = asm(src(".thread 0", ".csr TICK_INT, 434"))
    text = program.listing_text()
    assert "| LDIH r7, 1" in text
    assert "| CSRW TICK_INT, r7" in text


# -------------------------------------------------------------- entry points
def test_assemble_accepts_text_a_path_and_a_path_string(tmp_path):
    text = src(".thread 0", "NOP")
    path = tmp_path / "x.loom"
    path.write_text(text, encoding="utf-8")

    from_text = assemble(text, filename="inline.loom", isa=ISA)
    from_path = assemble(pathlib.Path(path), isa=ISA)
    from_string = assemble(str(path), isa=ISA)

    assert from_text.words == from_path.words == from_string.words
    assert from_text.source == "inline.loom"
    assert from_path.source.endswith("x.loom")


def test_strict_turns_a_deadline_error_into_an_exception():
    text = src(".thread 0", ".tick 4", "SETD 0", "NOP", "NOP", "WAITD 1")
    relaxed = asm(text)
    assert relaxed.words                      # an image is still produced
    assert relaxed.deadline_errors
    with pytest.raises(AsmError):
        asm(text, strict=True)


def test_deadline_check_can_be_skipped_entirely():
    program = asm(src(".thread 0", ".tick 4", "SETD 0", "NOP", "NOP", "WAITD 1"),
                  deadline_check=False)
    assert program.deadlines == {}
    assert program.errors == []


# ---------------------------------------------------- disassembler round trip
_SAMPLE = {
    "rd": "r1", "ra": "r2", "rb": "r3", "imm": "1", "rel": "0", "abs": "0",
    "pin": "IN0", "val": "1", "edge": "RISE", "flag": "5", "tmo": "T",
    "cond": "TICK", "csr": "TD",
}


def every_mnemonic_source():
    lines = [".thread 0"]
    for instr in ISA.instructions:
        operands = [_SAMPLE[operand_base(op)] for op in instr.ops]
        lines.append("        %s %s" % (instr.name, ", ".join(operands)))
    return src(*lines)


def test_the_sample_program_really_uses_every_mnemonic():
    program = asm(every_mnemonic_source())
    seen = {ISA.decode(w)[0].name for w in program.words.values()}
    assert seen == {i.name for i in ISA.instructions}
    assert len(program.words) == len(ISA.instructions)


def test_assemble_disassemble_assemble_gives_the_same_words():
    program = asm(every_mnemonic_source())
    text = src(".thread 0", *["        " + disassemble(program.words[a], ISA)
                              for a in sorted(program.words)])
    again = asm(text)
    assert again.words == program.words


def test_disassembly_of_a_reserved_word_is_a_word_directive():
    assert disassemble(0xE000, ISA) == ".word 0xE000"
    assert asm(src(".thread 0", disassemble(0xE000, ISA))).words == {0: 0xE000}


# --------------------------------------- enumerated operands come from isa.yaml
def instruction_with_operand(base):
    for instr in ISA.instructions:
        for op in instr.ops:
            if operand_base(op) == base:
                return instr
    return None


def render(instr, **overrides):
    parts = [overrides.get(operand_base(op), _SAMPLE[operand_base(op)])
             for op in instr.ops]
    return "        %s %s" % (instr.name, ", ".join(parts))


def test_the_isa_defines_the_enums_the_assembler_uses():
    assert set(ISA.enums) == {"edge", "cond"}


@pytest.mark.parametrize("base", sorted(ISA.enums))
def test_every_enum_name_in_the_yaml_is_accepted_in_any_case(base):
    instr = instruction_with_operand(base)
    assert instr is not None, base
    for name, value in ISA.enums[base].items():
        for spelling in (name, name.lower(), name.swapcase()):
            program = asm(src(".thread 0", render(instr, **{base: spelling})))
            assert ISA.decode(program.words[0])[1][base] == value, spelling


@pytest.mark.parametrize("base", sorted(ISA.enums))
def test_an_enum_operand_also_accepts_the_numbers_the_yaml_names(base):
    instr = instruction_with_operand(base)
    for value in sorted(set(ISA.enums[base].values())):
        program = asm(src(".thread 0", render(instr, **{base: str(value)})))
        assert ISA.decode(program.words[0])[1][base] == value


@pytest.mark.parametrize("base", sorted(ISA.enums))
def test_a_number_the_yaml_does_not_name_is_rejected(base):
    instr = instruction_with_operand(base)
    high = max(ISA.enums[base].values())
    message = errors_of(src(".thread 0",
                            render(instr, **{base: str(high + 1)})))[0].message
    assert "%s %d is outside 0..%d" % (base, high + 1, high) in message


def test_edge_3_is_rejected_because_isa_yaml_names_only_0_to_2():
    """SEMANTICS.md 6.4: e == 3 is never true, so the language will not write it."""
    assert "edge 3 is outside 0..2" in errors_of(
        src(".thread 0", "WAITE IN0, 3"))[0].message


def test_an_unnamed_enum_encoding_is_still_reachable_with_word():
    word = ISA.encode("WAITE", pin=8, edge=3, tmo=0)
    assert asm(src(".thread 0", ".word 0x%04X" % word)).words == {0: word}


@pytest.mark.parametrize("base", sorted(ISA.enums))
def test_the_disassembler_prints_the_yaml_spelling(base):
    instr = instruction_with_operand(base)
    for name in ISA.enums[base]:
        program = asm(src(".thread 0", render(instr, **{base: name.lower()})))
        assert name in disassemble(program.words[0], ISA)


def test_names_module_holds_no_spelling_that_lives_in_the_yaml():
    """Q4: the enum spellings are single-sourced from isa.yaml."""
    import tools.loomasm.names as names
    text = pathlib.Path(names.__file__).read_text(encoding="utf-8")
    for base, table in ISA.enums.items():
        for name in table:
            assert not re.search(r"\b%s\b" % re.escape(name), text), name
    assert not hasattr(names, "EDGE_NAMES")
    assert not hasattr(names, "COND_NAMES")


# ------------------------------------------------- unnamed pin index warnings
def pin_warnings(program):
    return [d for d in program.warnings if d.kind == "pin"]


@pytest.mark.parametrize("index", [13, 14, 15, 22, 27, 31])
def test_a_pin_index_isa_yaml_does_not_name_warns(index):
    program = asm(src(".thread 0", "SETP %d, 1" % index))
    assert program.errors == []
    assert program.words[0] == ISA.encode("SETP", pin=index, val=1)
    warnings = pin_warnings(program)
    assert len(warnings) == 1
    assert warnings[0].severity == "warning"
    assert warnings[0].line == 2
    assert "pin index %d has no name in isa.yaml" % index in warnings[0].message


@pytest.mark.parametrize("index", sorted(ISA.pins))
def test_a_named_pin_never_warns(index):
    name = ISA.pins[index]["name"]
    assert pin_warnings(asm(src(".thread 0", "SETP %d, 1" % index))) == []
    assert pin_warnings(asm(src(".thread 0", "SETP %s, 1" % name))) == []


def test_the_warning_is_not_dodged_by_a_pins_alias():
    program = asm(src(".thread 0",
                      ".pins SPARE = 13",
                      "SETP SPARE, 1",
                      "JP SPARE, 1, 0"))
    # once per use, not also at the declaration
    assert [d.line for d in pin_warnings(program)] == [3, 4]


def test_the_warning_is_not_dodged_by_an_equ():
    program = asm(src(".thread 0", ".equ SPARE = 22", "SETP SPARE, 1"))
    assert [d.line for d in pin_warnings(program)] == [3]


def test_an_alias_of_a_named_pin_does_not_warn():
    program = asm(src(".thread 0", ".pins TX = OUT0", ".pins B = TX",
                      "SETP TX, 1", "SETP B, 0"))
    assert pin_warnings(program) == []


def test_the_pin_warning_is_not_fatal():
    program = asm(src(".thread 0", "SETP 13, 1"))
    assert program.ok and program.words
    assert all(not d.fatal for d in program.diagnostics)


def test_disassembler_omits_a_clear_timeout_bit_and_names_enums():
    assert disassemble(ISA.encode("WAITP", pin=8, val=0, tmo=0), ISA) == \
        "WAITP IN0, 0"
    assert disassemble(ISA.encode("WAITP", pin=8, val=0, tmo=1), ISA) == \
        "WAITP IN0, 0, T"
    assert disassemble(ISA.encode("WAITE", pin=9, edge=1, tmo=0), ISA) == \
        "WAITE IN1, FALL"
    assert disassemble(ISA.encode("WAITB", cond=2, tmo=0), ISA) == "WAITB INQ_NE"
    assert disassemble(ISA.encode("CSRR", rd=3, csr=9), ISA) == "CSRR r3, NOW"
    assert disassemble(ISA.encode("RET"), ISA) == "RET"
