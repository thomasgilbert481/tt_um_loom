"""The firmware programs assemble, and their schedules are met.

``firmware/uart_hello.loom`` is the M1 acceptance program (docs/PLAN.md M1):
"LOOM\\r\\n" on OUT0, 8N1, 115200 baud from a 50 MHz clock. The words are
checked against ``tools.loomisa.encode`` called independently of the
assembler, so a bug in operand handling cannot hide behind itself.
"""

import pytest

from tools.loomasm import assemble_file, disassemble
from tools.loomisa import REPO, load

ISA = load()

UART_HELLO = REPO / "firmware" / "uart_hello.loom"
UART_TX = REPO / "firmware" / "uart_tx.loom"

TICK_INT = 434                    # 50 MHz / 115200 baud, rounded down
MESSAGE = "LOOM\r\n"


@pytest.fixture(scope="module")
def hello():
    return assemble_file(UART_HELLO, isa=ISA)


@pytest.fixture(scope="module")
def tx():
    return assemble_file(UART_TX, isa=ISA)


def at(program, label, offset=0):
    return program.words[program.symbols[label] + offset]


# ------------------------------------------------------------- uart_hello
def test_uart_hello_assembles_without_a_diagnostic(hello):
    assert hello.errors == []
    assert hello.warnings == []
    assert hello.words


def test_uart_hello_is_thread_0_and_starts_at_the_reset_vector(hello):
    assert list(hello.threads) == [0]
    assert hello.threads[0].entry == 0
    assert hello.threads[0].size == len(hello.words)


def test_uart_hello_sets_the_115200_baud_divider(hello):
    """.csr TICK_INT, 434 is MOV16 into r7 followed by CSRW."""
    words = [hello.words[a] for a in range(5)]
    assert words == [
        ISA.encode("LDI", rd=7, imm=TICK_INT & 0xFF),
        ISA.encode("LDIH", rd=7, imm=TICK_INT >> 8),
        ISA.encode("CSRW", csr=ISA.csr_by_name["TICK_INT"], ra=7),
        ISA.encode("LDI", rd=7, imm=0),
        ISA.encode("CSRW", csr=ISA.csr_by_name["TICK_FRAC"], ra=7),
    ]


def test_uart_hello_idles_high_then_anchors_the_deadline(hello):
    out0 = ISA.pin_by_name["OUT0"]
    assert at(hello, "start", 0) == ISA.encode("SETP", pin=out0, val=1)
    assert at(hello, "start", 1) == ISA.encode("SETD", imm=0)


def test_uart_hello_sends_loom_crlf_through_a_call_per_character(hello):
    entry = hello.symbols["send_byte"]
    start = hello.symbols["start"] + 2            # just past SETP and SETD
    expected = []
    for character in MESSAGE:
        expected.append(ISA.encode("LDI", rd=0, imm=ord(character)))
        expected.append(ISA.encode("CALL", abs=entry))
    expected.append(ISA.encode("HALT"))
    got = [hello.words[start + i] for i in range(len(expected))]
    assert got == expected


def test_uart_hello_send_byte_is_a_start_bit_eight_bits_and_a_stop_bit(hello):
    out0 = ISA.pin_by_name["OUT0"]
    assert at(hello, "send_byte", 0) == ISA.encode("SETP", pin=out0, val=0)
    assert at(hello, "send_byte", 1) == ISA.encode("WAITD", imm=1)
    assert at(hello, "send_byte", 2) == ISA.encode("LDI", rd=1, imm=8)
    assert at(hello, "bit", 0) == ISA.encode("SHRI", rd=0, imm=1)
    assert at(hello, "one", 0) == ISA.encode("SETP", pin=out0, val=1)
    assert at(hello, "next", 0) == ISA.encode("WAITD", imm=1)
    assert at(hello, "next", 1) == ISA.encode(
        "DJNZ", rd=1, rel=hello.symbols["bit"] - (hello.symbols["next"] + 2))
    assert at(hello, "next", 2) == ISA.encode("SETP", pin=out0, val=1)
    assert at(hello, "next", 3) == ISA.encode("WAITD", imm=1)
    assert at(hello, "next", 4) == ISA.encode("RET")


def test_uart_hello_uses_one_waitd_per_bit_and_no_fifo_instruction(hello):
    names = [ISA.decode(hello.words[a])[0].name for a in sorted(hello.words)]
    assert names.count("WAITD") == 3          # start, data-bit loop, stop
    assert all(ISA.decode(hello.words[a])[1]["imm"] == 1
               for a in sorted(hello.words)
               if ISA.decode(hello.words[a])[0].name == "WAITD")
    assert not {"PUSH", "POP", "WAITB", "SHO", "SHI"} & set(names)
    assert names.count("CALL") == len(MESSAGE)
    assert names.count("RET") == 1
    assert names.count("HALT") == 1


def test_uart_hello_schedule_is_met_with_a_large_margin(hello):
    result = hello.deadlines[0]
    assert result.period == TICK_INT
    assert result.infeasible == []
    assert result.unbounded == []
    assert len(result.pairs) == 5
    # the longest path is the data-bit loop: DJNZ SHRI BC SETP BRA WAITD
    assert result.worst_slots == 6
    assert result.worst_slack == TICK_INT - 6 * 4 == 410
    assert all(p.slack > TICK_INT // 2 for p in result.pairs)


def test_uart_hello_ret_reaches_the_next_start_bit_in_time(hello):
    """The stop-bit WAITD of one byte anchors the start bit of the next."""
    result = hello.deadlines[0]
    stop = max(p.src_addr for p in result.pairs)
    through_ret = [p for p in result.pairs if p.src_addr == stop]
    assert len(through_ret) == 1
    assert through_ret[0].slots == 5          # RET LDI CALL SETP WAITD
    assert through_ret[0].feasible is True


# --------------------------------------------------------------- uart_tx
def test_uart_tx_still_assembles(tx):
    assert tx.errors == []
    assert tx.words
    assert list(tx.threads) == [0]
    assert tx.threads[0].entry == 0


def test_uart_tx_schedule_is_met_with_a_large_margin(tx):
    result = tx.deadlines[0]
    assert result.period == TICK_INT
    assert result.infeasible == []
    assert result.unbounded == []
    assert result.worst_slots == 6
    assert result.worst_slack == 410


def test_uart_tx_pop_is_re_anchored_by_setd_so_nothing_is_unbounded(tx):
    names = [ISA.decode(tx.words[a])[0].name for a in sorted(tx.words)]
    assert "POP" in names                     # the blocking instruction is there
    assert tx.deadlines[0].unbounded == []    # and SETD makes it harmless
    assert [d for d in tx.warnings if d.kind == "deadline"] == []


# --------------------------------------------------------------- both files
@pytest.mark.parametrize("path", [UART_HELLO, UART_TX])
def test_firmware_round_trips_through_the_disassembler(path):
    program = assemble_file(path, isa=ISA)
    text = ".thread 0\n" + "".join(
        "        %s\n" % disassemble(program.words[a], ISA)
        for a in sorted(program.words))
    from tools.loomasm import assemble_text
    again = assemble_text(text, "round-trip.loom", isa=ISA)
    assert again.words == program.words


@pytest.mark.parametrize("path", [UART_HELLO, UART_TX])
def test_firmware_is_laid_out_for_the_default_1024_word_memory(path):
    program = assemble_file(path, isa=ISA)
    assert program.imem_words == 1024
    assert program.to_image()["imem_words"] == 1024


@pytest.mark.parametrize("path", [UART_HELLO, UART_TX])
def test_firmware_also_fits_the_256_word_m1_build(path):
    """Thread 0's reset vector is 0 whatever IMEM_WORDS is (D-017), so the
    image is identical in the flop build SEMANTICS section 5 describes."""
    big = assemble_file(path, isa=ISA)
    small = assemble_file(path, isa=ISA, imem_words=256)
    assert small.errors == []
    assert small.words == big.words
    assert small.imem_words == 256
    assert max(small.words) < 256


@pytest.mark.parametrize("path", [UART_HELLO, UART_TX])
def test_firmware_listing_and_image_are_produced(path):
    program = assemble_file(path, isa=ISA)
    listing = program.listing_text()
    assert "deadline analysis" in listing
    assert "0 errors" in listing
    image = program.to_image()
    assert image["isa"] == ISA.version
    assert image["source"].endswith(path.name)
    assert len(image["words"]) == len(program.words)
