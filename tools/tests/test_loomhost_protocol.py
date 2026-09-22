"""tools.loomhost.protocol: the byte layout of docs/HOST_PROTOCOL.md.

Pure functions, no model. The worked examples of HOST_PROTOCOL are checked
byte for byte; example 1 is checked against the CMD format rather than its
printed CMD byte, which contradicts the format (docs/spec-questions/firmware.md,
FW-1).
"""

import pytest

from tools.loomhost import protocol as P


def hx(text):
    return bytes.fromhex(text.replace(" ", ""))


# ------------------------------------------------------------------ CMD byte
@pytest.mark.parametrize("space", range(6))
@pytest.mark.parametrize("write", [False, True])
def test_cmd_byte_is_rw_space_and_four_zero_bits(space, write):
    cmd = P.cmd_byte(space, write)
    assert cmd == (write << 7) | (space << 4)
    assert cmd & 0x0F == 0
    assert P.parse_cmd(cmd) == (write, space)


def test_parse_cmd_ignores_the_reserved_low_nibble():
    assert P.parse_cmd(0x9F) == (True, P.SPACE_IMEM)


def test_cmd_byte_rejects_a_space_that_does_not_fit():
    with pytest.raises(ValueError):
        P.cmd_byte(8, True)


# ---------------------------------------------------------- worked examples
def test_worked_example_1_load_three_words_at_imem_0():
    """HOST_PROTOCOL prints CMD 0x81 here; the format gives 0x90 (FW-1)."""
    tx = P.encode_write(P.SPACE_IMEM, 0x0000, [0x1234, 0x5678, 0x9ABC])
    assert tx == hx("90 00 00 12 34 56 78 9A BC")
    # The printed byte would be taken as a CTRL write by the format:
    assert P.parse_cmd(0x81) == (True, P.SPACE_CTRL)


def test_worked_example_1_run_thread_0():
    assert P.encode_write(P.SPACE_CTRL, P.CTRL["RUN"], [0b0001]) == hx("80 00 02 00 01")


def test_worked_example_2_read_outq_1_twice():
    tx = P.encode_read(P.SPACE_FIFO, P.fifo_addr(1), 2)
    assert tx == hx("30 00 01 00 00 00 00 00")
    rx = hx("00 00 00 00 D1 11 D2 22")
    assert P.decode_read_response(rx, 2) == [0xD111, 0xD222]


def test_worked_example_3_step_thread_2_then_read_pc_and_r0():
    assert P.encode_write(P.SPACE_STEP, P.step_addr(2), [1]) == hx("D0 00 02 00 01")
    assert P.encode_read(P.SPACE_DEBUG, P.debug_addr(2, P.DEBUG["PC"]), 1) == hx("40 02 08 00 00 00")
    assert P.encode_read(P.SPACE_DEBUG, P.debug_addr(2, P.DEBUG["r0"]), 1) == hx("40 02 00 00 00 00")


# ---------------------------------------------------------------- addresses
def test_debug_address_is_thread_in_9_8_and_register_in_7_0():
    assert P.debug_addr(3, 0x26) == 0x0326
    assert P.debug_addr(1, P.DEBUG_CSR_BASE + 0x0A) == 0x011A


def test_fifo_addresses_data_and_status():
    assert [P.fifo_addr(t) for t in range(4)] == [0, 1, 2, 3]
    assert [P.fifo_addr(t, status=True) for t in range(4)] == [0x100, 0x101, 0x102, 0x103]


@pytest.mark.parametrize("bad", [-1, 4, "0"])
def test_thread_numbers_are_checked(bad):
    with pytest.raises(ValueError):
        P.debug_addr(bad, 0)


def test_addresses_increment_and_wrap_except_in_the_fifo_space():
    assert P.word_addresses(P.SPACE_IMEM, 0xFFFE, 3) == [0xFFFE, 0xFFFF, 0x0000]
    assert P.word_addresses(P.SPACE_FIFO, 0x0002, 3) == [2, 2, 2]
    assert P.word_addresses(P.SPACE_DEBUG, 0x0125, 2) == [0x0125, 0x0126]


def test_register_maps_match_host_protocol():
    assert (P.CTRL["ID"], P.CTRL["RUN"], P.CTRL["RESET"], P.CTRL["RESET_PC3"]) == (0, 2, 4, 0x0B)
    assert (P.CTRL["IRQ_EN"], P.CTRL["IRQ_STAT"], P.CTRL["IRQ_STAT2"]) == (0x10, 0x11, 0x12)
    assert (P.CTRL["SFLAGS"], P.CTRL["SFLAGS_CLR"], P.CTRL["OD_MASK"]) == (0x13, 0x14, 0x15)
    assert (P.CTRL["PIN_OUT"], P.CTRL["PIN_OE"], P.CTRL["PIN_IN"]) == (0x16, 0x17, 0x18)
    assert (P.CTRL["CAPS"], P.CTRL["BADOP"], P.CTRL["SWIRQ"], P.CTRL["IRQ_EN2"]) == (
        0x19, 0x1A, 0x1B, 0x1C)
    assert [P.DEBUG["r%d" % i] for i in range(8)] == list(range(8))
    assert (P.DEBUG["PC"], P.DEBUG["FLAGS"], P.DEBUG["TD"], P.DEBUG["NOW"]) == (8, 9, 0xA, 0xB)
    assert (P.DEBUG["SR"], P.DEBUG["CNT"], P.DEBUG["CRC"], P.DEBUG["RS0"]) == (0xC, 0xD, 0xE, 0xF)
    assert (P.DEBUG["STEPS"], P.DEBUG["RS1_DEPTH"], P.DEBUG["WAIT_ACTIVE"]) == (0x20, 0x21, 0x22)
    assert (P.DEBUG["DT"], P.DEBUG["TICK_SEEN"], P.DEBUG["LAT"], P.DEBUG["FIFO_CNT"]) == (
        0x23, 0x24, 0x25, 0x26)


# ------------------------------------------------------------ frame decode
def test_encode_write_checks_words_and_address():
    with pytest.raises(ValueError):
        P.encode_write(P.SPACE_CTRL, 0, [0x10000])
    with pytest.raises(ValueError):
        P.encode_write(P.SPACE_CTRL, 0x10000, [1])


def test_decode_transaction_write_and_read():
    t = P.decode_transaction(hx("90 01 00 AB CD 12 34"))
    assert (t.write, t.space, t.addr, t.words, t.count, t.partial) == (
        True, P.SPACE_IMEM, 0x100, [0xABCD, 0x1234], 2, False)
    assert t.addresses == [0x100, 0x101]
    r = P.decode_transaction(hx("40 02 08 00 00 00 00"))
    assert (r.write, r.space, r.addr, r.count, r.partial) == (False, P.SPACE_DEBUG, 0x208, 1, True)
    assert P.decode_transaction(hx("90 01")) is None


def test_chip_events_commit_writes_after_their_second_byte():
    events = P.chip_events(P.encode_write(P.SPACE_CTRL, 0x10, [1, 2, 3]))
    assert [(e.after_byte, e.kind, e.addr, e.word) for e in events] == [
        (5, "write", 0x10, 1), (7, "write", 0x11, 2), (9, "write", 0x12, 3)]


def test_chip_events_drop_a_word_cut_in_half():
    tx = P.encode_write(P.SPACE_IMEM, 0, [0x1111, 0x2222])[:-1]
    assert [e.word for e in P.chip_events(tx)] == [0x1111]


def test_chip_events_fetch_reads_before_their_first_byte():
    events = P.chip_events(P.encode_read(P.SPACE_FIFO, 2, 3))
    assert [(e.after_byte, e.kind, e.addr, e.index) for e in events] == [
        (4, "read", 2, 0), (6, "read", 2, 1), (8, "read", 2, 2)]
    # a read cut after the first byte of its second word still fetched it
    assert len(P.chip_events(P.encode_read(P.SPACE_FIFO, 2, 2)[:-1])) == 2
    assert P.chip_events(hx("40 00 00 00")) == []


# ---------------------------------------------------------- packed words
def test_fifo_status_word_layout():
    word = P.pack_fifo_status(inq=4, outq=0, depth=4)
    assert word == (0 << 8) | (4 << 4) | (1 << 3) | (0 << 2) | (0 << 1) | 1
    assert P.unpack_fifo_status(word) == {"outq": 0, "inq": 4, "outq_empty": 1,
                                          "outq_full": 0, "inq_empty": 0, "inq_full": 1}
    assert P.unpack_fifo_status(P.pack_fifo_status(0, 4, 4))["outq_full"] == 1


def test_caps_layout_is_semantics_section_5():
    assert P.decode_caps(0x8000)["imem_words"] == 256          # the M1 build
    caps = P.decode_caps(0xA00A)
    assert (caps["imem_words"], caps["fifos"], caps["fifo_depth"]) == (1024, 1, 4)
    assert P.encode_caps(1024, fifo_depth=4) == 0xA00A
    word = P.encode_caps(512, fifo_depth=8, bit_engine=True, dmem=True, boot_rom=True,
                         setp_deadline=True, bit_engine_auto=True, bit_engine_enc=True)
    assert word & 0x200 and P.decode_caps(word)["bit_engine_enc"] == 1   # CAPS[9], slice A
    back = P.decode_caps(word)
    assert (back["imem_words"], back["fifo_depth"], back["bit_engine"], back["dmem"],
            back["boot_rom"], back["setp_deadline"], back["bit_engine_auto"]) == (
        512, 8, 1, 1, 1, 1, 1)


def test_irq_stat_and_debug_packings():
    assert P.pack_irq_stat(0xA5, 0b0011, 0b1000) == 0xA538
    assert P.unpack_irq_stat(0xA538) == {"sflags": 0xA5, "inq_not_full": 3, "outq_not_empty": 8}
    assert P.pack_rs1_depth(0x3FF, 2) == 0x0BFF
    assert P.unpack_rs1_depth(0x0BFF) == (0x3FF, 2)
    assert P.pack_lat(1, 1, 16) == 0x70
    assert P.unpack_lat(0x70) == {"valid": 1, "value": 1, "pin": 16}
    assert P.unpack_fifo_counts(P.pack_fifo_counts(3, 4)) == (3, 4)


def test_register_names_are_case_insensitive():
    assert P.reg_name_key("R3") == "r3"
    assert P.reg_name_key("pc") == "PC"
    assert P.reg_name_key("tick_int") == "TICK_INT"
