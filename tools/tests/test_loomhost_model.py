"""The Loom host API over ModelTransport: every call goes through SPI bytes.

Each test drives the golden model only through ``tools.loomhost.Loom`` (and
so through encoded HOST_PROTOCOL transactions), then checks the result both
through the API and against the model's own state. The last section runs
the model directly against SEMANTICS 6.7 and HOST_PROTOCOL for the host
features it does not implement yet; ModelTransport's shim supplies them, and
those direct tests are expected failures until the model catches up
(docs/spec-questions/firmware.md, FW-8).
"""

import pytest

from tools.loomasm import assemble
from tools.loomhost import (Loom, LoomError, LoomStateError, LoomTimeout,
                            LoomVerifyError, ModelTransport, load_image, thread_mask)
from tools.loomhost import protocol as P
from tools.loomisa import load as load_isa
from tools.loomsim import Machine
from tools.protomodels.bench import Bench

ISA = load_isa()

ECHO = """
        .thread 0
loop:   POP     r0
        ADDI    r0, 1
        PUSH    r0
        JMP     loop
"""

COUNT = """
        .thread 0
        LDI     r1, 0
loop:   ADDI    r1, 1
        JMP     loop
"""


def make(source=None, **kwargs):
    transport = ModelTransport(**kwargs)
    loom = Loom(transport, isa=ISA)
    program = None
    if source is not None:
        program = assemble(source, isa=ISA)
        loom.load(program)
    return loom, transport, program


# ----------------------------------------------------------------- identity
def test_id_version_and_caps():
    loom, t, _ = make()
    assert loom.id() == 0x4C4D
    loom.check_id()
    assert loom.version() == (0, 1)
    caps = loom.caps()
    assert (caps["imem_words"], caps["fifos"], caps["fifo_depth"]) == (1024, 1, 4)
    assert caps["raw"] == t.machine.caps


def test_m1_build_has_no_fifos():
    loom, t, _ = make(features=())
    assert loom.caps()["fifos"] == 0
    with pytest.raises(LoomStateError, match="no FIFOs"):
        loom.push(0, [1])
    # the FIFO space reads 0 and ignores writes, without BADOP
    loom.write(P.SPACE_FIFO, 0, [5])
    assert loom.read(P.SPACE_FIFO, 0x100) == [0]
    assert loom.badop() == 0


def test_thread_mask():
    assert thread_mask(None) == 0xF
    assert thread_mask(2) == 4
    assert thread_mask([0, 3]) == 9
    with pytest.raises(ValueError):
        thread_mask(4)


# --------------------------------------------------------------------- load
def test_load_writes_imem_and_verifies_by_readback():
    loom, t, program = make(ECHO)
    for addr, word in program.words.items():
        assert t.machine.imem[addr] == word
    assert loom.verify(program) == []
    assert loom.read_imem(0, len(program.words)) == [program.words[a]
                                                     for a in sorted(program.words)]


def test_load_accepts_every_image_form(tmp_path):
    program = assemble(ECHO, isa=ISA)
    image = program.to_image()
    path = tmp_path / "echo.json"
    path.write_text(program.image_json())
    src = tmp_path / "echo.loom"
    src.write_text(ECHO)
    words = dict(program.words)
    for form in (program, image, words, [words[a] for a in sorted(words)], path, str(path), src):
        assert load_image(form)[0] == words


def test_load_while_running_is_refused_before_anything_is_sent():
    loom, t, program = make(COUNT)
    loom.run(0)
    before = t.transactions
    with pytest.raises(LoomStateError, match="running"):
        loom.load({0x200: 0x1234})
    assert t.transactions == before + 1                 # only the RUN read
    assert 0x200 not in t.machine.imem


def test_load_checks_the_memory_size():
    loom, t, _ = make(imem_words=256)
    two_threads = ECHO + ".thread 1\n NOP\n"
    with pytest.raises(LoomError, match="laid out for 1024"):
        loom.load(assemble(two_threads, isa=ISA))           # thread 1 at 0x100
    with pytest.raises(LoomError, match="outside"):
        loom.load({300: 1})
    loom.load(assemble(two_threads, isa=ISA, imem_words=256))
    loom.load(assemble(ECHO, isa=ISA))                      # thread 0 only: portable
    assert t.machine.imem[0x40] == ISA.encode("NOP")        # thread 1 of 256 words


def test_load_reports_a_readback_mismatch():
    class Flaky(ModelTransport):
        def _read(self, space, addr):
            value = super()._read(space, addr)
            return value ^ 1 if space == P.SPACE_IMEM and addr == 2 else value

    loom = Loom(Flaky(), isa=ISA)
    with pytest.raises(LoomVerifyError, match="0x002"):
        loom.load(assemble(ECHO, isa=ISA))


def test_imem_access_while_running_sets_badop_15():
    loom, t, _ = make(COUNT)
    loom.run(0)
    loom.write(P.SPACE_IMEM, 0x300, [0xBEEF])           # raw: no state check
    assert loom.read(P.SPACE_IMEM, 0) == [0]
    assert loom.badop() == P.BADOP_ACCESS                # read and cleared
    assert loom.badop() == 0
    assert 0x300 not in t.machine.imem


# ------------------------------------------------------------- run control
def test_run_halt_and_the_registers_while_running():
    loom, t, _ = make(COUNT)
    loom.run(0)
    assert loom.running() == 1
    assert loom.read_reg(0, "r1") == 0                   # r0..r7 read 0 while running
    assert loom.read_reg(0, "NOW") > 0                   # the rest reads at any time
    with pytest.raises(LoomStateError, match="running"):
        loom.write_reg(0, "r1", 5)
    loom.halt(0)
    assert loom.running() == 0
    count = loom.read_reg(0, "r1")
    assert count > 10 and count == t.machine.threads[0].regs[1]


def test_run_is_read_modify_write_unless_exclusive():
    loom, t, _ = make(COUNT)
    loom.run(0)
    loom.run(2)
    assert loom.running() == 0b0101
    loom.run(1, exclusive=True)
    assert loom.running() == 0b0010
    loom.halt()
    assert loom.running() == 0


def test_step_executes_exactly_one_slot_and_matches_the_model():
    loom, t, _ = make(COUNT)
    loom.step(0)
    assert loom.read_reg(0, "PC") == 1 and loom.read_reg(0, "STEPS") == 1
    loom.step(0, 5)
    state = loom.dump(0)
    model = t.machine.dump_thread(0)
    for key in ("r0", "r1", "PC", "FLAGS", "TD", "STEPS", "RS0", "RS1", "DEPTH",
                "WAIT_ACTIVE", "TICK_INT", "TICK_FRAC", "OUTGRP", "INGRP", "DT",
                "TICK_SEEN", "RUN", "HALTED", "BADOP"):
        assert state[key] == model[key], key
    assert state["STEPS"] == 6 and state["r1"] == 3
    assert state["NOW"] <= model["NOW"]                   # time went on after the read


def test_step_of_a_running_thread_is_refused():
    loom, t, _ = make(COUNT)
    loom.run(0)
    with pytest.raises(LoomStateError):
        loom.step(0)


def test_dump_has_the_whole_debug_space():
    loom, t, _ = make(ECHO)
    state = loom.dump(1)
    for key in ["r%d" % i for i in range(8)] + [
            "PC", "FLAGS", "TD", "NOW", "SR", "CNT", "CRC", "RS0", "RS1", "DEPTH",
            "STEPS", "WAIT_ACTIVE", "DT", "TICK_SEEN", "LAT_VALID", "LAT_VAL",
            "LAT_PIN", "INQ_CNT", "OUTQ_CNT", "TICK_INT", "BE_CFG", "CRC_INIT",
            "TID", "RUN", "HALTED", "BADOP"]:
        assert key in state
    assert state["TID"] == 1 and state["PC"] == 0x100 and state["TICK_INT"] == 1


def test_reset_and_reset_vectors():
    loom, t, _ = make(COUNT)
    loom.step(0, 3)
    loom.write_reg(0, "FLAGS", 7)
    loom.set_reset_pc(0, 0x40)
    assert loom.reset_pc(0) == 0x40
    loom.reset(0)
    state = loom.dump(0)
    assert (state["PC"], state["FLAGS"], state["DEPTH"], state["WAIT_ACTIVE"]) == (0x40, 0, 0, 0)
    assert state["r1"] == 1                               # registers untouched
    assert [loom.reset_pc(t) for t in (1, 2, 3)] == [0x100, 0x200, 0x300]
    loom.run(0)
    with pytest.raises(LoomStateError):
        loom.reset(0)


def test_halt_instruction_sets_halted_and_irq_stat2():
    loom, t, _ = make(".thread 0\n NOP\n HALT\n")
    loom.run(0)
    loom.wait_halted(0)
    assert loom.halted() == 1 and loom.running() == 0
    assert loom.irq_status()["halted"] == 1
    assert not loom.irq_pending()
    loom.irq_enable2(1)
    assert loom.irq_pending()
    loom.run(0)                                           # 0 -> 1 clears HALTED
    assert loom.halted() == 0


# ----------------------------------------------------- registers and CSRs
def test_register_writes_while_halted():
    loom, t, _ = make(COUNT)
    loom.write_reg(0, "r5", 0x1234)
    loom.write_reg(0, "PC", 0x10)
    loom.write_reg(0, "RS1", 0x2AA)
    loom.write_reg(0, "DEPTH", 2)
    th = t.machine.threads[0]
    assert (th.regs[5], th.pc, th.rs1, th.depth) == (0x1234, 0x10, 0x2AA, 2)
    assert loom.read_reg(0, "RS1") == 0x2AA and loom.read_reg(0, "DEPTH") == 2
    with pytest.raises(LoomError, match="read-only"):
        loom.write_reg(0, "NOW", 1)
    with pytest.raises(LoomError, match="no debug register"):
        loom.read_reg(0, "XYZ")


def test_steps_is_writable_while_halted():
    loom, t, _ = make(COUNT)
    loom.write_reg(0, "STEPS", 100)
    loom.step(0)
    assert loom.read_reg(0, "STEPS") == 101


def test_debug_writes_to_a_running_thread_are_dropped():
    loom, t, _ = make(COUNT)
    loom.run(0)
    outgrp = P.DEBUG_CSR_BASE + ISA.csr_by_name["OUTGRP"]
    loom.write(P.SPACE_DEBUG, P.debug_addr(0, outgrp), [0x55])          # raw
    loom.write(P.SPACE_DEBUG, P.debug_addr(0, P.DEBUG["TD"]), [0x55])
    loom.write(P.SPACE_DEBUG, P.debug_addr(0, P.DEBUG["r1"]), [0xAAAA])
    loom.halt(0)
    th = t.machine.threads[0]
    assert (th.outgrp, th.td) == (0, 0)
    assert th.regs[1] != 0xAAAA and th.pc in (1, 2)


def test_csr_window_by_name_and_number():
    loom, t, _ = make(COUNT)
    loom.write_csr(thread=0, csr="TICK_INT", value=434)   # HOST_PROTOCOL's example
    loom.write_csr(0, 0x01, 8)                            # TICK_FRAC by number
    assert t.machine.threads[0].tick_int == 434
    assert loom.read_csr(0, "tick_int") == 434 and loom.read_csr(0, "TICK_FRAC") == 8
    assert loom.read_csr(3, "TID") == 3
    loom.write_csr(0, "BE_CFG", 0x3FF)                     # not built: reads 0
    assert loom.read_csr(0, "BE_CFG") == 0
    with pytest.raises(LoomError, match="read-only"):
        loom.write_csr(0, "TID", 1)
    with pytest.raises(LoomError, match="no CSR"):
        loom.read_csr(0, "NOPE")


def test_global_csrs_go_through_ctrl():
    loom, t, _ = make()
    loom.write_csr(0, "OD_MASK", 0x81)
    loom.write_csr(0, "SFLAGS", 0x05)
    assert t.machine.od_mask == 0x81 and loom.od_mask() == 0x81
    assert loom.read_csr(2, "SFLAGS") == 0x05 == loom.sflags()
    assert loom.read_csr(0, "HOST_IRQ") == 0
    with pytest.raises(LoomError):
        loom.write_csr(0, "PIN_IN", 1)
    with pytest.raises(LoomError):
        loom.write_csr(0, "HOST_IRQ", 1)


def test_sflags_set_and_clear():
    loom, t, _ = make()
    loom.set_sflags(0xF0)
    loom.clear_sflags(0x30)
    assert loom.sflags() == 0xC0 == t.machine.sflags


# -------------------------------------------------------------------- FIFOs
def test_push_and_pop_through_a_running_thread():
    loom, t, _ = make(ECHO)
    loom.run(0)
    # Twice the depth: push() waits for room. (More than INQ + OUTQ + the word
    # the thread holds would deadlock, since nothing pops OUTQ meanwhile.)
    loom.push(0, range(10, 18))
    assert loom.pop(0, 8) == list(range(11, 19))
    status = loom.fifo_status(0)
    assert (status["inq"], status["outq"], status["inq_empty"], status["outq_empty"],
            status["depth"]) == (0, 0, 1, 1, 4)
    assert loom.badop() == 0


def test_fifo_status_counts_and_full_flags():
    loom, t, _ = make(ECHO)                               # halted: nothing consumes
    loom.push(0, [1, 2, 3, 4])
    s = loom.fifo_status(0)
    assert (s["inq"], s["inq_full"], s["inq_empty"]) == (4, 1, 0)
    assert (loom.read_reg(0, "INQ_CNT"), loom.read_reg(0, "OUTQ_CNT")) == (4, 0)
    assert loom.dump(0)["INQ_CNT"] == 4


def test_push_to_a_full_inq_drops_the_word_and_sets_badop_14():
    loom, t, _ = make(ECHO)
    loom.push(0, [1, 2, 3, 4])
    loom.push(0, [5], check=False)
    assert loom.badop() == P.BADOP_FIFO
    assert t.machine.threads[0].inq == [1, 2, 3, 4]
    with pytest.raises(LoomTimeout):
        loom.push(0, [6], max_polls=3)


def test_pop_from_an_empty_outq_reads_0_and_sets_badop_14():
    loom, t, _ = make(ECHO)
    assert loom.pop_raw(0, 2) == [0, 0]
    assert loom.badop() == P.BADOP_FIFO
    assert loom.pop_available(0) == []
    with pytest.raises(LoomTimeout):
        loom.pop(0, 1, max_polls=3)


def test_one_transaction_pops_several_words_without_crossing_threads():
    loom, t, _ = make(ECHO)
    loom.run(0)
    loom.push(0, [1, 2, 3])
    t.idle(200)
    assert loom.read(P.SPACE_FIFO, P.fifo_addr(0), 3) == [2, 3, 4]
    assert loom.badop() == 0


def test_fifo_reads_peek_then_pop_when_the_word_is_complete():
    """The M2 RTL's reading (docs/spec-questions/rtl-m2.md 1): nothing is lost."""
    loom, t, _ = make(ECHO)
    loom.run(0)
    loom.push(0, [1, 2, 3])
    t.idle(200)                                           # OUTQ = [2, 3, 4]
    tx = P.encode_read(P.SPACE_FIFO, P.fifo_addr(0), 2)
    rx = t.transfer(tx[:-1])                              # the second word cut short
    assert P.decode_read_response(rx + b"\x00", 2)[0] == 2
    assert loom.fifo_status(0)["outq"] == 2               # only the whole word popped
    assert loom.badop() == 0
    assert loom.read(P.SPACE_FIFO, P.fifo_addr(0), 3) == [3, 4, 0]
    assert loom.badop() == P.BADOP_FIFO                   # the third word found it empty
    assert loom.fifo_status(0)["outq"] == 0


def test_reset_empties_both_fifos():
    loom, t, _ = make(ECHO)
    loom.push(0, [1, 2])
    loom.run(0)
    t.idle(100)
    loom.push(0, [7])
    loom.halt(0)
    loom.push(0, [8, 9])
    loom.reset(0)
    s = loom.fifo_status(0)
    assert (s["inq"], s["outq"]) == (0, 0)


# ----------------------------------------------------------- BADOP and IRQ
def test_reserved_opcode_sets_the_thread_bit():
    loom, t, _ = make()
    loom.load({0x100: 0xE000, 0x101: ISA.encode("HALT")})  # 0xE000 decodes to nothing
    assert ISA.decode(0xE000) is None
    loom.run(1)
    loom.wait_halted(1)
    assert loom.badop(clear=False) == 0b0010
    assert loom.dump(1)["BADOP"] == 1
    loom.clear_badop(0b0010)
    assert loom.badop() == 0


def test_irq_stat_reflects_sflags_and_fifos():
    loom, t, _ = make(ECHO)
    s = loom.irq_status()
    assert (s["sflags"], s["inq_not_full"], s["outq_not_empty"]) == (0, 0xF, 0)
    loom.push(0, [1, 2, 3, 4])
    loom.set_sflags(0x81)
    s = loom.irq_status()
    assert (s["sflags"], s["inq_not_full"], s["outq_not_empty"]) == (0x81, 0xE, 0)
    loom.run(0)
    t.idle(200)
    assert loom.irq_status()["outq_not_empty"] == 1
    assert not loom.irq_pending()
    loom.irq_enable(0x0001)                               # OUTQ[0] not empty
    assert loom.irq_pending()
    loom.pop(0, 4)
    assert not loom.irq_pending()


def test_swirq_is_set_by_csrw_host_irq_and_cleared_by_the_host():
    loom, t, _ = make(".thread 2\n CSRW HOST_IRQ, r0\n HALT\n")
    loom.run(2)
    loom.wait_halted(2)
    assert loom.swirq() == 0b0100
    assert loom.irq_pending()                             # SWIRQ needs no enable
    loom.clear_swirq(0b0100)
    assert loom.swirq() == 0 and not loom.irq_pending()


# --------------------------------------------------------------------- pins
def test_pin_registers():
    bench = Bench()
    t = ModelTransport(bench)
    loom = Loom(t, isa=ISA)
    loom.set_pin_out(0x2A05)
    loom.set_pin_oe(0x05)
    assert loom.pin_out() == 0x2A05 and loom.pin_oe() == 0x05
    assert t.machine.uo_out == 0x2A and t.machine.uio_oe == 0x05
    # loopback: uio bits the chip drives read back through PIN_IN
    assert loom.pin_in() & 0xFF == 0x05
    bench.ui_idle = 0x10 | 0x81                           # IN0 and IN4 high
    t.idle(4)
    assert loom.pin_in() >> 8 == 0b10001


def test_delay_advances_model_time():
    loom, t, _ = make()
    before = t.cycle
    loom.delay(2e-6)                                      # 100 clocks at 50 MHz
    assert t.cycle - before == 100


def test_transport_timing_is_at_least_64_clocks_per_byte():
    with pytest.raises(ValueError):
        ModelTransport(clocks_per_byte=32)
    t = ModelTransport(clocks_per_byte=80, cs_setup=4, cs_hold=4, cs_gap=8)
    before = t.cycle
    t.transfer(P.encode_read(P.SPACE_CTRL, 0, 1))
    assert t.cycle - before == 6 * 80 + 16


# ---------------------------------------------------- the model versus spec
# ModelTransport supplies these; the model's own host API does not have them
# yet. Each is recorded in docs/spec-questions/firmware.md (FW-8).
def _machine():
    m = Machine({}, features={"FIFO"})
    return m


def test_model_push_to_full_inq_sets_badop_14():
    m = _machine()
    for word in range(5):
        m.host_fifo_push(0, word)
        m.step_cycle()
    assert m.badop & (1 << 14)


def test_model_pop_from_empty_outq_sets_badop_14():
    m = _machine()
    assert m.host_fifo_pop(0) == 0
    m.step_cycle()
    assert m.badop & (1 << 14)


def test_model_reset_empties_fifos():
    m = _machine()
    m.host_fifo_push(0, 1)
    m.step_cycle()
    m.host_reset_thread(0)
    m.step_cycle()
    assert m.host_fifo_status(0)["inq"] == 0


def test_model_drops_debug_writes_while_running():
    m = Machine({0: ISA.encode("JMP", abs=0)}, features={"FIFO"})
    m.host_set_run(1)
    m.run_cycles(8)
    m.host_write_debug(0, "OUTGRP", 0x55)
    m.run_cycles(2)
    m.host_set_run(0)
    m.run_cycles(8)
    assert m.threads[0].outgrp == 0


def test_model_steps_is_writable():
    m = _machine()
    m.host_write_debug(0, "STEPS", 7)
    m.step_cycle()
    assert m.threads[0].steps == 7
