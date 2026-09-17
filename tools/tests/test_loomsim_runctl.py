"""Run control, the host port and the optional FIFOs (SEMANTICS 7 and 6.7).

Check IDs: SCHED-3 in its simulation form (a thread with RUN clear changes no
state), plus the host rules of ``docs/HOST_PROTOCOL.md``: single-step is
observably identical to free running, debug access to r0..r7 needs a halted
thread, and IMEM access needs a quiet machine.
"""

import pytest

from tools.loomisa import load
from tools.loomsim import LoomsimError, Machine
from tools.loomsim.harness import assemble, run_thread

ISA = load()


def machine_with(program, thread=0, **kwargs):
    kwargs.setdefault("isa", ISA)
    return Machine(assemble(ISA, program, thread * 0x100), **kwargs)


def step_one_slot(machine, thread=0, limit=8):
    """Ask for one slot and run until its record appears."""
    machine.host_step(thread)
    for _ in range(limit):
        record = machine.step_cycle()
        if record is not None and record.thread == thread:
            return record
    return None


# --------------------------------------------------------------------- RUN
def test_run_starts_a_thread_and_is_visible_from_the_next_cycle():
    machine = machine_with([("NOP", {})] * 8)
    machine.host_set_run(0b0001)
    assert machine.run == 0                      # not yet committed
    machine.step_cycle()
    assert machine.run == 0b0001


def test_clearing_run_stops_the_thread():
    machine = machine_with([("NOP", {})] * 40)
    machine.host_set_run(0b0001)
    machine.run_cycles(20)
    before = machine.threads[0].steps
    machine.host_set_run(0b0000)
    machine.run_cycles(40)
    # At most one more slot was already in flight when RUN went away.
    assert 0 <= machine.threads[0].steps - before <= 1
    assert machine.run == 0


def test_a_thread_with_run_clear_changes_nothing():
    machine = machine_with([("LDI", dict(rd=1, imm=0x55))] * 4)
    machine.run_cycles(40)
    assert machine.threads[0].regs == [0] * 8
    assert machine.threads[0].pc == 0
    assert machine.threads[0].steps == 0


def test_halt_sets_halted_and_clears_run():
    machine, records = run_thread(ISA, [("HALT", {})])
    assert machine.halted == 0b0001
    assert machine.run == 0
    assert machine.threads[0].pc == 1            # PC advanced past the HALT


def test_writing_run_again_clears_halted_and_resumes():
    machine, records = run_thread(ISA, [("HALT", {}), ("LDI", dict(rd=1, imm=7)),
                                        ("HALT", {})])
    assert machine.halted == 0b0001
    machine.host_set_run(0b0001)
    machine.step_cycle()
    assert machine.halted == 0
    assert machine.run == 0b0001
    machine.run_cycles(20)
    assert machine.threads[0].regs[1] == 7
    assert machine.halted == 0b0001


def test_halt_only_stops_its_own_thread():
    image = {}
    image.update(assemble(ISA, [("HALT", {})], 0x000))
    image.update(assemble(ISA, [("JMP", dict(abs=0x100))], 0x100))
    machine = Machine(image, isa=ISA)
    machine.host_set_run(0b0011)
    machine.run_cycles(40)
    assert machine.halted == 0b0001
    assert machine.run == 0b0010
    assert machine.threads[1].steps > 5


# -------------------------------------------------------------------- STEP
def test_step_executes_exactly_one_slot():
    machine = machine_with([("LDI", dict(rd=1, imm=0x11)),
                            ("LDI", dict(rd=1, imm=0x22))])
    record = step_one_slot(machine)
    assert record is not None and record.pc == 0
    assert machine.threads[0].pc == 1
    assert machine.threads[0].regs[1] == 0x11
    assert machine.threads[0].steps == 1
    machine.run_cycles(20)
    assert machine.threads[0].steps == 1         # and nothing more happened


def test_stepping_is_identical_to_free_running():
    program = [("LDI", dict(rd=1, imm=3)),
               ("ADDI", dict(rd=2, imm=1)),
               ("DJNZ", dict(rd=1, rel=-2)),
               ("SETP", dict(pin=16, val=1)),
               ("HALT", {})]
    _, free = run_thread(ISA, program)

    machine = machine_with(program)
    stepped = []
    while not (machine.halted & 1):
        record = step_one_slot(machine)
        assert record is not None
        stepped.append(record)
        machine.run_cycles(1)                    # let the commit land
    assert [r.as_tuple() for r in stepped] == [r.as_tuple() for r in free]


def test_a_stepped_stalled_wait_keeps_wait_active():
    machine = machine_with([("WAITP", dict(pin=8, val=1)), ("HALT", {})])
    for expected in (1, 2, 3):
        record = step_one_slot(machine)
        assert record is not None
        assert record.done is False
        assert record.next_pc == 0
        machine.run_cycles(1)
        assert machine.threads[0].wait_active == 1
        assert machine.threads[0].pc == 0
        assert machine.threads[0].steps == expected


def test_step_is_ignored_while_the_thread_runs():
    machine = machine_with([("JMP", dict(abs=0))])
    machine.host_set_run(0b0001)
    machine.run_cycles(8)
    machine.host_step(0)
    machine.step_cycle()
    assert machine.step_req == 0


def test_a_valid_slot_consumes_the_step_request():
    machine = machine_with([("NOP", {})] * 4)
    machine.host_step(0)
    machine.step_cycle()
    assert machine.step_req == 0b0001            # visible from the next cycle
    machine.run_cycles(8)
    assert machine.step_req == 0


def test_stepping_several_threads_independently():
    image = {}
    for thread in range(4):
        image.update(assemble(ISA, [("LDI", dict(rd=1, imm=thread + 1)),
                                    ("HALT", {})], thread * 0x100))
    machine = Machine(image, isa=ISA)
    for thread in (2, 0):
        step_one_slot(machine, thread)
        machine.run_cycles(1)
    assert machine.threads[2].regs[1] == 3
    assert machine.threads[0].regs[1] == 1
    assert machine.threads[1].regs[1] == 0
    assert machine.threads[3].regs[1] == 0


# ------------------------------------------------------------------- RESET
def test_thread_reset_restores_pc_flags_deadline_and_stack():
    program = [("CALL", dict(abs=0x20)), ("HALT", {})]
    extra = {0x20: ISA.encode("HALT")}

    def setup(machine):
        machine.host_write_debug(0, "FLAGS", 0b111)

    machine, _ = run_thread(ISA, program, setup=setup, extra=extra)
    machine.host_write_debug(0, "r3", 0x1234)
    machine.host_write_debug(0, "WAIT_ACTIVE", 1)
    machine.step_cycle()
    assert machine.threads[0].depth == 1

    now_before = machine.threads[0].now
    machine.host_reset_thread(0)
    machine.step_cycle()
    thread = machine.threads[0]
    assert thread.pc == 0                        # RESET_PC[0]
    assert (thread.z, thread.c, thread.t) == (0, 0, 0)
    assert thread.td == now_before
    assert thread.depth == 0
    assert thread.wait_active == 0
    assert thread.regs[3] == 0x1234              # registers are untouched


def test_reset_pc_is_host_writable():
    machine = machine_with([("HALT", {})])
    machine.host_write_reset_pc(0, 0x123)
    machine.host_reset_thread(0)
    machine.step_cycle()
    assert machine.threads[0].pc == 0x123


@pytest.mark.parametrize("words,expected", [
    (1024, [0x000, 0x100, 0x200, 0x300]),
    (512, [0x000, 0x080, 0x100, 0x180]),
    (256, [0x000, 0x040, 0x080, 0x0C0]),
    (128, [0x000, 0x020, 0x040, 0x060]),
])
def test_reset_vectors_divide_the_memory_into_four(words, expected):
    """SEMANTICS 5: RESET_PC[t] = t * (IMEM_WORDS / 4); threads never alias."""
    machine = Machine({}, imem_words=words, isa=ISA)
    assert machine.reset_pc == expected
    assert [t.pc for t in machine.threads] == expected
    assert len(set(expected)) == 4


def test_each_thread_starts_at_its_own_vector_in_a_small_memory():
    image = {}
    for thread in range(4):
        image.update(assemble(ISA, [("LDI", dict(rd=1, imm=thread + 1)),
                                    ("HALT", {})], thread * 64))
    machine = Machine(image, imem_words=256, isa=ISA)
    machine.host_set_run(0b1111)
    machine.run_cycles(32)
    assert machine.halted == 0b1111
    assert [t.regs[1] for t in machine.threads] == [1, 2, 3, 4]


# ------------------------------------------------------------------- DEBUG
def test_registers_are_only_host_visible_while_the_thread_is_halted():
    machine = machine_with([("JMP", dict(abs=0))])
    machine.host_write_debug(0, "r4", 0xABCD)
    machine.step_cycle()
    assert machine.host_read_debug(0, "r4") == 0xABCD

    machine.host_set_run(0b0001)
    machine.run_cycles(8)
    assert machine.host_read_debug(0, "r4") == 0          # reads 0 while running
    machine.host_write_debug(0, "r4", 0x1111)             # and writes are dropped
    machine.run_cycles(8)
    machine.host_set_run(0b0000)
    machine.run_cycles(8)
    assert machine.host_read_debug(0, "r4") == 0xABCD


def test_registers_stay_closed_until_a_stepped_slot_has_drained():
    """"Halted" includes "no valid slot of t in F, D, X or W" (SEMANTICS 7)."""
    machine = machine_with([("LDI", dict(rd=1, imm=0x33)), ("HALT", {})])
    machine.host_write_debug(0, "r1", 0x11)
    machine.step_cycle()
    assert machine.thread_halted_for_debug(0) is True
    assert machine.host_read_debug(0, "r1") == 0x11

    asked_at = machine.cycle
    machine.host_step(0)
    open_by_cycle = {}
    for _ in range(10):
        open_by_cycle[machine.cycle] = machine.thread_halted_for_debug(0)
        machine.step_cycle()
    # Closed from the cycle STEP_REQ becomes visible until the slot has
    # committed, then open again; the value the slot wrote is then readable.
    closed = sorted(c for c, is_open in open_by_cycle.items() if not is_open)
    assert closed == list(range(closed[0], closed[-1] + 1))
    assert closed[0] == asked_at + 1           # STEP_REQ is visible next cycle
    assert open_by_cycle[closed[-1] + 1] is True
    assert machine.host_read_debug(0, "r1") == 0x33
    assert machine.threads[0].steps == 1


def test_other_debug_registers_are_readable_while_running():
    machine = machine_with([("JMP", dict(abs=0))])
    machine.host_set_run(0b0001)
    machine.run_cycles(12)
    assert machine.host_read_debug(0, "PC") == 0
    assert machine.host_read_debug(0, "STEPS") > 0
    assert machine.host_read_debug(0, "NOW") > 0
    assert machine.host_read_debug(0, "TID") == 0


def test_a_debug_pc_write_clears_wait_active():
    machine = machine_with([("WAITP", dict(pin=8, val=1)), ("HALT", {})])
    machine.host_set_run(0b0001)
    machine.run_cycles(20)
    assert machine.threads[0].wait_active == 1
    machine.host_set_run(0)
    machine.run_cycles(8)
    machine.host_write_debug(0, "PC", 1)
    machine.step_cycle()
    assert machine.threads[0].wait_active == 0
    assert machine.threads[0].pc == 1


def test_dump_thread_covers_the_debug_space():
    machine, _ = run_thread(ISA, [("LDI", dict(rd=2, imm=9)), ("HALT", {})])
    dump = machine.dump_thread(0)
    assert dump["r2"] == 9
    assert dump["PC"] == 2
    assert dump["HALTED"] == 1
    assert dump["STEPS"] == 2


def test_unknown_debug_registers_raise():
    machine = machine_with([("HALT", {})])
    with pytest.raises(LoomsimError):
        machine.host_read_debug(0, "NOSUCH")
    with pytest.raises(LoomsimError):
        machine.host_write_debug(0, "NOW", 1)          # NOW is read only


# -------------------------------------------------------------------- IMEM
def test_host_imem_writes_work_while_everything_is_halted():
    machine = Machine({}, isa=ISA)
    machine.host_write_imem(0, ISA.encode("LDI", rd=1, imm=0x5A))
    machine.host_write_imem(1, ISA.encode("HALT"))
    machine.step_cycle()
    assert machine.host_read_imem(0) == ISA.encode("LDI", rd=1, imm=0x5A)
    machine.host_set_run(0b0001)
    machine.run_cycles(24)
    assert machine.threads[0].regs[1] == 0x5A
    assert machine.badop == 0


def test_host_imem_access_while_running_is_dropped_and_flagged():
    machine = machine_with([("JMP", dict(abs=0))])
    machine.host_set_run(0b0001)
    machine.run_cycles(8)
    machine.host_write_imem(0x200, ISA.encode("HALT"))
    machine.step_cycle()
    assert machine.imem.get(0x200, 0) == 0
    assert machine.badop & (1 << 15)
    assert machine.host_read_imem(0x200) == 0


def test_badop_bit_fifteen_is_host_clearable():
    machine = machine_with([("JMP", dict(abs=0))])
    machine.host_set_run(0b0001)
    machine.run_cycles(8)
    machine.host_write_imem(0, 0)
    machine.step_cycle()
    assert machine.badop & (1 << 15)
    machine.host_clear_badop(1 << 15)
    machine.step_cycle()
    assert machine.badop == 0


# ------------------------------------------------------------------- FIFOs
def test_fifos_are_absent_unless_the_feature_is_built():
    machine = machine_with([("HALT", {})])
    with pytest.raises(LoomsimError):
        machine.host_fifo_push(0, 1)


def test_pop_takes_what_the_host_pushed():
    machine = machine_with([("POP", dict(rd=1)), ("POP", dict(rd=2)), ("HALT", {})],
                           features={"FIFO"})
    machine.host_fifo_push(0, 0x1234)
    machine.host_fifo_push(0, 0x5678)
    machine.host_set_run(0b0001)
    machine.run_cycles(40)
    assert machine.threads[0].regs[1] == 0x1234
    assert machine.threads[0].regs[2] == 0x5678
    assert machine.host_fifo_status(0)["inq"] == 0
    assert machine.badop == 0


def test_pop_stalls_while_the_queue_is_empty():
    machine = machine_with([("POP", dict(rd=1)), ("HALT", {})], features={"FIFO"})
    machine.host_set_run(0b0001)
    records = machine.run_cycles(24)
    pops = [r for r in records if r.mnemonic == "POP"]
    assert len(pops) >= 4
    assert all(not r.done for r in pops)
    assert machine.threads[0].wait_active == 1
    machine.host_fifo_push(0, 0xBEEF)
    records = machine.run_cycles(24)
    done = [r for r in records if r.mnemonic == "POP" and r.done]
    assert len(done) == 1
    assert done[0].val == 0xBEEF
    assert machine.threads[0].wait_active == 0


def test_push_fills_the_queue_and_then_stalls():
    program = [("PUSH", dict(ra=1)), ("ADDI", dict(rd=1, imm=1)),
               ("JMP", dict(abs=0))]
    machine = machine_with(program, features={"FIFO"})
    machine.host_write_debug(0, "r1", 1)
    machine.host_set_run(0b0001)
    machine.run_cycles(200)
    assert machine.host_fifo_status(0)["outq"] == 4          # depth 4
    assert machine.threads[0].wait_active == 1
    assert [machine.host_fifo_pop(0) for _ in range(1)] == [1]
    machine.run_cycles(24)
    assert machine.host_fifo_status(0)["outq"] == 4
    assert machine.threads[0].regs[1] >= 5


def test_host_fifo_pop_of_an_empty_queue_returns_zero():
    machine = machine_with([("HALT", {})], features={"FIFO"})
    assert machine.host_fifo_pop(0) == 0


@pytest.mark.parametrize("cond", [1, 2, 3])
def test_waitb_conditions_with_the_fifo_feature(cond):
    machine = machine_with([("WAITB", dict(cond=cond)), ("HALT", {})],
                           features={"FIFO"})
    if cond == 2:
        machine.host_fifo_push(0, 0x0042)                 # INQ not empty
    if cond == 3:
        machine.host_write_debug(0, "TICK_INT", 64)       # ticks are rare now
    machine.host_set_run(0b0001)
    machine.run_cycles(200)
    assert machine.badop == 0
    assert machine.halted == 0b0001                        # every condition came true


def test_waitb_two_waits_for_the_host_to_push():
    machine = machine_with([("WAITB", dict(cond=2)), ("HALT", {})],
                           features={"FIFO"})
    machine.host_set_run(0b0001)
    machine.run_cycles(24)
    assert machine.halted == 0
    machine.host_fifo_push(0, 1)
    machine.run_cycles(24)
    assert machine.halted == 0b0001
    assert machine.host_fifo_status(0)["inq"] == 1         # WAITB does not pop


def test_waitb_one_stalls_while_the_output_queue_is_full():
    program = [("PUSH", dict(ra=1)), ("PUSH", dict(ra=1)), ("PUSH", dict(ra=1)),
               ("PUSH", dict(ra=1)), ("WAITB", dict(cond=1)), ("HALT", {})]
    machine = machine_with(program, features={"FIFO"})
    machine.host_set_run(0b0001)
    machine.run_cycles(60)
    assert machine.host_fifo_status(0)["outq"] == 4
    assert machine.halted == 0
    machine.host_fifo_pop(0)
    machine.run_cycles(24)
    assert machine.halted == 0b0001


def test_waitb_zero_is_a_badop_because_the_bit_engine_is_absent():
    machine = machine_with([("WAITB", dict(cond=0)), ("HALT", {})],
                           features={"FIFO"})
    machine.host_set_run(0b0001)
    machine.run_cycles(24)
    assert machine.halted == 0b0001
    assert machine.badop == 0b0001


def test_tick_seen_is_cleared_by_every_slot():
    """WAITB 3 waits for a tick since the thread's previous slot."""
    machine = machine_with([("WAITB", dict(cond=3)), ("NOP", {}),
                            ("WAITB", dict(cond=3)), ("HALT", {})],
                           features={"FIFO"})
    machine.host_write_debug(0, "TICK_INT", 40)
    machine.host_set_run(0b0001)
    records = machine.run_cycles(400)
    waits = [r for r in records if r.mnemonic == "WAITB"]
    # The second WAITB has to wait for a fresh tick: its own previous slot
    # cleared TICK_SEEN, so it stalls for most of a 40-clock tick period.
    assert len([r for r in waits if not r.done]) >= 5
    assert len([r for r in waits if r.done]) == 2
