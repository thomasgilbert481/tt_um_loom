"""Slot timing: phases, the four-cycle cadence, and thread isolation.

Check IDs: L2-SLOT (a thread's slots land on its own phase and nothing takes
more than one slot except a stall), SCHED-2 and ISO-1 in their simulation form
(two threads never disturb each other's timing).
"""

import pytest

from tools.loomisa import load
from tools.loomsim import SLOT_CLOCKS, Machine
from tools.loomsim.harness import assemble, run_thread

ISA = load()


def retires(machine, cycles):
    """Run ``cycles`` cycles, returning ``(w_cycle, record)`` for each retire."""
    out = []
    for _ in range(cycles):
        cycle = machine.cycle
        record = machine.step_cycle()
        if record is not None:
            out.append((cycle, record))
    return out


def spinner(thread):
    """An image with an endless one-instruction loop at a thread's reset vector."""
    base = thread * 0x100
    return {base: ISA.encode("JMP", abs=base)}


def test_a_slot_takes_four_stages_and_retires_three_cycles_after_fetch():
    """RUN is visible from cycle 1, so thread 0's first F is cycle 4, W cycle 7."""
    machine = Machine(spinner(0), isa=ISA)
    machine.host_set_run(0b0001)
    seen = retires(machine, 16)
    assert [cycle for cycle, _ in seen] == [7, 11, 15]
    assert [record.x_cycle for _, record in seen] == [6, 10, 14]


@pytest.mark.parametrize("thread", [0, 1, 2, 3])
def test_each_thread_retires_on_its_own_phase_every_four_cycles(thread):
    machine = Machine(spinner(thread), isa=ISA)
    machine.host_set_run(1 << thread)
    seen = retires(machine, 40)
    cycles = [cycle for cycle, _ in seen]
    assert len(cycles) >= 8
    for cycle in cycles:
        assert cycle % SLOT_CLOCKS == (thread + 3) % SLOT_CLOCKS
    gaps = {b - a for a, b in zip(cycles, cycles[1:])}
    assert gaps == {SLOT_CLOCKS}
    for _, record in seen:
        assert record.thread == thread
        assert record.x_cycle % SLOT_CLOCKS == (thread + 2) % SLOT_CLOCKS


def test_all_four_threads_share_the_pipeline_round_robin():
    image = {}
    for thread in range(4):
        image.update(spinner(thread))
    machine = Machine(image, isa=ISA)
    machine.host_set_run(0b1111)
    seen = retires(machine, 24)
    # One retire in every cycle from 4 on, cycling through the threads.
    assert [cycle for cycle, _ in seen] == list(range(4, 24))
    assert [record.thread for _, record in seen][:8] == [1, 2, 3, 0, 1, 2, 3, 0]


def test_a_halted_thread_produces_no_records():
    machine = Machine(spinner(0), isa=ISA)
    assert retires(machine, 16) == []
    assert machine.threads[0].steps == 0


def test_a_taken_branch_costs_exactly_one_slot():
    """No branch penalty: the taken and not-taken paths run at the same rate."""
    taken = [("BZ", dict(rel=1)), ("NOP", {}), ("HALT", {})]

    def setup(machine):
        machine.host_write_debug(0, "FLAGS", 0b001)

    _, records = run_thread(ISA, taken, setup=setup)
    assert [r.pc for r in records] == [0, 2]             # the NOP was skipped
    assert records[0].x_cycle + 4 == records[1].x_cycle

    # The not-taken path costs the same slot, one instruction later.
    program = [("ADDI", dict(rd=0, imm=0)), ("BNZ", dict(rel=-2)), ("HALT", {})]
    _, records = run_thread(ISA, program)
    assert [r.mnemonic for r in records] == ["ADDI", "BNZ", "HALT"]
    gaps = [b.x_cycle - a.x_cycle for a, b in zip(records, records[1:])]
    assert gaps == [4, 4]


def test_a_loop_runs_one_instruction_per_slot_whatever_the_path():
    """Four slots per iteration of a four-instruction loop, taken branch included."""
    program = [("LDI", dict(rd=1, imm=3)),
               ("NOP", {}),
               ("NOP", {}),
               ("DJNZ", dict(rd=1, rel=-3)),
               ("HALT", {})]
    _, records = run_thread(ISA, program)
    assert len(records) == 1 + 3 * 3 + 1
    gaps = {b.x_cycle - a.x_cycle for a, b in zip(records, records[1:])}
    assert gaps == {SLOT_CLOCKS}


def test_two_threads_do_not_disturb_each_others_timing():
    """Thread 0's slot cycles are identical with and without thread 2 running."""
    program = [("LDI", dict(rd=1, imm=4)),
               ("ADDI", dict(rd=1, imm=1)),
               ("DJNZ", dict(rd=1, rel=-2)),
               ("HALT", {})]
    image = assemble(ISA, program, 0)

    machine = Machine(image, isa=ISA)
    machine.host_set_run(0b0001)
    alone = [(cycle, record.pc) for cycle, record in retires(machine, 200)]

    busy_image = dict(image)
    busy_image.update(spinner(2))
    machine = Machine(busy_image, isa=ISA)
    machine.host_set_run(0b0101)
    together = [(cycle, record.pc) for cycle, record in retires(machine, 200)
                if record.thread == 0]

    assert alone == together
    assert len(alone) > 8


def test_a_stalled_wait_re_issues_once_per_slot_and_keeps_its_pc():
    """A wait that cannot complete produces one record per slot with done false."""
    program = [("WAITP", dict(pin=8, val=1)), ("HALT", {})]
    machine = Machine(assemble(ISA, program), isa=ISA)
    machine.host_set_run(0b0001)
    seen = retires(machine, 40)
    stalls = [record for _, record in seen if record.mnemonic == "WAITP"]
    assert len(stalls) >= 6
    for record in stalls:
        assert record.done is False
        assert record.pc == 0
        assert record.next_pc == 0
    gaps = {b.x_cycle - a.x_cycle for a, b in zip(stalls, stalls[1:])}
    assert gaps == {SLOT_CLOCKS}
    assert machine.threads[0].wait_active == 1
    assert machine.threads[0].steps == len(stalls)


def test_steps_counts_every_valid_slot_including_stalls():
    program = [("NOP", {}), ("NOP", {}), ("HALT", {})]
    machine, records = run_thread(ISA, program)
    assert machine.threads[0].steps == 3 == len(records)


def test_the_phase_counter_is_free_running():
    machine = Machine({}, isa=ISA)
    for expected in range(9):
        assert machine.ph == expected % SLOT_CLOCKS
        assert machine.cycle == expected
        machine.step_cycle()


def test_own_state_is_never_stale_across_back_to_back_slots():
    """A register written by one slot is readable by the thread's next slot."""
    program = [("LDI", dict(rd=1, imm=0x10)),
               ("ADDI", dict(rd=1, imm=1)),
               ("MOV", dict(rd=2, ra=1)),
               ("HALT", {})]
    machine, records = run_thread(ISA, program)
    assert records[1].val == 0x11
    assert records[2].val == 0x11
    assert machine.threads[0].regs[2] == 0x11
