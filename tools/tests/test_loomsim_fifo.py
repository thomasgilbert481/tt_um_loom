"""FIFOs, SEMANTICS 6.7, on the golden model (feature ``"FIFO"``).

``FIFO_DEPTH`` is a power of two from 2 to 8; counts change at an edge and are
visible from the next cycle; a host push to a full INQ and a host pop of an
empty OUTQ are dropped and set ``BADOP[14]``; ``CTRL.RESET`` empties both
queues of the thread; a thread push or pop committing at the same edge as a
host pop or push is applied too (``count + pushes - pops``), while the host
push is still judged on the count from before the edge.

Timing reference: with ``RUN`` written in cycle 0, thread 0's slots have X
cycles 6, 10, 14, ... and commit at X + 2, so a host action issued in cycle
X + 1 (the slot's W cycle) lands at the same edge as the slot's commit.
"""

import pytest

from tools.loomisa import load
from tools.loomsim import BADOP_FIFO, LoomsimError, Machine
from tools.loomsim.harness import assemble

ISA = load()


def fifo_machine(program, depth=4, thread=0, features=("FIFO",), **kwargs):
    kwargs.setdefault("isa", ISA)
    return Machine(assemble(ISA, program, thread * 0x100), features=features,
                   fifo_depth=depth, **kwargs)


def run_records(machine, cycles, thread=None):
    out = []
    for _ in range(cycles):
        record = machine.step_cycle()
        if record is not None and (thread is None or record.thread == thread):
            out.append(record)
    return out


def step_to(machine, cycle):
    """Run until the start of ``cycle``; return the records produced on the way."""
    out = []
    while machine.cycle < cycle:
        record = machine.step_cycle()
        if record is not None:
            out.append(record)
    return out


# ----------------------------------------------------------- the depth
@pytest.mark.parametrize("depth,log2", [(2, 1), (4, 2), (8, 3)])
def test_caps_reports_the_depth(depth, log2):
    machine = fifo_machine([("HALT", {})], depth=depth, imem_words=256)
    assert machine.caps == 0x8000 | (1 << 3) | log2
    assert machine.host_read_ctrl("CAPS") == machine.caps


@pytest.mark.parametrize("depth", [1, 3, 6, 16, 0])
def test_other_depths_are_refused(depth):
    with pytest.raises(LoomsimError):
        fifo_machine([("HALT", {})], depth=depth)


@pytest.mark.parametrize("depth", [2, 8])
def test_a_thread_push_fills_exactly_depth_entries_then_stalls(depth):
    machine = fifo_machine([("PUSH", dict(ra=1)), ("ADDI", dict(rd=1, imm=1)),
                            ("JMP", dict(abs=0))], depth=depth)
    machine.host_set_run(0b0001)
    records = run_records(machine, 60 + 12 * depth)
    assert machine.threads[0].outq == list(range(depth))
    assert machine.host_fifo_status(0)["outq"] == depth
    assert machine.threads[0].wait_active == 1
    stalls = [r for r in records if r.mnemonic == "PUSH" and not r.done]
    assert stalls and all(r.pc == 0 and r.next_pc == 0 for r in stalls)
    assert machine.badop == 0


@pytest.mark.parametrize("depth", [2, 8])
def test_host_pushes_fill_inq_and_the_next_one_sets_badop_14(depth):
    machine = fifo_machine([("HALT", {})], depth=depth)
    for word in range(depth):
        machine.host_fifo_push(0, 0x100 + word)
        machine.step_cycle()
    assert machine.threads[0].inq == [0x100 + w for w in range(depth)]
    assert machine.badop == 0
    machine.host_fifo_push(0, 0xDEAD)
    assert machine.badop == 0                          # set at the edge, not before
    machine.step_cycle()
    assert machine.badop == BADOP_FIFO == 1 << 14
    assert machine.threads[0].inq == [0x100 + w for w in range(depth)]   # dropped
    assert machine.host_read_ctrl("BADOP") == 1 << 14


def test_a_host_pop_of_an_empty_outq_returns_zero_and_sets_badop_14_at_the_edge():
    machine = fifo_machine([("HALT", {})])
    assert machine.host_fifo_pop(2) == 0
    assert machine.badop == 0
    machine.step_cycle()
    assert machine.badop == 1 << 14
    assert machine.threads[2].outq == []


def test_host_fifo_error_flags_without_popping():
    """The port's pop half when the peek was empty but a PUSH filled the queue
    in between (SEMANTICS 6.7): the error is set, the entry stays."""
    machine = fifo_machine([("HALT", {})])
    machine.threads[1].outq.append(0xBEEF)
    machine.host_fifo_error()
    assert machine.badop == 0                          # set at the edge, not before
    machine.step_cycle()
    assert machine.badop == 1 << 14
    assert machine.threads[1].outq == [0xBEEF]


def test_badop_14_is_write_one_to_clear():
    machine = fifo_machine([("HALT", {})])
    machine.host_fifo_pop(0)
    machine.step_cycle()
    machine.host_write_ctrl("BADOP", 0x000F)           # other bits: no effect
    machine.step_cycle()
    assert machine.badop == 1 << 14
    machine.host_clear_badop(1 << 14)
    machine.step_cycle()
    assert machine.badop == 0


# ------------------------------------------------------ peek and atomic pop
def test_host_fifo_peek_has_no_side_effects():
    machine = fifo_machine([("LDI", dict(rd=1, imm=0x41)), ("PUSH", dict(ra=1)),
                            ("LDI", dict(rd=1, imm=0x42)), ("PUSH", dict(ra=1)),
                            ("HALT", {})])
    assert machine.host_fifo_peek(0) is None           # empty: None, no BADOP
    machine.host_set_run(0b0001)
    machine.run_cycles(40)
    for _ in range(5):
        assert machine.host_fifo_peek(0) == 0x41
        machine.step_cycle()
    assert machine.threads[0].outq == [0x41, 0x42]
    assert machine.badop == 0
    assert machine.host_fifo_peek(1) is None
    machine.step_cycle()
    assert machine.badop == 0


def test_a_host_pop_returns_the_head_and_removes_it_at_the_edge():
    machine = fifo_machine([("LDI", dict(rd=1, imm=0x41)), ("PUSH", dict(ra=1)),
                            ("LDI", dict(rd=1, imm=0x42)), ("PUSH", dict(ra=1)),
                            ("HALT", {})])
    machine.host_set_run(0b0001)
    machine.run_cycles(40)
    assert machine.host_fifo_pop(0) == 0x41
    assert machine.host_fifo_status(0)["outq"] == 2    # not yet
    machine.step_cycle()
    assert machine.host_fifo_status(0)["outq"] == 1
    assert machine.host_fifo_peek(0) == 0x42
    assert machine.host_fifo_pop(0) == 0x42
    machine.step_cycle()
    assert machine.host_fifo_peek(0) is None and machine.badop == 0


def test_peek_then_pop_is_how_a_transport_reads_several_words():
    """The SPI split: peek when a word is loaded, pop when it has gone out."""
    machine = fifo_machine([("HALT", {})])
    words = []
    machine.threads[0].outq.extend([7, 8, 9])          # back door, for brevity
    for _ in range(3):
        word = machine.host_fifo_peek(0)
        machine.run_cycles(16)                         # the word shifts out
        assert machine.host_fifo_pop(0) == word
        words.append(word)
        machine.step_cycle()
    assert words == [7, 8, 9] and machine.badop == 0


# ------------------------------------------------------ counts and visibility
def test_a_host_push_is_visible_from_the_next_cycle():
    machine = fifo_machine([("HALT", {})])
    machine.host_fifo_push(1, 5)
    assert machine.host_fifo_status(1)["inq"] == 0
    assert machine.host_read_debug(1, 0x26) == 0
    machine.step_cycle()
    assert machine.host_fifo_status(1)["inq"] == 1
    assert machine.host_read_debug(1, 0x26) == 0x0100


def test_a_thread_pop_is_visible_from_its_commit_edge():
    machine = fifo_machine([("POP", dict(rd=1)), ("HALT", {})])
    machine.host_fifo_push(0, 0x77)
    machine.host_set_run(0b0001)
    counts = {}
    records = []
    for _ in range(20):
        counts[machine.cycle] = len(machine.threads[0].inq)
        record = machine.step_cycle()
        if record is not None:
            records.append(record)
    pop = [r for r in records if r.mnemonic == "POP"][0]
    assert pop.done and pop.val == 0x77
    assert counts[pop.x_cycle + 1] == 1
    assert counts[pop.x_cycle + 2] == 0


@pytest.mark.parametrize("push_cycle,pop_x", [(9, 10), (10, 14)])
def test_a_pop_decides_on_the_count_visible_in_its_x_cycle(push_cycle, pop_x):
    machine = fifo_machine([("POP", dict(rd=1)), ("HALT", {})])
    machine.host_set_run(0b0001)
    step_to(machine, push_cycle)
    machine.host_fifo_push(0, 0x99)
    records = run_records(machine, 12, thread=0)
    done = [r for r in records if r.mnemonic == "POP" and r.done]
    assert [r.x_cycle for r in done] == [pop_x]


def test_push_and_pop_use_wait_active_and_change_no_flags():
    machine = fifo_machine([("POP", dict(rd=2)), ("PUSH", dict(ra=2)), ("HALT", {})])
    machine.host_write_debug(0, "FLAGS", 0b111)
    machine.host_set_run(0b0001)
    records = run_records(machine, 16)
    assert all(r.flags == 0b111 for r in records)
    assert machine.threads[0].wait_active == 1
    machine.host_fifo_push(0, 0x1234)
    records = run_records(machine, 20)
    assert [r.mnemonic for r in records if r.done] == ["POP", "PUSH", "HALT"]
    assert all(r.flags == 0b111 for r in records)
    assert machine.threads[0].wait_active == 0
    assert machine.threads[0].outq == [0x1234]


# ------------------------------------------------------- same-edge rules
def test_a_host_push_and_a_thread_pop_at_the_same_edge_are_both_applied():
    machine = fifo_machine([("POP", dict(rd=1)), ("HALT", {})])
    machine.host_fifo_push(0, 0xAAAA)
    machine.host_set_run(0b0001)
    step_to(machine, 7)                                # the POP (X 6) is in W
    machine.host_fifo_push(0, 0xBBBB)
    record = machine.step_cycle()
    assert record.mnemonic == "POP" and record.done and record.val == 0xAAAA
    assert machine.threads[0].inq == [0xBBBB]          # 1 + 1 - 1
    assert machine.badop == 0


def test_a_host_push_to_a_full_inq_loses_even_if_a_thread_pop_lands_at_that_edge():
    """Accepted iff INQ_CNT < FIFO_DEPTH as visible in the cycle before the edge."""
    machine = fifo_machine([("POP", dict(rd=1)), ("HALT", {})], depth=2)
    machine.host_fifo_push(0, 1)
    machine.step_cycle()
    machine.host_fifo_push(0, 2)
    machine.host_set_run(0b0001)
    step_to(machine, 7)
    assert len(machine.threads[0].inq) == 2
    machine.host_fifo_push(0, 3)
    record = machine.step_cycle()
    assert record.mnemonic == "POP" and record.val == 1
    assert machine.threads[0].inq == [2]               # 2 + 0 - 1
    assert machine.badop == 1 << 14


def test_a_host_pop_and_a_thread_push_at_the_same_edge_are_both_applied():
    machine = fifo_machine([("LDI", dict(rd=1, imm=0x11)), ("PUSH", dict(ra=1)),
                            ("LDI", dict(rd=1, imm=0x22)), ("PUSH", dict(ra=1)),
                            ("HALT", {})])
    machine.host_set_run(0b0001)
    step_to(machine, 19)                               # second PUSH (X 18) in W
    assert machine.threads[0].outq == [0x11]
    assert machine.host_fifo_pop(0) == 0x11
    record = machine.step_cycle()
    assert record.mnemonic == "PUSH" and record.x_cycle == 18
    assert machine.threads[0].outq == [0x22]           # 1 + 1 - 1
    assert machine.badop == 0


def test_a_host_pop_of_an_empty_outq_at_the_edge_of_a_thread_push():
    machine = fifo_machine([("LDI", dict(rd=1, imm=0x33)), ("PUSH", dict(ra=1)),
                            ("HALT", {})])
    machine.host_set_run(0b0001)
    step_to(machine, 11)                               # the PUSH (X 10) is in W
    assert machine.host_fifo_pop(0) == 0               # empty in this cycle
    record = machine.step_cycle()
    assert record.mnemonic == "PUSH"
    assert machine.threads[0].outq == [0x33]           # 0 + 1 - 0
    assert machine.badop == 1 << 14


def test_a_push_that_saw_a_full_queue_in_x_stalls_even_if_the_host_pops_meanwhile():
    machine = fifo_machine([("PUSH", dict(ra=1)), ("PUSH", dict(ra=1)),
                            ("PUSH", dict(ra=1)), ("HALT", {})], depth=2)
    machine.host_set_run(0b0001)
    records = step_to(machine, 15)                     # third PUSH (X 14) is in W
    assert len(machine.threads[0].outq) == 2
    machine.host_fifo_pop(0)
    records += run_records(machine, 8)
    pushes = [(r.x_cycle, r.done) for r in records if r.mnemonic == "PUSH"]
    assert pushes == [(6, True), (10, True), (14, False), (18, True)]
    assert len(machine.threads[0].outq) == 2


# --------------------------------------------------------------- CTRL.RESET
def test_ctrl_reset_empties_both_fifos_of_that_thread_only():
    machine = fifo_machine([("PUSH", dict(ra=0)), ("PUSH", dict(ra=0)),
                            ("PUSH", dict(ra=0)), ("HALT", {})])
    machine.host_set_run(0b0001)
    machine.run_cycles(30)
    machine.host_fifo_push(0, 1)
    machine.host_fifo_push(1, 2)
    machine.step_cycle()
    machine.host_fifo_push(0, 3)
    machine.step_cycle()
    assert (len(machine.threads[0].inq), len(machine.threads[0].outq)) == (2, 3)
    machine.host_write_ctrl("RESET", 0b0001)
    assert machine.host_read_debug(0, 0x26) == 0x0203  # until the edge
    machine.step_cycle()
    assert machine.host_read_debug(0, 0x26) == 0
    assert machine.host_fifo_status(0) == {"inq": 0, "outq": 0, "depth": 4}
    assert machine.threads[1].inq == [2]               # other threads untouched
    assert machine.irq_stat & 0x11 == 0x10             # INQ[0] not full, OUTQ[0] empty
    assert machine.host_fifo_pop(0) == 0
    machine.step_cycle()
    assert machine.badop == 1 << 14


# ------------------------------------------------------ status and debug
def test_the_status_word_layout():
    machine = fifo_machine([("PUSH", dict(ra=0)), ("PUSH", dict(ra=0)), ("HALT", {})])
    assert machine.host_fifo_status_word(0) == 0b1010  # OUTQ_EMPTY, INQ_EMPTY
    for word in range(4):
        machine.host_fifo_push(0, word)
        machine.step_cycle()
    assert machine.host_fifo_status_word(0) == (4 << 4) | 0b1001   # INQ_FULL
    machine.host_set_run(0b0001)
    machine.run_cycles(30)
    assert machine.host_fifo_status_word(0) == (2 << 8) | (4 << 4) | 0b0001


def test_debug_0x26_is_read_only():
    machine = fifo_machine([("HALT", {})], depth=8)
    for word in range(8):
        machine.host_fifo_push(3, word)
        machine.step_cycle()
    assert machine.host_read_debug(3, 0x26) == 0x0800
    assert (machine.host_read_debug(3, "INQ_CNT"), machine.host_read_debug(3, "OUTQ_CNT")) == (8, 0)
    machine.host_write_debug(3, 0x26, 0)
    machine.step_cycle()
    assert machine.host_read_debug(3, "FIFO_CNT") == 0x0800
    with pytest.raises(LoomsimError):
        machine.host_write_debug(3, "FIFO_CNT", 0)


def test_the_host_fifo_calls_need_the_feature():
    machine = Machine({}, isa=ISA)
    for call in (lambda: machine.host_fifo_peek(0), lambda: machine.host_fifo_pop(0),
                 lambda: machine.host_fifo_status_word(0)):
        with pytest.raises(LoomsimError):
            call()


@pytest.mark.parametrize("depth", [2, 8])
def test_an_echo_thread_round_trips_a_full_queue(depth):
    machine = fifo_machine([("POP", dict(rd=0)), ("ADDI", dict(rd=0, imm=1)),
                            ("PUSH", dict(ra=0)), ("JMP", dict(abs=0))], depth=depth)
    for word in range(depth):
        machine.host_fifo_push(0, 10 * word)
        machine.step_cycle()
    machine.host_set_run(0b0001)
    machine.run_cycles(20 * depth)
    got = []
    while machine.host_fifo_peek(0) is not None:
        got.append(machine.host_fifo_pop(0))
        machine.step_cycle()
    assert got == [10 * w + 1 for w in range(depth)]
    assert machine.badop == 0


# -------------------------------------------------------------------- WAITB
def test_waitb_two_waits_for_inq_and_waitb_one_for_room_in_outq():
    program = [("WAITB", dict(cond=2)), ("PUSH", dict(ra=0)), ("PUSH", dict(ra=0)),
               ("WAITB", dict(cond=1)), ("HALT", {})]
    machine = fifo_machine(program, depth=2)
    machine.host_set_run(0b0001)
    machine.run_cycles(24)
    assert machine.threads[0].pc == 0                  # INQ empty
    machine.host_fifo_push(0, 1)
    machine.run_cycles(24)
    assert machine.threads[0].pc == 3                  # OUTQ full
    assert machine.threads[0].inq == [1]               # WAITB does not pop
    machine.host_fifo_pop(0)
    machine.run_cycles(12)
    assert machine.halted == 0b0001 and machine.badop == 0


@pytest.mark.parametrize("features,badop", [
    (("FIFO",), 1),                   # condition 0 needs the bit engine as well
    (("BE",), 1),                     # WAITB itself is built with the FIFOs
    (("FIFO", "BE"), 0),              # built: the engine is always idle at M2
])
def test_waitb_zero_needs_fifos_and_the_bit_engine(features, badop):
    machine = fifo_machine([("WAITB", dict(cond=0, tmo=1)), ("HALT", {})],
                           features=features)
    machine.host_write_debug(0, "FLAGS", 0b100)        # T set beforehand
    machine.host_write_debug(0, "TD", 0x8000)          # the timeout is far away
    machine.host_set_run(0b0001)
    records = run_records(machine, 20)
    assert machine.badop == badop
    assert records[0].done is True and len(records) == 2
    # Built, the condition ends the wait and clears T; unbuilt it is a NOP.
    assert records[0].t == (0 if not badop else 1)


@pytest.mark.parametrize("cond", [1, 2, 3])
def test_waitb_one_to_three_are_badop_nops_with_only_the_bit_engine(cond):
    machine = fifo_machine([("WAITB", dict(cond=cond)), ("HALT", {})], features=("BE",))
    machine.host_set_run(0b0001)
    run_records(machine, 20)
    assert machine.badop == 1 and machine.halted == 1
