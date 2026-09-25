"""Deadline checker tests for tools/loomasm."""

import pytest

from tools.loomasm import assemble_text
from tools.loomasm.deadline import SLOT_CLOCKS
from tools.loomisa import load

ISA = load()


def src(*lines):
    return "\n".join(lines) + "\n"


def asm(text, **kwargs):
    return assemble_text(text, "t.loom", isa=ISA, **kwargs)


def report(text):
    return asm(text).deadlines[0]


def test_slot_is_four_clocks_as_semantics_section_2_says():
    assert SLOT_CLOCKS == int(ISA.meta["slot_clocks"]) == 4


# ------------------------------------------------------------- straight line
def test_straight_line_counts_the_instructions_between_and_the_waitd():
    result = report(src(".thread 0", ".tick 100",
                        "SETD 0", "NOP", "NOP", "WAITD 1", "HALT"))
    assert len(result.pairs) == 1
    pair = result.pairs[0]
    assert pair.src_name == "SETD"
    assert pair.slots == 3                  # NOP, NOP, WAITD
    assert pair.clocks == 12
    assert pair.limit == 100
    assert pair.slack == 88
    assert pair.feasible is True
    assert result.infeasible == [] and result.unbounded == []


def test_waitd_to_waitd_budget_scales_with_the_tick_count():
    result = report(src(".thread 0", ".tick 100",
                        "SETD 0", "WAITD 1", "NOP", "WAITD 3", "HALT"))
    pair = [p for p in result.pairs if p.src_name == "WAITD"][0]
    assert pair.ticks == 3
    assert pair.slots == 2                  # NOP, WAITD
    assert pair.limit == 300 and pair.slack == 292


def test_ld_and_st_cost_two_slots_each():
    """SEMANTICS 6.11 (D-027): a data-memory access is its first slot plus a
    completion slot, so the checker prices LD and ST at two slots."""
    result = report(src(".thread 0", ".tick 100",
                        "SETD 0", "LD r1, r2, 3", "ST r1, r2, 4", "NOP",
                        "WAITD 1", "HALT"))
    pair = result.pairs[0]
    assert pair.slots == 6                  # LD 2, ST 2, NOP, WAITD
    assert pair.clocks == 24


def test_no_anchor_means_nothing_to_check():
    result = report(src(".thread 0", ".tick 100", "NOP", "NOP", "HALT"))
    assert result.pairs == []
    assert any("no WAITD or SETD" in note for note in result.notes)


# ------------------------------------------------ branches of different length
def test_two_branch_paths_take_the_longer_one():
    result = report(src(
        ".thread 0", ".tick 100",
        "        SETD 0",
        "        BZ long",
        "        NOP",
        "        BRA join",
        "long:   NOP",
        "        NOP",
        "        NOP",
        "join:   WAITD 1",
        "        HALT"))
    assert len(result.pairs) == 1
    # short path: BZ NOP BRA WAITD = 4; long path: BZ NOP NOP NOP WAITD = 5
    assert result.pairs[0].slots == 5
    assert result.pairs[0].clocks == 20


def test_the_shorter_path_alone_would_have_been_feasible():
    """The check takes the worst path, not the one the reader had in mind."""
    text = src(
        ".thread 0", ".tick 12",             # budget 12 clocks = 3 slots
        "        SETD 0",
        "        BZ long",
        "        BRA join",
        "long:   NOP",
        "        NOP",
        "join:   WAITD 1",
        "        HALT")
    result = report(text)
    # short path BZ BRA WAITD = 3 slots fits; long path BZ NOP NOP WAITD does not
    assert result.pairs[0].slots == 4
    assert result.pairs[0].clocks == 16 and result.pairs[0].limit == 12
    assert result.pairs[0].feasible is False


# --------------------------------------------------------------------- loops
def test_djnz_loop_that_contains_a_waitd_is_bounded():
    result = report(src(
        ".thread 0", ".tick 100",
        "        LDI r1, 8",
        "        SETD 0",
        "loop:   NOP",
        "        WAITD 1",
        "        DJNZ r1, loop",
        "        HALT"))
    assert result.unbounded == []
    assert result.infeasible == []
    found = {(p.src_name, p.slots) for p in result.pairs}
    assert ("SETD", 2) in found              # NOP, WAITD
    assert ("WAITD", 3) in found             # DJNZ, NOP, WAITD (the loop back edge)


def test_a_loop_with_no_deadline_instruction_is_unbounded():
    result = report(src(
        ".thread 0", ".tick 100",
        "        SETD 0",
        "loop:   NOP",
        "        DJNZ r1, loop",
        "        WAITD 1",
        "        HALT"))
    assert len(result.pairs) == 1
    assert result.pairs[0].unbounded
    assert result.pairs[0].clocks is None
    assert result.pairs[0].feasible is None
    assert len(result.unbounded) == 1


def test_an_unbounded_path_is_a_warning_not_an_error():
    program = asm(src(".thread 0", ".tick 100",
                      "        SETD 0",
                      "loop:   NOP",
                      "        DJNZ r1, loop",
                      "        WAITD 1",
                      "        HALT"))
    assert program.errors == []
    warnings = [d for d in program.warnings if d.kind == "deadline"]
    assert len(warnings) == 1
    assert "unbounded between deadlines" in warnings[0].message
    assert warnings[0].line == 6             # the line of the target WAITD


# ---------------------------------------------------------------- infeasible
def test_an_infeasible_schedule_is_an_error_level_diagnostic():
    text = src(".thread 0", ".tick 4",
               "SETD 0", "NOP", "NOP", "WAITD 1", "HALT")
    program = asm(text)
    result = program.deadlines[0]
    assert result.pairs[0].slots == 3 and result.pairs[0].clocks == 12
    assert result.pairs[0].limit == 4
    assert result.pairs[0].slack == -8
    assert result.pairs[0].feasible is False

    errors = program.deadline_errors
    assert len(errors) == 1
    assert "deadline cannot be met" in errors[0].message
    assert "short by 8" in errors[0].message
    assert errors[0].line == 6               # the WAITD that cannot be reached
    # the image is still produced; only --strict refuses it
    assert len(program.words) == 5


def test_exactly_meeting_the_deadline_is_feasible():
    result = report(src(".thread 0", ".tick 12",
                        "SETD 0", "NOP", "NOP", "WAITD 1", "HALT"))
    assert result.pairs[0].clocks == 12 and result.pairs[0].limit == 12
    assert result.pairs[0].slack == 0 and result.pairs[0].feasible is True


# ------------------------------------------------- unbounded instruction kinds
@pytest.mark.parametrize("instruction", [
    "WAITP IN0, 1", "WAITE IN0, RISE", "WAITS 2", "WAITB INQ_NE",
    "DLY 3", "PUSH r0", "POP r0",
])
def test_untimed_waits_and_fifo_access_are_unbounded(instruction):
    result = report(src(".thread 0", ".tick 100",
                        "SETD 0", instruction, "WAITD 1", "HALT"))
    assert result.pairs[0].unbounded, instruction


@pytest.mark.parametrize("instruction", [
    "WAITP IN0, 1, T", "WAITE IN0, RISE, T", "WAITS 2, T", "WAITB INQ_NE, T",
])
def test_waits_with_the_timeout_bit_are_bounded(instruction):
    result = report(src(".thread 0", ".tick 100",
                        "SETD 0", instruction, "WAITD 1", "HALT"))
    assert not result.pairs[0].unbounded, instruction
    assert result.pairs[0].slots == 2


# ------------------------------------------------------- .bounded (declared)
GUARD = "guarded by the WAITB above; only this thread pops INQ"


def bounded_src(reason=GUARD, instruction="POP r0", declare=True):
    lines = [".thread 0", ".tick 100", "SETD 0"]
    if declare:
        lines.append('.bounded "%s"' % reason)
    lines += [instruction, "WAITD 1", "HALT"]
    return src(*lines)


@pytest.mark.parametrize("instruction", ["POP r0", "PUSH r0"])
def test_a_declared_fifo_access_costs_one_slot_and_the_pair_is_proved(instruction):
    result = report(bounded_src(instruction=instruction))
    pair = result.pairs[0]
    assert not pair.unbounded
    assert pair.slots == 2                  # the POP/PUSH and the WAITD
    assert pair.clocks == 8 and pair.slack == 92 and pair.feasible is True
    assert result.unbounded == [] and result.infeasible == []


@pytest.mark.parametrize("instruction", ["POP r0", "PUSH r0"])
def test_an_undeclared_fifo_access_is_still_unbounded(instruction):
    """Nothing changes for code that declares nothing."""
    result = report(bounded_src(instruction=instruction, declare=False))
    assert result.pairs[0].unbounded
    assert result.declarations == []


def test_a_declaration_is_refused_on_anything_but_push_or_pop():
    """DLY stalls by design, so the directive does not accept it at all."""
    from tools.loomasm import AsmError
    with pytest.raises(AsmError) as info:
        asm(src(".thread 0", ".tick 100", "SETD 0",
                '.bounded "no"', "DLY 3", "WAITD 1", "HALT"))
    messages = [d.message for d in info.value.diagnostics]
    assert any("applies to POP or PUSH" in m and "DLY" in m for m in messages)


def test_the_declaration_is_recorded_with_its_address_line_and_reason():
    result = report(bounded_src())
    assert len(result.declarations) == 1
    declared = result.declarations[0]
    assert (declared.addr, declared.name, declared.line) == (1, "POP", 5)
    assert declared.reason == GUARD
    assert str(declared) == "0x001 POP (line 5): %s" % GUARD


def test_the_summary_says_where_it_trusted_a_declaration():
    from tools.loomasm.deadline import summary_lines
    text = "\n".join(summary_lines(report(bounded_src())))
    assert "bounded by declaration: 0x001 POP (line 5): %s" % GUARD in text
    assert "1 bounded by declaration" in text


def test_the_listing_prints_the_reason_under_the_instruction():
    program = asm(bounded_src())
    body = [row for row in program.listing
            if row.strip().startswith("| bounded by declaration")]
    assert [row.strip() for row in body] == \
        ["| bounded by declaration: %s" % GUARD]
    # and again in the summary block, with the address and the line
    assert any(row.strip() == "bounded by declaration: 0x001 POP (line 5): %s"
               % GUARD for row in program.listing)
    assert program.bounded == {1: GUARD}
    info = [w for w in program.word_info if w.addr == 1][0]
    assert info.bounded == GUARD and info.timing == "blocking"


def test_a_reason_may_hold_a_comma_and_a_semicolon():
    reason = "safe, because: nothing else pops; see SEMANTICS 6.7"
    result = report(bounded_src(reason=reason))
    assert result.declarations[0].reason == reason
    assert not result.pairs[0].unbounded


def test_setd_after_a_blocking_instruction_removes_the_pair():
    """The uart_tx.loom idiom: POP, then SETD re-anchors, so nothing is
    unbounded even though POP can stall for ever."""
    result = report(src(
        ".thread 0", ".tick 100",
        "loop:   WAITD 1",
        "        POP r0",
        "        SETD 0",
        "        BRA loop"))
    assert result.unbounded == []
    assert [(p.src_name, p.slots) for p in result.pairs] == [("SETD", 2)]


# ------------------------------------------------------------- control flow
def test_call_and_ret_are_followed():
    result = report(src(
        ".thread 0", ".tick 100",
        "        SETD 0",
        "        CALL sub",
        "        HALT",
        "sub:    NOP",
        "        WAITD 1",
        "        RET"))
    pair = [p for p in result.pairs if p.src_name == "SETD"][0]
    assert pair.slots == 3                   # CALL, NOP, WAITD


def test_halt_ends_a_path():
    result = report(src(".thread 0", ".tick 100",
                        "SETD 0", "HALT", "NOP", "WAITD 1"))
    assert result.pairs == []


def test_each_thread_is_analysed_separately():
    program = asm(src(
        ".thread 0", ".tick 100", "SETD 0", "NOP", "WAITD 1", "HALT",
        ".thread 1", ".tick 8", "SETD 0", "NOP", "NOP", "WAITD 1", "HALT"))
    assert program.deadlines[0].infeasible == []
    assert len(program.deadlines[1].infeasible) == 1
    assert program.deadlines[0].period == 100
    assert program.deadlines[1].period == 8


# --------------------------------------------------------- period discovery
def test_a_constant_csr_tick_int_declares_the_period():
    result = report(src(".thread 0", ".csr TICK_INT, 434",
                        "SETD 0", "NOP", "WAITD 1", "HALT"))
    assert result.period == 434
    assert result.pairs[0].feasible is True


def test_without_a_period_only_slot_counts_are_reported():
    result = report(src(".thread 0", "SETD 0", "NOP", "WAITD 1", "HALT"))
    assert result.period is None
    assert result.pairs[0].slots == 2
    assert result.pairs[0].limit is None
    assert result.pairs[0].slack is None
    assert result.pairs[0].feasible is None
    assert any("tick period unknown" in note for note in result.notes)


def test_deadline_check_off_suppresses_the_analysis():
    program = asm(src(".thread 0", ".tick 4", ".deadline_check off",
                      "SETD 0", "NOP", "NOP", "WAITD 1", "HALT"))
    assert program.deadlines[0].pairs == []
    assert program.deadline_errors == []
    assert any("disabled" in note for note in program.deadlines[0].notes)


def test_deadline_check_is_per_thread():
    program = asm(src(
        ".thread 0", ".tick 4", ".deadline_check off",
        "SETD 0", "NOP", "NOP", "WAITD 1", "HALT",
        ".thread 1", ".tick 4",
        "SETD 0", "NOP", "NOP", "WAITD 1", "HALT"))
    assert program.deadlines[0].pairs == []
    assert len(program.deadlines[1].infeasible) == 1


# ------------------------------------------- the SETD anchor's own ticks (Q5)
def test_a_setd_anchor_contributes_its_own_ticks_to_the_budget():
    result = report(src(".thread 0", ".tick 10",
                        "SETD 3", "NOP", "NOP", "NOP", "WAITD 1", "HALT"))
    pair = result.pairs[0]
    assert pair.src_ticks == 3 and pair.ticks == 1
    assert pair.budget_ticks == 4                    # m + k
    assert pair.limit == 40
    assert pair.slots == 4 and pair.clocks == 16
    assert pair.slack == 24 and pair.feasible is True


@pytest.mark.parametrize("m,limit,feasible", [(0, 10, False), (2, 30, True)])
def test_the_setd_credit_can_decide_feasibility(m, limit, feasible):
    result = report(src(".thread 0", ".tick 10",
                        "SETD %d" % m, "NOP", "NOP", "NOP", "WAITD 1", "HALT"))
    assert result.pairs[0].slots == 4                # the path never changes
    assert result.pairs[0].limit == limit
    assert result.pairs[0].feasible is feasible


def test_a_waitd_anchor_adds_nothing_of_its_own():
    result = report(src(".thread 0", ".tick 10",
                        "SETD 0", "WAITD 5", "NOP", "WAITD 2", "HALT"))
    from_setd = [p for p in result.pairs if p.src_name == "SETD"][0]
    from_waitd = [p for p in result.pairs if p.src_name == "WAITD"][0]
    assert from_setd.src_ticks == 0 and from_setd.budget_ticks == 5
    assert from_waitd.src_ticks == 0                 # not the 5 of WAITD 5
    assert from_waitd.budget_ticks == 2 and from_waitd.limit == 20


def test_the_infeasible_message_quotes_the_combined_budget():
    program = asm(src(".thread 0", ".tick 10",
                      "SETD 1", "NOP", "NOP", "NOP", "NOP", "NOP", "WAITD 1",
                      "HALT"))
    message = program.deadline_errors[0].message
    assert "6 slots = 24 clocks" in message
    assert "budget 2 x 10 = 20 clocks" in message    # (m + k) x P
    assert "short by 4" in message


def test_a_csrw_td_on_the_path_withdraws_the_setd_credit():
    result = report(src(".thread 0", ".tick 100",
                        "SETD 5", "CSRW TD, r0", "NOP", "WAITD 1", "HALT"))
    pair = result.pairs[0]
    assert pair.src_ticks is None
    assert pair.anchor_credit_known is False
    assert pair.budget_ticks == 1 and pair.limit == 100      # k * P, not 6 * P
    assert any("CSRW TD" in note for note in result.notes)


def test_the_csr_directive_writing_td_withdraws_the_credit_too():
    result = report(src(".thread 0", ".tick 100",
                        "SETD 5", ".csr TD, 50", "WAITD 1", "HALT"))
    assert result.pairs[0].src_ticks is None
    assert result.pairs[0].budget_ticks == 1


def test_a_csrw_to_another_csr_does_not_withdraw_the_credit():
    result = report(src(".thread 0", ".tick 100",
                        "SETD 5", "CSRW OUTGRP, r0", "WAITD 1", "HALT"))
    assert result.pairs[0].src_ticks == 5
    assert result.pairs[0].budget_ticks == 6


def test_the_listing_marks_a_withdrawn_credit_and_shows_the_budget_ticks():
    from tools.loomasm.deadline import summary_lines
    result = report(src(".thread 0", ".tick 100",
                        "SETD 5", "CSRW TD, r0", "WAITD 1", "HALT"))
    text = "\n".join(summary_lines(result))
    assert "TD rewritten" in text
    assert "ticks" in text                           # the budget-ticks column


# ------------------------------------------------ WAITD 0 is transparent (Q6)
def test_waitd_0_is_one_slot_and_neither_starts_nor_ends_an_interval():
    result = report(src(".thread 0", ".tick 100",
                        "SETD 0", "NOP", "WAITD 0", "NOP", "WAITD 1", "HALT"))
    assert len(result.pairs) == 1
    pair = result.pairs[0]
    assert pair.src_name == "SETD" and pair.src_addr == 0
    assert pair.dst_addr == 4                        # the WAITD 1, not WAITD 0
    assert pair.slots == 4                           # NOP, WAITD 0, NOP, WAITD 1
    assert not any(p.src_addr == 2 or p.dst_addr == 2 for p in result.pairs)


def test_waitd_0_is_never_itself_reported_as_infeasible():
    program = asm(src(".thread 0", ".tick 4",
                      "SETD 0", "NOP", "NOP", "WAITD 0", "HALT"))
    assert program.deadlines[0].pairs == []
    assert program.deadline_errors == []
    # the same shape with WAITD 1 is infeasible, so the exemption is doing work
    worse = asm(src(".thread 0", ".tick 4",
                    "SETD 0", "NOP", "NOP", "WAITD 1", "HALT"))
    assert len(worse.deadlines[0].infeasible) == 1


def test_waitd_0_does_not_anchor_a_loop_out_of_unboundedness():
    result = report(src(".thread 0", ".tick 100",
                        "        SETD 0",
                        "loop:   WAITD 0",
                        "        DJNZ r1, loop",
                        "        WAITD 1",
                        "        HALT"))
    assert len(result.pairs) == 1
    assert result.pairs[0].unbounded


def test_waitd_0_still_costs_its_slot_against_a_real_deadline():
    tight = report(src(".thread 0", ".tick 12",
                       "SETD 0", "WAITD 0", "NOP", "WAITD 1", "HALT"))
    assert tight.pairs[0].slots == 3 and tight.pairs[0].clocks == 12
    assert tight.pairs[0].feasible is True
    short = report(src(".thread 0", ".tick 8",
                       "SETD 0", "WAITD 0", "NOP", "WAITD 1", "HALT"))
    assert short.pairs[0].feasible is False


# ------------------------------------------------------------------ summary
def test_the_summary_block_lists_every_pair_and_the_worst_slack():
    result = report(src(".thread 0", ".tick 100",
                        "SETD 0", "NOP", "WAITD 1", "NOP", "NOP", "WAITD 1",
                        "HALT"))
    from tools.loomasm.deadline import summary_lines
    text = "\n".join(summary_lines(result))
    assert "thread 0 deadline analysis (tick period 100 clocks)" in text
    assert "2 deadline pairs, 0 unbounded, 0 infeasible" in text
    assert "worst slack 88 clocks" in text
    assert result.worst_slack == 88
    assert result.worst_slots == 3


def test_the_summary_marks_a_missed_deadline():
    from tools.loomasm.deadline import summary_lines
    result = report(src(".thread 0", ".tick 4",
                        "SETD 0", "NOP", "NOP", "WAITD 1", "HALT"))
    text = "\n".join(summary_lines(result))
    assert "MISSED" in text
    assert "1 infeasible" in text


def test_the_summary_marks_an_unbounded_pair():
    from tools.loomasm.deadline import summary_lines
    result = report(src(".thread 0", ".tick 100",
                        "SETD 0", "DLY 1", "WAITD 1", "HALT"))
    text = "\n".join(summary_lines(result))
    assert "UNBOUNDED" in text
    assert "1 unbounded" in text


# --------------------------------------------- the SETD's phase (T-1, sound)
# docs/VERIFICATION.md tools finding T-1: a SETD reads NOW up to P - 1 clocks
# after the tick that set it, so with sound_setd the phase comes off the budget
# of each pair it starts, and the target's slot-grid grace (3 clocks) goes on.

def sound(text):
    return asm(text, sound_setd=True).deadlines[0]


def test_default_is_still_the_old_optimistic_rule():
    pair = report(src(".thread 0", ".tick 100",
                      "SETD 0", "NOP", "WAITD 1", "HALT")).pairs[0]
    assert pair.src_phase is None and pair.limit == 100


def test_a_setd_at_the_thread_entry_has_an_unknown_phase():
    pair = sound(src(".thread 0", ".tick 100",
                     "SETD 0", "NOP", "WAITD 1", "HALT")).pairs[0]
    assert pair.src_phase == 99                        # P - 1
    assert pair.limit == 100 - 99 + 3                  # 4 clocks: one slot
    assert pair.feasible is False                      # NOP, WAITD: 8 clocks


def test_setd_one_gives_the_whole_tick_back():
    pair = sound(src(".thread 0", ".tick 100",
                     "SETD 1", "NOP", "WAITD 1", "HALT")).pairs[0]
    assert pair.limit == 200 - 99 + 3 and pair.feasible is True


def test_a_setd_right_after_a_waitd_is_at_most_seven_clocks_into_its_tick():
    result = sound(src(".thread 0", ".tick 100",
                       "SETD 1", "WAITD 1", "SETD 0", "NOP", "WAITD 1", "HALT"))
    pair = [p for p in result.pairs if p.src_addr == 2][0]
    assert pair.src_phase == 3 + 4                     # WAITD lag + one slot
    assert pair.limit == 100 - 7 + 3


def test_a_tick_restart_fixes_the_phase_two_clocks_before_the_next_slot():
    result = sound(src(".thread 0", ".tick 100",
                       "SETD 1", "WAITP IN0, 1, T", "CSRW TICK_INT, r7",
                       "SETD 0", "NOP", "WAITD 1", "HALT"))
    pair = [p for p in result.pairs if p.src_addr == 3][0]
    assert pair.src_phase == 4 - 2


def test_a_timed_wait_or_a_pop_before_the_setd_makes_the_phase_unknown():
    for waiter in ("WAITP IN0, 1, T", "POP r0"):
        result = sound(src(".thread 0", ".tick 100",
                           "SETD 1", "WAITD 1", waiter,
                           "SETD 0", "NOP", "WAITD 1", "HALT"))
        pair = [p for p in result.pairs if p.src_addr == 3][0]
        assert pair.src_phase == 99, waiter


def test_a_whole_period_without_a_known_position_makes_the_phase_unknown():
    body = ["NOP"] * 30                                # 120 clocks > P = 100
    result = sound(src(".thread 0", ".tick 100", "SETD 1", "WAITD 1",
                       *body, "SETD 0", "WAITD 1", "HALT"))
    pair = [p for p in result.pairs if p.src_addr == 32][0]
    assert pair.src_phase == 99


def test_the_phase_is_the_worst_over_every_path_into_the_setd():
    result = sound(src(".thread 0", ".tick 100",
                       "SETD 1", "WAITD 1",
                       "BZ far",                        # 2: one path skips ahead
                       "NOP", "NOP", "NOP",
                       "far: SETD 0", "WAITD 1", "HALT"))
    pair = [p for p in result.pairs if p.src_name == "SETD" and p.src_addr == 6][0]
    # the long way: the SETD's X cycle is five slots after the WAITD's
    # (BZ, three NOPs, then its own); the short way, two
    assert pair.src_phase == 3 + 4 * 5


def test_waitd_pairs_do_not_change():
    text = src(".thread 0", ".tick 33",
               "SETD 1", "WAITD 1", "NOP", "WAITD 1", "HALT")
    old = [p for p in report(text).pairs if p.src_name == "WAITD"][0]
    new = [p for p in sound(text).pairs if p.src_name == "WAITD"][0]
    assert (old.limit, old.slack) == (new.limit, new.slack) == (33, 25)


def test_the_error_names_the_phase():
    program = asm(src(".thread 0", ".tick 100",
                      "SETD 0", "NOP", "WAITD 1", "HALT"), sound_setd=True)
    messages = [d.message for d in program.errors]
    assert any("1 x 100 - 99 (SETD phase) + 3 = 4 clocks" in m for m in messages)
