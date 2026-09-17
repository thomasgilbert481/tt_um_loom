"""Every [M1] instruction on the golden model: result, flags, PC, slot count.

Check IDs: L2-DIR (one directed case per mnemonic), L2-SLOT (exact slot count),
ISA-1 (reserved words execute as NOP and set BADOP).  Every program here is
built with ``tools.loomisa.encode``; no instruction word is written literally.
"""

import pytest

from tools.loomisa import load
from tools.loomsim import Machine
from tools.loomsim.harness import assemble, run_thread

ISA = load()


def flags(z=0, c=0, t=0):
    """Pack ``{T, C, Z}`` the way the retire record reports it."""
    return (t << 2) | (c << 1) | z


def run_case(name, operands, regs=None, setup=None, extra=None, **kwargs):
    """Run ``name`` at PC 0 followed by HALT; return (machine, records)."""
    program = [(name, operands), ("HALT", {})]
    if extra is not None:
        program = [(name, operands)]

    def do_setup(machine):
        for index, value in (regs or {}).items():
            machine.host_write_debug(0, "r%d" % index, value)
        if setup is not None:
            setup(machine)

    return run_thread(ISA, program, setup=do_setup, extra=extra, **kwargs)


HALT_AT_5 = {5: ISA.encode("HALT")}

# name, operands, preset registers, expectations.
# Expectations: val/rd = register write (val None means no write), f = flags
# after the slot, pc = next_pc of the slot, slots = valid slots to the halt.
CASES = [
    # ---- ALU, three operand (SEMANTICS 6.1)
    ("ADD", dict(rd=1, ra=2, rb=3), {2: 0x1234, 3: 0x0001},
     dict(rd=1, val=0x1235, f=flags())),
    ("ADD", dict(rd=1, ra=2, rb=3), {2: 0xFFFF, 3: 0x0001},
     dict(rd=1, val=0x0000, f=flags(z=1, c=1))),
    ("SUB", dict(rd=1, ra=2, rb=3), {2: 0x0005, 3: 0x0003},
     dict(rd=1, val=0x0002, f=flags(c=1))),
    ("SUB", dict(rd=1, ra=2, rb=3), {2: 0x0003, 3: 0x0005},
     dict(rd=1, val=0xFFFE, f=flags(c=0))),
    ("AND", dict(rd=1, ra=2, rb=3), {2: 0xF0F0, 3: 0x0FF0},
     dict(rd=1, val=0x00F0, f=flags())),
    ("OR", dict(rd=1, ra=2, rb=3), {2: 0xF000, 3: 0x000F},
     dict(rd=1, val=0xF00F, f=flags())),
    ("XOR", dict(rd=1, ra=2, rb=3), {2: 0xFFFF, 3: 0xFFFF},
     dict(rd=1, val=0x0000, f=flags(z=1))),
    ("SHL", dict(rd=1, ra=2, rb=3), {2: 0x8001, 3: 1},
     dict(rd=1, val=0x0002, f=flags(c=1))),
    ("SHR", dict(rd=1, ra=2, rb=3), {2: 0x8001, 3: 1},
     dict(rd=1, val=0x4000, f=flags(c=1))),
    ("ROR", dict(rd=1, ra=2, rb=3), {2: 0x0001, 3: 1},
     dict(rd=1, val=0x8000, f=flags(c=1))),
    # ---- ALU immediate
    ("ADDI", dict(rd=1, imm=3), {1: 0x0004}, dict(rd=1, val=0x0007, f=flags())),
    ("SUBI", dict(rd=1, imm=4), {1: 0x0004}, dict(rd=1, val=0x0000, f=flags(z=1, c=1))),
    ("ANDI", dict(rd=1, imm=0x0F), {1: 0xFFFF}, dict(rd=1, val=0x000F, f=flags())),
    ("ORI", dict(rd=1, imm=0x0F), {1: 0xF000}, dict(rd=1, val=0xF00F, f=flags())),
    ("XORI", dict(rd=1, imm=0x3F), {1: 0x003F}, dict(rd=1, val=0x0000, f=flags(z=1))),
    ("SHLI", dict(rd=1, imm=4), {1: 0x1001}, dict(rd=1, val=0x0010, f=flags(c=1))),
    ("SHRI", dict(rd=1, imm=4), {1: 0x1008}, dict(rd=1, val=0x0100, f=flags(c=1))),
    ("CMPI", dict(rd=1, imm=7), {1: 0x0007}, dict(val=None, f=flags(z=1, c=1))),
    # ---- immediates
    ("LDI", dict(rd=3, imm=0x41), {3: 0xFFFF}, dict(rd=3, val=0x0041, f=flags())),
    ("LDIH", dict(rd=3, imm=0x41), {3: 0x0042}, dict(rd=3, val=0x4142, f=flags())),
    # ---- unary
    ("MOV", dict(rd=1, ra=2), {2: 0x0000}, dict(rd=1, val=0x0000, f=flags())),
    ("NOT", dict(rd=1, ra=2), {2: 0x0F0F}, dict(rd=1, val=0xF0F0, f=flags())),
    ("NEG", dict(rd=1, ra=2), {2: 0x0001}, dict(rd=1, val=0xFFFF, f=flags())),
    ("NEG", dict(rd=1, ra=2), {2: 0x0000}, dict(rd=1, val=0x0000, f=flags(z=1, c=1))),
    ("CMP", dict(rd=1, ra=2), {1: 0x0002, 2: 0x0002}, dict(val=None, f=flags(z=1, c=1))),
    ("TEST", dict(rd=1, ra=2), {1: 0x00F0, 2: 0x000F}, dict(val=None, f=flags(z=1))),
    ("REV", dict(rd=1, ra=2), {2: 0x0001}, dict(rd=1, val=0x8000, f=flags())),
    ("PAR", dict(rd=1, ra=2), {2: 0x0007}, dict(rd=1, val=0x0007, f=flags(c=1))),
    ("SWAP", dict(rd=1, ra=2), {2: 0x1234}, dict(rd=1, val=0x3412, f=flags())),
    # ---- pins (the pad effects are checked in test_loomsim_pins.py)
    ("SETP", dict(pin=16, val=1), {}, dict(val=None, f=flags())),
    ("OEP", dict(pin=3, val=1), {}, dict(val=None, f=flags())),
    ("OUT", dict(ra=1), {1: 0x0000}, dict(val=None, f=flags())),
    ("IN", dict(rd=1), {1: 0xFFFF}, dict(rd=1, val=0x0000, f=flags())),
    # ---- waits that complete on their first issue
    ("WAITD", dict(imm=1), {}, dict(val=None, f=flags())),
    ("WAITP", dict(pin=8, val=0), {}, dict(val=None, f=flags())),
    ("DLY", dict(imm=0), {}, dict(val=None, f=flags())),
    ("SETD", dict(imm=4), {}, dict(val=None, f=flags())),
    ("NOP", dict(), {}, dict(val=None, f=flags())),
    # ---- shared flags
    ("SIG", dict(flag=2), {}, dict(val=None, f=flags())),
    ("CLR", dict(flag=2), {}, dict(val=None, f=flags())),
    # ---- CSR
    ("CSRR", dict(rd=1, csr=0x0C), {1: 0xFFFF}, dict(rd=1, val=0x0000, f=flags())),
    ("CSRW", dict(csr=0x00, ra=1), {1: 434}, dict(val=None, f=flags())),
]


@pytest.mark.parametrize("name,operands,regs,expect", CASES,
                         ids=["%s-%d" % (case[0], n) for n, case in enumerate(CASES)])
def test_single_instruction(name, operands, regs, expect):
    machine, records = run_case(name, operands, regs)
    record = records[0]
    assert record.mnemonic == name
    assert record.done is True
    assert record.pc == 0
    assert record.ir == ISA.encode(name, **operands)
    if expect["val"] is None:
        assert record.we is False
    else:
        assert record.we is True
        assert record.rd == expect["rd"]
        assert record.val == expect["val"]
        assert machine.threads[0].regs[expect["rd"]] == expect["val"]
    assert record.flags == expect["f"]
    assert record.next_pc == expect.get("pc", 1)
    assert len(records) == expect.get("slots", 2)


def test_waite_times_out_in_one_slot():
    """WAITE with T=1 and a deadline already reached ends in one slot, T set."""
    machine, records = run_case("WAITE", dict(pin=8, edge=0, tmo=1))
    assert records[0].done is True
    assert records[0].flags == flags(t=1)
    assert records[0].next_pc == 1
    assert len(records) == 2


def test_waits_completes_when_the_host_has_set_the_flag():
    def setup(machine):
        machine.host_write_sflags_set(1 << 3)

    machine, records = run_case("WAITS", dict(flag=3), setup=setup)
    assert records[0].done is True
    assert len(records) == 2
    assert machine.sflags == 0          # done by condition clears the flag


def test_jmp_call_ret_halt():
    """The four control instructions, including their next_pc and slot count."""
    machine, records = run_case("JMP", dict(abs=5), extra=HALT_AT_5)
    assert records[0].next_pc == 5 and len(records) == 2

    machine, records = run_case("CALL", dict(abs=5), extra=HALT_AT_5)
    assert records[0].next_pc == 5 and len(records) == 2
    assert machine.threads[0].rs0 == 1
    assert machine.threads[0].depth == 1

    machine, records = run_case("RET", dict())          # empty stack: PC + 1
    assert records[0].next_pc == 1 and len(records) == 2

    machine, records = run_thread(ISA, [("HALT", {})])
    assert len(records) == 1
    assert records[0].next_pc == 1
    assert machine.halted == 0b0001
    assert machine.run == 0


def test_csrr_reads_tid_of_the_running_thread():
    program = [("CSRR", dict(rd=1, csr=0x0C)), ("HALT", {})]
    machine, records = run_thread(ISA, program, thread=2)
    assert records[0].val == 2


def test_csrw_then_csrr_round_trip():
    program = [("LDI", dict(rd=1, imm=0x2A)),
               ("CSRW", dict(csr=0x00, ra=1)),
               ("CSRR", dict(rd=2, csr=0x00)),
               ("HALT", {})]
    machine, records = run_thread(ISA, program)
    assert records[2].val == 0x2A
    assert machine.threads[0].tick_int == 0x2A


def test_csrw_flags_writes_z_c_t():
    program = [("LDI", dict(rd=1, imm=0b101)),
               ("CSRW", dict(csr=0x0B, ra=1)),
               ("HALT", {})]
    machine, records = run_thread(ISA, program)
    assert records[1].flags == flags(z=1, c=0, t=1)
    assert machine.threads[0].z == 1 and machine.threads[0].t == 1


def test_read_only_and_unimplemented_csrs():
    """NOW and TID ignore writes; an unbuilt CSR reads 0 and ignores writes."""
    program = [("LDI", dict(rd=1, imm=0xFF)),
               ("CSRW", dict(csr=0x09, ra=1)),      # NOW is read only
               ("CSRW", dict(csr=0x0D, ra=1)),      # SR belongs to the bit engine
               ("CSRR", dict(rd=2, csr=0x0D)),
               ("CSRR", dict(rd=3, csr=0x15)),      # HOST_IRQ is write only
               ("HALT", {})]
    machine, records = run_thread(ISA, program)
    assert records[3].val == 0
    assert records[4].val == 0
    assert machine.badop == 0            # a CSR that is not built is not a BADOP


def test_csrw_host_irq_sets_the_software_interrupt_of_that_thread():
    program = [("CSRW", dict(csr=0x15, ra=0)), ("HALT", {})]
    machine, _ = run_thread(ISA, program, thread=2)
    assert machine.swirq == 0b0100


def test_csrr_sflags_sees_the_forwarded_value_from_the_slot_in_w():
    """SEMANTICS 3: SFLAGS in X includes the effect of the slot in W."""
    image = {}
    image.update(assemble(ISA, [("SIG", dict(flag=4)), ("HALT", {})], 0x000))
    # The NOP puts thread 1's CSRR in X while thread 0's SIG is in W: thread 1
    # issues at cycles 1, 5, ... so its second slot has X cycle 7, and thread
    # 0's SIG has X cycle 6.
    image.update(assemble(ISA, [("NOP", {}), ("CSRR", dict(rd=1, csr=0x14)),
                                ("HALT", {})], 0x100))
    machine = Machine(image, isa=ISA)
    machine.host_set_run(0b0011)
    records = machine.run_cycles(24)
    read = [r for r in records if r.mnemonic == "CSRR"][0]
    sig = [r for r in records if r.mnemonic == "SIG"][0]
    assert read.x_cycle == sig.x_cycle + 1          # the very next cycle
    assert read.val == 0b0001_0000                  # forwarded, not stale


def test_reserved_words_are_nops_that_set_badop():
    """SEMANTICS 9: a word that matches no instruction is a one-slot NOP."""
    reserved = [0xE000, 0xF000, 0x5100, 0x6800]
    for word in reserved:
        assert ISA.decode(word) is None
        image = {0: word, 1: ISA.encode("HALT")}
        machine = Machine(image)
        machine.host_set_run(0b0001)
        records = []
        for _ in range(200):
            record = machine.step_cycle()
            if record is not None:
                records.append(record)
            if machine.halted & 1:
                break
        assert len(records) == 2, hex(word)
        assert records[0].done is True
        assert records[0].next_pc == 1
        assert records[0].we is False
        assert machine.badop == 0b0001, hex(word)


def test_unbuilt_features_are_nops_that_set_badop():
    """Bit engine, data memory and FIFOs are absent in the default build."""
    for name, operands in [("SHO", {}), ("SHI", {}), ("LDSR", dict(ra=1)),
                           ("STSR", dict(rd=1)), ("CRCI", {}), ("STCRC", dict(rd=1)),
                           ("LD", dict(rd=1, ra=2, imm=0)),
                           ("ST", dict(rd=1, ra=2, imm=0)),
                           ("PUSH", dict(ra=1)), ("POP", dict(rd=1)),
                           ("WAITB", dict(cond=3))]:
        machine, records = run_case(name, operands)
        assert len(records) == 2, name
        assert records[0].done is True, name
        assert records[0].we is False, name
        assert records[0].next_pc == 1, name
        assert machine.badop == 0b0001, name


def test_badop_is_sticky_until_the_host_clears_it():
    program = [("SHO", {}), ("NOP", {}), ("HALT", {})]
    machine, records = run_thread(ISA, program)
    assert machine.badop == 0b0001
    machine.host_clear_badop(0b0001)
    machine.step_cycle()
    assert machine.badop == 0


def test_badop_is_per_thread():
    program = [("SHO", {}), ("HALT", {})]
    machine, records = run_thread(ISA, program, thread=1)
    assert machine.badop == 0b0010


def test_every_m1_mnemonic_has_a_directed_case():
    """The table above must cover every mnemonic this milestone builds."""
    covered = {case[0] for case in CASES}
    covered |= {"WAITE", "WAITS", "JMP", "CALL", "RET", "HALT"}
    covered |= {"BZ", "BNZ", "BC", "BNC", "BT", "BNT", "DJNZ", "JP"}  # control file
    unbuilt = {"SHO", "SHI", "LDSR", "STSR", "CRCI", "STCRC", "LD", "ST",
               "PUSH", "POP", "WAITB"}
    missing = {i.name for i in ISA.instructions} - covered - unbuilt
    assert missing == set()
