"""The public API: images, run_until, the trace callback and the CLI.

Also covers SEMANTICS section 10: the model is deterministic given the image,
the host actions and the pad values, so two identical runs agree slot by slot.
"""

import json

import pytest

from tools.loomisa import load
from tools.loomsim import (CycleTrace, LoomsimError, Machine, RetireRecord,
                           image_from_obj, load_image_file)
from tools.loomsim.__main__ import main
from tools.loomsim.harness import assemble

ISA = load()

PROGRAM = [("LDI", dict(rd=1, imm=0x41)),
           ("SETP", dict(pin=16, val=1)),
           ("HALT", {})]


def write_image(path, program=PROGRAM):
    words = assemble(ISA, program)
    payload = {"words": {str(addr): word for addr, word in words.items()}}
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


# -------------------------------------------------------------------- images
def test_image_from_obj_accepts_the_assembler_format():
    words = image_from_obj({"words": {"0": 1, "0x10": 2, 32: 3}})
    assert words == {0: 1, 0x10: 2, 32: 3}


def test_image_from_obj_accepts_a_bare_mapping():
    assert image_from_obj({0: 0x1234}) == {0: 0x1234}


def test_load_image_file_round_trip(tmp_path):
    path = write_image(tmp_path / "image.json")
    words = load_image_file(path)
    assert words == assemble(ISA, PROGRAM)


def test_load_image_can_be_called_after_construction():
    machine = Machine(isa=ISA)
    machine.load_image(assemble(ISA, PROGRAM))
    machine.host_set_run(0b0001)
    machine.run_cycles(24)
    assert machine.threads[0].regs[1] == 0x41


def test_instruction_memory_wraps_within_its_size():
    machine = Machine({0x100: ISA.encode("HALT")}, imem_words=256, isa=ISA)
    assert machine.imem == {0: ISA.encode("HALT")}
    assert [t.pc for t in machine.threads] == [0, 64, 128, 192]


def test_a_bad_memory_size_is_rejected():
    with pytest.raises(LoomsimError):
        Machine({}, imem_words=300, isa=ISA)


def test_unwritten_instruction_memory_reads_zero():
    machine = Machine({}, isa=ISA)
    assert machine.imem.get(7, 0) == 0


# ----------------------------------------------------------------- run_until
def test_run_until_stops_on_the_predicate():
    machine = Machine(assemble(ISA, PROGRAM), isa=ISA)
    machine.host_set_run(0b0001)
    result = machine.run_until(lambda m: bool(m.halted & 1), max_cycles=100)
    assert result.fired is True
    assert result.cycles == machine.cycle
    assert [r.mnemonic for r in result.records] == ["LDI", "SETP", "HALT"]


def test_run_until_reports_a_timeout_without_raising():
    machine = Machine({0: ISA.encode("JMP", abs=0)}, isa=ISA)
    machine.host_set_run(0b0001)
    result = machine.run_until(lambda m: bool(m.halted & 1), max_cycles=50)
    assert result.fired is False
    assert result.cycles == 50
    assert machine.cycle == 50


def test_run_until_costs_nothing_when_it_is_already_true():
    machine = Machine({}, isa=ISA)
    result = machine.run_until(lambda m: True, max_cycles=10)
    assert (result.fired, result.cycles, machine.cycle) == (True, 0, 0)


def test_run_until_can_watch_the_pads():
    machine = Machine(assemble(ISA, PROGRAM), isa=ISA)
    machine.host_set_run(0b0001)
    result = machine.run_until(lambda m: m.uo_out != 0, max_cycles=100)
    assert result.fired is True
    assert machine.uo_out == 1


# ------------------------------------------------------------ trace callback
def test_the_trace_callback_sees_every_cycle_once():
    seen = []
    machine = Machine(assemble(ISA, PROGRAM), isa=ISA, on_cycle=seen.append)
    machine.host_set_run(0b0001)
    machine.run_cycles(16)
    assert [t.cycle for t in seen] == list(range(16))
    assert all(isinstance(t, CycleTrace) for t in seen)
    assert [t.ph for t in seen[:5]] == [0, 1, 2, 3, 0]
    retires = [t for t in seen if t.retire is not None]
    assert [t.cycle for t in retires] == [7, 11, 15]
    assert isinstance(retires[0].retire, RetireRecord)


def test_the_trace_reports_pads_before_the_edge_that_changes_them():
    seen = []
    machine = Machine(assemble(ISA, PROGRAM), isa=ISA, on_cycle=seen.append)
    machine.host_set_run(0b0001)
    machine.run_cycles(20)
    setp = [t.retire for t in seen if t.retire is not None
            and t.retire.mnemonic == "SETP"][0]
    assert seen[setp.x_cycle + 1].uo_out == 0
    assert seen[setp.x_cycle + 2].uo_out == 1


# ------------------------------------------------------------- determinism
def test_two_identical_runs_agree_slot_by_slot():
    def go():
        machine = Machine(assemble(ISA, PROGRAM), isa=ISA)
        machine.set_pad_inputs(ui_in=0x0F, uio_in=0xF0)
        machine.host_set_run(0b0001)
        return [r.as_tuple() for r in machine.run_cycles(40)]

    assert go() == go()


def test_reset_returns_the_machine_to_cycle_zero_but_keeps_memory():
    machine = Machine(assemble(ISA, PROGRAM), isa=ISA)
    machine.host_set_run(0b0001)
    machine.run_cycles(24)
    assert machine.threads[0].regs[1] == 0x41
    image = dict(machine.imem)
    machine.reset()
    assert machine.cycle == 0
    assert machine.threads[0].regs[1] == 0
    assert machine.run == 0 and machine.pin_out == 0
    assert machine.imem == image


def test_caps_has_the_layout_semantics_five_fixes():
    """[2:0] log2 FIFO depth, [3] FIFO, [4] BE, [5] DMEM, [6] ROM, [15:12] log2 words."""
    assert Machine({}, imem_words=256, isa=ISA).caps == 0x8000
    assert Machine({}, imem_words=1024, isa=ISA).caps == 0xA000
    assert Machine({}, imem_words=512, isa=ISA).caps == 0x9000


def test_caps_reports_the_optional_features():
    machine = Machine({}, imem_words=256, fifo_depth=4, features={"FIFO"}, isa=ISA)
    assert machine.caps == 0x8000 | (1 << 3) | 2          # depth 4 -> log2 2
    machine = Machine({}, imem_words=256, fifo_depth=8, features={"FIFO"}, isa=ISA)
    assert machine.caps & 0x7 == 3
    machine = Machine({}, imem_words=256, features={"BE", "DMEM", "BOOTROM"},
                      isa=ISA)
    assert machine.caps == 0x8000 | (1 << 4) | (1 << 5) | (1 << 6)
    assert machine.caps & 0xF == 0                        # no FIFOs: depth reads 0


def test_repr_is_informative():
    machine = Machine({}, isa=ISA)
    assert "cycle=0" in repr(machine)


def test_retire_record_formats_itself():
    machine = Machine(assemble(ISA, PROGRAM), isa=ISA)
    machine.host_set_run(0b0001)
    records = machine.run_cycles(12)
    text = str(records[0])
    assert "LDI" in text and "r1=0041" in text and "done" in text


# --------------------------------------------------------------------- CLI
def test_cli_runs_an_image_and_prints_the_trace(tmp_path, capsys):
    path = write_image(tmp_path / "image.json")
    assert main(["run", str(path), "--cycles", "60"]) == 0
    out = capsys.readouterr().out
    assert "LDI" in out
    assert "SETP" in out
    assert "uo_out=01" in out
    assert "halted=1" in out


def test_cli_trace_flag_prints_every_cycle(tmp_path, capsys):
    path = write_image(tmp_path / "image.json")
    main(["run", str(path), "--cycles", "12", "--trace"])
    out = capsys.readouterr().out
    assert out.count("ph=") == 12


def test_cli_run_mask_selects_the_thread(tmp_path, capsys):
    words = assemble(ISA, PROGRAM, 0x100)
    payload = {"words": {hex(addr): word for addr, word in words.items()}}
    path = tmp_path / "t1.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    main(["run", str(path), "--cycles", "60", "--run-mask", "0b0010"])
    out = capsys.readouterr().out
    assert " t1 " in out
    assert "halted=2" in out


def test_cli_until_halt_stops_early(tmp_path, capsys):
    path = write_image(tmp_path / "image.json")
    main(["run", str(path), "--cycles", "5000", "--until-halt"])
    out = capsys.readouterr().out
    assert "stopped at cycle 16," in out


def test_cli_feature_flag_enables_the_fifos(tmp_path, capsys):
    program = [("PUSH", dict(ra=1)), ("HALT", {})]
    path = write_image(tmp_path / "fifo.json", program)
    main(["run", str(path), "--cycles", "60", "--feature", "FIFO"])
    assert "badop=0000" in capsys.readouterr().out
    main(["run", str(path), "--cycles", "60"])
    assert "badop=0001" in capsys.readouterr().out
