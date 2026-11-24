"""tools.loomnew: the scaffold's three files work as generated.

The files are written to a temporary tree, never into the repository: the
model and the test are imported from there under the names they would have
in the repository, the test's ``SOURCE`` is pointed at the generated
firmware, and every test body of the generated module is run on the golden
model. So "the generated test passes as generated" is checked on every
pytest run, not only when someone runs the scaffold. The same for the
receiver scaffold (``--rx``), and both generated tests assemble with the
staged-edge check and assert it is clean.
"""

import importlib.util
import sys

import pytest

from tools import loomnew
from tools.loomasm import assemble_file
from tools.loomnew import __main__ as cli
from tools.tests.fw_backend import MODEL

NAME = "loomnew_selftest"


def _import(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _run_bodies(test):
    """Every test body of a generated module on the golden model, each
    parametrized body once per parameter set; returns how many ran."""
    ran = 0
    for n in sorted(vars(test)):
        if not n.startswith("test_"):
            continue
        body = getattr(test, n)
        args = body.__code__.co_varnames[:body.__code__.co_argcount]
        sets = [{}]
        for mark in getattr(body, "pytestmark", []):
            if mark.name == "parametrize":
                names = [s.strip() for s in mark.args[0].split(",")]
                sets = [dict(base, **dict(zip(names, v if len(names) > 1 else [v])))
                        for base in sets for v in mark.args[1]]
        for kw in sets:
            if "backend" in args:
                kw = dict(kw, backend=MODEL)
            body(**kw)
            ran += 1
    return ran


@pytest.fixture(params=[False, True], ids=["tx", "rx"])
def generated_any(request, tmp_path):
    yield from _generate(tmp_path, rx=request.param)


@pytest.fixture
def generated(tmp_path):
    yield from _generate(tmp_path, rx=False)


@pytest.fixture
def generated_rx(tmp_path):
    yield from _generate(tmp_path, rx=True)


def _generate(tmp_path, rx):
    if rx:
        where = loomnew.create(NAME, tmp_path, pin="IN2", tick=8, rx=True)
    else:
        where = loomnew.create(NAME, tmp_path, pin="OUT2", tick=25)
    model_name = "tools.protomodels." + NAME
    test_name = "tools.tests.test_fw_" + NAME
    try:
        _import(model_name, where["model"])
        test = _import(test_name, where["test"])
        test.SOURCE = where["firmware"]
        yield where, test
    finally:
        sys.modules.pop(model_name, None)
        sys.modules.pop(test_name, None)


def test_loomnew_firmware_assembles_strict(generated):
    where, _ = generated
    prog = assemble_file(where["firmware"], strict=True, staged_check=True)
    assert prog.diagnostics == [] and prog.deadlines[0].late_staged == []
    assert prog.deadlines[0].worst_slack == 5
    assert len(prog.words) == 13 and max(prog.words) < 128
    text = where["firmware"].read_text(encoding="utf-8")
    assert ".imem   512" in text and "(D-032)" in text and "(D-035)" in text


def test_loomnew_wave_at_tick_16_trips_the_staged_check(generated):
    """Why the wave's .tick is 17: at 16 its 12-clock stretch leaves a slack
    of 4, and the staged check the generated test runs flags the pair."""
    where, _ = generated
    src = where["firmware"]
    text = src.read_text(encoding="utf-8")
    assert "        .tick   17 " in text
    src.write_text(text.replace("        .tick   17 ", "        .tick   16 "),
                   encoding="utf-8")
    prog = assemble_file(src, strict=True, staged_check=True)
    assert [d for d in prog.warnings if "staged edge may land up to 1 clock late"
            in d.message]


def test_loomnew_rx_firmware_assembles_strict(generated_rx):
    where, _ = generated_rx
    prog = assemble_file(where["firmware"], strict=True, staged_check=True)
    assert prog.diagnostics == [] and prog.warnings == []
    rep = prog.deadlines[0]
    assert rep.period == 4 and rep.pairs and rep.worst_slack >= 0
    assert len(prog.words) == 25 and max(prog.words) < 128
    text = where["firmware"].read_text(encoding="utf-8")
    assert ".imem   512" in text and "(D-032)" in text and "(D-035)" in text
    assert "RX = IN2" in text and "baudot_rx.loom" in text


def test_loomnew_generated_bodies_pass_on_the_model(generated):
    _, test = generated
    assert _run_bodies(test) == 3


def test_loomnew_generated_rx_bodies_pass_on_the_model(generated_rx):
    """Bytes, a framing error and a sender 3 % off rate either way, each
    received on the golden model as generated."""
    _, test = generated_rx
    assert _run_bodies(test) == 5


def test_loomnew_scenario_names_carry_the_protocol_name(generated_any):
    """CI loads every module together and refuses a duplicate name."""
    _, test = generated_any
    assert all(n.startswith("test_%s_" % NAME)
               for n in vars(test) if n.startswith("test_"))


def test_loomnew_refuses_to_overwrite_and_bad_names(tmp_path):
    loomnew.create("abc", tmp_path)
    with pytest.raises(FileExistsError):
        loomnew.create("abc", tmp_path)
    loomnew.create("abc", tmp_path, force=True)
    for bad in ("Abc", "1abc", "a-b", "x"):
        with pytest.raises(ValueError):
            loomnew.render(bad)
    with pytest.raises(ValueError):
        loomnew.render("abc", tick=16)
    with pytest.raises(ValueError):
        loomnew.render("abc", tick=3, rx=True)
    assert "RX = IN0" in loomnew.render("abc", rx=True)["firmware"]


def test_loomnew_cli_rejects_an_input_pin(tmp_path, capsys):
    with pytest.raises(SystemExit):
        cli.main(["abc", "--pin", "IN0", "--root", str(tmp_path)])
    assert cli.main(["abc", "--root", str(tmp_path)]) == 0
    assert "test_fw_abc.py" in capsys.readouterr().out


def test_loomnew_cli_rx_rejects_an_output_pin(tmp_path, capsys):
    with pytest.raises(SystemExit):
        cli.main(["abc", "--rx", "--pin", "OUT0", "--root", str(tmp_path)])
    assert cli.main(["abc", "--rx", "--pin", "BIDIR3", "--root", str(tmp_path)]) == 0
    assert "test_fw_abc.py" in capsys.readouterr().out
    text = (tmp_path / "firmware" / "abc.loom").read_text(encoding="utf-8")
    assert "RX = BIDIR3" in text
