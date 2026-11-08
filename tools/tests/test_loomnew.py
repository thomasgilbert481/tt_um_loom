"""tools.loomnew: the scaffold's three files work as generated.

The files are written to a temporary tree, never into the repository: the
model and the test are imported from there under the names they would have
in the repository, the test's ``SOURCE`` is pointed at the generated
firmware, and every test body of the generated module is run on the golden
model. So "the generated test passes as generated" is checked on every
pytest run, not only when someone runs the scaffold.
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


@pytest.fixture
def generated(tmp_path):
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
    prog = assemble_file(where["firmware"], strict=True)
    assert prog.diagnostics == []
    assert len(prog.words) == 13 and max(prog.words) < 128
    text = where["firmware"].read_text(encoding="utf-8")
    assert ".imem   512" in text and "(D-032)" in text and "(D-035)" in text


def test_loomnew_generated_bodies_pass_on_the_model(generated):
    _, test = generated
    bodies = [getattr(test, n) for n in sorted(vars(test)) if n.startswith("test_")]
    assert len(bodies) == 3
    for body in bodies:
        if "backend" in body.__code__.co_varnames[:body.__code__.co_argcount]:
            body(backend=MODEL)
        else:
            body()


def test_loomnew_scenario_names_carry_the_protocol_name(generated):
    """CI loads every module together and refuses a duplicate name."""
    _, test = generated
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
        loomnew.render("abc", tick=12)


def test_loomnew_cli_rejects_an_input_pin(tmp_path, capsys):
    with pytest.raises(SystemExit):
        cli.main(["abc", "--pin", "IN0", "--root", str(tmp_path)])
    assert cli.main(["abc", "--root", str(tmp_path)]) == 0
    assert "test_fw_abc.py" in capsys.readouterr().out
