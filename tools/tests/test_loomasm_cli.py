"""Tests for ``python -m tools.loomasm``."""

import json

import pytest

from tools.loomasm.__main__ import main
from tools.loomisa import REPO

GOOD = """\
        .thread 0
        .tick   100
start:  SETD    0
        NOP
        WAITD   1
        HALT
"""

INFEASIBLE = """\
        .thread 0
        .tick   4
        SETD    0
        NOP
        NOP
        WAITD   1
        HALT
"""

BROKEN = """\
        .thread 0
        FOO     r1
        BZ      nowhere
"""


def write(tmp_path, text, name="p.loom"):
    path = tmp_path / name
    path.write_text(text, encoding="utf-8")
    return str(path)


def test_writes_an_image_and_reports_the_word_count(tmp_path, capsys):
    source = write(tmp_path, GOOD)
    out = tmp_path / "build" / "p.json"
    assert main([source, "-o", str(out)]) == 0
    captured = capsys.readouterr()
    assert "wrote" in captured.err and "4 words" in captured.err
    image = json.loads(out.read_text(encoding="utf-8"))
    assert image["threads"] == {"0": {"entry": 0, "size": 4}}
    assert image["symbols"]["start"] == 0


def test_image_goes_to_stdout_when_no_output_is_given(tmp_path, capsys):
    assert main([write(tmp_path, GOOD)]) == 0
    image = json.loads(capsys.readouterr().out)
    assert image["isa"] and image["words"]


def test_listing_to_stdout_and_to_a_file(tmp_path, capsys):
    source = write(tmp_path, GOOD)
    assert main([source, "--listing"]) == 0
    text = capsys.readouterr().out
    assert "ADDR  WORD  TH  TIMING" in text
    assert "thread 0 deadline analysis (tick period 100 clocks)" in text

    listing = tmp_path / "p.lst"
    assert main([source, "--listing", str(listing)]) == 0
    assert listing.read_text(encoding="utf-8") == text


def test_an_infeasible_schedule_still_writes_an_image(tmp_path, capsys):
    source = write(tmp_path, INFEASIBLE)
    out = tmp_path / "p.json"
    assert main([source, "-o", str(out)]) == 0
    assert "deadline cannot be met" in capsys.readouterr().err
    assert out.exists()


def test_strict_refuses_an_infeasible_schedule(tmp_path, capsys):
    source = write(tmp_path, INFEASIBLE)
    out = tmp_path / "p.json"
    assert main([source, "-o", str(out), "--strict"]) == 1
    assert "no image written" in capsys.readouterr().err
    assert not out.exists()


def test_no_deadline_check_silences_the_analysis(tmp_path, capsys):
    source = write(tmp_path, INFEASIBLE)
    assert main([source, "-o", str(tmp_path / "p.json"),
                 "--no-deadline-check", "--strict"]) == 0
    assert "deadline cannot be met" not in capsys.readouterr().err


TWO_THREADS = """\
        .thread 0
a:      NOP
        .thread 1
b:      NOP
"""


def test_imem_words_defaults_to_1024(tmp_path, capsys):
    assert main([write(tmp_path, TWO_THREADS)]) == 0
    image = json.loads(capsys.readouterr().out)
    assert image["imem_words"] == 1024
    assert image["symbols"] == {"a": 0, "b": 256}


def test_imem_words_moves_the_thread_origins(tmp_path, capsys):
    assert main([write(tmp_path, TWO_THREADS), "--imem-words", "256"]) == 0
    image = json.loads(capsys.readouterr().out)
    assert image["imem_words"] == 256
    assert image["symbols"] == {"a": 0, "b": 64}
    assert image["threads"] == {"0": {"entry": 0, "size": 1},
                                "1": {"entry": 64, "size": 1}}


def test_imem_words_rejects_a_size_that_is_not_a_power_of_two(tmp_path):
    with pytest.raises(SystemExit):
        main([write(tmp_path, TWO_THREADS), "--imem-words", "100"])


def test_the_imem_directive_works_from_the_command_line(tmp_path, capsys):
    source = write(tmp_path, ".imem 128\n" + TWO_THREADS)
    assert main([source]) == 0
    image = json.loads(capsys.readouterr().out)
    assert image["imem_words"] == 128
    assert image["symbols"] == {"a": 0, "b": 32}


def test_the_option_beats_the_directive_and_warns(tmp_path, capsys):
    source = write(tmp_path, ".imem 128\n" + TWO_THREADS)
    assert main([source, "--imem-words", "512"]) == 0
    captured = capsys.readouterr()
    assert "--imem-words 512 was given" in captured.err
    assert json.loads(captured.out)["imem_words"] == 512


def test_code_past_the_end_of_a_small_memory_fails(tmp_path, capsys):
    source = write(tmp_path, ".thread 3\n" + "NOP\n" * 20)
    assert main([source, "--imem-words", "64"]) == 1
    assert "past the end of instruction memory (64 words)" \
        in capsys.readouterr().err


def test_a_broken_source_fails_and_reports_every_error(tmp_path, capsys):
    assert main([write(tmp_path, BROKEN)]) == 1
    err = capsys.readouterr().err
    assert "unknown instruction 'FOO'" in err
    assert "unknown symbol 'nowhere'" in err


def test_a_missing_file_fails_cleanly(tmp_path, capsys):
    assert main([str(tmp_path / "nope.loom")]) == 1
    assert "loomasm:" in capsys.readouterr().err


def test_quiet_prints_errors_but_not_warnings(tmp_path, capsys):
    unbounded = ".thread 0\n.tick 100\nSETD 0\nDLY 1\nWAITD 1\nHALT\n"
    source = write(tmp_path, unbounded)
    assert main([source, "-o", str(tmp_path / "a.json")]) == 0
    assert "unbounded between deadlines" in capsys.readouterr().err
    assert main([source, "-o", str(tmp_path / "b.json"), "-q"]) == 0
    assert "unbounded" not in capsys.readouterr().err


@pytest.mark.parametrize("name", ["uart_hello.loom", "uart_tx.loom"])
def test_the_firmware_builds_from_the_command_line(name, tmp_path, capsys):
    source = str(REPO / "firmware" / name)
    image = tmp_path / (name.replace(".loom", ".json"))
    listing = tmp_path / (name.replace(".loom", ".lst"))
    assert main([source, "-o", str(image), "--listing", str(listing),
                 "--strict"]) == 0
    assert json.loads(image.read_text(encoding="utf-8"))["words"]
    assert "0 errors, 0 warnings" in listing.read_text(encoding="utf-8")
