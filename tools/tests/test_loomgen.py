"""Tests for the constrained-random program generator (L2-RAND support).

The generator feeds test/test_cosim.py, so its own rules are tested here, in
plain pytest, without a simulator: determinism by seed, every control-flow
target inside the thread's region, every M1 mnemonic produced across seeds,
JSON round trip, and every generated program running in the golden model
without a Python exception.
"""

import json

import pytest

from tools.loomgen import (M1_MNEMONICS, PROFILES, GeneratedProgram,
                           LoomgenError, check_program, generate,
                           static_target)
from tools.loomgen.runner import run_model
from tools.loomisa import load

ISA = load()


def test_same_seed_same_program():
    a = generate(seed=11, threads=4, profile="mixed")
    b = generate(seed=11, threads=4, profile="mixed")
    assert a.image == b.image
    assert a.entries == b.entries
    assert a.run_mask == b.run_mask
    assert json.dumps(a.to_obj(), sort_keys=True) == json.dumps(b.to_obj(), sort_keys=True)


def test_different_seeds_differ():
    images = {tuple(sorted(generate(seed=s, threads=2).image.items())) for s in range(20)}
    assert len(images) == 20


@pytest.mark.parametrize("profile", sorted(PROFILES))
@pytest.mark.parametrize("seed", range(25))
def test_generator_rules_hold(profile, seed):
    prog = generate(seed=seed, threads=1 + seed % 4, profile=profile)
    check_program(prog, ISA)          # raises LoomgenError on any violation
    for t in prog.running():
        lo, hi = prog.region(t)
        for addr in range(lo, hi):
            target = static_target(ISA, addr, prog.image[addr])
            if target is not None:
                assert lo <= target < hi, (
                    f"seed {seed} thread {t}: word at {addr:#x} jumps to {target:#x}, "
                    f"outside {lo:#x}..{hi:#x}")


def test_every_m1_mnemonic_is_generated():
    seen = set()
    for seed in range(200):
        prog = generate(seed=seed, threads=4, profile=sorted(PROFILES)[seed % len(PROFILES)])
        for word in prog.image.values():
            decoded = ISA.decode(word)
            if decoded is not None:
                seen.add(decoded[0].name)
    missing = sorted(set(M1_MNEMONICS) - seen)
    assert not missing, f"never generated in 200 seeds: {missing}"


def test_json_round_trip():
    prog = generate(seed=3, threads=3, profile="pins")
    back = GeneratedProgram.from_obj(json.loads(json.dumps(prog.to_obj())))
    assert back.image == prog.image
    assert back.entries == prog.entries
    assert back.run_mask == prog.run_mask
    assert [back.stimulus.at(c) for c in range(0, 3000, 97)] == \
           [prog.stimulus.at(c) for c in range(0, 3000, 97)]


@pytest.mark.parametrize("imem_words", [64, 256, 1024])
def test_memory_sizes(imem_words):
    prog = generate(seed=5, threads=4, imem_words=imem_words)
    assert all(0 <= a < imem_words for a in prog.image)
    assert prog.entries == [t * imem_words // 4 for t in range(4)]


def test_bad_arguments():
    with pytest.raises(LoomgenError):
        generate(seed=0, threads=0)
    with pytest.raises(LoomgenError):
        generate(seed=0, imem_words=300)


@pytest.mark.parametrize("seed", range(12))
def test_programs_run_in_the_model(seed):
    prog = generate(seed=seed, threads=1 + seed % 4,
                    profile=sorted(PROFILES)[seed % len(PROFILES)])
    retired = []
    machine = run_model(prog, cycles=5000, on_record=retired.append)
    assert retired, "no instruction retired"
    # Something must actually complete, not only stall in waits.
    assert any(r.done for r in retired)
    assert machine is not None


# ------------------------------------------------------------- the build
# A program is generated for the build CTRL.CAPS reports, and the harness
# builds the golden model the same way (test/test_cosim.py). The M2
# constructs have their own file, tools/tests/test_loomgen_m2.py.
from tools.loomgen import (DEFAULT_FEATURES, M2_CSR_NAMES,  # noqa: E402
                           M2_MNEMONICS, UNBUILT_MNEMONICS)


def _names_and_fields(prog):
    for word in prog.image.values():
        decoded = ISA.decode(word)
        if decoded is not None:
            yield decoded[0].name, decoded[1]


@pytest.mark.parametrize("seed", range(40))
def test_the_m1_build_never_executes_an_m2_construct(seed):
    """``features=()``: every M2 word is there, but as a NOP + BADOP word of
    SEMANTICS 9 (the generator does not guard or loop on them)."""
    prog = generate(seed=seed, threads=1 + seed % 4,
                    profile=sorted(PROFILES)[seed % len(PROFILES)],
                    features=())
    check_program(prog, ISA)
    assert prog.features == ()


def test_the_m1_build_still_produces_every_unbuilt_word():
    seen = set()
    for seed in range(40):
        prog = generate(seed=seed, threads=4, profile="mixed", features=())
        seen |= {name for name, _ in _names_and_fields(prog)}
    assert set(UNBUILT_MNEMONICS) <= seen, \
        f"unbuilt words missing: {sorted(set(UNBUILT_MNEMONICS) - seen)}"


def test_the_m2_build_executes_them_instead():
    seen = set()
    lat = 0
    be_csrs = {ISA.csr_by_name[n] for n in M2_CSR_NAMES}
    csr_hits = 0
    for seed in range(20):
        prog = generate(seed=seed, threads=4, profile="mixed",
                        features=DEFAULT_FEATURES)
        for name, fields in _names_and_fields(prog):
            seen.add(name)
            lat += fields["lat"] if name == "SETP" else 0
            csr_hits += int(name in ("CSRR", "CSRW") and fields["csr"] in be_csrs)
    assert set(M2_MNEMONICS) <= seen, sorted(set(M2_MNEMONICS) - seen)
    assert lat and csr_hits


@pytest.mark.parametrize("seed", range(4))
def test_m1_build_programs_run_in_the_model(seed):
    prog = generate(seed=seed, threads=4, profile=sorted(PROFILES)[seed],
                    features=())
    retired = []
    run_model(prog, cycles=3000, on_record=retired.append)
    assert any(r.done for r in retired)
