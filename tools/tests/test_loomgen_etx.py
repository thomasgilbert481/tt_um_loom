"""The generator in a build with the Ethernet serializer (SEMANTICS 6.12, D-036).

* A build without the serializer draws exactly the random stream it drew
  before ``ETX`` existed: the programs below are pinned by a hash of their
  JSON, taken from the generator at 2ef9be2 (branch ``eth``) before W-119.
* With ``ETX`` one running thread writes the ETX CSRs and no other does
  (6.12.7 ownership; two writers are untested, 6.12.9), every image keeps
  the generator's rules, and the programs run on the model.
* :class:`tools.loomgen.etxgen.Schedule` gives 6.12.6's edges.
"""

import hashlib
import json

import pytest

from tools.loomgen import LoomgenError, generate
from tools.loomgen.etxgen import ETX_CSRS, Schedule, cfg_value
from tools.loomgen.runner import run_model
from tools.loomisa import load as load_isa

ISA = load_isa()
AVOID = ("csrw_pin_out_high_bits",)
ETX_BUILD = ("BE", "BEENC", "DMEM", "ETX", "FIFO", "SETPD")

#: (features, profile, seed, imem_words or "host") -> sha256(JSON)[:16] at 2ef9be2.
PINNED = {
    ("BE,BEENC,DMEM,FIFO,SETPD", "alu", 1, 256): "a15bf4d1ae39795e",
    ("BE,BEENC,DMEM,FIFO,SETPD", "alu", 77, 512): "93d67c5643604ade",
    ("BE,BEENC,DMEM,FIFO,SETPD", "m2", 1, 512): "4acaab321cca4ae1",
    ("BE,BEENC,DMEM,FIFO,SETPD", "m2", 77, 256): "0439f5d224800a4f",
    ("BE,BEENC,DMEM,FIFO,SETPD", "m2", 5001, "host"): "a7d04da9fb73f1c2",
    ("BE,FIFO,SETPD", "m2", 1, 512): "f61bd58893abe615",
    ("BE,FIFO,SETPD", "mixed", 2, 512): "2ba2f6174529f86a",
    ("BE,FIFO,SETPD", "timing", 1, 512): "fc355f3fa957a529",
    ("BE,FIFO,SETPD", "m2", 5001, "host"): "81ea60fd294ffd1f",
    ("", "m2", 1, 512): "829b6cc6e0b88c6f",
    ("", "pins", 1, 512): "fb59ab3e26bfbb40",
}


def _digest(prog) -> str:
    return hashlib.sha256(json.dumps(prog.to_obj(), sort_keys=True).encode()).hexdigest()[:16]


@pytest.mark.parametrize("key", sorted(PINNED, key=str))
def test_builds_without_the_serializer_are_unchanged(key):
    features, profile, seed, imem = key
    feats = tuple(f for f in features.split(",") if f)
    if imem == "host":
        prog = generate(seed=seed, threads=2, imem_words=512, profile=profile,
                        cycles=8000, features=feats, host_traffic=True)
    else:
        prog = generate(seed=seed, threads=4 if seed != 3 else 2, imem_words=imem,
                        profile=profile, cycles=4000, avoid=AVOID, features=feats)
    assert _digest(prog) == PINNED[key]


def _etx_writers(prog):
    numbers = {ISA.csr_by_name[n] for n in ETX_CSRS}
    writers = set()
    for addr, word in prog.image.items():
        decoded = ISA.decode(word)
        if decoded and decoded[0].name == "CSRW" and decoded[1]["csr"] in numbers:
            writers.add(next(t for t in prog.running()
                             if prog.region(t)[0] <= addr < prog.region(t)[1]))
    return writers


@pytest.mark.parametrize("seed", range(1, 13))
def test_one_writer_and_the_rules_hold(seed):
    profile = ("m2", "timing", "pins", "alu", "mixed")[seed % 5]
    prog = generate(seed=seed, threads=4 if seed % 6 else 2, imem_words=512,
                    profile=profile, avoid=AVOID, features=ETX_BUILD, isa=ISA)
    assert prog.features == ETX_BUILD
    assert len(_etx_writers(prog)) == 1        # generate() ran check_program too


def test_etx_programs_run_and_write_the_serializer():
    numbers = {ISA.csr_by_name[n]: n for n in ETX_CSRS}
    seen = set()

    def keep(rec):
        decoded = ISA.decode(rec.ir)
        if rec.done and decoded and decoded[0].name == "CSRW" \
                and decoded[1]["csr"] in numbers:
            seen.add(numbers[decoded[1]["csr"]])

    for seed in (1, 2, 3, 4):
        run_model(generate(seed=seed, threads=4, imem_words=512, profile="mixed",
                           avoid=AVOID, features=ETX_BUILD, isa=ISA), 4000, keep)
    assert seen == set(ETX_CSRS)


def test_unknown_features_are_still_refused():
    with pytest.raises(LoomgenError):
        generate(seed=1, features=("ETX", "NOSUCH"), isa=ISA)


def test_schedule_is_semantics_6_12_6():
    # N = 2 with PRE: K_m = s + 2N(64 + 16 m), the window of word 1 is
    # K_0 .. K_0 + 32N - 1; without PRE K_0 = s and word 1's window starts at s + 1.
    sch = Schedule(2, 1, 1, 6)
    assert [sch.k(m) for m in range(3)] == [256, 320, 384]
    assert sch.window(1) == (256, 319)
    flat = Schedule(3, 0, 0, 0)
    assert flat.k(0) == 0 and flat.window(1) == (1, 95)
    # release at s + N (2 NBITS + IDL): 2 words, the last of one byte, FCS.
    assert sch.release(sch.tail(2, 1)) == 2 * (2 * (64 + 16 + 8 + 32) + 6)
    assert cfg_value(1, 1, 1, 6) == 0x01B1                # the reset value, 6.12.2
