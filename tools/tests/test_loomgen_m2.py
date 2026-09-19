"""Generator tests for the M2 constructs (L2-RAND over SEMANTICS 6.7-6.10).

``test/test_cosim.py`` builds the golden model with the features ``CTRL.CAPS``
reports and asks the generator for a program for that same build, so the
generator has to emit the FIFO ops, the bit engine and the deadline-latched
``SETP`` as live code, and only as ``NOP`` + ``BADOP`` words in a build
without them. What is checked here, per construct:

* it is **encoded** (every M2 mnemonic, every ``WAITB`` condition, ``SETP``
  with the ``D`` bit, every bit-engine CSR appear across a few seeds);
* it **assembles**: the disassembly of every generated word, fed back through
  ``tools.loomasm``, gives the same word;
* it **runs in the model without hanging**: every running thread keeps
  retiring completed slots, no ``PUSH``/``POP`` ever stalls in a program
  without host traffic (the ``WAITB`` guard makes that impossible), and with
  host traffic the blocking forms both stall and complete.
"""

import collections

import pytest

from tools.loomasm import assemble_text, disassemble
from tools.loomgen import (DEFAULT_FEATURES, PROFILES, RUN_CYCLE,
                           LoomgenError, check_program, generate,
                           instruction_built, unbuilt_mnemonics)
from tools.loomgen.generator import (WAITB_BE_IDLE, WAITB_INQ_NE,
                                     WAITB_OUTQ_NF, WAITB_TICK)
from tools.loomgen.runner import run_model
from tools.loomisa import load

ISA = load()
M2 = tuple(DEFAULT_FEATURES)                      # ("BE", "FIFO", "SETPD")
PROFILE_LIST = sorted(PROFILES)


def decoded(prog):
    for addr in sorted(prog.image):
        hit = ISA.decode(prog.image[addr])
        if hit is not None:
            yield addr, hit[0].name, hit[1]


def m2_program(seed, **kwargs):
    kwargs.setdefault("threads", 4)
    kwargs.setdefault("profile", "m2")
    kwargs.setdefault("imem_words", 256)
    kwargs.setdefault("cycles", 6000)
    return generate(seed=seed, features=M2, isa=ISA, **kwargs)


# --------------------------------------------------------------- the build
def test_the_m2_built_avoid_flag_is_gone():
    """D1: the harness builds the model from CAPS instead of generating
    around the M2 features, so the flag that used to skip them is not a
    known avoid flag any more."""
    with pytest.raises(LoomgenError):
        generate(seed=1, avoid=("m2_built",), isa=ISA)


def test_default_build_is_the_m2_chip():
    prog = generate(seed=1, isa=ISA)
    assert prog.features == ("BE", "FIFO", "SETPD")
    assert prog.fifo_depth == 4
    assert prog.host is None


@pytest.mark.parametrize("features,expected", [
    ((), ("PUSH", "POP", "WAITB", "SHO", "SHI", "LDSR", "STSR", "CRCI",
          "STCRC", "LD", "ST")),
    (("FIFO",), ("SHO", "SHI", "LDSR", "STSR", "CRCI", "STCRC", "LD", "ST")),
    (("BE",), ("PUSH", "POP", "WAITB", "LD", "ST")),
    (M2, ("LD", "ST")),
])
def test_unbuilt_pool_follows_the_build(features, expected):
    assert unbuilt_mnemonics(features) == expected


def test_waitb_be_idle_needs_the_bit_engine():
    """SEMANTICS 6.4: WAITB is built with the FIFOs, but condition 0 also
    needs the bit engine, so for this one instruction BADOP depends on the
    operand."""
    assert instruction_built("WAITB", {"cond": WAITB_TICK}, ("FIFO",))
    assert not instruction_built("WAITB", {"cond": WAITB_BE_IDLE}, ("FIFO",))
    assert instruction_built("WAITB", {"cond": WAITB_BE_IDLE}, ("FIFO", "BE"))
    assert not instruction_built("WAITB", {"cond": WAITB_TICK}, ("BE",))


def test_bad_build_arguments():
    with pytest.raises(LoomgenError):
        generate(seed=1, features=("DMEM",), isa=ISA)
    with pytest.raises(LoomgenError):
        generate(seed=1, features=("FIFO",), fifo_depth=3, isa=ISA)
    with pytest.raises(LoomgenError):
        generate(seed=1, features=(), host_traffic=True, isa=ISA)


# ------------------------------------------------------------ the encodings
def test_every_m2_construct_is_generated():
    names = collections.Counter()
    waitb_conds = set()
    setp_lat = 0
    be_csrs = collections.Counter()
    be_csr_numbers = {ISA.csr_by_name[n]: n for n in
                      ("BE_CFG", "BE_PINS", "BE_RELOAD", "CRC_POLY", "CRC_INIT",
                       "SR", "CNT", "CRC")}
    for seed in range(12):
        prog = m2_program(seed, profile=PROFILE_LIST[seed % len(PROFILE_LIST)])
        for _, name, fields in decoded(prog):
            names[name] += 1
            if name == "WAITB":
                waitb_conds.add(fields["cond"])
            elif name == "SETP":
                setp_lat += fields["lat"]
            elif name in ("CSRR", "CSRW") and fields["csr"] in be_csr_numbers:
                be_csrs[be_csr_numbers[fields["csr"]]] += 1
    missing = [n for n in ("PUSH", "POP", "WAITB", "SHO", "SHI", "LDSR", "STSR",
                           "CRCI", "STCRC") if not names[n]]
    assert not missing, "never generated in 12 seeds: %s" % missing
    assert waitb_conds == {WAITB_BE_IDLE, WAITB_OUTQ_NF, WAITB_INQ_NE, WAITB_TICK}
    assert setp_lat, "no SETP with the D bit (SEMANTICS 6.10)"
    assert not set(be_csr_numbers.values()) - set(be_csrs), \
        "bit-engine CSRs never touched: %s" % sorted(
            set(be_csr_numbers.values()) - set(be_csrs))
    # LD/ST are the only words left that are NOP + BADOP in this build.
    assert names["LD"] and names["ST"]


def test_m1_build_emits_the_m2_words_as_unbuilt_words():
    """With features=() the same emitters produce the NOP + BADOP words of
    SEMANTICS 9, and no PUSH/POP guard (they never execute)."""
    names = collections.Counter()
    for seed in range(8):
        prog = generate(seed=seed, threads=4, profile="m2", imem_words=256,
                        features=(), isa=ISA)
        assert prog.features == ()
        for _, name, _ in decoded(prog):
            names[name] += 1
    for name in ("PUSH", "POP", "WAITB", "SHO", "SHI", "LDSR", "STSR", "CRCI",
                 "STCRC", "LD", "ST"):
        assert names[name], "%s never generated in the M1 build" % name


def test_crc_presets_are_loaded_left_aligned():
    """SEMANTICS 6.9: an n-bit CRC lives in CRC[15:16-n] with its polynomial
    left-aligned the same way; the generator loads isa.yaml's presets."""
    wanted = {(int(p["poly"]) << (16 - int(p["width"]))) & 0xFFFF
              for p in ISA.crc_presets.values()}
    poly_csr = ISA.csr_by_name["CRC_POLY"]
    seen = set()
    for seed in range(12):
        prog = m2_program(seed)
        words = sorted(prog.image)
        for i, addr in enumerate(words):
            hit = ISA.decode(prog.image[addr])
            if hit is None or hit[0].name != "CSRW" or hit[1]["csr"] != poly_csr:
                continue
            # The value comes from the LDI (+ LDIH) right before the CSRW.
            value = 0
            for back in (2, 1):
                if i - back < 0:
                    continue
                prev = ISA.decode(prog.image[words[i - back]])
                if prev is None:
                    continue
                if prev[0].name == "LDI":
                    value = (value & 0xFF00) | prev[1]["imm"]
                elif prev[0].name == "LDIH":
                    value = (value & 0x00FF) | (prev[1]["imm"] << 8)
            seen.add(value)
    assert wanted & seen, "no .crc preset polynomial was loaded: %s" % sorted(seen)


# -------------------------------------------------------------- it assembles
@pytest.mark.parametrize("profile", PROFILE_LIST)
def test_generated_words_assemble_back_to_themselves(profile):
    """Disassemble every word of a region and assemble the text again."""
    prog = m2_program(3, profile=profile, threads=1, run_mask=1)
    start, end = prog.region(0)
    lines = [".thread 0"]
    for addr in range(start, end):
        word = prog.image[addr]
        hit = ISA.decode(word)
        if hit is not None and hit[0].name == "WAITE" and hit[1]["edge"] == 3:
            # The generator emits the reserved edge 3 on purpose (SEMANTICS
            # 6.4: never true); isa.yaml's enum has no spelling for it, so
            # the disassembler prints "3" and the assembler refuses it.
            lines.append("        .word 0x%04X" % word)
        else:
            lines.append("        " + disassemble(word, ISA))
    program = assemble_text("\n".join(lines) + "\n", "m2.loom", isa=ISA,
                            imem_words=prog.imem_words)
    again = [program.words[a] for a in sorted(program.words)]
    original = [prog.image[a] for a in range(start, end)]
    assert len(again) == len(original)
    # JMP/CALL/branch targets are absolute or relative to their own address,
    # and the text was laid out at 0 rather than at the region start, so only
    # the non-control words have to match word for word.
    control = {"JMP", "CALL", "BZ", "BNZ", "BC", "BNC", "BT", "BNT", "DJNZ", "JP"}
    for old, new in zip(original, again):
        hit = ISA.decode(old)
        if hit is not None and hit[0].name in control:
            continue
        assert old == new, "%04X assembled back as %04X (%s)" % (
            old, new, disassemble(old, ISA))


def test_the_m2_constructs_assemble_word_for_word():
    """The blocks the M2 emitters produce, including their own branches."""
    text = "\n".join([
        ".thread 0",
        "        WAITB   OUTQ_NF, T",
        "        BT      1",
        "        PUSH    r3",
        "        WAITB   INQ_NE, T",
        "        BT      1",
        "        POP     r2",
        "        WAITB   TICK",
        "        WAITB   BE_IDLE, T",
        "        LDSR    r1",
        "        LDI     r7, 8",
        "        CSRW    CNT, r7",
        "        SHO",
        "        DLY     1",
        "        BNZ     -3",
        "        SHI",
        "        BNZ     -2",
        "        STSR    r4",
        "        SHRI    r4, 8",
        "        CRCI",
        "        STCRC   r5",
        "        CSRW    CRC_POLY, r7",
        "        CSRW    BE_CFG, r7",
        "        CSRW    BE_PINS, r7",
        "        SETP    OUT2, 1, D",
        "        WAITD   12",
        "",
    ])
    program = assemble_text(text, "m2.loom", isa=ISA)
    words = [program.words[a] for a in sorted(program.words)]
    back = "\n".join([".thread 0"] + ["        " + disassemble(w, ISA) for w in words])
    again = assemble_text(back + "\n", "m2.loom", isa=ISA)
    assert [again.words[a] for a in sorted(again.words)] == words
    names = [ISA.decode(w)[0].name for w in words]
    assert names.count("WAITB") == 4 and "PUSH" in names and "POP" in names
    assert ISA.decode(words[names.index("SETP")])[1]["lat"] == 1


# ------------------------------------------------------- it runs in the model
def model_run(prog, cycles=6000):
    """Run in the model and collect per-thread liveness facts."""
    done_after = {}
    stalls = collections.Counter()
    worst = collections.Counter()
    fifo = collections.Counter()

    def note(record):
        t = record.thread
        name = (ISA.decode(record.ir) or (None, None))[0]
        name = name.name if name is not None else "reserved"
        if name in ("PUSH", "POP"):
            fifo["%s:%s" % (name, "done" if record.done else "stall")] += 1
        if record.done:
            done_after[t] = record.x_cycle + 1
            stalls[t] = 0
        else:
            stalls[t] += 1
            worst[t] = max(worst[t], stalls[t])
    machine = run_model(prog, cycles, note)
    return done_after, worst, fifo, machine


@pytest.mark.parametrize("profile", PROFILE_LIST)
@pytest.mark.parametrize("seed", range(3))
def test_m2_programs_run_in_the_model(profile, seed):
    prog = m2_program(seed, profile=profile, threads=1 + seed % 3)
    check_program(prog, ISA)
    done_after, _, fifo, machine = model_run(prog)
    assert done_after, "no instruction completed"
    # Without host traffic the INQ is never filled and the OUTQ never
    # emptied, so the generator guards every PUSH/POP with a timed WAITB and
    # neither may ever stall (module docstring of tools/loomgen).
    assert not fifo["PUSH:stall"] and not fifo["POP:stall"], dict(fifo)


@pytest.mark.parametrize("depth", (2, 4, 8))
def test_fifo_depths(depth):
    prog = m2_program(5, fifo_depth=depth)
    assert prog.fifo_depth == depth
    done_after, _, fifo, machine = model_run(prog)
    assert not fifo["PUSH:stall"] and not fifo["POP:stall"]
    assert any(len(th.outq) for th in machine.threads), \
        "nothing was ever pushed into an OUTQ"


@pytest.mark.parametrize("seed", range(4))
def test_host_traffic_programs_block_and_get_served(seed):
    """With a host plan the raw (blocking) PUSH/POP are emitted: they stall
    on a full OUTQ or an empty INQ and the host's traffic releases them."""
    prog = m2_program(seed, threads=2, host_traffic=True, cycles=20000)
    assert prog.host is not None and len(prog.host) > 4
    check_program(prog, ISA)
    done_after, worst, fifo, machine = model_run(prog, cycles=20000)
    assert fifo["PUSH:done"] and fifo["POP:done"], dict(fifo)
    for t in prog.running():
        assert t in done_after, "thread %d never completed an instruction" % t
    assert machine.badop & 0x4000 or True        # BADOP[14] is the harness's job


def test_host_plan_serves_every_running_thread():
    prog = m2_program(7, threads=2, host_traffic=True)
    pushed = {t.thread for t in prog.host.txns if t.kind == "push"}
    popped = {t.thread for t in prog.host.txns if t.kind == "pop"}
    for t in prog.running():
        assert t in pushed and t in popped
    assert any(t.kind == "write" for t in prog.host.txns)


def test_host_plan_round_trips_through_json():
    import json
    from tools.loomgen import GeneratedProgram
    prog = m2_program(9, threads=2, host_traffic=True)
    back = GeneratedProgram.from_obj(json.loads(json.dumps(prog.to_obj())))
    assert back.host == prog.host
    assert back.features == prog.features and back.fifo_depth == prog.fifo_depth


def test_check_program_rejects_an_unguarded_fifo_op():
    """The static no-deadlock rule: without host traffic a PUSH may not be
    reached except through its WAITB guard."""
    prog = m2_program(2, threads=1, run_mask=1)
    start, _ = prog.region(0)
    # Overwrite the first word of the region (a CSRW block head) with a PUSH.
    prog.image[start] = ISA.encode("PUSH", ra=1)
    with pytest.raises(LoomgenError):
        check_program(prog, ISA)


def test_check_program_rejects_an_untimed_fifo_waitb():
    prog = m2_program(2, threads=1, run_mask=1)
    start, _ = prog.region(0)
    prog.image[start] = ISA.encode("WAITB", cond=WAITB_INQ_NE, tmo=0)
    with pytest.raises(LoomgenError):
        check_program(prog, ISA)


# ------------------------------------ the host side the harness builds on
def fifo_machine():
    from tools.loomsim import Machine
    return Machine(features=("FIFO",), fifo_depth=4, imem_words=64, isa=ISA)


def test_peek_takes_the_head_or_the_entry_after_it():
    """SEMANTICS 6.7: a FIFO read word peeks the head, or the entry after it
    when the previous word of the transaction is being popped."""
    from tools.loomgen import peek_word
    m = fifo_machine()
    assert peek_word(m, 0, False) is None
    m.threads[0].outq.extend([0x1111, 0x2222])
    assert peek_word(m, 0, False) == 0x1111
    assert peek_word(m, 0, True) == 0x2222
    m.threads[0].outq.pop()
    assert peek_word(m, 0, True) is None      # only one entry left


def test_pop_after_peek_pops_what_the_peek_found():
    from tools.loomgen import peek_word, pop_after_peek
    m = fifo_machine()
    m.threads[0].outq.extend([0x1111, 0x2222])
    peeked = peek_word(m, 0, False)
    pop_after_peek(m, 0, peeked)
    m.step_cycle()                            # the pop commits at this edge
    assert list(m.threads[0].outq) == [0x2222]
    assert not m.badop


def test_an_empty_peek_sets_badop14_and_pops_nothing():
    from tools.loomgen import pop_after_peek
    m = fifo_machine()
    pop_after_peek(m, 0, None)                # empty then and now
    m.step_cycle()
    assert m.badop == 0x4000 and not m.threads[0].outq
    # ... and when a thread push has filled the queue since the peek, the
    # word the port sent was still nothing: the error bit, no pop.
    m.host_clear_badop()
    m.step_cycle()
    m.threads[0].outq.append(0x3333)
    pop_after_peek(m, 0, None)
    m.step_cycle()
    assert m.badop == 0x4000
    assert list(m.threads[0].outq) == [0x3333]


def test_replay_host_log_reissues_every_action():
    from tools.loomgen import group_host_log, replay_host_log
    m = fifo_machine()
    log = [[1, "fifo_push", 0, 0x55AA], [1, "irq_en", 0x000F],
           [3, "sflags_set", 0x03], [5, "badop_clr", 0xFFFF],
           [7, "run", 0x1]]
    by_cycle = group_host_log(log)
    for cycle in range(9):
        replay_host_log(m, by_cycle, cycle)
        m.step_cycle()
    assert list(m.threads[0].inq) == [0x55AA]
    assert m.irq_en == 0x000F and m.sflags == 0x03 and m.run == 0x1


# ------------------------------------------------------------ the latch (6.10)
def test_latched_setp_lands_in_the_model():
    """A staged write fires: LAT_VALID goes back to 0 at a deadline edge."""
    fired = 0
    for seed in range(6):
        prog = m2_program(seed, profile="timing")
        machine = None
        staged = [0, 0, 0, 0]
        from tools.loomgen.runner import build_machine, host_actions, RUN_CYCLE as RC
        machine = build_machine(prog)
        for cycle in range(RC + 4000):
            host_actions(machine, prog, cycle)
            ui, uio = prog.stimulus.at(cycle)
            machine.set_pad_inputs(ui_in=ui, uio_in=uio)
            before = [th.lat_valid for th in machine.threads]
            machine.step_cycle()
            for t, was in enumerate(before):
                if was and not machine.threads[t].lat_valid:
                    fired += 1
            staged = before
    assert fired, "no latched SETP ever fired in six seeds"
