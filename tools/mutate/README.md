# `tools/mutate` — mutation testing (VERIFICATION L7)

Injects one-line faults into `src/*.v`, runs the test suite on each, and reports
which were killed and which survived. This is MUT-RUN of
[`docs/VERIFICATION.md`](../../docs/VERIFICATION.md); MUT-TARGET wants at least
90 per cent killed on `loom_core`, `loom_timer`, `loom_pins`, `loom_be`,
`loom_fifo` and `loom_spi_host`, with every survivor either documented as an
equivalent mutant or answered with a new check.

**`src/` is never written to.** A mutant is a copy of `src/` in a scratch
directory with one line changed, compiled through `make SRC_DIR=<copy>` — the
same trick the hand-written mutants in [`test/README.md`](../../test/README.md)
use. No code here opens a file under `src/` for writing, and a checkout refuses
a mutation whose path is not repo-relative, so the real tree cannot be reached
by a stray absolute path either.

## Running it

From a WSL shell that has the OSS CAD Suite on PATH:

```sh
source scripts/dev_env.sh
"$LOOM_PY" -m tools.mutate list                          # what would run
"$LOOM_PY" -m tools.mutate run --jobs 12 --out mut.jsonl # the run itself
"$LOOM_PY" -m tools.mutate report mut.jsonl              # the tables
"$LOOM_PY" -m tools.mutate show loom_fifo_L0031          # one mutant, in context
```

Useful options for `run`:

| Option | Meaning |
|---|---|
| `--modules a,b` | which `src/*.v` to mutate (default: the six of MUT-TARGET) |
| `--operators arith,relop` | only these operators |
| `--sample N` / `--core-sample N` | at most N mutants per module; `loom_core` has far more than the rest, so it has its own cap |
| `--seed S` | which sample. Sampling is a hash order, not a shuffle: raising `--sample` only adds mutants, so a run can be deepened with `--resume` |
| `--jobs N` | mutants at a time (start with 4; 12 is comfortable on 16 cores) |
| `--resume` | skip mutants already in `--out` |
| `--only id,id` | keep only these mutants (an id prefix is enough), for re-running one survivor |
| `--rungs a,b` | run only part of the ladder, e.g. `--rungs lint` to size a run |
| `--work` / `--build` | scratch roots (default `~/loom_mutate_work`, `~/loom_mutate_build`; keep them off `/mnt/c`, it is slow) |
| `--no-baseline` | skip the sanity run of the ladder on the unmutated source |
| `--strict` | exit 1 if a survivor is not documented in `equivalents.json` (the gate a nightly job would use) |

Results are appended to `--out` as JSON lines while the run goes, so an
interrupted run loses nothing and `--resume` picks it up.

## The operators

Each mutant changes exactly one line. Two operators that produce the same text
are one mutant.

| Operator | What it does |
|---|---|
| `arith` | `+` <-> `-` (never the `+:` / `-:` of a part-select) |
| `relop` | `<` <-> `<=`, `>` <-> `>=`; a `<=` that is a nonblocking assignment is left alone |
| `eqop` | `==` <-> `!=` |
| `bitop` | `&` <-> `\|`, `&&` <-> `\|\|`, including the reduction forms |
| `cond_inv` | `if (X)` -> `if (!(X))` |
| `const` | a numeric literal that is not an index, +/- 1 in its own width and base (`4'd7` -> `4'd8`, `16'd0` -> `16'd65535`, `1'b0` -> `1'b1`) |
| `index` | a constant inside a bit-select or part-select, +/- 1 (`d[15]` -> `d[14]`, `now_all[16*t +: 16]` -> `[15*t +: 16]`) |
| `stuck` | a one-bit control signal inside an `if` condition forced to `1'b0` or `1'b1` |

The mutator is line-oriented and parses no more Verilog than it must
(`operators.py`). A lexical pass blanks comments, string literals, compiler
directives and anything an `ifdef` disables, so no operator can fire inside
one. A token pass then tracks just enough context to tell a nonblocking
assignment from a comparison, a bit-select from a declared width, and a one-bit
signal from a vector. Declared widths and array bounds are deliberately *not*
mutated: resizing a register is a different fault class and every one of them
dies in the linter, which would only flatter the kill rate.

`tools/tests/test_mutate_operators.py` is the mutator's own test: given a line,
exactly these mutants.

## The check ladder

Each mutant runs the checks below in order and stops at the first one that
fails; that check is what killed it. Cheapest first, so a mutant that dies
early costs seconds and only a survivor pays for the whole suite.

| Rung | What runs | Cost on the unmutated design |
|---|---|---|
| `lint` | `verilator --lint-only -Wall -Isrc --top-module tt_um_loom src/*.v`, the same command as `scripts/check_all.sh` | 0.2 s |
| `cosim-short` | `test_cosim`, 3 seeds x 2500 cycles, no host-port seed | 5 s |
| unit (own) | the L1 modules closest to the mutated file (`test_fifo` for `loom_fifo.v`, and so on) | 6 to 80 s each |
| `cosim-deep` | `test_cosim`, 6 seeds x 4000 cycles plus one seed over the SPI host port | 25 s |
| unit (rest) | the remaining L1 modules, cheapest first | about 4 minutes in total |

That is L1 plus L2 of `docs/VERIFICATION.md`, which is what MUT-RUN asks for.
The L3 firmware suite (`test_fw`, about two minutes) and the FLOPS-memory
variant (`test_flops`) are not in the ladder.

A rung that outruns its timeout counts as killed by that rung: the mutant made
the design hang, which is a failure.

**Lint kills count, and are reported separately.** CI's `lint` job runs
Verilator with `-Wall` and fails on a warning, so a mutant that only trips the
linter is genuinely caught. It still says nothing about the tests, so the
report gives both the overall kill rate and the rate among the mutants that got
past the linter ("killed in simulation").

## Survivors and equivalent mutants

Survivors are the output that matters. MUT-TARGET allows one only if it is an
equivalent mutant *and* the reason is written down, so the reasons live in
`equivalents.json`: mutant id -> why the change is not observable. The report
then marks each survivor `equivalent - <reason>` or **open**, and gives a kill
rate with the equivalents taken out of the denominator next to the raw one.

The key is the mutant id, whose last field is a digest of the mutated line. An
entry therefore stops applying the moment that line of RTL changes, and the
claim has to be made again rather than being inherited by a different mutant.

An **open** survivor is a hole in the suite, not a tidy-up job: it wants a new
check, and the bug ledger rule in `CLAUDE.md` applies to what the new check
finds.

## What a run costs

Measured on Thomas's laptop (WSL Ubuntu, 16 cores, Icarus, scratch on ext4):
the whole ladder on the unmutated design is 319 s in series. A mutant that dies
at the linter therefore costs 0.6 s, one that dies in the short co-simulation
about 6 s, and a survivor the whole 319 s — nearer 600 s with twelve of them
sharing the machine. The total is dominated by the survivors, not the count.
See `docs/VERIFICATION.md` L7 for the first run's numbers.

Two traps worth knowing if a run has to be restarted:

* keep the work and build roots off `/mnt/c` (the default `~/…` is right);
  they are hard-link copies of one template, so a checkout is milliseconds,
  and 800 of them cost almost no disk;
* a run detached with `setsid nohup` inside `wsl -- bash -c '…'` is killed the
  moment that `wsl` invocation returns. Start long runs from a shell that
  stays alive.

## Files

| File | Contents |
|---|---|
| `operators.py` | the mutator: `Mutation`, `mutations_for_text`, `mutations_for_file` |
| `runner.py` | work directories, the ladder, the thread pool, `MutantResult` |
| `report.py` | the kill-rate, per-operator, per-check and survivor tables |
| `equivalents.json` | the documented equivalent mutants, id -> reason |
| `__main__.py` | the command line |

Its own tests are `tools/tests/test_mutate_operators.py` (the operators) and
`tools/tests/test_mutate_runner.py` (the ladder, the results parsing, the
checkout, the sampling, the report). Neither needs a simulator, so
`scripts/check_all.sh` runs both through `pytest -q tools`.
