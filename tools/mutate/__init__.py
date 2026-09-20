"""
tools.mutate: the L7 mutation tool of `docs/VERIFICATION.md` (MUT-RUN).

`operators` makes one-line mutants of the Verilog in `src/`; `runner` builds
each one in a scratch copy and runs the check ladder on it; `report` turns the
results into the kill-rate and survivor tables. `python -m tools.mutate --help`
is the command line. Nothing here writes to `src/`.
"""

from tools.mutate.operators import (  # noqa: F401
    OPERATORS,
    Mutation,
    mutations_for_file,
    mutations_for_text,
)

__all__ = ["OPERATORS", "Mutation", "mutations_for_file", "mutations_for_text"]
