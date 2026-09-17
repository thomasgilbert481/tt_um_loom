# Bug ledger

Every bug found by verification gets a row. Newest at the bottom.

| # | Date | Module | Symptom | Root cause | Found by (check ID) | Now covered by |
|---|---|---|---|---|---|---|
| 1 | 2026-09-15 | tt_um_loom (M0) | Verilator -Wall WIDTHEXPAND on comparisons against integer localparams | unsized compare | L0-LINT | zero-extended compare operands; lint in CI |
| 2 | 2026-09-17 | test/Makefile (template) | CI `gl_test` fails to compile: `Unknown module type: ihp_dff_r` in `sg13cmos5l_stdcell.v` | the cmos5l template Makefile omits the PDK primitives file `sg13cmos5l_udp.v`; design not involved | PHY-GL (first CI run) | UDP file added to the GL sources; `gl_test` runs on every push |
