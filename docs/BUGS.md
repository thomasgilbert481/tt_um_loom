# Bug ledger

Every bug found by verification gets a row. Newest at the bottom.

| # | Date | Module | Symptom | Root cause | Found by (check ID) | Now covered by |
|---|---|---|---|---|---|---|
| 1 | 2026-09-15 | tt_um_loom (M0) | Verilator -Wall WIDTHEXPAND on comparisons against integer localparams | unsized compare | L0-LINT | zero-extended compare operands; lint in CI |
| 2 | 2026-09-17 | test/Makefile (template) | CI `gl_test` fails to compile: `Unknown module type: ihp_dff_r` in `sg13cmos5l_stdcell.v` | the cmos5l template Makefile omits the PDK primitives file `sg13cmos5l_udp.v`; design not involved | PHY-GL (first CI run) | UDP file added to the GL sources; `gl_test` runs on every push |
| 3 | 2026-09-18 | SEMANTICS 4 (spec), loom_timer, loomsim | `WAITB 3` could miss ticks indefinitely: at a tick period of 41 clocks a `WAITB 3; SETP` loop showed gaps of 84 | the rule cleared TICK_SEEN at every slot's commit edge, losing a tick that landed at edge x+1, between the slot's X cycle and its commit | RTL implementation review (docs/spec-questions/rtl-m2.md 4), confirmed by test_fifo against a literal model of the old rule | rule now clears only what the slot saw in X; RTL and golden model changed in one commit; test_fifo.test_waitb_tick_seen asserts no gap exceeds period + 4; co-simulation compares TICK_SEEN |
