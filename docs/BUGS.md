# Bug ledger

Every bug found by verification gets a row. Newest at the bottom.

| # | Date | Module | Symptom | Root cause | Found by (check ID) | Now covered by |
|---|---|---|---|---|---|---|
| 1 | 2026-09-15 | tt_um_loom (M0) | Verilator -Wall WIDTHEXPAND on comparisons against integer localparams | unsized compare | L0-LINT | zero-extended compare operands; lint in CI |
