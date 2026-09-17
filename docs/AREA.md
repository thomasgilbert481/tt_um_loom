# Area and timing log

One row per hardening or synthesis run that matters. Cells = standard cells after synthesis; util = placement utilisation reported by the TT flow.

| Date | Milestone | Config | Cells | Flops | Util | WNS @20ns | Notes |
|---|---|---|---|---|---|---|---|
| 2026-09-17 | M0 | hard-wired UART TX, 6x4, CI run 35244479499 | 214 synth / 314 placed | 33 | 0.52% | +13.22 ns setup, +0.13 ns hold | die 916,214 um2, core 902,417 um2; DRC 0, LVS 0, antenna 0; precheck pass; DFF `dfrbpq_1` = 49.0 um2; LibreLane 3.1.0.dev3 |
