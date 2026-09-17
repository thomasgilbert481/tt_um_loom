# Area and timing log

One row per hardening or synthesis run that matters. Cells = standard cells after synthesis; util = placement utilisation reported by the TT flow.

| Date | Milestone | Config | Cells | Flops | Util | WNS @20ns | Notes |
|---|---|---|---|---|---|---|---|
| 2026-09-17 | M0 | hard-wired UART TX, 6x4, CI run 35244479499 | 214 synth / 314 placed | 33 | 0.52% | +13.22 ns setup, +0.13 ns hold | die 916,214 um2, core 902,417 um2; DRC 0, LVS 0, antenna 0; precheck pass; DFF `dfrbpq_1` = 49.0 um2; LibreLane 3.1.0.dev3 |
| 2026-09-17 | M1, before hardening | Yosys generic `synth`, flop imem 256x16 | 27,662 generic | 6,034 | n/a | n/a | imem 13,911 cells / 4,112 flops (50% of cells, 68% of flops); core pipeline 7,807 / 731; regfile 2,730 / 512; timers 1,723 / 388; ALU 946; host_ctl 826 / 238; decode x2 714; pins 132 / 56; SPI 61 / 35. Logic without imem about 13.7K generic cells. Mapped cell count and utilisation come from the first CI hardening of this commit. |
