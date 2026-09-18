# Area and timing log

One row per hardening or synthesis run that matters. Cells = standard cells after synthesis; util = placement utilisation reported by the TT flow.

| Date | Milestone | Config | Cells | Flops | Util | WNS @20ns | Notes |
|---|---|---|---|---|---|---|---|
| 2026-09-17 | M0 | hard-wired UART TX, 6x4, CI run 35244479499 | 214 synth / 314 placed | 33 | 0.52% | +13.22 ns setup, +0.13 ns hold | die 916,214 um2, core 902,417 um2; DRC 0, LVS 0, antenna 0; precheck pass; DFF `dfrbpq_1` = 49.0 um2; LibreLane 3.1.0.dev3 |
| 2026-09-17 | M1, before hardening | Yosys generic `synth`, flop imem 256x16 | 27,662 generic | 6,034 | n/a | n/a | imem 13,911 cells / 4,112 flops (50% of cells, 68% of flops); core pipeline 7,807 / 731; regfile 2,730 / 512; timers 1,723 / 388; ALU 946; host_ctl 826 / 238; decode x2 714; pins 132 / 56; SPI 61 / 35. Logic without imem about 13.7K generic cells. Mapped cell count and utilisation come from the first CI hardening of this commit. |
| 2026-09-18 | M1 hardened | CI run 35272974272, commit 03a1bca, 6x4, flop imem 256x16 | 30,988 synth / 42,559 placed | 6,039 (`dfrbpq_1`) | 78.8% | typ +1.45 ns (0 violations); slow 1.08 V 125 C -9.95 ns (1,084 endpoints, TNS -3,219 ns); fast +7.96 ns; worst hold +0.12 ns | stdcell area 710,760 um2 of 902,417 um2 core; synth area 559,128 um2, of which flops 296K (53%); DRC 0, LVS 0, antenna 0; precheck pass; gl_test pass; 188 max-slew and 17 max-cap violations; wall time: gds 4 h 39 min (detailed routing at 79% on Metal1-Metal4), precheck 3 h 27 min |

Reading of the M1 row: the design fits and is tape-out clean at the sign-off corner, but 79% utilisation leaves no room for the M2 features (FIFOs and bit engines are about 80-100K um2 more), makes each CI hardening take hours, and costs timing margin. The flop instruction memory is about half the cell area, so the M2 memory decision is also the area decision.
