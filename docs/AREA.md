# Area and timing log

One row per hardening or synthesis run that matters. Cells = standard cells after synthesis; util = placement utilisation reported by the TT flow.

| Date | Milestone | Config | Cells | Flops | Util | WNS @20ns | Notes |
|---|---|---|---|---|---|---|---|
| 2026-09-17 | M0 | hard-wired UART TX, 6x4, CI run 35244479499 | 214 synth / 314 placed | 33 | 0.52% | +13.22 ns setup, +0.13 ns hold | die 916,214 um2, core 902,417 um2; DRC 0, LVS 0, antenna 0; precheck pass; DFF `dfrbpq_1` = 49.0 um2; LibreLane 3.1.0.dev3 |
| 2026-09-17 | M1, before hardening | Yosys generic `synth`, flop imem 256x16 | 27,662 generic | 6,034 | n/a | n/a | imem 13,911 cells / 4,112 flops (50% of cells, 68% of flops); core pipeline 7,807 / 731; regfile 2,730 / 512; timers 1,723 / 388; ALU 946; host_ctl 826 / 238; decode x2 714; pins 132 / 56; SPI 61 / 35. Logic without imem about 13.7K generic cells. Mapped cell count and utilisation come from the first CI hardening of this commit. |
| 2026-09-18 | M1 hardened | CI run 35272974272, commit 03a1bca, 6x4, flop imem 256x16 | 30,988 synth / 42,559 placed | 6,039 (`dfrbpq_1`) | 78.8% | typ +1.45 ns (0 violations); slow 1.08 V 125 C -9.95 ns (1,084 endpoints, TNS -3,219 ns); fast +7.96 ns; worst hold +0.12 ns | stdcell area 710,760 um2 of 902,417 um2 core; synth area 559,128 um2, of which flops 296K (53%); DRC 0, LVS 0, antenna 0; precheck pass; gl_test pass; 188 max-slew and 17 max-cap violations; wall time: gds 4 h 39 min (detailed routing at 79% on Metal1-Metal4), precheck 3 h 27 min |

Reading of the M1 row: the design fits and is tape-out clean at the sign-off corner, but 79% utilisation leaves no room for the M2 features (FIFOs and bit engines are about 80-100K um2 more), makes each CI hardening take hours, and costs timing margin. The flop instruction memory is about half the cell area, so the M2 memory decision is also the area decision.

| Date | Milestone | Config | Cells | Flops | Util | Timing | Notes |
|---|---|---|---|---|---|---|---|
| 2026-09-18 | M0.5 SRAM smoke | branch sram-smoke 565673f, run 35377845679, 2x2, 512x16 macro FS at (12, 40) | 494 stdcells + 1 macro | (tester only) | 41.5% | setup +11.07 slow / +11.32 typ / +11.46 fast; hold +0.124 worst | macro 45,309 um2 of 126,685 um2 core; stdcells 7,203 um2; hardening about 5 min; precheck 9/9 incl. KLayout SG13CMOS5L DRC 0 violations; gl_test 5/5; recipe in facts section 11 |
| 2026-09-19 | M2 + macro hardened | branch macro-core d41fe34 (M2 RTL of 52ee9f8 + D-020/D-021), run 35419160398, 6x4, macro FS at (12, 40) | 28,825 stdcells + 1 macro (75,908 instances with fill and taps) | 3,027 | 51.5% (stdcells 48.9%) | typ +6.21 ns; fast +11.07 ns; slow -2.07 ns (23 endpoints, TNS -26.9 ns); hold +0.28 typ / +0.61 slow / +0.10 fast, 0 violations | stdcell area 419,165 um2 + macro 45,309 um2 of 902,417 um2 core; DRC 0, LVS 0, antenna 0; precheck pass; gl_test 14/15 (a test bug, BUGS 4; 15/15 locally with the fix); max-slew 156 slow / 57 typ, max-cap 28-29; Magic illegal overlaps 10 (the smoke test's four stripe crossings, two boxes each, facts 12); worst path launched by the macro +2.07 ns slow; power 11.8 mW typ; wall time gds 3 h 53 min, precheck about 2 h |

Reading of the macro row: the macro freed a third of the core (stdcell area 710,760 -> 419,165 um2) and the whole typical-corner margin came back (+1.45 -> +6.21 ns). The slow corner is down to two paths, both far more buffering than logic: 22 endpoints behind the deadline-latch fire logic, whose host thread decode went through `buf_1` fanout buffers with slews up to 1.85 ns ahead of a 16-bit subtract (D-022 moves the decode behind the subtract), and one `tx_th` path through a series chain of six `buf_1` (-0.26 ns). The flow's settings explain the weak buffering: `SYNTH_STRATEGY` "AREA 0", `MAX_FANOUT_CONSTRAINT` 10, `RUN_POST_GRT_RESIZER_TIMING` false, `PL_TIMING_DRIVEN` false; only the typical corner is sign-off (`TIMING_VIOLATION_CORNERS` `*typ*`). Knobs to try if slow-corner closure is wanted: a DELAY synthesis strategy, post-GRT resizer timing, timing-driven placement (each costs one 4-hour hardening to evaluate, and `src/config.json` changes need a DECISIONS entry).

### M2 RTL, Yosys generic synthesis (not hardened), 2026-09-18

| Step | Flat cells | Flops | Longest path (ltp, generic cells) | Notes |
|---|---|---|---|---|
| M1 baseline | 27,560 | 6,039 | 48 | timer compare path |
| D-019 one-hot W rings | 26,966 | 6,076 | 48 | core 7,807 -> 6,512 cells; thread-select cone depth 22 -> 5, fanout 308 -> 33 |
| + FIFOs | 29,240 | 6,695 | 48 | +2,274 cells, +619 flops |
| + IRQ fix | 29,323 | 6,702 | 49 | |
| + bit engine (manual) | 31,917 | 7,099 | 48 | loom_be 792 cells |
| + deadline-latched SETP, VERSION 2 | 33,425 | 7,138 | 63 | watch: the latch-fire compare (NOW against the next TD) is a new deep path; imem is still 13,905 of the cells until the macro replaces it |
| + SRAM macro instruction memory, 512 words (D-020, branch macro-core) | 19,456 (19,455 + the macro) | 3,027 | 61 | the macro as a blackbox (read like LibreLane reads the MACROS `nl` view): -13,969 cells, -4,111 flops against the line above (4,096 array bits and 16 read-data flops go; the 9th host write-address bit adds one); `loom_imem_macro` is 11 generic cells (address mux, `~we`, the macro); flattened instance `u_loom.u_imem.g_macro.u_macro.sram` |
| main after the macro-core merge (06bf925, adds the TICK_SEEN fix) | 19,493 | 3,028 | 62 | |
| + D-022 latch-fire restructure | 19,921 | 3,028 | 60 | three subtractors per thread instead of one; the TD' mux and `tick` move behind them |

