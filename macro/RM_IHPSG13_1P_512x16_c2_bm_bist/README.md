# RM_IHPSG13_1P_512x16_c2_bm_bist

IHP foundry-provided 512 x 16 single-port SRAM macro with bit mask and BIST
(1 kByte). Vendored here so the Tiny Tapeout `gds` workflow can harden against
a fixed copy instead of whatever the PDK checkout happens to contain.

## Source

- **Repository**: <https://github.com/IHP-GmbH/IHP-Open-PDK>
- **Commit**: `2bbec755dc67ca3db0261c3d6163e15735d66710`
  (the commit the Tiny Tapeout cmos5l flow pins; see `docs/tt_cmos5l_facts.md`)
- **Path**: `ihp-sg13g2/libs.ref/sg13g2_sram/{gds,lef,lib,cdl,verilog}/`

The cmos5l PDK does not carry its own SRAM: at this commit
`ihp-sg13cmos5l/libs.ref/sg13cmos5l_sram` is a symlink to
`../../ihp-sg13g2/libs.ref/sg13g2_sram`, so these are literally the SG13G2
macros re-exported. They have **never been taped out on cmos5l**; that is the
whole point of this branch.

## Files

| File | Bytes | What it is |
|---|---|---|
| `RM_IHPSG13_1P_512x16_c2_bm_bist.gds` | 466,654 | layout, streamed into the final GDS |
| `RM_IHPSG13_1P_512x16_c2_bm_bist.lef` | 74,001 | abstract: `SIZE 236.8 BY 191.34`, `CLASS BLOCK` |
| `RM_IHPSG13_1P_512x16_c2_bm_bist.cdl` | 403,966 | transistor netlist, for LVS |
| `RM_IHPSG13_1P_512x16_c2_bm_bist_typ_1p20V_25C.lib` | 49,142 | nominal corner |
| `RM_IHPSG13_1P_512x16_c2_bm_bist_fast_1p32V_m55C.lib` | 49,139 | fast corner (`min_*`) |
| `RM_IHPSG13_1P_512x16_c2_bm_bist_slow_1p08V_125C.lib` | 49,203 | slow corner (`max_*`) |
| `RM_IHPSG13_1P_512x16_c2_bm_bist.v` | 7,504 | simulation model (wrapper + `specify`) |
| `RM_IHPSG13_1P_core_behavioral_bm_bist.v` | 3,951 | `SRAM_1P_behavioral_bm_bist`, the model the wrapper instantiates |

Sizes are the byte counts reported by the GitHub contents API at that commit
and match the downloaded files exactly. The GDS is 0.47 MB, far under GitHub's
50 MB limit. `.gitattributes` already carries `*.gds binary`.

Both Verilog files are needed for simulation: the wrapper `.v` instantiates
`SRAM_1P_behavioral_bm_bist`, which lives in the `_core_behavioral_bm_bist.v`
file. Compile the model with `-DFUNCTIONAL`, otherwise the wrapper takes the
`specify`/timing-check path instead of the behavioural one.

The model is **not** a synthesis source. It is referenced from the `MACROS`
block in `src/config.json` and a port-only blackbox
(`src/RM_IHPSG13_1P_512x16_c2_bm_bist.v`) stands in for it during synthesis,
exactly as `tt_um_urish_sram_test` does.

## Physical summary (read out of the LEF, not from memory)

- `SIZE 236.8 BY 191.34`, `SYMMETRY X Y R90`, `ORIGIN 0 0`, `CLASS BLOCK`.
- Only `Metal1..Metal4` and `Via2` geometry: nothing above Metal4, so the macro
  fits the five-metal cmos5l stack (Metal1..Metal4 + TopMetal1).
- Power pins are **vertical Metal4 stripes, 2.81 um wide**:
  - `VSS!` (GROUND): 20 stripes, full height (y 0 .. 191.34).
  - `VDD!` (POWER): 20 stripes. 16 of them only span y 0 .. 38.825; the four in
    the middle (x 104.12, 114.42, 119.57, 129.87) are full height.
  - `VDDARRAY!` (POWER): 16 stripes at the same x as the 16 short `VDD!` ones,
    spanning y 45.465 .. 191.34.
  - So outside the middle band, one x column carries `VDD!` at the bottom and
    `VDDARRAY!` at the top with a 6.64 um gap between them.
- Column pitch: **11.24 um between same-net stripes, 5.62 um between a POWER
  column and the GROUND column next to it** — over the left part of the macro
  (VDD x0 = 4.26 + 11.24k, VSS x0 = 9.88 + 11.24k, k = 0..7) and again over the
  right part (VDD x0 = 151.05 + 11.24k, VSS x0 = 145.43 + 11.24k). The right
  part is shifted **+0.67 um** relative to the left part's grid, and the middle
  (x 88 .. 151) is irregular. `src/pdn_cfg.tcl` explains what that costs.
- The Metal4 `OBS` fills every gap between the power columns, leaving only
  0.26 um of clear space either side of each pin and declaring
  `SPACING 0.21`. A Metal4 PDN stripe can therefore only cross this macro if it
  sits inside a power pin column.

## Licence

Apache License 2.0. The files carry the IHP PDK Authors copyright header
(`Copyright 2025 IHP PDK Authors` / `Copyright 2023 IHP PDK Authors`) and the
Apache-2.0 notice; the upstream repository's `LICENSE` is Apache-2.0 as well.
No modifications were made to any vendored file.
