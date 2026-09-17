# Tiny Tapeout on IHP CMOS5L — verified fact sheet

Research date: 2026-09-15. Every fact below has a source URL. Anything not found is
marked **NOT FOUND**. Items marked **[DERIVED]** are arithmetic on cited numbers, not
quoted values.

Pinned reference points used throughout:

- TT GDS action branch `ihp-cmos5l`, HEAD `3412659` (2026-09-14) — <https://github.com/TinyTapeout/tt-gds-action/tree/ihp-cmos5l>
- `tt-support-tools` branch **`ihp-sg13cmos5l`** (note: *not* `ihp-cmos5l`), HEAD `da63c99` (2026-03-29) — <https://github.com/TinyTapeout/tt-support-tools/tree/ihp-sg13cmos5l>
- `tt-multiplexer` branch `ihp-sg13cmos5l`, HEAD `3e49649` (2026-03-28) — <https://github.com/TinyTapeout/tt-multiplexer/tree/ihp-sg13cmos5l>
- IHP-Open-PDK pinned commit `2bbec755dc67ca3db0261c3d6163e15735d66710` (dev branch, 2026-09-08)

---

## 1. What is IHP CMOS5L / `sg13cmos5l`?

### Identity and status

- The IHP Open PDK repo hosts it alongside SG13G2: *"As of March 2023, this repository is
  targeting the SG13G2 process node. It also hosts the SG13CMOS5L process node, a CMOS-only
  variant with a reduced metal stack."*
  <https://raw.githubusercontent.com/IHP-GmbH/IHP-Open-PDK/2bbec755dc67ca3db0261c3d6163e15735d66710/README.md>
- Status is **preview, not production**: *"IHP is currently treating the existing content as a
  preview only … the open source PDK is not intended to be used for production at this moment.
  The same applies to the SG13CMOS5L process node."* (same README)
- **Public? Yes, but only on the `dev` branch.** At the pinned dev commit the repo root contains
  `ihp-sg13cmos5l/`, `ihp-sg13g2/`, `ihp-common/`, `Makefile.sg13cmos5l`, `Makefile.sg13g2`.
  The **`main` branch root contains only `ihp-sg13g2`** — no `ihp-sg13cmos5l` directory.
  (verified via GitHub contents API on both refs, repo <https://github.com/IHP-GmbH/IHP-Open-PDK>)
- A separate staging repo exists: <https://github.com/IHP-GmbH/ihp-sg13cmos5l> — its README says it
  is *"meant to be used **only** as a temporary storage during the development of the
  `build/compile` migration script for the sg13 cmos5l PDK"*, titled around the **M1-M4-TM1 stack**,
  Apache-2.0.
- Standard-cell library release notes: Technology `SG13CMOS5L`, Revision `rev0.1.4`, dated
  **17 Feb 2026**. *"This is the initial release of the `sg13cmos5l_stdcell` digital library. The
  library has been migrated from the library `sg13g2_stdcell`."*
  <https://raw.githubusercontent.com/IHP-GmbH/IHP-Open-PDK/2bbec755dc67ca3db0261c3d6163e15735d66710/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/doc/ReleaseNotes.txt>
- The IHP Open PDK **documentation site does not mention SG13CMOS5L at all** (process specs page
  covers only SG13S and SG13G2): <https://ihp-open-pdk-docs.readthedocs.io/en/main/process_specs/01_general.html>

### Where the TT GDS action gets the PDK

`install_sg13cmos5l.sh` in `tt-gds-action@ihp-cmos5l`
(<https://github.com/TinyTapeout/tt-gds-action/blob/ihp-cmos5l/install_sg13cmos5l.sh>):

```bash
IHP_PDK_REPO="https://github.com/IHP-GmbH/IHP-Open-PDK.git"
# IHP-Open-PDK dev branch, 2026-09-08 ("Merge pull request #1219 from IHP-GmbH/feat/chipText")
IHP_PDK_REV="2bbec755dc67ca3db0261c3d6163e15735d66710"
```

It shallow-fetches that exact commit into `$PDK_ROOT` and writes `$PDK_ROOT/ihp-sg13cmos5l/SOURCES`.
Note this is a **single pinned commit of the dev branch**, not a release. It was changed on
2026-09-14 because the older script cloned both IHP-Open-PDK and the standalone `ihp-sg13cmos5l`
repo and broke once the dev branch absorbed the directory — see issue
<https://github.com/TinyTapeout/tt-gds-action/issues/52> (opened 2026-09-11, urish: *"Should be
fixed now."*).

### Differences vs sg13g2 that matter to a digital designer

| Item | `sg13cmos5l` | `sg13g2` | Source |
|---|---|---|---|
| Routing layers in tech LEF | `Metal1, Via1, Metal2, Via2, Metal3, Via3, Metal4, TopVia1, TopMetal1` (**5 metals**) | `Metal1…Metal4, Via4, Metal5, TopVia1, TopMetal1, TopVia2, TopMetal2` (**7 metals**) | tech LEFs at pinned commit: `.../ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/lef/sg13cmos5l_tech.lef` vs `.../ihp-sg13g2/libs.ref/sg13g2_stdcell/lef/sg13g2_tech.lef` |
| Cells in `.celllist` | **84** | **84** | `doc/sg13cmos5l_stdcell.celllist` / `doc/sg13g2_stdcell.celllist` at pinned commit |
| Cell families tracked by TT | 49 base cells across 15 categories | same set | <https://github.com/TinyTapeout/tt-support-tools/blob/ihp-sg13cmos5l/tech/ihp-sg13cmos5l/cells.json> and `categories.json` |
| Row height (site) | 3.78 µm | 3.78 µm | LEF `SIZE` lines, both libs |
| **DFF `dfrbp_1`** | `SIZE 13.92 BY 3.78` = **52.62 µm²** [DERIVED] | `SIZE 13.92 BY 3.78` — identical | `sg13cmos5l_stdcell.lef` / `sg13g2_stdcell.lef` |
| **Scan DFF `sdfrbp_1`** | `SIZE 18.24 BY 3.78` = **68.95 µm²** [DERIVED] | identical | same |
| `inv_1` / `nand2_1` / `nor2_1` / `buf_1` | 1.44 / 1.92 / 1.92 / 1.92 × 3.78 µm | identical | same |
| `mux2_1` | 4.80 × 3.78 µm | identical | same |
| `xor2_1` / `xnor2_1` | 3.84 × 3.78 µm | identical | same |
| Liberty corners shipped | `typ_1p20V_25C`, `typ_1p50V_25C`, `fast_1p32V_m40C`, `fast_1p65V_m40C`, `slow_1p08V_125C`, `slow_1p35V_125C` | — | `.../sg13cmos5l_stdcell/doc/` listing at pinned commit |
| Corner TT signs off at | `nom_typ_1p20V_25C` | `nom_typ_1p20V_25C` | `tech.py` `IHPTech.tt_corner` on `ihp-sg13cmos5l` branch |

**Bottom line: the cell library is byte-for-byte the same geometry as sg13g2** (it was migrated
from it). The real delta for a digital designer is the **metal stack** and, downstream of that,
what TT lets you route on:

- On the cmos5l branch `tech.py` sets `project_top_metal_layer = "Metal4"`, which becomes
  `RT_MAX_LAYER: Metal4` in the generated LibreLane config.
  <https://github.com/TinyTapeout/tt-support-tools/blob/ihp-sg13cmos5l/tech.py>
  On the sg13g2 (`main`) branch the same field is `"TopMetal1"`.
  <https://github.com/TinyTapeout/tt-support-tools/blob/main/tech.py>
  → **user designs get one fewer routing layer than on the sg13g2 shuttles.** TopMetal1 is taken by
  the mux/PDN spine.
- `librelane_pdk_args = "--pdk ihp-sg13cmos5l --manual-pdk"` — the `--manual-pdk` flag is
  cmos5l-specific (sg13g2 branch is plain `--pdk ihp-sg13g2`). Same file.

Max clock as a *process* number: **NOT FOUND**. Nothing in the PDK or TT docs states a CMOS5L
fmax. See §4 for the system-level limit.

---

## 2. Tile dimensions and valid `tiles` values

### Authoritative table (`DIE_AREA`, µm, `llx lly urx ury`)

<https://github.com/TinyTapeout/tt-support-tools/blob/ihp-sg13cmos5l/tech/ihp-sg13cmos5l/tile_sizes.yaml>

```yaml
1x1: "0 0 202.08 154.98"
1x2: "0 0 202.08 313.74"
2x1: "0 0 419.52 154.98"
2x2: "0 0 419.52 313.74"
3x1: "0 0 636.96 154.98"
3x2: "0 0 636.96 313.74"
3x4: "0 0 636.96 710.64"
4x1: "0 0 854.40 154.98"
4x2: "0 0 854.40 313.74"
4x4: "0 0 854.40 710.64"
5x4: "0 0 1071.84 710.64"
6x1: "0 0 1289.28 154.98"
6x2: "0 0 1289.28 313.74"
6x4: "0 0 1289.28 710.64"
8x1: "0 0 1724.16 154.98"
8x2: "0 0 1724.16 313.74"
```

- **1×1 tile = 202.08 µm (W) × 154.98 µm (H) = 31,318 µm²** [DERIVED area].
  Jane Street's own figure agrees: *"At approximately 200um × 150um per tile"*
  <https://blog.janestreet.com/protocol-emulator-asic-competition/>
- Orientation: the **first number is columns (X / width), the second is rows (Y / height)**.
  So `8x4` would mean 8 tiles wide × 4 tiles tall. Confirmed by the table: `8x1` is
  1724.16 wide × 154.98 tall.
- X pitch between tiles is 217.44 µm and the 1×1 block is 202.08 µm wide, i.e. ~15.36 µm of
  inter-tile gap [DERIVED from the table rows].

### Is `8x4` valid for the cmos5l flow? **No — not today.**

- `tile_sizes.yaml` (above) tops out at **`8x2` (1724.16 × 313.74 µm = 540,938 µm² [DERIVED])**
  and **`6x4` (1289.28 × 710.64 µm = 916,214 µm² [DERIVED])**. There is **no `8x4` key**.
- Validation is purely "is this key in `tile_sizes`":
  ```python
  elif tiles not in tile_sizes.keys():
      errors.append(f"Invalid value for 'tiles' in 'project' section: {tiles}")
  ```
  <https://github.com/TinyTapeout/tt-support-tools/blob/ihp-sg13cmos5l/project_info.py>
- The flow also needs a matching floorplan DEF at
  `tech/ihp-sg13cmos5l/def/tt_block_<tiles>_pgvdd.def`
  (`project.py`, `create_user_config`: `def_template = f"dir::../tt/tech/{self.pdk}/def/tt_block_{tiles}_{def_suffix}.def"`).
  <https://github.com/TinyTapeout/tt-support-tools/blob/ihp-sg13cmos5l/project.py>
  DEFs present: `1x1, 1x2, 2x2, 3x2, 3x4, 4x2, 4x4, 5x4, 6x2, 6x4, 8x2`.
  <https://github.com/TinyTapeout/tt-support-tools/tree/ihp-sg13cmos5l/tech/ihp-sg13cmos5l/def>
  → **`2x1`, `3x1`, `4x1`, `6x1`, `8x1` pass `info.yaml` validation but have no DEF and will fail
  hardening.** Practically usable set = the 11 DEF names above.
- The template's `info.yaml` comment is **stale and wrong** — it says
  `# How many tiles your design occupies? A single tile is about 167x108 uM.` and
  `tiles: "1x1"  # Valid values: 1x1, 1x2, 2x2, 3x2, 4x2, 6x2 or 8x2`.
  Those are sky130 numbers and an out-of-date list (it omits 3x4/4x4/5x4/6x4).
  <https://github.com/TinyTapeout/ttihp-verilog-template/blob/cmos5l/info.yaml>
- Jane Street is explicit that 8x4 is aspirational:
  > *"The current maximum area is 6x4 tiles per design. We are working on the possibility of
  > scaling up to 8x4 tiles (~30% more area)."*
  > *"An 6x4 allocation is 24 tiles. At approximately 200um × 150um per tile, that's about
  > 0.7 mm² of nominal tile area."*
  <https://blog.janestreet.com/protocol-emulator-asic-competition/>
- [DERIVED] If an `8x4` DEF is ever added on the same pitch it would be
  1724.16 × 710.64 µm = 1,225,257 µm², i.e. **+33.7 % over 6x4** — consistent with JS's "~30 % more".

### Chip-level context (mux config)

<https://github.com/TinyTapeout/tt-multiplexer/blob/ihp-sg13cmos5l/cfg/ihp-sg13cmos5l.yaml>

- Die 3,600,000 × 5,000,000 (DB units) = **3.60 mm × 5.00 mm**; margins 445 µm L/R, 460 µm T/B.
- `site: width 480, height 3780` → 0.48 µm × 3.78 µm.
- `tt.grid: x: 12, y: 20`.
- Routing tracks declared only for `Metal1…Metal4, TopMetal1` — again confirming the 5-metal stack.
- `spine: vlayer Metal4, hlayer Metal3`.
- Power gates named `tt_pg_1v5` / `tt_pg_3v3` (identical naming to the sg13g2 config in the same repo).

### Real usage on the first CMOS5L shuttle

`ttihp0p4` is described on its run page as *"sg13cmos5l 130nm open source PDK"*, an experimental
shuttle opened 2026-03-27, closed 2026-03-28 — <https://tinytapeout.com/runs/ttihp0p4/>.
From the machine-readable index <https://index.tinytapeout.com/ttihp0p4.json> (40 projects):

- tiles histogram: `1x1`:20, `1x2`:7, `2x2`:7, `3x2`:2, `3x4`:1, `4x4`:1, `6x2`:2.
- **No project used 8x2, 6x4 or 5x4.** So the large tile sizes are untested in cmos5l silicon.

---

## 3. SRAM on IHP through Tiny Tapeout

### Does an SRAM macro exist for `sg13cmos5l`? — **Yes, by symlink to the SG13G2 macros.**

At the pinned PDK commit, `ihp-sg13cmos5l/libs.ref/` contains:

```
dir      sg13cmos5l_io
symlink  sg13cmos5l_sram  ->  ../../ihp-sg13g2/libs.ref/sg13g2_sram
dir      sg13cmos5l_stdcell
```

(verified via GitHub contents API; the symlink target string is literally
`../../ihp-sg13g2/libs.ref/sg13g2_sram`). Repo: <https://github.com/IHP-GmbH/IHP-Open-PDK>

So there is **no separately characterized CMOS5L SRAM** — the CMOS5L PDK simply re-exports the
SG13G2 `RM_IHPSG13_*` macros. Encouragingly for the 5-metal stack, the `RM_IHPSG13_1P_1024x8`
**LEF abstract only declares Metal1, Metal2, Metal3, Metal4 and Via2 geometry — no Metal5,
no TopMetal** (layer histogram from
<https://raw.githubusercontent.com/IHP-GmbH/IHP-Open-PDK/2bbec755dc67ca3db0261c3d6163e15735d66710/ihp-sg13g2/libs.ref/sg13g2_sram/lef/RM_IHPSG13_1P_1024x8_c2_bm_bist.lef>:
Metal2 ×262, Metal4 ×33, Metal3 ×22, Via2 ×8, Metal1 ×1).
**Caveat: that is the LEF abstract only. Whether the macro *GDS* is clean against CMOS5L DRC
(and whether the bitcell layers exist in CMOS5L at all) is NOT FOUND — nobody has published a
CMOS5L SRAM tape-out.** No SRAM project appears on `ttihp0p4` (index checked, §2).

### SG13G2 SRAM macro sizes — `SIZE` lines read directly from each `.lef`

All from `https://raw.githubusercontent.com/IHP-GmbH/IHP-Open-PDK/2bbec755dc67ca3db0261c3d6163e15735d66710/ihp-sg13g2/libs.ref/sg13g2_sram/lef/<NAME>.lef`

**Single port (`RM_IHPSG13_1P_*`)**

| Macro | LEF `SIZE` (µm) | Area µm² [DERIVED] |
|---|---|---|
| `64x16_c2` | 236.80 × 64.36 | 15,240 |
| `256x8_c3_bm_bist` | 236.80 × 74.10 | 17,547 |
| `512x8_c3_bm_bist` | 236.80 × 110.38 | 26,138 |
| `1024x8_c2_bm_bist` | **146.88 × 336.46** | 49,419 |
| `4096x8_c3_bm_bist` | 236.80 × 618.30 | 146,413 |
| `256x16_c2_bm_bist` | 236.80 × 118.78 | 28,127 |
| `512x16_c2_bm_bist` | 236.80 × 191.34 | 45,309 |
| `1024x16_c2_bm_bist` | 236.80 × 336.46 | 79,674 |
| `4096x16_c3_bm_bist` | 416.64 × 618.30 | 257,608 |
| `256x32_c2_bm_bist` | 416.64 × 118.78 | 49,489 |
| `512x32_c2_bm_bist` | 416.64 × 191.34 | 79,720 |
| `1024x32_c2_bm_bist` | 416.64 × 336.46 | 140,183 |
| `2048x32_c2_bm_bist` | 416.64 × 626.70 | 261,108 |
| `8192x32_c4` | 1520.16 × 618.30 | 939,915 |
| `256x48_c2_bm_bist` | 596.48 × 118.78 | 70,850 |
| `64x64_c2_bm_bist` | 784.48 × 64.36 | 50,489 |
| `256x64_c2_bm_bist` | 784.48 × 118.78 | 93,181 |
| `512x64_c2_bm_bist` | 784.48 × 191.34 | 150,102 |
| `1024x64_c2_bm_bist` | 784.48 × 336.46 | 263,914 |
| `2048x64_c2_bm_bist` | 784.48 × 626.70 | 491,633 |

**Dual port (`RM_IHPSG13_2P_*`)**

| Macro | LEF `SIZE` (µm) |
|---|---|
| `64x22_c2_bm_bist` | 526.03 × 74.87 |
| `64x32_c2` | 702.83 × 74.87 |
| `256x8_c2_bm_bist` | 278.51 × 136.97 |
| `512x8_c2_bm_bist` | 261.17 × 219.77 |
| `256x16_c2_bm_bist` | 419.95 × 136.97 |
| `512x16_c2_bm_bist` | 402.61 × 219.77 |
| `1024x16_c2_bm_bist` | 402.61 × 385.37 |
| `256x32_c2_bm_bist` | 702.83 × 136.97 |
| `512x32_c2_bm_bist` | 685.49 × 219.77 |
| `1024x32_c2_bm_bist` | 685.49 × 385.37 |

TT's own memory page publishes the same numbers with tile counts
(<https://tinytapeout.com/specs/memory/>), e.g. 1024x8 → 49,419.24 µm², "2x1", ~76 % of tile area,
~5,390 bits/tile; 1024x16 → 79,673.73 µm², "2x2", ~60.5 %; 1024x32 → 140,182.69 µm², "3x4", ~31 %.
That page also says: *"One of these macros, 1024x8, has been successfully taped out and tested to
be working"* and *"Integrating the IHP SRAM macro at this stage is not trivial. There have been many
changes since the example project linked above, and additional changes will come after TTIHP26A."*

### The reference project: `tt_um_urish_sram_test`

Chip page: <https://tinytapeout.com/chips/ttihp0p2/tt_um_urish_sram_test> ·
Repo: <https://github.com/urish/ttihp-sram-test>

- Macro: **`RM_IHPSG13_1P_1024x8_c2_bm_bist`** (1 kbyte, single port, with BIST).
- **Tiles: `2x2`** — `info.yaml` says `tiles: "2x2"`, author Uri Shaked, `clock_hz: 0`.
  (The rendered chip page's tile count is unreliable; trust `info.yaml`.)
  <https://github.com/urish/ttihp-sram-test/blob/main/info.yaml>
- Macro physical size **146.88 × 336.46 µm**. A 2x2 tile die is 419.52 × 313.74 µm, so the macro is
  **taller than a 2-row block by ~22.7 µm** — which is exactly why it is placed `R90`
  (rotated) at `[42, 80]` [DERIVED from the DIE_AREA table + the config below].
- **Silicon result: the macro works.** TT states the 1024x8 macro *"has been successfully taped out
  and tested to be working"* (<https://tinytapeout.com/specs/memory/>), and links this project as
  the example. Independent per-die measurement report: **NOT FOUND**.
- The macro was vendored into the repo (`macro/RM_IHPSG13_1P_1024x8_c2_bm_bist/` with `.gds .lef
  .cdl` + three `.lib` corners), sourced from IHP-Open-PDK commit `7c124b73`
  (<https://github.com/urish/ttihp-sram-test/blob/main/macro/RM_IHPSG13_1P_1024x8_c2_bm_bist/README.md>).

**Current (LibreLane / ttihp26a) integration — `src/config.json`**
(<https://github.com/urish/ttihp-sram-test/blob/main/src/config.json>). This is the pattern to copy;
note it uses `MACROS`, **not** `EXTRA_LEFS` / `EXTRA_GDS_FILES`:

```json
"MACROS": {
  "RM_IHPSG13_1P_1024x8_c2_bm_bist": {
    "instances": { "sram": { "location": [42, 80], "orientation": "R90" } },
    "gds":   ["dir::../macro/RM_IHPSG13_1P_1024x8_c2_bm_bist/RM_IHPSG13_1P_1024x8_c2_bm_bist.gds"],
    "lef":   ["dir::../macro/RM_IHPSG13_1P_1024x8_c2_bm_bist/RM_IHPSG13_1P_1024x8_c2_bm_bist.lef"],
    "lib": {
      "nom_*": ["dir::.../RM_IHPSG13_1P_1024x8_c2_bm_bist_typ_1p20V_25C.lib"],
      "min_*": ["dir::.../RM_IHPSG13_1P_1024x8_c2_bm_bist_fast_1p32V_m55C.lib"],
      "max_*": ["dir::.../RM_IHPSG13_1P_1024x8_c2_bm_bist_slow_1p08V_125C.lib"],
      "*":     ["dir::.../RM_IHPSG13_1P_1024x8_c2_bm_bist_typ_1p20V_25C.lib"]
    },
    "nl":    ["dir::./RM_IHPSG13_1P_1024x8_c2_bm_bist.v"],
    "spice": ["dir::../macro/RM_IHPSG13_1P_1024x8_c2_bm_bist/RM_IHPSG13_1P_1024x8_c2_bm_bist.cdl"]
  }
},
"PDN_MACRO_CONNECTIONS": [
  "sram VPWR VGND VDD! VSS!",
  "sram VPWR VGND VDDARRAY! VSS!"
],
"PDN_CFG": "dir::pdn_cfg.tcl",
"MAGIC_MACRO_STD_CELL_SOURCE": "PDK",
"ERROR_ON_MAGIC_DRC": false,
"MAGIC_EXT_ABSTRACT_CELLS": ["RM_IHPSG13_.*"],
"FP_PDN_VPITCH": 38.87
```

**Caveats, each documented as a comment in that config or its PDN script:**

1. *"SRAM power pins are on Metal4, need Metal4↔TopMetal1 connection"* — the default macro PDN grid
   tries TopMetal1↔TopMetal2, which fails because TT sets `FP_PDN_MULTILAYER: 0` (no TopMetal2
   stripes). A custom `pdn_cfg.tcl` is required; its macro grid is
   `add_pdn_connect -grid macro -layers "Metal4 $::env(PDN_VERTICAL_LAYER)"`.
   <https://github.com/urish/ttihp-sram-test/blob/main/src/pdn_cfg.tcl>
   **This is a real advantage for cmos5l**: the macro's power pins are already on Metal4, and
   cmos5l's top routing layer is TopMetal1, so the same trick applies.
2. *"SRAM GDS has no PR boundary layer; read full GDS instead of blackbox"* → `MAGIC_MACRO_STD_CELL_SOURCE: "PDK"`.
3. *"SRAM macro internals trigger massive DRC in Magic; skip"* → `ERROR_ON_MAGIC_DRC: false`
   (and `RUN_KLAYOUT_DRC: 0` is already TT's default).
4. *"Blackbox SRAM macros during SPICE extraction for LVS, as the macros are not LVS clean"* →
   `MAGIC_EXT_ABSTRACT_CELLS: ["RM_IHPSG13_.*"]`. The commit that added this is literally
   `fix: blackbox SRAM macros during LVS` (2026-03-03).
5. `FP_PDN_VPITCH` was reduced from TT's default 50.0 to 38.87.
6. Historic pain (git log of the same repo, 2024-10/11): `fix: sram macro won't connect to PDN`,
   `fix: some metal5 pdn stripes are too short`, `fix: missing cdl + lib files for the SRAM macro`,
   `fix: skip LVS (workaround for SRAM macro LVS issue)`, `fix: disable DPO`.
   The **metal5** fix is sg13g2-only and would not apply on cmos5l.

The **original ttihp0p2 (OpenLane 2) config** used the older variable names, for reference:
`ADDITIONAL_LEFS`, `ADDITIONAL_GDS`, `ADDITIONAL_LIBS`, `CDL_FILE`, `MACRO_PLACEMENT`,
`PDN_TCL`, `ENABLE_DPO: 0`, plus a `GDS_ALLOW_EMPTY` regex for the `RM_IHPSG13_1P_BITKIT_*` cells.
(`git show` of that commit in <https://github.com/urish/ttihp-sram-test>.)

---

## 4. Maximum practical clock

From <https://tinytapeout.com/specs/clock/>:

- *"The frequency of the clock signal can be configured by the user, between 1 Hz and 66.5 MHz."*
- The RP2040 generates it *"using its PWM or PIO hardware peripherals to divide the RP2040 system clock."*
- *"The documentation specifies a maximum input frequency of 66 MHz"* for the I/O pad.
- *"Therefore, we believe that the maximum clock frequency for your designs will be around 66 MHz."*
- *"We expect a latency (insertion delay) of up to 10 nanoseconds between the chip's I/O pad and
  your project's clock."*
- Example waveform shown: *"25.179 MHz clock waveform generated by the RP2040 chip."*
- **No IHP- or CMOS5L-specific clock numbers appear on this page.**

From <https://tinytapeout.com/faq/>: max clock *"At least 50MHz"* for TT04–TT10, with silicon
characterisation ongoing.

TT's default sign-off constraint in the cmos5l template is `"CLOCK_PERIOD": 20` ns = **50 MHz**
(<https://github.com/TinyTapeout/ttihp-verilog-template/blob/cmos5l/src/config.json>).

Firmware-side clocking API (<https://github.com/TinyTapeout/tt-micropython-firmware>):
`tt.clock_project_once()`, `tt.clock_project_PWM(freq_hz)`, `tt.clock_project_stop()`. The demo
board v3 firmware also has a PIO clock class that calls `set_RP_system_clock(100_000_000)` before
starting (`src/ttboard/util/platform/rp2.py`).

**Measured/tested clock ranges on IHP silicon: NOT FOUND.** The best proxy is declared
`clock_hz` on the first CMOS5L shuttle `ttihp0p4` (<https://index.tinytapeout.com/ttihp0p4.json>):
top five declared values are **100 MHz, 64 MHz, 64 MHz, 50.4 MHz, 50 MHz**. These are author
declarations, not measurements.

---

## 5. Demo board host access and RP2040 GPIO mapping

### API

`tt-micropython-firmware` README (<https://github.com/TinyTapeout/tt-micropython-firmware>):

```python
tt.ui_in.value = 0xAA;  tt.ui_in[7] = 1;  tt.ui_in[4:2] = 0b101   # index 7 is MSB
b = tt.uo_out.value;    if tt.uo_out[5]: ...
tt.uio_in[2] = 1;       tt.uio_oe_pico.value = 0b100
```

Fast path: `import ttboard.util.platform as platform` →
`read_ui_in_byte() / write_ui_in_byte(v) / read_uio_byte() / write_uio_byte(v) /
read_uo_out_byte() / write_uio_outputenable(v)`.

**SPI / bit-bang:** there is **no hardware-SPI driver for the project pins in the firmware.**
Grepping `src/` on `main` finds SPI only inside *example project* testbenches
(e.g. `src/examples/tt_um_rgbled_decoder/`), never in `ttboard/`. What the firmware does use
is PIO — for clock generation (`src/ttboard/util/platform/rp2.py`) and in
`src/tests/counter_read.py`, `src/tests/counter_speed.py`, `src/ttboard/fpga/fabricfoxv2.py`.
So today: **bit-bang via the byte helpers, or roll your own PIO/`machine.SPI`.**

### RP2040 GPIO map (TT04+ demo board, `GPIOMapTT04`)

Source: <https://github.com/TinyTapeout/tt-micropython-firmware/blob/v2.0.4/src/ttboard/pins/gpio_map.py>,
cross-checked against the register masks in
<https://github.com/TinyTapeout/tt-micropython-firmware/blob/main/src/ttboard/util/platform/rp2040.py>
(`# 0x1E1E00 == … so GPIO 9-12 and 17-20` for `ui_in`; `# for bidir, all uio bits are in a line
starting at GPIO 21`; `# clock is on GPIO 0`).

| Project pin | RP2040 GPIO | RP2040 F1 (SPI) function |
|---|---|---|
| clk | GP0 | SPI0 RX |
| `uo_out[0]` | GP5 | SPI0 CSn |
| `uo_out[1]` | GP6 *(muxed with CTRL_ENA)* | SPI0 SCK |
| `uo_out[2]` | GP7 *(muxed with nCRST)* | SPI0 TX |
| `uo_out[3]` | GP8 *(muxed with CINC)* | SPI1 RX |
| `ui_in[0..3]` | GP9, GP10, GP11, GP12 | SPI1 CSn, SPI1 SCK, SPI1 TX, SPI1 RX |
| `uo_out[4]` | GP13 | SPI1 CSn |
| `uo_out[5]` | GP14 | SPI1 SCK |
| **`uo_out[6]`** | **GP15** | **SPI1 TX** |
| **`uo_out[7]`** | **GP16** | **SPI0 RX** |
| `ui_in[4]` | GP17 | SPI0 CSn |
| **`ui_in[5]`** | **GP18** | **SPI0 SCK** |
| **`ui_in[6]`** | **GP19** | **SPI0 TX** |
| **`ui_in[7]`** | **GP20** | **SPI0 RX** |
| `uio[0..7]` | GP21…GP28 | SPI0 CSn, SPI0 SCK, SPI0 TX, SPI1 RX, SPI1 CSn, SPI1 SCK, SPI1 TX, SPI1 RX |

RP2040 F1 column is quoted from **Table 2, "General Purpose Input/Output (GPIO) Bank 0 Functions",
§1.4.3** of the RP2040 datasheet — <https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf>
(resolves to `https://pip-assets.raspberrypi.com/categories/814-rp2040/documents/RP-008371-DS-1-rp2040-datasheet.pdf`).

**Answer to the question you actually care about — yes, a hardware SPI peripheral lands there:**

- **SPI0 master fits exactly**: `CSn = ui_in[4]/GP17`, `SCK = ui_in[5]/GP18`, `TX(MOSI) = ui_in[6]/GP19`,
  `RX(MISO) = uo_out[7]/GP16`. All four are RP2040 **SPI0** functions on those very pins.
  [DERIVED by joining the two cited tables.]
- `ui_in[7]/GP20` is SPI0 **RX** — an input to the RP2040 — so it is **not** usable as a fourth
  driven SPI0 output. Plan around that.
- **SPI1 master also fits, entirely on `uio`**: `RX = uio[3]/GP24` or `uio[7]/GP28`,
  `CSn = uio[4]/GP25`, `SCK = uio[5]/GP26`, `TX = uio[6]/GP27`. [DERIVED, same join.]
- Caution: TT's *recommended* Pmod SPI order is `uio[0]=CS, uio[1]=MOSI, uio[2]=MISO, uio[3]=SCK`
  (top row) and `uio[4..7]` the same on the bottom row
  (<https://tinytapeout.com/specs/pinouts/>). That ordering does **not** match the RP2040's
  hardware SPI pin functions. If you want hardware SPI from the demo board, pick the RP2040 order
  above, not the TT Pmod convention.

### Demo board v3 is different (RP2350B, not RP2040)

`main` firmware sets `GPIOMap = GPIOMapTTDBv3`
(<https://github.com/TinyTapeout/tt-micropython-firmware/blob/main/src/ttboard/pins/gpio_map.py>),
whose map is `ui_in[0..7] = GP17…GP24`, `uio[0..7] = GP25…GP32`, `uo_out[0..7] = GP33…GP40`,
`RP_PROJCLK = 16`, `PROJECT_nRST = 14`, plus `ADC1..ADC5 = 41..45`
(<https://github.com/TinyTapeout/tt-micropython-firmware/blob/main/src/ttboard/pins/gpio_map_dbv3.py>).
GPIO numbers > 29 mean an RP2350B-class part, not an RP2040. **RP2350 SPI pin functions for that map
were NOT checked** — do not assume the SPI0 trick above carries over.

### PMODs

<https://tinytapeout.com/specs/pinouts/>: there are input-only (`ui_in`), output-only (`uo_out`) and
bidirectional (`uio`) Pmod connectors. Recommended SPI: *"SPI uses CS, MOSI, MISO and SCK and
therefore requires only one row of pins of the Pmod connector, preferably the upper row."*
<https://tinytapeout.com/specs/pcb/> describes *"two sets of PMODs"* — *"a simple in,
bidirectional, and out trio"* at the bottom plus a second set mixing inputs and outputs, and states
the demo board uses an **RP2040**, *"clocked at 12MHz, but has a PLL which is used to increase the
system clock greatly."* A per-Pmod-pin ↔ bit wiring table with schematic references:
**NOT FOUND** on the public docs pages.

---

## 6. Jane Street competition

Source for everything here: <https://blog.janestreet.com/protocol-emulator-asic-competition/>

- Process/flow: *"We're targeting IHP's 130nm CMOS5L process through our friends at Tiny Tapeout.
  Start with the CMOS5L Verilog template"* →
  <https://github.com/TinyTapeout/ttihp-verilog-template/tree/cmos5l>
- Area: *"The current maximum area is 6x4 tiles per design. We are working on the possibility of
  scaling up to 8x4 tiles (~30% more area)."* / *"An 6x4 allocation is 24 tiles. At approximately
  200um × 150um per tile, that's about 0.7 mm² of nominal tile area."* (~1K logic cells per tile.)
- Deadline: *"Submit your design by January 18th, 2027."*
- Prize: winners' designs fabbed on the **March 2027 CMOS5L shuttle**; *"Winners will receive chips
  and dev boards back after fabrication."*
- Open source: *"Your submission should be open source so others can use and build on it."*
- Teams: *"This is a much bigger project than the puzzle, so we strongly recommend working in teams."*
- Target protocols: UART, SPI, I2C; stretch goals include low-speed USB and 10 Mbps Ethernet.
- **Submission process**: sign up via the Google form
  (<https://docs.google.com/forms/d/e/1FAIpQLSeF7fq756MegxZRQxotBwUJYZx-cL9MrGjxV0z4uD_J0sADxQ/viewform>);
  a final submission link is sent closer to the deadline. Contact `asic-competition@janestreet.com`.
- **There is no FAQ section on the blog post.** Fetched twice with different prompts; no FAQ
  headings, no clock-speed requirement, no pin-budget rule beyond the standard TT 8/8/8.
- **A Tiny Tapeout blog/news post about this competition: NOT FOUND.** <https://tinytapeout.com/news/>
  runs Sept 2022 → July 2026 and contains no mention of CMOS5L, Jane Street, or a 2027 IHP shuttle.
- Also **NOT FOUND**: any published rules addendum, errata, or update page beyond the blog text.

---

## 7. Hard macros inside a multi-tile TT IHP project

**Yes, the flow supports it, by design.** Mechanism, from
<https://github.com/TinyTapeout/tt-support-tools/blob/ihp-sg13cmos5l/project.py>:

```python
def create_merged_config(self):
    config = read_config("src/config")        # YOUR config.json — MACROS live here
    user_config = read_config("src/user_config")  # TT-generated, wins on conflict
    config.update(user_config)
    write_config(config, "src/config_merged")
```

and the TT-generated half:

```python
config = {
    "DESIGN_NAME": self.info.top_module,
    "VERILOG_FILES": [...],
    "DIE_AREA": die_area,                       # from tile_sizes.yaml
    "FP_DEF_TEMPLATE": f"dir::../tt/tech/{self.pdk}/def/tt_block_{tiles}_{def_suffix}.def",
    "VDD_PIN": "VPWR", "GND_PIN": "VGND",
    "RT_MAX_LAYER": self.tech.project_top_metal_layer,   # "Metal4" for cmos5l
}
```

`src/config_merged.json` is then handed straight to
`python -m librelane … --pdk ihp-sg13cmos5l --manual-pdk … src/config_merged.json`.
So **anything LibreLane understands that TT does not itself set — including `MACROS`,
`PDN_MACRO_CONNECTIONS`, `PDN_CFG`, `MAGIC_EXT_ABSTRACT_CELLS` — survives into the run.**
What you cannot override are `DIE_AREA`, `FP_DEF_TEMPLATE`, `RT_MAX_LAYER`, `DESIGN_NAME`,
`VERILOG_FILES`, `VDD_PIN`, `GND_PIN` (TT's dict wins).

**Config snippet / example:** the working reference is `tt_um_urish_sram_test`'s `src/config.json`
reproduced in §3 — <https://github.com/urish/ttihp-sram-test/blob/main/src/config.json> — plus its
custom `src/pdn_cfg.tcl`. It is a `2x2` multi-tile design with a hard macro, so
*macro-inside-multi-tile* is demonstrated.

**LibreLane reference for `MACROS`:** <https://librelane.readthedocs.io/en/latest/usage/using_macros.html>
— *"Macros is a dictionary … where the keys are the name of the macro (not instances thereof)"*,
with per-macro `instances` (`location` in microns, `orientation`), `gds`, `lef`, `vh`, `nl`, `pnl`,
`spice`, `lib` (timing-corner wildcard keys), `spef`, `sdf`, `json_h`. On `PDN_MACRO_CONNECTIONS`:
*"If you don't want to use `USE_POWER_PINS`, you can use this variable to manually hook up instance
connections."* Note on flattening: *"Yosys will rename instances to use the dot notation, i.e, the
name of an instance inside another instance will be `instance_a.instance_b`."*
The docs do **not** mark `EXTRA_LEFS` / `EXTRA_GDS_FILES` as deprecated, but the current example
project uses `MACROS` exclusively — use `MACROS`.

**A Tiny Tapeout documentation page specifically about hard macros in user projects: NOT FOUND.**
<https://tinytapeout.com/guides/> lists 14 guides; none covers macros or SRAM. The only pointer is
the "not trivial" warning on <https://tinytapeout.com/specs/memory/>.

---

## 8. Gotchas worth acting on

1. **`8x4` does not exist yet.** Design for `6x4` (1289.28 × 710.64 µm) as the ceiling; it is also
   the larger of the two big options (6x4 ≈ 0.92 mm² vs 8x2 ≈ 0.54 mm² [DERIVED]). If 8x4 lands,
   it is a `tile_sizes.yaml` entry + a `tt_block_8x4_pgvdd.def`; watch
   <https://github.com/TinyTapeout/tt-support-tools/tree/ihp-sg13cmos5l/tech/ihp-sg13cmos5l>.
2. **The tools branch is `ihp-sg13cmos5l`, the action branch is `ihp-cmos5l`.** `tt-gds-action`'s
   `tools-ref` defaults to `ihp-sg13cmos5l`
   (<https://github.com/TinyTapeout/tt-gds-action/blob/ihp-cmos5l/action.yml>).
3. **The template's devcontainer is stale and targets the wrong PDK.**
   `.devcontainer/Dockerfile` sets `ENV PDK=ihp-sg13g2`, `ARG TT_SUPPORT_TOOLS_BRANCH=main`, and
   `pip install librelane==3.0.0.dev44`, while CI uses `pdk: ihp-sg13cmos5l`, tools branch
   `ihp-sg13cmos5l`, and the action's default **`librelane 3.1.0.dev3`**.
   <https://github.com/TinyTapeout/ttihp-verilog-template/blob/cmos5l/.devcontainer/Dockerfile>
   vs <https://github.com/TinyTapeout/ttihp-verilog-template/blob/cmos5l/.github/workflows/gds.yaml>
   and <https://github.com/TinyTapeout/tt-gds-action/blob/ihp-cmos5l/action.yml>.
   Local hardening will silently use sg13g2 unless you override `PDK`.
4. **`RT_MAX_LAYER = Metal4`** — one fewer routing layer than the sg13g2 shuttles. Budget routing
   resources accordingly on a dense 6x4.
5. **The `tiles` keys `2x1/3x1/4x1/6x1/8x1` validate but have no DEF** and will fail hardening.
6. **The PDK is a pinned dev-branch commit under an explicit "preview only" warning**, and the
   `ihp-sg13cmos5l` tree is not on IHP-Open-PDK `main`. Expect churn; pin your own copy if
   reproducibility matters.
7. **SRAM on cmos5l is unproven.** The macros are reachable (symlink) and the 1024x8 LEF stays
   within Metal4, but no CMOS5L SRAM tape-out exists and the `ttihp0p4` shuttle had none.

---

## 9. Community report added 2026-09-15 (Tiny Tapeout Discord, unverified)

Reply received by Thomas on the TT Discord after asking about SRAM macros on cmos5l
(summarised; no link survived the paste, so treat as a lead, not a source):

- A community member named Ken reportedly has an SRAM macro working on cmos5l; his
  repo may not be public yet. Approach: tune the PDN so the power stripes align
  exactly with the macro's power pins (same idea as the `FP_PDN_VPITCH` change in
  `tt_um_urish_sram_test`).
- TT acceptance is expected if the macro is DRC clean. Macros currently do not pass
  the TT precheck; the responder expects that to be fixed.
- The Jane Street post mentions SRAM, so the responder assumes the organisers intend
  to resolve macro issues. There is no cmos5l-template SRAM example yet.

Action: ask Ken for the config (PDN pitch, macro placement) when public; the M2 gate
must include a precheck run, and the FLOPS fallback stays live until macros pass it.

Update 2026-09-16 (same Discord thread, Matt Venn, Tiny Tapeout): TT should be able to
handle hard macros; precheck DRC issues still exist on SG13G2; he has not tried CMOS5L
yet. Consequence: acceptance is not the open question, the precheck is, and nobody on
TT's side has run it on cmos5l. Thomas asked Ken for his config on 2026-09-15 (no answer
yet). The plan now has an M0.5 SRAM smoke test (branch `sram-smoke`, due 2026-09-28) so
the first precheck result exists in September; see `docs/PLAN.md` and D-015.
