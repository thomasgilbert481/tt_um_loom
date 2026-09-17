# Custom PDN configuration for the RM_IHPSG13_1P_512x16_c2_bm_bist macro
# on IHP sg13cmos5l.  SPDX-License-Identifier: Apache-2.0
#
# Adapted from librelane/scripts/openroad/common/pdn_cfg.tcl (the PDN_MULTILAYER
# == 0 branch) and from src/pdn_cfg.tcl of tt_um_urish_sram_test.
#
# =============================================================================
# WHY THIS FILE EXISTS, AND WHY IT IS NOT THE SG13G2 RECIPE
# =============================================================================
# The reference project (tt_um_urish_sram_test, SG13G2) says "SRAM power pins
# are on Metal4, need Metal4 <-> TopMetal1 connection" and does
#     add_pdn_connect -grid macro -layers "Metal4 $::env(PDN_VERTICAL_LAYER)"
# That works on SG13G2 because there PDN_VERTICAL_LAYER is TopMetal1:
#     ihp-sg13g2/libs.tech/librelane/config.tcl
#       PDN_VERTICAL_LAYER   TopMetal1
#       PDN_HORIZONTAL_LAYER TopMetal2
# so the macro's Metal4 power pins are reached from a *different*, higher layer
# through a via, and the macro's Metal4 obstructions never matter.
#
# On cmos5l the two PDN layers are one step lower:
#     ihp-sg13cmos5l/libs.tech/librelane/config.tcl
#       FP_PDN_VERTICAL_LAYER   Metal4        (vertical stripes)
#       FP_PDN_HORIZONTAL_LAYER TopMetal1     (horizontal stripes)
#       FP_PDN_VWIDTH 1.0  VSPACING 2.0  VPITCH 50.0  VOFFSET 10.0
# and Tiny Tapeout keeps FP_PDN_MULTILAYER = 0 for user projects, so a user
# block gets Metal4 vertical stripes and Metal1 rails and nothing else.
# TopMetal1 is not ours to use: tt-multiplexer's own ol2/tt_top/pdn.tcl builds
# the chip-level grid out of TopMetal1 horizontal stripes and connects them
# down to each user block through
#     add_pdn_connect -grid macro -layers "Metal4 TopMetal1"
# with -cells "^tt_(ctrl|mux|pg|um)(_.*)?".  Putting our own TopMetal1 stripes
# inside the block would collide with that, so we do not.
#
# That leaves exactly one way to power the macro on cmos5l: the block's Metal4
# stripes must land *inside* the macro's own Metal4 power pins, same net, same
# layer, so the shapes merge and no via is needed. This is what the Tiny
# Tapeout Discord report meant by "tune the PDN so the stripes line up exactly
# with the macro's power pins" (docs/tt_cmos5l_facts.md section 9) -- on cmos5l
# it is not an optimisation, it is the only option.
#
# The macro is built for it. Its Metal4 OBS fills every gap between the power
# columns and declares SPACING 0.21, leaving only the 2.81 um pin columns open.
#
# =============================================================================
# THE NUMBERS, FROM THE LEF
# =============================================================================
# macro/RM_IHPSG13_1P_512x16_c2_bm_bist/RM_IHPSG13_1P_512x16_c2_bm_bist.lef,
# SIZE 236.8 BY 191.34, placed at (12.0, 10.0) orientation N (R0):
#
#   VDD! / VDDARRAY! columns  x0 = 4.26 + 11.24k   (k = 0..7)
#                             x0 = 151.05 + 11.24k (k = 0..7)
#   VSS! columns              x0 = 9.88 + 11.24k   (k = 0..7)
#                             x0 = 145.43 + 11.24k (k = 0..7)
#   every column 2.81 um wide; irregular in the middle (x 88 .. 151).
#
# So: same-net pitch 11.24, POWER-to-GROUND centre distance 5.62, and the right
# half of the macro is offset +0.67 um from the left half's grid. Hence:
#
#   PDN_VWIDTH   2.1    unchanged from Tiny Tapeout's config.json. The block's
#                       power pins are these stripes, so their width is what
#                       tt_top connects to; do not change it casually.
#   PDN_VSPACING 3.52   OpenROAD places the GROUND stripe of a pair at
#                       (spacing + width) from the POWER stripe's centre
#                       (straps.cpp: group_pitch = spacing_ + width_, and pos
#                       is the strap CENTRE). 3.52 + 2.1 = 5.62 = the macro's
#                       POWER-to-GROUND column distance.
#   PDN_VPITCH   67.44  = 6 x 11.24. Every multiple of 11.24 keeps the stripes
#                       on the column grid; 6 is the smallest multiple whose
#                       stripes step straight over the macro's irregular middle
#                       band (nominal columns k = 8..12) instead of landing in
#                       it. With k = 1, 7, 13, 19 the four POWER stripes and
#                       four GROUND stripes over the macro all sit inside a
#                       real pin.
#   PDN_VOFFSET  26.36  OpenROAD sweeps from the grid domain's xMin, which for
#                       the core grid is the core area from the TT floorplan
#                       DEF: x = 2.88 (tt_block_2x2_pgvdd.def, ROW ... 2880).
#                       2.88 + 26.36 = 29.24 = 12.0 (macro x) + 17.24, and
#                       17.24 = 16.905 (the k=1 VDD! column centre) + 0.335.
#
# The +0.335 um is deliberate: it splits the macro's +0.67 um left/right
# mismatch in half so the same uniform stripe grid sits inside the pins on both
# halves. Verified stripe positions, in macro-local microns:
#
#   POWER  17.24  84.68  152.12  219.56   all inside a VDD!/VDDARRAY! column
#   GROUND 22.86  90.30  157.74  225.18   all inside a VSS! column
#   worst clearance to the macro's Metal4 OBS: 0.28 um (needs 0.21)
#
# OpenROAD only snaps strap centres to the routing-track grid when
# -snap_to_grid is given (straps.cpp gates snapToGrid on snap_), and neither
# this script nor LibreLane's default passes it, so these positions survive
# exactly as written. Metal4 tracks are on a 0.48 um pitch, which is
# incommensurate with the macro's 11.24, so snapping would ruin the alignment:
# do not add -snap_to_grid here.
#
# KNOWN RISK, deliberately accepted (see docs/info.md): outside the middle band
# a POWER column carries VDD! only up to y = 38.825 and VDDARRAY! only from
# y = 45.465, with a Metal4 OBS rectangle bridging y 39.085 .. 45.205. Our
# full-height POWER stripes cross that 6.12 um band. PDN_MACRO_CONNECTIONS ties
# both VDD! and VDDARRAY! to VPWR, so it is electrically what we want, but it
# is metal over an obstruction and it is the first thing to look at if the
# hardening or the precheck complains. The GROUND stripes are clean: VSS!
# columns run the full height.

source $::env(SCRIPTS_DIR)/openroad/common/set_global_connections.tcl
set_global_connections

# -----------------------------------------------------------------------------
# DEBUG (smoke test): print what the macro's power pins are connected to, and
# where the macro sits. CI run 2 stopped at PDN-0232/0233 ("macro grid has no
# shapes") while every global_connect call reported "0 connections made"; if
# the pins below print UNCONNECTED, pdngen sees all of the macro's pin metal
# as foreign obstruction and removes every stripe over it.
# -----------------------------------------------------------------------------
foreach loom_inst [[ord::get_db_block] getInsts] {
    if { [[$loom_inst getMaster] isBlock] } {
        set loom_bb [$loom_inst getBBox]
        puts "LOOMDBG macro [$loom_inst getName] master [[$loom_inst getMaster] getName] bbox_dbu [$loom_bb xMin] [$loom_bb yMin] [$loom_bb xMax] [$loom_bb yMax]"
        foreach loom_it [$loom_inst getITerms] {
            set loom_mt [$loom_it getMTerm]
            set loom_ty [$loom_mt getSigType]
            if { $loom_ty eq "POWER" || $loom_ty eq "GROUND" } {
                set loom_net [$loom_it getNet]
                if { $loom_net eq "NULL" } { set loom_nn "UNCONNECTED" } else { set loom_nn [$loom_net getName] }
                puts "LOOMDBG   pin [$loom_mt getName] sigtype $loom_ty -> $loom_nn"
            }
        }
    }
}

set secondary []
foreach vdd $::env(VDD_NETS) gnd $::env(GND_NETS) {
    if { $vdd != $::env(VDD_NET)} {
        lappend secondary $vdd

        set db_net [[ord::get_db_block] findNet $vdd]
        if {$db_net == "NULL"} {
            set net [odb::dbNet_create [ord::get_db_block] $vdd]
            $net setSpecial
            $net setSigType "POWER"
        }
    }

    if { $gnd != $::env(GND_NET)} {
        lappend secondary $gnd

        set db_net [[ord::get_db_block] findNet $gnd]
        if {$db_net == "NULL"} {
            set net [odb::dbNet_create [ord::get_db_block] $gnd]
            $net setSpecial
            $net setSigType "GROUND"
        }
    }
}

set_voltage_domain -name CORE -power $::env(VDD_NET) -ground $::env(GND_NET) \
    -secondary_power $secondary

# -----------------------------------------------------------------------------
# Standard cell grid: Metal4 vertical stripes only (FP_PDN_MULTILAYER = 0),
# with the stripes declared as the block's power pins so tt_top can find them.
# This is LibreLane's default single-layer branch, unchanged except that the
# pitch/spacing/offset above put the stripes on the macro's power columns.
# -----------------------------------------------------------------------------
define_pdn_grid \
    -name stdcell_grid \
    -starts_with POWER \
    -voltage_domain CORE \
    -pins $::env(PDN_VERTICAL_LAYER)

add_pdn_stripe \
    -grid stdcell_grid \
    -layer $::env(PDN_VERTICAL_LAYER) \
    -width $::env(PDN_VWIDTH) \
    -pitch $::env(PDN_VPITCH) \
    -offset $::env(PDN_VOFFSET) \
    -spacing $::env(PDN_VSPACING) \
    -starts_with POWER -extend_to_core_ring

# Standard cell rails on Metal1, and the via down from the Metal4 stripes.
if { $::env(PDN_ENABLE_RAILS) == 1 } {
    add_pdn_stripe \
        -grid stdcell_grid \
        -layer $::env(PDN_RAIL_LAYER) \
        -width $::env(PDN_RAIL_WIDTH) \
        -followpins

    add_pdn_connect \
        -grid stdcell_grid \
        -layers "$::env(PDN_RAIL_LAYER) $::env(PDN_VERTICAL_LAYER)"
}

# -----------------------------------------------------------------------------
# Macro grid.
#
# -halo "0 0" instead of the usual 10 um: the halo is what blocks the stdcell
# grid's stripes around the macro, and we need those stripes to run right up to
# the macro edge so they abut the macro grid's own stripes and form one
# continuous Metal4 rail from the bottom of the die to the top.
#
# The macro grid then repeats exactly the same stripes over the macro itself.
# For an instance grid OpenROAD sweeps from the instance's bbox xMin
# (InstanceGrid::getDomainArea returns the instance bbox), so the offset is
# macro-relative: 17.24 lands the first POWER stripe at die x 12.0 + 17.24 =
# 29.24, the same absolute position the stdcell grid produces. Same layer, same
# net, same x, so the two sets merge rather than fight. If OpenROAD turns out
# not to trim the stdcell stripes at the macro at all, these are simply
# redundant copies of shapes that are already there.
#
# There is deliberately no add_pdn_connect for this grid: with Metal4 as both
# the PDN layer and the macro's pin layer there is no via to make. The
# connection is the geometric overlap of stripe and pin.
# -----------------------------------------------------------------------------
define_pdn_grid \
    -macro \
    -default \
    -name macro \
    -starts_with POWER \
    -halo "0 0"

add_pdn_stripe \
    -grid macro \
    -layer $::env(PDN_VERTICAL_LAYER) \
    -width $::env(PDN_VWIDTH) \
    -pitch $::env(PDN_VPITCH) \
    -offset 17.24 \
    -spacing $::env(PDN_VSPACING) \
    -starts_with POWER
