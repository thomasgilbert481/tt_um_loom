# Copyright 2025 LibreLane Contributors
#
# Adapted from OpenLane
#
# Copyright 2020-2022 Efabless Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# ---------------------------------------------------------------------------
# Loom (tt_um_loom, branch sram-smoke) changes, SPDX-License-Identifier: Apache-2.0
# PDN configuration for the RM_IHPSG13_1P_512x16_c2_bm_bist macro on IHP
# sg13cmos5l.
#
# Everything below this comment block is LibreLane 3.1.0.dev3's default
# scripts/openroad/common/pdn_cfg.tcl, verbatim, with ONE change: the
# default's macro grid (its last ten lines) is replaced by
# loom_macro_power_bridges at the end of the file. All stripe geometry still
# comes from the FP_PDN_* keys in src/config.json.
#
# =============================================================================
# WHY THE DEFAULT MACRO GRID CANNOT WORK ON CMOS5L
# =============================================================================
# The default macro grid connects a macro's pins to the block grid with
#     add_pdn_connect -grid macro -layers "PDN_VERTICAL_LAYER PDN_HORIZONTAL_LAYER"
# i.e. Metal4 <-> TopMetal1 on cmos5l. Tiny Tapeout builds user blocks with
# FP_PDN_MULTILAYER = 0 (no TopMetal1 in the block, and the precheck forbids it,
# docs/tt_cmos5l_facts.md section 10), so that grid has nothing to connect to
# and pdngen stops with PDN-0232/PDN-0233 (CI runs 2 and 3). The macro's power
# pins are on Metal4, the block's only stripe layer, so the only possible
# connection is same-layer overlap.
#
# pdngen will not make that overlap by itself: it treats the macro's Metal4
# pins and OBS as obstructions and cuts every core-grid stripe a spacing short
# of the macro (0.48 um in CI run 4). Its same-net exception (shape.cpp,
# Shape::cut) only applies when the stripe fully covers the pin in x, and a
# stripe wider than the 2.81 um pin column would hit the Metal4 OBS that the
# macro places 0.26 um either side of every column. So in run 4 the macro was
# an island: PSM's connectivity walk started on the macro's own pin shapes and
# reported every rail, via stack and stripe of the (internally connected)
# stdcell grid as "unconnected" (654 x PSM-0038, PSM-0039 on VDDARRAY!).
#
# =============================================================================
# WHAT THIS SCRIPT DOES INSTEAD
# =============================================================================
# The stripe grid (FP_PDN_VPITCH 67.44 = 6 x 11.24, VSPACING 3.52, VOFFSET 26.36,
# macro at x = 12) puts four POWER and four GROUND stripes exactly inside the
# macro's power columns (x-centres, macro-local: POWER 17.24 84.68 152.12
# 219.56, GROUND 22.86 90.30 157.74 225.18; derivation in src/config.json and
# docs/tt_cmos5l_facts.md section 10). pdngen still cuts them at the macro
# edge. After pdngen has written its shapes, loom_macro_power_bridges adds one
# short Metal4 rectangle ("bridge") per aligned stripe end: same x-range as the
# stripe (so it stays inside the pin column, 0.28 um or more from the macro's
# Metal4 OBS), running from loom_bridge_reach inside the pin to
# loom_bridge_reach into the stripe. It only bridges when the stripe lies
# wholly inside a pin column of the SAME net and that column ends at the macro
# edge the stripe stops at, so it cannot short anything. Which columns are
# reachable is set by the LEF:
#     VDD!       local y 0 .. 38.825     touches the signal-pin edge only
#     VDDARRAY!  local y 45.465 .. 191.34 touches the opposite edge only
#     VSS!       full height             both edges
# so VDD! is bridged from one side of the macro and VDDARRAY! from the other,
# which is why the macro sits with standard-cell rows both above and below it
# (src/config.json).
#
# The wrapper then runs check_power_grid on every supply and lets its error
# propagate, so a grid that is still not connected stops the run here, a few
# minutes in, instead of after detailed routing (LibreLane's own call right
# after pdngen only warns, and Checker.PowerGridViolations defers its error to
# the end of the flow).
#
# =============================================================================
# THE STRIPE NUMBERS (src/config.json FP_PDN_*), FROM THE LEF
# =============================================================================
# macro/RM_IHPSG13_1P_512x16_c2_bm_bist/RM_IHPSG13_1P_512x16_c2_bm_bist.lef,
# columns 2.81 um wide, macro-local x:
#   VDD! / VDDARRAY!  x0 = 4.26 + 11.24k and 151.05 + 11.24k   (k = 0..7)
#   VSS!              x0 = 9.88 + 11.24k and 145.43 + 11.24k   (k = 0..7)
#   irregular middle band x 88 .. 151 (four full-height VDD!, five VSS!)
# so same-net pitch 11.24, POWER-to-GROUND centre distance 5.62, and the right
# half sits +0.67 um off the left half's grid.
#   VWIDTH   2.1    Tiny Tapeout's value; these stripes are the block's power
#                   pins, which tt_top reaches with TopMetal1 vias.
#   VSPACING 3.52   pdngen puts the GROUND stripe at (spacing + width) from the
#                   POWER stripe centre: 3.52 + 2.1 = 5.62.
#   VPITCH   67.44  = 6 x 11.24, the smallest multiple that steps over the
#                   middle band instead of landing in it.
#   VOFFSET  26.36  pdngen sweeps from the core xMin 2.88: 2.88 + 26.36 = 29.24
#                   = 12.0 (macro x) + 17.24, and 17.24 is the k = 1 column
#                   centre 16.905 + 0.335, which splits the 0.67 um half-to-half
#                   mismatch so both halves' stripes sit inside their columns
#                   (0.02 um to spare on one side; 0.28 um or more to the OBS).
# Do not add -snap_to_grid: Metal4 tracks (0.48 um) are incommensurate with
# 11.24 um, and snapping would pull stripes out of the columns.

source $::env(SCRIPTS_DIR)/openroad/common/io.tcl
source $::env(SCRIPTS_DIR)/openroad/common/set_global_connections.tcl
set_global_connections

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



if { $::env(PDN_MULTILAYER) == 1 } {

    set arg_list [list]
    if { $::env(PDN_ENABLE_PINS) } {
        lappend arg_list -pins "$::env(PDN_VERTICAL_LAYER) $::env(PDN_HORIZONTAL_LAYER)"
    }

    define_pdn_grid \
        -name stdcell_grid \
        -starts_with POWER \
        -voltage_domain CORE \
        {*}$arg_list

    set arg_list [list]
    append_if_equals arg_list PDN_EXTEND_TO "core_ring" -extend_to_core_ring
    append_if_equals arg_list PDN_EXTEND_TO "boundary" -extend_to_boundary

    add_pdn_stripe \
        -grid stdcell_grid \
        -layer $::env(PDN_VERTICAL_LAYER) \
        -width $::env(PDN_VWIDTH) \
        -pitch $::env(PDN_VPITCH) \
        -offset $::env(PDN_VOFFSET) \
        -spacing $::env(PDN_VSPACING) \
        -starts_with POWER \
        {*}$arg_list

    add_pdn_stripe \
        -grid stdcell_grid \
        -layer $::env(PDN_HORIZONTAL_LAYER) \
        -width $::env(PDN_HWIDTH) \
        -pitch $::env(PDN_HPITCH) \
        -offset $::env(PDN_HOFFSET) \
        -spacing $::env(PDN_HSPACING) \
        -starts_with POWER \
        {*}$arg_list

    add_pdn_connect \
        -grid stdcell_grid \
        -layers "$::env(PDN_VERTICAL_LAYER) $::env(PDN_HORIZONTAL_LAYER)"
} else {

    set arg_list [list]
    if { $::env(PDN_ENABLE_PINS) } {
        lappend arg_list -pins "$::env(PDN_VERTICAL_LAYER)"
    }

    define_pdn_grid \
        -name stdcell_grid \
        -starts_with POWER \
        -voltage_domain CORE \
        {*}$arg_list

    set arg_list [list]
    append_if_equals arg_list PDN_EXTEND_TO "core_ring" -extend_to_core_ring
    append_if_equals arg_list PDN_EXTEND_TO "boundary" -extend_to_boundary

    add_pdn_stripe \
        -grid stdcell_grid \
        -layer $::env(PDN_VERTICAL_LAYER) \
        -width $::env(PDN_VWIDTH) \
        -pitch $::env(PDN_VPITCH) \
        -offset $::env(PDN_VOFFSET) \
        -spacing $::env(PDN_VSPACING) \
        -starts_with POWER \
        {*}$arg_list
}

# Adds the standard cell rails if enabled.
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


# Adds the core ring if enabled.
if { $::env(PDN_CORE_RING) == 1 } {
    if { $::env(PDN_MULTILAYER) == 1 } {
        set arg_list [list]
        append_if_flag arg_list PDN_CORE_RING_ALLOW_OUT_OF_DIE -allow_out_of_die
        append_if_flag arg_list PDN_CORE_RING_CONNECT_TO_PADS -connect_to_pads
        append_if_equals arg_list PDN_EXTEND_TO "boundary" -extend_to_boundary
        append_if_exists_argument arg_list PDN_CORE_RING_CONNECT_TO_PAD_LAYERS -connect_to_pad_layers

        set pdn_core_vertical_layer $::env(PDN_VERTICAL_LAYER)
        set pdn_core_horizontal_layer $::env(PDN_HORIZONTAL_LAYER)

        if { [info exists ::env(PDN_CORE_VERTICAL_LAYER)] } {
            set pdn_core_vertical_layer $::env(PDN_CORE_VERTICAL_LAYER)
        }

        if { [info exists ::env(PDN_CORE_HORIZONTAL_LAYER)] } {
            set pdn_core_horizontal_layer $::env(PDN_CORE_HORIZONTAL_LAYER)
        }

        add_pdn_ring \
            -grid stdcell_grid \
            -layers "$pdn_core_vertical_layer $pdn_core_horizontal_layer" \
            -widths "$::env(PDN_CORE_RING_VWIDTH) $::env(PDN_CORE_RING_HWIDTH)" \
            -spacings "$::env(PDN_CORE_RING_VSPACING) $::env(PDN_CORE_RING_HSPACING)" \
            -core_offset "$::env(PDN_CORE_RING_VOFFSET) $::env(PDN_CORE_RING_HOFFSET)" \
            {*}$arg_list

        if { [info exists ::env(PDN_CORE_VERTICAL_LAYER)] } {
            add_pdn_connect \
                -grid stdcell_grid \
                -layers "$::env(PDN_CORE_VERTICAL_LAYER) $::env(PDN_HORIZONTAL_LAYER)"
        }

        if { [info exists ::env(PDN_CORE_HORIZONTAL_LAYER)] } {
            add_pdn_connect \
                -grid stdcell_grid \
                -layers "$::env(PDN_CORE_HORIZONTAL_LAYER) $::env(PDN_VERTICAL_LAYER)"
        }

        if { [info exists ::env(PDN_CORE_VERTICAL_LAYER)] && [info exists ::env(PDN_CORE_HORIZONTAL_LAYER)] } {
            add_pdn_connect \
                -grid stdcell_grid \
                -layers "$::env(PDN_CORE_VERTICAL_LAYER) $::env(PDN_CORE_HORIZONTAL_LAYER)"
        }

    } else {
        throw APPLICATION "PDN_CORE_RING cannot be used when PDN_MULTILAYER is set to false."
    }
}


# =============================================================================
# Replacement for the default macro grid: Metal4 bridges, see the header.
# =============================================================================

# Longest stripe-end-to-pin gap that is bridged, and how far a bridge runs
# into the pin and into the stripe. Microns.
set ::loom_bridge_max_gap 5.0
set ::loom_bridge_reach   1.5

proc loom_macro_power_bridges {} {
    set block [ord::get_db_block]
    set tech [ord::get_db_tech]
    set layer_name $::env(PDN_VERTICAL_LAYER)
    set layer [$tech findLayer $layer_name]
    set dbu [$block getDbUnitsPerMicron]
    set max_gap [expr {round($::loom_bridge_max_gap * $dbu)}]
    set reach [expr {round($::loom_bridge_reach * $dbu)}]

    # --- 1. every power/ground pin rectangle of every hard macro on the stripe
    #        layer, in die coordinates, with the block net it is tied to
    set columns [list]
    set counts [dict create]
    foreach inst [$block getInsts] {
        if { ![[$inst getMaster] isBlock] } { continue }
        set iname [$inst getName]
        set orient [$inst getOrient]
        if { $orient ne "R0" && $orient ne "MX" } {
            error "LOOMPDN: $iname has orientation $orient; only R0 (N) and MX (FS) keep the Metal4 power columns vertical"
        }
        set bb [$inst getBBox]
        set bx0 [$bb xMin]
        set by0 [$bb yMin]
        set by1 [$bb yMax]
        puts "LOOMPDN macro $iname orient $orient bbox_um [expr {$bx0 / double($dbu)}] [expr {$by0 / double($dbu)}] [expr {[$bb xMax] / double($dbu)}] [expr {$by1 / double($dbu)}]"
        foreach iterm [$inst getITerms] {
            set mterm [$iterm getMTerm]
            set sig [$mterm getSigType]
            if { $sig ne "POWER" && $sig ne "GROUND" } { continue }
            set key "$iname/[$mterm getName]"
            set net [$iterm getNet]
            if { $net eq "NULL" } {
                error "LOOMPDN: $key is not connected to any net (check PDN_MACRO_CONNECTIONS)"
            }
            dict set counts $key 0
            set ux1 ""
            foreach mpin [$mterm getMPins] {
                foreach box [$mpin getGeometry] {
                    if { [$box isVia] } { continue }
                    if { [[$box getTechLayer] getName] ne $layer_name } { continue }
                    set x1 [expr {$bx0 + [$box xMin]}]
                    set x2 [expr {$bx0 + [$box xMax]}]
                    if { $orient eq "R0" } {
                        set y1 [expr {$by0 + [$box yMin]}]
                        set y2 [expr {$by0 + [$box yMax]}]
                    } else {
                        set y1 [expr {$by1 - [$box yMax]}]
                        set y2 [expr {$by1 - [$box yMin]}]
                    }
                    lappend columns [list $net $key $x1 $y1 $x2 $y2]
                    if { $ux1 eq "" } {
                        lassign [list $x1 $y1 $x2 $y2] ux1 uy1 ux2 uy2
                    } else {
                        set ux1 [expr {min($ux1, $x1)}]
                        set uy1 [expr {min($uy1, $y1)}]
                        set ux2 [expr {max($ux2, $x2)}]
                        set uy2 [expr {max($uy2, $y2)}]
                    }
                }
            }
            # Cross-check the hand-written transform against OpenDB's own.
            set ib [$iterm getBBox]
            if { $ux1 ne "" && ($ux1 != [$ib xMin] || $uy1 != [$ib yMin] || $ux2 != [$ib xMax] || $uy2 != [$ib yMax]) } {
                error "LOOMPDN: transformed pins of $key ($ux1 $uy1 $ux2 $uy2) disagree with the ITerm bbox ([$ib xMin] [$ib yMin] [$ib xMax] [$ib yMax])"
            }
        }
    }
    if { [llength $columns] == 0 } {
        puts "LOOMPDN no hard macro with $layer_name power pins; nothing to bridge"
        return
    }

    # --- 2. the stripe-layer shapes pdngen wrote, listed once before any
    #        bridge is added
    set stripes [list]
    set nets [lsort -unique [lmap c $columns {lindex $c 0}]]
    foreach net $nets {
        foreach swire [$net getSWires] {
            foreach sbox [$swire getWires] {
                if { [$sbox isVia] } { continue }
                if { [[$sbox getTechLayer] getName] ne $layer_name } { continue }
                lappend stripes [list $net [$sbox xMin] [$sbox yMin] [$sbox xMax] [$sbox yMax]]
            }
        }
    }

    # --- 3. bridge every stripe end that stops just short of a same-net column
    set new_wire [dict create]
    set made 0
    foreach s $stripes {
        lassign $s snet sx1 sy1 sx2 sy2
        foreach c $columns {
            lassign $c cnet key px1 py1 px2 py2
            if { $cnet ne $snet } { continue }
            if { $sx1 < $px1 || $sx2 > $px2 } { continue }
            set gap_top [expr {$sy1 - $py2}]
            set gap_bot [expr {$py1 - $sy2}]
            if { $gap_top >= 0 && $gap_top <= $max_gap } {
                # the stripe starts just above the column's top end
                set b [list $sx1 [expr {$py2 - $reach}] $sx2 [expr {min($sy1 + $reach, $sy2)}]]
                set gap $gap_top
            } elseif { $gap_bot >= 0 && $gap_bot <= $max_gap } {
                # the stripe ends just below the column's bottom end
                set b [list $sx1 [expr {max($sy2 - $reach, $sy1)}] $sx2 [expr {$py1 + $reach}]]
                set gap $gap_bot
            } else {
                continue
            }
            set net_name [$snet getName]
            if { ![dict exists $new_wire $net_name] } {
                dict set new_wire $net_name [odb::dbSWire_create $snet "ROUTED"]
            }
            lassign $b bx1 by1 bx2 by2
            set sb [odb::dbSBox_create [dict get $new_wire $net_name] $layer $bx1 $by1 $bx2 $by2 "STRIPE"]
            if { $sb eq "NULL" || $sb eq "" } {
                error "LOOMPDN: could not create bridge $b on $net_name"
            }
            dict incr counts $key
            incr made
            puts [format "LOOMPDN bridge %-5s -> %-22s x %.3f..%.3f y %.3f..%.3f (gap %.3f um)" \
                $net_name $key [expr {$bx1 / double($dbu)}] [expr {$bx2 / double($dbu)}] \
                [expr {$by1 / double($dbu)}] [expr {$by2 / double($dbu)}] [expr {$gap / double($dbu)}]]
        }
    }

    # --- 4. every supply pin of every macro needs at least one bridge
    set missing [list]
    dict for {key n} $counts {
        puts "LOOMPDN $key: $n bridge(s)"
        if { $n == 0 } { lappend missing $key }
    }
    if { [llength $missing] > 0 } {
        # List the stripe shapes so the failure can be read off the log.
        foreach s $stripes {
            lassign $s snet sx1 sy1 sx2 sy2
            puts [format "LOOMPDN stripe %-5s x %.3f..%.3f y %.3f..%.3f" [$snet getName] \
                [expr {$sx1 / double($dbu)}] [expr {$sx2 / double($dbu)}] \
                [expr {$sy1 / double($dbu)}] [expr {$sy2 / double($dbu)}]]
        }
        error "LOOMPDN: no stripe could be bridged to: [join $missing {, }]"
    }
    puts "LOOMPDN $made bridge(s) added"
}

# Run the bridges right after LibreLane's `pdngen` (scripts/openroad/pdn.tcl
# calls it exactly once, after sourcing this file), then check the grid and
# let a failure stop the step.
if { [info commands ::loom_pdngen_unwrapped] eq "" } {
    rename ::pdngen ::loom_pdngen_unwrapped
    proc ::pdngen { args } {
        ::loom_pdngen_unwrapped {*}$args
        foreach flag {-reset -ripup -report_only -check_only} {
            if { [lsearch -exact $args $flag] >= 0 } { return }
        }
        loom_macro_power_bridges
        foreach net_name [concat $::env(VDD_NETS) $::env(GND_NETS)] {
            puts "LOOMPDN check_power_grid -net $net_name"
            check_power_grid -net $net_name
        }
    }
}
