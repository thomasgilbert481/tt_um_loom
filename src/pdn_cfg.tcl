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
# default's macro grid (its last ten lines) is replaced by the pdngen wrapper
# at the end of the file. All stripe geometry still comes from the FP_PDN_*
# keys in src/config.json.
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
# pdngen will not make that overlap by itself: it treats a FIXED macro's
# Metal4 pins and OBS as obstructions and cuts every core-grid stripe a spacing
# short of the macro (0.48 um in CI run 4). Its same-net exception (shape.cpp,
# Shape::cut) only applies when the stripe fully covers the pin in x, and a
# stripe wider than the 2.81 um pin column would hit the Metal4 OBS that the
# macro places 0.26 um either side of every column. In run 4 the macro was
# therefore an island, and PSM's connectivity walk, which starts on a
# top-layer node, started on the macro's own pins and reported the whole
# (internally connected) stdcell grid as "unconnected" (654 x PSM-0038).
#
# Short Metal4 bridges from the cut stripe ends into the pin columns (CI run 5)
# connect the macro and pass LVS and the cmos5l sign-off DRC, but the stripes
# are the block's power pins, and the Tiny Tapeout precheck (pin_check.py)
# requires every Metal4 power port to reach within 10 um of both the top and
# the bottom block edge. The eight stripes cut by the macro became sixteen
# half-height ports and failed it.
#
# =============================================================================
# WHAT THIS SCRIPT DOES INSTEAD: FULL-HEIGHT STRIPES THROUGH THE PIN COLUMNS
# =============================================================================
# The stripe grid (FP_PDN_VPITCH 67.44 = 6 x 11.24, VSPACING 3.52, VOFFSET 26.36,
# macro at x = 12) puts every stripe that crosses the macro exactly inside one
# of the macro's power columns of the same net (x-centres, macro-local: POWER
# 17.24 84.68 152.12 219.56, GROUND 22.86 90.30 157.74 225.18; the numbers
# are derived below). So the stripes do not need to stop at the macro at all.
# pdngen only treats FIXED instances as obstructions (grid.cpp,
# Grid::makeInitialObstructions skips instances that are not fixed), so the
# wrapper at the end of this file marks the hard macros PLACED for the
# duration of the real pdngen call and restores their status afterwards. The
# rows under and beside the macro are already cut, so no rail or via is made
# over it; the stripes simply run from the bottom of the core to the top,
# through the pin columns, and every stripe is one full-height power pin.
#
# Nothing then relies on the stripes being aligned by luck. After pdngen the
# wrapper checks, for every stripe-layer shape that overlaps a macro, that it
# lies inside power columns of its own net over the whole macro height
# (except gaps of at most loom_max_pin_gap between two such columns), and that
# every macro supply pin is overlapped by at least one stripe; otherwise the
# step fails with the offending shapes listed. Then it runs check_power_grid
# on every supply and lets its error stop the step (LibreLane's own call right
# after pdngen only warns, and Checker.PowerGridViolations defers its error to
# the end of the flow).
#
# The one gap: a regular POWER column carries VDD! up to local y 38.825 and
# VDDARRAY! from 45.465, and the LEF puts a Metal4 OBS rectangle across the
# column in between (y 39.085 .. 45.205). The POWER stripes cross that band.
# The macro GDS has no Metal4 at all within 1 um of any of the four POWER
# stripes' x-ranges in the band (checked with gdstk, 2026-09-18), so the
# stripe only joins VDD! to VDDARRAY!, which PDN_MACRO_CONNECTIONS ties to
# VPWR anyway. The OBS is what makes pdngen and the router keep out; the
# router does not flag it because it skips checks between two fixed shapes
# (FlexGC_main.cpp), and the cmos5l sign-off DRC sees only the real metal.
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
#   VWIDTH   2.1    Tiny Tapeout's value and the precheck's minimum power-port
#                   width on cmos5l (tech_data.py power_pins_min_width).
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
# Replacement for the default macro grid: full-height stripes through the
# macro's power columns, verified. See the header.
# =============================================================================

# Longest stretch of a macro that a stripe may cross without a same-net pin
# under it, and only between two same-net pins (the VDD!/VDDARRAY! split is
# 6.64 um). Microns.
set ::loom_max_pin_gap 7.0

proc loom_check_macro_stripes {} {
    set block [ord::get_db_block]
    set layer_name $::env(PDN_VERTICAL_LAYER)
    set dbu [$block getDbUnitsPerMicron]
    set max_gap [expr {round($::loom_max_pin_gap * $dbu)}]
    set um [expr {1.0 / $dbu}]

    # --- 1. every power/ground pin rectangle of every hard macro on the stripe
    #        layer, in die coordinates, with the block net it is tied to
    set macros [list]
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
        set bx1 [$bb xMax]
        set by1 [$bb yMax]
        lappend macros [list $iname $bx0 $by0 $bx1 $by1]
        puts [format "LOOMPDN macro %s orient %s bbox %.3f %.3f %.3f %.3f status %s" $iname $orient \
            [expr {$bx0 * $um}] [expr {$by0 * $um}] [expr {$bx1 * $um}] [expr {$by1 * $um}] \
            [$inst getPlacementStatus]]
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
    if { [llength $macros] == 0 } {
        puts "LOOMPDN no hard macros; nothing to check"
        return
    }

    # --- 2. every stripe-layer shape of every net, checked against every macro
    set bad [list]
    foreach net [$block getNets] {
        if { ![$net isSpecial] } { continue }
        foreach swire [$net getSWires] {
            foreach sbox [$swire getWires] {
                if { [$sbox isVia] } { continue }
                if { [[$sbox getTechLayer] getName] ne $layer_name } { continue }
                set sx1 [$sbox xMin]
                set sy1 [$sbox yMin]
                set sx2 [$sbox xMax]
                set sy2 [$sbox yMax]
                foreach m $macros {
                    lassign $m iname mx0 my0 mx1 my1
                    if { $sx2 <= $mx0 || $sx1 >= $mx1 || $sy2 <= $my0 || $sy1 >= $my1 } { continue }
                    set desc [format "%s x %.3f..%.3f y %.3f..%.3f" [$net getName] \
                        [expr {$sx1 * $um}] [expr {$sx2 * $um}] [expr {$sy1 * $um}] [expr {$sy2 * $um}]]
                    set ylo [expr {max($sy1, $my0)}]
                    set yhi [expr {min($sy2, $my1)}]
                    # same-net pin rectangles that contain the stripe's x-range
                    set cover [list]
                    foreach c $columns {
                        lassign $c cnet key px1 py1 px2 py2
                        if { $cnet ne $net } { continue }
                        if { $px1 > $sx1 || $px2 < $sx2 } { continue }
                        if { $py2 <= $ylo || $py1 >= $yhi } { continue }
                        lappend cover [list $py1 $py2 $key]
                    }
                    set cover [lsort -integer -index 0 $cover]
                    set cur $ylo
                    set keys [list]
                    set why ""
                    foreach r $cover {
                        lassign $r py1 py2 key
                        if { $py1 > $cur } {
                            if { $cur == $ylo } {
                                set why [format "no same-net pin under it from y %.3f to %.3f" [expr {$cur * $um}] [expr {$py1 * $um}]]
                                break
                            }
                            if { $py1 - $cur > $max_gap } {
                                set why [format "crosses %.3f um without a same-net pin (y %.3f..%.3f)" \
                                    [expr {($py1 - $cur) * $um}] [expr {$cur * $um}] [expr {$py1 * $um}]]
                                break
                            }
                            puts [format "LOOMPDN   %s crosses a %.3f um gap between same-net pins at y %.3f..%.3f" \
                                $desc [expr {($py1 - $cur) * $um}] [expr {$cur * $um}] [expr {$py1 * $um}]]
                        }
                        set cur [expr {max($cur, $py2)}]
                        lappend keys $key
                    }
                    if { $why eq "" && $cur < $yhi } {
                        set why [format "no same-net pin under it from y %.3f to %.3f" [expr {$cur * $um}] [expr {$yhi * $um}]]
                    }
                    if { $why ne "" } {
                        lappend bad "$desc over $iname: $why"
                        continue
                    }
                    foreach key [lsort -unique $keys] { dict incr counts $key }
                    puts "LOOMPDN stripe $desc runs inside [join [lsort -unique $keys] { + }]"
                }
            }
        }
    }
    if { [llength $bad] > 0 } {
        foreach b $bad { puts "LOOMPDN BAD $b" }
        error "LOOMPDN: [llength $bad] stripe shape(s) over a macro are not inside same-net power pins (would short or float); fix FP_PDN_V* or the macro location"
    }

    # --- 3. every supply pin of every macro needs at least one stripe
    set missing [list]
    dict for {key n} $counts {
        puts "LOOMPDN $key: $n stripe(s)"
        if { $n == 0 } { lappend missing $key }
    }
    if { [llength $missing] > 0 } {
        error "LOOMPDN: no stripe runs through: [join $missing {, }]"
    }
}

# Wrap LibreLane's single `pdngen` call (scripts/openroad/pdn.tcl sources this
# file first): hard macros are un-fixed only while the real pdngen runs, so
# its core-grid stripes are not cut at them; then the result is verified and
# the grid is checked with errors that stop the step.
if { [info commands ::loom_pdngen_unwrapped] eq "" } {
    rename ::pdngen ::loom_pdngen_unwrapped
    proc ::pdngen { args } {
        foreach flag {-reset -ripup -report_only -check_only} {
            if { [lsearch -exact $args $flag] >= 0 } {
                return [::loom_pdngen_unwrapped {*}$args]
            }
        }
        set released [list]
        foreach inst [[ord::get_db_block] getInsts] {
            if { [[$inst getMaster] isBlock] && [$inst isFixed] } {
                lappend released [list $inst [$inst getPlacementStatus]]
                $inst setPlacementStatus "PLACED"
            }
        }
        set rc [catch { ::loom_pdngen_unwrapped {*}$args } msg opts]
        foreach r $released {
            lassign $r inst status
            $inst setPlacementStatus $status
        }
        if { $rc } {
            return -options $opts $msg
        }
        loom_check_macro_stripes
        foreach net_name [concat $::env(VDD_NETS) $::env(GND_NETS)] {
            puts "LOOMPDN check_power_grid -net $net_name"
            check_power_grid -net $net_name
        }
    }
}
