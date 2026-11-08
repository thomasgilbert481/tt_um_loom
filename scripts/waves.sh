#!/bin/bash
# waves.sh [OUT_DIR]: capture the demo scenarios on the golden model and draw
# their timing diagrams as SVG (tools/waves.py). Run from the repo root in the
# dev environment (source scripts/dev_env.sh). Default OUT_DIR: /tmp/loom_waves.
set -e
O=${1:-/tmp/loom_waves}; mkdir -p "$O"
PY=${LOOM_PY:-python3}
C="$PY -m tools.waves capture"; W="$PY -m tools.waves draw"
$C test_fw_four_at_once_d035 test_four_d035_at_50mhz_ticks "$O/four.json"
$C test_fw_hot_swap test_swap_dshot_for_ws2812_while_three_run "$O/swap.json"
$C test_fw_bridge_ir_ws2812 test_bridge_a_frame_and_its_repeat_change_the_strip_as_mapped "$O/irws.json"
$W "$O/four.json" "$O/four_overview.svg" 1150 2900 bench=0 "DMX in=ui0" "DShot=uo0" "NEC LED=uo1" "Wieg D0=uio0" "Wieg D1=uio1"
$W "$O/four.json" "$O/four_zoom.svg" 1600 1700 bench=0 "DMX in=ui0" "DShot=uo0" "NEC LED=uo1" "Wieg D0=uio0" "Wieg D1=uio1"
$W "$O/swap.json" "$O/swap_with.svg" 1640 2450 bench=0 "UART t0=uo0" "DMX t1=uo1" "SCL t2=uio0" "SDA t2=uio1" "t3 A/B=uo3"
$W "$O/swap.json" "$O/swap_without.svg" 1640 2450 bench=1 "UART t0=uo0" "DMX t1=uo1" "SCL t2=uio0" "SDA t2=uio1" "t3 A=uo3"
$W "$O/swap.json" "$O/swap_zoom.svg" 1640 1800 bench=0 "UART t0=uo0" "DMX t1=uo1" "SCL t2=uio0" "SDA t2=uio1" "t3 A/B=uo3"
$W "$O/swap.json" "$O/swap_b.svg" 2040 2120 bench=0 "UART t0=uo0" "DMX t1=uo1" "SCL t2=uio0" "SDA t2=uio1" "t3 B=uo3"
$W "$O/irws.json" "$O/irws_overview.svg" 1800 6600 bench=0 "IR in=ui0" "WS2812=uo0"
$W "$O/irws.json" "$O/irws_zoom.svg" 1840 1900 bench=0 "IR in=ui0" "WS2812=uo0"
ls -la "$O"/*.svg
