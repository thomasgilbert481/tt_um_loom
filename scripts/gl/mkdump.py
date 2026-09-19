"""Write matching $dumpvars include files for an RTL run and a gate-level run.

The gate-level netlist keeps most RTL register names as escaped identifiers,
one net per bit (``\\u_loom.u_core.pc_all[13] ``). For every signal in SIGNALS
(plus any given with --extra) that the netlist still has, the GL include dumps
each bit net and the RTL include dumps the vector, so diffvcd.py can compare
the two runs clock by clock. Signals must exist under those names in the RTL
too, or the RTL compile fails; the list below is checked against loom_top and
loom_core as of D-022.

    python mkdump.py NETLIST OUT_DIR [--extra u_loom.u_core.foo,...]

Writes OUT_DIR/gl_dump.vh and OUT_DIR/rtl_dump.vh.
"""
import argparse
import pathlib
import re
import sys

SIGNALS = """
u_loom.run u_loom.halted u_loom.imem_rdata u_loom.u_imem.addr u_loom.u_imem.en
u_loom.cw_out_mask u_loom.cw_out_data u_loom.cw_oe_mask u_loom.cw_oe_data
u_loom.od_mask_reg u_loom.pin_in_reg u_loom.badop u_loom.sflags u_loom.swirq
u_loom.irq u_loom.resetpc_all u_loom.h_run u_loom.h_run_we u_loom.h_dbg_req
u_loom.u_core.pc_all u_loom.u_core.wa_all u_loom.u_core.z_all u_loom.u_core.c_all
u_loom.u_core.t_all u_loom.u_core.lat_valid_all u_loom.u_core.lat_val_all
u_loom.u_core.lat_pin_all u_loom.u_core.w_lat u_loom.u_core.w_lat_pin
u_loom.u_core.w_lat_val u_loom.u_core.woh_pc u_loom.u_core.woh_aux
u_loom.u_core.woh_lat u_loom.u_core.woh_rf u_loom.u_core.woh_tmr
u_loom.u_core.woh_be u_loom.u_core.woh_fifo u_loom.u_core.vd u_loom.u_core.vx
u_loom.u_core.pcd u_loom.u_core.pcx u_loom.u_core.fsel u_loom.u_core.op_a
u_loom.u_core.op_b u_loom.u_core.infl u_loom.u_core.now_all u_loom.u_core.td_all_w
u_loom.u_core.dt_all_w u_loom.u_core.tick_int_all u_loom.u_core.tick_frac_all
u_loom.u_core.steps_all u_loom.u_core.pp_all u_loom.u_core.outgrp_all
u_loom.u_core.ingrp_all u_loom.u_core.rs0_all u_loom.u_core.rs1_all
u_loom.u_core.depth_all u_loom.u_core.step_req_r u_loom.u_core.w_halt
u_loom.u_core.w_steps u_loom.u_core.w_prev_pins u_loom.u_core.w_wait_active
u_loom.u_core.w_rs_we u_loom.u_core.w_badop u_loom.u_core.td_th u_loom.u_core.tx_th
""".split()

TOP_PORTS = ("tb.clk", "tb.uo_out", "tb.uio_out", "tb.uio_oe")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("netlist")
    ap.add_argument("out_dir")
    ap.add_argument("--extra", default="", help="comma-separated extra RTL signal paths")
    args = ap.parse_args()

    wanted = SIGNALS + [s for s in args.extra.split(",") if s]
    text = pathlib.Path(args.netlist).read_text()
    found = {}
    for m in re.finditer(r"\\(u_loom\.[A-Za-z0-9_.]+)(\[\d+\])?\s", text):
        base, idx = m.group(1), m.group(2) or ""
        if base in wanted:
            found.setdefault(base, set()).add(idx)

    out = pathlib.Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    gl = ["$dumpvars(1, %s);" % p for p in TOP_PORTS]
    rtl = list(gl)
    for base in wanted:
        if base not in found:
            print("not in the netlist (skipped):", base, file=sys.stderr)
            continue
        for idx in sorted(found[base], key=lambda s: int(s[1:-1]) if s else -1):
            gl.append("$dumpvars(0, tb.user_project.\\%s%s );" % (base, idx))
        rtl.append("$dumpvars(0, tb.user_project.%s);" % base)
    (out / "gl_dump.vh").write_text("\n".join(gl) + "\n")
    (out / "rtl_dump.vh").write_text("\n".join(rtl) + "\n")
    print("%d GL bit nets for %d RTL signals" % (sum(len(v) for v in found.values()), len(found)),
          file=sys.stderr)


if __name__ == "__main__":
    main()
