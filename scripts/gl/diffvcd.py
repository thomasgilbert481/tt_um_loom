"""Compare an RTL dump with a gate-level dump of the same test, clock by clock.

    python diffvcd.py RTL.vcd GL.vcd [--max N] [--xscan C1,C2,...]

Both dumps come from the same cocotb test (mkdump.py wrote the $dumpvars
lists), so clock k of one is clock k of the other until the first divergence.
Every GL bit net named after an RTL signal (``\\u_loom.u_core.pc_all[13]``)
is compared with that bit of the RTL vector, sampled 1 ps before each rising
edge of tb.clk. Reports:
  - the first cycles where a bit is 0 in one run and 1 in the other;
  - the first cycle after the top-level outputs first became fully known in
    the GL run at which one of them is X/Z again (RTL simulation hides X that
    the netlist propagates: BUGS 4), with every dumped GL signal that is X/Z
    at that cycle;
  - with --xscan, every X/Z GL signal at the listed cycles.
"""
import argparse
import re
from bisect import bisect_right


def parse(path):
    """VCD -> ({code: [(full name, width, lsb)]}, {code: ([times], [values])})."""
    ids, changes, scope, t = {}, {}, [], 0
    with open(path) as f:
        for line in f:
            if line.startswith("$enddefinitions"):
                break
            tok = line.split()
            if not tok:
                continue
            if tok[0] == "$scope":
                scope.append(tok[2])
            elif tok[0] == "$upscope":
                scope.pop()
            elif tok[0] == "$var":
                width, code, name = int(tok[2]), tok[3], tok[4]
                rng = tok[5] if len(tok) > 6 and tok[5].startswith("[") else ""
                full, lsb = ".".join(scope + [name]), 0
                if rng:
                    m = re.match(r"\[(\d+)(?::(\d+))?\]", rng)
                    if m.group(2) is not None:
                        lsb = int(m.group(2))
                    else:
                        full += rng
                ids.setdefault(code, []).append((full, width, lsb))
                changes.setdefault(code, ([], []))
        for line in f:
            c = line[:1]
            if c == "#":
                t = int(line[1:])
            elif c and c in "01xzXZ":
                ch = changes.get(line[1:].strip())
                if ch is not None:
                    ch[0].append(t)
                    ch[1].append(c.lower())
            elif c in ("b", "B"):
                val, code = line[1:].split()
                ch = changes.get(code)
                if ch is not None:
                    ch[0].append(t)
                    ch[1].append(val.lower())
    return ids, changes


def value_at(ch, t):
    i = bisect_right(ch[0], t) - 1
    return ch[1][i] if i >= 0 else "x"


def bit_of(vec, width, k):
    """Bit k (0 = LSB) of a VCD binary string that may be shorter than width."""
    if len(vec) < width:
        vec = ("0" if vec[0] in "01" else vec[0]) * (width - len(vec)) + vec
    return vec[width - 1 - k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rtl")
    ap.add_argument("gl")
    ap.add_argument("--max", type=int, default=12, help="0/1 divergences to report")
    ap.add_argument("--xscan", default="", help="cycles at which to list X/Z GL signals")
    args = ap.parse_args()

    rids, rch = parse(args.rtl)
    gids, gch = parse(args.gl)
    clk = next(code for code, lst in rids.items() for n, w, l in lst if n == "tb.clk")
    rises = [t for t, v in zip(*rch[clk]) if v == "1"]

    rsig = {}
    for code, lst in rids.items():
        for name, w, l in lst:
            if name.startswith("tb.user_project."):
                rsig[name[len("tb.user_project."):]] = (code, w, l)
            elif name.startswith("tb."):
                rsig[name] = (code, w, l)

    pairs = []          # (label, gl code, gl width, gl bit, rtl sig, rtl bit, is_top)
    gl_bits = {}        # GL base name -> [(bit, code)] for the X listings
    for code, lst in gids.items():
        for name, w, l in lst:
            if name.startswith("tb.") and not name.startswith("tb.user_project.") and name != "tb.clk":
                if name in rsig:
                    for k in range(w):
                        pairs.append(("%s[%d]" % (name, k), code, w, k, rsig[name], k, True))
                continue
            if not name.startswith("tb.user_project."):
                continue
            n = name[len("tb.user_project."):].lstrip("\\")
            m = re.match(r"(.*?)\[(\d+)\]$", n)
            base, bit = (m.group(1), int(m.group(2))) if m else (n, None)
            gl_bits.setdefault(base, []).append((bit or 0, code))
            if base not in rsig or w != 1:
                continue
            rc, rw, rl = rsig[base]
            k = (bit - rl) if bit is not None else 0
            if 0 <= k < rw:
                pairs.append((base if bit is None else "%s[%d]" % (base, bit),
                              code, 1, 0, (rc, rw, rl), k, False))
    print("comparing %d bits over %d clocks" % (len(pairs), len(rises)))

    def gl_x_signals(ts):
        out = []
        for base in sorted(gl_bits):
            bits = sorted(gl_bits[base])
            s = "".join(value_at(gch[c], ts)[-1] for _, c in reversed(bits))
            if "x" in s or "z" in s:
                out.append("%s=%s" % (base, s))
        return out

    reported, top_known, top_x = 0, None, None
    for cyc, tr in enumerate(rises):
        ts = tr - 1
        diffs, top_bad = [], False
        for label, gcode, gw, gk, (rc, rw, rl), rk, is_top in pairs:
            gv = value_at(gch[gcode], ts)
            gb = bit_of(gv, gw, gk) if gw > 1 else gv[-1]
            if is_top and gb in "xz":
                top_bad = True
            rv = value_at(rch[rc], ts)
            rb = bit_of(rv, rw, rk) if rw > 1 else rv[-1]
            if gb != rb and gb in "01" and rb in "01":
                diffs.append("%s rtl=%s gl=%s" % (label, rb, gb))
        if top_known is None and not top_bad:
            top_known = cyc
        elif top_known is not None and top_x is None and top_bad:
            top_x = (cyc, tr, gl_x_signals(ts))
        if diffs and reported < args.max:
            print("cycle %d t=%d: %d bits differ: %s" % (cyc, tr, len(diffs), "; ".join(sorted(diffs)[:40])))
            reported += 1
    if not reported:
        print("no 0/1 differences")
    if top_x:
        print("GL outputs went X/Z again at cycle %d t=%d (known from cycle %s); X/Z GL signals then:"
              % (top_x[0], top_x[1], top_known))
        print("  " + "\n  ".join(top_x[2]))
    else:
        print("GL outputs stay known after cycle %s" % top_known)
    for c in [int(x) for x in args.xscan.split(",") if x]:
        if 0 <= c < len(rises):
            print("xscan cycle %d: %s" % (c, " ".join(gl_x_signals(rises[c] - 1)) or "none"))


if __name__ == "__main__":
    main()
