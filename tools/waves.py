"""tools.waves: run one firmware scenario on the golden model and record every
pad change, then draw timing diagrams as SVG.

    python -m tools.waves capture MODULE SCENARIO OUT.json
    python -m tools.waves draw IN.json OUT.svg T0_US T1_US LABEL=PIN [LABEL=PIN ...]

PIN is a pin name from isa.yaml (IN0, OUT3, BIDIR5) or a raw pad (uo6).
Times are microseconds at 50 MHz (20 ns a clock). A scenario with several
benches records each; `bench=N` as the first spec draws bench N (default 0).
scripts/waves.sh captures and draws the demo set.
"""
import importlib
import json
import sys

CLK_NS = 20


def capture(module, scenario, out):
    from tools.protomodels import bench as B
    from tools.tests import fw_backend as F
    benches = []
    orig = B.Bench.__init__

    def init(self, *a, **k):
        orig(self, *a, **k)
        last = [None]
        rec = []
        benches.append(rec)

        class Rec(B.Model):
            def observe(self, lines):
                v = (lines.ui, lines.uio, lines.uo)
                if v != last[0]:
                    rec.append((lines.cycle,) + v)
                    last[0] = v
        self.add(Rec())
    B.Bench.__init__ = init
    try:
        mod = importlib.import_module("tools.tests." + module)
        for sc in F.scenarios(mod):
            if sc.name == scenario:
                sc.func(backend=F.ModelBackend(), **sc.kwargs)
                break
        else:
            raise SystemExit("no scenario " + scenario)
    finally:
        B.Bench.__init__ = orig
    json.dump({"module": module, "scenario": scenario, "clk_ns": CLK_NS,
               "benches": benches}, open(out, "w"))
    for i, rec in enumerate(benches):
        print("bench", i, len(rec), "changes, last cycle", rec[-1][0] if rec else None)


def pad(pin):
    if pin[:2] in ("uo", "ui") and pin[2:].isdigit():
        return pin[:2], int(pin[2:])
    if pin.startswith("uio") and pin[3:].isdigit():
        return "uio", int(pin[3:])
    from tools.protomodels.bench import pad_of
    return pad_of(pin)


def draw(inp, out, t0, t1, specs):
    d = json.load(open(inp))
    which = 0
    if specs and specs[0].startswith("bench="):
        which = int(specs[0].split("=")[1])
        specs = specs[1:]
    ch = d["benches"][which]
    c0, c1 = int(t0 * 1000 / CLK_NS), int(t1 * 1000 / CLK_NS)
    idx = {"ui": 1, "uio": 2, "uo": 3}
    W, LH, LM, TOP = 900, 34, 120, 24
    H = TOP + LH * len(specs) + 30
    sx = (W - LM - 20) / max(1, c1 - c0)
    s = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 %d %d" '
         'font-family="monospace" font-size="12">' % (W, H),
         '<rect width="100%" height="100%" fill="#ffffff"/>']
    for row, spec in enumerate(specs):
        label, pin = spec.split("=")
        kind, bit = pad(pin)
        y_hi = TOP + row * LH + 6
        y_lo = y_hi + LH - 14
        s.append('<text x="8" y="%d" fill="#222">%s</text>' % (y_lo, label))
        v = 0
        for c in ch:
            if c[0] > c0:
                break
            v = (c[idx[kind]] >> bit) & 1
        pts = [(LM, y_hi if v else y_lo)]
        for c in ch:
            if c[0] <= c0 or c[0] >= c1:
                continue
            nv = (c[idx[kind]] >> bit) & 1
            if nv != v:
                x = LM + (c[0] - c0) * sx
                pts.append((x, y_hi if v else y_lo))
                pts.append((x, y_hi if nv else y_lo))
                v = nv
        pts.append((W - 20, y_hi if v else y_lo))
        s.append('<polyline fill="none" stroke="#1f5fbf" stroke-width="1.2" '
                 'points="%s"/>' % " ".join("%.1f,%.1f" % p for p in pts))
    yb = H - 14
    for k in range(6):
        t = t0 + (t1 - t0) * k / 5
        x = LM + (W - LM - 20) * k / 5
        s.append('<line x1="%.1f" y1="%d" x2="%.1f" y2="%d" stroke="#bbb"/>'
                 % (x, TOP, x, yb - 10))
        s.append('<text x="%.1f" y="%d" fill="#444" text-anchor="middle">'
                 '%.1f us</text>' % (x, yb, t))
    s.append("</svg>")
    open(out, "w").write("\n".join(s))


if __name__ == "__main__":
    if sys.argv[1] == "capture":
        capture(*sys.argv[2:5])
    else:
        draw(sys.argv[2], sys.argv[3], float(sys.argv[4]), float(sys.argv[5]),
             sys.argv[6:])
