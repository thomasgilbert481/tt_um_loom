"""tools.waves: capture a scenario's pad changes on the model and draw them."""
import json

from tools import waves


def test_waves_capture_and_draw_uart(tmp_path):
    js = tmp_path / "uart.json"
    svg = tmp_path / "uart.svg"
    waves.capture("test_fw_uart", "test_uart_tx_sends_a_host_pushed_string", str(js))
    d = json.loads(js.read_text())
    assert d["clk_ns"] == 20
    rec = d["benches"][0]
    assert len(rec) > 10
    cycles = [c[0] for c in rec]
    assert cycles == sorted(cycles)
    t1 = rec[-1][0] * 20 / 1000 + 1
    waves.draw(str(js), str(svg), 0.0, t1, ["TX=uo0"])
    s = svg.read_text()
    assert s.startswith("<svg") and s.rstrip().endswith("</svg>")
    assert "polyline" in s


def test_waves_capture_restores_bench_init(tmp_path):
    from tools.protomodels import bench as B
    before = B.Bench.__init__
    waves.capture("test_fw_uart", "test_uart_tx_sends_a_host_pushed_string",
                  str(tmp_path / "a.json"))
    assert B.Bench.__init__ is before
