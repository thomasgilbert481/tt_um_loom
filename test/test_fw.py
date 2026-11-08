# SPDX-License-Identifier: Apache-2.0
"""L3-UART-TX/RX, L3-SPI-MASTER, L3-SPI-SLAVE and L3-I2C-MASTER on the RTL.

The M2 exit criterion (``docs/PLAN.md``) is that the firmware passes its L3
tests "with data moving through the SPI host port". This module runs the very
bodies pytest runs on the golden model (``tools/tests/test_fw_uart.py``,
``test_fw_spi.py``, ``test_fw_spi_slave.py``, ``test_fw_i2c.py``) against
``src/`` instead: nothing is re-written here, the bodies simply get the RTL
backend (``test/rtl_bench.py``), so

* ``RtlBench`` clocks ``tb.v`` one edge per bench cycle and resolves the pads
  exactly as ``tools.protomodels.bench.Bench`` does for the model, and the
  unchanged ``tools.protomodels`` UART, SPI and I2C models sit on those pads;
* ``tools.loomhost.SimTransport`` clocks every host byte - the program image,
  the tick CSRs, ``RUN``, the FIFO pushes and pops, ``BADOP`` - through the
  real SPI pads (``ui_in[4:6]``, ``uo_out[7]``) of the design;
* the bodies run in a thread (``cocotb.task.bridge``) so that they can stay
  synchronous, and every ``bench.step`` enters the simulator through
  ``cocotb.task.resume``. See ``rtl_bench.py`` for why.

Each scenario resets the chip, loads its program through the host port and
runs it, so a failure here is a failure of the RTL, the firmware or the host
library, never of a previous test.

Scenarios marked ``model_only`` in those modules are skipped, with the reason
in the mark: a simulated clock costs a fraction of a millisecond here, so the
115200-baud UART cases (434 clocks a bit) and the middle of the SPI mode
sweep would cost minutes and prove what their faster siblings already prove.
Everything else runs on both backends, error cases included: UART framing
error, overrun, glitch and break; I2C NACK, refused data byte, clock
stretching and stuck-SCL timeout; SPI slave idle byte, dropped byte and a
transaction cut off in the middle of a byte.

``LOOM_FW_SET`` chooses the set: unset or ``default`` as above;
``model_only`` runs only the skipped scenarios, for the one long run that
shows they pass on the RTL too (``docs/PLAN.md`` M4); ``all`` runs both.
``LOOM_FW_ONLY`` (a comma-separated list of module names, such as
``test_fw_spi_slave``) keeps only those modules, for checking one program.
"""

import logging
import os
import sys

import cocotb

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from rtl_bench import RtlBackend, run_body                        # noqa: E402
from tools.tests import test_fw_i2c, test_fw_spi                  # noqa: E402
from tools.tests import test_fw_spi_slave, test_fw_uart           # noqa: E402
from tools.tests import test_fw_ps2, test_fw_ws2812               # noqa: E402
from tools.tests import test_fw_jtag, test_fw_swd                 # noqa: E402
from tools.tests import test_fw_usb                               # noqa: E402
from tools.tests import test_fw_i2c_slave, test_fw_can            # noqa: E402
from tools.tests import test_fw_manchester                        # noqa: E402
from tools.tests import test_fw_four_at_once                      # noqa: E402
from tools.tests import test_fw_pmod                              # noqa: E402
from tools.tests import test_fw_ir_nec                            # noqa: E402
from tools.tests import test_fw_onewire                           # noqa: E402
from tools.tests import test_fw_lin                               # noqa: E402
from tools.tests import test_fw_hdlc                              # noqa: E402
from tools.tests import test_fw_wiegand                           # noqa: E402
from tools.tests import test_fw_dmx                               # noqa: E402
from tools.tests import test_fw_dshot                             # noqa: E402
from tools.tests import test_fw_sent                              # noqa: E402
from tools.tests import test_fw_bridge_uart_dmx                   # noqa: E402
from tools.tests import test_fw_four_at_once_d035                 # noqa: E402
from tools.tests import test_fw_hot_swap                          # noqa: E402
from tools.tests import test_fw_dali                              # noqa: E402
from tools.tests import test_fw_ir_rc                             # noqa: E402
from tools.tests import test_fw_dcc                               # noqa: E402
from tools.tests import test_fw_modbus                            # noqa: E402
from tools.tests import test_fw_sdi12                             # noqa: E402
from tools.tests import test_fw_quadrature                        # noqa: E402
from tools.tests import test_fw_mdio                              # noqa: E402
from tools.tests import test_fw_audio                             # noqa: E402
from tools.tests import test_fw_bridge_wiegand_uart               # noqa: E402
from tools.tests import test_fw_bridge_ir_ws2812                  # noqa: E402
from tools.tests import test_fw_bridge_dshot_uart                 # noqa: E402
from tools.tests import test_fw_arinc429                          # noqa: E402
from tools.tests import test_fw_mil1553                           # noqa: E402
from tools.tests import test_fw_biss                              # noqa: E402
from tools.tests import test_fw_sbus                              # noqa: E402
from tools.tests import test_fw_dht22                             # noqa: E402
from tools.tests import test_fw_hx711                             # noqa: E402
from tools.tests import test_fw_tm1637                            # noqa: E402
from tools.tests import test_fw_ir_sirc                           # noqa: E402
from tools.tests import test_fw_knx                               # noqa: E402
from tools.tests import test_fw_iolink                            # noqa: E402
from tools.tests import test_fw_adb                               # noqa: E402
from tools.tests import test_fw_irda                              # noqa: E402
from tools.tests import test_fw_hdq                               # noqa: E402
from tools.tests import test_fw_joybus                            # noqa: E402
from tools.tests import test_fw_cec                               # noqa: E402
from tools.tests import test_fw_hd44780                           # noqa: E402
from tools.tests import test_fw_psi5                              # noqa: E402
from tools.tests import test_fw_cppm                              # noqa: E402
from tools.tests import test_fw_bell202                           # noqa: E402
from tools.tests import test_fw_ds1302                            # noqa: E402
from tools.tests import test_fw_morse                             # noqa: E402
from tools.tests import test_fw_microwire                         # noqa: E402
from tools.tests import test_fw_baudot                            # noqa: E402
from tools.tests import test_fw_x10                               # noqa: E402
from tools.tests import test_fw_padctl                            # noqa: E402
from tools.tests import test_fw_em4100                            # noqa: E402
from tools.tests import test_fw_ax25                              # noqa: E402
from tools.tests import test_fw_pocsag                            # noqa: E402
from tools.tests import test_fw_hub75                             # noqa: E402
from tools.tests import test_fw_vga                               # noqa: E402
from tools.tests import test_fw_iec62056                          # noqa: E402
from tools.tests import test_fw_lego_pf                           # noqa: E402
from tools.tests import test_fw_dtmf                              # noqa: E402
from tools.tests import test_fw_swim                              # noqa: E402
from tools.tests import test_fw_centronics                        # noqa: E402
from tools.tests import test_fw_gpib                              # noqa: E402
from tools.tests import test_fw_zxtape                            # noqa: E402
from tools.tests import test_fw_cbm_iec                           # noqa: E402
from tools.tests import test_fw_irig                              # noqa: E402
from tools.tests import test_fw_dcf77                             # noqa: E402
from tools.tests import test_fw_kcs                               # noqa: E402
from tools.tests import test_fw_j1708                             # noqa: E402
from tools.tests import test_fw_bldc                              # noqa: E402
from tools.tests import test_fw_j1850                             # noqa: E402
from tools.tests import test_fw_callerid                          # noqa: E402
from tools.tests import test_fw_lanc                              # noqa: E402
from tools.tests import test_fw_rdm                               # noqa: E402
from tools.tests import test_fw_ltc                               # noqa: E402
from tools.tests import test_fw_endat                             # noqa: E402
from tools.tests import test_fw_mdb                               # noqa: E402
from tools.tests import test_fw_kline                             # noqa: E402
from tools.tests import test_fw_nmea                              # noqa: E402
from tools.tests import test_fw_mbp                               # noqa: E402
from tools.tests import test_fw_magstripe                         # noqa: E402
from tools.tests import test_fw_wspr                              # noqa: E402
from tools.tests import test_fw_psk31                             # noqa: E402
from tools.tests import test_fw_unio                              # noqa: E402
from tools.tests.fw_backend import scenarios                      # noqa: E402

MODULES = (test_fw_uart, test_fw_spi, test_fw_spi_slave, test_fw_i2c,
           test_fw_ws2812, test_fw_ps2, test_fw_jtag, test_fw_swd,
           test_fw_usb, test_fw_i2c_slave, test_fw_can, test_fw_manchester,
           test_fw_pmod,
           test_fw_four_at_once, test_fw_ir_nec, test_fw_onewire,
           test_fw_lin, test_fw_hdlc, test_fw_wiegand, test_fw_dmx,
           test_fw_dshot, test_fw_sent,
           test_fw_bridge_uart_dmx, test_fw_four_at_once_d035,
           test_fw_hot_swap, test_fw_dali, test_fw_ir_rc, test_fw_dcc,
           test_fw_modbus, test_fw_sdi12, test_fw_quadrature,
           test_fw_mdio, test_fw_audio, test_fw_bridge_wiegand_uart,
           test_fw_bridge_ir_ws2812, test_fw_bridge_dshot_uart,
           test_fw_arinc429, test_fw_mil1553, test_fw_biss, test_fw_sbus,
           test_fw_dht22, test_fw_hx711, test_fw_tm1637, test_fw_ir_sirc,
           test_fw_knx, test_fw_iolink, test_fw_adb, test_fw_irda,
           test_fw_hdq, test_fw_joybus, test_fw_cec, test_fw_hd44780,
           test_fw_psi5, test_fw_cppm, test_fw_bell202, test_fw_ds1302,
           test_fw_morse, test_fw_microwire, test_fw_baudot, test_fw_x10,
           test_fw_padctl, test_fw_em4100, test_fw_ax25, test_fw_pocsag,
           test_fw_hub75, test_fw_vga, test_fw_iec62056, test_fw_lego_pf,
           test_fw_dtmf, test_fw_swim, test_fw_centronics, test_fw_gpib,
           test_fw_zxtape, test_fw_cbm_iec, test_fw_irig, test_fw_dcf77,
           test_fw_kcs, test_fw_j1708, test_fw_bldc, test_fw_j1850,
           test_fw_callerid, test_fw_lanc, test_fw_rdm, test_fw_ltc,
           test_fw_endat, test_fw_mdb, test_fw_kline, test_fw_nmea,
           test_fw_mbp, test_fw_magstripe, test_fw_wspr, test_fw_psk31,
           test_fw_unio)


def _make(scenario):
    """One cocotb test that runs ``scenario`` on the RTL backend."""

    async def run(dut):
        await run_body(dut, scenario.func, RtlBackend(dut), **scenario.kwargs)

    run.__name__ = run.__qualname__ = scenario.name
    run.__module__ = __name__
    run.__doc__ = "%s on the RTL%s" % (
        scenario.module, "" if not scenario.kwargs else
        " (" + ", ".join("%s=%s" % kv for kv in scenario.kwargs.items()) + ")")
    return cocotb.test()(run)


#: ``(name, reason)`` of every scenario that runs on the golden model only.
SKIPPED = []

_SET = os.environ.get("LOOM_FW_SET", "default")
_ONLY = [m for m in os.environ.get("LOOM_FW_ONLY", "").split(",") if m]
if _ONLY:
    _unknown = set(_ONLY) - {m.__name__.rsplit(".", 1)[-1] for m in MODULES}
    if _unknown:
        raise RuntimeError("LOOM_FW_ONLY names no such module: %s" % sorted(_unknown))
    MODULES = tuple(m for m in MODULES if m.__name__.rsplit(".", 1)[-1] in _ONLY)
if _SET not in ("default", "model_only", "all"):
    raise RuntimeError("LOOM_FW_SET=%r: expected default, model_only or all" % _SET)

for _module in MODULES:
    for _scenario in scenarios(_module):
        if _scenario.model_only and _SET == "default":
            SKIPPED.append((_scenario.name, _scenario.model_only))
            continue
        if not _scenario.model_only and _SET == "model_only":
            continue
        if _scenario.name in globals():
            raise RuntimeError("two firmware scenarios are called %r" % _scenario.name)
        globals()[_scenario.name] = _make(_scenario)

_log = logging.getLogger("cocotb.test_fw")
for _name, _reason in SKIPPED:
    _log.info("model only, not run here: %s (%s)", _name, _reason)
