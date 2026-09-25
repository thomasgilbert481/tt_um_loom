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
from tools.tests.fw_backend import scenarios                      # noqa: E402

MODULES = (test_fw_uart, test_fw_spi, test_fw_spi_slave, test_fw_i2c,
           test_fw_ws2812, test_fw_ps2, test_fw_jtag, test_fw_swd,
           test_fw_usb, test_fw_i2c_slave, test_fw_can, test_fw_manchester)


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
