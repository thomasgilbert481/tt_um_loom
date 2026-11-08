"""cocotb module: run one firmware test module's bodies on the RTL.

``test/test_fw.py`` runs the modules listed in its ``MODULES``, which is
how a module gets into CI. This module runs any one module without that
edit, so a new protocol can be tried on the RTL straight after the
scaffold:

    bash scripts/try_rtl.sh test_fw_<name> [SUBSTRING]

which runs ``make -C test COCOTB_TEST_MODULES=tools.loomnew.rtl`` with

* ``LOOM_RTL_MODULE``: the module name in ``tools/tests`` (required);
* ``LOOM_RTL_MATCH``: keep only scenarios whose name contains it;
* ``LOOM_FW_SET``: ``default`` skips ``model_only`` bodies (as CI does),
  ``all`` runs them too.

The bodies, the RTL backend and the bridge are those of ``test/test_fw.py``
(``test/rtl_bench.py``): nothing is re-implemented here.
"""

import importlib
import os
import sys

import cocotb

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for _p in (_REPO, os.path.join(_REPO, "test")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from rtl_bench import RtlBackend, run_body                        # noqa: E402
from tools.tests.fw_backend import scenarios                      # noqa: E402

_NAME = os.environ.get("LOOM_RTL_MODULE", "")
_MATCH = os.environ.get("LOOM_RTL_MATCH", "")
_SET = os.environ.get("LOOM_FW_SET", "default")
if not _NAME:
    raise RuntimeError("set LOOM_RTL_MODULE to a module of tools/tests, "
                       "for example test_fw_stepper")
_MODULE = importlib.import_module("tools.tests." + _NAME)

SKIPPED = []


def _make(scenario):
    async def run(dut):
        await run_body(dut, scenario.func, RtlBackend(dut), **scenario.kwargs)

    run.__name__ = run.__qualname__ = scenario.name
    run.__module__ = __name__
    run.__doc__ = "%s on the RTL" % scenario.module
    return cocotb.test()(run)


for _scenario in scenarios(_MODULE):
    if _MATCH and _MATCH not in _scenario.name:
        continue
    if _scenario.model_only and _SET != "all":
        SKIPPED.append((_scenario.name, _scenario.model_only))
        continue
    globals()[_scenario.name] = _make(_scenario)

for _name, _reason in SKIPPED:
    print("model only, not run here: %s (%s)" % (_name, _reason))
