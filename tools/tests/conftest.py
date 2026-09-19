"""The ``backend`` fixture of the L3 firmware tests (``tools/tests/fw_backend.py``).

pytest runs the bodies on the golden model; the same bodies run on the RTL
under cocotb (``test/test_fw.py``), where the backend is
``test/rtl_bench.py``'s ``RtlBackend``. The fixture is parametrised so the
backend shows in the test id (``...[model]``).
"""

import pytest

from tools.tests.fw_backend import MODEL


@pytest.fixture(params=[MODEL], ids=[MODEL.name])
def backend(request):
    return request.param
