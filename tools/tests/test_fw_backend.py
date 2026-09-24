"""The collection ``test/test_fw.py`` uses to run these bodies on the RTL.

``tools/tests/fw_backend.py`` turns the L3 firmware modules into scenarios:
one per test body per parameter set. A body that took no ``backend``, or a
name that collided with another module's, would silently stop being run on
the RTL, so it is checked here rather than noticed later.
"""

import inspect

import pytest

from tools.tests import test_fw_i2c, test_fw_spi, test_fw_spi_slave, test_fw_uart
from tools.tests.fw_backend import MODEL, model_only, scenarios

MODULES = (test_fw_uart, test_fw_spi, test_fw_spi_slave, test_fw_i2c)


def bodies(module):
    return [name for name, obj in vars(module).items()
            if name.startswith("test_") and inspect.isfunction(obj)]


@pytest.mark.parametrize("module", MODULES, ids=lambda m: m.__name__.rsplit(".", 1)[-1])
def test_every_body_takes_a_backend(module):
    """Otherwise it runs on the model only, without anyone saying so."""
    found = {s.func.__name__ for s in scenarios(module)}
    assert found == set(bodies(module))


def test_names_are_unique_across_the_modules():
    names = [s.name for m in MODULES for s in scenarios(m)]
    assert len(names) == len(set(names))


def test_parameters_are_expanded_one_scenario_per_case():
    found = {s.name: s.kwargs for s in scenarios(test_fw_uart)
             if s.func.__name__ == "test_uart_rx_tolerates_three_per_cent_at_the_fastest_rate"}
    assert found == {
        "test_uart_rx_tolerates_three_per_cent_at_the_fastest_rate_skew_0_97": {"skew": 0.97},
        "test_uart_rx_tolerates_three_per_cent_at_the_fastest_rate_skew_1_03": {"skew": 1.03}}


def test_every_module_has_scenarios_for_both_backends():
    for module in MODULES:
        found = list(scenarios(module))
        assert found and any(s.model_only is None for s in found), module.__name__
        for scenario in found:
            assert scenario.model_only is None or scenario.model_only.strip()


def test_model_only_marks_the_function_and_the_scenarios():
    @model_only("because")
    def test_example(backend):
        pass

    assert test_example.model_only == "because"


def test_the_model_backend_builds_a_bench_and_a_host():
    bench = MODEL.bench(pullups=0b11)
    loom = MODEL.loom(bench)
    assert loom.transport.bench is bench
    assert loom.id() == 0x4C4D and bench.cycle > 0


def test_the_model_backend_stops_at_a_read_of_memory_never_loaded():
    """docs/spec-questions/firmware-m3.md item 9: what test/tb.v's X check
    does on the RTL. Thread 0 runs one NOP and fetches word 1, which no image
    loaded."""
    from tools.loomisa import load
    from tools.tests.fw_backend import UnloadedReadError
    bench = MODEL.bench(image={0: load().encode("NOP")})
    bench.machine.host_set_run(0b0001)
    with pytest.raises(UnloadedReadError, match="0x001"):
        bench.step(20)
