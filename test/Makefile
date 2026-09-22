# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
FST ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
# Keep in sync with source_files in info.yaml. loom_isa.vh is a header for
# testbenches and formal only, so it is deliberately not listed.
PROJECT_SOURCES  = tt_um_loom.v
PROJECT_SOURCES += loom_top.v
PROJECT_SOURCES += loom_sync.v
PROJECT_SOURCES += loom_spi_host.v
PROJECT_SOURCES += loom_host_ctl.v
PROJECT_SOURCES += loom_imem.v
PROJECT_SOURCES += loom_imem_macro.v
PROJECT_SOURCES += loom_core.v
PROJECT_SOURCES += loom_decode.v
PROJECT_SOURCES += loom_regfile.v
PROJECT_SOURCES += loom_alu.v
PROJECT_SOURCES += loom_timer.v
PROJECT_SOURCES += loom_pins.v
PROJECT_SOURCES += loom_fifo.v
PROJECT_SOURCES += loom_be.v

# The vendored simulation model of the SRAM macro (D-020), as on the sram-smoke
# branch. Needed for BOTH the RTL and the gate-level case: the netlist
# instantiates the macro as a blackbox, so the model has to be supplied either
# way. The core behavioural file comes first because the wrapper instantiates
# the module it defines, and -DFUNCTIONAL selects that behavioural path inside
# the wrapper (without it the wrapper uses its specify/timing-check branch).
# src/RM_IHPSG13_1P_512x16_c2_bm_bist.v (the synthesis and lint blackbox) is
# deliberately NOT in PROJECT_SOURCES: compiling it as well would redefine the
# module.
MACRO_DIR     = $(PWD)/../macro/RM_IHPSG13_1P_512x16_c2_bm_bist
MACRO_MODELS  = $(MACRO_DIR)/RM_IHPSG13_1P_core_behavioral_bm_bist.v
MACRO_MODELS += $(MACRO_DIR)/RM_IHPSG13_1P_512x16_c2_bm_bist.v

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl
COMPILE_ARGS    += -DFUNCTIONAL
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))
VERILOG_SOURCES += $(MACRO_MODELS)

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DSIM
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_io/verilog/sg13cmos5l_io.v
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/verilog/sg13cmos5l_stdcell.v
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/verilog/sg13cmos5l_udp.v
VERILOG_SOURCES += $(MACRO_MODELS)

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS 		+= -I$(SRC_DIR)

# Include the testbench sources:
VERILOG_SOURCES += $(PWD)/tb.v
TOPLEVEL = tb

# tools/ must be importable for tools.loomisa; the tests also add the repo
# root to sys.path themselves so this works when make is run from anywhere.
export PYTHONPATH := $(PWD)/..:$(PYTHONPATH)

# List test modules to run, separated by commas and without the .py suffix.
# Gate level runs every module that can see a netlist: about 35 minutes for
# 70 tests, inside the time precheck takes anyway (the gl_test job runs next
# to it). Three modules are RTL only: test_cosim (it skips itself on a
# netlist, having none of the internal signals it compares), test_flops (it
# drives the second, FLOPS-memory instance in tb.v) and test_fw (the L3
# firmware tests of tools/tests/test_fw_*.py through rtl_bench.py, which
# reads the retire record). test_mem runs on both: its one FLOPS-instance
# test skips itself when GATES=yes, exactly as test_flops does, and
# everything else in it goes through the pins.
ifneq ($(GATES),yes)
COCOTB_TEST_MODULES ?= test_host,test_alu,test_ctrl,test_pins,test_timing,test_uart,test_fifo,test_irq,test_be,test_setpd,test_mem,test_cosim,test_fw,test_flops
else
COCOTB_TEST_MODULES ?= test_host,test_alu,test_ctrl,test_pins,test_timing,test_uart,test_fifo,test_irq,test_be,test_setpd,test_mem
endif

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
