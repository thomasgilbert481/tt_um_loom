# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
FST ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
MACRO_DIR = $(PWD)/../macro/RM_IHPSG13_1P_512x16_c2_bm_bist
PROJECT_SOURCES = tt_um_loom.v loom_imem_macro.v

# The vendored SRAM macro simulation model. Needed for BOTH the RTL and the
# gate-level case: the synthesised netlist instantiates the macro as a
# blackbox, so the model has to be supplied either way. The core behavioural
# file must come first because the wrapper instantiates the module it defines.
# -DFUNCTIONAL selects the behavioural path inside the wrapper; without it the
# wrapper uses its specify/timing-check branch instead.
# src/RM_IHPSG13_1P_512x16_c2_bm_bist.v (the synthesis blackbox) is deliberately
# NOT in PROJECT_SOURCES: compiling it as well would redefine the module.
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

# List test modules to run, separated by commas and without the .py suffix:
COCOTB_TEST_MODULES = test

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
