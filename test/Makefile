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
PROJECT_SOURCES += loom_core.v
PROJECT_SOURCES += loom_decode.v
PROJECT_SOURCES += loom_regfile.v
PROJECT_SOURCES += loom_alu.v
PROJECT_SOURCES += loom_timer.v
PROJECT_SOURCES += loom_pins.v

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DSIM
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_io/verilog/sg13cmos5l_io.v
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/verilog/sg13cmos5l_stdcell.v
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/verilog/sg13cmos5l_udp.v

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
# Gate-level simulation is far slower, so CI's gl_test runs the subset that
# proves the netlist end to end (host port, timing, UART). Run the full set at
# gate level by hand with: make GATES=yes COCOTB_TEST_MODULES=<list>
ifneq ($(GATES),yes)
COCOTB_TEST_MODULES ?= test_host,test_alu,test_ctrl,test_pins,test_timing,test_uart
else
COCOTB_TEST_MODULES ?= test_host,test_timing,test_uart
endif

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
