# tt_um_loom on an iCEBreaker (iCE40UP5K), M2 bench build.
# From a WSL shell after `source scripts/dev_env.sh`:
#     make -C fpga/icebreaker            # bitstream and reports in build/
#     make -C fpga/icebreaker prog       # flash it (board on USB)

PROJ    = loom_icebreaker
SRC_DIR = ../../src
BUILD   = build
FREQ    = 12

# The silicon sources from info.yaml. IMEM_IMPL "FLOPS" means the macro branch
# of loom_imem is never built, but Yosys still checks loom_imem at its default
# parameters (the macro), so the macro's port blackbox from src/ is read as a
# library cell and synthesis prunes the unused branch.
SRCS = $(PROJ).v \
       $(addprefix $(SRC_DIR)/, tt_um_loom.v loom_top.v loom_sync.v \
         loom_spi_host.v loom_host_ctl.v loom_imem.v loom_imem_macro.v \
         loom_core.v loom_decode.v loom_regfile.v loom_alu.v loom_timer.v \
         loom_pins.v loom_fifo.v loom_be.v)
MACRO_BB = $(SRC_DIR)/RM_IHPSG13_1P_512x16_c2_bm_bist.v

all: $(BUILD)/$(PROJ).bin

$(BUILD):
	mkdir -p $@

$(BUILD)/$(PROJ).json: $(SRCS) | $(BUILD)
	yosys -q -l $(BUILD)/yosys.log \
	  -p "read_verilog -sv -lib $(MACRO_BB); read_verilog -sv -I$(SRC_DIR) $(SRCS); synth_ice40 -top $(PROJ) -json $@; tee -o $(BUILD)/utilisation.txt stat"

$(BUILD)/$(PROJ).asc: $(BUILD)/$(PROJ).json icebreaker.pcf
	nextpnr-ice40 --up5k --package sg48 --json $< --pcf icebreaker.pcf \
	  --asc $@ --freq $(FREQ) --report $(BUILD)/nextpnr_report.json \
	  2> $(BUILD)/nextpnr.log

$(BUILD)/$(PROJ).bin: $(BUILD)/$(PROJ).asc
	icepack $< $@

prog: $(BUILD)/$(PROJ).bin
	iceprog $<

clean:
	rm -rf $(BUILD)

.PHONY: all prog clean
