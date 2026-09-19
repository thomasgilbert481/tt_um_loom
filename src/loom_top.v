/*
 * loom_top: everything below the Tiny Tapeout wrapper. All parameters live
 * here (docs/ARCHITECTURE.md section 14).
 * SPDX-License-Identifier: Apache-2.0
 *
 * Timing contract: this module only wires submodules together and owns the
 * single-port arbitration for the instruction memory. The host wins the port
 * whenever it asks for it, which it is only allowed to do while `core_busy`
 * is low (RUN == 0 and no step in flight), so a core fetch is never lost.
 * All pad outputs are registers inside loom_pins or loom_spi_host.
 *
 * The retire record (tr_*) is simulation and debug only; it is exported so a
 * testbench can reach it, and left unconnected in tt_um_loom.
 */

`default_nettype none

module loom_top #(
    parameter [15:0]  IMEM_WORDS = 16'd256,
    parameter integer FIFO_DEPTH = 4,
    parameter [15:0]  ID_VALUE   = 16'h4C4D,
    parameter [15:0]  VERSION    = 16'h0002
) (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       clk,
    input  wire       rst_n,

    // Retire record (docs/SEMANTICS.md section 8), valid during W.
    output wire       tr_valid,
    output wire [1:0] tr_thread,
    output wire [9:0] tr_pc,
    output wire [15:0] tr_ir,
    output wire       tr_done,
    output wire       tr_we,
    output wire [2:0] tr_rd,
    output wire [15:0] tr_val,
    output wire [2:0] tr_flags,
    output wire [9:0] tr_next_pc
);

  localparam integer IMEM_AW = $clog2(IMEM_WORDS);
  localparam [31:0]  IMEM_LOG2 = $clog2(IMEM_WORDS);
  localparam [31:0]  FIFO_LOG2 = $clog2(FIFO_DEPTH);
  // CAPS (docs/SEMANTICS.md 5): [15:12] log2(IMEM_WORDS), [11:9] zero,
  // [8] bit engine auto mode, [7] deadline-latched SETP, [6] boot ROM,
  // [5] data memory, [4] bit engine (manual mode), [3] FIFOs,
  // [2:0] log2(FIFO_DEPTH).
  localparam [15:0]  CAPS_VAL = {IMEM_LOG2[3:0], 3'd0,
                                 1'b0,    // [8] bit engine auto mode (M3)
                                 1'b1,    // [7] deadline-latched SETP
                                 1'b0,    // [6] boot ROM
                                 1'b0,    // [5] data memory
                                 1'b1,    // [4] bit engine, manual mode
                                 1'b1,    // [3] FIFOs
                                 FIFO_LOG2[2:0]};

  // --------------------------------------------------------------- host SPI
  wire       cs_active, byte_done, miso;
  wire [7:0] rx_byte, tx_byte;

  loom_spi_host u_spi (
      .clk(clk), .rst_n(rst_n),
      .cs_n_pad(ui_in[4]), .sck_pad(ui_in[5]), .mosi_pad(ui_in[6]),
      .miso(miso), .cs_active(cs_active),
      .byte_done(byte_done), .rx_byte(rx_byte), .tx_byte(tx_byte)
  );

  // ------------------------------------------------------------------- pins
  wire [31:0] pin_in_vec;
  wire [15:0] pin_in_reg, pin_out_reg;
  wire [7:0]  pin_oe_reg, od_mask_reg;
  wire        cw_valid, cw_od_we;
  wire [13:0] cw_out_mask, cw_out_data;
  wire [7:0]  cw_oe_mask, cw_oe_data, cw_od;
  wire [13:0] lw_out_mask, lw_out_data;
  wire [7:0]  lw_oe_mask, lw_oe_data;
  wire        h_pout_we, h_poe_we, h_od_we;
  wire [15:0] h_pout;
  wire [7:0]  h_poe, h_od;
  wire [5:0]  out_pins;

  loom_pins u_pins (
      .clk(clk), .rst_n(rst_n),
      .pad_in({ui_in[7], ui_in[3:0], uio_in}),
      .cw_valid(cw_valid), .cw_out_mask(cw_out_mask), .cw_out_data(cw_out_data),
      .cw_oe_mask(cw_oe_mask), .cw_oe_data(cw_oe_data),
      .cw_od_we(cw_od_we), .cw_od(cw_od),
      .lw_out_mask(lw_out_mask), .lw_out_data(lw_out_data),
      .lw_oe_mask(lw_oe_mask), .lw_oe_data(lw_oe_data),
      .h_out_we(h_pout_we), .h_out(h_pout),
      .h_oe_we(h_poe_we), .h_oe(h_poe),
      .h_od_we(h_od_we), .h_od(h_od),
      .pin_in_vec(pin_in_vec), .pin_in_reg(pin_in_reg),
      .pin_out_reg(pin_out_reg), .pin_oe_reg(pin_oe_reg),
      .od_mask_reg(od_mask_reg),
      .uio_out(uio_out), .uio_oe(uio_oe), .out_pins(out_pins)
  );

  // ------------------------------------------------------ instruction memory
  wire [IMEM_AW-1:0] core_imem_addr, h_imem_addr;
  wire               core_imem_en, h_imem_req, h_imem_we;
  wire [15:0]        imem_rdata, h_imem_wdata;

  loom_imem #(.WORDS(IMEM_WORDS), .AW(IMEM_AW)) u_imem (
      .clk(clk),
      .en   (h_imem_req | core_imem_en),
      .we   (h_imem_req & h_imem_we),
      .addr (h_imem_req ? h_imem_addr : core_imem_addr),
      .wdata(h_imem_wdata),
      .rdata(imem_rdata)
  );

  // ------------------------------------------------------------------- core
  wire        h_run_we, h_step_we, h_rpc_we, h_sfset_we, h_sfclr_we;
  wire        h_badop_clr_we, h_badop_set15, h_swirq_clr_we;
  wire [3:0]  h_run, h_reset, h_step, h_swirq_clr;
  wire [1:0]  h_rpc_sel;
  wire [9:0]  h_rpc;
  wire [7:0]  h_sfset, h_sfclr;
  wire [15:0] h_badop_clr;
  wire        h_dbg_req, h_dbg_wr, h_dbg_ack;
  wire [1:0]  h_dbg_thread;
  wire [7:0]  h_dbg_reg;
  wire [15:0] h_dbg_wdata, h_dbg_rdata;
  wire [3:0]  run, halted, swirq;
  wire [15:0] badop;
  wire [7:0]  sflags;
  wire [39:0] resetpc_all;
  wire        core_busy, irq;
  wire [3:0]  h_inq_push, h_outq_pop;
  wire [15:0] h_fifo_wdata;
  wire        h_badop_set14;
  wire [47:0] fifo_stat;
  wire [63:0] outq_head, outq_next;

  loom_core #(.IMEM_AW(IMEM_AW), .IMEM_WORDS(IMEM_WORDS),
              .FIFO_DEPTH(FIFO_DEPTH)) u_core (
      .clk(clk), .rst_n(rst_n),
      .imem_addr(core_imem_addr), .imem_en(core_imem_en),
      .imem_rdata(imem_rdata),
      .pin_in_vec(pin_in_vec), .pin_in_reg(pin_in_reg),
      .pin_out_reg(pin_out_reg), .pin_oe_reg(pin_oe_reg),
      .od_mask_reg(od_mask_reg),
      .cw_valid(cw_valid), .cw_out_mask(cw_out_mask), .cw_out_data(cw_out_data),
      .cw_oe_mask(cw_oe_mask), .cw_oe_data(cw_oe_data),
      .cw_od_we(cw_od_we), .cw_od(cw_od),
      .lw_out_mask(lw_out_mask), .lw_out_data(lw_out_data),
      .lw_oe_mask(lw_oe_mask), .lw_oe_data(lw_oe_data),
      .h_run_we(h_run_we), .h_run(h_run), .h_reset(h_reset),
      .h_step_we(h_step_we), .h_step(h_step),
      .h_rpc_we(h_rpc_we), .h_rpc_sel(h_rpc_sel), .h_rpc(h_rpc),
      .h_sfset_we(h_sfset_we), .h_sfset(h_sfset),
      .h_sfclr_we(h_sfclr_we), .h_sfclr(h_sfclr),
      .h_badop_clr_we(h_badop_clr_we), .h_badop_clr(h_badop_clr),
      .h_badop_set15(h_badop_set15),
      .h_swirq_clr_we(h_swirq_clr_we), .h_swirq_clr(h_swirq_clr),
      .h_inq_push(h_inq_push), .h_fifo_wdata(h_fifo_wdata),
      .h_outq_pop(h_outq_pop), .h_badop_set14(h_badop_set14),
      .fifo_stat(fifo_stat), .outq_head(outq_head), .outq_next(outq_next),
      .h_dbg_req(h_dbg_req), .h_dbg_wr(h_dbg_wr),
      .h_dbg_thread(h_dbg_thread), .h_dbg_reg(h_dbg_reg),
      .h_dbg_wdata(h_dbg_wdata), .h_dbg_ack(h_dbg_ack),
      .h_dbg_rdata(h_dbg_rdata),
      .run(run), .halted(halted), .badop(badop), .sflags(sflags),
      .swirq(swirq), .resetpc_all(resetpc_all), .core_busy(core_busy),
      .tr_valid(tr_valid), .tr_thread(tr_thread), .tr_pc(tr_pc), .tr_ir(tr_ir),
      .tr_done(tr_done), .tr_we(tr_we), .tr_rd(tr_rd), .tr_val(tr_val),
      .tr_flags(tr_flags), .tr_next_pc(tr_next_pc)
  );

  // ------------------------------------------------------------ host layer
  loom_host_ctl #(
      .IMEM_AW(IMEM_AW), .ID_VALUE(ID_VALUE), .VERSION(VERSION), .CAPS(CAPS_VAL)
  ) u_host (
      .clk(clk), .rst_n(rst_n),
      .cs_active(cs_active), .byte_done(byte_done),
      .rx_byte(rx_byte), .tx_byte(tx_byte),
      .h_imem_req(h_imem_req), .h_imem_we(h_imem_we),
      .h_imem_addr(h_imem_addr), .h_imem_wdata(h_imem_wdata),
      .h_imem_rdata(imem_rdata),
      .h_run_we(h_run_we), .h_run(h_run), .h_reset(h_reset),
      .h_step_we(h_step_we), .h_step(h_step),
      .h_rpc_we(h_rpc_we), .h_rpc_sel(h_rpc_sel), .h_rpc(h_rpc),
      .h_sfset_we(h_sfset_we), .h_sfset(h_sfset),
      .h_sfclr_we(h_sfclr_we), .h_sfclr(h_sfclr),
      .h_badop_clr_we(h_badop_clr_we), .h_badop_clr(h_badop_clr),
      .h_badop_set15(h_badop_set15),
      .h_swirq_clr_we(h_swirq_clr_we), .h_swirq_clr(h_swirq_clr),
      .h_inq_push(h_inq_push), .h_fifo_wdata(h_fifo_wdata),
      .h_outq_pop(h_outq_pop), .h_badop_set14(h_badop_set14),
      .fifo_stat(fifo_stat), .outq_head(outq_head), .outq_next(outq_next),
      .h_dbg_req(h_dbg_req), .h_dbg_wr(h_dbg_wr),
      .h_dbg_thread(h_dbg_thread), .h_dbg_reg(h_dbg_reg),
      .h_dbg_wdata(h_dbg_wdata), .h_dbg_ack(h_dbg_ack),
      .h_dbg_rdata(h_dbg_rdata),
      .h_pout_we(h_pout_we), .h_pout(h_pout),
      .h_poe_we(h_poe_we), .h_poe(h_poe),
      .h_od_we(h_od_we), .h_od(h_od),
      .run(run), .halted(halted), .badop(badop), .sflags(sflags),
      .swirq(swirq), .resetpc_all(resetpc_all), .core_busy(core_busy),
      .pin_out_reg(pin_out_reg), .pin_oe_reg(pin_oe_reg),
      .od_mask_reg(od_mask_reg), .pin_in_reg(pin_in_reg),
      .irq(irq)
  );

  assign uo_out = {miso, irq, out_pins};

endmodule
