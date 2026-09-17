/*
 * loom_host_ctl: host command layer (docs/HOST_PROTOCOL.md).
 * SPDX-License-Identifier: Apache-2.0
 *
 * Consumes the byte stream from loom_spi_host and turns it into register and
 * memory accesses:  CMD = {RW, SPACE[2:0], 0000}, ADDR = 16 bits MSB first,
 * then 16-bit words MSB first with the address auto-incrementing. A read
 * inserts one dummy byte after the address. CS_n rising ends the transaction
 * at any point and a word cut in half is discarded.
 *
 * M1 builds spaces 0 (CTRL), 1 (IMEM), 4 (DEBUG) and 5 (STEP). Spaces 2
 * (DMEM) and 3 (FIFO) read 0 and ignore writes until M2.
 *
 * Timing contract:
 *   Every effect of a write word is a one-clock pulse on the corresponding
 *   output, asserted two clocks after the last SCK rising edge of that word,
 *   so it commits at a clock edge like any other register write (SEMANTICS 7).
 *   A read word is fetched as soon as the address byte completes (and, for
 *   the following words, one full byte time ahead), which is at least 8 SCK
 *   periods of slack for the first word and 2 byte times afterwards.
 *   `tx_byte` is updated in the cycle after `byte_done` and is therefore
 *   stable before loom_spi_host loads it on the next falling SCK edge.
 *   Host IMEM access is refused unless `core_busy` is low (RUN == 0 and no
 *   step in flight): writes are dropped, reads return 0, and BADOP[15] is set.
 */

`default_nettype none

module loom_host_ctl #(
    parameter integer IMEM_AW   = 8,
    parameter [15:0]  ID_VALUE  = 16'h4C4D,
    parameter [15:0]  VERSION   = 16'h0001,
    parameter [15:0]  CAPS      = 16'h0100
) (
    input  wire              clk,
    input  wire              rst_n,

    // ------------------------------------------------------- SPI byte layer
    input  wire              cs_active,
    input  wire              byte_done,
    input  wire [7:0]        rx_byte,
    output wire [7:0]        tx_byte,

    // ------------------------------------------------------------ IMEM port
    output wire              h_imem_req,
    output wire              h_imem_we,
    output wire [IMEM_AW-1:0] h_imem_addr,
    output wire [15:0]       h_imem_wdata,
    input  wire [15:0]       h_imem_rdata,

    // ----------------------------------------------------- core run control
    output reg               h_run_we,
    output reg  [3:0]        h_run,
    output reg  [3:0]        h_reset,
    output reg               h_step_we,
    output reg  [3:0]        h_step,
    output reg               h_rpc_we,
    output reg  [1:0]        h_rpc_sel,
    output reg  [9:0]        h_rpc,
    output reg               h_sfset_we,
    output reg  [7:0]        h_sfset,
    output reg               h_sfclr_we,
    output reg  [7:0]        h_sfclr,
    output reg               h_badop_clr_we,
    output reg  [15:0]       h_badop_clr,
    output wire              h_badop_set15,
    output reg               h_swirq_clr_we,
    output reg  [3:0]        h_swirq_clr,

    // ---------------------------------------------------------- debug port
    output reg               h_dbg_req,
    output reg               h_dbg_wr,
    output reg  [1:0]        h_dbg_thread,
    output reg  [7:0]        h_dbg_reg,
    output reg  [15:0]       h_dbg_wdata,
    input  wire              h_dbg_ack,
    input  wire [15:0]       h_dbg_rdata,

    // ------------------------------------------------------------ pin regs
    output reg               h_pout_we,
    output reg  [15:0]       h_pout,
    output reg               h_poe_we,
    output reg  [7:0]        h_poe,
    output reg               h_od_we,
    output reg  [7:0]        h_od,

    // -------------------------------------------------------------- status
    input  wire [3:0]        run,
    input  wire [3:0]        halted,
    input  wire [15:0]       badop,
    input  wire [7:0]        sflags,
    input  wire [3:0]        swirq,
    input  wire [39:0]       resetpc_all,
    input  wire              core_busy,
    input  wire [15:0]       pin_out_reg,
    input  wire [7:0]        pin_oe_reg,
    input  wire [7:0]        od_mask_reg,
    input  wire [15:0]       pin_in_reg,

    output wire              irq
);

  // Spaces 2 (DMEM) and 3 (FIFO) are not built at M1: they fall through to
  // the default arms, which read 0 and ignore writes.
  localparam [2:0] SP_CTRL = 3'd0, SP_IMEM = 3'd1;
  localparam [2:0] SP_DBG  = 3'd4, SP_STEP = 3'd5;

  localparam [2:0] S_CMD = 3'd0, S_AH = 3'd1, S_AL = 3'd2, S_WHI = 3'd3;
  localparam [2:0] S_WLO = 3'd4, S_DUM = 3'd5, S_RHI = 3'd6, S_RLO = 3'd7;

  reg [2:0]  st;
  reg [7:0]  cmd;
  reg [15:0] addr;
  reg [7:0]  wr_hi;
  reg [7:0]  tx_byte_r;
  reg [7:0]  cur_lo;
  reg [15:0] fetch_word;
  reg        fetch_busy;
  reg        imem_phase;
  reg        fetch_go;
  reg        imem_wr_pulse;
  reg [15:0] wr_addr_q;
  reg [7:0]  wr_hi_q;
  reg [7:0]  wr_lo_q;
  reg        badop15_wr;
  reg        badop15_rd;

  wire       is_write = cmd[7];
  wire [2:0] space    = cmd[6:4];
  wire       imem_ok  = ~core_busy;

  assign tx_byte = tx_byte_r;

  // ------------------------------------------------------------- IRQ logic
  reg [15:0] irq_en;
  wire [15:0] irq_stat = {sflags, 4'b0000, 4'b0000};   // no FIFOs at M1
  assign irq = (|(irq_stat & irq_en)) | (|swirq);

  // ------------------------------------------------------- CTRL space reads
  reg [15:0] ctrl_rd;
  always @(*) begin
    case (addr[7:0])
      8'h00:   ctrl_rd = ID_VALUE;
      8'h01:   ctrl_rd = VERSION;
      8'h02:   ctrl_rd = {12'd0, run};
      8'h03:   ctrl_rd = {12'd0, halted};
      8'h08:   ctrl_rd = {6'd0, resetpc_all[9:0]};
      8'h09:   ctrl_rd = {6'd0, resetpc_all[19:10]};
      8'h0A:   ctrl_rd = {6'd0, resetpc_all[29:20]};
      8'h0B:   ctrl_rd = {6'd0, resetpc_all[39:30]};
      8'h10:   ctrl_rd = irq_en;
      8'h11:   ctrl_rd = irq_stat;
      8'h12:   ctrl_rd = {12'd0, halted};
      8'h13:   ctrl_rd = {8'd0, sflags};
      8'h15:   ctrl_rd = {8'd0, od_mask_reg};
      8'h16:   ctrl_rd = pin_out_reg;
      8'h17:   ctrl_rd = {8'd0, pin_oe_reg};
      8'h18:   ctrl_rd = pin_in_reg;
      8'h19:   ctrl_rd = CAPS;
      8'h1A:   ctrl_rd = badop;
      8'h1B:   ctrl_rd = {12'd0, swirq};
      default: ctrl_rd = 16'd0;
    endcase
  end

  // --------------------------------------------------------- read fetching
  // fetch_go starts a read of `addr`; fetch_word is valid when fetch_busy
  // falls, always well before the byte that carries it is shifted out.
  assign h_imem_req   = (fetch_busy && (space == SP_IMEM) && imem_ok && !imem_phase)
                        || imem_wr_pulse;
  assign h_imem_we    = imem_wr_pulse;
  assign h_imem_addr  = imem_wr_pulse ? wr_addr_q[IMEM_AW-1:0] : addr[IMEM_AW-1:0];
  assign h_imem_wdata = {wr_hi_q, wr_lo_q};

  assign h_badop_set15 = badop15_wr | badop15_rd;

  // CMD[3:0] is reserved and ignored; the latched write address only needs as
  // many bits as the instruction memory has.
  wire _unused = &{1'b0, cmd[3:0], wr_addr_q[15:IMEM_AW]};

  // ------------------------------------------------------------- main FSM
  wire [15:0] wr_word = {wr_hi, rx_byte};

  always @(posedge clk) begin
    if (!rst_n) begin
      st             <= S_CMD;
      cmd            <= 8'd0;
      addr           <= 16'd0;
      wr_hi          <= 8'd0;
      tx_byte_r      <= 8'd0;
      cur_lo         <= 8'd0;
      fetch_go       <= 1'b0;
      irq_en         <= 16'd0;
      h_run_we       <= 1'b0;
      h_run          <= 4'd0;
      h_reset        <= 4'd0;
      h_step_we      <= 1'b0;
      h_step         <= 4'd0;
      h_rpc_we       <= 1'b0;
      h_rpc_sel      <= 2'd0;
      h_rpc          <= 10'd0;
      h_sfset_we     <= 1'b0;
      h_sfset        <= 8'd0;
      h_sfclr_we     <= 1'b0;
      h_sfclr        <= 8'd0;
      h_badop_clr_we <= 1'b0;
      h_badop_clr    <= 16'd0;
      badop15_wr     <= 1'b0;
      h_swirq_clr_we <= 1'b0;
      h_swirq_clr    <= 4'd0;
      h_pout_we      <= 1'b0;
      h_pout         <= 16'd0;
      h_poe_we       <= 1'b0;
      h_poe          <= 8'd0;
      h_od_we        <= 1'b0;
      h_od           <= 8'd0;
      imem_wr_pulse  <= 1'b0;
      wr_addr_q      <= 16'd0;
      wr_hi_q        <= 8'd0;
      wr_lo_q        <= 8'd0;
    end else begin
      // Every control output is a one-clock pulse.
      h_run_we       <= 1'b0;
      h_reset        <= 4'd0;
      h_step_we      <= 1'b0;
      h_rpc_we       <= 1'b0;
      h_sfset_we     <= 1'b0;
      h_sfclr_we     <= 1'b0;
      h_badop_clr_we <= 1'b0;
      badop15_wr     <= 1'b0;
      h_swirq_clr_we <= 1'b0;
      h_pout_we      <= 1'b0;
      h_poe_we       <= 1'b0;
      h_od_we        <= 1'b0;
      imem_wr_pulse  <= 1'b0;
      fetch_go       <= 1'b0;

      if (!cs_active) begin
        st <= S_CMD;
      end else if (byte_done) begin
        case (st)
          S_CMD: begin
            cmd <= rx_byte;
            st  <= S_AH;
          end
          S_AH: begin
            addr[15:8] <= rx_byte;
            st         <= S_AL;
          end
          S_AL: begin
            addr[7:0] <= rx_byte;
            if (is_write) begin
              st <= S_WHI;
            end else begin
              tx_byte_r <= 8'd0;          // the dummy turnaround byte
              fetch_go  <= 1'b1;
              st        <= S_DUM;
            end
          end
          S_WHI: begin
            wr_hi <= rx_byte;
            st    <= S_WLO;
          end
          S_WLO: begin
            addr <= addr + 16'd1;
            st   <= S_WHI;
          end
          S_DUM: begin
            tx_byte_r <= fetch_word[15:8];
            cur_lo    <= fetch_word[7:0];
            addr      <= addr + 16'd1;
            fetch_go  <= 1'b1;
            st        <= S_RHI;
          end
          S_RHI: begin
            tx_byte_r <= cur_lo;
            st        <= S_RLO;
          end
          S_RLO: begin
            tx_byte_r <= fetch_word[15:8];
            cur_lo    <= fetch_word[7:0];
            addr      <= addr + 16'd1;
            fetch_go  <= 1'b1;
            st        <= S_RHI;
          end
          default: st <= S_CMD;
        endcase
      end

      // ------------------------------------------------------ write effects
      if (cs_active && byte_done && (st == S_WLO)) begin
        case (space)
          SP_CTRL: begin
            case (addr[7:0])
              8'h02: begin h_run_we <= 1'b1; h_run <= wr_word[3:0]; end
              8'h04: begin h_reset  <= wr_word[3:0];                end
              8'h08: begin h_rpc_we <= 1'b1; h_rpc_sel <= 2'd0; h_rpc <= wr_word[9:0]; end
              8'h09: begin h_rpc_we <= 1'b1; h_rpc_sel <= 2'd1; h_rpc <= wr_word[9:0]; end
              8'h0A: begin h_rpc_we <= 1'b1; h_rpc_sel <= 2'd2; h_rpc <= wr_word[9:0]; end
              8'h0B: begin h_rpc_we <= 1'b1; h_rpc_sel <= 2'd3; h_rpc <= wr_word[9:0]; end
              8'h10: irq_en <= wr_word;
              8'h13: begin h_sfset_we <= 1'b1; h_sfset <= wr_word[7:0]; end
              8'h14: begin h_sfclr_we <= 1'b1; h_sfclr <= wr_word[7:0]; end
              8'h15: begin h_od_we    <= 1'b1; h_od    <= wr_word[7:0]; end
              8'h16: begin h_pout_we  <= 1'b1; h_pout  <= wr_word;      end
              8'h17: begin h_poe_we   <= 1'b1; h_poe   <= wr_word[7:0]; end
              8'h1A: begin h_badop_clr_we <= 1'b1; h_badop_clr <= wr_word; end
              8'h1B: begin h_swirq_clr_we <= 1'b1; h_swirq_clr <= wr_word[3:0]; end
              default: ;
            endcase
          end
          SP_IMEM: begin
            if (imem_ok) begin
              imem_wr_pulse <= 1'b1;
              wr_addr_q     <= addr;
              wr_hi_q       <= wr_hi;
              wr_lo_q       <= rx_byte;
            end else begin
              badop15_wr <= 1'b1;
            end
          end
          SP_STEP: begin
            h_step_we <= 1'b1;
            h_step    <= (4'd1 << addr[1:0]);
          end
          default: ;   // DMEM, FIFO and unknown spaces ignore writes at M1
        endcase
      end
    end
  end

  // ------------------------------------------------------ read data engine
  always @(posedge clk) begin
    if (!rst_n) begin
      fetch_busy <= 1'b0;
      imem_phase <= 1'b0;
      fetch_word <= 16'd0;
    end else if (fetch_go) begin
      fetch_busy <= 1'b1;
      imem_phase <= 1'b0;
      fetch_word <= 16'd0;
    end else if (fetch_busy) begin
      case (space)
        SP_CTRL: begin fetch_word <= ctrl_rd; fetch_busy <= 1'b0; end
        SP_IMEM: begin
          if (!imem_ok) begin
            fetch_word <= 16'd0;
            fetch_busy <= 1'b0;
          end else if (!imem_phase) begin
            imem_phase <= 1'b1;
          end else begin
            fetch_word <= h_imem_rdata;
            fetch_busy <= 1'b0;
          end
        end
        SP_DBG: begin
          if (h_dbg_ack) begin
            fetch_word <= h_dbg_rdata;
            fetch_busy <= 1'b0;
          end
        end
        default: begin fetch_word <= 16'd0; fetch_busy <= 1'b0; end
      endcase
    end
  end

  // A refused IMEM read also sets BADOP[15]; done here so that the pulse is
  // generated once per refused word.
  always @(posedge clk) begin
    if (!rst_n) badop15_rd <= 1'b0;
    else        badop15_rd <= fetch_busy && (space == SP_IMEM) && !imem_ok && !badop15_rd;
  end

  // ----------------------------------------------------- debug port driver
  always @(posedge clk) begin
    if (!rst_n) begin
      h_dbg_req    <= 1'b0;
      h_dbg_wr     <= 1'b0;
      h_dbg_thread <= 2'd0;
      h_dbg_reg    <= 8'd0;
      h_dbg_wdata  <= 16'd0;
    end else if (h_dbg_req && h_dbg_ack) begin
      h_dbg_req <= 1'b0;
      h_dbg_wr  <= 1'b0;
    end else if (fetch_go && (space == SP_DBG)) begin
      h_dbg_req    <= 1'b1;
      h_dbg_wr     <= 1'b0;
      h_dbg_thread <= addr[9:8];
      h_dbg_reg    <= addr[7:0];
    end else if (cs_active && byte_done && (st == S_WLO) && (space == SP_DBG)) begin
      h_dbg_req    <= 1'b1;
      h_dbg_wr     <= 1'b1;
      h_dbg_thread <= addr[9:8];
      h_dbg_reg    <= addr[7:0];
      h_dbg_wdata  <= wr_word;
    end
  end

endmodule
