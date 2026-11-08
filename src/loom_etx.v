/*
 * loom_etx: the Ethernet serializer (docs/SEMANTICS.md 6.12, D-036).
 * SPDX-License-Identifier: Apache-2.0
 *
 * One shared transmit block. It takes 16-bit words from CSR writes
 * (ETX_DATA, ETX_LAST, ETX_LAST1), sends them as a Manchester-coded frame
 * with the preamble and SFD before and the CRC-32 FCS after, ends the frame
 * with TP_IDL, and gives loom_pins the pad pair for uo_out[1:0] while it is
 * busy. Global state, reset by rst_n only (not by CTRL.RESET, not by the
 * host).
 *
 * Timing contract:
 *   w_*_we and w_val come from loom_core's W stage: a CSRW with X cycle x is
 *   presented during cycle x+1 and commits here at edge x+2 (6.12.2). The
 *   strobes are already qualified by w_valid and dec_ok; at most one is high.
 *   Every output is a function of this module's registers only (no path
 *   from the write port), so a CSRR, JP, WAITP or IN in X cycle x sees the
 *   value in cycle x, and the pad pair changes only at an edge:
 *     cfg_rd  = {IDL, FCS, PRE, HALFN}          (ETX_CFG, 0x16)
 *     stat_rd = {REF, UNDER, FDONE, RDY, BUSY}  (ETX_STAT, 0x1A)
 *     busy, rdy                                 (pin_in(14), pin_in(13))
 *     tdp, tdn                                  TD+, TD- while busy (6.12.7)
 *   loom_pins muxes tdp/tdn onto uo_out[1:0] with busy as the select and
 *   does not register them again (MF-3).
 *
 * State encoding (6.12.1 allows any encoding that reproduces ETX_CFG,
 * ETX_STAT, pin_in(13/14) and the pads):
 *   EPH is {busy_r, eph[1:0]}: busy_r == 0 is P_IDLE (eph is then 0), and
 *     eph 0..3 is P_PRE, P_DATA, P_FCS, P_TAIL. BUSY is the flop itself.
 *   TXL is kept as the pad pair {pdp, pdn} = {TXL, ~TXL} in P_PRE, P_DATA
 *     and P_FCS; entering P_TAIL loads (1, 0), so in every busy cycle the
 *     pads are exactly {pdp, pdn} and the 6.12.7 mux is a 2:1 mux of flops.
 *   `sh` holds the bits of SH not yet begun: a load takes SH[15:1] (bit 0
 *     begins at once, into TXB), and every later data bit of the word
 *     takes sh[0] and shifts `sh` right, so 6.12.5 rule 4's SH[POS + 1] is
 *     always sh[0] (no 16:1 mux, and no flop for SH[0], which is TXB).
 *   CRC32 shifts left one place at every FCS bit after the first, so
 *     6.12.5 rule 6's ~CRC32[30 - POS] is always ~crc[30] (no 32:1 mux).
 *     During the data bits the register is exactly 6.12.3's CRC32.
 *   POS, DIV, HB, TXB, SL, S1, HOLD, HV, HL, H1, LSEEN, FDONE, UNDER, REF
 *     are as 6.12.1 names them.
 */

`default_nettype none

module loom_etx (
    input  wire        clk,
    input  wire        rst_n,

    // W-stage CSR writes (from loom_core), qualified, at most one high.
    input  wire        w_cfg_we,     // ETX_CFG   0x16
    input  wire        w_data_we,    // ETX_DATA  0x17
    input  wire        w_last_we,    // ETX_LAST  0x18
    input  wire        w_last1_we,   // ETX_LAST1 0x19
    input  wire        w_stat_we,    // ETX_STAT  0x1A
    input  wire [15:0] w_val,

    // Reads and status, functions of the registers only.
    output wire [9:0]  cfg_rd,
    output wire [4:0]  stat_rd,
    output wire        busy,
    output wire        rdy,
    output wire        tdp,
    output wire        tdn
);

  localparam [1:0] E_PRE = 2'd0, E_DATA = 2'd1, E_FCS = 2'd2, E_TAIL = 2'd3;
  localparam [31:0] CRC_POLY = 32'h04C1_1DB7;

  // ------------------------------------------------------------ registers
  reg [3:0]  halfn, idl;               // configuration
  reg        pre_en, fcs_en;
  reg        busy_r;                   // EPH != P_IDLE
  reg [1:0]  eph;                      // phase within busy
  reg [3:0]  div;
  reg        hb;
  reg [5:0]  pos;
  reg        txb;
  reg        pdp, pdn;                 // pad pair while busy
  reg [14:0] sh;                       // SH[15:1]; SH[0] is TXB
  reg        sl, s1;
  reg [15:0] hold;
  reg        hv, hl, h1;
  reg        lseen;
  reg [31:0] crc;
  reg        fdone, under, refd;

  function [31:0] crc_step;            // 6.12.3, one data bit v
    input [31:0] c;
    input        v;
    begin
      crc_step = {c[30:0], 1'b0} ^ ((c[31] ^ v) ? CRC_POLY : 32'd0);
    end
  endfunction

  // ------------------------------------------------- the write at this edge
  wire w_dat  = w_data_we | w_last_we | w_last1_we;   // a data write
  wire w_lst  = w_last_we | w_last1_we;               // last = kind != ETX_DATA
  wire w_one  = w_last1_we;                           // one  = kind == ETX_LAST1

  // --------------------------------------------- 6.12.5 decisions (pre-edge)
  wire in_pre  = busy_r & (eph == E_PRE);
  wire in_data = busy_r & (eph == E_DATA);
  wire in_fcs  = busy_r & (eph == E_FCS);
  wire in_tail = busy_r & (eph == E_TAIL);
  wire bnd     = busy_r & (div == 4'd0);              // e ends a half bit
  wire hb1     = bnd & hb;                            // ... the second half
  wire lastbit = s1 ? (pos == 6'd7) : (pos == 6'd15); // the word's last bit

  wire second    = bnd & ~in_tail & ~hb;                          // rule 1
  wire pre_next  = hb1 & in_pre & (pos != 6'd63);                 // rule 2
  wire load_edge = hb1 & ((in_pre & (pos == 6'd63))
                          | (in_data & lastbit & ~sl));           // rule 3
  wire load_ok   = load_edge & hv;
  wire underrun  = load_edge & ~hv;
  wire data_next = hb1 & in_data & ~lastbit;                      // rule 4
  wire data_end  = hb1 & in_data & lastbit & sl;                  // rule 5
  wire fcs_start = data_end & fcs_en;
  wire fcs_next  = hb1 & in_fcs & (pos != 6'd31);                 // rule 6
  wire fcs_end   = hb1 & in_fcs & (pos == 6'd31);
  wire tail_step = bnd & in_tail;                                 // rule 7
  wire tail_rel  = tail_step & ({2'b00, idl} == pos + 6'd1);
  wire ent_tail  = underrun | (data_end & ~fcs_en) | fcs_end;     // enter the tail
  wire rel_go   = tail_rel | (ent_tail & (idl == 4'd0));
  wire tail_go   = ent_tail & (idl != 4'd0);

  // Start (6.12.4): an idle block takes a data write unless UNDER is set.
  wire start = ~busy_r & w_dat & ~under;

  // Accept (6.12.4): EPH is P_PRE or P_DATA before e and also after e (the
  // only ways out of those two at e are entering the tail and the FCS),
  // LSEEN and UNDER clear, and HOLD empty or taken by a load at e.
  wire stays_pd = (in_pre | in_data) & ~ent_tail & ~fcs_start;
  wire accept   = busy_r & w_dat & stays_pd & ~lseen & ~under & (~hv | load_ok);

  // The bit that begins at this edge, if any (6.12.4 "begin bit").
  wire begin_bit = pre_next | load_ok | data_next | fcs_start | fcs_next;
  reg  bit_v;
  always @(*) begin
    if (pre_next)       bit_v = pos[0] | (pos == 6'd62);   // p_{POS+1}
    else if (load_ok)   bit_v = hold[0];                   // HOLD[0]
    else if (data_next) bit_v = sh[0];                     // SH[POS + 1]
    else if (fcs_start) bit_v = ~crc[31];                  // f_0
    else                bit_v = ~crc[30];                  // f_{POS+1}
  end
  wire start_v = pre_en ? 1'b1 : w_val[0];                 // p_0 or w[0]

  // ------------------------------------------------------- configuration
  always @(posedge clk) begin
    if (!rst_n) begin
      halfn <= 4'd1; pre_en <= 1'b1; fcs_en <= 1'b1; idl <= 4'd6;
    end else if (w_cfg_we & ~busy_r) begin
      halfn  <= w_val[3:0];
      pre_en <= w_val[4];
      fcs_en <= w_val[5];
      idl    <= w_val[9:6];
    end
  end

  // ------------------------------------------- phase, half-bit timing, pads
  // EPH, DIV, HB, POS, TXB and the pad pair. Exactly one branch of 6.12.5
  // applies at a boundary: rule 1 needs HB == 0 outside the tail, the rest
  // need HB == 1 or the tail.
  always @(posedge clk) begin
    if (!rst_n) begin
      busy_r <= 1'b0; eph <= E_PRE; div <= 4'd0; hb <= 1'b0; pos <= 6'd0;
      txb <= 1'b0; pdp <= 1'b0; pdn <= 1'b0;
    end else if (start) begin
      busy_r <= 1'b1;
      eph    <= pre_en ? E_PRE : E_DATA;
      div    <= halfn;
      pos    <= 6'd0;
      hb     <= 1'b0;
      txb    <= start_v;
      pdp    <= ~start_v;
      pdn    <= start_v;
    end else if (busy_r) begin
      if (!bnd) begin
        div <= div - 4'd1;
      end else begin
        div <= halfn;
        if (second) begin                       // rule 1: TXL <= TXB
          hb  <= 1'b1;
          pdp <= txb;
          pdn <= ~txb;
        end else if (begin_bit) begin           // rules 2, 3, 4, 5, 6
          hb  <= 1'b0;
          txb <= bit_v;
          pdp <= ~bit_v;
          pdn <= bit_v;
          pos <= (load_ok | fcs_start) ? 6'd0 : pos + 6'd1;
          if (load_ok)        eph <= E_DATA;
          else if (fcs_start) eph <= E_FCS;
        end else if (rel_go) begin             // release: EPH <= P_IDLE
          // HB, POS, TXB and the pad pair keep their values: not visible
          // while idle (W-118 QUESTIONS.md Q3).
          busy_r <= 1'b0;
          eph    <= E_PRE;
        end else if (tail_go) begin             // enter the tail
          eph <= E_TAIL;
          pos <= 6'd0;
          pdp <= 1'b1;
          pdn <= 1'b0;
        end else if (tail_step) begin           // rule 7, not the release
          pos <= pos + 6'd1;
        end
      end
    end
  end

  // ------------------------------------------------- SH, HOLD and LSEEN
  always @(posedge clk) begin
    if (!rst_n) begin
      sh <= 15'd0; sl <= 1'b0; s1 <= 1'b0;
      hold <= 16'd0; hv <= 1'b0; hl <= 1'b0; h1 <= 1'b0; lseen <= 1'b0;
    end else if (start) begin
      lseen <= w_lst;
      if (pre_en) begin                         // word 0 waits in HOLD
        hold <= w_val; hv <= 1'b1; hl <= w_lst; h1 <= w_one;
      end else begin                            // word 0 goes straight to SH
        sh <= w_val[15:1]; sl <= w_lst; s1 <= w_one; hv <= 1'b0;
      end
    end else if (busy_r) begin
      if (load_ok) begin                        // rule 3, HV == 1
        sh <= hold[15:1]; sl <= hl; s1 <= h1; hv <= 1'b0;
      end else if (data_next) begin             // rule 4: next bit to SH[0]
        sh <= {1'b0, sh[14:1]};
      end
      if (ent_tail | rel_go) hv <= 1'b0;
      if (rel_go)            lseen <= 1'b0;
      if (accept) begin                         // last assignment wins: HV
        hold <= w_val; hv <= 1'b1; hl <= w_lst; h1 <= w_one;   // stays 1
        lseen <= w_lst;
      end
    end
  end

  // ---------------------------------------------------------------- CRC-32
  always @(posedge clk) begin
    if (!rst_n) begin
      crc <= 32'd0;
    end else if (start) begin
      crc <= pre_en ? 32'hFFFF_FFFF : crc_step(32'hFFFF_FFFF, w_val[0]);
    end else if (load_ok) begin
      crc <= crc_step(crc, hold[0]);
    end else if (data_next) begin
      crc <= crc_step(crc, sh[0]);
    end else if (fcs_next) begin
      crc <= {crc[30:0], 1'b0};                 // encoding only, see header
    end
  end

  // ------------------------------------------------------- sticky status
  wire set_ref   = (w_cfg_we & busy_r)
                   | (w_dat & ~busy_r & under)
                   | (w_dat & busy_r & ~accept);
  wire [2:0] clr = w_stat_we ? w_val[4:2] : 3'd0;   // {REF, UNDER, FDONE}

  always @(posedge clk) begin
    if (!rst_n) begin
      fdone <= 1'b0; under <= 1'b0; refd <= 1'b0;
    end else begin
      refd  <= set_ref  | (refd  & ~clr[2]);    // set wins over clear
      under <= underrun | (under & ~clr[1]);
      fdone <= rel_go  | (fdone & ~clr[0]);
    end
  end

  // ---------------------------------------------------------------- views
  assign busy    = busy_r;
  assign rdy     = ~under & (~busy_r | ((in_pre | in_data) & ~lseen & ~hv));
  assign cfg_rd  = {idl, fcs_en, pre_en, halfn};
  assign stat_rd = {refd, under, fdone, rdy, busy_r};
  assign tdp     = pdp;
  assign tdn     = pdn;

endmodule
