/*
 * loom_alu: 16-bit ALU, shifter and unary unit for Loom.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Timing contract: purely combinational. `y`, `c_out`, `z_out` follow the
 * inputs with no clock. It is used in the X stage of a slot, so its outputs
 * are registered into the W stage at the edge that ends X.
 *
 * Operand convention (docs/SEMANTICS.md 6.1), supplied by loom_core:
 *   sel_alu   : a = ra, b = rb, funct = ADD SUB AND OR XOR SHL SHR ROR
 *   sel_alui  : a = rd, b = zero-extended imm6, funct = ADDI .. SHRI CMPI
 *   sel_unary : a = ra (CMP/TEST: a = rd, b = ra), funct = MOV NOT NEG CMP
 *               TEST REV PAR SWAP
 * Shift amounts are b[3:0] in every class. The caller decides whether the
 * result is written back (CMP, CMPI and TEST write nothing) and whether the
 * flags are taken (MOV, LDI and LDIH set none).
 */

`default_nettype none

module loom_alu (
    input  wire        sel_alu,
    input  wire        sel_alui,
    input  wire        sel_unary,
    input  wire [2:0]  funct,
    input  wire [15:0] a,
    input  wire [15:0] b,
    output wire [15:0] y,
    output wire        c_out,
    output wire        z_out
);

  // funct 7 is ROR in the ALU class and CMPI in the ALUI class, and SWAP in
  // the unary class; both are handled in the `default` arm below.
  localparam [2:0] F_ADD  = 3'd0, F_SUB  = 3'd1, F_AND = 3'd2, F_OR   = 3'd3;
  localparam [2:0] F_XOR  = 3'd4, F_SHL  = 3'd5, F_SHR = 3'd6;
  localparam [2:0] F_MOV  = 3'd0, F_NOT  = 3'd1, F_NEG = 3'd2, F_CMP  = 3'd3;
  localparam [2:0] F_TEST = 3'd4, F_REV  = 3'd5, F_PAR = 3'd6;

  // ------------------------------------------------------------- arithmetic
  wire [16:0] sum  = {1'b0, a} + {1'b0, b};
  wire [15:0] diff = a - b;
  wire        ge   = (a >= b);          // SUB/CMP carry means "no borrow"

  // ---------------------------------------------------------------- shifts
  wire [3:0]  amt = b[3:0];
  wire        amt_nz = |amt;
  wire [15:0] shl = a << amt;
  wire [15:0] shr = a >> amt;
  wire [31:0] rot = {a, a} >> amt;
  wire [15:0] ror = rot[15:0];

  // Carry out of a shift is the last bit shifted out (0 when the amount is 0).
  // SHL by n: bit 16-n of a, i.e. a[16-n]; SHR by n: a[n-1]; ROR: bit 15 of
  // the result.
  wire [15:0] shl_c_sel = a >> (5'd16 - {1'b0, amt});
  wire        shl_c     = amt_nz & shl_c_sel[0];
  wire [15:0] shr_c_sel = a >> (amt - 4'd1);
  wire        shr_c     = amt_nz & shr_c_sel[0];
  wire        ror_c     = amt_nz & ror[15];

  // ----------------------------------------------------------------- unary
  wire [15:0] rev = {a[0],  a[1],  a[2],  a[3],  a[4],  a[5],  a[6],  a[7],
                     a[8],  a[9],  a[10], a[11], a[12], a[13], a[14], a[15]};
  wire [15:0] swp = {a[7:0], a[15:8]};
  wire        par = ^a;
  wire [15:0] andres = a & b;

  // ------------------------------------------------------------- selection
  reg [15:0] res;
  reg        cy;

  always @(*) begin
    res = 16'd0;
    cy  = 1'b0;
    if (sel_alu | sel_alui) begin
      case (funct)
        F_ADD: begin res = sum[15:0];  cy = sum[16]; end
        F_SUB: begin res = diff;       cy = ge;      end     // also CMPI (f=7)
        F_AND: begin res = a & b;      cy = 1'b0;    end
        F_OR:  begin res = a | b;      cy = 1'b0;    end
        F_XOR: begin res = a ^ b;      cy = 1'b0;    end
        F_SHL: begin res = shl;        cy = shl_c;   end
        F_SHR: begin res = shr;        cy = shr_c;   end
        default: begin
          // ALU f=7 is ROR, ALUI f=7 is CMPI (a compare, flags of a - b).
          if (sel_alui) begin res = diff; cy = ge;     end
          else          begin res = ror;  cy = ror_c;  end
        end
      endcase
    end else if (sel_unary) begin
      case (funct)
        F_MOV:  begin res = a;             cy = 1'b0;      end
        F_NOT:  begin res = ~a;            cy = 1'b0;      end
        F_NEG:  begin res = 16'd0 - a;     cy = (a == 16'd0); end
        F_CMP:  begin res = diff;          cy = ge;        end
        F_TEST: begin res = andres;        cy = 1'b0;      end
        F_REV:  begin res = rev;           cy = 1'b0;      end
        F_PAR:  begin res = a;             cy = par;       end
        default:begin res = swp;           cy = 1'b0;      end
      endcase
    end
  end

  assign y     = res;
  assign c_out = cy;
  assign z_out = (res == 16'd0);

  // Only one bit of each shift-carry selector and the low half of the rotate
  // are consumed; the rest exist only to keep the expressions readable.
  wire _unused = &{1'b0, rot[31:16], shl_c_sel[15:1], shr_c_sel[15:1]};

endmodule
