`default_nettype none
`timescale 1ns / 1ps

/* This testbench just instantiates the module and makes some convenient wires
   that can be driven / tested by the cocotb test.py.
*/
module tb ();

  // Dump the signals to a FST file. You can view it with gtkwave or surfer.
  // Dumping the whole design (including the instruction memories) roughly
  // triples the run time of the suite, so it is opt-in:
  //     make PLUSARGS=+dump
  initial begin
    if ($test$plusargs("dump")) begin
      $dumpfile("tb.fst");
      $dumpvars(0, tb);
    end
    #1;
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;

  // Tiny Tapeout pad model for the bidirectional pins: a bit the design
  // drives reads back its own value, a bit it releases reads what the
  // testbench drives (uio_drv, which also carries external pull-ups).
  // docs/SEMANTICS.md section 3 requires this loopback.
  reg  [7:0] uio_drv;
  wire [7:0] uio_in = (uio_out & uio_oe) | (uio_drv & ~uio_oe);

  // Replace tt_um_example with your module name:
  tt_um_loom user_project (
      .ui_in  (ui_in),    // Dedicated inputs
      .uo_out (uo_out),   // Dedicated outputs
      .uio_in (uio_in),   // IOs: Input path
      .uio_out(uio_out),  // IOs: Output path
      .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
      .ena    (ena),      // enable - goes high when design is selected
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // not reset
  );

`ifndef GL_TEST
  // The FLOPS fallback build (D-020): a second, independent tt_um_loom with
  // the 256 x 16 flip-flop instruction memory instead of the SRAM macro, on
  // its own pins (same names with a _flops suffix, same pad model). Only
  // test_flops.py drives it; nothing else toggles clk_flops, so the other
  // tests pay only its compile time. The gate-level netlist is the MACRO
  // build, so this instance exists in RTL simulation only.
  reg        clk_flops;
  reg        rst_n_flops;
  reg        ena_flops;
  reg  [7:0] ui_in_flops;
  reg  [7:0] uio_drv_flops;
  wire [7:0] uo_out_flops;
  wire [7:0] uio_out_flops;
  wire [7:0] uio_oe_flops;
  wire [7:0] uio_in_flops = (uio_out_flops & uio_oe_flops)
                            | (uio_drv_flops & ~uio_oe_flops);

  tt_um_loom #(.IMEM_IMPL("FLOPS"), .IMEM_WORDS(256)) user_project_flops (
      .ui_in  (ui_in_flops),
      .uo_out (uo_out_flops),
      .uio_in (uio_in_flops),
      .uio_out(uio_out_flops),
      .uio_oe (uio_oe_flops),
      .ena    (ena_flops),
      .clk    (clk_flops),
      .rst_n  (rst_n_flops)
  );
`endif

endmodule
