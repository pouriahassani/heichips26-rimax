// SPDX-FileCopyrightText: 2026 XXX
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
// Description: SystemVerilog testbench for the heichips26_RiMaX module.
//
// RiMaX serialises the picorv32 memory interface onto the chip pins, so the
// testbench has to play the part of the eFPGA on the other side of that link:
// it deserialises the requests, serves them out of a memory array, and models
// an AXI UART Lite at 0x40600000.
//
// The program is embedded in this file, so the testbench has no external
// dependencies. It exercises instruction fetch, a word store and load, the
// byte and half word strobes, and the FlotiMaX custom instruction, printing
// one character per passing test.
//
//   expected output:  "RiMaX\r\nWSF\r\n"
//        R i M a X    the core is fetching and executing
//        W            word store and load round trip
//        S            sb and sh carried the right MEM_WSTRB across the link
//        F            the FlotiMaX custom instruction returned 1.0 * 2.0

`timescale 1ns / 1ps

module heichips26_RiMaX_tb;

  parameter real CLK_FREQ      = 50.0e6;
  localparam real CLK_PERIOD_NS = 1e9 / CLK_FREQ;

  localparam int  MEM_WORDS = 1024;                 // 4 KiB is enough here
  localparam logic [31:0] UART_BASE = 32'h4060_0000;
  // "RiMaX" CR LF "WSF" CR LF -- \r is not a Verilog escape, so use octal
  localparam int EXP_LEN = 12;

  // ------------------------------------------------------------------
  //  chip pins
  // ------------------------------------------------------------------
  logic clk   = 1'b0;
  logic rst_n = 1'b0;

  wire [15:0] uo_out;          // chip -> fpga payload
  logic [15:0] ui_in;          // fpga -> chip payload
  wire [15:0] uio_out, uio_oe; // chip drive
  logic [15:0] uio_fpga;       // fpga drive
  wire [15:0] uio_bus;

  // uio_oe decides who owns each bit, the two sides never drive the same one
  assign uio_bus = (uio_out & uio_oe) | (uio_fpga & ~uio_oe);

  heichips26_RiMaX dut_heichips26_RiMaX (
    .ui_in   (ui_in),
    .uo_out  (uo_out),
    .uio_in  (uio_bus),
    .uio_out (uio_out),
    .uio_oe  (uio_oe),
    .ena     (1'b1),
    .clk     (clk),
    .rst_n   (rst_n)
  );

  /* verilator lint_off STMTDLY */
  always #(CLK_PERIOD_NS / 2) clk = ~clk;
  /* verilator lint_on STMTDLY */

  // ------------------------------------------------------------------
  //  link decode, matching pico_adapter_to_fpga
  // ------------------------------------------------------------------
  wire [1:0] lnk_type  = uio_bus[5:4];
  wire [3:0] lnk_wstrb = uio_bus[11:8];
  wire       lnk_stb   = uio_bus[12];
  wire [1:0] lnk_beat  = uio_bus[14:13];
  wire       lnk_last  = uio_bus[15];

  localparam logic [1:0] REG_RD = 2'b00, REG_W = 2'b01, MEM_RD = 2'b10, MEM_W = 2'b11;
  localparam logic [1:0] ADDR_L = 2'b00, ADDR_H = 2'b01, DATA_L = 2'b10, DATA_H = 2'b11;

  localparam logic [2:0] F_COLLECT = 3'd0, F_SERVE = 3'd1,
                         F_RSP_LO  = 3'd2, F_RSP_HI = 3'd3, F_WACK = 3'd4;

  logic [2:0]  fstate;
  logic [31:0] req_addr, req_wdata, rsp_data;
  logic [3:0]  req_wstrb;
  logic        req_write;

  logic [31:0] mem [0:MEM_WORDS-1];

  // captured UART bytes
  localparam int UART_MAX = 256;
  logic [7:0] uart_log [0:UART_MAX-1];
  logic [7:0] expected [0:EXP_LEN-1];
  int         uart_n;

  wire take = lnk_stb && (fstate == F_COLLECT);

  // FPGA_READY on uio_in[0], RD_STB on [1], WR_DONE on [2]
  // FPGA_READY, RD_STB, WR_DONE on uio_in[2:0]
  always_comb uio_fpga = {13'b0,
                          (fstate == F_WACK),
                          (fstate == F_RSP_LO) || (fstate == F_RSP_HI),
                          (fstate == F_COLLECT)};

  wire [15:0] rsp_lo = rsp_data[15:0];
  wire [15:0] rsp_hi = rsp_data[31:16];
  always_comb ui_in = (fstate == F_RSP_LO) ? rsp_lo :
                      (fstate == F_RSP_HI) ? rsp_hi : 16'h0000;

  wire in_mem  = (req_addr >> 2) < MEM_WORDS;
  wire is_uart = (req_addr & 32'hFFFF_0000) == UART_BASE;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      fstate    <= F_COLLECT;
      req_addr  <= '0;
      req_wdata <= '0;
      req_wstrb <= '0;
      req_write <= 1'b0;
      rsp_data  <= '0;
    end else begin
      case (fstate)
        F_COLLECT: begin
          if (take) begin
            case (lnk_beat)
              ADDR_L: begin
                req_addr[15:0] <= uo_out;
                // a register access never sends the upper half
                if (lnk_type == REG_RD || lnk_type == REG_W)
                  req_addr[31:16] <= 16'h0000;   // must equal REG_ADDR_HI
              end
              ADDR_H: req_addr [31:16] <= uo_out;
              DATA_L: req_wdata[15:0]  <= uo_out;
              DATA_H: req_wdata[31:16] <= uo_out;
              default: ;
            endcase
            req_wstrb <= lnk_wstrb;
            req_write <= (lnk_type == REG_W) || (lnk_type == MEM_W);
            if (lnk_last) fstate <= F_SERVE;
          end
        end

        F_SERVE: begin
          if (req_write) begin
            if (is_uart) begin
              // AXI UART Lite: TX FIFO at +4, everything else ignored here
              if (req_addr[15:0] == 16'h0004) begin
                if (uart_n < UART_MAX) uart_log[uart_n] = req_wdata[7:0];
                uart_n = uart_n + 1;
                $write("%c", req_wdata[7:0]);
              end
            end else if (in_mem) begin
              if (req_wstrb[0]) mem[req_addr>>2][ 7: 0] <= req_wdata[ 7: 0];
              if (req_wstrb[1]) mem[req_addr>>2][15: 8] <= req_wdata[15: 8];
              if (req_wstrb[2]) mem[req_addr>>2][23:16] <= req_wdata[23:16];
              if (req_wstrb[3]) mem[req_addr>>2][31:24] <= req_wdata[31:24];
            end
            fstate <= F_WACK;
          end else begin
            if (is_uart)
              // status register at +8: TX empty, never full, no RX data
              rsp_data <= (req_addr[15:0] == 16'h0008) ? 32'h0000_0004 : 32'h0000_0000;
            else
              rsp_data <= in_mem ? mem[req_addr>>2] : 32'hDEAD_BEEF;
            fstate <= F_RSP_LO;
          end
        end

        F_RSP_LO: fstate <= F_RSP_HI;
        F_RSP_HI: fstate <= F_COLLECT;
        F_WACK  : fstate <= F_COLLECT;
        default : fstate <= F_COLLECT;
      endcase
    end
  end

  // ------------------------------------------------------------------
  //  protocol checks on the pins
  // ------------------------------------------------------------------
  int errors = 0;

  always @(posedge clk) begin
    if (rst_n) begin
      // every uio bit must be driven by exactly one side
      if ($isunknown(uio_oe)) begin
        $display("FAIL: uio_oe is unknown"); errors = errors + 1;
      end
      // the payload must be resolved whenever a beat is on the bus
      if (lnk_stb && $isunknown(uo_out)) begin
        $display("FAIL: uo_out unknown while stb is high"); errors = errors + 1;
      end
      // the FPGA must never be asked to take a beat it is not collecting
      if (lnk_stb && lnk_last && $isunknown(lnk_type)) begin
        $display("FAIL: request type unknown on the last beat"); errors = errors + 1;
      end
    end
  end

  // ------------------------------------------------------------------
  //  stimulus
  // ------------------------------------------------------------------
  int i;
  int mismatch;

  initial begin
    $dumpfile("heichips26_RiMaX_tb.fst");
    $dumpvars;

    for (i = 0; i < MEM_WORDS; i++) mem[i] = 32'h0000_0000;
    mem[  0] = 32'h0180006f;
    mem[  1] = 32'h0082a303;
    mem[  2] = 32'h00837313;
    mem[  3] = 32'hfe031ce3;
    mem[  4] = 32'h00a2a223;
    mem[  5] = 32'h00008067;
    mem[  6] = 32'h406002b7;
    mem[  7] = 32'h05200513;
    mem[  8] = 32'hfe5ff0ef;
    mem[  9] = 32'h06900513;
    mem[ 10] = 32'hfddff0ef;
    mem[ 11] = 32'h04d00513;
    mem[ 12] = 32'hfd5ff0ef;
    mem[ 13] = 32'h06100513;
    mem[ 14] = 32'hfcdff0ef;
    mem[ 15] = 32'h05800513;
    mem[ 16] = 32'hfc5ff0ef;
    mem[ 17] = 32'h00d00513;
    mem[ 18] = 32'hfbdff0ef;
    mem[ 19] = 32'h00a00513;
    mem[ 20] = 32'hfb5ff0ef;
    mem[ 21] = 32'h10000413;
    mem[ 22] = 32'hdeadc3b7;
    mem[ 23] = 32'heef38393;
    mem[ 24] = 32'h00742023;
    mem[ 25] = 32'h00042483;
    mem[ 26] = 32'h00749663;
    mem[ 27] = 32'h05700513;
    mem[ 28] = 32'hf95ff0ef;
    mem[ 29] = 32'h0aa00593;
    mem[ 30] = 32'h00b40023;
    mem[ 31] = 32'h5cc00593;
    mem[ 32] = 32'h00b41123;
    mem[ 33] = 32'h00042483;
    mem[ 34] = 32'h05ccc637;
    mem[ 35] = 32'heaa60613;
    mem[ 36] = 32'h00c49663;
    mem[ 37] = 32'h05300513;
    mem[ 38] = 32'hf6dff0ef;
    mem[ 39] = 32'h3f800637;
    mem[ 40] = 32'h400006b7;
    mem[ 41] = 32'h06d61733;
    mem[ 42] = 32'h400007b7;
    mem[ 43] = 32'h00f71663;
    mem[ 44] = 32'h04600513;
    mem[ 45] = 32'hf51ff0ef;
    mem[ 46] = 32'h00d00513;
    mem[ 47] = 32'hf49ff0ef;
    mem[ 48] = 32'h00a00513;
    mem[ 49] = 32'hf41ff0ef;
    mem[ 50] = 32'h0000006f;


    uart_n = 0;
    expected[ 0] = "R"; expected[ 1] = "i"; expected[ 2] = "M"; expected[ 3] = "a";
    expected[ 4] = "X"; expected[ 5] = 8'h0D; expected[ 6] = 8'h0A;
    expected[ 7] = "W"; expected[ 8] = "S";  expected[ 9] = "F";
    expected[10] = 8'h0D; expected[11] = 8'h0A;

    rst_n = 1'b0;
    #(4 * CLK_PERIOD_NS);
    rst_n = 1'b1;

    // let the program run; it halts in a loop after printing
    fork
      begin : watchdog
        #(400000 * CLK_PERIOD_NS);
        $display("");
        $display("FAIL: timeout, only %0d UART bytes seen", uart_n);
        errors = errors + 1;
      end
      begin : wait_done
        wait (uart_n >= EXP_LEN);
        #(20 * CLK_PERIOD_NS);
      end
    join_any
    disable fork;

    $display("");
    $display("----------------------------------------");
    $display(" UART bytes received : %0d (expected %0d)", uart_n, EXP_LEN);

    mismatch = 0;
    if (uart_n != EXP_LEN) mismatch = 1;
    for (i = 0; i < EXP_LEN; i = i + 1)
      if (i < uart_n && uart_log[i] !== expected[i]) begin
        $display(" FAIL: byte %0d is 0x%02x, expected 0x%02x", i, uart_log[i], expected[i]);
        mismatch = 1;
      end
    if (mismatch != 0) errors = errors + 1;

    if (errors == 0)
      $display(" PASS: simulation complete.");
    else
      $fatal(1, "FAIL: %0d error(s)", errors);
    $display("----------------------------------------");
    $finish;
  end

endmodule // heichips26_RiMaX_tb
