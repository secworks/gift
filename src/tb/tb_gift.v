//======================================================================
//
// tb_gift.v
// -----------
// Testbench for the gift top level wrapper
//
//
// Author: Joachim Strombergson
// Copyright (c) 2019, Assured AB
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

`default_nettype none

module tb_gift();

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter DEBUG     = 0;
  parameter DUMP_WAIT = 0;

  parameter CLK_HALF_PERIOD = 1;
  parameter CLK_PERIOD = 2 * CLK_HALF_PERIOD;

  localparam ADDR_NAME0        = 8'h00;
  localparam ADDR_NAME1        = 8'h01;
  localparam ADDR_VERSION      = 8'h02;

  localparam ADDR_CTRL         = 8'h08;
  localparam CTRL_INIT_BIT     = 0;
  localparam CTRL_NEXT_BIT     = 1;

  localparam ADDR_STATUS       = 8'h09;
  localparam STATUS_READY_BIT  = 0;

  localparam ADDR_KEY0         = 8'h10;
  localparam ADDR_KEY1         = 8'h11;
  localparam ADDR_KEY2         = 8'h12;
  localparam ADDR_KEY3         = 8'h13;

  localparam ADDR_BLOCK0       = 8'h20;
  localparam ADDR_BLOCK1       = 8'h21;
  localparam ADDR_BLOCK2       = 8'h22;
  localparam ADDR_BLOCK3       = 8'h23;

  localparam ADDR_RESULT0      = 8'h30;
  localparam ADDR_RESULT1      = 8'h31;
  localparam ADDR_RESULT2      = 8'h32;
  localparam ADDR_RESULT3      = 8'h33;


  //----------------------------------------------------------------
  // Register and Wire declarations.
  //----------------------------------------------------------------
  reg [31 : 0] cycle_ctr;
  reg [31 : 0] error_ctr;
  reg [31 : 0] tc_ctr;
  reg          tb_monitor;

  reg           tb_clk;
  reg           tb_reset_n;
  reg           tb_cs;
  reg           tb_we;
  reg [7 : 0]   tb_address;
  reg [31 : 0]  tb_write_data;
  wire [31 : 0] tb_read_data;

  reg [31 : 0] read_data;


  //----------------------------------------------------------------
  // Device Under Test.
  //----------------------------------------------------------------
  gift dut(
           .clk(tb_clk),
           .reset_n(tb_reset_n),

           .cs(tb_cs),
           .we(tb_we),

           .address(tb_address),
           .write_data(tb_write_data),
           .read_data(tb_read_data)
           );


  //----------------------------------------------------------------
  // clk_gen
  //
  // Always running clock generator process.
  //----------------------------------------------------------------
  always
    begin : clk_gen
      #CLK_HALF_PERIOD;
      tb_clk = !tb_clk;
    end // clk_gen


  //----------------------------------------------------------------
  // sys_monitor()
  //
  // An always running process that creates a cycle counter and
  // conditionally displays information about the DUT.
  //----------------------------------------------------------------
  always
    begin : sys_monitor
      cycle_ctr = cycle_ctr + 1;
      #(CLK_PERIOD);
      if (tb_monitor)
        begin
          dump_dut_state();
        end
    end


  //----------------------------------------------------------------
  // dump_dut_state()
  //
  // Dump the state of the dump when needed.
  //----------------------------------------------------------------
  task dump_dut_state;
    begin
      $display("State of DUT");
      $display("------------");
      $display("Cycle: %08d", cycle_ctr);
      $display("Inputs and outputs:");
      $display("");
      $display("Core inputs and outputs:");
      $display("init: 0x%01x, next: 0x%01x, ready: 0x%01x",
               dut.core.init, dut.core.next, dut.core.ready);
      $display("key:    0x%032x", dut.core.key);
      $display("block:  0x%032x", dut.core.block);
      $display("result: 0x%032x", dut.core.result);
      $display("");
    end
  endtask // dump_dut_state


  //----------------------------------------------------------------
  // reset_dut()
  //
  // Toggle reset to put the DUT into a well known state.
  //----------------------------------------------------------------
  task reset_dut;
    begin
      $display("*** Toggle reset.");
      tb_reset_n = 0;
      #(2 * CLK_PERIOD);
      tb_reset_n = 1;
    end
  endtask // reset_dut


  //----------------------------------------------------------------
  // display_test_result()
  //
  // Display the accumulated test results.
  //----------------------------------------------------------------
  task display_test_result;
    begin
      if (error_ctr == 0)
        begin
          $display("*** All %02d test cases completed successfully", tc_ctr);
        end
      else
        begin
          $display("*** %02d tests completed - %02d test cases did not complete successfully.",
                   tc_ctr, error_ctr);
        end
    end
  endtask // display_test_result


  //----------------------------------------------------------------
  // init_sim()
  //
  // Initialize all counters and testbed functionality as well
  // as setting the DUT inputs to defined values.
  //----------------------------------------------------------------
  task init_sim;
    begin
      cycle_ctr  = 0;
      error_ctr  = 0;
      tc_ctr     = 0;
      tb_monitor = 0;

      tb_clk        = 1'h0;
      tb_reset_n    = 1'h1;
      tb_cs         = 1'h0;
      tb_we         = 1'h0;
      tb_address    = 8'h0;
      tb_write_data = 32'h0;
    end
  endtask // init_sim


  //----------------------------------------------------------------
  // write_word()
  //
  // Write the given word to the DUT using the DUT interface.
  //----------------------------------------------------------------
  task write_word(input [11 : 0] address,
                  input [31 : 0] word);
    begin
      if (DEBUG)
        begin
          $display("*** Writing 0x%08x to 0x%02x.", word, address);
          $display("");
        end

      tb_address = address;
      tb_write_data = word;
      tb_cs = 1;
      tb_we = 1;
      #(2 * CLK_PERIOD);
      tb_cs = 0;
      tb_we = 0;
    end
  endtask // write_word


  //----------------------------------------------------------------
  // read_word()
  //
  // Read a data word from the given address in the DUT.
  // the word read will be available in the global variable
  // read_data.
  //----------------------------------------------------------------
  task read_word(input [11 : 0]  address);
    begin
      tb_address = address;
      tb_cs = 1;
      tb_we = 0;
      #(CLK_PERIOD);
      read_data = tb_read_data;
      tb_cs = 0;

      if (DEBUG)
        begin
          $display("*** Reading 0x%08x from 0x%02x.", read_data, address);
          $display("");
        end
    end
  endtask // read_word


  //----------------------------------------------------------------
  // wait_ready()
  //
  // Wait for the ready flag to be set in dut.
  //----------------------------------------------------------------
  task wait_ready;
    begin : wready
      read_word(ADDR_STATUS);
      while (read_data == 0)
        read_word(ADDR_STATUS);
    end
  endtask // wait_ready



  //----------------------------------------------------------------
  //----------------------------------------------------------------
  task enctest(input integer tc, input reg [127 : 0] key,
               input reg [127 : 0] plaintext, input reg [127 : 0] expected);
    begin : enctest
      reg [127 : 0] encres;

      tb_monitor = 1'h1;
      #(CLK_PERIOD);

      $display("*** TC%01d - Writing key and block to dut.", tc);
      write_word(ADDR_KEY0, key[031 : 000]);
      write_word(ADDR_KEY1, key[063 : 032]);
      write_word(ADDR_KEY2, key[095 : 064]);
      write_word(ADDR_KEY3, key[127 : 096]);

      write_word(ADDR_BLOCK0, plaintext[031 : 000]);
      write_word(ADDR_BLOCK1, plaintext[063 : 032]);
      write_word(ADDR_BLOCK2, plaintext[095 : 064]);
      write_word(ADDR_BLOCK3, plaintext[127 : 096]);

      write_word(ADDR_CTRL, 32'h1);


      $display("*** TC%01d - encryption started.", tc);
      write_word(ADDR_CTRL, 32'h2);
      #(CLK_PERIOD);

      wait_ready();
      $display("*** TC%01d - encryption completed.", tc);
      #(CLK_PERIOD);
      tb_monitor = 0;

      read_word(ADDR_RESULT0);
      encres[031 : 000] = read_data;
      read_word(ADDR_RESULT1);
      encres[063 : 032] = read_data;
      read_word(ADDR_RESULT2);
      encres[095 : 064] = read_data;
      read_word(ADDR_RESULT3);
      encres[127 : 096] = read_data;

      if (encres == expected)
        $display("*** TC%01d correct ciphertext generated: 0x%032x",
                 tc, encres);
      else
        begin
          error_ctr = error_ctr + 1;
          $display("*** TC%01d incorrect ciphertext generated", tc);
          $display("*** TC%01d expected: 0x%032x", tc, expected);
          $display("*** TC%01d got:      0x%032x", tc, encres);
        end
    end
  endtask // enctest


  //----------------------------------------------------------------
  // gift_test
  //----------------------------------------------------------------
  initial
    begin : gift_test
      $display("   -= Testbench for gift started =-");
      $display("     ============================");
      $display("");

      init_sim();
      reset_dut();

      enctest(1, 128'h0, 128'h0,
              128'hcd0bd738_388ad3f6_68b15a36_ceb6ff92);

      enctest(2, 128'hfedcba98_76543210_fedcba98_76543210,
              128'hfedcba98_76543210_fedcba98_76543210,
              128'h8422241a_6dbf5a93_46af4684_09ee0152);

      enctest(3, 128'hd0f5c59a_7700d3e7_99028fa9_f90ad837,
              128'he39c141f_a57dba43_f08a85b6_a91f86c1,
              128'h13ede67c_bdcc3dbf_400a62d6_977265ea);

      display_test_result();
      $display("");
      $display("*** gift simulation done. ***");
      $finish;
    end // gift_test
endmodule // tb_gift

//======================================================================
// EOF tb_gift.v
//======================================================================
