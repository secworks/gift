//======================================================================
//
// tb_gift_core.v
// --------------
// Testbench for the gift block cipher core.
//
// Author: Joachim Strombergson
// Copyright (c) 2021, Assured AB
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

module tb_gift_core();

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter DEBUG     = 0;
  parameter DUMP_WAIT = 0;

  parameter CLK_HALF_PERIOD = 1;
  parameter CLK_PERIOD = 2 * CLK_HALF_PERIOD;


  //----------------------------------------------------------------
  // Register and Wire declarations.
  //----------------------------------------------------------------
  reg [31 : 0] cycle_ctr;
  reg [31 : 0] error_ctr;
  reg [31 : 0] tc_ctr;
  reg          tb_monitor;

  reg            tb_clk;
  reg            tb_reset_n;
  reg            tb_init;
  reg            tb_next;
  wire           tb_ready;
  reg [127 : 0]  tb_key;
  reg [127 : 0]  tb_block;
  wire [127 : 0] tb_result;


  //----------------------------------------------------------------
  // Device Under Test.
  //----------------------------------------------------------------
  gift_core dut(
                .clk(tb_clk),
                .reset_n(tb_reset_n),

                .init(tb_init),
                .next(tb_next),
                .ready(tb_ready),

                .key(tb_key),

                .block(tb_block),
                .result(tb_result)
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
      $display("init = 0x%01x, next = 0x%01x", dut.init, dut.next);
      $display("key   = 0x%016x ", dut.key);
      $display("block = 0x%08x", dut.block);
      $display("ready  = 0x%01x", dut.ready);
      $display("result = 0x%08x", dut.result);
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
      $display("*** DUT before reset:");
      dump_dut_state();
      $display("*** Toggling reset.");
      tb_reset_n = 0;
      #(2 * CLK_PERIOD);
      tb_reset_n = 1;
      $display("*** DUT after reset:");
      dump_dut_state();
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
  // wait_ready()
  //
  // Wait for the ready flag in the dut to be set.
  //
  // Note: It is the callers responsibility to call the function
  // when the dut is actively processing and will in fact at some
  // point set the flag.
  //----------------------------------------------------------------
  task wait_ready;
    begin
      #(2 * CLK_PERIOD);
      while (!tb_ready)
        begin
          #(CLK_PERIOD);
          if (DUMP_WAIT)
            begin
              dump_dut_state();
            end
        end
    end
  endtask // wait_ready


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

      tb_clk     = 0;
      tb_reset_n = 1;
      tb_init    = 0;
      tb_next    = 0;
      tb_key     = 128'h0;
      tb_block   = 128'h0;
    end
  endtask // init_sim


  //----------------------------------------------------------------
  //----------------------------------------------------------------
  task enctest(input integer tc, input reg [127 : 0] key,
               input reg [127 : 0] plaintext, input reg [127 : 0] expected);
    begin
      tb_monitor = 1'h1;
      #(CLK_PERIOD);
      $display("*** TC%01d - encryption started.", tc);
      tb_key     = key;
      tb_block   = plaintext;

      tb_next    = 1'h1;
      #(CLK_PERIOD);
      tb_next    = 1'h0;

      wait_ready();
      $display("*** TC%01d - encryption completed.", tc);
      #(CLK_PERIOD);
      tb_monitor = 0;

      if (tb_result == expected)
        $display("*** TC%01d correct ciphertext generated: 0x%032x",
                 tc, tb_result);
      else
        begin
          error_ctr = error_ctr + 1;
          $display("*** TC%01d incorrect ciphertext generated", tc);
          $display("*** TC%01d expected: 0x%032x", tc, expected);
          $display("*** TC%01d got:      0x%032x", tc, tb_result);
        end
    end
  endtask // enctest


  //----------------------------------------------------------------
  // gift_core_test
  //
  // Test vectors from:
  //----------------------------------------------------------------
  initial
    begin : gift_core_test
      $display("*** Simulation of GIFT core started.");
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
      $display("*** Simulation of GIFT core completed.");
      $finish;
    end // gift_core_test
endmodule // tb_gift_core

//======================================================================
// EOF tb_gift_core.v
//======================================================================
