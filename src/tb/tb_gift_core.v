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
  reg            tb_encdec;
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

                .encdec(tb_encdec),
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
      $display("");
      $display("State of DUT");
      $display("------------");
      $display("Cycle: %08d", cycle_ctr);
      $display("Inputs and outputs:");
      $display("next:   0x%01x, encdec: 0x%01x, ready: 0x%01x",
               dut.next, dut.encdec, dut.ready);
      $display("key:    0x%032x ", dut.key);
      $display("block:  0x%032x", dut.block);
      $display("result: 0x%032x", dut.result);
      $display("");
      $display("Intermediate values:");
      $display("enc_subcell_state: 0x%032x", dut.gift_logic.enc_subcell_state);
      $display("enc_permute_state: 0x%032x", dut.gift_logic.enc_permute_state);
      $display("enc_addkey_state:  0x%032x", dut.gift_logic.enc_addkey_state);
      $display("");
      $display("dec_subcell_state: 0x%032x", dut.gift_logic.dec_subcell_state);
      $display("dec_permute_state: 0x%032x", dut.gift_logic.dec_permute_state);
      $display("dec_addkey_state:  0x%032x", dut.gift_logic.dec_addkey_state);
      $display("");
      $display("Internal state:");
      $display("ready_reg:          0x%1x,   ready_new: 0x%1x,   ready_we: 0x%1x",
               dut.ready_reg, dut.ready_new, dut.ready_we);
      $display("state_reg:          0x%032x, state_new: 0x%032x, state_we: 0x%1x",
               dut.state_reg, dut.state_new, dut.state_we);
      $display("key_reg:            0x%032x, key_new:   0x%032x, key_we:   0x%1x",
               dut.key_reg, dut.key_new, dut.key_we);
      $display("rc_reg:             0x%02x, rc_new:   0x%02x, rc_we:   0x%1x",
               dut.rc_reg, dut.rc_new, dut.rc_we);
      $display("round_ctr_reg:      0x%08x, round_ctr_new: 0x%08x, round_ctr_we: 0x%1x",
               dut.round_ctr_reg, dut.round_ctr_new, dut.round_ctr_we);
      $display("gift_core_ctrl_reg: 0x%08x, gift_core_ctrl_new: 0x%08x, gift_core_ctrl_we: 0x%1x",
               dut.gift_core_ctrl_reg, dut.gift_core_ctrl_new, dut.gift_core_ctrl_we);
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
      tb_encdec  = 0;
      tb_next    = 0;
      tb_key     = 128'h0;
      tb_block   = 128'h0;
    end
  endtask // init_sim


  //----------------------------------------------------------------
  //----------------------------------------------------------------
  task enctest(input reg [127 : 0] key, input reg [127 : 0] plaintext,
               input reg [127 : 0] expected);
    begin
      tc_ctr = tc_ctr + 1;

      tb_monitor = 1'h0;
      #(CLK_PERIOD);
      $display("*** TC%01d - encryption started.", tc_ctr);
      tb_encdec  = 1;
      tb_key     = key;
      tb_block   = plaintext;

      tb_next    = 1'h1;
      #(CLK_PERIOD);
      tb_next    = 1'h0;

      wait_ready();
      $display("*** TC%01d - encryption completed.", tc_ctr);
      #(CLK_PERIOD);
      tb_monitor = 0;

      if (tb_result == expected)
        $display("*** TC%01d correct ciphertext generated: 0x%032x",
                 tc_ctr, tb_result);
      else
        begin
          error_ctr = error_ctr + 1;
          $display("*** TC%01d incorrect ciphertext generated", tc_ctr);
          $display("*** expected: 0x%032x", expected);
          $display("*** got:      0x%032x", tb_result);
        end
      $display("");
    end
  endtask // enctest


  //----------------------------------------------------------------
  // dectest
  //----------------------------------------------------------------
  task dectest(input reg [127 : 0] key, input reg [127 : 0] ciphertext,
               input reg [127 : 0] expected);
    begin
      tc_ctr = tc_ctr + 1;

      tb_monitor = 1'h1;
      #(CLK_PERIOD);
      $display("*** TC%01d - decryption started.", tc_ctr);
      tb_encdec  = 0;
      tb_key     = key;
      tb_block   = ciphertext;

      tb_next    = 1'h1;
      #(CLK_PERIOD);
      tb_next    = 1'h0;

      wait_ready();
      $display("*** TC%01d - decryption completed.", tc_ctr);
      #(CLK_PERIOD);
      tb_monitor = 0;

      if (tb_result == expected)
        $display("*** TC%01d correct plaintext generated: 0x%032x",
                 tc_ctr, tb_result);
      else
        begin
          error_ctr = error_ctr + 1;
          $display("*** TC%01d incorrect plaintext generated", tc_ctr);
          $display("*** expected: 0x%032x", expected);
          $display("*** got:      0x%032x", tb_result);
        end
      $display("");
    end
  endtask // dectest


  //----------------------------------------------------------------
  // gift_core_test
  //
  // Test vectors from:
  // https://github.com/giftcipher/gift/
  //----------------------------------------------------------------
  initial
    begin : gift_core_test
      $display("*** Simulation of GIFT core started.");
      $display("");

      init_sim();
      reset_dut();


      enctest(128'h0,
              128'h0,
              128'hcd0bd738_388ad3f6_68b15a36_ceb6ff92);

      enctest(128'hfedcba98_76543210_fedcba98_76543210,
              128'hfedcba98_76543210_fedcba98_76543210,
              128'h8422241a_6dbf5a93_46af4684_09ee0152);

      enctest(128'hd0f5c59a_7700d3e7_99028fa9_f90ad837,
              128'he39c141f_a57dba43_f08a85b6_a91f86c1,
              128'h13ede67c_bdcc3dbf_400a62d6_977265ea);


      dectest(128'h0,
              128'hcd0bd738_388ad3f6_68b15a36_ceb6ff92,
              128'h0);

      dectest(128'hfedcba98_76543210_fedcba98_76543210,
              128'h8422241a_6dbf5a93_46af4684_09ee0152,
              128'hfedcba98_76543210_fedcba98_76543210);

      dectest(128'hd0f5c59a_7700d3e7_99028fa9_f90ad837,
              128'h13ede67c_bdcc3dbf_400a62d6_977265ea,
              128'he39c141f_a57dba43_f08a85b6_a91f86c1);


      display_test_result();
      $display("");
      $display("*** Simulation of GIFT core completed.");
      $finish;
    end // gift_core_test
endmodule // tb_gift_core

//======================================================================
// EOF tb_gift_core.v
//======================================================================
