//======================================================================
//
// gift_core.v
// ----------
// The GIFT core. This core implements the GIFT-128-128 with
// a block size of 128 bits.
//
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

module gift_core(
                input wire            clk,
                input wire            reset_n,

                input wire            init,
                input wire            next,
                output wire           ready,

                input wire [127 : 0]  key,

                input wire [127 : 0]  block,
                output wire [127 : 0] result
               );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam GIFT128_ROUNDS = 6'h28;

  localparam CTRL_IDLE  = 2'h0;
  localparam CTRL_INIT  = 2'h1;
  localparam CTRL_NEXT  = 2'h2;


  //----------------------------------------------------------------
  // Round functions with sub functions.
  //----------------------------------------------------------------
  function [7 : 0] gm2(input [7 : 0] op);
    begin
      gm2 = {op[6 : 0], 1'b0} ^ (8'h1b & {8{op[7]}});
    end
  endfunction // gm2


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [127 : 0] state_reg;
  reg [127 : 0] state_new;
  reg           state_we;

  reg           ready_reg;
  reg           ready_new;
  reg           ready_we;

  reg [5 : 0]   round_ctr_reg;
  reg [5 : 0]   round_ctr_new;
  reg           round_ctr_we;
  reg           round_ctr_rst;
  reg           round_ctr_inc;

  reg [1 : 0]   gift_core_ctrl_reg;
  reg [1 : 0]   gift_core_ctrl_new;
  reg           gift_core_ctrl_we;


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign ready = ready_reg;
  assign result = state_reg;


  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with asynchronous
  // active low reset. All registers have write enable.
  //----------------------------------------------------------------
  always @ (posedge clk or negedge reset_n)
    begin: reg_update
      if (!reset_n)
        begin
          round_ctr_reg      <= 6'h0;
          state_reg          <= 128'h0;
          ready_reg          <= 1'h1;
          gift_core_ctrl_reg <= CTRL_IDLE;
        end
      else
        begin
          if (state_we)
            state_reg <= state_new;

          if (ready_we)
            ready_reg <= ready_new;

          if (round_ctr_we)
            round_ctr_reg <= round_ctr_new;

          if (gift_core_ctrl_we)
            gift_core_ctrl_reg <= gift_core_ctrl_new;
        end
    end // reg_update


  //----------------------------------------------------------------
  // gift_logic
  //----------------------------------------------------------------
  always @*
    begin : gift_logic
      state_new = 128'h0;
      state_we  = 1'h0;

    end


  //----------------------------------------------------------------
  // round_ctr
  //
  // The round counter with reset and increase logic.
  //----------------------------------------------------------------
  always @*
    begin : round_ctr
      round_ctr_new = 6'h0;
      round_ctr_we  = 1'h0;

      if (round_ctr_rst)
        begin
          round_ctr_new = 6'h0;
          round_ctr_we  = 1'h1;
        end
      else if (round_ctr_inc)
        begin
          round_ctr_new = round_ctr_reg + 1'h1;
          round_ctr_we  = 1'h1;
        end
    end // round_ctr


  //----------------------------------------------------------------
  // gift_core_ctrl
  //
  // Control FSM for gift core.
  //----------------------------------------------------------------
  always @*
    begin : gift_core_ctrl
      ready_new          = 1'h0;
      ready_we           = 1'h0;
      round_ctr_rst      = 1'h0;
      round_ctr_inc      = 1'h0;
      gift_core_ctrl_new = CTRL_IDLE;
      gift_core_ctrl_we  = 1'h0;

      case (gift_core_ctrl_reg)
        CTRL_IDLE:
          begin
            if (init)
              begin
                ready_new          = 1'h0;
                ready_we           = 1'h1;
                gift_core_ctrl_new = CTRL_INIT;
                gift_core_ctrl_we  = 1'h1;
              end
            else if (next)
              begin
                ready_new          = 1'h0;
                ready_we           = 1'h1;
                gift_core_ctrl_new = CTRL_NEXT;
                gift_core_ctrl_we  = 1'h1;
              end
          end

        CTRL_INIT:
          begin
            ready_new          = 1'h1;
            ready_we           = 1'h1;
            gift_core_ctrl_new = CTRL_IDLE;
            gift_core_ctrl_we  = 1'h1;
          end

        CTRL_NEXT:
          begin
            ready_new          = 1'h1;
            ready_we           = 1'h1;
            gift_core_ctrl_new = CTRL_IDLE;
            gift_core_ctrl_we  = 1'h1;
          end

        default:
          begin

          end
      endcase // case (gift_core_ctrl_reg)

    end // gift_core_ctrl
endmodule // gift_core

//======================================================================
// EOF gift_core.v
//======================================================================
