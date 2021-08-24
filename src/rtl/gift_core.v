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
  function [3 : 0] gs(input [3 : 0] x);
    begin
      case (x)
        4'h0 : gs = 4'h1;
        4'h1 : gs = 4'ha;
        4'h2 : gs = 4'h4;
        4'h3 : gs = 4'hc;
        4'h4 : gs = 4'h6;
        4'h5 : gs = 4'hf;
        4'h6 : gs = 4'h3;
        4'h7 : gs = 4'h9;
        4'h8 : gs = 4'h2;
        4'h9 : gs = 4'hd;
        4'ha : gs = 4'hb;
        4'hb : gs = 4'h7;
        4'hc : gs = 4'h5;
        4'hd : gs = 4'h0;
        4'he : gs = 4'h8;
        4'hf : gs = 4'he;
        default:
          begin
          end
      endcase // case (nybble)
    end
  endfunction // gs


  // SubCells.
  function [127 : 0] SubCells(input [127 : 0] x);
    begin
      SubCells[003 : 000] = gs(x[003 : 000]);
      SubCells[007 : 004] = gs(x[007 : 004]);
      SubCells[011 : 008] = gs(x[011 : 008]);
      SubCells[015 : 012] = gs(x[015 : 012]);
      SubCells[019 : 016] = gs(x[019 : 016]);
      SubCells[023 : 020] = gs(x[023 : 020]);
      SubCells[027 : 024] = gs(x[027 : 024]);
      SubCells[031 : 028] = gs(x[031 : 028]);

      SubCells[035 : 032] = gs(x[035 : 032]);
      SubCells[039 : 036] = gs(x[039 : 036]);
      SubCells[043 : 040] = gs(x[043 : 040]);
      SubCells[047 : 044] = gs(x[047 : 044]);
      SubCells[051 : 048] = gs(x[051 : 048]);
      SubCells[055 : 052] = gs(x[055 : 052]);
      SubCells[059 : 056] = gs(x[059 : 056]);
      SubCells[063 : 060] = gs(x[063 : 060]);

      SubCells[067 : 064] = gs(x[067 : 064]);
      SubCells[071 : 068] = gs(x[071 : 068]);
      SubCells[075 : 072] = gs(x[075 : 072]);
      SubCells[079 : 076] = gs(x[079 : 076]);
      SubCells[083 : 080] = gs(x[083 : 080]);
      SubCells[087 : 084] = gs(x[087 : 084]);
      SubCells[091 : 088] = gs(x[091 : 088]);
      SubCells[095 : 092] = gs(x[095 : 092]);

      SubCells[099 : 096] = gs(x[099 : 096]);
      SubCells[103 : 100] = gs(x[103 : 100]);
      SubCells[107 : 104] = gs(x[107 : 104]);
      SubCells[111 : 108] = gs(x[111 : 108]);
      SubCells[115 : 112] = gs(x[115 : 112]);
      SubCells[119 : 116] = gs(x[119 : 116]);
      SubCells[123 : 120] = gs(x[123 : 120]);
      SubCells[127 : 124] = gs(x[127 : 124]);
    end
  endfunction // SubCells


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

      state_new = SubCells(state_reg);
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
