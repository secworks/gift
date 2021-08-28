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

                input wire            encdec,
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
  localparam CTRL_NEXT  = 2'h2;


  //----------------------------------------------------------------
  // Round functions with sub functions.
  //----------------------------------------------------------------
`include "gift_round_functions.vh"


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [127 : 0] key_reg;
  reg [127 : 0] key_new;
  reg           key_we;

  reg           encdec_reg;
  reg           encdec_we;

  reg           ready_reg;
  reg           ready_new;
  reg           ready_we;

  reg [127 : 0] state_reg;
  reg [127 : 0] state_new;
  reg           state_we;

  reg [5 : 0]   rc_reg;
  reg [5 : 0]   rc_new;
  reg           rc_we;

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
  reg init_cipher;
  reg update_cipher;


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
          key_reg            <= 128'h0;
          rc_reg             <= 6'h0;
          encdec_reg         <= 1'h0;
          ready_reg          <= 1'h1;
          gift_core_ctrl_reg <= CTRL_IDLE;
        end
      else
        begin
          if (key_we)
            key_reg <= key_new;

          if (state_we)
            state_reg <= state_new;

          if (rc_we)
            rc_reg <= rc_new;

          if (encdec_we)
            encdec_reg <= encdec;

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
      reg [127 : 0] enc_subcell_state;
      reg [127 : 0] enc_permute_state;
      reg [127 : 0] enc_addkey_state;
      reg [127 : 0] enc_key_new;
      reg [5 : 0]   enc_rc_new;

      reg [127 : 0] dec_subcell_state;
      reg [127 : 0] dec_permute_state;
      reg [127 : 0] dec_addkey_state;
      reg [127 : 0] dec_key_new;
      reg [5 : 0]   dec_rc_new;

      key_new   = 128'h0;
      key_we    = 1'h0;
      rc_new    = 6'h0;
      rc_we     = 1'h0;
      encdec_we = 1'h0;
      state_new = 128'h0;
      state_we  = 1'h0;

      enc_key_new       = UpdateKey(key_reg);
      enc_rc_new        = UpdateConstant(rc_reg);
      enc_subcell_state = SubCells(state_reg);
      enc_permute_state = PermBits(enc_subcell_state);
      enc_addkey_state  = AddRoundKey(enc_permute_state, key_reg, enc_rc_new);

      dec_key_new       = InvUpdateKey(key_reg);
      dec_rc_new        = InvUpdateConstant(rc_reg);
      dec_addkey_state  = InvAddRoundKey(state_reg, key_reg, dec_rc_new);
      dec_permute_state = InvPermBits(dec_addkey_state);
      dec_subcell_state = InvSubCells(dec_permute_state);

      if (init_cipher) begin
        // Sample all inputs.
        key_new   = key;
        key_we    = 1'h1;
        rc_new    = 6'h0;
        rc_we     = 1'h1;
        encdec_we = 1'h1;
        state_new = block;
        state_we  = 1'h1;
      end

      else if (update_cipher) begin
        if (encdec_reg) begin
          // Encryption.
          key_new   = enc_key_new;
          key_we    = 1'h1;
          rc_new    = enc_rc_new;
          rc_we     = 1'h1;
          state_new = enc_addkey_state;
          state_we  = 1'h1;
        end

        else begin
          // Decryption.
          key_new   = dec_key_new;
          key_we    = 1'h1;
          rc_new    = dec_rc_new;
          rc_we     = 1'h1;
          state_new = dec_subcell_state;
          state_we  = 1'h1;
        end
      end
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
      init_cipher        = 1'h0;
      update_cipher      = 1'h0;
      gift_core_ctrl_new = CTRL_IDLE;
      gift_core_ctrl_we  = 1'h0;

      case (gift_core_ctrl_reg)
        CTRL_IDLE:
          begin
            if (next)
              begin
                ready_new          = 1'h0;
                ready_we           = 1'h1;
                init_cipher        = 1'h1;
                round_ctr_rst      = 1'h1;
                gift_core_ctrl_new = CTRL_NEXT;
                gift_core_ctrl_we  = 1'h1;
              end
          end


        CTRL_NEXT:
          begin
            if (round_ctr_reg < GIFT128_ROUNDS) begin
              update_cipher      = 1'h1;
              round_ctr_inc      = 1'h1;
            end
            else begin
              ready_new          = 1'h1;
              ready_we           = 1'h1;
              gift_core_ctrl_new = CTRL_IDLE;
              gift_core_ctrl_we  = 1'h1;
            end
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
