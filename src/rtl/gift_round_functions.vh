//======================================================================
//
// gift_round_functions.v
// ----------------------
// Combinational functions to implement the GIFT encipher and
// decipher rounds.
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

  //----------------------------------------------------------------
  // Encyption functions.
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


  function [127 : 0] PermBits(input [127 : 0] x);
    begin
      PermBits[000] = x[000];
      PermBits[033] = x[001];
      PermBits[066] = x[002];
      PermBits[099] = x[003];
      PermBits[096] = x[004];
      PermBits[001] = x[005];
      PermBits[034] = x[006];
      PermBits[067] = x[007];
      PermBits[064] = x[008];
      PermBits[097] = x[009];
      PermBits[002] = x[010];
      PermBits[035] = x[011];
      PermBits[032] = x[012];
      PermBits[065] = x[013];
      PermBits[098] = x[014];
      PermBits[003] = x[015];

      PermBits[004] = x[016];
      PermBits[037] = x[017];
      PermBits[070] = x[018];
      PermBits[103] = x[019];
      PermBits[100] = x[020];
      PermBits[005] = x[021];
      PermBits[038] = x[022];
      PermBits[071] = x[023];
      PermBits[068] = x[024];
      PermBits[101] = x[025];
      PermBits[006] = x[026];
      PermBits[039] = x[027];
      PermBits[036] = x[028];
      PermBits[069] = x[029];
      PermBits[102] = x[030];
      PermBits[007] = x[031];

      PermBits[008] = x[032];
      PermBits[041] = x[033];
      PermBits[074] = x[034];
      PermBits[107] = x[035];
      PermBits[104] = x[036];
      PermBits[009] = x[037];
      PermBits[042] = x[038];
      PermBits[075] = x[039];
      PermBits[072] = x[040];
      PermBits[105] = x[041];
      PermBits[010] = x[042];
      PermBits[043] = x[043];
      PermBits[040] = x[044];
      PermBits[073] = x[045];
      PermBits[106] = x[046];
      PermBits[011] = x[047];

      PermBits[012] = x[048];
      PermBits[045] = x[049];
      PermBits[078] = x[050];
      PermBits[111] = x[051];
      PermBits[108] = x[052];
      PermBits[013] = x[053];
      PermBits[046] = x[054];
      PermBits[079] = x[055];
      PermBits[076] = x[056];
      PermBits[109] = x[057];
      PermBits[014] = x[058];
      PermBits[047] = x[059];
      PermBits[044] = x[060];
      PermBits[077] = x[061];
      PermBits[110] = x[062];
      PermBits[015] = x[063];

      PermBits[016] = x[064];
      PermBits[049] = x[065];
      PermBits[082] = x[066];
      PermBits[115] = x[067];
      PermBits[112] = x[068];
      PermBits[017] = x[069];
      PermBits[050] = x[070];
      PermBits[083] = x[071];
      PermBits[080] = x[072];
      PermBits[113] = x[073];
      PermBits[018] = x[074];
      PermBits[051] = x[075];
      PermBits[048] = x[076];
      PermBits[081] = x[077];
      PermBits[114] = x[078];
      PermBits[019] = x[079];

      PermBits[020] = x[080];
      PermBits[053] = x[081];
      PermBits[086] = x[082];
      PermBits[119] = x[083];
      PermBits[116] = x[084];
      PermBits[021] = x[085];
      PermBits[054] = x[086];
      PermBits[087] = x[087];
      PermBits[084] = x[088];
      PermBits[117] = x[089];
      PermBits[022] = x[090];
      PermBits[055] = x[091];
      PermBits[052] = x[092];
      PermBits[085] = x[093];
      PermBits[118] = x[094];
      PermBits[023] = x[095];

      PermBits[024] = x[096];
      PermBits[057] = x[097];
      PermBits[090] = x[098];
      PermBits[123] = x[099];
      PermBits[120] = x[100];
      PermBits[025] = x[101];
      PermBits[058] = x[102];
      PermBits[091] = x[103];
      PermBits[088] = x[104];
      PermBits[121] = x[105];
      PermBits[026] = x[106];
      PermBits[059] = x[107];
      PermBits[056] = x[108];
      PermBits[089] = x[109];
      PermBits[122] = x[110];
      PermBits[027] = x[111];

      PermBits[028] = x[112];
      PermBits[061] = x[113];
      PermBits[094] = x[114];
      PermBits[127] = x[115];
      PermBits[124] = x[116];
      PermBits[029] = x[117];
      PermBits[062] = x[118];
      PermBits[095] = x[119];
      PermBits[092] = x[120];
      PermBits[125] = x[121];
      PermBits[030] = x[122];
      PermBits[063] = x[123];
      PermBits[060] = x[124];
      PermBits[093] = x[125];
      PermBits[126] = x[126];
      PermBits[031] = x[127];
    end
  endfunction // PermBits


  function [127 : 0] AddRoundKey(input [127 : 0] state,
                                 input [127 : 0] k,
                                 input [5 : 0]   rc);
    begin : ark
      reg [31 : 0] u;
      reg [31 : 0] v;
      reg [127 : 0] s;
      integer i;

      u = k[095 : 064];
      v = k[031 : 000];

      s = state;
      for (i = 0 ; i < 32 ; i = i + 1) begin
        s[(4 * i + 2)] = state[(4 * i + 2)] ^ u[i];
        s[(4 * i + 1)] = state[(4 * i + 1)] ^ v[i];
      end

      s[127] = s[127] ^ 1'h1;
      s[023] = s[023] ^ rc[5];
      s[019] = s[019] ^ rc[4];
      s[015] = s[015] ^ rc[3];
      s[011] = s[011] ^ rc[2];
      s[007] = s[007] ^ rc[1];
      s[003] = s[003] ^ rc[0];

      AddRoundKey = s;
    end
  endfunction // AddRoundkey


  function [127 : 0] UpdateKey(input [127 : 0] k);
    begin : udk
      reg [15 : 0] rot12_k0;
      reg [15 : 0] rot2_k1;

      rot12_k0 = {k[011 : 000], k[015 : 012]};
      rot2_k1  = {k[017 : 016], k[031 : 018]};

      UpdateKey = {rot2_k1, rot12_k0, k[127 : 032]};
    end
  endfunction // UpdateKey


  function [5 : 0] UpdateConstant(input [5 : 0] rc);
    begin
      UpdateConstant = {rc[4 : 0], rc[5] ^ rc[4] ^ 1'h1};
    end
  endfunction // UpdateConstant


  //----------------------------------------------------------------
  // Decryption functions.
  //----------------------------------------------------------------
  function [3 : 0] inv_gs(input [3 : 0] x);
    begin
      case (x)
       4'h0 : inv_gs = 4'hd;
       4'h1 : inv_gs = 4'h0;
       4'h2 : inv_gs = 4'h8;
       4'h3 : inv_gs = 4'h6;
       4'h4 : inv_gs = 4'h2;
       4'h5 : inv_gs = 4'hc;
       4'h6 : inv_gs = 4'h4;
       4'h7 : inv_gs = 4'hb;
       4'h8 : inv_gs = 4'he;
       4'h9 : inv_gs = 4'h7;
       4'ha : inv_gs = 4'h1;
       4'hb : inv_gs = 4'ha;
       4'hc : inv_gs = 4'h3;
       4'hd : inv_gs = 4'h9;
       4'he : inv_gs = 4'hf;
       4'hf : inv_gs = 4'h5;
        default:
          begin
          end
      endcase // case (x)
    end
  endfunction // inv_gs



  function [127 : 0] InvSubCells(input [127 : 0] x);
    begin
      InvSubCells[003 : 000] = inv_gs(x[003 : 000]);
      InvSubCells[007 : 004] = inv_gs(x[007 : 004]);
      InvSubCells[011 : 008] = inv_gs(x[011 : 008]);
      InvSubCells[015 : 012] = inv_gs(x[015 : 012]);
      InvSubCells[019 : 016] = inv_gs(x[019 : 016]);
      InvSubCells[023 : 020] = inv_gs(x[023 : 020]);
      InvSubCells[027 : 024] = inv_gs(x[027 : 024]);
      InvSubCells[031 : 028] = inv_gs(x[031 : 028]);
      InvSubCells[035 : 032] = inv_gs(x[035 : 032]);
      InvSubCells[039 : 036] = inv_gs(x[039 : 036]);
      InvSubCells[043 : 040] = inv_gs(x[043 : 040]);
      InvSubCells[047 : 044] = inv_gs(x[047 : 044]);
      InvSubCells[051 : 048] = inv_gs(x[051 : 048]);
      InvSubCells[055 : 052] = inv_gs(x[055 : 052]);
      InvSubCells[059 : 056] = inv_gs(x[059 : 056]);
      InvSubCells[063 : 060] = inv_gs(x[063 : 060]);
      InvSubCells[067 : 064] = inv_gs(x[067 : 064]);
      InvSubCells[071 : 068] = inv_gs(x[071 : 068]);
      InvSubCells[075 : 072] = inv_gs(x[075 : 072]);
      InvSubCells[079 : 076] = inv_gs(x[079 : 076]);
      InvSubCells[083 : 080] = inv_gs(x[083 : 080]);
      InvSubCells[087 : 084] = inv_gs(x[087 : 084]);
      InvSubCells[091 : 088] = inv_gs(x[091 : 088]);
      InvSubCells[095 : 092] = inv_gs(x[095 : 092]);
      InvSubCells[099 : 096] = inv_gs(x[099 : 096]);
      InvSubCells[103 : 100] = inv_gs(x[103 : 100]);
      InvSubCells[107 : 104] = inv_gs(x[107 : 104]);
      InvSubCells[111 : 108] = inv_gs(x[111 : 108]);
      InvSubCells[115 : 112] = inv_gs(x[115 : 112]);
      InvSubCells[119 : 116] = inv_gs(x[119 : 116]);
      InvSubCells[123 : 120] = inv_gs(x[123 : 120]);
      InvSubCells[127 : 124] = inv_gs(x[127 : 124]);
    end
  endfunction // InvSubCells



  function [127 : 0] InvPermBits(input [127 : 0] x);
    begin
      InvPermBits[000] = x[000];
      InvPermBits[001] = x[033];
      InvPermBits[002] = x[066];
      InvPermBits[003] = x[099];
      InvPermBits[004] = x[096];
      InvPermBits[005] = x[001];
      InvPermBits[006] = x[034];
      InvPermBits[007] = x[067];
      InvPermBits[008] = x[064];
      InvPermBits[009] = x[097];
      InvPermBits[010] = x[002];
      InvPermBits[011] = x[035];
      InvPermBits[012] = x[032];
      InvPermBits[013] = x[065];
      InvPermBits[014] = x[098];
      InvPermBits[015] = x[003];
      InvPermBits[016] = x[004];
      InvPermBits[017] = x[037];
      InvPermBits[018] = x[070];
      InvPermBits[019] = x[103];
      InvPermBits[020] = x[100];
      InvPermBits[021] = x[005];
      InvPermBits[022] = x[038];
      InvPermBits[023] = x[071];
      InvPermBits[024] = x[068];
      InvPermBits[025] = x[101];
      InvPermBits[026] = x[006];
      InvPermBits[027] = x[039];
      InvPermBits[028] = x[036];
      InvPermBits[029] = x[069];
      InvPermBits[030] = x[102];
      InvPermBits[031] = x[007];
      InvPermBits[032] = x[008];
      InvPermBits[033] = x[041];
      InvPermBits[034] = x[074];
      InvPermBits[035] = x[107];
      InvPermBits[036] = x[104];
      InvPermBits[037] = x[009];
      InvPermBits[038] = x[042];
      InvPermBits[039] = x[075];
      InvPermBits[040] = x[072];
      InvPermBits[041] = x[105];
      InvPermBits[042] = x[010];
      InvPermBits[043] = x[043];
      InvPermBits[044] = x[040];
      InvPermBits[045] = x[073];
      InvPermBits[046] = x[106];
      InvPermBits[047] = x[011];
      InvPermBits[048] = x[012];
      InvPermBits[049] = x[045];
      InvPermBits[050] = x[078];
      InvPermBits[051] = x[111];
      InvPermBits[052] = x[108];
      InvPermBits[053] = x[013];
      InvPermBits[054] = x[046];
      InvPermBits[055] = x[079];
      InvPermBits[056] = x[076];
      InvPermBits[057] = x[109];
      InvPermBits[058] = x[014];
      InvPermBits[059] = x[047];
      InvPermBits[060] = x[044];
      InvPermBits[061] = x[077];
      InvPermBits[062] = x[110];
      InvPermBits[063] = x[015];
      InvPermBits[064] = x[016];
      InvPermBits[065] = x[049];
      InvPermBits[066] = x[082];
      InvPermBits[067] = x[115];
      InvPermBits[068] = x[112];
      InvPermBits[069] = x[017];
      InvPermBits[070] = x[050];
      InvPermBits[071] = x[083];
      InvPermBits[072] = x[080];
      InvPermBits[073] = x[113];
      InvPermBits[074] = x[018];
      InvPermBits[075] = x[051];
      InvPermBits[076] = x[048];
      InvPermBits[077] = x[081];
      InvPermBits[078] = x[114];
      InvPermBits[079] = x[019];
      InvPermBits[080] = x[020];
      InvPermBits[081] = x[053];
      InvPermBits[082] = x[086];
      InvPermBits[083] = x[119];
      InvPermBits[084] = x[116];
      InvPermBits[085] = x[021];
      InvPermBits[086] = x[054];
      InvPermBits[087] = x[087];
      InvPermBits[088] = x[084];
      InvPermBits[089] = x[117];
      InvPermBits[090] = x[022];
      InvPermBits[091] = x[055];
      InvPermBits[092] = x[052];
      InvPermBits[093] = x[085];
      InvPermBits[094] = x[118];
      InvPermBits[095] = x[023];
      InvPermBits[096] = x[024];
      InvPermBits[097] = x[057];
      InvPermBits[098] = x[090];
      InvPermBits[099] = x[123];
      InvPermBits[100] = x[120];
      InvPermBits[101] = x[025];
      InvPermBits[102] = x[058];
      InvPermBits[103] = x[091];
      InvPermBits[104] = x[088];
      InvPermBits[105] = x[121];
      InvPermBits[106] = x[026];
      InvPermBits[107] = x[059];
      InvPermBits[108] = x[056];
      InvPermBits[109] = x[089];
      InvPermBits[110] = x[122];
      InvPermBits[111] = x[027];
      InvPermBits[112] = x[028];
      InvPermBits[113] = x[061];
      InvPermBits[114] = x[094];
      InvPermBits[115] = x[127];
      InvPermBits[116] = x[124];
      InvPermBits[117] = x[029];
      InvPermBits[118] = x[062];
      InvPermBits[119] = x[095];
      InvPermBits[120] = x[092];
      InvPermBits[121] = x[125];
      InvPermBits[122] = x[030];
      InvPermBits[123] = x[063];
      InvPermBits[124] = x[060];
      InvPermBits[125] = x[093];
      InvPermBits[126] = x[126];
      InvPermBits[127] = x[031];
    end
  endfunction // InvPermBits


//======================================================================
// EOF gift_round_functions.v
//======================================================================
