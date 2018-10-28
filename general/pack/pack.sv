//========================================================================== //
// Copyright (c) 2018, Stephen Henry
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//========================================================================== //

`include "libtb2.vh"
`include "libv2_pkg.vh"

module pack #(parameter int N = 8, parameter int W = 32) (

   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

     input                                        clk
   , input                                        rst

   //======================================================================== //
   //                                                                         //
   // In                                                                      //
   //                                                                         //
   //======================================================================== //

   , input [N-1:0][W-1:0]                         in_w
   , input [N-1:0]                                in_vld_w

   //======================================================================== //
   //                                                                         //
   // Out                                                                     //
   //                                                                         //
   //======================================================================== //

   , output logic [N-1:0][W-1:0]                  out_r
   , output logic [N-1:0]                         out_vld_r
);

  //
  localparam int IDX_W  = $clog2(N);

  typedef logic [IDX_W-1:0] idx_t;

  //
  logic [N-1:0][W-1:0]                       out_r;
  logic [N-1:0][W-1:0]                       out_w;
  logic [N-1:0]                              out_en;

  //
  logic [N-1:0]                              out_vld_r;
  logic [N-1:0]                              out_vld_w;

  //
  logic [N-1:0][IDX_W-1:0]                   cnt;
  
  // ======================================================================== //
  //                                                                          //
  // Combinatorial Logic                                                      //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  function [IDX_W-1:0] popcnt(logic [N-1:0] n); begin
    popcnt      = '0;
    for (int i = 0; i < $bits(n); i++)
      popcnt += n[i] ? 'b1 : 'b0;
  end endfunction
  
  // ------------------------------------------------------------------------ //
  //
  function [N-1:0] mask_off_n(logic [N-1:0] a, int n); begin
    mask_off_n  = a;
    for (int i = 0; i < n; i++)
      mask_off_n [i]  = 'b0;
  end endfunction

  function [N-1:0] to_unary(logic [IDX_W-1:0] n); begin
    to_unary  = ~('1 << n);
  end endfunction

  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : cntrl_PROC


      for (int i = 0; i < N; i++) begin

        cnt [i]    = popcnt(mask_off_n(~in_vld_w, i));

        out_w [i]  = '0;
        
        for (int j = i; j < N; j++)
          out_w [i] |= (cnt [i] == idx_t'(j)) ? in_w [i] : '0;

      end // for (int i = 0; i < N; i++)

      out_vld_w      = to_unary(popcnt(in_vld_w));

      out_en         = out_vld_w;

    end // block: cntrl_PROC

  // ======================================================================== //
  //                                                                          //
  // Sequential Logic                                                         //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      out_vld_r <= '0;
    else
      out_vld_r <= out_vld_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    for (int i = 0; i < N; i++)
      if (out_en [i])
        out_r [i] <= out_w [i];

endmodule // pack

  
