//========================================================================== //
// Copyright (c) 2016, Stephen Henry
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

module mem_egress_pipe #(
    parameter int W = 8
  , parameter int MEMORY_LATENCY_N = 2
) (

   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

     input                                   clk
   , input                                   rst

   //======================================================================== //
   //                                                                         //
   // Control Interface                                                       //
   //                                                                         //
   //======================================================================== //

   , input                                   read_en
   //
   , output logic                            read_adv

   //======================================================================== //
   //                                                                         //
   // Mem Interface                                                           //
   //                                                                         //
   //======================================================================== //

   , input         [W-1:0]                   mem_rdata
   , output logic                            mem_ren

   //======================================================================== //
   //                                                                         //
   // Egress                                                                  //
   //                                                                         //
   //======================================================================== //

   , input                                   out_accept
   //
   , output logic                            out_valid_r
   , output logic [W-1:0]                    out_data_r
);

  // ======================================================================== //
  //                                                                          //
  // Wires                                                                    //
  //                                                                          //
  // ======================================================================== //

  logic                                      s0_adv;
  logic                                      s0_stall;
  //
  logic                                      s1_stall;
  //
  logic                                      s1_valid_w;
  logic                                      s1_valid_r;
  //
  logic                                      s2_stall;
  logic                                      s2_en;
  //
  logic [W-1:0]                              s2_ucode_r;
  logic [W-1:0]                              s2_ucode_w;
  //
  logic                                      s2_valid_w;
  logic                                      s2_valid_r;
  //
  logic                                      s1_pass;
  logic [W-1:0]                              s1_retain_w;
  logic [W-1:0]                              s1_retain_r;
  logic                                      s1_retain_en;
  //
  logic                                      s1_retain_valid_w;
  logic                                      s1_retain_valid_r;

  // ======================================================================== //
  //                                                                          //
  // Combinatorial Logic                                                      //
  //                                                                          //
  // ======================================================================== //

  //
  always_comb mem_ren  = s0_adv;
  always_comb read_adv = s0_adv;
  always_comb out_valid_r  = s2_valid_r;
  always_comb out_data_r = s2_ucode_r;
  //
  always_comb s0_adv  = read_en & (~s0_stall);
  always_comb s0_stall = s1_stall;
  //
  always_comb s1_stall =  (s1_retain_valid_r | s1_valid_r) & s2_stall;
  always_comb s1_valid_w  = (~rst) & s0_adv;
  always_comb s1_retain_w  = mem_rdata;
  always_comb s1_retain_en  = s1_valid_r & s1_stall;
  always_comb s1_pass = s1_valid_r | s1_retain_valid_r;
  always_comb s1_retain_valid_w  = (~rst) & (s1_pass & s1_stall);
  //
  always_comb s2_en = (~s2_stall) & (s1_valid_r | s1_retain_valid_r);
  always_comb s2_stall  = out_valid_r & (~out_accept);
  always_comb s2_ucode_w  = s1_retain_valid_r ? s1_retain_r : mem_rdata;
  always_comb s2_valid_w  =   (~rst)
                            & (   (s2_valid_r & s2_stall)
                                | (s1_pass & (~s1_stall))
                              )
                          ;

  // ======================================================================== //
  //                                                                          //
  // Flops                                                                    //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (s1_retain_en)
      s1_retain_r <= s1_retain_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    s1_retain_valid_r <= s1_retain_valid_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    begin : valid_reg_PROC
      s1_valid_r <= s1_valid_w;
      s2_valid_r <= s2_valid_w;
    end

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (s2_en)
      s2_ucode_r <= s2_ucode_w;

endmodule // mem_egress_pipe
