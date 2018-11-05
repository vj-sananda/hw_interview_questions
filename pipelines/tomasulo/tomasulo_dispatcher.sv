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

`include "tomasulo_pkg.vh"

module tomasulo_dispatcher (

   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

   //
     input                                        clk
   , input                                        rst

   //======================================================================== //
   //                                                                         //
   // CDB                                                                     //
   //                                                                         //
   //======================================================================== //

   //
   , input tomasulo_pkg::cdb_t                    cdb_r

   //======================================================================== //
   //                                                                         //
   // OUT                                                                     //
   //                                                                         //
   //======================================================================== //

   //
   , output                                       out_vld_r
   , output tomasulo_pkg::reg_t                   out_wa_r
   , output tomasulo_pkg::word_t                  out_wdata_r

   //======================================================================== //
   //                                                                         //
   // INST                                                                    //
   //                                                                         //
   //======================================================================== //

   //
   , input tomasulo_pkg::inst_t                   inst
   //
   , output logic                                 inst_adv

   //======================================================================== //
   //                                                                         //
   // DISPATCH                                                                //
   //                                                                         //
   //======================================================================== //

   //
   , input                                        arith_0_full_r
   , input                                        arith_1_full_r
   , input                                        logic_0_full_r
   , input                                        logic_1_full_r
   , input                                        mpy_full_r
   //
   , output tomasulo_pkg::dispatch_t              dis_r
   , output logic                                 arith_0_dis_vld_r
   , output logic                                 arith_1_dis_vld_r
   , output logic                                 logic_0_dis_vld_r
   , output logic                                 logic_1_dis_vld_r
   , output logic                                 mpy_dis_vld_r
);

  import tomasulo_pkg::*;

  //
  logic                                 arith_0_dis_vld_w;
  logic                                 arith_1_dis_vld_w;
  logic                                 logic_0_dis_vld_w;
  logic                                 logic_1_dis_vld_w;
  logic                                 mpy_dis_vld_w;
  //
  logic                                 dis_en;
  dispatch_t                            dis_w;
  //
  logic [1:0]                           rf__ren;
  reg_t [1:0]                           rf__ra;
  word_t [1:0]                          rf__rdata;
  //
  logic                                 rf__wen;
  reg_t                                 rf__wa;
  word_t                                rf__wdata;

  // ======================================================================== //
  //                                                                          //
  // Combinational Logic                                                      //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : foo_PROC

      //
      dis_en             = '0;
      dis_w              = '0;

      //
      arith_0_dis_vld_w  = '0;
      arith_1_dis_vld_w  = '0;
      logic_0_dis_vld_w  = '0;
      logic_1_dis_vld_w  = '0;
      mpy_dis_vld_w      = '0;

    end

  // ======================================================================== //
  //                                                                          //
  // Flops                                                                    //
  //                                                                          //
  // ======================================================================== //
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (dis_en)
      dis_r <= dis_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst) begin
      arith_0_dis_vld_r <= '0;
      arith_1_dis_vld_r <= '0;
      logic_0_dis_vld_r <= '0;
      logic_1_dis_vld_r <= '0;
      mpy_dis_vld_r     <= '0;
    end else begin
      arith_0_dis_vld_r <= arith_0_dis_vld_w;
      arith_1_dis_vld_r <= arith_1_dis_vld_w;
      logic_0_dis_vld_r <= logic_0_dis_vld_w;
      logic_1_dis_vld_r <= logic_1_dis_vld_w;
      mpy_dis_vld_r     <= mpy_dis_vld_w;
    end

  // ======================================================================== //
  //                                                                          //
  // Instances                                                                //
  //                                                                          //
  // ======================================================================== //
  
  // ------------------------------------------------------------------------ //
  //
  rf #(.N($bits(word_t)), .WR_N(1), .RD_N(2), .FLOP_OUT('b1)) u_rf (
    //
      .clk          (clk           )
    , .rst          (rst           )
    //
    , .ra           (rf__ra        )
    , .ren          (rf__ren       )
    , .rdata        (rf__rdata     )
    //
    , .wa           (rf__wa        )
    , .wen          (rf__wen       )
    , .wdata        (rf__wdata     )
  );

endmodule
