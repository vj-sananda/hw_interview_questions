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

module tomasulo_exe_mpy #(parameter int LATENCY_N = 5)(

   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

     input                                        clk
   , input                                        rst

   //======================================================================== //
   //                                                                         //
   // Issue                                                                   //
   //                                                                         //
   //======================================================================== //

   , input                                        iss_vld
   , input  tomasulo_pkg::issue_t                 iss

   //======================================================================== //
   //                                                                         //
   // Completion                                                              //
   //                                                                         //
   //======================================================================== //

   , output tomasulo_pkg::cdb_t                   cdb_r
);
  import tomasulo_pkg::*;

  //
  typedef struct packed {
    reg_t          wa;
    tag_t          tag;
    robid_t        robid;
  } delay_pipe_t;
  localparam int DELAY_PIPE_W  = $bits(delay_pipe_t);

  //
  logic [31:0]                          mpy__a;
  logic [31:0]                          mpy__b;
  logic                                 mpy__pass;
  //
  logic [1:0][31:0]                     mpy__y;
  logic                                 mpy__y_vld_r;

  //
  delay_pipe_t                          delay_pipe_in;
  delay_pipe_t                          delay_pipe_out_r;

  // ======================================================================== //
  //                                                                          //
  // Sequential Logic                                                         //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : cntrl_PROC

      //
      mpy__a         = '0;
      mpy__b         = '0;
      mpy__pass      = '0;

      //
      cdb_r          = '0;
      cdb_r.vld      = mpy__y_vld_r;
      cdb_r.wdata    = mpy__y [0];
      cdb_r.tag      = delay_pipe_out_r.tag;
      cdb_r.wa       = delay_pipe_out_r.wa;
      cdb_r.robid    = delay_pipe_out_r.robid;

      //
      delay_pipe_in  = '{wa:iss.wa, tag:iss.tag, robid:iss.robid};

    end // block: cntrl_PROC
  
  // ======================================================================== //
  //                                                                          //
  // Instances                                                                //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  delay_pipe #(.W(DELAY_PIPE_W), .N(LATENCY_N)) u_delay_pipe (
    //
      .clk               (clk                )
    , .rst               (rst                )
    //
    , .in                (delay_pipe_in      )
    , .out_r             (delay_pipe_out_r   )
  );
  
  // ------------------------------------------------------------------------ //
  //
  mlt5c #(.W(WORD_W)) u_mpy (
    //
      .clk               (clk                )
    , .rst               (rst                )
    //
    , .a                 (mpy__a             )
    , .b                 (mpy__b             )
    , .pass              (mpy__pass          )
    //
    , .y                 (mpy__y             )
    , .y_vld_r           (mpy__y_vld_r       )
    //
    , .busy_r            ()
  );

endmodule // tomasulo_exe_logic
