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

`include "rmw_long_latency_pkg.vh"

module rmw_long_latency_cache (

   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

   //
     input                                   clk
   , input                                   rst

   //======================================================================== //
   //                                                                         //
   // Issue                                                                   //
   //                                                                         //
   //======================================================================== //

   //
   , input                                   iss_vld_r
   , input rmw_long_latency_pkg::issue_t     iss_r
   //
   , output logic                            iss_rdy_w

   //======================================================================== //
   //                                                                         //
   // Completion                                                              //
   //                                                                         //
   //======================================================================== //

   //
   , output                                  cmpl_vld_r
   , input rmw_long_latency_pkg::word_t      cmpl_word_r

   //======================================================================== //
   //                                                                         //
   // Lookup                                                                  //
   //                                                                         //
   //======================================================================== //

   //
   , output logic                             tbl_wr_r
   , output rmw_long_latency_pkg::id_t        tbl_wr_id_r
   , output rmw_long_latency_pkg::word_t      tbl_wr_word_r
   //
   , input  logic                             tbl_rd_word_vld_r
   , input  rmw_long_latency_pkg::word_t      tbl_rd_word_r
   , input  rmw_long_latency_pkg::tag_t       tbl_rd_ctag_r
   //
   , output logic                             tbl_rd_r
   , output rmw_long_latency_pkg::id_t        tbl_rd_id_r
   , output rmw_long_latency_pkg::tag_t       tbl_rd_itag_r

   //======================================================================== //
   //                                                                         //
   // Exe                                                                     //
   //                                                                         //
   //======================================================================== //

   //
   , output logic                            exe_iss_vld_r
   , output rmw_long_latency_pkg::word_t     exe_iss_imm_r
   , output rmw_long_latency_pkg::op_t       exe_iss_op_r
   , output rmw_long_latency_pkg::word_t     exe_iss_reg_r

   //
   , input logic                             exe_wrbk_w
   , input rmw_long_latency_pkg::word_t      exe_wrbk_word_w
);
  import rmw_long_latency_pkg::*;

  typedef enum logic [1:0] {  OP_RDY           = 2'b00,
                              OP_AWAIT_BYPASS  = 2'b01,
                              OP_AWAIT_TAG     = 2'b10
                            } state_t;
  
  
  //
  typedef struct packed {
    state_t                   state;
    
    //
    issue_t                   issue;
    //
    logic                     tag_vld;
    tag_t                     tag;
    //
    ptr_t                     bypass_ptr;
    //
    word_t                    word;
  } table_t;

  //
  table_t                               table_issue;
  //
  table_t [N - 1:0]                     table_r;
  table_t [N - 1:0]                     table_w;
  logic [N - 1:0]                       table_en;
  //
  table_t                               table_alloc;
  table_t                               table_iss;
  table_t                               table_cmpl;
  //
  logic [N - 1:0]                       table_issue_vld;
  logic [N - 1:0]                       table_rd_ctag_vld;
  logic [N - 1:0]                       table_bypass_vld;
  logic [N - 1:0]                       table_wrbk_vld;
  //
  logic [N - 1:0]                       table_vld_r;
  logic [N - 1:0]                       table_vld_w;

  //
  ptr_t                                 alloc_ptr_r;
  ptr_t                                 alloc_ptr_w;
  logic                                 alloc_ptr_en;
  //
  ptr_t                                 iss_ptr_r;
  ptr_t                                 iss_ptr_w;
  logic                                 iss_ptr_en;
  //
  ptr_t                                 cmpl_ptr_r;
  ptr_t                                 cmpl_ptr_w;
  logic                                 cmpl_ptr_en;
  //
  ptr_t                                 retire_ptr_r;
  ptr_t                                 retire_ptr_w;
  logic                                 retire_ptr_en;
  //
  logic                                 full_w;
  logic                                 full_r;
  //
  logic                                 exe_wrbk_r;
  word_t                                exe_wrbk_word_r;
  //
  logic                                 exe_bypass;
  //
  logic                                 exe_iss_vld_w;
  word_t                                exe_iss_imm_w;
  op_t                                  exe_iss_op_w;
  word_t                                exe_iss_reg_w;

  // ======================================================================== //
  //                                                                          //
  // Combinatorial Logic                                                      //
  //                                                                          //
  // ======================================================================== //
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : table_ptr_PROC

      //
      table_alloc  = table_r [alloc_ptr_r];
      table_iss    = table_r [iss_ptr_r];
      table_cmpl   = table_r [cmpl_ptr_r];

    end // block: table_ptr_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : table_PROC

      //
      iss_rdy_w          = (~full_r);

      //
      table_issue        = '0;
      table_issue.issue  = iss_r;

      //
      table_en           = '0;
      table_rd_ctag_vld  = '0;
      table_bypass_vld   = '0;
      
      for (int i = 0; i < N; i++) begin

        //
        casez ({table_vld_r [i], iss_vld_r, iss_rdy_w})
          3'b0_11: table_issue_vld [i]  = (alloc_ptr_r == ptr_t'(i));
          default: table_issue_vld [i]  = 'b0;
        endcase // casez ({})

        //
        casez ({table_vld_r [i], tbl_rd_word_vld_r})
          2'b1_1:  table_rd_ctag_vld [i]  = (table_r [i].tag == tbl_rd_ctag_r);
          default: table_rd_ctag_vld [i]  = 'b0;
        endcase // casez ({})
        
        //
        casez ({table_vld_r [i], exe_wrbk_r})
          2'b1_1:  table_bypass_vld [i]  = (table_r [i].bypass_ptr == cmpl_ptr_r);
          default: table_bypass_vld [i]  = 'b0;
        endcase // casez ({})

        //
        table_w [i]  = table_r [i];
        
        //
        casez ({table_issue_vld [i], table_rd_ctag_vld [i], table_bypass_vld [i]})
          3'b1??: begin
            table_w [i].state  = 1'b0 ? OP_AWAIT_BYPASS : OP_AWAIT_TAG;
          end
          3'b01?: begin
            table_w [i].state  = OP_RDY;
            table_w [i].word   = tbl_rd_word_r;
          end
          3'b001: begin
            table_w [i].state  = OP_RDY;
            table_w [i].word   = exe_wrbk_word_r;
          end
          default:;
        endcase // casez ({iss_vld_r, iss_rdy_w, table_rd_ctag_vld [i], table_bypass_vld [i]})
        
        //
        table_en [i]  = (table_rd_ctag_vld [i] | table_bypass_vld [i]);

      end // for (int i = 0; i < N; i++)

    end // block: table_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : ptr_PROC

      //
      alloc_ptr_w    = 'b0 ? (alloc_ptr_r + 'b1) : alloc_ptr_r;
      alloc_ptr_en   = '0;

      //
      iss_ptr_w      = 'b0 ? (iss_ptr_r + 'b1) : iss_ptr_r;
      iss_ptr_en     = '0;
      
      //
      cmpl_ptr_w     = 'b0 ? (cmpl_ptr_r + 'b1) : cmpl_ptr_r;
      cmpl_ptr_en    = '0;
      
      //
      retire_ptr_w   = 'b0 ? (retire_ptr_r + 'b1) : retire_ptr_r;
      retire_ptr_en  = '0;

    end // block: ptr_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : bypass_PROC

      //
      exe_bypass     = 'b0;

      //
      exe_iss_vld_w  = '0;
      exe_iss_imm_w  = table_iss.issue.imm;
      exe_iss_op_w   = table_iss.issue.op;
      exe_iss_reg_w  = exe_bypass ? exe_wrbk_word_w : table_iss.word;

    end // block: bypass_PROC
  
  // ======================================================================== //
  //                                                                          //
  // Sequential Logic                                                         //
  //                                                                          //
  // ======================================================================== //
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      exe_wrbk_r <= '0;
    else
      exe_wrbk_r <= exe_wrbk_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (exe_wrbk_w)
      exe_wrbk_word_r <= exe_wrbk_word_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      alloc_ptr_r <= '0;
    else if (alloc_ptr_en)
      alloc_ptr_r <= alloc_ptr_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      iss_ptr_r <= '0;
    else if (iss_ptr_en)
      iss_ptr_r <= iss_ptr_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      cmpl_ptr_r <= '0;
    else if (cmpl_ptr_en)
      cmpl_ptr_r <= cmpl_ptr_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      retire_ptr_r <= '0;
    else if (retire_ptr_en)
      retire_ptr_r <= retire_ptr_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      full_r <= 'b0;
    else
      full_r <= full_w;
      
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      table_vld_r <= '0;
    else
      table_vld_r <= table_vld_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    for (int i = 0; i < N; i++)
      if (table_en [i])
        table_r [i] <= table_w [i];
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      exe_iss_vld_r <= 'b0;
    else
      exe_iss_vld_r <= exe_iss_vld_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (exe_iss_vld_w) begin
      exe_iss_imm_r <= exe_iss_imm_w;
      exe_iss_op_r  <= exe_iss_op_w;
      exe_iss_reg_r <= exe_iss_reg_w;
    end
  

endmodule
