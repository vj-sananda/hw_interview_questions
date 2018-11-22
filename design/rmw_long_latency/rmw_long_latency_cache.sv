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
    logic                     vld;
    state_t                   state;
    issue_t                   issue;
    tag_t                     tag;
    ptr_t                     byptr;
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
  table_t                               table_retire;
  //
  logic [N - 1:0]                       table_issue_vld;
  logic [N - 1:0]                       table_rd_ctag_vld;
  logic [N - 1:0]                       table_bypass_vld;
  logic [N - 1:0]                       table_retire_vld;

  //
  ptr_t                                 alloc_ptr_r;
  ptr_t                                 alloc_ptr_w;
  logic                                 alloc_ptr_en;
  //
  n_t                                   alloc_ptr_d;
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
  logic                                 alloc_adv;
  logic                                 iss_adv;
  logic                                 cmpl_adv;
  logic                                 retire_adv;
  //
  logic                                 full_w;
  logic                                 full_r;
  //
  logic                                 exe_wrbk_r;
  logic                                 exe_wrbk_bypass_r;
  word_t                                exe_wrbk_word_r;
  //
  logic                                 exe_wrbk_bypass_w;
  //
  logic                                 exe_iss_vld_w;
  word_t                                exe_iss_imm_w;
  op_t                                  exe_iss_op_w;
  word_t                                exe_iss_reg_w;
  //
  logic                                 tbl_wr_w;
  id_t                                  tbl_wr_id_w;
  word_t                                tbl_wr_word_w;
  //
  logic                                 tbl_bypass;
  ptr_t                                 tbl_bypass_ptr;
  //
  logic                                 tbl_rd_w;
  id_t                                  tbl_rd_id_w;
  tag_t                                 tbl_rd_itag_w;
  //
  logic                                 tbl_flm__alloc_vld;
  tag_t                                 tbl_flm__alloc_id;
  logic                                 tbl_flm__free_vld;
  tag_t                                 tbl_flm__free_id;
  logic                                 tbl_flm__clear;
  logic                                 tbl_flm__idle_r;
  logic                                 tbl_flm__busy_r;
  logic [IN_FLIGHT_N - 1:0]             tbl_flm__state_w;
  logic [IN_FLIGHT_N - 1:0]             tbl_flm__state_r;
  //
  tag_t                                 iss_tag;

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
      table_alloc   = table_r [alloc_ptr_r.p];
      table_iss     = table_r [iss_ptr_r.p];
      table_cmpl    = table_r [cmpl_ptr_r.p];
      table_retire  = table_r [retire_ptr_r.p];

      //
      iss_tag       = clz_tag(tbl_flm__state_r);

    end // block: table_ptr_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : table_PROC

      //
      table_issue        = '0;
      table_issue.issue  = iss_r;

      //
      table_en           = '0;
      table_rd_ctag_vld  = '0;
      table_bypass_vld   = '0;
      
      for (int i = 0; i < N; i++) begin

        //
        casez ({table_r [i].vld, iss_vld_r, iss_rdy_w, alloc_ptr_d [i]})
          4'b0_111: table_issue_vld [i]  = 'b1;
          default:  table_issue_vld [i]  = 'b0;
        endcase // casez ({})

        //
        casez ({table_r [i].vld, tbl_rd_word_vld_r})
          2'b1_1:  table_rd_ctag_vld [i]  = (table_r [i].tag == tbl_rd_ctag_r);
          default: table_rd_ctag_vld [i]  = 'b0;
        endcase // casez ({})
        
        //
        casez ({table_r [i].vld, exe_wrbk_r})
          2'b1_1:  table_bypass_vld [i]  = (table_r [i].byptr == cmpl_ptr_r);
          default: table_bypass_vld [i]  = 'b0;
        endcase // casez ({})

        //
        table_retire_vld [i] = '0;

        //
        table_w [i]       = table_r [i];
        
        //
        unique0 case (1'b1)
          table_issue_vld [i]: begin
            table_w [i].vld      = 'b1;
            table_w [i].state    = tbl_rd_w ? OP_AWAIT_BYPASS : OP_AWAIT_TAG;
            table_w [i].tag      = iss_tag;
            table_w [i].byptr    = tbl_bypass_ptr;
          end
          table_rd_ctag_vld [i]: begin
            table_w [i].state  = OP_RDY;
            table_w [i].word   = tbl_rd_word_r;
          end
          table_bypass_vld [i]: begin
            table_w [i].state  = OP_RDY;
            table_w [i].word   = exe_wrbk_word_r;
          end
          table_retire_vld [i]: begin
            table_w [i].vld  = 'b0;
          end
        endcase // unique0 case (1'b1)
        
        //
        table_en [i]  = (table_rd_ctag_vld [i] | table_bypass_vld [i]);

      end // for (int i = 0; i < N; i++)

    end // block: table_PROC
  
  // ------------------------------------------------------------------------ //
  //
  function ptr_t inc(ptr_t p); begin
    inc  = p;
    if (p == ptr_t'(N - 1)) begin
      inc.x  = ~inc.x;
      inc.p  = '0;
    end else begin
      inc.p  = p.p + 'b1;
    end
  end endfunction

  //
  function logic is_full (ptr_t a, ptr_t b); begin
    is_full  = (a.x ^ b.x) & (a.p == b.p);
  end endfunction

  //
  function n_t lr (n_t n, ptr_t p); begin
    n_t [1:0] n_extended  = ({n, n} >> p);
    lr                    = n_extended[0];
  end endfunction

  //
  function n_t table_match_on_id (id_t id); begin
    for (int i = 0; i < N; i++)
      table_vld [i]  = table_r [i].vld & (table_r [i].issue.id == id);
  end endfunction

  //
  always_comb
    begin : ptr_PROC

      //
      iss_rdy_w       = (~full_r);

      //
      alloc_adv       = iss_vld_r & iss_rdy_w;
      iss_adv         = '0;
      cmpl_adv        = '0;
      retire_adv      = '0;
      
      //
      alloc_ptr_w     = alloc_adv ? inc(alloc_ptr_r) : alloc_ptr_r;
      alloc_ptr_en    = alloc_adv;

      //
      iss_ptr_w       = iss_adv ? inc(iss_ptr_r) : iss_ptr_r;
      iss_ptr_en      = iss_adv;
      
      //
      cmpl_ptr_w      = cmpl_adv ? inc(cmpl_ptr_r) : cmpl_ptr_r;
      cmpl_ptr_en     = cmpl_adv;
      
      //
      retire_ptr_w    = retire_adv ? inc(retire_ptr_r) : retire_ptr_r;
      retire_ptr_en   = retire_adv;

      //
      alloc_ptr_d     = ('b1 << alloc_ptr_r);
      
      //
      full_w          = is_full(alloc_ptr_w, retire_ptr_w);

      //
      tbl_wr_w        = 'b0;
      tbl_wr_id_w     = '0;
      tbl_wr_word_w   = '0;

      //
      tbl_bypass      = '0;
      tbl_bypass_ptr  = '0;

      //
      tbl_rd_w        = alloc_adv & op_requires_tbl(iss_r.op) & (~tbl_bypass);
      tbl_rd_id_w     = iss_r.id;
      tbl_rd_itag_w   = iss_tag;

    end // block: ptr_PROC

  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : bypass_PROC

      //
      exe_wrbk_bypass_w  = '0;

      //
      exe_iss_vld_w      = '0;
      exe_iss_imm_w      = table_iss.issue.imm;
      exe_iss_op_w       = table_iss.issue.op;
      exe_iss_reg_w      = exe_wrbk_bypass_w ? exe_wrbk_word_w : table_iss.word;

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
    if (exe_wrbk_w) begin
      exe_wrbk_bypass_r <= exe_wrbk_bypass_w;
      exe_wrbk_word_r   <= exe_wrbk_word_w;
    end
  
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
  always_ff @(posedge clk) begin
    for (int i = 0; i < N; i++)
      if (rst)
        table_r [i].vld <= 'b0;
      else if (table_en [i])
        table_r [i] <= table_w [i];
  end
  
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
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      tbl_wr_r <= 'b0;
    else
      tbl_wr_r <= tbl_wr_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (tbl_wr_w) begin
      tbl_wr_id_r   <= tbl_wr_id_w;
      tbl_wr_word_r <= tbl_wr_word_w;
    end

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      tbl_rd_r <= 'b0;
    else
      tbl_rd_r <= tbl_rd_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (tbl_rd_w) begin
      tbl_rd_id_r   <= tbl_rd_id_w;
      tbl_rd_itag_r <= tbl_rd_itag_w;
    end
  
  // ======================================================================== //
  //                                                                          //
  // Instantiations                                                           //
  //                                                                          //
  // ======================================================================== //
  
  // ------------------------------------------------------------------------ //
  //
  flm #(.N(IN_FLIGHT_N)) u_tbl_flm (
    //
      .clk               (clk                )
    , .rst               (rst                )
    //
    , .alloc_vld         (tbl_flm__alloc_vld )
    , .alloc_id          (tbl_flm__alloc_id  )
    //
    , .free_vld          (tbl_flm__free_vld  )
    , .free_id           (tbl_flm__free_id   )
    //
    , .clear             (tbl_flm__clear     )
    //
    , .idle_r            (tbl_flm__idle_r    )
    , .busy_r            (tbl_flm__busy_r    )
    //
    , .state_w           (tbl_flm__state_w   )
    , .state_r           (tbl_flm__state_r   )
  );
  
endmodule
