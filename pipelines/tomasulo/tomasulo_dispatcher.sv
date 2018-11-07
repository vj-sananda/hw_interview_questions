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
   , input tomasulo_pkg::fifo_inst_t              inst
   , input                                        inst_vld
   //
   , output logic                                 inst_adv

   //======================================================================== //
   //                                                                         //
   // DISPATCH                                                                //
   //                                                                         //
   //======================================================================== //

   //
   , input [4:0]                                  rs_full_r
   //
   , output tomasulo_pkg::dispatch_t              dis_r
   , output [4:0]                                 dis_vld_r
);
  import tomasulo_pkg::*;

  //
  typedef struct packed {
    reg_t          wa;
    word_t         wdata;
  } out_t;
  localparam int OUT_W  = $bits(out_t);

  //
  logic [31:0]                          reg_busy_r;
  logic [31:0]                          reg_busy_w;
  logic                                 reg_busy_en;
  //
  logic                                 flm__alloc_vld;
  logic [4:0]                           flm__alloc_id;
  logic                                 flm__free_vld;
  logic [4:0]                           flm__free_id;
  logic                                 flm__idle_r;
  logic                                 flm__busy_r;
  tag_d_t                               flm__state_r;
  //
  logic                                 dis_en;
  dispatch_t                            dis_w;
  logic [4:0]                           dis_vld_w;
  logic                                 dis_emit;
  //
  logic [1:0]                           rf_reg__ren;
  reg_t [1:0]                           rf_reg__ra;
  word_t [1:0]                          rf_reg__rdata;
  //
  logic                                 rf_reg__wen;
  reg_t                                 rf_reg__wa;
  word_t                                rf_reg__wdata;
  //
  logic [1:0]                           rf_tag__ren;
  reg_t [1:0]                           rf_tag__ra;
  tag_t [1:0]                           rf_tag__rdata;
  //
  logic                                 rf_tag__wen;
  reg_t                                 rf_tag__wa;
  tag_t                                 rf_tag__wdata;
  //
  logic                                 rob__alloc_vld;
  out_t                                 rob__alloc_data;
  logic                                 rob__alloc_rdy;
  robid_t                               rob__alloc_id;
  //
  logic                                 rob__cmpl_vld;
  robid_t                               rob__cmpl_id;
  out_t                                 rob__cmpl_data;
  //
  logic                                 rob__retire_rdy;
  out_t                                 rob__retire_cmpl_data;
  out_t                                 rob__retire_alloc_data;
  logic                                 rob__retire_vld;
  //
  logic                                 rob__clear;
  logic                                 rob__idle_r;
  logic                                 rob__full_r;
  //
  logic                                 out_vld_w;
  reg_t                                 out_wa_w;
  word_t                                out_wdata_w;

  // ======================================================================== //
  //                                                                          //
  // Combinational Logic                                                      //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  function tag_t clz (tag_d_t in); begin
    clz = '0;
    for (int i = $bits(tag_d_t) - 1; i >= 0; i--)
      if (in [i])
        clz  = tag_t'(i);
  end endfunction

  //
  function oprand_t compute_oprand(int i); begin
    logic bypass_cdb;
    
    //
    bypass_cdb          = cdb_r.vld & (inst.ra [i] == cdb_r.wa);

    //
    compute_oprand [i]  = '0;
    casez ({reg_busy_r [inst.ra [i]], bypass_cdb})
      2'b1_0: begin
        dis_w.oprand [i].busy     = 'b1;
        dis_w.oprand [i].u.t.tag  = rf_tag__rdata [i];
      end
      2'b1_1: dis_w.oprand [i].u.w  = cdb_r.wdata;
      2'b0_?: dis_w.oprand [i].u.w  = rf_reg__rdata [i];
    endcase // casez ({compute_oprand [i].busy, bypass_cdb})

  end endfunction

  //
  function logic [4:0] update_dis_vld_w; begin
    logic [4:0] ret   = '0;

    //
    unique0 casez ({//
                    inst_vld, flm__busy_r, rob__full_r,
                    //
                    is_arith(inst.op), rs_full_r [1:0],
                    //
                    is_logic(inst.op), rs_full_r [3:2],
                    //
                    is_mpy(inst.op), rs_full_r [4]
                    })
      //
      11'b100_10?_???_??: ret [0]  = 'b1;
      11'b100_110_???_??: ret [1]  = 'b1;

      //
      11'b100_0??_10?_??: ret [2]  = 'b1;
      11'b100_0??_110_??: ret [3]  = 'b1;

      //
      11'b100_0??_0??_10: ret [4]  = 'b1;
      default:            ret      = '0;
    endcase // unique0 casez ({inst_vld, flm__busy_r...

    //
    update_dis_vld_w  = ret;

  end endfunction

  // ------------------------------------------------------------------------ //

  // Each instruction dispatch is annotated with an additional ID
  // denoting a position in the commit reorder buffer. This buffer,
  // which is not found in the original Tomasulo paper, plays no
  // functional role in the algorithm and is instead used to recover
  // the original instruction ordering before it is passed to the DV
  // environment.
  //
  always_comb
    begin : dispatch_PROC

      //
      dis_vld_w         = update_dis_vld_w();
      dis_emit          = (|dis_vld_w);

      //
      rf_reg__ren       = {2{dis_emit}};
      rf_reg__ra        = inst.ra;

      //
      rf_tag__ren       = {2{dis_emit}};
      rf_tag__ra        = inst.ra;

      //
      flm__alloc_vld    = dis_emit;
      flm__alloc_id     = clz(flm__state_r);

      //
      rob__alloc_vld    = dis_emit;

      //
      dis_w             = '0;
      dis_w.opcode      = inst.op;
      dis_w.tag         = flm__alloc_id;
      dis_w.robid       = rob__alloc_id;
      dis_w.oprand [0]  = compute_oprand(0);
      dis_w.oprand [1]  = compute_oprand(1);
      dis_w.imm         = inst.imm;

      //
      rf_tag__wen       = dis_emit;
      rf_tag__wa        = inst.wa;
      rf_tag__wdata     = dis_w.tag;

    end // block: dispatch_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : reg_busy_PROC

      //
      reg_busy_w  = reg_busy_r;

      //
      if (cdb_r.vld)
        reg_busy_w [cdb_r.wa]  = 'b0;
      if (dis_emit)
        reg_busy_w [inst.wa]  = 'b1;

      //
      reg_busy_en             = (dis_emit | cdb_r.vld);
      
    end // block: reg_busy_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : completion_PROC

      //
      rf_reg__wen     = cdb_r.vld;
      rf_reg__wa      = cdb_r.wa;
      rf_reg__wdata   = cdb_r.wdata;

      //
      flm__free_vld   = cdb_r.vld;
      flm__free_id    = cdb_r.tag;

      //
      rob__cmpl_vld   = cdb_r.vld;
      rob__cmpl_id    = cdb_r.robid;
      rob__cmpl_data  = '{wa:cdb_r.wa, wdata:cdb_r.wdata};
      
    end // block: completion_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : out_PROC

      //
      out_vld_w        = rob__retire_vld;
      out_wa_w         = rob__retire_cmpl_data.wa;
      out_wdata_w      = rob__retire_cmpl_data.wdata;

      //
      rob__retire_rdy  = 'b1;

    end // block: dispatcher_PROC

  // ======================================================================== //
  //                                                                          //
  // Flops                                                                    //
  //                                                                          //
  // ======================================================================== //
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      reg_busy_r <= '0;
    else if (reg_busy_en)
      reg_busy_r <= reg_busy_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (dis_en)
      dis_r <= dis_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst) 
      dis_vld_r <= '0;
    else
      dis_vld_r <= dis_vld_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      out_vld_r <= 'b0;
    else
      out_vld_r <= out_vld_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (out_vld_w) begin
      out_wa_r    <= out_wa_w;
      out_wdata_r <= out_wdata_w;
    end
  
  // ======================================================================== //
  //                                                                          //
  // Instances                                                                //
  //                                                                          //
  // ======================================================================== //
  
  // ------------------------------------------------------------------------ //
  //
  flm #(.N(TAG_D_W)) u_reg_busy_flm (
    //
      .clk               (clk           )
    , .rst               (rst           )
    //
    , .alloc_vld         (flm__alloc_vld )
    , .alloc_id          (flm__alloc_id  )
    //
    , .free_vld          (flm__free_vld  )
    , .free_id           (flm__free_id   )
    //
    , .clear             ()
    //
    , .idle_r            (flm__idle_r    )
    , .busy_r            (flm__busy_r    )
    //
    , .state_w           ()
    , .state_r           (flm__state_r   )
  );

  // ------------------------------------------------------------------------ //
  //
  rf #(.W(WORD_W), .N(32), .WR_N(1), .RD_N(2)) u_rf_reg (
    //
      .clk          (clk               )
    , .rst          (rst               )
    //
    , .ra           (rf_reg__ra        )
    , .ren          (rf_reg__ren       )
    , .rdata        (rf_reg__rdata     )
    //
    , .wa           (rf_reg__wa        )
    , .wen          (rf_reg__wen       )
    , .wdata        (rf_reg__wdata     )
  );

  // ------------------------------------------------------------------------ //
  //
  rf #(.W(TAG_W), .N(32), .WR_N(1), .RD_N(2)) u_rf_tag (
    //
      .clk          (clk               )
    , .rst          (rst               )
    //
    , .ra           (rf_tag__ra        )
    , .ren          (rf_tag__ren       )
    , .rdata        (rf_tag__rdata     )
    //
    , .wa           (rf_tag__wa        )
    , .wen          (rf_tag__wen       )
    , .wdata        (rf_tag__wdata     )
  );

  // ------------------------------------------------------------------------ //
  //
  rob #(.W(OUT_W), .N(ROB_N)) u_rob (
    //
      .clk               (clk                )
    , .rst               (rst                )
    //
    , .alloc_vld         (rob__alloc_vld     )
    , .alloc_data        (rob__alloc_data    )
    //
    , .alloc_rdy         (rob__alloc_rdy     )
    , .alloc_id          (rob__alloc_id      )
    //
    , .cmpl_vld          (rob__cmpl_vld      )
    , .cmpl_id           (rob__cmpl_id       )
    , .cmpl_data         (rob__cmpl_data     )
    //
    , .retire_rdy        (rob__retire_rdy    )
    , .retire_cmpl_data  (rob__retire_cmpl_data)
    , .retire_alloc_data (rob__retire_alloc_data)
    , .retire_vld        (rob__retire_vld    )
    //
    , .clear             (rob__clear         )
    , .idle_r            (rob__idle_r        )
    , .full_r            (rob__full_r        )
  );

endmodule
