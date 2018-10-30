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

`include "read_modify_write_pkg.vh"

module read_modify_write (

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

   , input                                        in_pass
   , input read_modify_write_pkg::inst_t          in_inst
   , input read_modify_write_pkg::imm_t           in_imm
   //
   , output logic                                 in_accept

   //======================================================================== //
   //                                                                         //
   // Out                                                                     //
   //                                                                         //
   //======================================================================== //

   , output                                       out_vld_r
   , output read_modify_write_pkg::reg_t          out_wa_r
   , output read_modify_write_pkg::word_t         out_wdata_r

   //======================================================================== //
   //                                                                         //
   // Cntrl                                                                   //
   //                                                                         //
   //======================================================================== //

   , input  logic                                 replay_s4_req
   , input  logic                                 replay_s8_req
   , input  logic   [9:0]                         stall_req
);
  import read_modify_write_pkg::*;

  typedef struct packed {
    logic        o;
    logic [2:0]  p;
  } ptr_t;

  typedef struct packed {
    inst_t inst;
    imm_t  imm;
  } fifo_word_t;

  typedef struct packed {
    word_t [1:0] rdata;
    word_t wdata;
    inst_t inst;
    imm_t  imm;
    ptr_t  ptr;
    logic [1:0]  wrbk_vld;
    word_t wrbk_dat;
  } ucode_t;

  //

  //
  logic [9:1]                           vld_r;
  logic [9:1]                           vld_w;

  //
  ucode_t [9:1]                         ucode_r;
  ucode_t [9:1]                         ucode_w;

  //
  logic [9:0]                           adv;
  logic [9:1]                           en;
  logic [9:0]                           stall;
  logic [9:0]                           kill;

  //
  logic                                 commit_s8;
  logic                                 replay_s4_w;
  logic                                 replay_s8_w;
  logic                                 replay_s4_r;
  logic                                 replay_s8_r;
  logic                                 replay_s4;
  logic                                 replay_s8;

  //
  ptr_t                                 rd_ptr_arch_r;
  ptr_t                                 rd_ptr_arch_w;
  logic                                 rd_ptr_arch_en;
  //
  ptr_t                                 rd_ptr_spec_r;
  ptr_t                                 rd_ptr_spec_w;
  logic                                 rd_ptr_spec_en;
  //
  ptr_t                                 wr_ptr_r;
  ptr_t                                 wr_ptr_w;
  logic                                 wr_ptr_en;
  //
  logic                                 empty_w;
  logic                                 empty_r;
  logic                                 full_w;
  logic                                 full_r;
  //
  fifo_word_t [7:0]                     fifo_r;
  logic                                 fifo_en;
  //
  logic [1:0]                           rf__ren;
  reg_t [1:0]                           rf__ra;
  word_t [1:0]                          rf__rdata;
  //
  logic                                 rf__wen;
  reg_t                                 rf__wa;
  word_t                                rf__wdata;
  //
  word_t [1:0]                          fwd__s3_rdata;
  word_t [1:0]                          fwd__s4_rdata;
  word_t [1:0]                          fwd__s5_rdata;
  word_t [1:0]                          fwd__s6_rdata;
  
  //
  function word_t exe(opcode_t op, word_t [1:0] r); begin
    exe  = '0;
    case (op)
      OP_AND: exe   = (r[0] & r[1]);
      OP_NOT: exe   = ~r[0];
      OP_OR:  exe   = (r[0] | r[1]);
      OP_XOR: exe   = (r[0] ^ r[1]);
      OP_ADD: exe   = r[0] + r[1];
      OP_SUB: exe   = r[0] - r[1];
      OP_MOV0: exe  = r[0];
      OP_MOV1: exe  = r[1];
      default: exe  = r[0];
    endcase // case (op)
  end endfunction

  // ======================================================================== //
  //                                                                          //
  // Combinatorial Logic                                                      //
  //                                                                          //
  // ======================================================================== //
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : front_end_PROC

      //
      rd_ptr_arch_en = commit_s8;
      rd_ptr_arch_w  = commit_s8 ? (rd_ptr_arch_r + 'b1) : rd_ptr_arch_r;

      //
      rd_ptr_spec_en = (replay_s4 | replay_s8 | adv [0] );

      //
      casez ({replay_s8, replay_s4, adv [0]})
        3'b1??:  rd_ptr_spec_w  = ucode_r [8].ptr;
        3'b01?:  rd_ptr_spec_w  = ucode_r [4].ptr;
        3'b001:  rd_ptr_spec_w  = rd_ptr_spec_r + 'b1;
        default: rd_ptr_spec_w  = rd_ptr_spec_r;
      endcase // casez ({replay_s8_r, adv [0]})
      
      //
//      fifo_en   = in_vld & in_accept;

      //
      wr_ptr_en = fifo_en;
      wr_ptr_w  = (fifo_en) ? wr_ptr_r + 'b1 : wr_ptr_r;

      //
      empty_w  = (rd_ptr_spec_w == wr_ptr_w);
      full_w   = (rd_ptr_arch_w.o  ^ wr_ptr_w.o) &
                 (rd_ptr_arch_w.p == wr_ptr_w.p);

    end // block: front_end_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : pipe_PROC

      // Replays are injected synthetically by the TB. For correct
      // operation the restart address must be valid, therefore the
      // replay request is held until the associated stage becomes
      // valid, when it is then applied.
      //
      casez ({replay_s4_req, replay_s4_r, vld_r [4]})
        3'b1_0_?: replay_s4_w  = 1'b1;
        3'b?_1_1: replay_s4_w  = 1'b0;
        default:  replay_s4_w  = replay_s4_r;
      endcase // casez ({replay_s4_req, replay_s4_r, vld_r [4]})

      //
      replay_s4  = replay_s4_r & vld_r [4];

      //
      casez ({replay_s8_req, replay_s8_r, vld_r [8]})
        3'b1_0_?: replay_s8_w  = 1'b1;
        3'b?_1_1: replay_s8_w  = 1'b0;
        default:  replay_s8_w  = replay_s8_r;
      endcase // casez ({replay_s8_req, replay_s8_r, vld_r [8]})

      //
      replay_s8  = replay_s8_r & vld_r [8];

      // Kill stages on upstream replay.
      //
      kill       = '0;
      for (int i = 0; i < 10; i++) begin
        kill [i]  |= (i <= 8) ? replay_s8 : '0;
        kill [i]  |= (i <= 4) ? replay_s4 : '0;
      end

      // The final (output) stage cannot be stalled (for DV simplicity).
      //
      stall [9]    = 1'b0;

      // TEMP: Verilator warning incorrect?

      /* verilator lint_off ALWCOMBORDER */
      for (int i = 8; i >= 0; i--)
        stall [i]  = ((i == 0) ? 1'b1 : vld_r [i]) & (stall_req [i] | stall [i + 1]);
      /* verilator lint_on ALWCOMBORDER */
                  
                  //
      in_accept    = (~full_r);

      //
      for (int i = 0; i < 10; i++)
        adv [i]  = ((i == 0) ? (~empty_r) : vld_r [i]) & (~stall [i]) & (~kill [i]);
                  
      //
      for (int i = 1; i < 10; i++)
        en [i]    = adv [i - 1];
      
      //
      for (int i = 1; i < 10; i++) begin
        
        casez ({kill [i], stall [i]})
          2'b1?:   vld_w [i]  = 'b0;
          2'b00:   vld_w [i]  = adv [i - 1];
          default: vld_w [i]  = vld_r [i];
        endcase // casez ({kill [i], stall [i]})

      end // for (int i = 1; i < N; i++)

      // Commit point at stage 8. 
      //
      commit_s8    = (~replay_s8_r) & (adv [8]);
                 
      //
      out_vld_r    = vld_r [9];
      out_wa_r     = '0;
      out_wdata_r  = '0;

    end // block: pipe_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : fwd_s4_PROC

      // TODO: bypass writeback

      for (int i = 0; i < 2; i++) begin
        logic [2:0] byp;
        reg_t ra;
        
        //
        ra  = ucode_r [4].inst.ra [i];

        //
        byp [0]  = vld_r [7] & (ra == ucode_r [7].inst.wa);
        byp [1]  = vld_r [8] & (ra == ucode_r [8].inst.wa);
        byp [2]  = vld_r [9] & (ra == ucode_r [9].inst.wa);
        
        //
        priority casez ({byp, ucode_r [4].wrbk_vld})
          4'b1??_?: fwd__s4_rdata [i]  = ucode_r [8].wdata;
          4'b01?_?: fwd__s4_rdata [i]  = ucode_w [9].wdata;
          4'b001_?: fwd__s4_rdata [i]  = ucode_r [9].wdata;
          4'b000_1: fwd__s4_rdata [i]  = ucode_r [4].wrbk;
          default:  fwd__s4_rdata [i]  = rf__rdata [i];
        endcase

      end // for (int i  = 0; i < 2; i++)

    end // block: fwd_s4_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : fwd_s5_PROC

      for (int i = 0; i < 2; i++) begin
        logic [2:0] byp;
        reg_t ra;
        
        //
        ra  = ucode_r [5].inst.ra [i];

        //
        byp [0]  = vld_r [7] & (ra == ucode_r [7].inst.wa);
        byp [1]  = vld_r [8] & (ra == ucode_r [8].inst.wa);
        byp [2]  = vld_r [9] & (ra == ucode_r [9].inst.wa);
        
        //
        priority casez (byp)
          3'b1??:  fwd__s5_rdata [i]  = ucode_r [8].wdata;
          3'b01?:  fwd__s5_rdata [i]  = ucode_w [9].wdata;
          3'b001:  fwd__s5_rdata [i]  = ucode_r [9].wdata;
          default: fwd__s5_rdata [i]  = ucode_r [5].rdata [i];
        endcase

      end // for (int i  = 0; i < 2; i++)

    end // block: fwd_s5_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : fwd_s6_PROC

      for (int i = 0; i < 2; i++) begin
        logic [2:0] byp;
        reg_t ra;
        
        //
        ra  = ucode_r [6].inst.ra [i];

        //
        byp [0]  = vld_r [7] & (ra == ucode_r [7].inst.wa);
        byp [1]  = vld_r [8] & (ra == ucode_r [8].inst.wa);
        byp [2]  = vld_r [9] & (ra == ucode_r [9].inst.wa);
        
        //
        priority casez (byp)
          3'b1??:  fwd__s6_rdata [i]  = ucode_r [8].wdata;
          3'b01?:  fwd__s6_rdata [i]  = ucode_w [9].wdata;
          3'b001:  fwd__s6_rdata [i]  = ucode_r [9].wdata;
          default: fwd__s6_rdata [i]  = ucode_r [6].rdata [i];
        endcase

      end // for (int i  = 0; i < 2; i++)

    end // block: fwd_s6_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : ucode_PROC

      // S0
      //
      ucode_w [1]           = '0;
      ucode_w [1].inst      = fifo_r [rd_ptr_spec_r].inst;
      ucode_w [1].imm       = fifo_r [rd_ptr_spec_r].imm;
      ucode_w [1].ptr       = rd_ptr_spec_r;

      // S2
      //
      ucode_w [2]           = ucode_r [1];

      // S3
      //
      ucode_w [3]           = ucode_r [2];
      for (int i = 0; i < 2; i++)
        ucode_w [3].wrbk_vld  = rf__wen & (rf__wa == ucode_r [i].inst.ra [i]);
      ucode_w [3].wrbk        = rf__wdata;

      // S4
      //
      ucode_w [4]             = ucode_r [3];
      ucode_w [4].rdata       = fwd__s3_rdata;

      // S5
      //
      ucode_w [5]             = ucode_r [4];
      ucode_w [5].rdata       = fwd__s4_rdata;
      
      // S6
      //
      ucode_w [6]             = ucode_r [5];
      ucode_w [6].rdata       = fwd__s5_rdata;

      // S7
      //
      ucode_w [7]             = ucode_r [6];
      ucode_w [7].rdata       = fwd__s6_rdata;
      ucode_w [7].wdata       = exe(ucode_r [6].inst.op, ucode_w [7].rdata);

      // S8
      //
      ucode_w [8]             = ucode_r [7];

      // S9
      //
      ucode_w [9]             = ucode_r [8];

    end // block: ucode_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : rf_PROC

      //
      for (int i = 0; i < 2; i++) 
        rf__ren [i] = adv [3] & (~ucode_r [3].wrbk_vld [i]);
      
      rf__ra     = ucode_r [3].inst.ra;

      //
      rf__wen    = adv [9];
      rf__wa     = ucode_r [9].inst.wa;
      rf__wdata  = ucode_r [9].wdata;

    end // block: rf_PROC
  
  // ======================================================================== //
  //                                                                          //
  // Sequential Logic                                                         //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      vld_r <= 'b0;
    else
      vld_r <= vld_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    for (int i = 1; i < 10; i++)
      if (en [i])
        ucode_r [i] <= ucode_w [i];

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      replay_s4_r <= '0;
    else
      replay_s4_r <= replay_s4_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      replay_s8_r <= '0;
    else
      replay_s8_r <= replay_s8_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst) 
      rd_ptr_arch_r <= '0;
    else if (rd_ptr_arch_en)
      rd_ptr_arch_r <= rd_ptr_arch_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst) 
      rd_ptr_spec_r <= '0;
    else if (rd_ptr_spec_en)
      rd_ptr_spec_r <= rd_ptr_spec_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst) 
      wr_ptr_r <= '0;
    else if (wr_ptr_en)
      wr_ptr_r <= wr_ptr_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (fifo_en)
      fifo_r [wr_ptr_r] <= '{inst:in_inst, imm:in_imm};

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
      empty_r <= 'b1;
    else
      empty_r <= empty_w;
  
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
  
endmodule // read_modify_write
