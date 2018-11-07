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

module tomasulo_rs #(parameter int N = 4, parameter int LATENCY_N = 2) (

   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

     input                                        clk
   , input                                        rst

   //======================================================================== //
   //                                                                         //
   // Scheduler                                                               //
   //                                                                         //
   //======================================================================== //

   //
   , input tomasulo_pkg::sch_t                    sch_r

   //======================================================================== //
   //                                                                         //
   // CDB                                                                     //
   //                                                                         //
   //======================================================================== //

   , input tomasulo_pkg::cdb_t                    cdb_r

   //
   , input  logic                                 cdb_gnt
   //
   , output logic                                 cdb_req

   //======================================================================== //
   //                                                                         //
   // Status                                                                  //
   //                                                                         //
   //======================================================================== //

   , output logic                                 full_r

   //======================================================================== //
   //                                                                         //
   // Dispatch                                                                //
   //                                                                         //
   //======================================================================== //

   , input                                        dis_vld_r
   , input tomasulo_pkg::dispatch_t               dis_r

   //======================================================================== //
   //                                                                         //
   // Issue                                                                   //
   //                                                                         //
   //======================================================================== //

   , output logic                                 iss_vld_r
   , output tomasulo_pkg::issue_t                 iss_r
);
  import tomasulo_pkg::*;

  //
  typedef logic [N - 1:0]               n_d_t;
  typedef logic [$clog2(N) - 1:0]       n_t;

  //
  typedef struct packed {
    opcode_t       opcode;
    oprand_t [1:0] oprand;
    tag_t          tag;
    imm_t          imm;
    robid_t        robid;
  } rs_entry_t;

  //
  rs_entry_t [N - 1:0]             rs_entry_r;
  rs_entry_t [N - 1:0]             rs_entry_w;
  logic      [N - 1:0]             rs_entry_en;
  //
  n_d_t                            rs_vld_r;
  n_d_t                            rs_vld_w;
  logic                            rs_vld_en;
  //
  n_d_t                            rs_vld_set;
  n_d_t                            rs_vld_clr;
  //
  n_d_t                            rs_rdy_r;
  n_d_t                            rs_rdy_w;
  logic                            rs_rdy_en;
  //
  n_d_t                            rs_rdy_set;
  n_d_t                            rs_rdy_clr;
  //
  logic                            full_w;
  //
  n_d_t                            rdy_sel;
  n_t                              rdy_idx;
  //
  n_d_t                            vld_sel;
  n_t                              vld_idx;
  //
  n_d_t                            nvld_sel;
  n_t                              nvld_idx;
  //
  n_d_t                            cdb_req_n;
  //
  n_d_t                            iss_vld_d_w;
  issue_t [N-1:0]                  iss_d_w;
  //
  logic                            iss_vld_w;
  issue_t                          iss_w;

  // ======================================================================== //
  //                                                                          //
  // Combinational Logic                                                      //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  function n_d_t ffs(n_d_t in); begin
    ffs  = '0;
    for (int i = $bits(n_d_t) - 1; i >= 0; i--)
      if (in [i])
        ffs  = ('b1 << i);
  end endfunction

  //
  function n_t encode(n_d_t in); begin
    encode  = '0;
    for (int i = $bits(n_d_t) - 1; i >= 0; i--)
      if (in [i])
        encode  = n_t'(i);
  end endfunction
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : sel_PROC

      //
      rdy_sel   = ffs(rs_rdy_r);
      rdy_idx   = encode(rdy_sel);

      //
      vld_sel   = ffs(rs_vld_r);
      vld_idx   = encode(vld_sel);

      //
      nvld_sel  = ffs(~rs_vld_r);
      nvld_idx  = encode(nvld_sel);
      
    end // block: sel_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : rs_dispatch_PROC

      //
      rs_vld_set   = '0;
      rs_vld_clr   = '0;

      //
      cdb_req_n    = '0;

      //
      rs_rdy_set   = '0;
      rs_rdy_clr   = '0;

      //
      iss_vld_d_w  = '0;
      iss_d_w      = '0;

      //
      for (int i = 0; i < N; i++) begin
        logic [1:0] capture_cdb;

        //
        rs_entry_t rs  = rs_entry_r [i];

        for (int oprnd = 0; oprnd < 2; oprnd++) begin
          oprand_t o           = rs.oprand [oprnd];

          capture_cdb [oprnd]  = cdb_r.vld & o.busy & (o.u.t.tag == cdb_r.tag);
        end

        //
        rs_entry_w [i]   = rs_entry_r [i];
        rs_entry_en [i]  = 'b0;

        //
        casez ({rs_vld_r [i], rs_rdy_r [i], dis_vld_r, nvld_sel[i], cdb_r.vld, rdy_sel [i]})

          // Current RS-entry is inactive and is selected for allocation to
          // incoming dispatched instruction.
          //
          6'b0011??: begin
            rs_vld_set [i]         = 'b1;
            
            //
            rs_entry_en [i]        = 'b1;
            rs_entry_w [i].opcode  = dis_r.opcode;
            rs_entry_w [i].oprand  = dis_r.oprand;
            rs_entry_w [i].tag     = dis_r.tag;
            rs_entry_w [i].robid   = dis_r.robid;
            rs_entry_w [i].imm     = dis_r.imm;
          end // case: 5'b0011?

          // Current RS-entry is active, but not ready. Snoop inbound
          // traffic on CDB for writes to the same tag as those busy
          // oprands in the station.
          //
          6'b10??1?: begin
            rs_entry_en [i]  = (|capture_cdb);

            for (int o = 0; o < 2; o++) begin
              if (capture_cdb [o]) begin
                rs_entry_w [i].oprand [o].busy  = 'b0;
                rs_entry_w [i].oprand [o].u.w   = cdb_r.wdata;
              end
            end

            // If neither busy bits are set at the end of this round,
            // the station entry becomes ready for issue.
            //
            rs_rdy_set [i]  = ~(rs_entry_w [i].oprand [0].busy |
                                rs_entry_w [i].oprand [1].busy);
          end // case: 5'b10??1

          // The current station entry is valid, ready and is
          // scheduled for issue.
          //
          6'b1_1_?_?_?_1: begin
            cdb_req_n [i]   = (~sch_r [LATENCY_N]);

            if (cdb_gnt) begin
              rs_vld_clr [i]     = cdb_gnt;

              iss_vld_d_w [i]    = 'b1;
              for (int o = 0; o < 2; o++)
                iss_d_w [i].rdata [o]  = rs_entry_r [i].oprand [o].u.w;
              iss_d_w [i].op           = rs_entry_r [i].opcode;
              iss_d_w [i].tag          = rs_entry_r [i].tag;
              iss_d_w [i].imm          = rs_entry_r [i].imm;
              iss_d_w [i].robid        = rs_entry_r [i].robid;
            end
            
          end // case: 6'b1_1_?_?_?_1

          default:;
          
        endcase // casez ({rs_vld_r [i]})

      end // for (int i = 0; i < N; i++)

      //
      rs_vld_w   = rs_vld_set | rs_vld_r & (~rs_vld_clr);
      rs_vld_en  = |{rs_vld_set, rs_vld_clr};

      //
      cdb_req    = (|cdb_req_n);

      //
      rs_rdy_w   = rs_rdy_set | rs_rdy_r & (~rs_rdy_clr);
      rs_rdy_en  = |{rs_rdy_set, rs_rdy_clr};

      //
      full_w     = (rs_vld_w == '1);

      //
      iss_vld_w  = (|iss_vld_d_w);

      //
      iss_w      = '0;
      for (int i = 0; i < N; i++)
        iss_w |= iss_d_w [i];
      
    end // block: rs_dispatch_PROC
  
  // ======================================================================== //
  //                                                                          //
  // Flops                                                                    //
  //                                                                          //
  // ======================================================================== //
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    for (int i = 0; i < N; i++)
      if (rs_entry_en [i])
        rs_entry_r [i] <= rs_entry_w [i];
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      rs_vld_r <= '0;
    else if (rs_vld_en)
      rs_vld_r <= rs_vld_w;
      
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      rs_rdy_r <= '0;
    else if (rs_rdy_en)
      rs_rdy_r <= rs_rdy_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      full_r <= '0;
    else
      full_r <= full_w;

  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      iss_vld_r <= 'b0;
    else
      iss_vld_r <= iss_vld_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (iss_vld_w)
      iss_r <= iss_w;
  
endmodule // tomasulo_rs

  
