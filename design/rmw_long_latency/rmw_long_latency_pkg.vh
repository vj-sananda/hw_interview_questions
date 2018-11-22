`ifndef __RMW_LONG_LATENCY_PKG_VH__
`define __RMW_LONG_LATENCY_PKG_VH__

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

package rmw_long_latency_pkg;

  //
  typedef logic [15:0] id_t;
  typedef logic [31:0] word_t;

  //
  typedef enum logic [1:0] {  OP_NOP   = 2'b00,
                              OP_ADDI  = 2'b01,
                              OP_SUBI  = 2'b10,
                              OP_MOVI  = 2'b11
                            } op_t;

  //
  function logic op_requires_tbl(op_t op); begin
    op_requires_tbl  = ((op == OP_ADDI) || (op == OP_SUBI));
  end endfunction

  //
  typedef struct packed {
    id_t id;
    word_t imm;
    op_t op;
  } issue_t;

  // The number of commands that may be enqueued. Typically around the
  // lookup latency.
  //
  localparam int N  = 64;

  typedef logic [N - 1:0] n_t;

  // The maximum number of concurrent in flight commands to TBL.
  //
  localparam int IN_FLIGHT_N  = 16;
  localparam int TAG_W  = $clog2(IN_FLIGHT_N);
  typedef logic [TAG_W-1:0] tag_t;

  function tag_t clz_tag(logic [IN_FLIGHT_N - 1:0] n); begin
    clz_tag = '0;
    for (int i = IN_FLIGHT_N - 1; i >= 0; i--)
      if (!n [i])
        clz_tag  = tag_t'(i);
  end endfunction
  
  //
  localparam int PTR_W  = $clog2(N);
  typedef struct packed {
    logic             x;
    logic [PTR_W-1:0] p;
  } ptr_t;

endpackage // rmw_long_latency_pkg

`endif
