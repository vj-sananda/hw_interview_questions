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

`ifndef __VERT_UCODE_QUICKSORT_PKG_VH__
`define __VERT_UCODE_QUICKSORT_PKG_VH__

package vert_ucode_quicksort_pkg;

  localparam int N = 16;

  localparam int W  = 32;

  localparam int BANK_N  = 2;

  typedef logic [BANK_N-1:0]         bank_n_d_t;
  typedef logic [$clog2(BANK_N)-1:0] bank_n_t;
  typedef logic signed [$clog2(N):0] n_t;
  typedef logic [$clog2(N)-1:0]      addr_t;
  typedef logic [W-1:0]              w_t;

  typedef enum logic [2:0] {  BANK_IDLE       = 3'b000,
                              BANK_LOADING    = 3'b001,
                              BANK_READY      = 3'b010,
                              BANK_SORTING    = 3'b011,
                              BANK_SORTED     = 3'b100,
                              BANK_UNLOADING  = 3'b101
                            } bank_status_t;

  typedef struct packed {
    bank_status_t status;
    n_t          n;
    logic        error;
  } bank_state_t;
  localparam int BANK_CONTEXT_W  = $bits(bank_status_t);

  typedef enum   logic [2:0] {  ENQUEUE_FSM_IDLE  = 3'b000,
                                ENQUEUE_FSM_LOAD  = 3'b101
                                } enqueue_fsm_t;
  localparam int ENQUEUE_FSM_BUSY_B = 2;

  typedef enum   logic [2:0] {  DEQUEUE_FSM_IDLE  = 3'b000,
                                DEQUEUE_FSM_EMIT  = 3'b101
                                } dequeue_fsm_t;
  localparam int DEQUEUE_FSM_BUSY_B  = 2;

  typedef logic [7:0] pc_t;

  typedef struct packed {
    logic        foo;
  } sort_context_t;
  localparam int SORT_CONTEXT_W  = $bits(sort_context_t);

  typedef enum   logic [1:0] { UNCOND  = 2'b00,
                               EQ      = 2'b01,
                               GT      = 2'b10,
                               LE      = 2'b11
                               } cc_t;

  typedef logic [2:0] imm_t;
  typedef logic [7:0] field_A_t;

  typedef enum        logic [2:0] { R0     = 3'b000,
                                    R1     = 3'b001,
                                    R2     = 3'b010,
                                    R3     = 3'b011,
                                    R4     = 3'b100,
                                    R5     = 3'b101,
                                    R6     = 3'b110,
                                    BLINK  = 3'b111} reg_t;

  typedef enum        logic [2:0] { REG_N
                                    } reg_special_t;

  typedef enum logic [3:0] { NOP    = 4'b0000,
                             JCC    = 4'b0001,
                             PP     = 4'b0010,
                             MEM    = 4'b0100,
                             MOV    = 4'b0110,
                             ARITH  = 4'b0111,
                             CRET   = 4'b1100,
                             CNTRL  = 4'b1111
                            } opcode_t;

  typedef struct packed {
    opcode_t opcode;
    union packed {
      struct packed {
        logic [11:0] padding;
      } nop;
      struct  packed {
        logic [1:0] padding;
        cc_t cc;
        field_A_t A;
      } jcc;
      struct  packed {
        logic is_pop;
        reg_t dst;
        logic [7:0] padding;
      } pp;
      struct  packed {
        logic is_st;
        reg_t dst;
        logic padding0;
        reg_t src;
        logic [3:0] padding1;
      } mem;
      struct  packed {
        logic is_imm_or_special;
        reg_t dst;
        union packed {
          struct packed {
            logic padding0;
            reg_t src;
            logic [3:0] padding1;
          } simple;
          struct packed {
            logic is_special;
            struct packed {
              reg_special_t special;
              logic [3:0] padding;
            } special;
            struct packed {
              logic [3:0] padding;
              imm_t imm;
            } immediate;
          } imm_or_special;
        } u;
      } mov;
      struct  packed {
        logic is_sub;
        reg_t dst;
        logic wren;
        reg_t src0;
        logic is_imm;
        union packed {
          reg_t src1;
          imm_t imm;
        } u;
      } arith;
      struct  packed {
        logic is_call;
        logic [2:0] padding;
        field_A_t a;
      } cret;
      struct  packed {
        logic is_wait;
        logic [10:0] padding;
      } cntrl;
    } u;
  } inst_t;


  typedef struct     packed {
    logic            is_emit;
    logic            is_wait;
    logic            is_load;
    logic            is_store;
    logic            is_jump;
    logic            is_push;
    logic            is_pop;
    
    //
    logic            dst_en;
    reg_t            dst;

    //
    logic            src0_en;
    reg_t            src0;
    logic            src1_en;
    reg_t            src1;

    //
    logic            src0_is_zero;

    //
    logic            has_imm;
    imm_t            imm;

    //
    logic            has_special;
    reg_special_t    special;

    //
    logic            inv_src1;
    logic            cin;

    //
    logic            flag_en;

    //
    logic            invalid_inst;
    
  } ucode_t;


  function ucode_t decode(inst_t inst); begin
    ucode_t ucode  = '0;
    case (inst.opcode)
      NOP: begin
      end
      JCC: begin
      end
      PP: begin
      end
      MEM: begin
      end
      MOV: begin
      end
      ARITH: begin
      end
      CRET: begin
        case (inst.u.cret.is_call)
          1'b1: begin
            ucode.dst_en  = 'b1;
            ucode.dst     = BLINK;
          end
          default: begin
            ucode.src0_en  = 'b1;
            ucode.src0     = BLINK;
          end
        endcase // case (inst.u.cret.is_call)
      end
      CNTRL: begin
        ucode.is_emit  = (~inst.u.cntrl.is_wait);
        ucode.is_wait  = inst.u.cntrl.is_wait;
      end
      default: begin
        ucode.invalid_inst  = 'b1;
      end
    endcase // case (inst.opcode)
    return ucode;
  end endfunction
    

endpackage // fsm_quicksort_pkg
  
`endif //  `ifndef __VERT_UCODE_QUICKSORT_PKG_VH__
