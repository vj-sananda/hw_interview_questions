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

`include "vert_ucode_quicksort_pkg.vh"

module vert_ucode_quicksort (

   //======================================================================== //
   //                                                                         //
   // Misc.                                                                   //
   //                                                                         //
   //======================================================================== //

     input                                        clk
   , input                                        rst

   //======================================================================== //
   //                                                                         //
   // Unsorted                                                                //
   //                                                                         //
   //======================================================================== //

   //
   , input                                        unsorted_vld
   , input                                        unsorted_sop
   , input                                        unsorted_eop
   , input     [vert_ucode_quicksort_pkg::W-1:0]  unsorted_dat
   //
   , output logic                                 unsorted_rdy

   //======================================================================== //
   //                                                                         //
   // Sorted                                                                  //
   //                                                                         //
   //======================================================================== //

   //
   , output logic                                 sorted_vld_r
   , output logic                                 sorted_sop_r
   , output logic                                 sorted_eop_r
   , output logic                                 sorted_err_r
   , output logic [vert_ucode_quicksort_pkg::W-1:0] sorted_dat_r

   //======================================================================== //
   //                                                                         //
   // Control                                                                 //
   //                                                                         //
   //======================================================================== //

   //
   , output logic                            busy_r
);
  import vert_ucode_quicksort_pkg::*;

  typedef struct packed {
    logic        vld;
    logic        sop;
    logic        eop;
    logic        err;
    bank_n_t     idx;
  } dequeue_momento_t;

  typedef struct packed {
    logic        is_dat_p;
    logic        is_dat_i;
    logic        is_dat_j;
  } sort_momento_t;

  bank_state_t   [BANK_N-1:0]           bank_state_r;
  bank_state_t   [BANK_N-1:0]           bank_state_w;
  logic          [BANK_N-1:0]           bank_state_en;
  //
  bank_n_d_t                            queue_idle;
  bank_n_d_t                            queue_ready;
  bank_n_d_t                            queue_sorted;
  //
  addr_t                                enqueue_idx_r;
  addr_t                                enqueue_idx_w;
  logic                                 enqueue_idx_en;
  //
  bank_n_d_t                            dequeue_ack;
  bank_n_d_t                            dequeue_sel;
  bank_n_d_t                            dequeue_vld;
  //
  addr_t                                dequeue_idx_r;
  addr_t                                dequeue_idx_w;
  logic                                 dequeue_idx_en;
  //
  logic                                 unsorted_rdy;
  //
  enqueue_fsm_t                         enqueue_fsm_r;
  enqueue_fsm_t                         enqueue_fsm_w;
  logic                                 enqueue_fsm_en;
  //
  bank_n_t                              enqueue_bank_idx_r;
  bank_n_t                              enqueue_bank_idx_w;
  logic                                 enqueue_bank_idx_en;
  //
  enqueue_fsm_t                         dequeue_fsm_r;
  enqueue_fsm_t                         dequeue_fsm_w;
  logic                                 dequeue_fsm_en;
  //
  bank_n_t                              sort_bank_idx_r;
  bank_n_t                              sort_bank_idx_w;
  logic                                 sort_bank_idx_en;
  //
  bank_n_t                              dequeue_bank_idx_r;
  bank_n_t                              dequeue_bank_idx_w;
  logic                                 dequeue_bank_idx_en;
  //
  bank_state_t                          enqueue_bank;
  logic                                 enqueue_bank_en;
  //
  bank_state_t                          dequeue_bank;
  logic                                 dequeue_bank_en;
  //
  bank_state_t                          sort_bank;
  logic                                 sort_bank_en;
  //
  `SPSRAM_SIGNALS(enqueue__, W, $clog2(N));
  `SPSRAM_SIGNALS(dequeue__, W, $clog2(N));
  `SPSRAM_SIGNALS(sort__, W, $clog2(N));
  //
  logic                                 sorted_vld_w;
  logic                                 sorted_sop_w;
  logic                                 sorted_eop_w;
  logic                                 sorted_err_w;
  w_t                                   sorted_dat_w;
  //
  bank_n_d_t                            sortqueue_sel;
  //
  logic  [BANK_N-1:0]                   spsram_bank__en;
  logic  [BANK_N-1:0]                   spsram_bank__wen;
  addr_t [BANK_N-1:0]                   spsram_bank__addr;
  w_t    [BANK_N-1:0]                   spsram_bank__din;
  w_t    [BANK_N-1:0]                   spsram_bank__dout;
  //
  dequeue_momento_t                     dequeue_momento_in;
  dequeue_momento_t                     dequeue_momento_out_r;
  //
  logic                                 stack__cmd_vld;
  logic                                 stack__cmd_push;
  logic                                 stack__cmd_clr;
  sort_context_t                        stack__cmd_push_dat;
  sort_context_t                        stack__cmd_pop_dat_r;
  logic                                 stack__empty_r;
  logic                                 stack__full_r;
  //
  sort_momento_t                        sort_momento_in;
  sort_momento_t                        sort_momento_out_r;
  //
  pc_t                                  pc_r;
  pc_t                                  pc_w;
  //
  inst_t                                inst;
  //
  ucode_t                               ucode;
  
  // ======================================================================== //
  //                                                                          //
  // Combinatorial Logic                                                      //
  //                                                                          //
  // ======================================================================== //
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : queue_ctrl_PROC

      //
      queue_idle  = '0;
      for (int i = 0; i < BANK_N; i++)
        queue_idle [i]  = (bank_state_r [i].status == BANK_IDLE);

      //
      queue_ready  = '0;
      for (int i = 0; i < BANK_N; i++)
        queue_ready [i]  = (bank_state_r [i].status == BANK_READY);

      //
      queue_sorted       = '0;
      for (int i = 0; i < BANK_N; i++)
        queue_sorted [i]  = (bank_state_r [i].status == BANK_SORTED);

    end // block: queue_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : enqueue_fsm_PROC

      //
      enqueue__en          = '0;
      enqueue__wen         = '0;
      enqueue__addr        = '0;
      enqueue__din         = unsorted_dat;

      //
      enqueue_bank_idx_en  = '0;
      enqueue_bank_idx_w   = enqueue_bank_idx_r + 1'b1;

      //
      enqueue_bank         = '0;
      enqueue_bank_en      = '0;

      //
      enqueue_fsm_w        = enqueue_fsm_r;

      // verilator lint_off CASEINCOMPLETE
      unique case (enqueue_fsm_r)

        ENQUEUE_FSM_IDLE: begin

          if (unsorted_vld) begin
            enqueue__en          = 'b1;
            enqueue__wen         = 'b1;
            enqueue__addr        = '0;

            //
            enqueue_bank_en      = '1;
            enqueue_bank         = 0;
            enqueue_bank.status  = unsorted_eop ? BANK_READY : BANK_LOADING;

            //
            if (!unsorted_eop)
              enqueue_fsm_w  = ENQUEUE_FSM_LOAD;
            else
              enqueue_bank_idx_en  = 'b1;
          end
        end // case: ENQUEUE_FSM_IDLE

        ENQUEUE_FSM_LOAD: begin

          if (unsorted_vld) begin
            enqueue__en    = 'b1;
            enqueue__wen   = 'b1;
            enqueue__addr  = addr_t'(enqueue_idx_r);

            if (unsorted_eop) begin
              enqueue_bank_idx_en  = 'b1;

              //
              enqueue_bank         = '0;
              enqueue_bank.status  = BANK_READY;
              enqueue_bank.n       = {1'b0, enqueue_idx_r};
              enqueue_bank_en      = '1;

              //              
              enqueue_fsm_w        = ENQUEUE_FSM_IDLE;
            end
            
          end

        end // case: ENQUEUE_FSM_LOAD

      endcase // unique case (enqueue_fsm_r)
      // verilator lint_on CASEINCOMPLETE

      //
      unsorted_rdy    = queue_idle [enqueue_bank_idx_r];

      //
      enqueue_fsm_en  = (enqueue_fsm_r [ENQUEUE_FSM_BUSY_B] |
                         enqueue_fsm_w [ENQUEUE_FSM_BUSY_B]);

      //
      enqueue_idx_en  = enqueue_fsm_en;

      //
      unique case (enqueue_fsm_r)
        ENQUEUE_FSM_IDLE: enqueue_idx_w  = 'b1;
        default:          enqueue_idx_w  = enqueue_idx_r + 'b1;
      endcase // unique case (enqueue_fsm_r)

    end // block: enqueue_fsm_PROC

  // ------------------------------------------------------------------------ //
  //
  // Algorithm:
  //
  // function partition (A, lo, hi) is
  //   pivot := A[hi];
  //   i := lo;
  //   for j := lo to hi - 1 do
  //     if A[j] < pivot then
  //       swap A[i] with A[j];
  //       i := i + 1;
  //   swap A[i] with A[hi];
  //   return i;
  //
  // function quicksort (A, lo, hi) is
  //   if lo < hi:
  //     p := partition(A, lo, hi)
  //     quicksort(A, lo, p - 1)
  //     quicksort(A, p + 1, hi)
  //
  // Pseudo-code:
  //
  // PROC PARTITION(A, lo, hi):
  // PRT +    : POP R0           ; R0 <- lo
  // PRT +    : POP R1           ; R1 <- hi
  //;  pivot := A[hi];
  // PRT +    : LD R2, [R1]      ; R2 <- A[hi]        (R2 === pivot)
  //;  i := lo
  // PRT +    : MOV R3, R0       ; R3 <- lo           (R3 === i)
  //;  j := lo
  // PRT +    : MOV R4, R0       ; R4 <- lo           (R4 === j)
  // __loop_start:
  // PRT +    : SUB 0, R1, R4    ;
  // PRT +    : BEQ __end        ;
  //;  if (A[j] < pivot) then
  // PRT +    : LD R5, [R4]      ; R5 <- A[j]
  // PRT +    : SUB 0, R5, R2    ;
  // PRT +    : BGT __end_loop   ; if ((A[j] - pivot) > 0) goto __end_of_loop
  //;  swap A[i] with A[j]
  // PRT +    : LD R6, [R3]      ;
  // PRT +    : ST [R3], R5
  // PRT +    : ST [R4], R6
  //;  i := i + 1
  // PRT +    : ADD R3, R3, 1    ; R3 <- (R3 + 1)
  // __end_of_loop:
  // PRT +    : ADD R4, R4, 1    ; R4 <- (R4 + 1)
  // PRT +    : B __loop_start   ;
  // __end:
  // PRT +    : LD R0, [R3]      ;
  // PRT +    : LD R1, [R4]      ;
  // PRT +    : ST [R3], R1      ;
  // PRT +    : ST [R4], R0      ;
  // PRT +    : MOV R0,R3        ; ret <- pivot
  // PRT +    : RET              ; PC <- BLINK
  //
  // PROC QUICKSORT:
  // QS +     : PUSH BLINK        ;
  // QS +     : POP R0            ; hi <- R0
  // QS +     : POP R1            ; lo <- R1
  // QS +     : SUB 0, R0, R1     ; 
  // QS +     : BNZ __end         ; if ((hi - lo) > 0) goto __end;
  // QS +     : PUSH R0           ; 
  // QS +     : PUSH R1           ;
  //;   p := partition(A, lo, hi)
  // QS +     : CALL PARTITION    ; r0 := partition(A, lo, hi); BLINK <- QS + 7
  //;   quicksort(A, lo, p - 1)
  // QS +     : SUB R2, R0, 1     ;
  // QS +     : PUSH R0           ;
  // QS +     : PUSH R2           ;
  // QS +     : CALL QUICKSORT    ; quicksort(A, lo, p - 1); BLINK <- QS +  8
  //;   quicksort(A, p + 1, hi)
  // QS +     : ADD R2, R0, 1     ;
  // QS +     : PUSH R2           ;
  // QS +     : PUSH R2           ;
  // QS +     : CALL QUICKSORT    ; quicksort(A, p + 1, hi); BLINK <- QS +  9
  // __end:
  // QS +     : POP BLINK         ; 
  // QS +     : RET               ; PC <- BLINK
  //
  // PROC MAIN(A, lo, hi):
  //    0     : JMP MAIN         ; PC <- MAIN
  // MAIN     : WAIT_RDY         ; wait until queue_ready == 1
  // MAIN +  1: PUSH 0           ; lo <- 0
  // MAIN +  2: PUSH n           ; hi <- n
  // MAIN +  3: CALL QUICKSORT   ; call quicksort(A, lo, hi); PC <- QS + 0; BLINK <- MAIN + 4
  // MAIN +  4: JMP MAIN         ; goto __main
  //
  `include "vert_ucode_quicksort_insts.vh"
  //
  always_comb
    begin : quicksort_prog_PROC

      inst  = '0;

      case (pc_r)
        RESET_VECTOR: inst_nop;
        
        default: ;
        
      endcase // case (pc_r)

    end // block: quicksort_prog_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : ucode_PROC

      //
      ucode  = decode(inst);


    end // block: ucode_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : dequeue_fsm_PROC

      //
      dequeue__en             = 'b0;
      dequeue__wen            = 'b0;
      dequeue__addr           = 'b0;
      dequeue__din            = 'b0;

      //
      dequeue_bank_idx_en     = '0;
      dequeue_bank_idx_w      = dequeue_bank_idx_r + 'b1;

      //
      dequeue_ack             = 'b1;

      //
      dequeue_bank            = 'b0;
      dequeue_bank_en         = 'b0;

      //
      dequeue_momento_in.vld  = '0;
      dequeue_momento_in.sop  = '0;
      dequeue_momento_in.eop  = '0;
      dequeue_momento_in.err  = '0;
      dequeue_momento_in.idx  = dequeue_bank_idx_r;

      sorted_err_w            = 'b0;
      
      //
      dequeue_fsm_w           = dequeue_fsm_r;

      // verilator lint_off CASEINCOMPLETE
      unique case (dequeue_fsm_r)

        DEQUEUE_FSM_IDLE: begin

          if (queue_sorted [dequeue_bank_idx_r]) begin
            bank_state_t st         = bank_state_r [dequeue_bank_idx_r];
            
            dequeue__en             = 'b1;
            dequeue__addr           = 'b0;

            //
            dequeue_momento_in.vld  = '1;
            dequeue_momento_in.sop  = '1;
            dequeue_momento_in.err  = st.error;

            dequeue_bank_en         = 'b1;
            dequeue_bank            = st;
            
            if (st.n == '0) begin
              dequeue_momento_in.eop  = '1;

              dequeue_bank.status     = BANK_IDLE;
            end else begin
              dequeue_momento_in.eop  = '0;
              
              dequeue_bank.status    = BANK_UNLOADING;
              dequeue_fsm_w          = DEQUEUE_FSM_EMIT;
            end
          end
        end

        DEQUEUE_FSM_EMIT: begin
          bank_state_t st         = bank_state_r [dequeue_bank_idx_r];
          
          dequeue__en             = 'b1;
          dequeue__addr           = dequeue_idx_r;

          //
          dequeue_momento_in.vld  = '1;
          dequeue_momento_in.sop  = '0;
          dequeue_momento_in.eop  = '0;
          dequeue_momento_in.err  = st.error;

          if (dequeue_idx_r == addr_t'(st.n)) begin
            dequeue_bank_idx_en     = 'b1;

            //
            dequeue_momento_in.eop  = '1;

            //
            dequeue_bank_en         = 'b1;
            dequeue_bank            = st;
            dequeue_bank.status     = BANK_IDLE;

            dequeue_fsm_w           = DEQUEUE_FSM_IDLE;
          end
          
        end // case: DEQUEUE_FSM_EMIT
      endcase // unique case (dequeue_fsm_r)
      // verilator lint_on CASEINCOMPLETE

      //
      dequeue_fsm_en  = (dequeue_fsm_w [DEQUEUE_FSM_BUSY_B] |
                         dequeue_fsm_r [DEQUEUE_FSM_BUSY_B]);

      //
      dequeue_idx_en  = dequeue_fsm_en;

      //
      unique case (dequeue_fsm_r)
        DEQUEUE_FSM_IDLE: dequeue_idx_w  = 'b1;
        default:          dequeue_idx_w  = dequeue_idx_r + 'b1;
      endcase // unique case (dequeue_fsm_r)

    end // block: dequeue_fsm_PROC

  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : bank_PROC

      for (int i = 0; i < BANK_N; i++) begin

        //
        bank_state_en [i]  = '0;
        bank_state_w [i]   = '0;

        if (bank_n_t'(i)   == enqueue_bank_idx_r) begin
          bank_state_en [i]  |= enqueue_bank_en;
          bank_state_w [i]   |= enqueue_bank;
        end
        if (bank_n_t'(i)   == sort_bank_idx_r) begin
          bank_state_en [i]  |= sort_bank_en;
          bank_state_w [i]   |= sort_bank;
        end
        if (bank_n_t'(i)   == dequeue_bank_idx_r) begin
          bank_state_en [i]  |= dequeue_bank_en;
          bank_state_w [i]   |= dequeue_bank;
        end

      end

    end // block: bank_PROC

  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : spsram_PROC

      for (int i = 0; i < BANK_N; i++) begin
        
        spsram_bank__en [i]    = '0;
        spsram_bank__wen [i]   = '0;
        spsram_bank__addr [i]  = '0;
        spsram_bank__din [i]   = '0;

        if (bank_n_t'(i) == enqueue_bank_idx_r) begin
          spsram_bank__en [i]    |= enqueue__en;
          spsram_bank__wen [i]   |= enqueue__wen;
          spsram_bank__addr [i]  |= enqueue__addr;
          spsram_bank__din [i]   |= enqueue__din;
        end
        if (bank_n_t'(i) == sort_bank_idx_r) begin
          spsram_bank__en [i]    |= sort__en;
          spsram_bank__wen [i]   |= sort__wen;
          spsram_bank__addr [i]  |= sort__addr;
          spsram_bank__din [i]   |= sort__din;
        end
        if (bank_n_t'(i) == dequeue_bank_idx_r) begin
          spsram_bank__en [i]    |= dequeue__en;
          spsram_bank__wen [i]   |= dequeue__wen;
          spsram_bank__addr [i]  |= dequeue__addr;
          spsram_bank__din [i]   |= dequeue__din;
        end

      end // for (int i = 0; i < BANK_N; i++)

    end // block: spsram_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_comb
    begin : sorted_out_PROC

      //
      sorted_vld_w  = dequeue_momento_out_r.vld;
      sorted_sop_w  = dequeue_momento_out_r.sop;
      sorted_eop_w  = dequeue_momento_out_r.eop;
      sorted_err_w  = dequeue_momento_out_r.err;
      sorted_dat_w  = spsram_bank__dout [dequeue_momento_out_r.idx];

    end // block: sorted_out_PROC
  
  // ======================================================================== //
  //                                                                          //
  // Sequential Logic                                                         //
  //                                                                          //
  // ======================================================================== //
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      pc_r <= vert_ucode_quicksort_pkg::RESET_VECTOR;
    else
      pc_r <= pc_w;
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      enqueue_fsm_r <= ENQUEUE_FSM_IDLE;
    else if (enqueue_fsm_en)
      enqueue_fsm_r <= enqueue_fsm_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      dequeue_fsm_r <= DEQUEUE_FSM_IDLE;
    else if (dequeue_fsm_en)
      dequeue_fsm_r <= dequeue_fsm_w;
      
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      enqueue_idx_r <= 'b0;
    else if (enqueue_idx_en)
      enqueue_idx_r <= enqueue_idx_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      dequeue_idx_r <= 'b0;
    else if (dequeue_idx_en)
      dequeue_idx_r <= dequeue_idx_w;

  // ------------------------------------------------------------------------ //
  // TODO - rename to SCOREBOARD
  always_ff @(posedge clk) begin : bank_reg_PROC
    if (rst) begin
      for (int i = 0; i < BANK_N; i++)
        bank_state_r [i] <= '{status:BANK_IDLE, default:'0};
    end else begin
      for (int i = 0; i < BANK_N; i++)
        if (bank_state_en [i])
          bank_state_r [i] <= bank_state_w [i];
    end
  end // block: bank_reg_PROC
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      enqueue_bank_idx_r <= '0;
    else if (enqueue_bank_idx_en)
      enqueue_bank_idx_r <= enqueue_bank_idx_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      dequeue_bank_idx_r <= '0;
    else if (dequeue_bank_idx_en)
      dequeue_bank_idx_r <= dequeue_bank_idx_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      sort_bank_idx_r <= '0;
    else if (sort_bank_idx_en)
      sort_bank_idx_r <= sort_bank_idx_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (rst)
      sorted_vld_r <= 'b0;
    else
      sorted_vld_r <= sorted_vld_w;
  
  // ------------------------------------------------------------------------ //
  //
  always_ff @(posedge clk)
    if (sorted_vld_w) begin
      sorted_sop_r  = sorted_sop_w;
      sorted_eop_r  = sorted_eop_w;
      sorted_err_r  = sorted_err_w;
      sorted_dat_r  = sorted_dat_w;
    end

  
  // ======================================================================== //
  //                                                                          //
  // Instances                                                                //
  //                                                                          //
  // ======================================================================== //

  // ------------------------------------------------------------------------ //
  //
  stack #(.W(SORT_CONTEXT_W), .N(16)) u_stack (
    //
      .clk               (clk                )
    , .rst               (rst                )
    //
    , .cmd_vld           (stack__cmd_vld     )
    , .cmd_push          (stack__cmd_push    )
    , .cmd_push_dat      (stack__cmd_push_dat)
    , .cmd_clr           (stack__cmd_clr     )
    //
    , .cmd_pop_dat_r     (stack__cmd_pop_dat_r)
    //
    , .empty_r           (stack__empty_r     )
    , .full_r            (stack__full_r      )
  );

  // ------------------------------------------------------------------------ //
  //
  delay_pipe #(.W($bits(dequeue_momento_t)), .N(1)) u_dequeue_momento_delay_pipe (
    //
      .clk               (clk                     )
    , .rst               (rst                     )
    //
    , .in                (dequeue_momento_in      )
    , .out_r             (dequeue_momento_out_r   )
  );

  // ------------------------------------------------------------------------ //
  //
  delay_pipe #(.W($bits(sort_momento_t)), .N(1)) u_sort_momento_delay_pipe (
    //
      .clk               (clk                     )
    , .rst               (rst                     )
    //
    , .in                (sort_momento_in         )
    , .out_r             (sort_momento_out_r      )
  );

  // ------------------------------------------------------------------------ //
  //
  generate for (genvar g = 0; g < BANK_N; g++) begin
  
    spsram #(.W(W), .N(N)) u_spsram_bank (
      //
        .clk           (clk                       )
      //
      , .en            (spsram_bank__en [g]       )
      , .wen           (spsram_bank__wen [g]      )
      , .addr          (spsram_bank__addr [g]     )
      , .din           (spsram_bank__din [g]      )
      //
      , .dout          (spsram_bank__dout [g]     )
    );

  end endgenerate

endmodule // fsm_quicksort
