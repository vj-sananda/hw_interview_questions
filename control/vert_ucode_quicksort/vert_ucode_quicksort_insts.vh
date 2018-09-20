`ifndef __VERT_UCODE_QUICKSORT_INSTS_VH__
 `define __VERT_UCODE_QUICKSORT_INSTS_VH__

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

task inst_mov; begin
  inst  = '0;
end endtask

task inst_st; begin
  inst  = '0;
end endtask

task inst_ld; begin
  inst  = '0;
end endtask

task inst_add; begin
  inst  = '0;
end endtask

task inst_sub; begin
  inst  = '0;
end endtask

task inst_push; begin
  inst  = '0;
end endtask

task inst_pop; begin
  inst  = '0;
end endtask

task inst_wait; begin
  inst  = '0;
end endtask

task inst_call; begin
  inst  = '0;
end endtask

task inst_b; begin
  inst  = '0;
end endtask

task inst_ret; begin
  inst  = '0;
end endtask
   
function inst_t decode (inst_t inst); begin
  return '0;
end endfunction


`endif //  `ifndef __VERT_UCODE_QUICKSORT_INSTS_VH__

