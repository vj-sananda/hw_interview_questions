##========================================================================== //
## Copyright (c) 2016-2018, Stephen Henry
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##
## * Redistributions of source code must retain the above copyright notice, this
##   list of conditions and the following disclaimer.
##
## * Redistributions in binary form must reproduce the above copyright notice,
##   this list of conditions and the following disclaimer in the documentation
##   and/or other materials provided with the distribution.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
## LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
## CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
## SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
## INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
## CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.
##========================================================================== //

macro(emit_answer target answer_sv answer_cc)
  message(STATUS "Emit soluion: ${target}")
  libtb2_add_include_path(${CMAKE_CURRENT_SOURCE_DIR})
  libtb2_add_include_path(${CMAKE_CURRENT_BINARY_DIR})
  get_filename_component(answer ${answer_sv} NAME_WE)
  libtb2_verilate_sc(verilate_${target} ${answer} vlib)
  add_executable(${target} ${answer_cc})
  add_dependencies(${target} verilate_${target}) 
  target_include_directories(${target} PUBLIC
    ${LIBTB_ROOT}/src
    ${CMAKE_CURRENT_BINARY_DIR}
    )
  target_link_libraries(${target} tb2 systemc scv ${vlib} vcom)
endmacro()

add_subdirectory(one_or_two)
add_subdirectory(latency)
add_subdirectory(multi_counter)
add_subdirectory(multi_counter_variants)
add_subdirectory(gates_from_MUX2X1)
add_subdirectory(increment)
add_subdirectory(fifo_async)
add_subdirectory(fifo_multi_push)
add_subdirectory(fifo_n)
add_subdirectory(fifo_sr)
add_subdirectory(fifo_ptr)
add_subdirectory(count_ones)
add_subdirectory(mcp_formulation)
add_subdirectory(detect_sequence)
add_subdirectory(vending_machine_fsm)
add_subdirectory(vending_machine_dp)
add_subdirectory(clk_div_by_3)
add_subdirectory(missing_duplicated_word)
add_subdirectory(multiply_by_21)
#add_subdirectory(sorted_lists)
add_subdirectory(ultra_wide_accumulator)
add_subdirectory(zero_indices_slow)
add_subdirectory(zero_indices_fast)
add_subdirectory(count_zeros_32)
add_subdirectory(div_by_3)
add_subdirectory(fused_multiply_add)
add_subdirectory(fibonacci)
add_subdirectory(using_full_adders)
add_subdirectory(simd)
add_subdirectory(linked_list_fifo)
add_subdirectory(doubly_linked_list)
