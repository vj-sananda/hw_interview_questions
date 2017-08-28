//========================================================================== //
// Copyright (c) 2016, Stephen Henry
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

#include <libtb.h>
#include <vector>
#include <deque>
#include <sstream>
#include "Vlinked_list_fifo.h"

#define PORTS(__func)                           \
  __func(cmd_pass, bool)                        \
  __func(cmd_push, bool)                        \
  __func(cmd_id, uint32_t)                      \
  __func(cmd_push_data, uint32_t)               \
  __func(cmd_pop_data, uint32_t)                \
  __func(clear, bool)                           \
  __func(full_r, bool)                          \
  __func(empty_r, bool)                         \
  __func(nempty_r, uint32_t)                    \
  __func(busy_r, bool)

static constexpr int ID_N = 4;
static constexpr int PTR_N = 255;

using namespace libtb;
struct LinkedListFifoTb : libtb::TopLevel
{
  using UUT = Vlinked_list_fifo;
  using IdT = uint32_t;
  using DataT = uint32_t;
  enum class OpT { PUSH, POP };
  SC_HAS_PROCESS(LinkedListFifoTb);
  LinkedListFifoTb(sc_core::sc_module_name mn = "t")
    : uut_("uut") {
    expected_.resize(4);
    cnt_ = 0;
    uut_.clk(clk());
    uut_.rst(rst());
#define __bind_signal(__name, __type)           \
    uut_.__name(__name##_);
    PORTS(__bind_signal)
#undef __bind_signals
  }
  bool run_test() {
    cmd_idle();
    int ops = 10000;
    std::vector<OpT> op(2);
    while (ops > 0) {
      t_wait_sync();
      if (cnt_ == 0 && !empty_r_)
        LIBTB_REPORT_ERROR("device should report empty.");
      if (cnt_ != 0 && empty_r_)
        LIBTB_REPORT_ERROR("device not should report empty.");
      if (cnt_ == PTR_N && !full_r_)
        LIBTB_REPORT_ERROR("device should report full.");
      if (cnt_ != PTR_N && full_r_)
        LIBTB_REPORT_ERROR("device should not report full.");
      
      op.clear();
      const IdT id = random_integer_in_range(ID_N - 1);
      if (cnt_ < PTR_N)
        op.push_back(OpT::PUSH);
      if (expected_[id].size())
        op.push_back(OpT::POP);

      if (op.size()) {
        if (*choose_random(op.begin(), op.end()) == OpT::PUSH) {
          ++cnt_;
          cmd_push(id, random<DataT>());
        } else {
          --cnt_;
          cmd_pop(id);
        }
        --ops;
      }
    }
    return false;
  }
  void cmd_push(IdT id, DataT data) {
    cmd_pass_ = true;
    cmd_push_ = true;
    cmd_id_ = id;
    cmd_push_data_ = data;
    expected_[id].push_back(data);
    t_wait_posedge_clk();
    cmd_idle();
    t_wait_posedge_clk();
  }
  void cmd_pop(IdT id) {
    DataT actual{};
    cmd_pass_ = true;
    cmd_push_ = false;
    cmd_id_ = id;
    t_wait_posedge_clk();
    t_wait_sync();
    actual = cmd_pop_data_;
    if (expected_[id].size() == 0) {
      LIBTB_REPORT_ERROR("Pop from empty queue!");
    }
    const DataT expected = expected_[id].front();
    if (actual != expected) {
      std::stringstream ss;
      ss << "Mismatch:" << std::hex
         << "expected " << expected << " actual " << actual;
      LIBTB_REPORT_INFO(ss.str());
    }
    expected_[id].pop_front();
    cmd_idle();
    t_wait_posedge_clk();

  }
  void cmd_idle() {
    cmd_pass_ = false;
    cmd_push_ = false;
    cmd_id_ = 0;
    cmd_push_data_ = 0;
  }
  void clear() {
    clear_ = true;
    t_wait_posedge_clk();
    clear_ = false;
  }
  std::vector<std::deque<DataT>> expected_;
  std::size_t cnt_;
#define __declare_signal(__name, __type)        \
  sc_core::sc_signal<__type> __name##_;
  PORTS(__declare_signal)
#undef __declare_signal
  UUT uut_;
};

int sc_main(int argc, char **argv)
{
  return libtb::LibTbSim<LinkedListFifoTb>(argc, argv).start();
}
