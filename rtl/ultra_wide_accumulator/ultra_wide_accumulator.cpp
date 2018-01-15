//========================================================================== //
// Copyright (c) 2017, Stephen Henry
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
#include <deque>
#include <sstream>
#include "vobj/Vultra_wide_accumulator.h"

#define PORTS(__func)                           \
  __func(pass, bool)                            \
  __func(clear, bool)                           \
  __func(x, sc_dt::sc_bv<128>)                  \
  __func(y_r, sc_dt::sc_bv<128>)                \
  __func(y_vld_r, bool)

struct UltraWideAccumulatorTb : libtb::TopLevel
{
  using UUT = Vultra_wide_accumulator;
  SC_HAS_PROCESS(UltraWideAccumulatorTb);
  UltraWideAccumulatorTb(sc_core::sc_module_name mn = "t")
    : uut_("uut")
  {
    SC_METHOD(checker);
    dont_initialize();
    sensitive << e_reset_done();
    uut_.clk(clk());
    uut_.rst(rst());
#define __bind_signal(__name, __type)           \
    uut_.__name(__name##_);
    PORTS(__bind_signal)
#undef __bind_signals
  }
  void checker() {
    if (y_vld_r_) {
      std::stringstream ss;
      sc_dt::sc_biguint<128> b = expected_.front();
      ss << "Expected=0x" << std::hex << b;

      const uint64_t b_hi = b.range(127, 64).to_uint64();
      const uint64_t b_lo = b.range(63, 0).to_uint64();

      const uint64_t y_hi = y_r_.read().range(127, 64).to_uint64();
      const uint64_t y_lo = y_r_.read().range(63, 0).to_uint64();
      if (b_hi != y_hi || b_lo != y_lo) {
        ss << " actual=" << y_r_.read().to_string(sc_dt::SC_HEX);
        LIBTB_REPORT_ERROR(ss.str());
      } else {
        LIBTB_REPORT_INFO(ss.str());
      }
      expected_.pop_front();
    }
    next_trigger(clk().posedge_event());
  }
  bool run_test() {
    t_wait_reset_done();

    clear_ = true;
    t_wait_posedge_clk();
    clear_ = false;
    t_wait_posedge_clk();
    
    int n = 1000000;
    sc_dt::sc_biguint<128> acc = 0;
    while (n--) {
      const int inc = libtb::random_integer_in_range(1 << 31, 0);
      acc += sc_dt::sc_biguint<128>(inc);
      expected_.push_back(acc);
      pass_ = true;
      x_ = inc;
      t_wait_posedge_clk();
    }
    return false;
  }
  std::deque<sc_dt::sc_biguint<128> > expected_;
#define __declare_signal(__name, __type)        \
  sc_core::sc_signal<__type> __name##_;
  PORTS(__declare_signal)
#undef __declare_signal
  UUT uut_;
};

int sc_main (int argc, char **argv) {
  return libtb::LibTbSim<UltraWideAccumulatorTb>(argc, argv).start();
}
