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
#include <deque>
#include <vector>
#include "vobj/Vsimd.h"

#define PORTS(__func)                           \
  __func(pass, bool)                            \
  __func(op, uint32_t)                          \
  __func(A, uint32_t)                           \
  __func(B, uint32_t)                           \
  __func(Y_r, uint32_t)                         \
  __func(valid_r, bool)


void set16(uint32_t &y, uint32_t a, int i) {
  uint32_t mask = 0xFFFF << (i * 16);
  y &= (~mask);
  y |= (a << (i * 16));
}
uint32_t get16(uint32_t y, int i) {
  return (y >> (16 * i)) & 0xFFFF;
}
void set8(uint32_t &y, uint32_t a, int i) {
  uint32_t mask = 0xFF << (i * 8);
  y &= (~mask);
  y |= (a << (i * 8));
}
uint32_t get8(uint32_t y, int i) {
  return (y >> (8 * i)) & 0xFF;
}
  
enum class OpT : uint8_t { OP_SEL0 = 0,
    OP_SEL1, OP_ADD32, OP_SUB32, OP_ADD16, OP_SUB16,
    OP_ADD8, OP_SUB8, OP_ADDSUB16, OP_SUBADD16, OP_ADDSUB8,
    OP_SUBADD8
};
const char * to_string(const OpT op) {
  switch (op) {
#define _stringify(OP) case OP: return #OP; break;
    _stringify(OpT::OP_SEL0)
    _stringify(OpT::OP_SEL1)
    _stringify(OpT::OP_ADD32)
    _stringify(OpT::OP_SUB32)
    _stringify(OpT::OP_ADD16)
    _stringify(OpT::OP_SUB16)
    _stringify(OpT::OP_ADDSUB16)
    _stringify(OpT::OP_SUBADD16)
    _stringify(OpT::OP_ADD8)
    _stringify(OpT::OP_SUB8)
    _stringify(OpT::OP_ADDSUB8)
    _stringify(OpT::OP_SUBADD8)
#undef _stringify
  }
  return "<Invalid Op>";
}

using namespace libtb;
struct SIMDTb : TopLevel
{
  static constexpr int T_N = 100000;
  using UUT = Vsimd;
  using DataT = uint32_t;
  struct Expectation {
    OpT op;
    DataT A, B, Y;
  };
  SC_HAS_PROCESS(SIMDTb);
  SIMDTb(sc_core::sc_module_name mn = "t")
    : uut_("uut")
  {
    SC_METHOD(m_checker);
    dont_initialize();
    sensitive << e_tb_sample();
    uut_.clk(clk());
    uut_.rst(rst());
#define __bind_signal(__name, __type)           \
    uut_.__name(__name##_);
    PORTS(__bind_signal)
#undef __bind_signals
      }
  bool run_test () {
    t_wait_reset_done();
    int n = T_N;
    std::vector<OpT> ops{
      OpT::OP_SEL0, OpT::OP_SEL1, OpT::OP_ADD32, OpT::OP_SUB32,
      OpT::OP_ADD16, OpT::OP_SUB16, 
      OpT::OP_ADD8, OpT::OP_SUB8,
      OpT::OP_ADDSUB16, OpT::OP_SUBADD16,
      OpT::OP_ADDSUB8, OpT::OP_SUBADD8
    };
    while (n--) {
      pass_ = true;
      const OpT op = *choose_random(ops.begin(), ops.end());
      const DataT A = random<DataT>();
      const DataT B = random<DataT>();
      op_ = static_cast<uint32_t>(op);
      A_ = A;
      B_ = B;
      const DataT E = compute_expected(op, A, B);
      expected_.push_back({op, A, B, E});
      t_wait_posedge_clk();
      pass_ = false;
    }
    LIBTB_REPORT_INFO("Stimulus done");
    t_wait_posedge_clk(10);
    return false;
  }
  DataT compute_expected(const OpT op, const DataT A, const DataT B) {
    DataT r{};
    switch (op) {
    case OpT::OP_SEL0: {
      r = A;
    } break;
    case OpT::OP_SEL1: {
      r = B;
    } break;
    case OpT::OP_ADD32: {
      r = (A + B);
    } break;
    case OpT::OP_SUB32: {
      r = (A - B);
    } break;
    case OpT::OP_ADD16: {
      for (int i = 0; i < 2; i++)
        set16(r, get16(A, i) + get16(B, i), i);
    } break;
    case OpT::OP_ADDSUB16: {
      set16(r, get16(A, 0) - get16(B, 0), 0);
      set16(r, get16(A, 1) + get16(B, 1), 1);
    } break;
    case OpT::OP_SUBADD16: {
      set16(r, get16(A, 0) + get16(B, 0), 0);
      set16(r, get16(A, 1) - get16(B, 1), 1);
    } break;
    case OpT::OP_ADD8: {
      for (int i = 0; i < 4; i++)
        set8(r, get8(A, i) + get8(B, i), i);
    } break;
    case OpT::OP_ADDSUB8: {
      set8(r, get8(A, 0) - get8(B, 0), 0);
      set8(r, get8(A, 1) + get8(B, 1), 1);
      set8(r, get8(A, 2) - get8(B, 2), 2);
      set8(r, get8(A, 3) + get8(B, 3), 3);
    } break;
    case OpT::OP_SUBADD8: {
      set8(r, get8(A, 0) + get8(B, 0), 0);
      set8(r, get8(A, 1) - get8(B, 1), 1);
      set8(r, get8(A, 2) + get8(B, 2), 2);
      set8(r, get8(A, 3) - get8(B, 3), 3);
    } break;
    case OpT::OP_SUB16: {
      for (int i = 0; i < 2; i++)
        set16(r, get16(A, i) - get16(B, i), i);
    } break;
    case OpT::OP_SUB8: {
      for (int i = 0; i < 4; i++)
        set8(r, get8(A, i) - get8(B, i), i);
    } break;
    }
    return r;
  }
  void m_checker() {
    if (valid_r_) {
      if (expected_.size() == 0)
        LIBTB_REPORT_ERROR("Unexpected response");

      const DataT actual = Y_r_;
      const Expectation expected = expected_.front();
      expected_.pop_front();

      if (actual != expected.Y) {
        std::stringstream ss;
        ss << "Mismatch: {" << std::hex
           << "A=" << expected.A << ",B=" << expected.B
           << ",op=" << to_string(expected.op) << ","
           << "Y (actual) =" << actual << ","
           << "Y (expected) =" << expected.Y << "}"
          ;
        LIBTB_REPORT_ERROR(ss.str());
      } else {
        std::stringstream ss;
        ss << "Validated: {"
           << "OP=" << to_string(expected.op) << ","
           << "A=" << expected.A << ","
           << "B=" << expected.B << ","
           << "Y=" << expected.Y << "}";
        LIBTB_REPORT_INFO(ss.str());
      }
    }
  }
  std::deque<Expectation> expected_;
#define __declare_signal(__name, __type)        \
  sc_core::sc_signal<__type> __name##_;
  PORTS(__declare_signal)
#undef __declare_signal
  UUT uut_;
};

int sc_main(int argc, char **argv)
{
  return libtb::LibTbSim<SIMDTb>(argc, argv).start();
}
