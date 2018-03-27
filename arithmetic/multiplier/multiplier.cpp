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

#include <libtb2.hpp>
#include <deque>
#include "vobj/Vmultiplier.h"

typedef Vmultiplier uut_t;

#define PORTS(__func)                           \
  __func(a, uint32_t)                           \
  __func(b, uint32_t)                           \
  __func(pass, bool)                            \
  __func(y, vluint64_t)                         \
  __func(y_vld_r, bool)                         \
  __func(busy_r, bool)

class MultiplierTb : libtb2::Top<uut_t> {

  struct Expect {
    Expect(uint32_t a, uint32_t b)
        : a(a), b(b)
    {}
    uint32_t a, b;
  };
  
 public:
  SC_HAS_PROCESS(MultiplierTb);
  MultiplierTb(sc_core::sc_module_name mn = "t")
      : uut_("uut")
      , clk_("clk")
      , rst_("rst")
#define __construct_ports(__name, __type)       \
      , __name ## _ (#__name )
      PORTS(__construct_ports)
#undef __construct_ports
  {
    //
    resetter_.clk(clk_);
    resetter_.rst(rst_);
    //
    sampler_.clk(clk_);
    //
    wd_.clk(clk_);
    //
    uut_.clk(clk_);
    uut_.rst(rst_);
#define __bind_ports(__name, __type)            \
    uut_.__name(__name ## _);
    PORTS(__bind_ports)
#undef __bind_ports

    SC_THREAD(t_stimulus);
    SC_METHOD(m_checker);
    sensitive << sampler_.sample();
    dont_initialize();
    register_uut(uut_);
    vcd_on();
  }
 private:
  void t_stimulus() {
    struct stimulus_constraint : scv_constraint_base {
      scv_smart_ptr<uint32_t> v;
      SCV_CONSTRAINT_CTOR(stimulus_constraint) {
        SCV_CONSTRAINT(v() < 1024);
      }
    } a("a_constrained"), b("b_constrained");
    
    resetter_.wait_reset_done();
    while (true) {
      LIBTB2_ASSERT(!busy_r_);

      a.next();
      b.next();

      const uint32_t actual_a = *a.v;
      const uint32_t actual_b = *b.v;
      
      a_ = actual_a;
      b_ = actual_b;
      pass_ = true;
      expected_.push_back(Expect(actual_a, actual_b));
      wait_cycles();
      pass_ = false;
      wait_cycles(5);
    }
  }
  void m_checker() {
    if (y_vld_r_) {
      const Expect e = expected_.front(); expected_.pop_front();
      const vluint64_t actual = y_;

      const vluint64_t expected = (
          static_cast<vluint64_t>(e.a) * static_cast<vluint64_t>(e.b));
      if (actual != expected) {
        LOGGER(ERROR) << "Validating: a=" << e.a << " b=" << e.b
                      << "\tMismatch detected:"
                      << " expected= " << expected
                      << " actual= " << actual
                      << "\n";
      } else {
        LOGGER(INFO) << "Validating: a=" << e.a << " b=" << e.b
                     << "\tValidated! y=" << expected << "\n";
      }
    }
  }
  void wait_cycles(std::size_t cycles = 1) {
    while (cycles--)
      wait(clk_.posedge_event());
  }
  std::deque<Expect> expected_;
  sc_core::sc_clock clk_;
  sc_core::sc_signal<bool> rst_;
#define __declare_signal(__name, __type)        \
  sc_core::sc_signal<__type> __name##_;
  PORTS(__declare_signal)
#undef __declare_signal
  libtb2::Resetter resetter_;
  libtb2::Sampler sampler_;
  libtb2::SimWatchDogCycles wd_;
  uut_t uut_;
};

int sc_main(int argc, char ** argv) {
  MultiplierTb tb;
  return libtb2::Sim::start(argc, argv);
}
