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

#include <libtb2.hpp>
#include <deque>

#include "vobj/Vpack.h"

#define PORTS(__func)                           \
  __func(in_w, sc_bv<256>)                      \
  __func(in_vld_w, uint32_t)                    \
  __func(out_r, sc_bv<256>)                     \
  __func(out_vld_r, uint32_t)

struct PackTb : libtb2::Top<Vpack> {
  struct Expectation {
    uint32_t x [8];
    std::size_t n;
  };
  
  SC_HAS_PROCESS(PackTb);
  PackTb(sc_core::sc_module_name mn = "t")
      : uut_("uut")
#define __construct_ports(__name, __type)       \
        , __name ## _(#__name)
        PORTS(__construct_ports)
#undef __construct_ports
      , clk_("clk")
      , rst_("rst") {
    uut_.clk(clk_);
    uut_.rst(rst_);
#define __bind_signals(__name, __type)          \
    uut_.__name(__name ## _);
    PORTS(__bind_signals)
#undef __bind_signals

    resetter_.clk(clk_);
    resetter_.rst(rst_);

    sampler_.clk(clk_);
    
    SC_METHOD(m_check_output);
    sensitive << sampler_.sample();
    dont_initialize();

    SC_THREAD(t_stimulus);

    generate_stimulus(1024);
    register_uut(uut_);
    vcd_on();
  };
 private:
  void generate_stimulus(std::size_t cnt = 16) {
    struct n_constraint : scv_constraint_base {
      scv_smart_ptr<uint32_t> n;
      SCV_CONSTRAINT_CTOR(n_constraint) {
        SCV_CONSTRAINT((n() >= 0) && (n() < 8));
      }
    } n_constraint_c("n_constraint");
    
    scv_smart_ptr<uint32_t> x_ptr;

    while (cnt-- != 0) {
      Expectation e;

      e.n = *n_constraint_c.n;
      for (uint32_t i = 0; i < e.n; i++, x_ptr->next()) {
        e.x [i] = *x_ptr;
      }
      n_constraint_c.next();

      expect_.push_back(e);
    }
  }
  void t_stimulus() {
    in_w_ = 0;
    in_vld_w_ = 0;

    for (std::size_t i = 0; i < expect_.size(); i++) {
      const Expectation & e = expect_[i];

      uint32_t val_in_vld_w = 0;
      sc_bv<256> val_in_w;
      for (int j = 0; j < e.n; j++) {
        val_in_vld_w |= (1 << j);

        const int msb = (j + 1) * 32 - 1;
        const int lsb = j * 32;
        val_in_w.range(msb, lsb) = e.x[j];

      }
      in_w_ = val_in_w;
      in_vld_w_ = val_in_vld_w;
      wait(clk_.posedge_event());
      in_vld_w_ = 0;
    }

    wait(100, SC_NS);
    sc_core::sc_stop();
  }
  void m_check_output() {
    if (out_vld_r_) {
      LIBTB2_ERROR_ON(expect_.size() == 0);

      std::size_t n = libtb2::popcount(out_vld_r_.read());

      const Expectation & e = expect_.front();
      LIBTB2_ERROR_ON(n != e.n);
      for (int i = 0; i < e.n; i++) {

        LIBTB2_ERROR_ON(
            out_r.range((i + 1) * 32 - 1, i * 32) != e.x [i]);
      }
      expect_.pop_front();
    }
  }
  libtb2::Resetter resetter_;
  libtb2::Sampler sampler_;
  sc_core::sc_clock clk_;
  sc_core::sc_signal<bool> rst_;
#define __declare_signal(__name, __type)        \
  sc_core::sc_signal<__type> __name##_;
  PORTS(__declare_signal)
#undef __declare_signal
  std::deque<Expectation> expect_;
  Vpack uut_;
};
SC_MODULE_EXPORT(PackTb);

int sc_main(int argc, char **argv) {
  PackTb tb("tb");
  return libtb2::Sim::start(argc, argv);
}
