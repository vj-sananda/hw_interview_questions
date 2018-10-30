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

#include <libtb2.hpp>
#include <vector>
#include <iostream>

#include "vobj/Vread_modify_write.h"

#define PORTS(__func)                           \
  __func(in_pass, bool)                         \
  __func(in_inst, uint32_t)                     \
  __func(in_imm, uint32_t)                      \
  __func(in_accept, bool)                       \
  __func(out_vld_r, bool)                       \
  __func(out_wa_r, uint32_t)                    \
  __func(out_wdata_r, uint32_t)                 \
  __func(replay_s4_req, bool)                   \
  __func(replay_s8_req, bool)                   \
  __func(stall_req, uint32_t)

typedef Vread_modify_write uut_t;

struct CombStallTb : libtb2::Top<uut_t> {
  SC_HAS_PROCESS(CombStallTb);
  CombStallTb(sc_core::sc_module_name mn = "t")
    : uut_("uut")
#define __construct_ports(__name, __type)       \
      , __name ## _(#__name)
    PORTS(__construct_ports)
#undef __construct_ports
    , clk_("clk")
    , rst_("rst")
  {
    uut_.clk(clk_);
    uut_.rst(rst_);
#define __bind_signals(__name, __type)          \
    uut_.__name(__name ## _);
    PORTS(__bind_signals)
#undef __bind_signals

    resetter_.clk(clk_);
    resetter_.rst(rst_);
    
    sampler_.clk(clk_);

    generate_stimulus(1024);

    SC_THREAD(t_in);
    
    SC_METHOD(m_out);
    sensitive << sampler_.sample();
    dont_initialize();

    stall_n_ = 0;
    SC_METHOD(m_stall);
    sensitive << clk_.posedge_event();

    SC_THREAD(t_replay);

    register_uut(uut_);
    vcd_on();
  }
private:
  void t_replay() {
    scv_smart_ptr<bool> replay;
    scv_bag<bool> replay_bag;
    replay_bag.add(true, 1);
    replay_bag.add(false, 99);
    replay->set_mode(replay_bag);
    
    replay_s8_req_ = false;
    replay_s4_req_ = false;
    while (true) {
      replay->next();
      replay_s8_req_ = *replay;

      replay->next();
      replay_s4_req_ = *replay;
      
      wait(clk_.posedge_event());
      replay_s8_req_ = false;
      replay_s4_req_ = false;
    }
  }
  void m_stall() {
    if ((++stall_n_ % 10) == 0) {
      stall_req_ = (1 << (*stall_ptr_ % 10));

      stall_ptr_->next();
    } else {
      stall_req_ = 0;
    }
  }
  void generate_stimulus(std::size_t n = 10) {
    scv_smart_ptr<uint32_t> rnd;
    stimulus_.clear();
    i_ = 0;
    j_ = 0;
    while (n-- != 0) {
      rnd->next();
      stimulus_.push_back(*rnd);
    }
  }
  void t_in() {
    in_pass_ = false;
    resetter_.wait_reset_done();

    wait(100, SC_NS);
    sc_core::sc_stop();
  }
  void m_out() {
  }
  libtb2::Resetter resetter_;
  libtb2::Sampler sampler_;
  std::vector<uint32_t> stimulus_;
  std::size_t i_, j_;
  sc_core::sc_clock clk_;
  sc_core::sc_signal<bool> rst_;
#define __declare_signal(__name, __type)        \
  sc_core::sc_signal<__type> __name##_;
  PORTS(__declare_signal)
#undef __declare_signal
  std::size_t stall_n_;
  scv_smart_ptr<uint32_t> stall_ptr_;
  uut_t uut_;
};
SC_MODULE_EXPORT(CombStallTb);

int sc_main(int argc, char **argv) {
  CombStallTb tb("tb");
  return libtb2::Sim::start(argc, argv);
}
