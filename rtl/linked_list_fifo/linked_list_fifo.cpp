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
#include "vobj/Vlinked_list_fifo.h"

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

const int ID_N = 4;
const int PTR_N = 255;

struct CmdXActorIntf : sc_core::sc_interface {
  virtual void idle() = 0;
  virtual void push(uint32_t id, uint32_t data) = 0;
  virtual uint32_t pop(uint32_t id) = 0;
};

struct CmdXActor : CmdXactorIntf, sc_core::sc_module {
  //
  sc_core::sc_in<bool> clk_;
  sc_core::sc_in<bool> rst_;
  //
  sc_core::sc_out<bool> cmd_pass_;
  sc_core::sc_out<bool> cmd_push_;
  sc_core::sc_out<uint32_t> cmd_id_;
  sc_core::sc_out<uint32_t> cmd_push_data_;
  sc_core::sc_in<uint32_t> cmd_pop_data_;
  sc_core::sc_out<bool> clear_;
  sc_core::sc_in<bool> full_r_;
  sc_core::sc_in<bool> empty_r_;
  sc_core::sc_in<uint32_t> nempty_r_;
  sc_core::sc_in<bool> busy_r_;

  CmdXActor(sc_core::sc_module_name mn = "t") { sampler_.clk(clk_); }
  void idle() {
    cmd_pass_ = false;
    cmd_push_ = false;
    cmd_id_ = 0;
    cmd_push_data_ = 0;
  }
  bool is_full() {
    sampler_.wait();
    return full_r_;
  }
  uint32_t non_empty_id() {
    sampler_.wait();
    return libtb2::pickone(libtb2::mask_bits(nempty_r_.read(), ID_N));
  }
  uint32_t id() {
    return 0;
  }
  bool is_empty(uint32_t id) {
    sampler_.wait();
    return empty_r_;
  }
  void push(uint32_t id, uint32_t data) {
    cmd_pass_ = true;
    cmd_push_ = true;
    cmd_id_ = id;
    cmd_push_data_ = data;
    wait(cmd.posedge_event());
    LOGGER(DEBUG) << "Push ID = " << id << " Push Data = " << data << "\n";
    LIBTB_ERROR_ON(full_r_);
    q_[id].push_back(data);
    idle();
  }
  uint32_t pop(uint32_t id) {
    cmd_pass_ = true;
    cmd_push_ = true;
    cmd_id_ = id;
    wait(clk_.posedge_event());
    wait(busy_r_.negedge_event());
    sampler_.wait();
    LOGGER(DEBUG) << "Pop ID = " << id << " Pop Data = " << cmd_pop_data << "\n";
    LIBTB_ERROR_ON(q_[id].empty());
    const uint32_t expected = q_[id].front(); q_[id].pop_front();
    const uint32_t actual = cmd_pop_data_;
    idle();
    LIBTB2_ERROR_ON(expected != actual);
    return actual;
  }
private:
  std::deque<uint32_t> q_[ID_N];
  libtb2::Sampler sampler_;
};

struct LinkedListFifoTb : libtb2::Top<LinkedListFifoTb> {
  typedef Vlinked_list_fifo uut_t;
  sc_core::sc_port<CmdXActorIntf> xact_;
  SC_HAS_PROCESS(LinkedListFifoTb);
  LinkedListFifoTb(sc_core::sc_module_name mn = "t")
    : uut_("uut") {
    //
    resetter_.clk(clk_);
    resetter_.rst(rst_);
    //
    sampler_.clk(clk_);
    //
    wd_.clk(clk_);
    //
    xact_.bind(x_);
    //
    x_.clk(clk_);
    x_.rst(rst_);
#define __bind_ports(__name, __type)            \
    x_.__name(__name ## _);
    PORTS(__bind_ports)
#undef __bind_ports
    //
    uut_.clk(clk_);
    uut_.rst(rst_);
#define __bind_ports(__name, __type)            \
    uut_.__name(__name ## _);
    PORTS(__bind_ports)
#undef __bind_ports
    SC_THREAD(t_stimulus);
  }
private:
  void t_stimulus() {
    scv_smart_ptr<bool> ppush;
    scv_smart_ptr<uint32_t> ppush_data;
    
    xact_->idle();
    resetter_.wait();
    while (true) {
      ppush->next();
      if (*ppush) {
        ppush_data->next();

        if (!xact_->is_full())
          xact_->push(xact_->id(), *ppush_data);
      } else {

        if (!xact_->is_empty())
          xact_->pop(xact_->non_empty_id());
      }
    }
  }
  CmdXActor x_;
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
SC_MODULE_EXPORT(LinkedListFifoTb);

int sc_main(int argc, char **argv) {
  LinkedListFifoTb tb;
  return libtb2::Sim::start(argc, argv);
}
