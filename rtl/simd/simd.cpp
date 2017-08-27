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
#include "Vsimd.h"

#define PORTS(__func)                           \
    __func(pass, bool)                          \
    __func(op, uint32_t)                        \
    __func(A, uint32_t)                         \
    __func(B, uint32_t)                         \
    __func(Y_r, uint32_t)                       \
    __func(valid_r, bool)

struct SIMDTb : libtb::TopLevel
{
    static constexpr int T_N = 1000;
    using UUT = Vsimd;
    using OpT = uint32_t;
    using DataT = uint32_t;
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
        while (n--) {
            pass_ = true;
            op_ = libtb::random_integer_in_range(0, 11);
            A_ = libtb::random<DataT>();
            B_ = libtb::random<DataT>();
            expected_.push_back(compute_expected(op_, A_, B_));
            t_wait_posedge_clk();
        }
        LIBTB_REPORT_INFO("Stimulus done");
        t_wait_posedge_clk(10);
        return false;
    }
    DataT compute_expected(const OpT op, const DataT A, const DataT B) {
        return DataT{};
    }
    void m_checker() {
        if (valid_r_) {
            if (expected_.size() == 0)
                LIBTB_REPORT_ERROR("Unexpected response");

            const DataT actual = Y_r_;
            const DataT expected = expected_.front();
            expected_.pop_front();

            if (actual != expected) {
                std::stringstream ss;
                ss << "Mismatch: "
                   << "Actual:" << actual
                   << " Expected:" << expected
                    ;
                LIBTB_REPORT_ERROR(ss.str());
            }
        }
    }
    std::deque<uint32_t> expected_;
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
