#include <gtest/gtest.h>
extern "C"
{
#include "../trapezoidal_ramp/trapezoidal_ramp.h"
#include "../constant_velocity/constant_velocity.h"
}

#include "vcd_tracer.hpp"
#include <array>
#include <cmath>
#include <string>
#include <fstream>
TEST(LibMotion,TP_RAMP)
{
    vcd_tracer::value<uint32_t>vset;
    vcd_tracer::value<uint32_t>v;
    vcd_tracer::top dumper("generators");
    {
        vcd_tracer::module analog(dumper.root, "tp_ramp");
        analog.elaborate(vset, "set_speed");
        analog.elaborate(v,"actual_speed");
    }        
    std::ofstream fout("tp_ramp.vcd");
    dumper.finalize_header(fout,std::chrono::system_clock::from_time_t(0));
        
    vset.set(4);
    v.set(0);
    struct trapezoidal_ramp params;
    params.v_max =4;
    params.a_max=2;
    move_to(&params,10);
    for (int i=0;i<5;i++)
    {
        v.set(ramp_update(&params,i));
        dumper.time_update_abs(fout, std::chrono::nanoseconds{ i });
    }
}