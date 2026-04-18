/*
VcdHAL implements a pipe between your test parameters and

output_value read(int input_value) : returns input value and writes it to vcd file.
void write(int output_value) : writes output_value to vcd

*/

/*
TEST(LibMotion,TrapezoidalRamp)
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
    params.end_velocity =4;
    params.a_max=2;
    move_to(&params,24);
    for (int i=0;i<9;i++)
    {
        v.set(ramp_update(&params,i));
        dumper.time_update_abs(fout, std::chrono::nanoseconds{ i });
    }
}
*/
