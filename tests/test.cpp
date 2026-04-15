#include <gtest/gtest.h>

// #include "../TrapezoidalRamp/TrapezoidalRamp.hpp"
#include "../LinearProfiles/ConstantVelocity.hpp"
#include "../LinearProfiles/LinearProfile.hpp"

#include "rapidcsv.h"
#include "vcd_tracer.hpp"
#include <array>
#include <cmath>
#include <fstream>
#include <string>

struct TestCase {
  int t;
  int target_velocity;
  int current_velocity;
  int x;
  int s;
};

inline std::vector<TestCase> LoadCSV() {
  rapidcsv::Document doc(
      "/home/kai/projects/canopen-ws/libmotion/tests/ramp_up.csv",
      rapidcsv::LabelParams(0, -1));
  size_t rows = doc.GetRowCount();
  std::vector<TestCase> tests;
  tests.reserve(rows);
  for (size_t i = 0; i < rows; ++i) {
    TestCase tc;
    tc.t = doc.GetCell<long long>("t", i);
    tc.target_velocity = doc.GetCell<long long>("target_velocity", i);
    tc.current_velocity = doc.GetCell<long long>("current_velocity", i);
    tc.x = doc.GetCell<long long>("x", i);
    tc.s = doc.GetCell<long long>("s", i);

    tests.push_back(tc);
  }
  return tests;
}

class ParametrizedTest : public ::testing::TestWithParam<TestCase> {};

static std::vector<TestCase> g_cases = LoadCSV();
INSTANTIATE_TEST_SUITE_P(CSVDrivenTests, ParametrizedTest,
                         ::testing::ValuesIn(g_cases));

TEST(ConstantVelocity, ConstantVelocity) {
  MotionProfile::ConstantVelocityProfile constantVelocity{};
  constantVelocity.initialize(500.0);
  EXPECT_EQ(constantVelocity.get_frequency(), 500.0);
  EXPECT_EQ(constantVelocity.getSegment(), MotionProfile::segment::constant);
}

TEST_P(ParametrizedTest, LinearProfile_RampUp) {
  constexpr double start_velocity = 0;
  constexpr double acceleration = 1000;
  constexpr double deceleration = 500;

  const auto &tc = GetParam();
  MotionProfile::LinearProfile rampUp{};
  rampUp.initialize(start_velocity, acceleration, deceleration);
  EXPECT_EQ(rampUp.calculate_frequency(tc.target_velocity, tc.t),
            tc.current_velocity);
}

/*
TEST_P(ParametrizedTest,MatchesSequence)
{
    const auto& tc = GetParam();
    struct trapezoidal_ramp params;
    params.v_max =3000;
    params.a_max=1000;
    move_to(&params,30000);
    EXPECT_EQ(params.t_acc,3);
    EXPECT_EQ(params.t_const,7);
    EXPECT_EQ(params.t_dcc,3);
    EXPECT_EQ(ramp_update(&params,tc.t),tc.v);
    EXPECT_EQ(get_segment(&params,tc.t),tc.s);
}

TEST(LibMotion,Segment)
{
    struct trapezoidal_ramp params;
    params.v_max =4;
    params.a_max=2;
    move_to(&params,24);
    EXPECT_EQ(params.t_acc,2);
    EXPECT_EQ(params.t_const,4);
    EXPECT_EQ(params.t_dcc,2);
    EXPECT_EQ(ramp_update(&params,0),0);
    EXPECT_EQ(ramp_update(&params,1),2);
    EXPECT_EQ(ramp_update(&params,2),4);
    EXPECT_EQ(ramp_update(&params,3),4);
    EXPECT_EQ(ramp_update(&params,4),4);
    EXPECT_EQ(ramp_update(&params,5),4);
    EXPECT_EQ(ramp_update(&params,6),4);
    EXPECT_EQ(ramp_update(&params,7),2);
    EXPECT_EQ(ramp_update(&params,8),0);
}

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
    params.v_max =4;
    params.a_max=2;
    move_to(&params,24);
    for (int i=0;i<9;i++)
    {
        v.set(ramp_update(&params,i));
        dumper.time_update_abs(fout, std::chrono::nanoseconds{ i });
    }
}
*/