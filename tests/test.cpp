#include <gtest/gtest.h>

#include <string>

#include "../LinearProfiles/ConstantVelocity.hpp"
#include "../LinearProfiles/LinearProfile.hpp"
#include "../TrapezoidalRamp/TrapezoidalRamp.hpp"
#include "rapidcsv.h"
#include "vcd_tracer.hpp"

struct TestCase {
  int t;
  double target_velocity;
  double current_velocity;
  double x;
  double s;
};

inline auto loadCSV(const std::string& path) -> std::vector<TestCase> {
  rapidcsv::Document doc(path, rapidcsv::LabelParams(0, -1));
  size_t rows = doc.GetRowCount();
  std::vector<TestCase> tests;
  tests.reserve(rows);
  for (size_t i = 0; i < rows; ++i) {
    TestCase tc{};
    tc.t = doc.GetCell<int>("t", i);
    tc.target_velocity = doc.GetCell<double>("target_velocity", i);
    tc.current_velocity = doc.GetCell<double>("current_velocity", i);
    tc.x = doc.GetCell<double>("x", i);
    tc.s = doc.GetCell<double>("s", i);

    tests.push_back(tc);
  }
  return tests;
}

class LinearProfileTestBase {
 protected:
  static constexpr double acceleration = 1000;
  static constexpr double deceleration = 500;

  void runTest(const std::string& filename, double start_velocity) {
    motion_profile::LinearProfile ramp{};
    ramp.initialize(start_velocity, acceleration, deceleration);

    auto cases = loadCSV(filename);

    for (const auto& testcase : cases) {
      EXPECT_EQ(ramp.calculateFrequency(testcase.target_velocity, testcase.t),
                testcase.current_velocity);
    }
  }
};

class ParameterizedRampUpTest : public ::testing::TestWithParam<std::string>,
                                protected LinearProfileTestBase {};

class ParameterizedRampDownTest : public ::testing::TestWithParam<std::string>,
                                  protected LinearProfileTestBase {};

INSTANTIATE_TEST_SUITE_P(RampUpTests, ParameterizedRampUpTest, ::testing::Values("ramp_up.csv"));

INSTANTIATE_TEST_SUITE_P(RampDownTests, ParameterizedRampDownTest,
                         ::testing::Values("ramp_down.csv"));

TEST(ConstantVelocity, ConstantVelocity) {
  constexpr double start_velocity = 500.0;
  motion_profile::ConstantVelocityProfile constant_velocity{};
  constant_velocity.initialize(start_velocity);
  EXPECT_EQ(constant_velocity.getFrequency(), 500.0);
  EXPECT_EQ(constant_velocity.getSegment(), motion_profile::segment::constant);
}

TEST_P(ParameterizedRampUpTest, LinearProfile_RampUp) {
  constexpr double start_velocity = 0;
  runTest(GetParam(), start_velocity);
}

TEST_P(ParameterizedRampDownTest, LinearProfile_RampDown) {
  constexpr double start_velocity = 4000;
  runTest(GetParam(), start_velocity);
}
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
