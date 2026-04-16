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

inline std::vector<TestCase> LoadCSV(const std::string& path) {
  rapidcsv::Document doc(
      path,
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

class LinearProfileTestBase {
protected:
  static constexpr double acceleration = 1000;
  static constexpr double deceleration = 500;

  void run_test(const std::string& filename, double start_velocity) {
    MotionProfile::LinearProfile ramp{};
    ramp.initialize(start_velocity, acceleration, deceleration);

    auto cases = LoadCSV(filename);

    for (const auto &tc : cases) {
      EXPECT_EQ(ramp.calculate_frequency(tc.target_velocity, tc.t),
                tc.current_velocity);
    }
  }
};

class ParameterizedRampUpTest
    : public ::testing::TestWithParam<std::string>,
      protected LinearProfileTestBase {};

class ParameterizedRampDownTest
    : public ::testing::TestWithParam<std::string>,
      protected LinearProfileTestBase {};

INSTANTIATE_TEST_SUITE_P(
    RampUpTests,
    ParameterizedRampUpTest,
    ::testing::Values("ramp_up.csv")
);

INSTANTIATE_TEST_SUITE_P(
    RampDownTests,
    ParameterizedRampDownTest,
    ::testing::Values("ramp_down.csv")
);

TEST(ConstantVelocity, ConstantVelocity) {
  MotionProfile::ConstantVelocityProfile constantVelocity{};
  constantVelocity.initialize(500.0);
  EXPECT_EQ(constantVelocity.get_frequency(), 500.0);
  EXPECT_EQ(constantVelocity.getSegment(), MotionProfile::segment::constant);
}

TEST_P(ParameterizedRampUpTest, LinearProfile_RampUp) {

  constexpr double start_velocity = 0;
  run_test(GetParam(), start_velocity);
}

TEST_P(ParameterizedRampDownTest, LinearProfile_RampDown) {

  constexpr double start_velocity = 4000;
  run_test(GetParam(), start_velocity);
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