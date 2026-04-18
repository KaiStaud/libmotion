#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "../LinearProfiles/ConstantVelocity.hpp"
#include "../LinearProfiles/LinearProfile.hpp"
#include "../TrapezoidalRamp/TrapezoidalRamp.hpp"
#include "../Homing/Homing.hpp"
#include "magic_enum.hpp"
#include "rapidcsv.h"
#include "vcd_tracer.hpp"

struct TestCase {
  int t;
  double target_velocity;
  double current_velocity;
  double x;
  double s;
};

struct HomingTestCase {
  int t;
  int ControlWord;
  int ModeOfOperation;
  int Input;
  int Output;
  motion_profile::homing::Progress Progress;
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

inline auto loadHomingCSV(const std::string& path) -> std::vector<HomingTestCase> {
  rapidcsv::Document doc(path, rapidcsv::LabelParams(0, -1));
  size_t rows = doc.GetRowCount();
  std::vector<HomingTestCase> tests;
  tests.reserve(rows);
  for (size_t i = 0; i < rows; ++i) {
    HomingTestCase tc{};
    tc.t = doc.GetCell<int>("t", i);
    tc.ControlWord = doc.GetCell<int>("ControlWord", i);
    tc.ModeOfOperation = doc.GetCell<int>("ModeOfOperation", i);
    tc.Input = doc.GetCell<int>("Input", i);
    tc.Output = doc.GetCell<int>("Output", i);
    auto progress_string=doc.GetCell<std::string>("Progress", i);
    tc.Progress = magic_enum::enum_cast<motion_profile::homing::Progress>(progress_string).value();
    tests.push_back(tc);
  }
  return tests;
}

class LinearProfileTestBase {
 protected:
  static constexpr double acceleration = 1000;
  static constexpr double deceleration = 500;

  void runTest(const std::string& filename, double start_velocity, double target_velocity) {
    motion_profile::LinearProfile ramp{};
    motion_profile::TargetConstraints constraints{.acceleration = acceleration,
                                                  .deceleration = deceleration,
                                                  .end_velocity = target_velocity,
                                                  .start_velocity = start_velocity};
    ramp.Parameterize(constraints);

    auto cases = loadCSV(filename);

    for (const auto& testcase : cases) {
      EXPECT_EQ(ramp.CalculateFrequency(testcase.t), testcase.current_velocity);
    }
  }
};

class ParameterizedRampUpTest : public ::testing::TestWithParam<std::string>,
                                protected LinearProfileTestBase {};

class ParameterizedRampDownTest : public ::testing::TestWithParam<std::string>,
                                  protected LinearProfileTestBase {};

class ParameterizedTrapezoidalTest : public ::testing::TestWithParam<std::string>,
                                     protected LinearProfileTestBase {};

INSTANTIATE_TEST_SUITE_P(RampUpTests, ParameterizedRampUpTest, ::testing::Values("ramp_up.csv"));

INSTANTIATE_TEST_SUITE_P(RampDownTests, ParameterizedRampDownTest,
                         ::testing::Values("ramp_down.csv"));

INSTANTIATE_TEST_SUITE_P(TrapezoidalTests, ParameterizedTrapezoidalTest,
                         ::testing::Values("trapezoidal.csv"));

TEST(ConstantVelocity, ConstantVelocity) {
  constexpr double start_velocity = 500.0;
  motion_profile::ConstantVelocityProfile constant_velocity{};
  motion_profile::TargetConstraints constraints{.acceleration = 0.0,
                                                .deceleration = 0.0,
                                                .end_velocity = start_velocity,
                                                .start_velocity = start_velocity};
  constant_velocity.Parameterize(constraints);

  EXPECT_EQ(constant_velocity.GetFrequency(0), 500.0);
  EXPECT_EQ(constant_velocity.GetSegment(), motion_profile::Segment::kConstant);
}

TEST_P(ParameterizedRampUpTest, LinearProfile_RampUp) {
  constexpr double start_velocity = 0;
  constexpr double target_velocity = 3000;

  runTest(GetParam(), start_velocity, target_velocity);
}

TEST_P(ParameterizedRampDownTest, LinearProfile_RampDown) {
  constexpr double start_velocity = 4000;
  constexpr double target_velocity = 0;

  runTest(GetParam(), start_velocity, target_velocity);
}

TEST_P(ParameterizedTrapezoidalTest, TrapezoidalProfile) {
  constexpr double start_velocity = 0;
  constexpr double constant_velocity = 3000.0;
  motion_profile::TargetConstraints constraints{.acceleration = acceleration,
                                                .deceleration = deceleration,
                                                .end_velocity = constant_velocity,
                                                .start_velocity = 0.0};

  constexpr double target_position = 30000;
  motion_profile::TrapezoidalRamp trapezoidal_ramp(constraints);
  trapezoidal_ramp.CalculateTimes(target_position);
  auto cases = loadCSV(GetParam());

  for (const auto& testcase : cases) {
    EXPECT_EQ(trapezoidal_ramp.GetFrequency(testcase.t), testcase.current_velocity)
        << "failed at t= " << testcase.t;
  }
}

/* Homing Tests */
/* Mock Hardware States */
uint32_t controlword_ = 0;
uint32_t mode_of_operation_ = 0;
bool input_ = 0;
bool output_ = 0;
uint8_t progress_ = 0;

/* Hardware IO-Functions */
auto read_gpo()-> bool{
return input_;
}

auto read_object(uint16_t index,uint8_t subindex) -> uint32_t //NOLINT
{
  constexpr int index_controlword = 0x6060;
  constexpr int index_homing_method = 0x6098;

  if (index == index_controlword){
    return controlword_;
  }
  else if(index == index_homing_method){
    return mode_of_operation_;
  }
  return 0;
}

auto enable_drive() -> void{
output_ = true;
}

auto disable_drive() -> void{
output_ = false;
}

TEST(LibMotion,Homing)
{
  auto cases = loadHomingCSV("homing.csv");
motion_profile::homing::HomingController ctrl(
    read_object,
    read_gpo,
    disable_drive,
    enable_drive);

    vcd_tracer::value<uint32_t>controlword;
    vcd_tracer::value<uint32_t>mode_of_operation;
    vcd_tracer::value<bool>input;
    vcd_tracer::value<bool>output;
    vcd_tracer::value<uint8_t>progress;
    vcd_tracer::top dumper("homing");
    {
        vcd_tracer::module digital(dumper.root, "homing");
        digital.elaborate(controlword, "controlword");
        digital.elaborate(mode_of_operation, "mode_of_operation");
        digital.elaborate(input,"input");
        digital.elaborate(output,"output");
        digital.elaborate(progress,"progress");

    }
    std::ofstream fout("homing.vcd");
    dumper.finalize_header(fout,std::chrono::system_clock::from_time_t(0));

  for (const auto& testcase : cases) {
    // Signals are valid between two absolute time updates.
    // Therefore time_update_abs needs to be called before assigning signals.
    dumper.time_update_abs(fout, std::chrono::nanoseconds{ testcase.t });
    // **First** write the hardware state!
    // progress and output are not assigned. They will be calculated from ctrl.Update() !
    input_ = testcase.Input;
    controlword_ = testcase.ControlWord;
    mode_of_operation_ = testcase.ModeOfOperation;
    // *Then** update the dump
    input.set(input_);
    controlword.set(controlword_);
    mode_of_operation.set(mode_of_operation_);
    auto tprogress = ctrl.Update();
    progress_ = magic_enum::enum_integer(tprogress);
    progress.set(progress_); // **Only** valid after Update()
    output.set(output_);

    EXPECT_EQ(tprogress, testcase.Progress)
        << "failed at t= " << testcase.t;
      }
}
