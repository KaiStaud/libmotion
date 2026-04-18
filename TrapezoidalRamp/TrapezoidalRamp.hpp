#pragma once
#include <memory>
#include <cmath>
#include "../LinearProfiles/MotionStrategy.hpp"
#include "../LinearProfiles/LinearProfile.hpp"
#include "../LinearProfiles/ConstantVelocity.hpp"
namespace motion_profile {

class TrapezoidalRamp {
 public:

 TrapezoidalRamp(motion_profile::TargetConstraints constraints) : constraints_(constraints),
    ramp_up_(std::make_unique<motion_profile::LinearProfile>()),
    ramp_down_(std::make_unique<motion_profile::LinearProfile>()),
    constant_velocity_(std::make_unique<motion_profile::ConstantVelocityProfile>())
    {
    constexpr double constant_velocity = 3000.0;
        const motion_profile::TargetConstraints constraints_up{
        .acceleration = constraints.acceleration,
        .deceleration = constraints.deceleration,
        .end_velocity = constraints.end_velocity,
        .start_velocity = 0.0
      };

        const motion_profile::TargetConstraints constraints_down{
        .acceleration = constraints.acceleration,
        .deceleration = constraints.deceleration,
        .end_velocity = constraints.start_velocity,
        .start_velocity = constraints.end_velocity
      };

        const motion_profile::TargetConstraints constraints_constant{
        .acceleration = 0.0,
        .deceleration = 0.0,
        .end_velocity = constant_velocity,
        .start_velocity = constant_velocity
      };

      ramp_up_->Parameterize(constraints_up);
      ramp_down_->Parameterize(constraints_down);
      constant_velocity_->Parameterize(constraints_constant);
 }

  auto GetFrequency(int t) -> double{
  double rpm =0;
  switch (GetSegment(t)) {
    case Segment::kRampUp:
      rpm = ramp_up_->GetFrequency(t);
      break;
    case Segment::kConstant:
      rpm = constant_velocity_->GetFrequency(t);
      break;
    case Segment::kRampDown:
      rpm = ramp_down_->GetFrequency(t-(t_accelerate_ + t_const_));
      break;
    default:
      rpm = 0;
      break;
  }
  velocity_ = rpm;
  return velocity_;
}

 auto CalculateTimes(double target_position) -> void {
  t_accelerate_ = static_cast<int>(constraints_.end_velocity / constraints_.acceleration);
  double p_acc = pow(t_accelerate_, 2) * constraints_.acceleration;
  t_const_ = static_cast<int>((target_position - p_acc)/constraints_.end_velocity);
  t_decellerate_ = static_cast<int>(constraints_.end_velocity / constraints_.deceleration);
 }

 auto GetSegment(uint32_t t) -> Segment {
  if (t <= t_accelerate_) {
    return Segment::kRampUp;
  } else if (t < (t_accelerate_ + t_const_)) {
    return Segment::kConstant;
  } else if ((t >= (t_accelerate_ + t_const_)) && ((t <= (t_accelerate_ + t_decellerate_ + t_const_)))){
    return Segment::kRampDown;
  }
  else{
    return Segment::kDone;
  }
}
 private:

  std::unique_ptr<MotionStrategy> ramp_up_;
  std::unique_ptr<MotionStrategy> constant_velocity_;
  std::unique_ptr<MotionStrategy> ramp_down_;
  Segment segment_=Segment::kDone;
  motion_profile::TargetConstraints constraints_={};
  int t_accelerate_ = 0;
  int t_decellerate_ = 0;
  int t_const_ = 0;
  double velocity_ = 0;
};
}  // namespace motion_profile
