#include "LinearProfile.hpp"

namespace motion_profile {

auto LinearProfile::GetSegment() -> Segment {
  return seg_;
}
auto LinearProfile::GetFrequency(int t) -> double {
  return CalculateFrequency(t);
}

auto LinearProfile::Parameterize(TargetConstraints constraints) -> void {
  a_max_acc_ = constraints.acceleration;
  a_max_dcc_ = constraints.deceleration;
  v_set_ = constraints.end_velocity;
  v0_ = constraints.start_velocity;
  v_ = constraints.start_velocity;
}

auto LinearProfile::CalculateFrequency(int t) -> double {
  if (v_ < v_set_) {
    a_ = a_max_acc_;
    seg_ = Segment::kRampUp;
  } else if (v_ > v_set_) {
    a_ = -a_max_dcc_;
    seg_ = Segment::kRampDown;
  } else {
    a_ = 0;
    seg_ = Segment::kDone;
    return v_set_;
  }
  v_ = v0_ + (a_ * t);
  return v_;
}
}  // namespace motion_profile
