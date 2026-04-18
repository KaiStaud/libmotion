#include "ConstantVelocity.hpp"

namespace motion_profile {

auto ConstantVelocityProfile::GetSegment() -> Segment {
  return Segment::kConstant;
};
auto ConstantVelocityProfile::GetFrequency(int t) -> double {
  return v_;
};
auto ConstantVelocityProfile::Parameterize(TargetConstraints constraints) -> void {
  v_ = constraints.end_velocity;
};
}  // namespace motion_profile
