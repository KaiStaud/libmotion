#pragma once
#include "MotionStrategy.hpp"

namespace motion_profile {

class ConstantVelocityProfile : public MotionStrategy {
 public:
  auto GetSegment() -> Segment override;
  auto GetFrequency(int t) -> double override;
  auto Parameterize(TargetConstraints constraints) -> void override;

 private:
  double v_ = 0;
};
}  // namespace motion_profile
