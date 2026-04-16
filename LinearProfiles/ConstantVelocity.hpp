#pragma once
#include "MotionStrategy.hpp"

namespace motion_profile {

class ConstantVelocityProfile : public MotionStrategy {
 public:
  auto getSegment() -> segment override;
  auto getFrequency() -> double override;
  void initialize(double p_v);

 private:
  double v_{};
};
}  // namespace motion_profile
