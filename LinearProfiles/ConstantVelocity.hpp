#pragma once
#include "MotionStrategy.hpp"

namespace MotionProfile {

class ConstantVelocityProfile : public MotionStrategy {
public:
  segment getSegment();
  double get_frequency();
  void initialize(double p_v);

private:
  double v;
};
} // namespace MotionProfile
