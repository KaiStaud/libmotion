#include "ConstantVelocity.hpp"

namespace MotionProfile {

  segment ConstantVelocityProfile::getSegment(){return segment::constant;};
  double ConstantVelocityProfile::get_frequency(){return v;};
  void ConstantVelocityProfile::initialize(double p_v){v=p_v;};
} // namespace MotionProfile