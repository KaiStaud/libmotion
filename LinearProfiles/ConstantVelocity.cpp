#include "ConstantVelocity.hpp"

namespace motion_profile {

segment ConstantVelocityProfile::getSegment() {
  return segment::constant;
};
double ConstantVelocityProfile::getFrequency() {
  return v_;
};
void ConstantVelocityProfile::initialize(double p_v) {
  v_ = p_v;
};
}  // namespace motion_profile
