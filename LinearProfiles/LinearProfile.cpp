#include "LinearProfile.hpp"

namespace motion_profile {

segment LinearProfile::getSegment() {
  return seg_;
}
double LinearProfile::getFrequency() {
  return v_;
}
void LinearProfile::initialize(double p_v0, double p_a_max_acc, double p_a_max_dcc) {
  v0_ = p_v0;
  v_ = p_v0;
  a_max_acc_ = p_a_max_acc;
  a_max_dcc_ = p_a_max_dcc;
}

double LinearProfile::calculateFrequency(double p_v_set, int t) {
  if (v_ < p_v_set) {
    a_ = a_max_acc_;
    seg_ = segment::ramp_up;
  } else if (v_ > p_v_set) {
    a_ = -a_max_dcc_;
    seg_ = segment::ramp_down;
  } else {
    a_ = 0;
    seg_ = segment::done;
  }

  v_ = v0_ + (a_ * t);
  return v_;
}
}  // namespace motion_profile
