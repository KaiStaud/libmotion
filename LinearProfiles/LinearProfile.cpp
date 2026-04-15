#include "LinearProfile.hpp"

namespace MotionProfile {

segment LinearProfile::getSegment() { return seg; }
double LinearProfile::get_frequency() { return v; }
void LinearProfile::initialize(double p_v0, double p_a_max_acc,
                               double p_a_max_dcc) {
  v0 = p_v0;
  v = p_v0;
  a_max_acc = p_a_max_acc;
  a_max_dcc = p_a_max_dcc;
}

double LinearProfile::calculate_frequency(double p_v_set, int t) {
  if (v < p_v_set) {
    a = a_max_acc;
    seg = segment::ramp_up;
  } else if (v > p_v_set) {
    a = -a_max_dcc;
    seg = segment::ramp_down;
  } else {
    a = 0;
    seg = segment::done;
  }

  v = v0 + a * t;
  return v;
}
} // namespace MotionProfile