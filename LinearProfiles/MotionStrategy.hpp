#pragma once
#include <stdint.h>

namespace MotionProfile {
enum class segment {
  ramp_up = 0,
  constant = 1,
  ramp_down = 2,
  done = 3,
};

class MotionStrategy {

  segment getSegment() = delete;
  double get_frequency() = delete;
};

} // namespace MotionProfile