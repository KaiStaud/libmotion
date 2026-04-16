#pragma once
#include <cstdint>

namespace motion_profile {
enum class segment : uint8_t {
  ramp_up = 0,
  constant = 1,
  ramp_down = 2,
  done = 3,
};

class MotionStrategy {
 public:
  virtual segment getSegment() = 0;   // NOLINT
  virtual double getFrequency() = 0;  // NOLINT

  virtual ~MotionStrategy() = default;
  MotionStrategy(MotionStrategy&&) = default;
  MotionStrategy() = default;
  MotionStrategy(const MotionStrategy&) = delete;
  auto operator=(const MotionStrategy&) -> MotionStrategy& = delete;
};

}  // namespace motion_profile
