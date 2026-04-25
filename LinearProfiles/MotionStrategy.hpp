#pragma once
#include <cstdint>

namespace motion_profile {
enum class Segment : uint8_t {
  kRampUp = 0,
  kConstant = 1,
  kRampDown = 2,
  kDone = 3,
};

struct TargetConstraints {
  double acceleration;   /* acceleration */
  double deceleration;   /* deceleration w/o sign */
  double end_velocity;   /* end velocity */
  double start_velocity; /* start velocity */
};

class MotionStrategy {
 public:
  virtual auto GetSegment() -> Segment = 0;
  virtual auto GetFrequency(int t) -> double = 0;
  virtual auto Parameterize(TargetConstraints constraints) -> void = 0;

  virtual ~MotionStrategy() = default;
  MotionStrategy(MotionStrategy&&) = default;
  MotionStrategy() = default;
  MotionStrategy(const MotionStrategy&) = delete;
  auto operator=(const MotionStrategy&) -> MotionStrategy& = delete;
};

}  // namespace motion_profile
