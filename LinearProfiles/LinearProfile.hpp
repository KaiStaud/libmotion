#pragma once
#include "MotionStrategy.hpp"

namespace motion_profile {

class LinearProfile : public MotionStrategy {
 public:
  auto GetSegment() -> Segment override;
  auto GetFrequency(int t) -> double override;
  /*
  @param p_v_set : target velocity in Hz
  @param t : elapsed time after start in ticks
  @return : frequency after t
  */
  auto CalculateFrequency(int t) -> double;
  auto Parameterize(TargetConstraints constraints) -> void override;

 private:
  double t_acc_ = 0;
  double v_ = 0;
  double a_max_acc_ = 0;
  double a_max_dcc_ = 0;

  double v_set_ = 0;
  double v0_ = 0;
  double a_ = 0;
  Segment seg_ = Segment::kDone;
};
}  // namespace motion_profile
