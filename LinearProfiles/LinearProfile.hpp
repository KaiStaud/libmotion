#pragma once
#include "MotionStrategy.hpp"

namespace motion_profile {

class LinearProfile : public MotionStrategy {
 public:
  segment getSegment() override;
  double getFrequency() override;
  void initialize(double p_v0, double p_a_max_acc, double p_a_max_dcc);
  /*
  @param p_v_set : target velocity in Hz
  @param t : elapsed time after start in ticks
  @return : frequency after t
  */
  double calculateFrequency(double p_v_set, int t);

 private:
  double t_acc_;
  double v_;
  double a_max_acc_;
  double a_max_dcc_;

  double v0_;
  double a_;
  segment seg_;
};
}  // namespace motion_profile
