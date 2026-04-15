#pragma once
#include "MotionStrategy.hpp"

namespace MotionProfile {

class LinearProfile : public MotionStrategy {

public:
  segment getSegment();
  double get_frequency();
  void initialize(double p_v0, double p_a_max_acc, double p_a_max_dcc);
  /*
  @param p_v_set : target velocity in Hz
  @param t : elapsed time after start in ticks
  @return : frequency after t
  */
  double calculate_frequency(double p_v_set, int t);

private:
  double t_acc;
  double v;
  double a_max_acc;
  double a_max_dcc;

  double v0;
  double a;
  segment seg;
};
} // namespace MotionProfile