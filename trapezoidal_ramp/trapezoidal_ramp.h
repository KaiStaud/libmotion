#include <stdint.h>
struct trapezoidal_ramp {
  // public
  uint32_t v_max;
  uint32_t a_max;
  // private
  double t_acc;
  double t_const;
  double t_dcc;
  uint32_t x;
  uint32_t v;
  uint32_t a;
};

enum ramp_error{
    ramp_ok,
    ramp_generic_error
};

enum segment {
  ramp_up,
  constant,
  ramp_down,
  done,
};

enum segment get_segment(struct trapezoidal_ramp* pparams, uint32_t t);
double ramp_update(struct trapezoidal_ramp *pparams,int64_t t);
enum ramp_error move_to(struct trapezoidal_ramp *pparams,int64_t x); 
