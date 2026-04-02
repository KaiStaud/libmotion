#include "trapezoidal_ramp.h"

#include <math.h>
enum segment get_segment(struct trapezoidal_ramp* pparams, uint32_t t) {
  if (t < pparams->t_acc) {
    return ramp_up;
  } else if (t < (pparams->t_acc + pparams->t_const)) {
    return constant;
  } else if (t >= (pparams->t_acc + pparams->t_const)) {
    return ramp_down;
  } else {
    return done;
  }
}

double ramp_update(struct trapezoidal_ramp* pparams, int64_t t) {
  switch (get_segment(pparams, t)) {
    case ramp_up:
      pparams->v = (pparams->a_max * t);
      break;
    case constant:
      pparams->v = pparams->v_max;
      break;
    case ramp_down:
      uint16_t t_dcc = t - pparams->t_const - pparams->t_acc;
      pparams->v = pparams->v_max - (pparams->a_max * t_dcc);
    default:
      break;
  }
  return pparams->v;
}

enum ramp_error move_to( struct trapezoidal_ramp* pparams,int64_t x) {
  // displacement due to acceleration and decelleration (symmetric!)
  pparams->t_acc = pparams->v_max / pparams->a_max;
  double p_acc = pow(pparams->t_acc, 2) * pparams->a_max;
  pparams->t_const = (x - p_acc)/pparams->v_max;
  pparams->t_dcc = pparams->t_acc;
  return ramp_ok;
}
