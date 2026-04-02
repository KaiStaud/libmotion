#include "../trapezoidal_ramp/trapezoidal_ramp.h"
#include "constant_velocity.h"
#include <math.h>
struct trapezoidal_ramp ramp;

void move_pvm(uint32_t a_max, uint32_t v_max)
{
    struct trapezoidal_ramp ramp;
    ramp.a_max=a_max;
    ramp.v_max=v_max;
    double x_up=0.5*pow(v_max,2)/a_max;
    move_to(&ramp,x_up);
}
double update(int t){
    // overwrite constant velocity segment
    if(get_segment(&ramp,t) == constant){
        //abort_ramp(&ramp);
        return ramp.v_max;
    }
    else{
        return ramp_update(&ramp,t);
    }
}