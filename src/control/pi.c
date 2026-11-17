#include "pi.h"

void pi_init(pi_t *p, float kp, float ki, float ts, float out_min, float out_max)
{
    p->kp = kp;
    p->ki = ki;
    p->ts = ts;
    p->out_min = out_min;
    p->out_max = out_max;
    p->integ = 0.0f;
}

void pi_reset(pi_t *p)
{
    p->integ = 0.0f;
}

void pi_set_limits(pi_t *p, float out_min, float out_max)
{
    p->out_min = out_min;
    p->out_max = out_max;
}

float pi_step(pi_t *p, float err, float ff)
{
    float integ_new = p->integ + p->ki * p->ts * err;
    float out = p->kp * err + integ_new + ff;

    /* Conditional integration: freeze the integrator when the output is
     * saturated and the error would drive it further into saturation. */
    if (out > p->out_max) {
        out = p->out_max;
        if (err > 0.0f) { integ_new = p->integ; }
    } else if (out < p->out_min) {
        out = p->out_min;
        if (err < 0.0f) { integ_new = p->integ; }
    }

    if (integ_new > p->out_max) { integ_new = p->out_max; }
    if (integ_new < p->out_min) { integ_new = p->out_min; }
    p->integ = integ_new;
    return out;
}
