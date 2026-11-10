#include "pll.h"
#include "mc_math.h"

void pll_init(pll_t *p, float bw_hz, float ts)
{
    float wn = MC_2PI * bw_hz;
    p->kp = 2.0f * wn;       /* critically damped (zeta = 1) */
    p->ki = wn * wn;
    p->ts = ts;
    p->theta = 0.0f;
    p->omega = 0.0f;
}

void pll_update(pll_t *p, float theta_meas)
{
    float err = mc_wrap_pi(theta_meas - p->theta);
    p->omega += p->ki * err * p->ts;
    p->theta = mc_wrap_2pi(p->theta + (p->omega + p->kp * err) * p->ts);
}
