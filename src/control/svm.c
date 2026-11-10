#include "svm.h"
#include "mc_math.h"
#include "ev_config.h"

void svm_calc(float v_alpha, float v_beta, float vdc, float duty[3])
{
    mc_ab_t vab = { v_alpha, v_beta };
    mc_abc_t v = mc_iclarke(vab);
    float vmax = v.a;
    float vmin = v.a;
    float voff;
    float inv;

    if (v.b > vmax) { vmax = v.b; }
    if (v.c > vmax) { vmax = v.c; }
    if (v.b < vmin) { vmin = v.b; }
    if (v.c < vmin) { vmin = v.c; }
    voff = 0.5f * (vmax + vmin);

    inv = (vdc > 1.0f) ? (1.0f / vdc) : 0.0f;
    duty[0] = mc_clampf(0.5f + (v.a - voff) * inv, INV_DUTY_MIN, INV_DUTY_MAX);
    duty[1] = mc_clampf(0.5f + (v.b - voff) * inv, INV_DUTY_MIN, INV_DUTY_MAX);
    duty[2] = mc_clampf(0.5f + (v.c - voff) * inv, INV_DUTY_MIN, INV_DUTY_MAX);
}
