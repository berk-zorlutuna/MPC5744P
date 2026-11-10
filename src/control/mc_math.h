/**
 * @file mc_math.h
 * @brief Motor control math: Clarke/Park transforms and helpers (float, FPU).
 *
 * Clarke transform is amplitude invariant, so the torque equation is
 * T = 1.5 * p * (psi * iq + (Ld - Lq) * id * iq).
 */
#ifndef MC_MATH_H
#define MC_MATH_H

#include <math.h>

#define MC_PI           3.14159265f
#define MC_2PI          6.28318531f
#define MC_SQRT3        1.73205081f
#define MC_INV_SQRT3    0.57735027f
#define MC_SQRT3_2      0.86602540f
#define MC_RPM_TO_RADS  (MC_2PI / 60.0f)

typedef struct { float a, b, c; } mc_abc_t;
typedef struct { float alpha, beta; } mc_ab_t;
typedef struct { float d, q; } mc_dq_t;

static inline float mc_clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

static inline float mc_wrap_2pi(float a)
{
    while (a >= MC_2PI) { a -= MC_2PI; }
    while (a < 0.0f)    { a += MC_2PI; }
    return a;
}

static inline float mc_wrap_pi(float a)
{
    while (a > MC_PI)   { a -= MC_2PI; }
    while (a <= -MC_PI) { a += MC_2PI; }
    return a;
}

static inline float mc_ramp(float cur, float target, float step)
{
    if (target > cur + step) { return cur + step; }
    if (target < cur - step) { return cur - step; }
    return target;
}

static inline mc_ab_t mc_clarke(mc_abc_t x)
{
    mc_ab_t r;
    r.alpha = (2.0f * x.a - x.b - x.c) * (1.0f / 3.0f);
    r.beta  = (x.b - x.c) * MC_INV_SQRT3;
    return r;
}

static inline mc_abc_t mc_iclarke(mc_ab_t x)
{
    mc_abc_t r;
    r.a = x.alpha;
    r.b = -0.5f * x.alpha + MC_SQRT3_2 * x.beta;
    r.c = -0.5f * x.alpha - MC_SQRT3_2 * x.beta;
    return r;
}

static inline mc_dq_t mc_park(mc_ab_t x, float s, float c)
{
    mc_dq_t r;
    r.d =  x.alpha * c + x.beta * s;
    r.q = -x.alpha * s + x.beta * c;
    return r;
}

static inline mc_ab_t mc_ipark(mc_dq_t x, float s, float c)
{
    mc_ab_t r;
    r.alpha = x.d * c - x.q * s;
    r.beta  = x.d * s + x.q * c;
    return r;
}

#endif /* MC_MATH_H */
