/**
 * @file pll.h
 * @brief Angle tracking observer: filters the measured angle and estimates speed.
 */
#ifndef PLL_H
#define PLL_H

typedef struct {
    float theta;   /* estimated angle [rad], 0..2pi */
    float omega;   /* estimated speed [rad/s] */
    float kp, ki, ts;
} pll_t;

void pll_init(pll_t *p, float bw_hz, float ts);
void pll_update(pll_t *p, float theta_meas);

#endif /* PLL_H */
