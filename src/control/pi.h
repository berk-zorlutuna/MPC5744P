/**
 * @file pi.h
 * @brief PI controller with output clamping and conditional-integration anti-windup.
 */
#ifndef PI_H
#define PI_H

typedef struct {
    float kp;
    float ki;
    float ts;
    float out_min;
    float out_max;
    float integ;
} pi_t;

void  pi_init(pi_t *p, float kp, float ki, float ts, float out_min, float out_max);
void  pi_reset(pi_t *p);
void  pi_set_limits(pi_t *p, float out_min, float out_max);
/** @param ff feed-forward added to the output before clamping */
float pi_step(pi_t *p, float err, float ff);

#endif /* PI_H */
