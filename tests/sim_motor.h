/**
 * @file sim_motor.h
 * @brief IPMSM + averaged inverter model for closed-loop FOC verification.
 */
#ifndef SIM_MOTOR_H
#define SIM_MOTOR_H

#include <stdbool.h>
#include "foc.h"

typedef struct { float id, iq, theta; } motor_t;

typedef struct {
    float rpm_start, rpm_end;   /* linear speed ramp over ramp_steps */
    int ramp_steps;
    float torque;               /* request [Nm] */
    float torque_rate;          /* Nm per step, 0 = step */
    bool from_asc;              /* start in active short circuit steady state */
    int steps;                  /* simulation length in PWM periods */
    float vdc;                  /* DC link [V] */
} sim_cfg_t;

typedef struct {
    float peak_current;         /* max |i| over the run (after the first step) */
    float t_at_10ms;            /* torque after 10 ms (100 steps) */
} sim_res_t;

foc_params_t sim_foc_params(void);
float sim_motor_torque(const motor_t *m);
float sim_current_mag(const motor_t *m);
void  sim_foc(const sim_cfg_t *cfg, foc_t *f, motor_t *m, sim_res_t *res);

#endif /* SIM_MOTOR_H */
