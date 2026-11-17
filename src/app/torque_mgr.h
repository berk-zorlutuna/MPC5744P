/**
 * @file torque_mgr.h
 * @brief Driver torque request: pedal map, regen, brake override,
 *        power/battery/thermal derating, speed limit and slew rate.
 *
 * Sign convention: positive torque accelerates the vehicle forward.
 */
#ifndef TORQUE_MGR_H
#define TORQUE_MGR_H

#include <stdbool.h>
#include "ev_types.h"
#include "diag.h"

typedef struct {
    float pedal;            /* 0..1 */
    bool pedal_valid;
    bool brake;
    gear_t gear;            /* engaged gear */
    bool drive_enabled;     /* vehicle state machine allows torque */
    float speed_rpm;        /* motor speed, signed (+ = forward) */
    float vdc;              /* DC link voltage */
    float bms_dis_i_max;    /* battery discharge current limit [A] */
    float bms_chg_i_max;    /* battery charge current limit [A] */
    float t_inv;            /* inverter temperature [C] */
    float t_mot;            /* motor temperature [C] */
    diag_reaction_t reaction;
} tq_in_t;

typedef struct {
    float t_out;            /* rate limited torque command [Nm] */
    float t_target;         /* before rate limit */
    bool bto_active;        /* brake-throttle override latched */
} tq_mgr_t;

void  tq_mgr_init(tq_mgr_t *m);
float tq_mgr_step(tq_mgr_t *m, const tq_in_t *in, float dt);

#endif /* TORQUE_MGR_H */
