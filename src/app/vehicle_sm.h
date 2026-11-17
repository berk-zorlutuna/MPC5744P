/**
 * @file vehicle_sm.h
 * @brief Vehicle state machine: HV contactor sequencing (precharge,
 *        shutdown, active discharge), gear engagement, inverter enable.
 */
#ifndef VEHICLE_SM_H
#define VEHICLE_SM_H

#include <stdbool.h>
#include <stdint.h>
#include "ev_types.h"
#include "diag.h"

typedef enum {
    VSM_INIT = 0,
    VSM_STANDBY,     /* LV on, HV off */
    VSM_PRECHARGE,   /* HV- and precharge relay closed */
    VSM_READY,       /* HV on, inverter off, P/N */
    VSM_DRIVE,       /* inverter running */
    VSM_FAULT,       /* inverter in safe state, waiting for key off */
    VSM_SHUTDOWN     /* opening contactors, discharging DC link */
} vsm_state_t;

typedef struct {
    bool key_on;
    bool brake;
    bool bms_ready;
    bool inverter_ready;     /* current offsets calibrated, no HW fault */
    gear_t gear_sel;         /* driver selector */
    float vbus;              /* inverter DC link voltage */
    float vbatt;             /* battery voltage from BMS */
    float speed_rpm;
    diag_reaction_t reaction;
} vsm_in_t;

typedef struct {
    bool k_neg, k_pre, k_pos;
    bool discharge;
    bool inv_enable;
    gear_t gear;             /* engaged gear */
    bool precharge_fail;     /* event flags, consumed by the application */
    bool discharge_fail;
} vsm_out_t;

typedef struct {
    vsm_state_t st;
    uint32_t t_ms;           /* time in state */
    uint8_t sub;             /* sub-step inside a state */
    vsm_out_t out;
} vsm_t;

void vsm_init(vsm_t *v);
void vsm_step(vsm_t *v, const vsm_in_t *in, uint32_t dt_ms);
const char *vsm_state_name(vsm_state_t s);

#endif /* VEHICLE_SM_H */
