/**
 * @file harness.h
 * @brief Integration harness: runs the complete application (app.c) on the
 *        BSP stub with simulated sensors, driver inputs and BMS traffic.
 */
#ifndef HARNESS_H
#define HARNESS_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_stub.h"
#include "ev_types.h"
#include "can_msgs.h"

typedef struct {
    float vdc;            /* simulated DC link voltage */
    float rpm;            /* simulated motor speed (drives the resolver angle) */
    double angle;         /* resolver angle accumulator [counts] */
    uint8_t alive;
    bool bms_tx;          /* BMS frames are sent every 10 ms */
    bool bms_ready;
    bool bms_fault;
    float bms_pack_v;
    unsigned fast_per_ms; /* current loop invocations per ms (10 nominal) */
    bool slow_enabled;    /* 1 ms task runs */
} harness_t;

extern harness_t g_h;

void h_reset(void);                    /* stub + app reset, nominal inputs */
void h_run_ms(unsigned ms);            /* nominal scheduling for ms milliseconds */
void h_run_fast(unsigned n);           /* n current-loop invocations only */
void h_set_gear(gear_t g);
void h_set_pedal(float frac);          /* both tracks consistent */
void h_set_pedal_volts(float v1, float v2);
bool h_reach_ready(void);              /* reset -> key on -> precharge -> READY */
bool h_reach_drive(void);              /* ... -> brake + D -> DRIVE (brake released after) */

uint16_t h_volts_to_counts(float v);
uint16_t h_amps_to_counts(float a);
uint16_t h_temp_to_counts(float deg_c);

void h_make_bms_frames(bsp_can_frame_t *st, bsp_can_frame_t *lim, uint8_t alive,
                       float pack_v, bool ready, bool fault);

#endif /* HARNESS_H */
