/**
 * @file pedal.h
 * @brief Accelerator pedal position sensor with two redundant tracks.
 *
 * Track 2 delivers half the voltage of track 1. Failure handling:
 *  - both tracks in range but disagreeing (plausibility fault): it is
 *    unknown which one is right -> pedal invalid, zero drive demand
 *  - exactly one track out of range (range fault): the other track is
 *    used in degraded mode (limp home, torque reduced by the fault reaction)
 *  - both tracks out of range: pedal invalid
 * Faults latch until pedal_init() (next key cycle).
 */
#ifndef PEDAL_H
#define PEDAL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float pos;           /* 0..1, 0 if invalid */
    bool valid;          /* pos may be used for drive torque */
    bool degraded;       /* single-track operation */
    bool range_err;      /* latched: any track out of range */
    bool range_err1;     /* latched: track 1 out of range */
    bool range_err2;     /* latched: track 2 out of range */
    bool plaus_err;      /* latched: track mismatch */
    uint16_t range1_ms;
    uint16_t range2_ms;
    uint16_t plaus_ms;
} pedal_t;

void pedal_init(pedal_t *p);
void pedal_update(pedal_t *p, float v1, float v2, uint16_t dt_ms);

#endif /* PEDAL_H */
