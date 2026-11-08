/**
 * @file pedal.h
 * @brief Accelerator pedal position sensor with two redundant tracks.
 */
#ifndef PEDAL_H
#define PEDAL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float pos;           /* 0..1, 0 if invalid */
    bool valid;
    bool range_err;      /* debounced out-of-range on any track */
    bool plaus_err;      /* debounced track mismatch */
    uint16_t range_ms;
    uint16_t plaus_ms;
} pedal_t;

void pedal_init(pedal_t *p);
void pedal_update(pedal_t *p, float v1, float v2, uint16_t dt_ms);

#endif /* PEDAL_H */
