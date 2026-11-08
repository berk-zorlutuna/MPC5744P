#include "pedal.h"
#include "ev_config.h"
#include "mc_math.h"

void pedal_init(pedal_t *p)
{
    p->pos = 0.0f;
    p->valid = false;
    p->range_err = false;
    p->plaus_err = false;
    p->range_ms = 0u;
    p->plaus_ms = 0u;
}

static bool out_of_range(float v)
{
    return (v < APPS_OOR_LOW_V) || (v > APPS_OOR_HIGH_V);
}

void pedal_update(pedal_t *p, float v1, float v2, uint16_t dt_ms)
{
    float n1 = mc_clampf((v1 - APPS1_MIN_V) / (APPS1_MAX_V - APPS1_MIN_V), 0.0f, 1.0f);
    float n2 = mc_clampf((v2 - APPS2_MIN_V) / (APPS2_MAX_V - APPS2_MIN_V), 0.0f, 1.0f);
    float pos;

    if (out_of_range(v1) || out_of_range(v2)) {
        if (p->range_ms < APPS_RANGE_DEBOUNCE_MS) { p->range_ms = (uint16_t)(p->range_ms + dt_ms); }
    } else {
        p->range_ms = 0u;
    }
    if (p->range_ms >= APPS_RANGE_DEBOUNCE_MS) { p->range_err = true; }

    if (fabsf(n1 - n2) > APPS_PLAUS_TOL) {
        if (p->plaus_ms < APPS_PLAUS_DEBOUNCE_MS) { p->plaus_ms = (uint16_t)(p->plaus_ms + dt_ms); }
    } else {
        p->plaus_ms = 0u;
    }
    if (p->plaus_ms >= APPS_PLAUS_DEBOUNCE_MS) { p->plaus_err = true; }

    /* Errors latch until pedal_init() (key cycle). Before debounce expires
     * a suspicious reading already yields zero torque. */
    p->valid = !p->range_err && !p->plaus_err && (p->range_ms == 0u) && (p->plaus_ms == 0u);

    /* Conservative: use the lower of the two tracks */
    pos = (n1 < n2) ? n1 : n2;
    pos = (pos <= APPS_DEADBAND) ? 0.0f : (pos - APPS_DEADBAND) / (1.0f - APPS_DEADBAND);
    p->pos = p->valid ? pos : 0.0f;
}
