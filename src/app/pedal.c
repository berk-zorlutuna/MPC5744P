#include "pedal.h"
#include "ev_config.h"
#include "mc_math.h"

void pedal_init(pedal_t *p)
{
    p->pos = 0.0f;
    p->valid = false;
    p->degraded = false;
    p->range_err = false;
    p->range_err1 = false;
    p->range_err2 = false;
    p->plaus_err = false;
    p->range1_ms = 0u;
    p->range2_ms = 0u;
    p->plaus_ms = 0u;
}

static bool out_of_range(float v)
{
    return (v < APPS_OOR_LOW_V) || (v > APPS_OOR_HIGH_V);
}

/* Debounce helper: counts while cond is true, returns true when expired */
static bool debounce(uint16_t *cnt, bool cond, uint16_t limit, uint16_t dt_ms)
{
    if (cond) {
        if (*cnt < limit) { *cnt = (uint16_t)(*cnt + dt_ms); }
    } else {
        *cnt = 0u;
    }
    return *cnt >= limit;
}

static float apply_deadband(float x)
{
    return (x <= APPS_DEADBAND) ? 0.0f : (x - APPS_DEADBAND) / (1.0f - APPS_DEADBAND);
}

void pedal_update(pedal_t *p, float v1, float v2, uint16_t dt_ms)
{
    float n1 = mc_clampf((v1 - APPS1_MIN_V) / (APPS1_MAX_V - APPS1_MIN_V), 0.0f, 1.0f);
    float n2 = mc_clampf((v2 - APPS2_MIN_V) / (APPS2_MAX_V - APPS2_MIN_V), 0.0f, 1.0f);
    bool oor1 = out_of_range(v1);
    bool oor2 = out_of_range(v2);

    if (debounce(&p->range1_ms, oor1, APPS_RANGE_DEBOUNCE_MS, dt_ms)) { p->range_err1 = true; }
    if (debounce(&p->range2_ms, oor2, APPS_RANGE_DEBOUNCE_MS, dt_ms)) { p->range_err2 = true; }
    p->range_err = p->range_err1 || p->range_err2;

    /* Plausibility is only meaningful while both tracks are in range */
    if (debounce(&p->plaus_ms, !oor1 && !oor2 && (fabsf(n1 - n2) > APPS_PLAUS_TOL),
                 APPS_PLAUS_DEBOUNCE_MS, dt_ms)) {
        p->plaus_err = true;
    }

    p->degraded = false;
    if (p->plaus_err || (p->range_err1 && p->range_err2)) {
        p->valid = false;
        p->pos = 0.0f;
    } else if (p->range_err1 || p->range_err2) {
        /* Limp home on the remaining healthy track */
        bool use1 = p->range_err2;
        p->degraded = true;
        p->valid = use1 ? !oor1 : !oor2;
        p->pos = p->valid ? apply_deadband(use1 ? n1 : n2) : 0.0f;
    } else {
        /* Normal: any suspicious reading (before debounce expires) already
         * yields zero demand. Conservative: use the lower track. */
        p->valid = !oor1 && !oor2 && (p->plaus_ms == 0u);
        p->pos = p->valid ? apply_deadband((n1 < n2) ? n1 : n2) : 0.0f;
    }
}
