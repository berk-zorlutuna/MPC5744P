#include "vehicle_sm.h"
#include "ev_config.h"
#include <math.h>

static void enter(vsm_t *v, vsm_state_t s)
{
    v->st = s;
    v->t_ms = 0u;
    v->sub = 0u;
}

void vsm_init(vsm_t *v)
{
    v->out.k_neg = v->out.k_pre = v->out.k_pos = false;
    v->out.discharge = false;
    v->out.inv_enable = false;
    v->out.gear = GEAR_P;
    v->out.precharge_fail = false;
    v->out.discharge_fail = false;
    enter(v, VSM_INIT);
}

static bool standstill(const vsm_in_t *in)
{
    return fabsf(in->speed_rpm) < GEAR_SHIFT_MAX_RPM;
}

/* Gear change rules while the inverter is running */
static void handle_gear_in_drive(vsm_t *v, const vsm_in_t *in)
{
    gear_t cur = v->out.gear;
    gear_t sel = in->gear_sel;

    if (sel == cur || sel == GEAR_INVALID) { return; }

    switch (sel) {
    case GEAR_N:
        v->out.gear = GEAR_N;                          /* always allowed */
        break;
    case GEAR_D:
    case GEAR_R:
        if (cur == GEAR_N) {
            /* N -> D/R: allowed at standstill, or when rolling in that direction */
            bool same_dir = (sel == GEAR_D) ? (in->speed_rpm > 0.0f) : (in->speed_rpm < 0.0f);
            if (standstill(in) || same_dir) { v->out.gear = sel; }
        } else if (standstill(in) && in->brake) {
            v->out.gear = sel;                         /* D <-> R */
        }
        break;
    case GEAR_P:
        if (standstill(in)) { v->out.gear = GEAR_P; }
        break;
    default:
        break;
    }
}

void vsm_step(vsm_t *v, const vsm_in_t *in, uint32_t dt_ms)
{
    vsm_out_t *o = &v->out;
    bool hv_state;
    bool key_off;

    v->t_ms += dt_ms;
    o->precharge_fail = false;
    o->discharge_fail = false;

    /* Global transitions for every state with HV connected */
    hv_state = (v->st == VSM_PRECHARGE) || (v->st == VSM_READY)
            || (v->st == VSM_DRIVE) || (v->st == VSM_FAULT);
    /* Key off while driving fast: keep HV and inverter control (zero torque
     * in N) until the vehicle is slow, instead of dropping HV at speed. */
    key_off = !in->key_on && !(v->st == VSM_DRIVE && fabsf(in->speed_rpm) >= KEYOFF_MAX_RPM);
    if (v->st == VSM_DRIVE && !in->key_on) { v->out.gear = GEAR_N; }

    if (hv_state && (key_off || in->reaction >= REACT_HV_OFF)) {
        enter(v, VSM_SHUTDOWN);
    } else if ((v->st == VSM_READY || v->st == VSM_DRIVE) && in->reaction >= REACT_TORQUE_OFF) {
        enter(v, VSM_FAULT);
    }

    switch (v->st) {
    case VSM_INIT:
        o->k_neg = o->k_pre = o->k_pos = false;
        o->discharge = false;
        o->inv_enable = false;
        o->gear = GEAR_P;
        enter(v, VSM_STANDBY);
        break;

    case VSM_STANDBY:
        o->k_neg = o->k_pre = o->k_pos = false;
        o->discharge = false;
        o->inv_enable = false;
        o->gear = GEAR_P;
        if (in->key_on && in->bms_ready && in->reaction < REACT_HV_OFF) {
            enter(v, VSM_PRECHARGE);
        }
        break;

    case VSM_PRECHARGE:
        o->inv_enable = false;
        o->discharge = false;
        if (v->sub == 0u) {
            o->k_neg = true;
            o->k_pre = true;
            o->k_pos = false;
            if (v->t_ms >= PRECHARGE_MIN_MS && in->vbatt > HV_UV_TRIP_V
                && in->vbus >= PRECHARGE_RATIO * in->vbatt) {
                o->k_pos = true;
                v->sub = 1u;
                v->t_ms = 0u;
            } else if (v->t_ms >= PRECHARGE_TIMEOUT_MS) {
                o->precharge_fail = true;
                enter(v, VSM_SHUTDOWN);
            }
        } else if (v->t_ms >= CONTACTOR_DELAY_MS) {
            o->k_pre = false;                           /* main contactor carries current */
            enter(v, VSM_READY);
        }
        break;

    case VSM_READY:
        o->inv_enable = false;
        if (in->gear_sel == GEAR_P || in->gear_sel == GEAR_N) {
            o->gear = in->gear_sel;
        }
        if ((in->gear_sel == GEAR_D || in->gear_sel == GEAR_R)
            && in->brake && standstill(in) && in->inverter_ready
            && in->reaction < REACT_TORQUE_OFF) {
            o->gear = in->gear_sel;
            enter(v, VSM_DRIVE);
        }
        break;

    case VSM_DRIVE:
        o->inv_enable = true;
        if (in->key_on) { handle_gear_in_drive(v, in); }
        if (o->gear == GEAR_P) {
            o->inv_enable = false;
            enter(v, VSM_READY);
        }
        break;

    case VSM_FAULT:
        /* Inverter safe state is chosen by the current loop (freewheel/ASC).
         * Leave only through key off -> SHUTDOWN. */
        o->inv_enable = false;
        o->gear = GEAR_N;
        break;

    case VSM_SHUTDOWN:
        o->inv_enable = false;
        o->gear = GEAR_P;
        if (v->sub == 0u) {
            o->k_pos = false;
            o->k_pre = false;
            if (v->t_ms >= CONTACTOR_DELAY_MS) {
                o->k_neg = false;
                o->discharge = true;
                v->sub = 1u;
                v->t_ms = 0u;
            }
        } else {
            if (in->vbus < HV_SAFE_V) {
                o->discharge = false;
                enter(v, VSM_STANDBY);
            } else if (v->t_ms >= DISCHARGE_TIMEOUT_MS) {
                o->discharge = false;
                o->discharge_fail = true;
                enter(v, VSM_STANDBY);
            }
        }
        break;

    default:
        vsm_init(v);
        break;
    }
}

const char *vsm_state_name(vsm_state_t s)
{
    static const char *const names[] = {
        "INIT", "STANDBY", "PRECHARGE", "READY", "DRIVE", "FAULT", "SHUTDOWN"
    };
    return ((unsigned)s < sizeof(names) / sizeof(names[0])) ? names[s] : "?";
}
