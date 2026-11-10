#include "torque_mgr.h"
#include "ev_config.h"
#include "mc_math.h"

void tq_mgr_init(tq_mgr_t *m)
{
    m->t_out = 0.0f;
    m->t_target = 0.0f;
    m->bto_active = false;
}

/* 1 below start, 0 above end, linear in between */
static float derate(float x, float start, float end)
{
    if (x <= start) { return 1.0f; }
    if (x >= end)   { return 0.0f; }
    return 1.0f - (x - start) / (end - start);
}

static float compute_target(tq_mgr_t *m, const tq_in_t *in)
{
    float dir, spd, w, pedal, t, k;
    float t_drive_max, t_regen_max, v_lim;

    if (!in->drive_enabled || (in->gear != GEAR_D && in->gear != GEAR_R)
        || in->reaction >= REACT_TORQUE_OFF) {
        m->bto_active = false;
        return 0.0f;
    }

    dir = (in->gear == GEAR_R) ? -1.0f : 1.0f;
    spd = in->speed_rpm * dir;                   /* + = moving in selected direction */
    w = fabsf(in->speed_rpm) * MC_RPM_TO_RADS;   /* mechanical rad/s */

    /* Brake-throttle override */
    pedal = in->pedal_valid ? in->pedal : 0.0f;
    if (in->brake && pedal > BTO_PEDAL_ON) {
        m->bto_active = true;
    } else if (pedal < BTO_PEDAL_OFF) {
        m->bto_active = false;
    }
    if (m->bto_active) { pedal = 0.0f; }

    /* Available drive torque: motor curve, motor power, battery power */
    t_drive_max = MOTOR_MAX_TORQUE_NM;
    t_regen_max = MOTOR_MAX_REGEN_NM;
    if (w > 1.0f) {
        float p_bat_dis = in->vdc * in->bms_dis_i_max * DRIVETRAIN_EFF;
        float p_bat_chg = in->vdc * in->bms_chg_i_max / DRIVETRAIN_EFF;
        t_drive_max = fminf(t_drive_max, MOTOR_MAX_POWER_W / w);
        t_drive_max = fminf(t_drive_max, p_bat_dis / w);
        t_regen_max = fminf(t_regen_max, MOTOR_MAX_REGEN_POWER_W / w);
        t_regen_max = fminf(t_regen_max, p_bat_chg / w);
    }
    if (in->bms_dis_i_max <= 0.0f) { t_drive_max = 0.0f; }
    if (in->bms_chg_i_max <= 0.0f) { t_regen_max = 0.0f; }

    /* No regen near standstill or when rolling against the selected gear */
    t_regen_max *= 1.0f - derate(spd, REGEN_FADE_LO_RPM, REGEN_FADE_HI_RPM);

    /* Speed limit: taper drive torque near the limit */
    v_lim = (in->gear == GEAR_R) ? REVERSE_MAX_RPM : MOTOR_MAX_RPM;
    t_drive_max *= derate(spd, v_lim - SPEED_LIMIT_BAND_RPM, v_lim);
    if (in->gear == GEAR_R) { t_drive_max *= REVERSE_TORQUE_SCALE; }

    /* Pedal map with coast and brake regen */
    if (in->brake) {
        t = -REGEN_BRAKE_NM;
    } else if (pedal <= 0.0f) {
        t = -REGEN_COAST_NM;
    } else {
        t = pedal * t_drive_max;
    }
    t = mc_clampf(t, -t_regen_max, t_drive_max);

    /* Thermal and DC-link undervoltage derating */
    k = derate(in->t_inv, TEMP_INV_DERATE_C, TEMP_INV_TRIP_C);
    k = fminf(k, derate(in->t_mot, TEMP_MOT_DERATE_C, TEMP_MOT_TRIP_C));
    k = fminf(k, 1.0f - derate(in->vdc, HV_UV_TRIP_V, HV_UV_DERATE_V));
    if (in->reaction == REACT_LIMP) { k *= LIMP_TORQUE_SCALE; }

    return t * k * dir;
}

float tq_mgr_step(tq_mgr_t *m, const tq_in_t *in, float dt)
{
    float rate;

    m->t_target = compute_target(m, in);

    if (in->reaction >= REACT_TORQUE_OFF || !in->drive_enabled) {
        m->t_out = 0.0f;                         /* immediate, no ramp */
        return m->t_out;
    }

    rate = (fabsf(m->t_out) < TORQUE_ZERO_BAND_NM) ? TORQUE_RATE_ZERO_NMPS : TORQUE_RATE_NMPS;
    m->t_out = mc_ramp(m->t_out, m->t_target, rate * dt);
    return m->t_out;
}
