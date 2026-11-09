/**
 * @file test_vehicle.c
 * @brief Unit-level verification cases: pedal (SWR-PED), torque manager
 *        (SWR-TQ), vehicle state machine (SWR-VSM), fault manager
 *        (SWR-DIAG) and CAN codec (SWR-COM).
 */
#include <math.h>
#include <string.h>
#include "vtest.h"
#include "ev_config.h"
#include "mc_math.h"
#include "pedal.h"
#include "torque_mgr.h"
#include "vehicle_sm.h"
#include "diag.h"
#include "can_msgs.h"
#include "harness.h"

/* volts for a normalized pedal position on each track */
static float v1_of(float x) { return APPS1_MIN_V + x * (APPS1_MAX_V - APPS1_MIN_V); }
static float v2_of(float x) { return APPS2_MIN_V + x * (APPS2_MAX_V - APPS2_MIN_V); }
static float db(float x) { return (x <= APPS_DEADBAND) ? 0.0f : (x - APPS_DEADBAND) / (1.0f - APPS_DEADBAND); }

static void pedal_run(pedal_t *p, float v1, float v2, unsigned ms)
{
    while (ms-- > 0u) { pedal_update(p, v1, v2, 1u); }
}

/* ======================================================================= */
/* Pedal                                                                   */
/* ======================================================================= */

/* SVC-PED-001: normal operation */
void svc_ped_001(void)
{
    pedal_t p;
    pedal_init(&p);

    pedal_update(&p, v1_of(0.0f), v2_of(0.0f), 1u);
    CHECK(p.valid && !p.degraded);
    CHECK_NEAR(p.pos, 0.0, 1e-6);

    pedal_update(&p, v1_of(APPS_DEADBAND * 0.9f), v2_of(APPS_DEADBAND * 0.9f), 1u);
    CHECK_NEAR(p.pos, 0.0, 1e-6);                         /* inside dead band */

    pedal_update(&p, v1_of(0.5f), v2_of(0.5f), 1u);
    CHECK_NEAR(p.pos, db(0.5f), 0.005);

    pedal_update(&p, v1_of(1.0f), v2_of(1.0f), 1u);
    CHECK_NEAR(p.pos, 1.0, 0.005);

    /* lower track is used (mismatch within tolerance) */
    pedal_update(&p, v1_of(0.60f), v2_of(0.55f), 1u);
    CHECK(p.valid);
    CHECK_NEAR(p.pos, db(0.55f), 0.005);
    vt_note("50 %% pedal -> %.4f (dead band %.0f %%), min-track selection verified",
            (double)db(0.5f), (double)(APPS_DEADBAND * 100.0f));
}

/* SVC-PED-002: range check */
void svc_ped_002(void)
{
    pedal_t p;
    pedal_init(&p);

    /* track 1 above range: zero demand at once, latched after debounce */
    pedal_run(&p, 4.9f, v2_of(0.5f), APPS_RANGE_DEBOUNCE_MS - 1u);
    CHECK(!p.range_err && !p.valid);
    CHECK_NEAR(p.pos, 0.0, 1e-6);
    pedal_run(&p, 4.9f, v2_of(0.5f), 1u);
    CHECK(p.range_err && p.range_err1 && !p.range_err2);

    /* stays latched when the signal returns */
    pedal_run(&p, v1_of(0.5f), v2_of(0.5f), 10u);
    CHECK(p.range_err1);

    /* both tracks out of range: invalid */
    pedal_init(&p);
    pedal_run(&p, 0.1f, 0.1f, APPS_RANGE_DEBOUNCE_MS);
    CHECK(p.range_err1 && p.range_err2 && !p.valid);
    CHECK_NEAR(p.pos, 0.0, 1e-6);

    /* short glitch below debounce time is tolerated */
    pedal_init(&p);
    pedal_run(&p, 0.1f, v2_of(0.0f), APPS_RANGE_DEBOUNCE_MS - 5u);
    pedal_run(&p, v1_of(0.0f), v2_of(0.0f), 5u);
    CHECK(!p.range_err && p.valid);
    vt_note("range fault latched after %u ms; %u ms glitch tolerated",
            (unsigned)APPS_RANGE_DEBOUNCE_MS, (unsigned)APPS_RANGE_DEBOUNCE_MS - 5u);
}

/* SVC-PED-003: plausibility between tracks */
void svc_ped_003(void)
{
    pedal_t p;
    pedal_init(&p);

    pedal_run(&p, v1_of(0.6f), v2_of(0.4f), 1u);          /* 20 % mismatch */
    CHECK(!p.valid);
    CHECK_NEAR(p.pos, 0.0, 1e-6);                         /* immediately zero */
    pedal_run(&p, v1_of(0.6f), v2_of(0.4f), APPS_PLAUS_DEBOUNCE_MS - 2u);
    CHECK(!p.plaus_err);
    pedal_run(&p, v1_of(0.6f), v2_of(0.4f), 1u);
    CHECK(p.plaus_err);
    pedal_run(&p, v1_of(0.5f), v2_of(0.5f), 10u);         /* agreement again: stays latched */
    CHECK(!p.valid && p.plaus_err);
    CHECK_NEAR(p.pos, 0.0, 1e-6);

    /* mismatch inside tolerance is accepted */
    pedal_init(&p);
    pedal_run(&p, v1_of(0.55f), v2_of(0.47f), 200u);
    CHECK(p.valid && !p.plaus_err);
    vt_note("20 %% mismatch: demand zero in first sample, latched at %u ms; 8 %% accepted",
            (unsigned)APPS_PLAUS_DEBOUNCE_MS);
}

/* SVC-PED-004: single-track degraded operation */
void svc_ped_004(void)
{
    pedal_t p;
    pedal_init(&p);

    /* track 2 shorted to ground: after debounce, track 1 is used */
    pedal_run(&p, v1_of(0.5f), 0.0f, APPS_RANGE_DEBOUNCE_MS);
    CHECK(p.range_err2 && !p.range_err1);
    CHECK(p.degraded && p.valid);
    CHECK_NEAR(p.pos, db(0.5f), 0.005);
    pedal_run(&p, v1_of(0.8f), 0.0f, 1u);
    CHECK_NEAR(p.pos, db(0.8f), 0.005);

    /* track 1 open (pulled high): track 2 is used */
    pedal_init(&p);
    pedal_run(&p, 5.0f, v2_of(0.3f), APPS_RANGE_DEBOUNCE_MS);
    CHECK(p.range_err1 && p.degraded && p.valid);
    CHECK_NEAR(p.pos, db(0.3f), 0.005);

    /* plausibility fault: no single-track fallback */
    pedal_init(&p);
    pedal_run(&p, v1_of(0.6f), v2_of(0.3f), APPS_PLAUS_DEBOUNCE_MS);
    CHECK(p.plaus_err && !p.valid && !p.degraded);
    vt_note("track 2 short: pos follows track 1 (%.3f at 80 %%); plausibility fault -> invalid", (double)p.pos);
}

/* ======================================================================= */
/* Torque manager                                                          */
/* ======================================================================= */

static tq_in_t tq_default(void)
{
    tq_in_t in;
    memset(&in, 0, sizeof(in));
    in.pedal_valid = true;
    in.gear = GEAR_D;
    in.drive_enabled = true;
    in.vdc = 350.0f;
    in.bms_dis_i_max = 400.0f;
    in.bms_chg_i_max = 200.0f;
    in.t_inv = 40.0f;
    in.t_mot = 40.0f;
    in.reaction = REACT_NONE;
    return in;
}

static float tq_settle(tq_mgr_t *m, const tq_in_t *in)
{
    int k;
    for (k = 0; k < 2000; k++) { (void)tq_mgr_step(m, in, TS_SLOW); }
    return m->t_out;
}

static float w_of(float rpm) { return rpm * MC_RPM_TO_RADS; }

/* SVC-TQ-001: drive torque envelope */
void svc_tq_001(void)
{
    tq_mgr_t m;
    tq_in_t in = tq_default();
    float t;

    tq_mgr_init(&m);
    in.pedal = 1.0f;
    in.speed_rpm = 1000.0f;
    CHECK_NEAR(tq_settle(&m, &in), MOTOR_MAX_TORQUE_NM, 0.01);
    in.pedal = 0.5f;
    CHECK_NEAR(tq_settle(&m, &in), 0.5f * MOTOR_MAX_TORQUE_NM, 0.01);

    in.pedal = 1.0f;
    in.speed_rpm = 8000.0f;
    t = tq_settle(&m, &in);
    CHECK_NEAR(t, MOTOR_MAX_POWER_W / w_of(8000.0f), 0.1);
    vt_note("full pedal: %.0f Nm at 1000 rpm, %.1f Nm at 8000 rpm (= %.0f kW)",
            (double)MOTOR_MAX_TORQUE_NM, (double)t, (double)(t * w_of(8000.0f) / 1000.0f));
}

/* SVC-TQ-002: battery discharge limit */
void svc_tq_002(void)
{
    tq_mgr_t m;
    tq_in_t in = tq_default();
    float exp_t;

    tq_mgr_init(&m);
    in.pedal = 1.0f;
    in.speed_rpm = 3000.0f;
    in.bms_dis_i_max = 50.0f;
    exp_t = 350.0f * 50.0f * DRIVETRAIN_EFF / w_of(3000.0f);
    CHECK_NEAR(tq_settle(&m, &in), exp_t, 0.1);

    in.bms_dis_i_max = 0.0f;
    CHECK_NEAR(tq_settle(&m, &in), 0.0, 0.01);        /* no drive torque */
    vt_note("I_dis = 50 A at 3000 rpm -> %.1f Nm; I_dis = 0 -> %.1f Nm", (double)exp_t, (double)m.t_out);
}

/* SVC-TQ-003: regenerative braking levels and limits */
void svc_tq_003(void)
{
    tq_mgr_t m;
    tq_in_t in = tq_default();
    float t_chg;

    tq_mgr_init(&m);
    in.pedal = 0.0f;
    in.speed_rpm = 2000.0f;
    CHECK_NEAR(tq_settle(&m, &in), -REGEN_COAST_NM, 0.01);
    in.brake = true;
    CHECK_NEAR(tq_settle(&m, &in), -REGEN_BRAKE_NM, 0.01);

    /* regen power limit at high speed */
    in.speed_rpm = 9000.0f;
    CHECK_NEAR(tq_settle(&m, &in), -MOTOR_MAX_REGEN_POWER_W / w_of(9000.0f), 0.1);

    /* battery charge current limit */
    in.speed_rpm = 3000.0f;
    in.bms_chg_i_max = 20.0f;
    t_chg = 350.0f * 20.0f / DRIVETRAIN_EFF / w_of(3000.0f);
    CHECK_NEAR(tq_settle(&m, &in), -t_chg, 0.1);
    in.bms_chg_i_max = 0.0f;
    CHECK_NEAR(tq_settle(&m, &in), 0.0, 0.01);
    vt_note("coast %.0f Nm, brake %.0f Nm, I_chg 20 A at 3000 rpm -> %.1f Nm",
            (double)-REGEN_COAST_NM, (double)-REGEN_BRAKE_NM, (double)-t_chg);
}

/* SVC-TQ-004: regen fade near standstill */
void svc_tq_004(void)
{
    tq_mgr_t m;
    tq_in_t in = tq_default();
    float mid = 0.5f * (REGEN_FADE_LO_RPM + REGEN_FADE_HI_RPM);

    tq_mgr_init(&m);
    in.brake = true;
    in.speed_rpm = REGEN_FADE_HI_RPM + 1.0f;
    CHECK_NEAR(tq_settle(&m, &in), -REGEN_BRAKE_NM, 0.5);
    in.speed_rpm = mid;
    CHECK_NEAR(tq_settle(&m, &in), -0.5f * MOTOR_MAX_REGEN_NM, 0.5);
    in.speed_rpm = REGEN_FADE_LO_RPM;
    CHECK_NEAR(tq_settle(&m, &in), 0.0, 0.01);
    in.speed_rpm = -300.0f;                         /* rolling backwards in D */
    CHECK_NEAR(tq_settle(&m, &in), 0.0, 0.01);
    vt_note("regen limit %.0f Nm at %.0f rpm, 0 at %.0f rpm and when rolling backwards",
            (double)(0.5f * MOTOR_MAX_REGEN_NM), (double)mid, (double)REGEN_FADE_LO_RPM);
}

/* SVC-TQ-005: brake-throttle override */
void svc_tq_005(void)
{
    tq_mgr_t m;
    tq_in_t in = tq_default();

    tq_mgr_init(&m);
    in.speed_rpm = 3000.0f;
    in.pedal = BTO_PEDAL_ON - 0.05f;              /* below threshold: no override */
    in.brake = true;
    (void)tq_settle(&m, &in);
    CHECK(!m.bto_active);

    in.pedal = 1.0f;
    (void)tq_settle(&m, &in);
    CHECK(m.bto_active);
    CHECK(m.t_out < 0.0f);                         /* no drive torque, brake regen */

    in.brake = false;                              /* brake released, pedal held */
    CHECK(tq_settle(&m, &in) <= 0.0f);
    CHECK(m.bto_active);
    in.pedal = BTO_PEDAL_OFF + 0.01f;
    (void)tq_settle(&m, &in);
    CHECK(m.bto_active);
    in.pedal = BTO_PEDAL_OFF - 0.01f;              /* released below 5 %: cleared */
    (void)tq_settle(&m, &in);
    CHECK(!m.bto_active);
    in.pedal = 0.5f;
    CHECK(tq_settle(&m, &in) > 0.0f);
    vt_note("override set above %.0f %% with brake, held until pedal < %.0f %%",
            (double)(BTO_PEDAL_ON * 100.0f), (double)(BTO_PEDAL_OFF * 100.0f));
}

/* SVC-TQ-006: reverse gear */
void svc_tq_006(void)
{
    tq_mgr_t m;
    tq_in_t in = tq_default();

    tq_mgr_init(&m);
    in.gear = GEAR_R;
    in.pedal = 1.0f;
    in.speed_rpm = -500.0f;
    CHECK_NEAR(tq_settle(&m, &in), -MOTOR_MAX_TORQUE_NM * REVERSE_TORQUE_SCALE, 0.01);
    in.speed_rpm = -REVERSE_MAX_RPM;
    CHECK_NEAR(tq_settle(&m, &in), 0.0, 0.01);
    in.speed_rpm = -(REVERSE_MAX_RPM - 0.5f * SPEED_LIMIT_BAND_RPM);
    CHECK_NEAR(tq_settle(&m, &in), -0.5f * MOTOR_MAX_TORQUE_NM * REVERSE_TORQUE_SCALE, 0.5);
    vt_note("reverse: %.0f Nm (40 %%), zero at %.0f rpm",
            (double)(-MOTOR_MAX_TORQUE_NM * REVERSE_TORQUE_SCALE), (double)-REVERSE_MAX_RPM);
}

/* SVC-TQ-007: forward speed limit */
void svc_tq_007(void)
{
    tq_mgr_t m;
    tq_in_t in = tq_default();

    tq_mgr_init(&m);
    in.pedal = 1.0f;
    in.speed_rpm = MOTOR_MAX_RPM;
    CHECK(tq_settle(&m, &in) <= 0.0f);
    in.speed_rpm = MOTOR_MAX_RPM - 0.5f * SPEED_LIMIT_BAND_RPM;
    CHECK_NEAR(tq_settle(&m, &in), 0.5f * MOTOR_MAX_POWER_W / w_of(in.speed_rpm), 0.5);
    in.speed_rpm = MOTOR_MAX_RPM - SPEED_LIMIT_BAND_RPM - 1.0f;
    CHECK_NEAR(tq_settle(&m, &in), MOTOR_MAX_POWER_W / w_of(in.speed_rpm), 0.5);
    vt_note("drive torque tapers to zero between %.0f and %.0f rpm",
            (double)(MOTOR_MAX_RPM - SPEED_LIMIT_BAND_RPM), (double)MOTOR_MAX_RPM);
}

/* SVC-TQ-008: thermal and DC-link undervoltage derating */
void svc_tq_008(void)
{
    tq_mgr_t m;
    tq_in_t in = tq_default();

    tq_mgr_init(&m);
    in.pedal = 1.0f;
    in.speed_rpm = 1000.0f;
    in.t_inv = TEMP_INV_DERATE_C;
    CHECK_NEAR(tq_settle(&m, &in), MOTOR_MAX_TORQUE_NM, 0.1);
    in.t_inv = 0.5f * (TEMP_INV_DERATE_C + TEMP_INV_TRIP_C);
    CHECK_NEAR(tq_settle(&m, &in), 0.5f * MOTOR_MAX_TORQUE_NM, 0.5);
    in.t_inv = TEMP_INV_TRIP_C;
    CHECK_NEAR(tq_settle(&m, &in), 0.0, 0.01);
    in.t_inv = 40.0f;
    in.t_mot = 0.5f * (TEMP_MOT_DERATE_C + TEMP_MOT_TRIP_C);
    CHECK_NEAR(tq_settle(&m, &in), 0.5f * MOTOR_MAX_TORQUE_NM, 0.5);
    in.t_mot = 40.0f;
    in.vdc = 0.5f * (HV_UV_TRIP_V + HV_UV_DERATE_V);
    CHECK_NEAR(tq_settle(&m, &in), 0.5f * MOTOR_MAX_TORQUE_NM, 0.5);
    vt_note("50 %% torque at midpoint of each derating band (inverter, motor, Vdc)");
}

/* SVC-TQ-009: torque slew rate */
void svc_tq_009(void)
{
    tq_mgr_t m;
    tq_in_t in = tq_default();
    float t, max_rate_hi = 0.0f, max_rate_lo = 0.0f, prev = 0.0f;
    int k;

    tq_mgr_init(&m);
    in.pedal = 1.0f;
    in.speed_rpm = 1000.0f;
    for (k = 0; k < 400; k++) {
        float rate;
        t = tq_mgr_step(&m, &in, TS_SLOW);
        rate = (t - prev) / TS_SLOW;
        if (fabsf(prev) < TORQUE_ZERO_BAND_NM) {
            if (rate > max_rate_lo) { max_rate_lo = rate; }
        } else if (rate > max_rate_hi) {
            max_rate_hi = rate;
        }
        prev = t;
    }
    CHECK(max_rate_lo <= TORQUE_RATE_ZERO_NMPS + 0.5f);
    CHECK(max_rate_hi <= TORQUE_RATE_NMPS + 0.5f);
    CHECK(max_rate_hi > 0.9f * TORQUE_RATE_NMPS);
    vt_note("max slope %.0f Nm/s near zero, %.0f Nm/s elsewhere", (double)max_rate_lo, (double)max_rate_hi);
}

/* SVC-TQ-010: torque removal */
void svc_tq_010(void)
{
    tq_mgr_t m;
    tq_in_t in = tq_default();

    tq_mgr_init(&m);
    in.pedal = 1.0f;
    in.speed_rpm = 1000.0f;
    (void)tq_settle(&m, &in);
    in.reaction = REACT_TORQUE_OFF;
    CHECK_NEAR(tq_mgr_step(&m, &in, TS_SLOW), 0.0, 1e-6);   /* same step, no ramp */

    in.reaction = REACT_NONE;
    (void)tq_settle(&m, &in);
    in.drive_enabled = false;
    CHECK_NEAR(tq_mgr_step(&m, &in, TS_SLOW), 0.0, 1e-6);

    in.drive_enabled = true;
    in.gear = GEAR_N;
    CHECK_NEAR(tq_settle(&m, &in), 0.0, 1e-6);
    in.gear = GEAR_P;
    CHECK_NEAR(tq_settle(&m, &in), 0.0, 1e-6);
    vt_note("zero torque within one 1 ms step on torque-off / drive disabled; zero in N and P");
}

/* SVC-TQ-011: limp mode scaling */
void svc_tq_011(void)
{
    tq_mgr_t m;
    tq_in_t in = tq_default();

    tq_mgr_init(&m);
    in.pedal = 1.0f;
    in.speed_rpm = 1000.0f;
    in.reaction = REACT_LIMP;
    CHECK_NEAR(tq_settle(&m, &in), LIMP_TORQUE_SCALE * MOTOR_MAX_TORQUE_NM, 0.1);
    vt_note("limp: %.0f Nm at full pedal", (double)(LIMP_TORQUE_SCALE * MOTOR_MAX_TORQUE_NM));
}

/* ======================================================================= */
/* Vehicle state machine                                                   */
/* ======================================================================= */

static vsm_in_t vsm_default(void)
{
    vsm_in_t in;
    memset(&in, 0, sizeof(in));
    in.bms_ready = true;
    in.inverter_ready = true;
    in.gear_sel = GEAR_P;
    in.vbatt = 350.0f;
    in.reaction = REACT_NONE;
    return in;
}

static void vsm_run(vsm_t *v, const vsm_in_t *in, uint32_t ms)
{
    uint32_t t;
    for (t = 0; t < ms; t += VSM_PERIOD_MS) { vsm_step(v, in, VSM_PERIOD_MS); }
}

static bool vsm_to_ready(vsm_t *v, vsm_in_t *in)
{
    vsm_init(v);
    *in = vsm_default();
    in->key_on = true;
    in->vbus = 345.0f;
    vsm_run(v, in, 300);
    return v->st == VSM_READY;
}

static bool vsm_to_drive(vsm_t *v, vsm_in_t *in)
{
    if (!vsm_to_ready(v, in)) { return false; }
    in->brake = true;
    in->gear_sel = GEAR_D;
    vsm_run(v, in, 20);
    in->brake = false;
    return v->st == VSM_DRIVE;
}

/* SVC-VSM-001: standby outputs */
void svc_vsm_001(void)
{
    vsm_t v;
    vsm_in_t in = vsm_default();
    vsm_init(&v);
    vsm_run(&v, &in, 50);
    CHECK(v.st == VSM_STANDBY);
    CHECK(!v.out.k_neg && !v.out.k_pre && !v.out.k_pos && !v.out.discharge);
    CHECK(!v.out.inv_enable && v.out.gear == GEAR_P);
    in.bms_ready = false;
    in.key_on = true;                       /* key on but BMS not ready: stay */
    vsm_run(&v, &in, 100);
    CHECK(v.st == VSM_STANDBY);
    vt_note("STANDBY: all contactors open, inverter disabled; no HV request without BMS ready");
}

/* SVC-VSM-002: precharge sequence */
void svc_vsm_002(void)
{
    vsm_t v;
    vsm_in_t in = vsm_default();
    uint32_t t;
    bool k_pos_early = false;

    vsm_init(&v);
    vsm_run(&v, &in, 20);
    in.key_on = true;
    in.vbus = 345.0f;                       /* already charged: minimum time still applies */
    vsm_run(&v, &in, 20);
    CHECK(v.st == VSM_PRECHARGE);
    CHECK(v.out.k_neg && v.out.k_pre && !v.out.k_pos);
    for (t = 0; t < PRECHARGE_MIN_MS - 20u; t += VSM_PERIOD_MS) {
        vsm_step(&v, &in, VSM_PERIOD_MS);
        k_pos_early = k_pos_early || v.out.k_pos;
    }
    CHECK(!k_pos_early);
    vsm_run(&v, &in, 20);
    CHECK(v.out.k_pos && v.out.k_pre);      /* overlap while K+ closes */
    vsm_run(&v, &in, CONTACTOR_DELAY_MS + 10u);
    CHECK(v.st == VSM_READY);
    CHECK(v.out.k_neg && v.out.k_pos && !v.out.k_pre);

    /* 95 % threshold */
    vsm_init(&v);
    in = vsm_default();
    in.key_on = true;
    in.vbus = 0.94f * in.vbatt;
    vsm_run(&v, &in, 500);
    CHECK(v.st == VSM_PRECHARGE && !v.out.k_pos);
    in.vbus = 0.96f * in.vbatt;
    vsm_run(&v, &in, 100);
    CHECK(v.st == VSM_READY);
    vt_note("K+ not closed before %u ms nor below 95 %% of Vbatt; precharge relay opened %u ms after K+",
            (unsigned)PRECHARGE_MIN_MS, (unsigned)CONTACTOR_DELAY_MS);
}

/* SVC-VSM-003: precharge timeout */
void svc_vsm_003(void)
{
    vsm_t v;
    vsm_in_t in = vsm_default();
    uint32_t t, t_fail = 0u;

    vsm_init(&v);
    in.key_on = true;
    in.vbus = 100.0f;
    for (t = 0; t < PRECHARGE_TIMEOUT_MS + 200u; t += VSM_PERIOD_MS) {
        vsm_step(&v, &in, VSM_PERIOD_MS);
        if (v.out.precharge_fail && t_fail == 0u) { t_fail = t; }
    }
    CHECK(t_fail >= PRECHARGE_TIMEOUT_MS - VSM_PERIOD_MS && t_fail <= PRECHARGE_TIMEOUT_MS + VSM_PERIOD_MS);
    CHECK(v.st == VSM_SHUTDOWN);
    CHECK(!v.out.k_pos && !v.out.k_pre);
    vt_note("precharge failure flagged at %u ms", (unsigned)t_fail);
}

/* SVC-VSM-004: conditions for entering DRIVE */
void svc_vsm_004(void)
{
    vsm_t v;
    vsm_in_t in;

    CHECK(vsm_to_ready(&v, &in));
    in.gear_sel = GEAR_D;                   /* no brake */
    vsm_run(&v, &in, 50);
    CHECK(v.st == VSM_READY && !v.out.inv_enable);

    in.brake = true;
    in.speed_rpm = GEAR_SHIFT_MAX_RPM + 10.0f;  /* moving */
    vsm_run(&v, &in, 50);
    CHECK(v.st == VSM_READY);

    in.speed_rpm = 0.0f;
    in.inverter_ready = false;              /* calibration not done / HW fault */
    vsm_run(&v, &in, 50);
    CHECK(v.st == VSM_READY);

    in.inverter_ready = true;
    in.reaction = REACT_LIMP;               /* limp still allows driving */
    vsm_run(&v, &in, 20);
    CHECK(v.st == VSM_DRIVE && v.out.gear == GEAR_D && v.out.inv_enable);

    CHECK(vsm_to_ready(&v, &in));
    in.brake = true;
    in.gear_sel = GEAR_R;
    vsm_run(&v, &in, 20);
    CHECK(v.st == VSM_DRIVE && v.out.gear == GEAR_R);
    vt_note("DRIVE refused without brake, above %.0f rpm, or inverter not ready", (double)GEAR_SHIFT_MAX_RPM);
}

/* SVC-VSM-005: gear changes while in DRIVE */
void svc_vsm_005(void)
{
    vsm_t v;
    vsm_in_t in;

    CHECK(vsm_to_drive(&v, &in));
    in.speed_rpm = 3000.0f;
    in.gear_sel = GEAR_R;                   /* D -> R at speed */
    vsm_run(&v, &in, 50);
    CHECK(v.out.gear == GEAR_D);
    in.gear_sel = GEAR_P;                   /* P at speed */
    vsm_run(&v, &in, 50);
    CHECK(v.st == VSM_DRIVE && v.out.gear == GEAR_D);
    in.gear_sel = GEAR_N;                   /* N always */
    vsm_run(&v, &in, 20);
    CHECK(v.out.gear == GEAR_N && v.out.inv_enable);
    in.gear_sel = GEAR_R;                   /* N -> R while rolling forward */
    vsm_run(&v, &in, 50);
    CHECK(v.out.gear == GEAR_N);
    in.gear_sel = GEAR_D;                   /* N -> D while rolling forward */
    vsm_run(&v, &in, 20);
    CHECK(v.out.gear == GEAR_D);

    in.speed_rpm = 0.0f;
    in.gear_sel = GEAR_R;                   /* D -> R at standstill without brake */
    vsm_run(&v, &in, 50);
    CHECK(v.out.gear == GEAR_D);
    in.brake = true;
    vsm_run(&v, &in, 20);
    CHECK(v.out.gear == GEAR_R);

    in.gear_sel = GEAR_P;                   /* P at standstill -> READY */
    vsm_run(&v, &in, 20);
    CHECK(v.st == VSM_READY && !v.out.inv_enable && v.out.gear == GEAR_P);
    vt_note("all gear-change rules exercised");
}

/* SVC-VSM-006: key off while moving */
void svc_vsm_006(void)
{
    vsm_t v;
    vsm_in_t in;

    CHECK(vsm_to_drive(&v, &in));
    in.speed_rpm = 3000.0f;
    in.key_on = false;
    vsm_run(&v, &in, 100);
    CHECK(v.st == VSM_DRIVE && v.out.gear == GEAR_N && v.out.inv_enable && v.out.k_pos);
    in.speed_rpm = KEYOFF_MAX_RPM + 10.0f;
    vsm_run(&v, &in, 50);
    CHECK(v.st == VSM_DRIVE);
    in.speed_rpm = KEYOFF_MAX_RPM - 10.0f;
    vsm_run(&v, &in, 10);
    CHECK(v.st == VSM_SHUTDOWN);
    vt_note("HV and inverter control kept until speed < %.0f rpm", (double)KEYOFF_MAX_RPM);
}

/* SVC-VSM-007: shutdown and active discharge */
void svc_vsm_007(void)
{
    vsm_t v;
    vsm_in_t in;

    CHECK(vsm_to_drive(&v, &in));
    in.key_on = false;
    vsm_run(&v, &in, 10);
    CHECK(v.st == VSM_SHUTDOWN);
    CHECK(!v.out.k_pos && !v.out.k_pre && v.out.k_neg && !v.out.inv_enable);
    vsm_run(&v, &in, CONTACTOR_DELAY_MS);
    CHECK(!v.out.k_neg && v.out.discharge);
    vsm_run(&v, &in, 100);
    CHECK(v.st == VSM_SHUTDOWN && v.out.discharge);   /* Vbus still high */
    in.vbus = HV_SAFE_V - 1.0f;
    vsm_run(&v, &in, 10);
    CHECK(v.st == VSM_STANDBY && !v.out.discharge);
    vt_note("K+ opened first, K- after %u ms, discharge until Vbus < %.0f V",
            (unsigned)CONTACTOR_DELAY_MS, (double)HV_SAFE_V);
}

/* SVC-VSM-008: discharge timeout */
void svc_vsm_008(void)
{
    vsm_t v;
    vsm_in_t in;
    uint32_t t, t_fail = 0u;

    CHECK(vsm_to_ready(&v, &in));
    in.key_on = false;
    for (t = 0; t < CONTACTOR_DELAY_MS + DISCHARGE_TIMEOUT_MS + 200u; t += VSM_PERIOD_MS) {
        vsm_step(&v, &in, VSM_PERIOD_MS);
        if (v.out.discharge_fail && t_fail == 0u) { t_fail = t; }
    }
    CHECK(t_fail > 0u && t_fail <= CONTACTOR_DELAY_MS + DISCHARGE_TIMEOUT_MS + 2u * VSM_PERIOD_MS);
    CHECK(v.st == VSM_STANDBY && !v.out.discharge);
    vt_note("discharge failure flagged %u ms after key off", (unsigned)t_fail);
}

/* SVC-VSM-009: fault reactions in the state machine */
void svc_vsm_009(void)
{
    vsm_t v;
    vsm_in_t in;

    CHECK(vsm_to_drive(&v, &in));
    in.reaction = REACT_TORQUE_OFF;
    vsm_run(&v, &in, 10);
    CHECK(v.st == VSM_FAULT && !v.out.inv_enable && v.out.k_pos && v.out.k_neg);
    in.reaction = REACT_NONE;               /* condition gone: stay in FAULT */
    vsm_run(&v, &in, 100);
    CHECK(v.st == VSM_FAULT);
    in.reaction = REACT_HV_OFF;
    vsm_run(&v, &in, 10);
    CHECK(v.st == VSM_SHUTDOWN && !v.out.k_pos);

    CHECK(vsm_to_ready(&v, &in));
    in.reaction = REACT_TORQUE_OFF;
    vsm_run(&v, &in, 10);
    CHECK(v.st == VSM_FAULT);

    vsm_init(&v);
    in = vsm_default();
    in.key_on = true;
    in.reaction = REACT_HV_OFF;             /* latched HV-off fault blocks precharge */
    vsm_run(&v, &in, 200);
    CHECK(v.st == VSM_STANDBY && !v.out.k_neg);
    vt_note("torque-off -> FAULT with HV kept; HV-off -> SHUTDOWN; HV-off blocks precharge");
}

/* ======================================================================= */
/* Fault manager                                                           */
/* ======================================================================= */

/* SVC-DIAG-001: reaction priority and debouncing */
void svc_diag_001(void)
{
    int k;
    diag_init();
    CHECK(diag_reaction() == REACT_NONE);
    diag_set(FLT_DISCHARGE);
    CHECK(diag_reaction() == REACT_WARN);
    diag_set(FLT_APPS_PLAUS);
    CHECK(diag_reaction() == REACT_LIMP);
    diag_set(FLT_SW_OVERCURRENT);
    CHECK(diag_reaction() == REACT_TORQUE_OFF);
    diag_set(FLT_BMS_TIMEOUT);
    CHECK(diag_reaction() == REACT_HV_OFF);
    CHECK(diag_mask() == ((1uL << FLT_DISCHARGE) | (1uL << FLT_APPS_PLAUS)
                        | (1uL << FLT_SW_OVERCURRENT) | (1uL << FLT_BMS_TIMEOUT)));

    diag_clear_all();
    for (k = 0; k < 9; k++) { diag_debounce(FLT_INV_OVERTEMP, true, 10u, 1u); }
    CHECK(!diag_is_set(FLT_INV_OVERTEMP));
    diag_debounce(FLT_INV_OVERTEMP, false, 10u, 1u);   /* interruption restarts debounce */
    for (k = 0; k < 9; k++) { diag_debounce(FLT_INV_OVERTEMP, true, 10u, 1u); }
    CHECK(!diag_is_set(FLT_INV_OVERTEMP));
    diag_debounce(FLT_INV_OVERTEMP, true, 10u, 1u);
    CHECK(diag_is_set(FLT_INV_OVERTEMP));
    diag_debounce(FLT_INV_OVERTEMP, false, 10u, 1u);   /* latched */
    CHECK(diag_is_set(FLT_INV_OVERTEMP));

    /* every fault has a defined reaction */
    for (k = 0; k < (int)FLT_COUNT; k++) {
        CHECK(diag_fault_reaction((diag_fault_t)k) != REACT_NONE);
    }
    diag_clear_all();
    vt_note("highest reaction wins; debounce restarts on interruption; %d faults mapped", (int)FLT_COUNT);
}

/* ======================================================================= */
/* CAN codec                                                               */
/* ======================================================================= */

/* SVC-COM-001: BMS message decoding */
void svc_com_001(void)
{
    bms_info_t b;
    bsp_can_frame_t st, lim;

    bms_info_init(&b);
    h_make_bms_frames(&st, &lim, 1u, 355.2f, true, false);
    st.data[2] = (uint8_t)(0xFF38u & 0xFFu);      /* -20.0 A */
    st.data[3] = (uint8_t)(0xFF38u >> 8);
    st.data[7] = can_xor_checksum(st.data, 7u);
    CHECK(can_decode_bms(&b, &st));
    CHECK(can_decode_bms(&b, &lim));
    CHECK_NEAR(b.pack_v, 355.2, 0.05);
    CHECK_NEAR(b.pack_i, -20.0, 0.05);
    CHECK_NEAR(b.soc, 80.0, 0.01);
    CHECK(b.ready && !b.fault);
    CHECK_NEAR(b.dis_i_max, 400.0, 0.01);
    CHECK_NEAR(b.chg_i_max, 200.0, 0.01);
    h_make_bms_frames(&st, &lim, 2u, 355.2f, false, true);
    CHECK(can_decode_bms(&b, &st));
    CHECK(!b.ready && b.fault);
    vt_note("decoded 355.2 V, -20.0 A, 80 %% SOC, limits 400/200 A, flags");
}

/* SVC-COM-002: rejection of invalid BMS frames */
void svc_com_002(void)
{
    bms_info_t b;
    bsp_can_frame_t st, lim, bad;

    bms_info_init(&b);
    h_make_bms_frames(&st, &lim, 3u, 300.0f, true, false);
    CHECK(can_decode_bms(&b, &st));
    CHECK(!can_decode_bms(&b, &st));               /* same alive counter */

    h_make_bms_frames(&st, &lim, 4u, 999.0f, true, false);
    bad = st;
    bad.data[0] ^= 0x01u;                          /* checksum error */
    CHECK(!can_decode_bms(&b, &bad));
    bad = st;
    bad.dlc = 7u;                                  /* wrong length */
    CHECK(!can_decode_bms(&b, &bad));
    bad = st;
    bad.id = 0x123u;                               /* foreign ID */
    CHECK(!can_decode_bms(&b, &bad));
    CHECK_NEAR(b.pack_v, 300.0, 0.05);             /* unchanged */
    CHECK(can_decode_bms(&b, &st));                /* valid next frame accepted */
    CHECK_NEAR(b.pack_v, 999.0, 0.05);
    vt_note("rejected: repeated alive counter, checksum error, DLC 7, foreign ID");
}

/* SVC-COM-003: BMS timeout */
void svc_com_003(void)
{
    bms_info_t b;
    bsp_can_frame_t st, lim;

    bms_info_init(&b);
    CHECK(bms_info_timeout(&b));                   /* nothing received yet */
    h_make_bms_frames(&st, &lim, 1u, 350.0f, true, false);
    (void)can_decode_bms(&b, &st);
    (void)can_decode_bms(&b, &lim);
    CHECK(!bms_info_timeout(&b));
    bms_info_tick(&b, BMS_TIMEOUT_MS - 1u);
    CHECK(!bms_info_timeout(&b));
    bms_info_tick(&b, 1u);
    CHECK(bms_info_timeout(&b));

    /* only one of the two messages missing is also a timeout */
    bms_info_init(&b);
    (void)can_decode_bms(&b, &st);
    CHECK(bms_info_timeout(&b));
    vt_note("timeout at %u ms; missing BMS_LIMITS alone also times out", (unsigned)BMS_TIMEOUT_MS);
}
