/**
 * @file test_main.c
 * @brief Host unit and integration tests for the portable firmware parts.
 *
 * Build and run: make test
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "ev_config.h"
#include "board_cfg.h"
#include "mc_math.h"
#include "pi.h"
#include "svm.h"
#include "pll.h"
#include "foc.h"
#include "pedal.h"
#include "torque_mgr.h"
#include "vehicle_sm.h"
#include "diag.h"
#include "can_msgs.h"
#include "app.h"
#include "bsp_stub.h"

static int s_fail;
static int s_checks;

#define CHECK(cond) do { s_checks++; if (!(cond)) { s_fail++; \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_NEAR(a, b, tol) do { s_checks++; double _a = (a), _b = (b); \
    if (fabs(_a - _b) > (tol)) { s_fail++; \
    printf("  FAIL %s:%d: %s = %g, expected %g +/- %g\n", __FILE__, __LINE__, #a, _a, _b, (double)(tol)); } } while (0)

/* ======================================================================= */
static void test_transforms(void)
{
    mc_abc_t i = { 10.0f, -3.0f, -7.0f };
    mc_ab_t ab = mc_clarke(i);
    mc_abc_t back = mc_iclarke(ab);
    float th = 1.1f;
    mc_dq_t dq = mc_park(ab, sinf(th), cosf(th));
    mc_ab_t ab2 = mc_ipark(dq, sinf(th), cosf(th));

    CHECK_NEAR(back.a, i.a, 1e-4);
    CHECK_NEAR(back.b, i.b, 1e-4);
    CHECK_NEAR(back.c, i.c, 1e-4);
    CHECK_NEAR(ab2.alpha, ab.alpha, 1e-4);
    CHECK_NEAR(ab2.beta, ab.beta, 1e-4);
    CHECK_NEAR(mc_wrap_pi(3.5f), 3.5f - MC_2PI, 1e-5);
    CHECK_NEAR(mc_wrap_2pi(-0.5f), MC_2PI - 0.5f, 1e-5);
}

static void test_pi(void)
{
    pi_t p;
    int k;
    pi_init(&p, 1.0f, 100.0f, 0.001f, -5.0f, 5.0f);
    for (k = 0; k < 10000; k++) { (void)pi_step(&p, 10.0f, 0.0f); }
    CHECK(p.integ <= 5.0f);                       /* no windup beyond limit */
    /* error reverses: output must leave saturation immediately */
    CHECK(pi_step(&p, -10.0f, 0.0f) < 5.0f);
}

static void test_svm(void)
{
    float d[3];
    float vdc = 400.0f;
    float vmax = vdc * MC_INV_SQRT3;
    int k;

    svm_calc(0.0f, 0.0f, vdc, d);
    CHECK_NEAR(d[0], 0.5, 1e-6);
    CHECK_NEAR(d[1], 0.5, 1e-6);
    CHECK_NEAR(d[2], 0.5, 1e-6);

    /* At 95 % of the linear limit duties stay inside [0,1] and the
     * reconstructed line-to-line voltages match the request. */
    for (k = 0; k < 360; k += 7) {
        float th = (float)k * MC_PI / 180.0f;
        float va = 0.95f * vmax * cosf(th), vb = 0.95f * vmax * sinf(th);
        mc_ab_t ab = { va, vb };
        mc_abc_t ref = mc_iclarke(ab);
        svm_calc(va, vb, vdc, d);
        CHECK(d[0] >= INV_DUTY_MIN - 1e-6f && d[0] <= INV_DUTY_MAX + 1e-6f);
        CHECK_NEAR((d[0] - d[1]) * vdc, ref.a - ref.b, 0.05);
    }
}

static void test_pll(void)
{
    pll_t p;
    float th = 0.0f;
    float w = 1000.0f;            /* rad/s electrical */
    int k;
    pll_init(&p, PLL_BW_HZ, TS_FAST);
    for (k = 0; k < 2000; k++) {
        th = mc_wrap_2pi(th + w * TS_FAST);
        pll_update(&p, th);
    }
    CHECK_NEAR(p.omega, w, 1.0);
    /* estimate is the prediction for the next sample */
    CHECK_NEAR(mc_wrap_pi(p.theta - th - w * TS_FAST), 0.0, 0.01);
}

/* ---------------- closed-loop FOC with an IPMSM model ---------------- */
typedef struct { float id, iq, theta; } motor_t;

/* Averaged inverter + dq motor model, constant mechanical speed */
static void motor_step(motor_t *m, const float duty[3], float vdc, float omega_e, float ts)
{
    const int sub = 20;
    float h = ts / (float)sub;
    float vmean = (duty[0] + duty[1] + duty[2]) * vdc / 3.0f;
    mc_abc_t v = { duty[0] * vdc - vmean, duty[1] * vdc - vmean, duty[2] * vdc - vmean };
    int n;
    for (n = 0; n < sub; n++) {
        mc_dq_t vdq = mc_park(mc_clarke(v), sinf(m->theta), cosf(m->theta));
        float did = (vdq.d - MOTOR_RS_OHM * m->id + omega_e * MOTOR_LQ_H * m->iq) / MOTOR_LD_H;
        float diq = (vdq.q - MOTOR_RS_OHM * m->iq - omega_e * (MOTOR_LD_H * m->id + MOTOR_PSI_WB)) / MOTOR_LQ_H;
        m->id += did * h;
        m->iq += diq * h;
        m->theta = mc_wrap_2pi(m->theta + omega_e * h);
    }
}

static mc_abc_t motor_iabc(const motor_t *m)
{
    mc_dq_t i = { m->id, m->iq };
    return mc_iclarke(mc_ipark(i, sinf(m->theta), cosf(m->theta)));
}

typedef struct {
    float rpm_start, rpm_end;   /* linear speed ramp over ramp_steps */
    int ramp_steps;
    float torque;
    float torque_rate;          /* Nm per step, 0 = step */
    bool from_asc;              /* start in active short circuit steady state */
    int steps;
} sim_cfg_t;

static float s_sim_peak;

static void sim_foc(const sim_cfg_t *cfg, foc_t *f, motor_t *m)
{
    foc_params_t prm = { MOTOR_RS_OHM, MOTOR_LD_H, MOTOR_LQ_H, MOTOR_PSI_WB, MOTOR_POLE_PAIRS,
                         TS_FAST, INV_I_MAX_A, INV_I_FW_MAX_A, CURRENT_LOOP_BW_HZ };
    float duty_applied[3] = { 0.5f, 0.5f, 0.5f };
    float k_rpm = MC_RPM_TO_RADS * MOTOR_POLE_PAIRS;
    float omega_e = cfg->rpm_start * k_rpm;
    int k;

    foc_init(f, &prm);
    memset(m, 0, sizeof(*m));
    s_sim_peak = 0.0f;
    if (cfg->from_asc) {
        /* ASC steady state: 0 = R*i + j*w*(L*i + psi) */
        float a = MOTOR_RS_OHM, b = omega_e * MOTOR_LQ_H, c = omega_e * MOTOR_LD_H;
        float det = a * a + b * c;
        mc_dq_t i0;
        m->id = -(b * omega_e * MOTOR_PSI_WB) / det;
        m->iq = -(a * omega_e * MOTOR_PSI_WB) / det;
        i0.d = m->id;
        i0.q = m->iq;
        foc_flying_start(f, i0, omega_e, 350.0f);
    }
    for (k = 0; k < cfg->steps; k++) {
        float imag;
        if (k < cfg->ramp_steps) {
            omega_e = (cfg->rpm_start + (cfg->rpm_end - cfg->rpm_start) * (float)k / (float)cfg->ramp_steps) * k_rpm;
        } else {
            omega_e = cfg->rpm_end * k_rpm;
        }
        f->torque_ref = (cfg->torque_rate > 0.0f)
            ? mc_ramp(f->torque_ref, cfg->torque, cfg->torque_rate) : cfg->torque;
        /* sample at start of period, apply previous period's duty (1 period delay) */
        foc_step(f, motor_iabc(m), m->theta, omega_e, 350.0f);
        motor_step(m, duty_applied, 350.0f, omega_e, TS_FAST);
        memcpy(duty_applied, f->duty, sizeof(duty_applied));
        imag = sqrtf(m->id * m->id + m->iq * m->iq);
        if (k > 0 && imag > s_sim_peak) { s_sim_peak = imag; }
    }
}

static float motor_torque(const motor_t *m)
{
    return 1.5f * MOTOR_POLE_PAIRS * (MOTOR_PSI_WB * m->iq + (MOTOR_LD_H - MOTOR_LQ_H) * m->id * m->iq);
}

static void test_foc_low_speed(void)
{
    foc_t f;
    motor_t m;
    float id, iq;

    foc_mtpa(&(foc_params_t){ MOTOR_RS_OHM, MOTOR_LD_H, MOTOR_LQ_H, MOTOR_PSI_WB,
                              MOTOR_POLE_PAIRS, TS_FAST, INV_I_MAX_A, 0, 0 }, 150.0f, &id, &iq);
    CHECK(id < 0.0f);                              /* IPMSM uses negative id */
    CHECK_NEAR(1.5f * MOTOR_POLE_PAIRS * (MOTOR_PSI_WB * iq + (MOTOR_LD_H - MOTOR_LQ_H) * id * iq), 150.0, 0.5);

    {
        sim_cfg_t cfg = { 2000.0f, 2000.0f, 0, 150.0f, 0.0f, false, 500 };   /* 50 ms */
        sim_foc(&cfg, &f, &m);
    }
    printf("  2000 rpm: id=%.1f (ref %.1f) iq=%.1f (ref %.1f) T=%.1f Nm m=%.2f\n",
           (double)m.id, (double)f.i_ref.d, (double)m.iq, (double)f.i_ref.q,
           (double)motor_torque(&m), (double)f.mod_index);
    CHECK_NEAR(m.id, f.i_ref.d, 3.0);
    CHECK_NEAR(m.iq, f.i_ref.q, 3.0);
    CHECK_NEAR(motor_torque(&m), 150.0, 3.0);
    CHECK(fabsf(f.id_fw) < 1.0f);                  /* no field weakening needed */
}

static void test_foc_regen(void)
{
    foc_t f;
    motor_t m;
    sim_cfg_t cfg = { 3000.0f, 3000.0f, 0, -100.0f, 0.0f, false, 500 };
    sim_foc(&cfg, &f, &m);
    printf("  3000 rpm regen: T=%.1f Nm\n", (double)motor_torque(&m));
    CHECK_NEAR(motor_torque(&m), -100.0, 3.0);
}

static void check_fw_result(const char *name, const foc_t *f, const motor_t *m, float t_min, float t_max)
{
    float imag = sqrtf(m->id * m->id + m->iq * m->iq);
    printf("  %s: id=%.1f iq=%.1f |i|=%.1f peak=%.1f T=%.1f Nm m=%.2f id_fw=%.1f\n", name,
           (double)m->id, (double)m->iq, (double)imag, (double)s_sim_peak,
           (double)motor_torque(m), (double)f->mod_index, (double)f->id_fw);
    CHECK(f->id_fw < -10.0f);                      /* field weakening active */
    CHECK(f->mod_index <= INV_VOLTAGE_MARGIN + 0.01f);
    CHECK(imag <= INV_I_MAX_A + 5.0f);
    CHECK(s_sim_peak < INV_I_TRIP_A);              /* no overcurrent trip on the way */
    CHECK_NEAR(m->id, f->i_ref.d, 5.0);
    CHECK_NEAR(m->iq, f->i_ref.q, 5.0);
    CHECK(motor_torque(m) >= t_min && motor_torque(m) <= t_max);
}

static void test_foc_field_weakening(void)
{
    foc_t f;
    motor_t m;
    /* accelerate 0 -> 12000 rpm in 1.5 s at 40 Nm: back-EMF at top speed
     * (302 V peak) is far above Vdc/sqrt3 (202 V) */
    sim_cfg_t up = { 0.0f, 12000.0f, 15000, 40.0f, 0.0f, false, 20000 };
    /* regen at 12000 rpm: request exceeds the voltage-limited capability */
    sim_cfg_t regen = { 6000.0f, 12000.0f, 5000, -60.0f, 0.0f, false, 10000 };
    /* flying start from active short circuit at 9000 rpm, ramped torque */
    sim_cfg_t fly = { 9000.0f, 9000.0f, 0, 60.0f, 0.15f, true, 5000 };
    /* flying start from ASC at 12000 rpm with a torque step */
    sim_cfg_t fly_step = { 12000.0f, 12000.0f, 0, 40.0f, 0.0f, true, 5000 };

    sim_foc(&up, &f, &m);
    check_fw_result("0->12000 rpm 40 Nm", &f, &m, 39.0f, 41.0f);
    sim_foc(&regen, &f, &m);
    check_fw_result("12000 rpm regen -60 Nm", &f, &m, -60.0f, -30.0f);
    sim_foc(&fly, &f, &m);
    check_fw_result("ASC->run 9000 rpm 60 Nm", &f, &m, 59.0f, 61.0f);
    sim_foc(&fly_step, &f, &m);
    check_fw_result("ASC->run 12000 rpm 40 Nm step", &f, &m, 39.0f, 41.0f);
}

/* ======================================================================= */
static void test_pedal(void)
{
    pedal_t p;
    int k;
    pedal_init(&p);
    pedal_update(&p, 0.8f, 0.4f, 1u);
    CHECK(p.valid);
    CHECK_NEAR(p.pos, 0.0, 1e-6);

    pedal_update(&p, 2.5f, 1.25f, 1u);           /* 50 % */
    CHECK(p.valid);
    CHECK_NEAR(p.pos, 0.4845, 0.01);

    /* track mismatch: immediately zero, latched after debounce */
    for (k = 0; k < (int)APPS_PLAUS_DEBOUNCE_MS - 1; k++) { pedal_update(&p, 3.0f, 0.5f, 1u); }
    CHECK(!p.valid && !p.plaus_err);
    CHECK_NEAR(p.pos, 0.0, 1e-6);
    pedal_update(&p, 3.0f, 0.5f, 1u);
    CHECK(p.plaus_err);
    pedal_update(&p, 2.5f, 1.25f, 1u);           /* stays latched */
    CHECK(!p.valid);

    pedal_init(&p);
    for (k = 0; k < (int)APPS_RANGE_DEBOUNCE_MS; k++) { pedal_update(&p, 4.9f, 2.45f, 1u); }
    CHECK(p.range_err);
}

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

static void test_torque_mgr(void)
{
    tq_mgr_t m;
    tq_in_t in = tq_default();
    float t;

    tq_mgr_init(&m);
    in.pedal = 1.0f;
    in.speed_rpm = 1000.0f;
    CHECK_NEAR(tq_settle(&m, &in), MOTOR_MAX_TORQUE_NM, 0.1);

    /* rate limit */
    tq_mgr_init(&m);
    t = tq_mgr_step(&m, &in, TS_SLOW);
    CHECK(t <= TORQUE_RATE_ZERO_NMPS * TS_SLOW + 1e-4f);

    /* power limit at high speed */
    tq_mgr_init(&m);
    in.speed_rpm = 8000.0f;
    CHECK_NEAR(tq_settle(&m, &in), MOTOR_MAX_POWER_W / (8000.0f * MC_RPM_TO_RADS), 0.5);

    /* brake-throttle override */
    tq_mgr_init(&m);
    in.speed_rpm = 3000.0f;
    in.brake = true;
    t = tq_settle(&m, &in);
    CHECK(m.bto_active);
    CHECK(t < 0.0f);                               /* brake regen */
    in.brake = false;                              /* pedal still pressed: stays overridden */
    CHECK(tq_settle(&m, &in) <= 0.0f);
    in.pedal = 0.0f;                               /* release pedal: override clears */
    (void)tq_settle(&m, &in);
    CHECK(!m.bto_active);

    /* coast regen fades out at standstill */
    tq_mgr_init(&m);
    in.speed_rpm = 2000.0f;
    CHECK_NEAR(tq_settle(&m, &in), -REGEN_COAST_NM, 0.1);
    in.speed_rpm = 100.0f;
    CHECK_NEAR(tq_settle(&m, &in), 0.0, 0.01);

    /* reverse: negative torque, scaled */
    tq_mgr_init(&m);
    in.gear = GEAR_R;
    in.pedal = 1.0f;
    in.speed_rpm = -500.0f;
    CHECK_NEAR(tq_settle(&m, &in), -MOTOR_MAX_TORQUE_NM * REVERSE_TORQUE_SCALE, 0.1);

    /* over-temperature derating and limp */
    tq_mgr_init(&m);
    in = tq_default();
    in.pedal = 1.0f;
    in.speed_rpm = 1000.0f;
    in.t_inv = 0.5f * (TEMP_INV_DERATE_C + TEMP_INV_TRIP_C);
    CHECK_NEAR(tq_settle(&m, &in), 0.5f * MOTOR_MAX_TORQUE_NM, 0.5);
    in.t_inv = 40.0f;
    in.reaction = REACT_LIMP;
    CHECK_NEAR(tq_settle(&m, &in), LIMP_TORQUE_SCALE * MOTOR_MAX_TORQUE_NM, 0.5);

    /* torque-off reaction: immediate zero */
    in.reaction = REACT_TORQUE_OFF;
    CHECK_NEAR(tq_mgr_step(&m, &in, TS_SLOW), 0.0, 1e-6);

    /* battery discharge limit */
    tq_mgr_init(&m);
    in = tq_default();
    in.pedal = 1.0f;
    in.speed_rpm = 3000.0f;
    in.bms_dis_i_max = 50.0f;
    CHECK_NEAR(tq_settle(&m, &in), 350.0f * 50.0f * DRIVETRAIN_EFF / (3000.0f * MC_RPM_TO_RADS), 0.5);
}

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
    for (t = 0; t < ms; t += 10u) { vsm_step(v, in, 10u); }
}

static void test_vsm(void)
{
    vsm_t v;
    vsm_in_t in = vsm_default();

    vsm_init(&v);
    vsm_run(&v, &in, 50);
    CHECK(v.st == VSM_STANDBY);
    CHECK(!v.out.k_neg && !v.out.k_pos);

    in.key_on = true;
    vsm_run(&v, &in, 20);                           /* outputs follow one cycle after entry */
    CHECK(v.st == VSM_PRECHARGE);
    CHECK(v.out.k_neg && v.out.k_pre && !v.out.k_pos);

    in.vbus = 340.0f;                               /* DC link charged */
    vsm_run(&v, &in, 200);
    CHECK(v.st == VSM_READY);
    CHECK(v.out.k_neg && v.out.k_pos && !v.out.k_pre);
    CHECK(!v.out.inv_enable);

    in.gear_sel = GEAR_D;                           /* without brake: refused */
    vsm_run(&v, &in, 50);
    CHECK(v.st == VSM_READY);
    in.brake = true;
    vsm_run(&v, &in, 20);
    CHECK(v.st == VSM_DRIVE && v.out.gear == GEAR_D && v.out.inv_enable);

    in.brake = false;
    in.speed_rpm = 3000.0f;
    in.gear_sel = GEAR_R;                           /* D->R at speed: refused */
    vsm_run(&v, &in, 50);
    CHECK(v.out.gear == GEAR_D);
    in.gear_sel = GEAR_P;                           /* P at speed: refused */
    vsm_run(&v, &in, 50);
    CHECK(v.st == VSM_DRIVE);

    in.key_on = false;                              /* key off at speed: keep control, neutral */
    vsm_run(&v, &in, 50);
    CHECK(v.st == VSM_DRIVE && v.out.gear == GEAR_N && v.out.inv_enable);

    in.speed_rpm = 0.0f;                            /* slow: shutdown sequence */
    vsm_run(&v, &in, 10);
    CHECK(v.st == VSM_SHUTDOWN);
    CHECK(!v.out.k_pos && !v.out.inv_enable);
    vsm_run(&v, &in, 100);
    CHECK(!v.out.k_neg && v.out.discharge);
    in.vbus = 30.0f;
    vsm_run(&v, &in, 10);
    CHECK(v.st == VSM_STANDBY && !v.out.discharge);

    /* precharge timeout */
    vsm_init(&v);
    in = vsm_default();
    in.key_on = true;
    in.vbus = 100.0f;
    {
        uint32_t t;
        bool fail = false;
        for (t = 0; t < PRECHARGE_TIMEOUT_MS + 100u; t += 10u) {
            vsm_step(&v, &in, 10u);
            fail = fail || v.out.precharge_fail;
        }
        CHECK(fail);
        CHECK(v.st == VSM_SHUTDOWN);
    }

    /* torque-off fault in drive -> FAULT, HV stays; HV-off -> shutdown */
    vsm_init(&v);
    in = vsm_default();
    in.key_on = true;
    in.vbus = 350.0f;
    in.brake = true;
    in.gear_sel = GEAR_D;
    vsm_run(&v, &in, 400);
    CHECK(v.st == VSM_DRIVE);
    in.reaction = REACT_TORQUE_OFF;
    vsm_run(&v, &in, 10);
    CHECK(v.st == VSM_FAULT && !v.out.inv_enable && v.out.k_pos);
    in.reaction = REACT_HV_OFF;
    vsm_run(&v, &in, 10);
    CHECK(v.st == VSM_SHUTDOWN);
}

static void test_diag(void)
{
    int k;
    diag_init();
    CHECK(diag_reaction() == REACT_NONE);
    diag_set(FLT_APPS_PLAUS);
    CHECK(diag_reaction() == REACT_LIMP);
    diag_set(FLT_SW_OVERCURRENT);
    CHECK(diag_reaction() == REACT_TORQUE_OFF);
    diag_clear_all();
    for (k = 0; k < 9; k++) { diag_debounce(FLT_INV_OVERTEMP, true, 10u, 1u); }
    CHECK(!diag_is_set(FLT_INV_OVERTEMP));
    diag_debounce(FLT_INV_OVERTEMP, true, 10u, 1u);
    CHECK(diag_is_set(FLT_INV_OVERTEMP));
    diag_clear_all();
}

/* ======================================================================= */
static void make_bms_frames(bsp_can_frame_t *st, bsp_can_frame_t *lim, uint8_t alive,
                            float pack_v, bool ready, bool fault)
{
    uint16_t v = (uint16_t)(pack_v * 10.0f);
    memset(st, 0, sizeof(*st));
    st->id = CAN_ID_BMS_STATUS;
    st->dlc = 8;
    st->data[0] = (uint8_t)v;
    st->data[1] = (uint8_t)(v >> 8);
    st->data[4] = 160;                              /* 80 % */
    st->data[5] = (uint8_t)((ready ? 1u : 0u) | (fault ? 2u : 0u));
    st->data[6] = alive & 0x0Fu;
    st->data[7] = can_xor_checksum(st->data, 7);

    memset(lim, 0, sizeof(*lim));
    lim->id = CAN_ID_BMS_LIMITS;
    lim->dlc = 8;
    lim->data[0] = (uint8_t)(4000u & 0xFFu);        /* 400 A */
    lim->data[1] = (uint8_t)(4000u >> 8);
    lim->data[2] = (uint8_t)(2000u & 0xFFu);        /* 200 A */
    lim->data[3] = (uint8_t)(2000u >> 8);
    lim->data[6] = alive & 0x0Fu;
    lim->data[7] = can_xor_checksum(lim->data, 7);
}

static void test_can(void)
{
    bms_info_t b;
    bsp_can_frame_t st, lim;
    vcu_info_t v;
    bsp_can_frame_t tx;

    bms_info_init(&b);
    CHECK(bms_info_timeout(&b));
    make_bms_frames(&st, &lim, 1, 355.2f, true, false);
    CHECK(can_decode_bms(&b, &st));
    CHECK(can_decode_bms(&b, &lim));
    CHECK_NEAR(b.pack_v, 355.2, 0.05);
    CHECK(b.ready && !b.fault);
    CHECK_NEAR(b.dis_i_max, 400.0, 0.01);
    CHECK_NEAR(b.chg_i_max, 200.0, 0.01);
    CHECK(!bms_info_timeout(&b));

    CHECK(!can_decode_bms(&b, &st));                /* same alive counter: rejected */
    make_bms_frames(&st, &lim, 2, 355.2f, true, false);
    st.data[0] ^= 0x01u;                            /* corrupt: checksum fails */
    CHECK(!can_decode_bms(&b, &st));

    bms_info_tick(&b, BMS_TIMEOUT_MS);
    CHECK(bms_info_timeout(&b));

    memset(&v, 0, sizeof(v));
    v.speed_rpm = -1234.0f;
    v.torque_nm = 123.4f;
    can_encode_vcu_status(&v, 5, &tx);
    CHECK(tx.id == CAN_ID_VCU_STATUS);
    CHECK((int16_t)(tx.data[2] | (tx.data[3] << 8)) == -1234);
    CHECK((int16_t)(tx.data[4] | (tx.data[5] << 8)) == 1234);
}

/* ======================================================================= */
/* Integration: application + BSP stub                                     */
/* ======================================================================= */
static uint8_t s_alive;

static uint16_t volts_to_counts(float v) { return (uint16_t)(v / ADC_V_PER_COUNT + 0.5f); }

static void run_ms(unsigned ms, float vdc)
{
    unsigned t, k;
    for (t = 0; t < ms; t++) {
        g_stub.adc_fast.vdc = (uint16_t)(vdc / VDC_V_PER_COUNT + 0.5f);
        for (k = 0; k < (unsigned)(PWM_FREQ_HZ / 1000.0f); k++) { app_fast_isr(); }
        if ((t % 10u) == 0u) {
            bsp_can_frame_t st, lim;
            make_bms_frames(&st, &lim, ++s_alive, 350.0f, true, false);
            bsp_stub_push_rx(&st);
            bsp_stub_push_rx(&lim);
        }
        app_task_1ms();
        app_background();
    }
}

static void test_app_integration(void)
{
    float vdc = 0.0f;

    bsp_stub_reset();
    g_stub.adc_slow.apps1 = volts_to_counts(0.8f);
    g_stub.adc_slow.apps2 = volts_to_counts(0.4f);
    g_stub.adc_slow.t_inv = volts_to_counts(2.5f);  /* 25 C */
    g_stub.adc_slow.t_mot = volts_to_counts(2.5f);
    g_stub.di[DI_GEAR_P] = true;

    app_init();
    run_ms(200, vdc);                               /* current sensor offset calibration */
    CHECK(app_current_cal_done());
    CHECK(app_vsm_state() == VSM_STANDBY);
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    CHECK(g_stub.wdg_kicks > 10u);                  /* supervision OK -> watchdog served */
    CHECK(g_stub.tx_count > 0u);

    g_stub.di[DI_KEY] = true;
    run_ms(50, vdc);
    CHECK(app_vsm_state() == VSM_PRECHARGE);
    CHECK(g_stub.dout[DO_K_NEG] && g_stub.dout[DO_K_PRE]);

    vdc = 345.0f;                                   /* precharge complete */
    run_ms(200, vdc);
    CHECK(app_vsm_state() == VSM_READY);
    CHECK(g_stub.dout[DO_K_POS] && !g_stub.dout[DO_K_PRE]);

    g_stub.di[DI_BRAKE] = true;                     /* brake + D */
    g_stub.di[DI_GEAR_P] = false;
    g_stub.di[DI_GEAR_D] = true;
    run_ms(30, vdc);
    CHECK(app_vsm_state() == VSM_DRIVE);
    CHECK(app_gear() == GEAR_D);
    CHECK(g_stub.pwm == STUB_PWM_RUN);

    g_stub.di[DI_BRAKE] = false;
    g_stub.adc_slow.apps1 = volts_to_counts(2.5f);  /* ~50 % pedal */
    g_stub.adc_slow.apps2 = volts_to_counts(1.25f);
    run_ms(300, vdc);
    printf("  integration: torque cmd at 50%% pedal = %.1f Nm\n", (double)app_torque_cmd());
    CHECK(app_torque_cmd() > 100.0f);

    /* gate driver desaturation -> torque off, FAULT, outputs off */
    g_stub.hw_fault = true;
    run_ms(20, vdc);
    CHECK(app_vsm_state() == VSM_FAULT);
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    CHECK(app_torque_cmd() == 0.0f);
    CHECK(g_stub.dout[DO_K_POS]);                   /* HV kept: torque-off reaction */

    /* key off -> shutdown, discharge, standby */
    g_stub.di[DI_KEY] = false;
    run_ms(100, vdc);
    CHECK(app_vsm_state() == VSM_SHUTDOWN);
    CHECK(!g_stub.dout[DO_K_POS] && !g_stub.dout[DO_K_NEG]);
    CHECK(g_stub.dout[DO_DISCHARGE]);
    vdc = 20.0f;
    run_ms(30, vdc);
    CHECK(app_vsm_state() == VSM_STANDBY);

    /* new key cycle clears the latched (now gone) fault */
    g_stub.hw_fault = false;
    g_stub.di[DI_KEY] = true;
    run_ms(20, vdc);
    CHECK(app_vsm_state() == VSM_PRECHARGE);
}

/* ======================================================================= */
int main(void)
{
    struct { const char *name; void (*fn)(void); } tests[] = {
        { "transforms", test_transforms },
        { "pi", test_pi },
        { "svm", test_svm },
        { "pll", test_pll },
        { "foc_low_speed", test_foc_low_speed },
        { "foc_regen", test_foc_regen },
        { "foc_field_weakening", test_foc_field_weakening },
        { "pedal", test_pedal },
        { "torque_mgr", test_torque_mgr },
        { "vehicle_sm", test_vsm },
        { "diag", test_diag },
        { "can", test_can },
        { "app_integration", test_app_integration },
    };
    unsigned i;
    for (i = 0; i < sizeof(tests) / sizeof(tests[0]); i++) {
        int before = s_fail;
        printf("[ RUN  ] %s\n", tests[i].name);
        tests[i].fn();
        printf("[ %s ] %s\n", (s_fail == before) ? " OK " : "FAIL", tests[i].name);
    }
    printf("\n%d checks, %d failed\n", s_checks, s_fail);
    return s_fail ? 1 : 0;
}
