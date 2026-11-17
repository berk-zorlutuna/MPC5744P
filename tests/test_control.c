/**
 * @file test_control.c
 * @brief Verification cases for the motor control software (SWR-CTL-xxx).
 */
#include <math.h>
#include <string.h>
#include "vtest.h"
#include "ev_config.h"
#include "mc_math.h"
#include "pi.h"
#include "svm.h"
#include "pll.h"
#include "foc.h"
#include "sim_motor.h"

#define VDC_NOM 350.0f

/* SVC-CTL-001: Clarke/Park transforms */
void svc_ctl_001(void)
{
    mc_abc_t i = { 10.0f, -3.0f, -7.0f };
    mc_ab_t ab = mc_clarke(i);
    mc_abc_t back = mc_iclarke(ab);
    float th = 1.1f;
    mc_dq_t dq = mc_park(ab, sinf(th), cosf(th));
    mc_ab_t ab2 = mc_ipark(dq, sinf(th), cosf(th));
    mc_abc_t bal;
    mc_dq_t dq_bal;

    CHECK_NEAR(back.a, i.a, 1e-4);
    CHECK_NEAR(back.b, i.b, 1e-4);
    CHECK_NEAR(back.c, i.c, 1e-4);
    CHECK_NEAR(ab2.alpha, ab.alpha, 1e-4);
    CHECK_NEAR(ab2.beta, ab.beta, 1e-4);

    /* amplitude invariance: balanced 100 A peak set aligned with theta
     * gives d = 100 A, q = 0 */
    bal.a = 100.0f * cosf(th);
    bal.b = 100.0f * cosf(th - MC_2PI / 3.0f);
    bal.c = 100.0f * cosf(th + MC_2PI / 3.0f);
    dq_bal = mc_park(mc_clarke(bal), sinf(th), cosf(th));
    CHECK_NEAR(dq_bal.d, 100.0, 1e-3);
    CHECK_NEAR(dq_bal.q, 0.0, 1e-3);

    CHECK_NEAR(mc_wrap_pi(3.5f), 3.5f - MC_2PI, 1e-5);
    CHECK_NEAR(mc_wrap_2pi(-0.5f), MC_2PI - 0.5f, 1e-5);
    vt_note("roundtrip error < 1e-4, balanced 100 A -> d=%.4f q=%.4f", (double)dq_bal.d, (double)dq_bal.q);
}

/* SVC-CTL-002: space vector modulation */
void svc_ctl_002(void)
{
    float d[3];
    float vmax = VDC_NOM * MC_INV_SQRT3;
    float max_err = 0.0f;
    int k;

    svm_calc(0.0f, 0.0f, VDC_NOM, d);
    CHECK_NEAR(d[0], 0.5, 1e-6);
    CHECK_NEAR(d[1], 0.5, 1e-6);
    CHECK_NEAR(d[2], 0.5, 1e-6);

    for (k = 0; k < 360; k += 5) {
        float th = (float)k * MC_PI / 180.0f;
        float va = 0.95f * vmax * cosf(th), vb = 0.95f * vmax * sinf(th);
        mc_ab_t ab = { va, vb };
        mc_abc_t ref = mc_iclarke(ab);
        float e1, e2;
        svm_calc(va, vb, VDC_NOM, d);
        CHECK(d[0] >= INV_DUTY_MIN && d[0] <= INV_DUTY_MAX);
        CHECK(d[1] >= INV_DUTY_MIN && d[1] <= INV_DUTY_MAX);
        CHECK(d[2] >= INV_DUTY_MIN && d[2] <= INV_DUTY_MAX);
        e1 = fabsf((d[0] - d[1]) * VDC_NOM - (ref.a - ref.b));
        e2 = fabsf((d[1] - d[2]) * VDC_NOM - (ref.b - ref.c));
        if (e1 > max_err) { max_err = e1; }
        if (e2 > max_err) { max_err = e2; }
    }
    CHECK(max_err < 0.05f);

    /* overmodulation request: duties stay clamped */
    svm_calc(1.5f * vmax, 0.0f, VDC_NOM, d);
    CHECK(d[0] <= INV_DUTY_MAX && d[1] >= INV_DUTY_MIN && d[2] >= INV_DUTY_MIN);
    vt_note("line-voltage error at 95%% of Vdc/sqrt3: max %.4f V", (double)max_err);
}

/* SVC-CTL-003: PI controller anti-windup */
void svc_ctl_003(void)
{
    pi_t p;
    float out;
    int k;
    pi_init(&p, 1.0f, 100.0f, 0.001f, -5.0f, 5.0f);
    for (k = 0; k < 10000; k++) { (void)pi_step(&p, 10.0f, 0.0f); }
    CHECK(p.integ <= 5.0f);
    out = pi_step(&p, -10.0f, 0.0f);
    CHECK(out < 5.0f);
    CHECK(out >= -5.0f);
    vt_note("integrator bounded at %.2f, output after error reversal %.2f", (double)p.integ, (double)out);
}

/* SVC-CTL-004: MTPA */
void svc_ctl_004(void)
{
    foc_params_t prm = sim_foc_params();
    static const float tq[] = { 1.0f, 50.0f, 150.0f, 230.0f, 300.0f, -100.0f };
    unsigned n;

    for (n = 0; n < sizeof(tq) / sizeof(tq[0]); n++) {
        float id, iq, t, imag;
        int off;
        foc_mtpa(&prm, tq[n], &id, &iq);
        t = 1.5f * MOTOR_POLE_PAIRS * (MOTOR_PSI_WB * iq + (MOTOR_LD_H - MOTOR_LQ_H) * id * iq);
        imag = sqrtf(id * id + iq * iq);
        CHECK(id < 0.0f);
        CHECK_NEAR(t, tq[n], 0.005 * fabs((double)tq[n]));
        /* optimality: any other id giving the same torque needs more current */
        for (off = -2; off <= 2; off += 4) {
            float id2 = id + (float)off;
            float iq2 = tq[n] / (1.5f * MOTOR_POLE_PAIRS * (MOTOR_PSI_WB + (MOTOR_LD_H - MOTOR_LQ_H) * id2));
            CHECK(sqrtf(id2 * id2 + iq2 * iq2) > imag);
        }
        if (n == 2) { vt_note("150 Nm -> id=%.1f A iq=%.1f A |i|=%.1f A", (double)id, (double)iq, (double)imag); }
    }

    /* surface magnet case (Ld = Lq): id = 0 */
    prm.ld = prm.lq;
    {
        float id, iq;
        foc_mtpa(&prm, 100.0f, &id, &iq);
        CHECK_NEAR(id, 0.0, 1e-6);
    }
}

/* SVC-CTL-005: torque production and current loop dynamics at low speed */
void svc_ctl_005(void)
{
    foc_t f;
    motor_t m;
    sim_res_t r;
    sim_cfg_t step = { 2000.0f, 2000.0f, 0, 150.0f, 0.0f, false, 500, VDC_NOM };
    sim_cfg_t full = { 3000.0f, 3000.0f, 0, MOTOR_MAX_TORQUE_NM, 0.0f, false, 500, VDC_NOM };

    sim_foc(&step, &f, &m, &r);
    CHECK_NEAR(r.t_at_10ms, 150.0, 0.02 * 150.0);          /* within 2 % after 10 ms */
    CHECK_NEAR(sim_motor_torque(&m), 150.0, 0.01 * 150.0); /* within 1 % at 50 ms */
    CHECK_NEAR(m.id, f.i_ref.d, 3.0);
    CHECK_NEAR(m.iq, f.i_ref.q, 3.0);
    CHECK(fabsf(f.id_fw) < 1.0f);
    CHECK(r.peak_current <= INV_I_MAX_A + 5.0f);
    vt_note("2000 rpm 150 Nm step: T(10ms)=%.1f T(50ms)=%.1f Nm, peak %.0f A",
            (double)r.t_at_10ms, (double)sim_motor_torque(&m), (double)r.peak_current);

    sim_foc(&full, &f, &m, &r);
    CHECK_NEAR(sim_motor_torque(&m), MOTOR_MAX_TORQUE_NM, 0.01f * MOTOR_MAX_TORQUE_NM);
    CHECK(sim_current_mag(&m) <= INV_I_MAX_A + 5.0f);
    CHECK(r.peak_current < INV_I_TRIP_A);
    vt_note("3000 rpm %.0f Nm (max): T=%.1f Nm |i|=%.0f A", (double)MOTOR_MAX_TORQUE_NM, (double)sim_motor_torque(&m), (double)sim_current_mag(&m));
}

/* SVC-CTL-006: regenerative torque at low speed */
void svc_ctl_006(void)
{
    foc_t f;
    motor_t m;
    sim_res_t r;
    sim_cfg_t cfg = { 3000.0f, 3000.0f, 0, -100.0f, 0.0f, false, 500, VDC_NOM };
    sim_foc(&cfg, &f, &m, &r);
    CHECK_NEAR(sim_motor_torque(&m), -100.0, 1.0);
    CHECK(r.peak_current < INV_I_TRIP_A);
    vt_note("3000 rpm -100 Nm: T=%.1f Nm", (double)sim_motor_torque(&m));
}

static void check_fw(const char *name, const foc_t *f, const motor_t *m, const sim_res_t *r,
                     float t_req, float t_tol)
{
    CHECK(f->id_fw < -10.0f);
    CHECK(f->mod_index <= INV_VOLTAGE_MARGIN + 0.01f);
    CHECK(sim_current_mag(m) <= INV_I_MAX_A + 5.0f);
    CHECK(r->peak_current < INV_I_TRIP_A);
    CHECK_NEAR(m->id, f->i_ref.d, 5.0);
    CHECK_NEAR(m->iq, f->i_ref.q, 5.0);
    CHECK_NEAR(sim_motor_torque(m), t_req, t_tol);
    vt_note("%s: T=%.1f Nm |i|=%.0f A peak=%.0f A m=%.2f id_fw=%.0f A", name,
            (double)sim_motor_torque(m), (double)sim_current_mag(m), (double)r->peak_current,
            (double)f->mod_index, (double)f->id_fw);
}

/* SVC-CTL-007: field weakening up to maximum speed */
void svc_ctl_007(void)
{
    foc_t f;
    motor_t m;
    sim_res_t r;
    sim_cfg_t up = { 0.0f, MOTOR_MAX_RPM, 15000, 40.0f, 0.0f, false, 20000, VDC_NOM };
    sim_foc(&up, &f, &m, &r);
    check_fw("0->12000 rpm at 40 Nm", &f, &m, &r, 40.0f, 1.0f);
}

/* SVC-CTL-008: voltage-limited regeneration at maximum speed */
void svc_ctl_008(void)
{
    foc_t f;
    motor_t m;
    sim_res_t r;
    sim_cfg_t regen = { 6000.0f, MOTOR_MAX_RPM, 5000, -60.0f, 0.0f, false, 10000, VDC_NOM };
    sim_foc(&regen, &f, &m, &r);
    check_fw("12000 rpm at -60 Nm", &f, &m, &r, -60.0f, 1.5f);
}

/* SVC-CTL-009: over-demand beyond the voltage-limited capability */
void svc_ctl_009(void)
{
    foc_t f;
    motor_t m;
    sim_res_t r;
    sim_cfg_t cfg = { 6000.0f, MOTOR_MAX_RPM, 5000, 120.0f, 0.0f, false, 10000, VDC_NOM };
    sim_foc(&cfg, &f, &m, &r);
    CHECK(sim_current_mag(&m) <= INV_I_MAX_A + 5.0f);
    CHECK(r.peak_current < INV_I_TRIP_A);
    CHECK(f.mod_index <= INV_VOLTAGE_MARGIN + 0.01f);
    CHECK(sim_motor_torque(&m) > 0.0f);
    CHECK(sim_motor_torque(&m) <= 120.0f);
    vt_note("120 Nm requested at 12000 rpm: delivered %.1f Nm, |i|=%.0f A, peak %.0f A, m=%.2f",
            (double)sim_motor_torque(&m), (double)sim_current_mag(&m), (double)r.peak_current,
            (double)f.mod_index);
}

/* SVC-CTL-010: inverter enable on a spinning machine (from ASC) */
void svc_ctl_010(void)
{
    foc_t f;
    motor_t m;
    sim_res_t r;
    sim_cfg_t fly = { 9000.0f, 9000.0f, 0, 60.0f, 0.15f, true, 5000, VDC_NOM };
    sim_cfg_t fly_step = { MOTOR_MAX_RPM, MOTOR_MAX_RPM, 0, 40.0f, 0.0f, true, 5000, VDC_NOM };
    /* 150 Nm at 6000 rpm = 94 kW, inside the 100 kW drive envelope */
    sim_cfg_t low = { 6000.0f, 6000.0f, 0, 150.0f, 0.15f, true, 5000, VDC_NOM };

    sim_foc(&fly, &f, &m, &r);
    check_fw("ASC->run 9000 rpm, 60 Nm ramped", &f, &m, &r, 60.0f, 1.0f);
    sim_foc(&fly_step, &f, &m, &r);
    check_fw("ASC->run 12000 rpm, 40 Nm step", &f, &m, &r, 40.0f, 1.0f);
    sim_foc(&low, &f, &m, &r);
    CHECK(r.peak_current < INV_I_TRIP_A);
    CHECK_NEAR(sim_motor_torque(&m), 150.0, 1.5);
    vt_note("ASC->run 6000 rpm, 150 Nm ramped: T=%.1f Nm peak=%.0f A",
            (double)sim_motor_torque(&m), (double)r.peak_current);
}

/* SVC-CTL-011: speed estimation */
void svc_ctl_011(void)
{
    pll_t p;
    float th = 0.0f;
    float w;
    int k;

    /* constant 6000 rpm electrical speed */
    w = 6000.0f * MC_RPM_TO_RADS * MOTOR_POLE_PAIRS;
    pll_init(&p, PLL_BW_HZ, TS_FAST);
    for (k = 0; k < 2000; k++) {
        th = mc_wrap_2pi(th + w * TS_FAST);
        pll_update(&p, th);
    }
    CHECK_NEAR(p.omega, w, 0.001 * (double)w);
    vt_note("6000 rpm: speed error %.3f %%", (double)(100.0f * (p.omega - w) / w));

    /* acceleration 0 -> 12000 rpm in 2 s (vehicle-like): tracking error */
    pll_init(&p, PLL_BW_HZ, TS_FAST);
    th = 0.0f;
    {
        float acc = MOTOR_MAX_RPM * MC_RPM_TO_RADS * MOTOR_POLE_PAIRS / 2.0f;
        float max_err = 0.0f;
        w = 0.0f;
        for (k = 0; k < 20000; k++) {
            w += acc * TS_FAST;
            th = mc_wrap_2pi(th + w * TS_FAST);
            pll_update(&p, th);
            if (k > 1000 && fabsf(p.omega - w) > max_err) { max_err = fabsf(p.omega - w); }
        }
        CHECK(max_err < 0.01f * w);
        vt_note("ramp 0->12000 rpm in 2 s: max speed error %.2f rad/s (%.3f %% of final)",
                (double)max_err, (double)(100.0f * max_err / w));
    }
}
