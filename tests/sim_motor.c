#include <string.h>
#include "sim_motor.h"
#include "ev_config.h"

foc_params_t sim_foc_params(void)
{
    foc_params_t p;
    p.rs = MOTOR_RS_OHM;
    p.ld = MOTOR_LD_H;
    p.lq = MOTOR_LQ_H;
    p.psi = MOTOR_PSI_WB;
    p.pole_pairs = MOTOR_POLE_PAIRS;
    p.ts = TS_FAST;
    p.i_max = INV_I_MAX_A;
    p.i_fw_max = INV_I_FW_MAX_A;
    p.current_bw_hz = CURRENT_LOOP_BW_HZ;
    return p;
}

float sim_motor_torque(const motor_t *m)
{
    return 1.5f * MOTOR_POLE_PAIRS * (MOTOR_PSI_WB * m->iq + (MOTOR_LD_H - MOTOR_LQ_H) * m->id * m->iq);
}

float sim_current_mag(const motor_t *m)
{
    return sqrtf(m->id * m->id + m->iq * m->iq);
}

/* Averaged inverter (pole voltages, common mode removed) + dq machine
 * equations, constant speed over one PWM period, 20 Euler sub-steps. */
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

void sim_foc(const sim_cfg_t *cfg, foc_t *f, motor_t *m, sim_res_t *res)
{
    foc_params_t prm = sim_foc_params();
    float duty_applied[3] = { 0.5f, 0.5f, 0.5f };
    float k_rpm = MC_RPM_TO_RADS * MOTOR_POLE_PAIRS;
    float omega_e = cfg->rpm_start * k_rpm;
    int k;

    foc_init(f, &prm);
    memset(m, 0, sizeof(*m));
    res->peak_current = 0.0f;
    res->t_at_10ms = 0.0f;

    if (cfg->from_asc) {
        /* ASC steady state: 0 = R*i + j*w*(L*i + psi) */
        float a = MOTOR_RS_OHM, b = omega_e * MOTOR_LQ_H, c = omega_e * MOTOR_LD_H;
        float det = a * a + b * c;
        mc_dq_t i0;
        m->id = -(b * omega_e * MOTOR_PSI_WB) / det;
        m->iq = -(a * omega_e * MOTOR_PSI_WB) / det;
        i0.d = m->id;
        i0.q = m->iq;
        foc_flying_start(f, i0, omega_e, cfg->vdc);
    }

    for (k = 0; k < cfg->steps; k++) {
        float imag;
        if (k < cfg->ramp_steps) {
            omega_e = (cfg->rpm_start + (cfg->rpm_end - cfg->rpm_start)
                       * (float)k / (float)cfg->ramp_steps) * k_rpm;
        } else {
            omega_e = cfg->rpm_end * k_rpm;
        }
        f->torque_ref = (cfg->torque_rate > 0.0f)
            ? mc_ramp(f->torque_ref, cfg->torque, cfg->torque_rate) : cfg->torque;

        /* sample at start of period, apply previous period's duty (1 period delay) */
        foc_step(f, motor_iabc(m), m->theta, omega_e, cfg->vdc);
        motor_step(m, duty_applied, cfg->vdc, omega_e, TS_FAST);
        memcpy(duty_applied, f->duty, sizeof(duty_applied));

        imag = sim_current_mag(m);
        if (k > 0 && imag > res->peak_current) { res->peak_current = imag; }
        if (k == 99) { res->t_at_10ms = sim_motor_torque(m); }
    }
}
