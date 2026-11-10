#include "foc.h"
#include "svm.h"
#include "ev_config.h"

#define VOLTAGE_ELLIPSE_RATIO 0.97f

void foc_init(foc_t *f, const foc_params_t *prm)
{
    float wc = MC_2PI * prm->current_bw_hz;

    f->prm = *prm;
    /* Bandwidth-based tuning: the PI zero cancels the electrical pole R/L */
    pi_init(&f->pi_d, prm->ld * wc, prm->rs * wc, prm->ts, -1.0f, 1.0f);
    pi_init(&f->pi_q, prm->lq * wc, prm->rs * wc, prm->ts, -1.0f, 1.0f);
    pi_init(&f->pi_fw, FW_KP, FW_KI, prm->ts, -prm->i_fw_max, 0.0f);
    f->mode = FOC_MODE_TORQUE;
    f->torque_ref = 0.0f;
    f->i_ref_direct.d = 0.0f;
    f->i_ref_direct.q = 0.0f;
    foc_reset(f);
}

void foc_reset(foc_t *f)
{
    pi_reset(&f->pi_d);
    pi_reset(&f->pi_q);
    pi_reset(&f->pi_fw);
    f->i_ref.d = f->i_ref.q = 0.0f;
    f->i_meas.d = f->i_meas.q = 0.0f;
    f->v_ref.d = f->v_ref.q = 0.0f;
    f->v_demand = 0.0f;
    f->id_fw = 0.0f;
    f->mod_index = 0.0f;
    f->duty[0] = f->duty[1] = f->duty[2] = 0.5f;
}

void foc_flying_start(foc_t *f, mc_dq_t i_meas, float omega_e, float vdc)
{
    const foc_params_t *p = &f->prm;
    float vfw = INV_FW_VOLTAGE_RATIO * vdc * MC_INV_SQRT3 * INV_VOLTAGE_MARGIN;
    float w = fabsf(omega_e);
    float id0 = mc_clampf(i_meas.d, -p->i_fw_max, 0.0f);

    foc_reset(f);
    if (w > 1.0f && p->ld > 0.0f) {
        /* At least the no-load FW current: w * (psi + Ld*id) = vfw */
        float id_nl = (vfw / w - p->psi) / p->ld;
        if (id_nl < id0) { id0 = mc_clampf(id_nl, -p->i_fw_max, 0.0f); }
    }
    f->pi_fw.integ = id0;
    f->id_fw = id0;
    /* Bumpless transfer: the feed-forward reproduces the back-EMF and
     * cross-coupling of the present current; the integrators carry the
     * resistive drop. */
    f->pi_d.integ = p->rs * i_meas.d;
    f->pi_q.integ = p->rs * i_meas.q;
}

void foc_mtpa(const foc_params_t *prm, float torque, float *id, float *iq)
{
    const float k = 1.5f * prm->pole_pairs;
    const float dl = prm->lq - prm->ld;          /* > 0 for IPMSM */
    float iq_ = torque / (k * prm->psi);         /* surface-magnet initial guess */
    float id_ = 0.0f;
    int n;

    if (dl > 1e-7f) {
        /* Newton iteration on the torque equation along the MTPA locus:
         *   id(iq) = a - sqrt(a^2 + iq^2),  a = psi / (2 dl)
         *   g(iq)  = k * iq * (psi - dl * id(iq)) - T = 0
         * Converges to < 0.01 % in 4 iterations over the full torque range. */
        const float a = prm->psi / (2.0f * dl);
        for (n = 0; n < 4; n++) {
            float r = sqrtf(a * a + iq_ * iq_);
            float g, gp;
            id_ = a - r;
            g = k * iq_ * (prm->psi - dl * id_) - torque;
            gp = k * (prm->psi - dl * id_) + k * dl * iq_ * iq_ / r;
            iq_ -= g / gp;
        }
        id_ = a - sqrtf(a * a + iq_ * iq_);
    }
    *id = id_;
    *iq = iq_;
}

void foc_step(foc_t *f, mc_abc_t i_abc, float theta_e, float omega_e, float vdc)
{
    const foc_params_t *p = &f->prm;
    float s, c, vmax, id_ref, iq_ref, iq_lim, theta_out;
    mc_ab_t i_ab, v_ab;
    mc_dq_t ff;

    /* ---- measurement ---- */
    s = sinf(theta_e);
    c = cosf(theta_e);
    i_ab = mc_clarke(i_abc);
    f->i_meas = mc_park(i_ab, s, c);

    vmax = vdc * MC_INV_SQRT3 * INV_VOLTAGE_MARGIN;
    if (vmax < 1.0f) { vmax = 1.0f; }

    /* ---- references ---- */
    if (f->mode == FOC_MODE_CURRENT) {
        id_ref = f->i_ref_direct.d;
        iq_ref = f->i_ref_direct.q;
        f->id_fw = 0.0f;
    } else {
        float id_mtpa, iq_cmd;
        foc_mtpa(p, f->torque_ref, &id_mtpa, &iq_cmd);

        /* Field weakening: when the (unsaturated) voltage demand approaches
         * the limit, integrate negative d-axis current to reduce back-EMF. */
        f->id_fw = pi_step(&f->pi_fw, INV_FW_VOLTAGE_RATIO * vmax - f->v_demand, 0.0f);

        id_ref = id_mtpa + f->id_fw;
        iq_ref = iq_cmd;
        if (f->id_fw < 0.0f) {
            /* Extra negative id adds reluctance torque: re-solve iq so the
             * torque stays equal to the request. */
            float flux = p->psi + (p->ld - p->lq) * mc_clampf(id_ref, -p->i_max, 0.0f);
            iq_ref = f->torque_ref / (1.5f * p->pole_pairs * flux);
        }
        (void)iq_cmd;
    }

    /* current circle limit, d-axis priority */
    id_ref = mc_clampf(id_ref, -p->i_max, p->i_max);
    iq_lim = sqrtf(p->i_max * p->i_max - id_ref * id_ref);

    /* voltage ellipse limit (steady state, R neglected):
     * w^2 * ((psi + Ld*id)^2 + (Lq*iq)^2) <= v^2. Caps |iq| once field
     * weakening alone cannot keep the voltage inside the hexagon. */
    if (fabsf(omega_e) > 1.0f) {
        float psi_v = VOLTAGE_ELLIPSE_RATIO * vmax / fabsf(omega_e);
        float psi_d = p->psi + p->ld * id_ref;
        float rem = psi_v * psi_v - psi_d * psi_d;
        float iq_v = (rem > 0.0f) ? sqrtf(rem) / p->lq : 0.0f;
        if (iq_v < iq_lim) { iq_lim = iq_v; }
    }
    iq_ref = mc_clampf(iq_ref, -iq_lim, iq_lim);
    f->i_ref.d = id_ref;
    f->i_ref.q = iq_ref;

    /* ---- current regulators with decoupling feed-forward ---- */
    ff.d = -omega_e * p->lq * f->i_meas.q;
    ff.q =  omega_e * (p->ld * f->i_meas.d + p->psi);

    /* PI regulators. The voltage vector is limited as a whole (direction
     * preserved) instead of d-axis priority, which can lock up at high
     * speed where the dq axes are strongly cross-coupled. Anti-windup:
     * while the vector is saturated, both integrators are frozen. */
    {
        float err_d = id_ref - f->i_meas.d;
        float err_q = iq_ref - f->i_meas.q;
        float int_d = f->pi_d.integ + f->pi_d.ki * p->ts * err_d;
        float int_q = f->pi_q.integ + f->pi_q.ki * p->ts * err_q;
        float vd_u = f->pi_d.kp * err_d + int_d + ff.d;
        float vq_u = f->pi_q.kp * err_q + int_q + ff.q;
        float mag = sqrtf(vd_u * vd_u + vq_u * vq_u);

        f->v_demand = mag;
        if (mag > vmax) {
            float k = vmax / mag;
            f->v_ref.d = vd_u * k;
            f->v_ref.q = vq_u * k;
        } else {
            f->v_ref.d = vd_u;
            f->v_ref.q = vq_u;
            f->pi_d.integ = int_d;
            f->pi_q.integ = int_q;
        }
    }

    f->mod_index = sqrtf(f->v_ref.d * f->v_ref.d + f->v_ref.q * f->v_ref.q)
                   / (vdc * MC_INV_SQRT3 + 1e-3f);

    /* ---- modulation ----
     * Sampled at the start of period k, duty loaded at k+1, applied k+1..k+2:
     * rotate the output vector by 1.5 sample periods. */
    theta_out = theta_e + 1.5f * omega_e * p->ts;
    v_ab = mc_ipark(f->v_ref, sinf(theta_out), cosf(theta_out));
    svm_calc(v_ab.alpha, v_ab.beta, vdc, f->duty);
}
