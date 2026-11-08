/**
 * @file foc.h
 * @brief Field oriented control of an IPMSM: torque -> MTPA currents,
 *        field weakening, dq current regulators with decoupling, SVM.
 */
#ifndef FOC_H
#define FOC_H

#include <stdbool.h>
#include "mc_math.h"
#include "pi.h"

typedef enum {
    FOC_MODE_TORQUE = 0,   /* references derived from torque_ref (normal driving) */
    FOC_MODE_CURRENT       /* direct id/iq references (commissioning/tests) */
} foc_mode_t;

typedef struct {
    float rs, ld, lq, psi, pole_pairs;
    float ts;
    float i_max;
    float i_fw_max;
    float current_bw_hz;
} foc_params_t;

typedef struct {
    foc_params_t prm;
    pi_t pi_d, pi_q, pi_fw;

    foc_mode_t mode;
    float torque_ref;        /* [Nm] */
    mc_dq_t i_ref_direct;    /* used in FOC_MODE_CURRENT */

    /* observable state */
    mc_dq_t i_ref;
    mc_dq_t i_meas;
    mc_dq_t v_ref;           /* applied (limited) voltage */
    float v_demand;          /* unsaturated voltage magnitude */
    float id_fw;
    float mod_index;         /* |v| / (vdc/sqrt3) */
    float duty[3];
} foc_t;

void foc_init(foc_t *f, const foc_params_t *prm);
void foc_reset(foc_t *f);

/**
 * @brief Reset and pre-load field weakening for enabling the inverter on a
 * spinning machine (back-EMF may already exceed the available voltage).
 * Above the ASC speed the inverter comes from active short circuit, so
 * i_meas (dq) is the short-circuit current; transfer is bumpless.
 */
void foc_flying_start(foc_t *f, mc_dq_t i_meas, float omega_e, float vdc);

/** Max-torque-per-ampere split of a torque request into id/iq. */
void foc_mtpa(const foc_params_t *prm, float torque, float *id, float *iq);

/**
 * @brief One current loop step. Call once per PWM period.
 * @param i_abc   measured phase currents [A]
 * @param theta_e electrical angle at sampling instant [rad]
 * @param omega_e electrical speed [rad/s]
 * @param vdc     DC link voltage [V]
 * Result duties are in f->duty.
 */
void foc_step(foc_t *f, mc_abc_t i_abc, float theta_e, float omega_e, float vdc);

#endif /* FOC_H */
