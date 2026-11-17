#include "app.h"
#include "bsp.h"
#include "board_cfg.h"
#include "ev_config.h"
#include "mc_math.h"
#include "foc.h"
#include "pll.h"
#include "pedal.h"
#include "torque_mgr.h"
#include "vehicle_sm.h"
#include "diag.h"
#include "can_msgs.h"

#define CAL_SAMPLES 1024u

typedef enum { PWM_OFF = 0, PWM_RUN, PWM_ASC } pwm_state_t;

/* ---------- owned by the fast ISR ---------- */
static foc_t s_foc;
static pll_t s_pll;
static pwm_state_t s_pwm;
static uint16_t s_sum_cnt;
static uint16_t s_pos_cnt;
static struct {
    float acc[3];
    uint32_t n;
    float off[3];
    volatile bool done;
    volatile bool bad;          /* offset out of tolerance: persists until power cycle */
} s_cal;

/* ---------- 1 ms task -> fast ISR (single-word writes, atomic) ---------- */
static volatile struct {
    float torque_ref;
    bool inv_run;
} s_cmd;

/* ---------- fast ISR -> 1 ms task ---------- */
static volatile struct {
    float speed_rpm;
    float vdc;
} s_meas;

/* ---------- owned by the 1 ms task ---------- */
static vsm_t s_vsm;
static tq_mgr_t s_tq;
static pedal_t s_pedal;
static bms_info_t s_bms;
static float s_t_inv, s_t_mot;
static bool s_key_prev;
static uint8_t s_div10;
static uint8_t s_alive;

/* ---------- supervision ---------- */
static volatile uint32_t s_fast_cnt;
static volatile uint32_t s_ms_cnt;
static volatile bool s_tick10;
static uint32_t s_fast_last, s_ms_last;

/* ======================================================================= */

void app_init(void)
{
    foc_params_t prm;

    prm.rs = MOTOR_RS_OHM;
    prm.ld = MOTOR_LD_H;
    prm.lq = MOTOR_LQ_H;
    prm.psi = MOTOR_PSI_WB;
    prm.pole_pairs = MOTOR_POLE_PAIRS;
    prm.ts = TS_FAST;
    prm.i_max = INV_I_MAX_A;
    prm.i_fw_max = INV_I_FW_MAX_A;
    prm.current_bw_hz = CURRENT_LOOP_BW_HZ;
    foc_init(&s_foc, &prm);
    pll_init(&s_pll, PLL_BW_HZ, TS_FAST);
    s_pwm = PWM_OFF;
    s_sum_cnt = 0u;
    s_pos_cnt = 0u;
    s_cal.acc[0] = s_cal.acc[1] = s_cal.acc[2] = 0.0f;
    s_cal.n = 0u;
    s_cal.done = false;
    s_cal.bad = false;

    s_cmd.torque_ref = 0.0f;
    s_cmd.inv_run = false;
    s_meas.speed_rpm = 0.0f;
    s_meas.vdc = 0.0f;

    vsm_init(&s_vsm);
    tq_mgr_init(&s_tq);
    pedal_init(&s_pedal);
    bms_info_init(&s_bms);
    diag_init();
    s_t_inv = s_t_mot = 25.0f;
    s_key_prev = false;
    s_div10 = 0u;
    s_alive = 0u;

    s_fast_cnt = s_ms_cnt = 0u;
    s_fast_last = s_ms_last = 0u;
    s_tick10 = false;

    bsp_pwm_disable();
}

/* ======================================================================= */
/* Fast ISR: current loop                                                  */
/* ======================================================================= */

static void calibrate_offsets(const bsp_adc_fast_t *raw)
{
    s_cal.acc[0] += (float)raw->ia;
    s_cal.acc[1] += (float)raw->ib;
    s_cal.acc[2] += (float)raw->ic;
    if (++s_cal.n >= CAL_SAMPLES) {
        unsigned k;
        for (k = 0; k < 3u; k++) {
            s_cal.off[k] = s_cal.acc[k] / (float)CAL_SAMPLES;
            if (fabsf(s_cal.off[k] - CUR_ZERO_COUNTS) * CUR_A_PER_COUNT > INV_I_OFFSET_MAX_A) {
                s_cal.bad = true;
                diag_set(FLT_CURRENT_OFFSET);
            }
        }
        s_cal.done = true;
    }
}

/* Safe state when torque is not allowed:
 *  - below ASC speed: all switches off (freewheel), back-EMF < Vdc
 *  - above ASC speed: active short circuit, prevents uncontrolled
 *    rectification into the battery
 *  - hardware overcurrent: always all-off (short may be in the bridge) */
static void apply_safe_state(float rpm_abs)
{
    bool hw = diag_is_set(FLT_HW_OVERCURRENT);
    pwm_state_t next;

    if (hw) {
        next = PWM_OFF;
    } else if (s_pwm == PWM_ASC) {
        next = (rpm_abs < ASC_EXIT_RPM) ? PWM_OFF : PWM_ASC;
    } else {
        next = (rpm_abs > ASC_ENTER_RPM) ? PWM_ASC : PWM_OFF;
    }

    if (next == PWM_ASC) {
        bsp_pwm_asc();
    } else {
        bsp_pwm_disable();
    }
    s_pwm = next;
    foc_reset(&s_foc);
}

void app_fast_isr(void)
{
    bsp_adc_fast_t raw;
    bsp_pos_t pos;
    mc_abc_t i;
    float vdc, theta_e, omega_e, rpm;

    bsp_adc_read_fast(&raw);
    bsp_pos_read(&pos);
    s_fast_cnt++;

    vdc = (float)raw.vdc * VDC_V_PER_COUNT;

    /* Rotor position: resolver angle -> electrical angle, PLL for speed */
    theta_e = mc_wrap_2pi((float)pos.angle * (MC_2PI / 65536.0f)
                          * (MOTOR_POLE_PAIRS / RESOLVER_POLE_PAIRS) + RESOLVER_OFFSET_RAD);
    pll_update(&s_pll, theta_e);
    omega_e = s_pll.omega;
    rpm = omega_e / (MOTOR_POLE_PAIRS * MC_RPM_TO_RADS);
    s_meas.speed_rpm = rpm;
    s_meas.vdc = vdc;

    if (bsp_pwm_hw_fault()) { diag_set(FLT_HW_OVERCURRENT); }
    if (vdc > HV_OV_TRIP_V) { diag_set(FLT_HV_OVERVOLT); }
    if (pos.fault) {
        if (++s_pos_cnt >= POS_SENSOR_FAULT_SAMPLES) { diag_set(FLT_POS_SENSOR); }
    } else {
        s_pos_cnt = 0u;
    }

    if (!s_cal.done) {
        /* Inverter is off before calibration completes */
        if (s_pwm == PWM_OFF) { calibrate_offsets(&raw); }
        apply_safe_state(fabsf(rpm));
        return;
    }

    i.a = ((float)raw.ia - s_cal.off[0]) * CUR_A_PER_COUNT;
    i.b = ((float)raw.ib - s_cal.off[1]) * CUR_A_PER_COUNT;
    i.c = ((float)raw.ic - s_cal.off[2]) * CUR_A_PER_COUNT;

    if (fabsf(i.a) > INV_I_TRIP_A || fabsf(i.b) > INV_I_TRIP_A || fabsf(i.c) > INV_I_TRIP_A) {
        diag_set(FLT_SW_OVERCURRENT);
    }
    if (fabsf(i.a + i.b + i.c) > INV_I_SUM_TRIP_A) {
        if (++s_sum_cnt >= INV_I_SUM_TRIP_SAMPLES) { diag_set(FLT_CURRENT_SUM); }
    } else {
        s_sum_cnt = 0u;
    }

    if (s_cmd.inv_run && diag_reaction() < REACT_TORQUE_OFF) {
        if (s_pwm != PWM_RUN) {
            static const float d50[3] = { 0.5f, 0.5f, 0.5f };
            mc_dq_t idq = mc_park(mc_clarke(i), sinf(theta_e), cosf(theta_e));
            foc_flying_start(&s_foc, idq, omega_e, vdc);
            bsp_pwm_set_duty(d50);
            bsp_pwm_enable();
            s_pwm = PWM_RUN;
        }
        s_foc.mode = FOC_MODE_TORQUE;
        s_foc.torque_ref = s_cmd.torque_ref;
        foc_step(&s_foc, i, theta_e, omega_e, vdc);
        bsp_pwm_set_duty(s_foc.duty);
    } else {
        apply_safe_state(fabsf(rpm));
    }
}

/* ======================================================================= */
/* 1 ms task                                                               */
/* ======================================================================= */

/* NTC in a pull-up divider. Open or shorted sensor reads as very hot so
 * the drive derates / trips (fail-safe). */
static float ntc_temp_c(uint16_t counts)
{
    float v = (float)counts * ADC_V_PER_COUNT;
    float r;
    if (v < 0.05f || v > ADC_VREF_V - 0.05f) { return 200.0f; }
    r = NTC_PULLUP_OHM * v / (ADC_VREF_V - v);
    return 1.0f / (1.0f / 298.15f + logf(r / NTC_R25_OHM) / NTC_BETA) - 273.15f;
}

static gear_t read_gear_selector(void)
{
    bool p = bsp_di_read(DI_GEAR_P);
    bool r = bsp_di_read(DI_GEAR_R);
    bool n = bsp_di_read(DI_GEAR_N);
    bool d = bsp_di_read(DI_GEAR_D);
    unsigned cnt = (unsigned)p + (unsigned)r + (unsigned)n + (unsigned)d;

    if (cnt != 1u) { return GEAR_INVALID; }   /* none or multiple: implausible */
    if (p) { return GEAR_P; }
    if (r) { return GEAR_R; }
    if (n) { return GEAR_N; }
    return GEAR_D;
}

static bool hv_connected(vsm_state_t s)
{
    return (s == VSM_PRECHARGE) || (s == VSM_READY) || (s == VSM_DRIVE) || (s == VSM_FAULT);
}

static void task_10ms(bool key, bool brake, gear_t gear_sel, float rpm, float vdc)
{
    vsm_in_t vin;
    vcu_info_t info;
    bsp_can_frame_t f;
    vsm_state_t st;

    vin.key_on = key;
    vin.brake = brake;
    vin.bms_ready = s_bms.ready && !s_bms.fault && !bms_info_timeout(&s_bms);
    vin.inverter_ready = s_cal.done && !s_cal.bad && !bsp_pwm_hw_fault();
    vin.gear_sel = gear_sel;
    vin.vbus = vdc;
    vin.vbatt = s_bms.pack_v;
    vin.speed_rpm = rpm;
    vin.reaction = diag_reaction();
    vsm_step(&s_vsm, &vin, VSM_PERIOD_MS);

    if (s_vsm.out.precharge_fail) { diag_set(FLT_PRECHARGE); }
    if (s_vsm.out.discharge_fail) { diag_set(FLT_DISCHARGE); }

    st = s_vsm.st;
    bsp_do_write(DO_K_NEG, s_vsm.out.k_neg);
    bsp_do_write(DO_K_PRE, s_vsm.out.k_pre);
    bsp_do_write(DO_K_POS, s_vsm.out.k_pos);
    bsp_do_write(DO_DISCHARGE, s_vsm.out.discharge);
    bsp_do_write(DO_GATE_EN, st != VSM_STANDBY && st != VSM_INIT);

    info.state = (uint8_t)st;
    info.gear = (uint8_t)s_vsm.out.gear;
    info.speed_rpm = rpm;
    info.torque_nm = s_tq.t_out;
    info.reaction = (uint8_t)diag_reaction();
    info.fault_mask = diag_mask();
    info.vdc = vdc;
    info.t_inv = s_t_inv;
    info.t_mot = s_t_mot;
    can_encode_vcu_status(&info, s_alive, &f);
    (void)bsp_can_send(&f);
    can_encode_vcu_diag(&info, &f);
    (void)bsp_can_send(&f);
    s_alive = (uint8_t)((s_alive + 1u) & 0x0Fu);
}

void app_task_1ms(void)
{
    bsp_adc_slow_t sl;
    bsp_can_frame_t f;
    tq_in_t tin;
    bool key, brake;
    gear_t gear_sel;
    float rpm = s_meas.speed_rpm;
    float vdc = s_meas.vdc;
    unsigned rx_budget = 8u;

    s_ms_cnt++;

    /* ---- inputs ---- */
    bsp_adc_read_slow(&sl);
    pedal_update(&s_pedal, (float)sl.apps1 * APPS_V_PER_COUNT,
                 (float)sl.apps2 * APPS_V_PER_COUNT, 1u);
    s_t_inv = ntc_temp_c(sl.t_inv);
    s_t_mot = ntc_temp_c(sl.t_mot);
    key = bsp_di_read(DI_KEY);
    brake = bsp_di_read(DI_BRAKE);
    gear_sel = read_gear_selector();

    while (rx_budget-- > 0u && bsp_can_receive(&f)) {
        (void)can_decode_bms(&s_bms, &f);
    }
    bms_info_tick(&s_bms, 1u);

    /* ---- key-on edge: new drive cycle, clear latched faults ---- */
    if (key && !s_key_prev && s_vsm.st == VSM_STANDBY) {
        diag_clear_all();
        pedal_init(&s_pedal);
        bsp_pwm_clear_hw_fault();
    }
    s_key_prev = key;

    /* ---- slow diagnostics ---- */
    diag_debounce(FLT_INV_OVERTEMP, s_t_inv > TEMP_INV_TRIP_C, TEMP_TRIP_DEBOUNCE_MS, 1u);
    diag_debounce(FLT_MOT_OVERTEMP, s_t_mot > TEMP_MOT_TRIP_C, TEMP_TRIP_DEBOUNCE_MS, 1u);
    diag_debounce(FLT_OVERSPEED, fabsf(rpm) > MOTOR_OVERSPEED_RPM, 10u, 1u);
    diag_debounce(FLT_HV_UNDERVOLT,
                  (s_vsm.st == VSM_READY || s_vsm.st == VSM_DRIVE) && vdc < HV_UV_TRIP_V, 50u, 1u);
    diag_debounce(FLT_BMS_TIMEOUT, hv_connected(s_vsm.st) && bms_info_timeout(&s_bms), 1u, 1u);
    diag_debounce(FLT_BMS_FAULT, hv_connected(s_vsm.st) && s_bms.fault, 1u, 1u);
    /* Calibration only runs at power-up: a bad offset must survive key cycles */
    if (s_cal.bad) { diag_set(FLT_CURRENT_OFFSET); }
    if (s_pedal.range_err) { diag_set(FLT_APPS_RANGE); }
    if (s_pedal.plaus_err) { diag_set(FLT_APPS_PLAUS); }

    /* ---- torque request ---- */
    tin.pedal = s_pedal.pos;
    tin.pedal_valid = s_pedal.valid;
    tin.brake = brake;
    tin.gear = s_vsm.out.gear;
    tin.drive_enabled = (s_vsm.st == VSM_DRIVE) && s_vsm.out.inv_enable;
    tin.speed_rpm = rpm;
    tin.vdc = vdc;
    tin.bms_dis_i_max = s_bms.dis_i_max;
    tin.bms_chg_i_max = s_bms.chg_i_max;
    tin.t_inv = s_t_inv;
    tin.t_mot = s_t_mot;
    tin.reaction = diag_reaction();
    s_cmd.torque_ref = tq_mgr_step(&s_tq, &tin, TS_SLOW);
    s_cmd.inv_run = s_vsm.out.inv_enable;

    /* ---- 10 ms ---- */
    if (++s_div10 >= VSM_PERIOD_MS) {
        s_div10 = 0u;
        task_10ms(key, brake, gear_sel, rpm, vdc);
        s_cmd.inv_run = s_vsm.out.inv_enable;
        s_tick10 = true;
    }
}

/* ======================================================================= */
/* Background                                                              */
/* ======================================================================= */

void app_background(void)
{
    /* Temporal supervision: the watchdog is only serviced if both periodic
     * tasks ran at the expected rate during the last 10 ms window. */
    if (s_tick10) {
        uint32_t fast = s_fast_cnt;
        uint32_t ms = s_ms_cnt;
        uint32_t dfast = fast - s_fast_last;
        uint32_t dms = ms - s_ms_last;
        const uint32_t exp_fast = (uint32_t)(PWM_FREQ_HZ * 0.01f);

        s_tick10 = false;
        s_fast_last = fast;
        s_ms_last = ms;
        if (dfast >= (exp_fast * 8u) / 10u && dfast <= (exp_fast * 12u) / 10u
            && dms >= 8u && dms <= 12u) {
            bsp_wdg_kick();
        }
    }
}

/* ======================================================================= */

vsm_state_t app_vsm_state(void) { return s_vsm.st; }
gear_t app_gear(void)           { return s_vsm.out.gear; }
float app_torque_cmd(void)      { return s_tq.t_out; }
float app_speed_rpm(void)       { return s_meas.speed_rpm; }
float app_vdc(void)             { return s_meas.vdc; }
bool app_current_cal_done(void) { return s_cal.done; }
