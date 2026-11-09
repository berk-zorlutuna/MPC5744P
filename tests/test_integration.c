/**
 * @file test_integration.c
 * @brief Software integration verification cases: complete application
 *        (scheduler, current loop, diagnostics, state machine, CAN) on the
 *        BSP stub with fault injection.
 */
#include <math.h>
#include <string.h>
#include "vtest.h"
#include "harness.h"
#include "app.h"
#include "diag.h"
#include "ev_config.h"
#include "board_cfg.h"

static float v1_of(float x) { return APPS1_MIN_V + x * (APPS1_MAX_V - APPS1_MIN_V); }
static float v2_of(float x) { return APPS2_MIN_V + x * (APPS2_MAX_V - APPS2_MIN_V); }

/* SVC-PED-005: pedal faults in the running application */
void svc_ped_005(void)
{
    float t;

    /* track 2 shorted to ground at full pedal -> limp on track 1 */
    CHECK(h_reach_drive());
    h_set_pedal_volts(v1_of(1.0f), 0.0f);
    h_run_ms(400);
    CHECK(diag_is_set(FLT_APPS_RANGE));
    CHECK(diag_reaction() == REACT_LIMP);
    CHECK(app_vsm_state() == VSM_DRIVE);
    t = app_torque_cmd();
    CHECK_NEAR(t, LIMP_TORQUE_SCALE * MOTOR_MAX_TORQUE_NM, 1.0);
    vt_note("track 2 short at full pedal: limp torque %.1f Nm", (double)t);

    /* plausibility fault (both in range, disagree): no drive torque */
    CHECK(h_reach_drive());
    h_set_pedal_volts(v1_of(0.8f), v2_of(0.4f));
    h_run_ms(300);
    CHECK(diag_is_set(FLT_APPS_PLAUS));
    CHECK_NEAR(app_torque_cmd(), 0.0, 0.01);
    /* regenerative braking remains available */
    g_h.rpm = 3000.0f;
    g_stub.di[DI_BRAKE] = true;
    h_run_ms(300);
    t = app_torque_cmd();
    CHECK(t < -1.0f);
    vt_note("plausibility fault: drive torque 0 Nm; brake regen at 3000 rpm still %.1f Nm", (double)t);
}

/* SVC-VSM-010: implausible gear selector */
void svc_vsm_010(void)
{
    CHECK(h_reach_ready());
    g_stub.di[DI_BRAKE] = true;
    g_stub.di[DI_GEAR_P] = false;
    g_stub.di[DI_GEAR_D] = true;
    g_stub.di[DI_GEAR_R] = true;            /* D and R at the same time */
    h_run_ms(100);
    CHECK(app_vsm_state() == VSM_READY);
    g_stub.di[DI_GEAR_D] = false;
    g_stub.di[DI_GEAR_R] = false;           /* no position */
    h_run_ms(100);
    CHECK(app_vsm_state() == VSM_READY);
    h_set_gear(GEAR_D);
    h_run_ms(30);
    CHECK(app_vsm_state() == VSM_DRIVE);
    vt_note("D+R and no-position selector states ignored; valid D engages");
}

/* SVC-DIAG-002: fault latching and clearing */
void svc_diag_002(void)
{
    unsigned t;
    CHECK(h_reach_drive());
    g_stub.adc_slow.t_inv = h_temp_to_counts(TEMP_INV_TRIP_C + 5.0f);
    h_run_ms(250);
    CHECK(diag_is_set(FLT_INV_OVERTEMP));
    CHECK(app_vsm_state() == VSM_FAULT);

    g_stub.adc_slow.t_inv = h_temp_to_counts(25.0f);    /* condition disappears */
    h_run_ms(200);
    CHECK(diag_is_set(FLT_INV_OVERTEMP));
    CHECK(app_vsm_state() == VSM_FAULT);

    g_stub.di[DI_KEY] = false;                          /* quick key off/on during shutdown */
    h_run_ms(30);
    g_stub.di[DI_KEY] = true;
    h_run_ms(30);
    CHECK(diag_is_set(FLT_INV_OVERTEMP));

    g_stub.di[DI_KEY] = false;
    g_h.vdc = 20.0f;
    for (t = 0; t < 500u && app_vsm_state() != VSM_STANDBY; t += 10u) { h_run_ms(10); }
    CHECK(app_vsm_state() == VSM_STANDBY);
    CHECK(diag_is_set(FLT_INV_OVERTEMP));
    g_stub.di[DI_KEY] = true;                           /* key-on edge in STANDBY */
    h_run_ms(5);
    CHECK(!diag_is_set(FLT_INV_OVERTEMP));
    vt_note("fault latched after condition removal and across a key cycle outside STANDBY; cleared on key-on in STANDBY");
}

/* SVC-DIAG-003: hardware overcurrent input */
void svc_diag_003(void)
{
    CHECK(h_reach_drive());
    CHECK(g_stub.pwm == STUB_PWM_RUN);
    g_stub.hw_fault = true;
    h_run_fast(1);
    CHECK(diag_is_set(FLT_HW_OVERCURRENT));
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    h_run_ms(20);
    CHECK(app_vsm_state() == VSM_FAULT);
    CHECK(g_stub.dout[DO_K_POS]);                     /* HV kept: torque-off reaction */
    CHECK_NEAR(app_torque_cmd(), 0.0, 1e-6);
    vt_note("PWM off in the same current-loop cycle; FAULT state within 20 ms; HV kept");
}

/* SVC-DIAG-004: software phase overcurrent */
void svc_diag_004(void)
{
    CHECK(h_reach_drive());
    g_stub.adc_fast.ia = h_amps_to_counts(INV_I_TRIP_A - 15.0f);
    h_run_fast(1);
    CHECK(!diag_is_set(FLT_SW_OVERCURRENT));
    g_stub.adc_fast.ia = h_amps_to_counts(0.0f);
    g_stub.adc_fast.ib = h_amps_to_counts(-(INV_I_TRIP_A + 15.0f));
    h_run_fast(1);
    CHECK(diag_is_set(FLT_SW_OVERCURRENT));
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    vt_note("%.0f A passes, -%.0f A on phase B trips in the same cycle",
            (double)(INV_I_TRIP_A - 15.0f), (double)(INV_I_TRIP_A + 15.0f));
}

/* SVC-DIAG-005: phase current sum plausibility */
void svc_diag_005(void)
{
    CHECK(h_reach_drive());
    g_stub.adc_fast.ia = h_amps_to_counts(INV_I_SUM_TRIP_A + 10.0f);
    h_run_fast(INV_I_SUM_TRIP_SAMPLES - 1u);
    CHECK(!diag_is_set(FLT_CURRENT_SUM));
    h_run_fast(1);
    CHECK(diag_is_set(FLT_CURRENT_SUM));
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    vt_note("sum error %.0f A detected after %u consecutive samples",
            (double)(INV_I_SUM_TRIP_A + 10.0f), (unsigned)INV_I_SUM_TRIP_SAMPLES);
}

/* SVC-DIAG-006: DC link overvoltage */
void svc_diag_006(void)
{
    CHECK(h_reach_drive());
    g_h.vdc = HV_OV_TRIP_V - 5.0f;
    h_run_fast(100);
    CHECK(!diag_is_set(FLT_HV_OVERVOLT));
    g_h.vdc = HV_OV_TRIP_V + 5.0f;
    h_run_fast(1);
    CHECK(diag_is_set(FLT_HV_OVERVOLT));
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    vt_note("%.0f V tolerated, %.0f V trips in the same cycle",
            (double)(HV_OV_TRIP_V - 5.0f), (double)(HV_OV_TRIP_V + 5.0f));
}

/* SVC-DIAG-007: DC link undervoltage */
void svc_diag_007(void)
{
    h_reset();
    h_run_ms(200);                                    /* STANDBY with 0 V: no fault */
    CHECK(!diag_is_set(FLT_HV_UNDERVOLT));

    CHECK(h_reach_drive());
    g_h.vdc = HV_UV_TRIP_V - 10.0f;
    h_run_ms(49);
    CHECK(!diag_is_set(FLT_HV_UNDERVOLT));
    h_run_ms(1);
    CHECK(diag_is_set(FLT_HV_UNDERVOLT));
    h_run_ms(10);
    CHECK(app_vsm_state() == VSM_FAULT);
    vt_note("undervoltage only monitored with HV connected; detected after 50 ms");
}

/* SVC-DIAG-008: over-temperature detection and temperature measurement */
void svc_diag_008(void)
{
    unsigned n;
    bool found = false;

    h_reset();
    g_stub.adc_slow.t_inv = h_temp_to_counts(TEMP_INV_TRIP_C + 5.0f);
    h_run_ms(TEMP_TRIP_DEBOUNCE_MS - 1u);
    CHECK(!diag_is_set(FLT_INV_OVERTEMP));
    h_run_ms(1);
    CHECK(diag_is_set(FLT_INV_OVERTEMP));
    /* reported temperature on CAN (VCU_DIAG byte 6 = T_inv + 40) */
    for (n = 1; n <= 4u && !found; n++) {
        const bsp_can_frame_t *f = &g_stub.tx_log[(g_stub.tx_count - n) % STUB_TX_LOG];
        if (f->id == CAN_ID_VCU_DIAG) {
            CHECK_NEAR((double)f->data[6] - 40.0, TEMP_INV_TRIP_C + 5.0f, 1.0);
            found = true;
        }
    }
    CHECK(found);

    h_reset();
    g_stub.adc_slow.t_mot = h_temp_to_counts(TEMP_MOT_TRIP_C + 5.0f);
    h_run_ms(TEMP_TRIP_DEBOUNCE_MS - 1u);
    CHECK(!diag_is_set(FLT_MOT_OVERTEMP));
    h_run_ms(1);
    CHECK(diag_is_set(FLT_MOT_OVERTEMP));

    h_reset();
    g_stub.adc_slow.t_inv = h_temp_to_counts(TEMP_INV_TRIP_C - 3.0f);
    g_stub.adc_slow.t_mot = h_temp_to_counts(TEMP_MOT_TRIP_C - 3.0f);
    h_run_ms(1000);
    CHECK(!diag_is_set(FLT_INV_OVERTEMP) && !diag_is_set(FLT_MOT_OVERTEMP));
    vt_note("trip after %u ms at trip+5 C; no trip at trip-3 C; CAN reports measured temperature",
            (unsigned)TEMP_TRIP_DEBOUNCE_MS);
}

/* SVC-DIAG-009: temperature sensor open / short circuit */
void svc_diag_009(void)
{
    h_reset();
    g_stub.adc_slow.t_inv = 4095u;                    /* open NTC (pulled up) */
    g_stub.adc_slow.t_mot = 0u;                       /* shorted NTC */
    h_run_ms(TEMP_TRIP_DEBOUNCE_MS + 5u);
    CHECK(diag_is_set(FLT_INV_OVERTEMP));
    CHECK(diag_is_set(FLT_MOT_OVERTEMP));
    vt_note("open and shorted NTC both read as over-temperature (fail-safe)");
}

/* SVC-DIAG-010: rotor position sensor fault */
void svc_diag_010(void)
{
    CHECK(h_reach_drive());
    g_stub.pos.fault = true;
    h_run_fast(POS_SENSOR_FAULT_SAMPLES - 1u);
    CHECK(!diag_is_set(FLT_POS_SENSOR));
    h_run_fast(1);
    CHECK(diag_is_set(FLT_POS_SENSOR));
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    vt_note("position fault detected after %u consecutive samples (%.1f ms)",
            (unsigned)POS_SENSOR_FAULT_SAMPLES, (double)((float)POS_SENSOR_FAULT_SAMPLES * TS_FAST * 1000.0f));
}

/* SVC-DIAG-011: overspeed and speed measurement */
void svc_diag_011(void)
{
    float est;
    h_reset();
    g_h.rpm = MOTOR_MAX_RPM + 500.0f;
    h_run_ms(300);
    est = app_speed_rpm();
    CHECK_NEAR(est, g_h.rpm, 0.01 * (double)g_h.rpm);
    CHECK(!diag_is_set(FLT_OVERSPEED));               /* below overspeed threshold */

    h_reset();
    g_h.rpm = MOTOR_OVERSPEED_RPM + 500.0f;
    h_run_ms(100);
    CHECK(diag_is_set(FLT_OVERSPEED));
    vt_note("speed estimate %.0f rpm at %.0f rpm; trip above %.0f rpm",
            (double)est, (double)(MOTOR_MAX_RPM + 500.0f), (double)MOTOR_OVERSPEED_RPM);
}

/* SVC-DIAG-012: BMS communication loss and BMS fault */
void svc_diag_012(void)
{
    CHECK(h_reach_drive());
    g_h.bms_tx = false;
    h_run_ms(BMS_TIMEOUT_MS - 20u);
    CHECK(!diag_is_set(FLT_BMS_TIMEOUT));
    h_run_ms(40);
    CHECK(diag_is_set(FLT_BMS_TIMEOUT));
    h_run_ms(20);
    CHECK(app_vsm_state() == VSM_SHUTDOWN);
    CHECK(!g_stub.dout[DO_K_POS]);

    CHECK(h_reach_drive());
    g_h.bms_fault = true;
    h_run_ms(30);
    CHECK(diag_is_set(FLT_BMS_FAULT));
    CHECK(app_vsm_state() == VSM_SHUTDOWN);
    CHECK(!g_stub.dout[DO_K_POS]);
    vt_note("BMS silent: fault within %u..%u ms, contactors opened; BMS fault flag -> HV off",
            (unsigned)BMS_TIMEOUT_MS - 20u, (unsigned)BMS_TIMEOUT_MS + 20u);
}

/* SVC-DIAG-013: precharge failure handling */
void svc_diag_013(void)
{
    unsigned t;
    h_reset();
    h_run_ms(150);
    g_stub.di[DI_KEY] = true;
    g_h.vdc = 100.0f;                                 /* DC link does not charge */
    h_run_ms(PRECHARGE_TIMEOUT_MS + 100u);
    CHECK(diag_is_set(FLT_PRECHARGE));
    CHECK(!g_stub.dout[DO_K_PRE] && !g_stub.dout[DO_K_POS]);
    g_h.vdc = 0.0f;
    h_run_ms(1000);
    CHECK(app_vsm_state() == VSM_STANDBY);
    CHECK(!g_stub.dout[DO_K_NEG]);                    /* no automatic retry */

    g_stub.di[DI_KEY] = false;                        /* key cycle -> retry */
    h_run_ms(20);
    g_stub.di[DI_KEY] = true;
    h_run_ms(30);
    CHECK(app_vsm_state() == VSM_PRECHARGE);
    g_h.vdc = 345.0f;
    for (t = 0; t < 500u && app_vsm_state() != VSM_READY; t += 10u) { h_run_ms(10); }
    CHECK(app_vsm_state() == VSM_READY);
    vt_note("precharge fault latched (no retry); retry after key cycle succeeds");
}

/* SVC-SAFE-001: freewheel / active short circuit selection */
void svc_safe_001(void)
{
    CHECK(h_reach_drive());
    g_h.rpm = ASC_ENTER_RPM + 500.0f;
    h_run_ms(100);
    diag_set(FLT_MOT_OVERTEMP);                       /* fault injection: torque-off */
    h_run_fast(1);
    CHECK(g_stub.pwm == STUB_PWM_ASC);
    g_h.rpm = ASC_EXIT_RPM + 500.0f;                  /* inside hysteresis band */
    h_run_ms(50);
    CHECK(g_stub.pwm == STUB_PWM_ASC);
    g_h.rpm = ASC_EXIT_RPM - 500.0f;
    h_run_ms(50);
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    g_h.rpm = ASC_EXIT_RPM + 500.0f;
    h_run_ms(50);
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    g_h.rpm = ASC_ENTER_RPM + 500.0f;
    h_run_ms(50);
    CHECK(g_stub.pwm == STUB_PWM_ASC);
    CHECK(app_vsm_state() == VSM_FAULT);
    vt_note("ASC above %.0f rpm, freewheel below %.0f rpm, hysteresis verified both ways",
            (double)ASC_ENTER_RPM, (double)ASC_EXIT_RPM);
}

/* SVC-SAFE-002: hardware overcurrent at high speed -> all switches off */
void svc_safe_002(void)
{
    CHECK(h_reach_drive());
    g_h.rpm = ASC_ENTER_RPM + 1000.0f;
    h_run_ms(100);
    g_stub.hw_fault = true;
    h_run_fast(1);
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    h_run_ms(50);
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    vt_note("at %.0f rpm a hardware overcurrent selects all-off, not ASC", (double)(ASC_ENTER_RPM + 1000.0f));
}

/* SVC-SAFE-003: task supervision and watchdog service */
void svc_safe_003(void)
{
    unsigned k0;

    h_reset();
    h_run_ms(200);
    CHECK(g_stub.wdg_kicks >= 18u);
    vt_note("nominal: %u services in 200 ms", g_stub.wdg_kicks);

    k0 = g_stub.wdg_kicks;
    g_h.fast_per_ms = 0u;                             /* current loop stalled */
    h_run_ms(100);
    CHECK(g_stub.wdg_kicks == k0);

    g_h.fast_per_ms = 5u;                             /* current loop at half rate */
    h_run_ms(100);
    CHECK(g_stub.wdg_kicks == k0);

    g_h.fast_per_ms = 10u;
    g_h.slow_enabled = false;                         /* 1 ms task stalled */
    h_run_ms(100);
    CHECK(g_stub.wdg_kicks == k0);

    g_h.slow_enabled = true;
    h_run_ms(100);
    CHECK(g_stub.wdg_kicks > k0);
    vt_note("no service with current loop stalled, at half rate, or 1 ms task stalled; resumes when nominal");
}

/* SVC-SAFE-004: safe outputs at start-up and in STANDBY */
void svc_safe_004(void)
{
    unsigned i;
    bool any = false;

    h_reset();
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    h_run_ms(300);
    for (i = 0; i < (unsigned)DO_COUNT; i++) { any = any || g_stub.dout[i]; }
    CHECK(!any);
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    CHECK(app_vsm_state() == VSM_STANDBY);
    vt_note("contactors, discharge, gate enable off and PWM disabled in STANDBY");
}

/* SVC-SAFE-005: current sensor offset calibration */
void svc_safe_005(void)
{
    unsigned t;

    h_reset();
    h_run_fast(1023u);
    CHECK(!app_current_cal_done());
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    h_run_fast(1u);
    CHECK(app_current_cal_done());
    CHECK(!diag_is_set(FLT_CURRENT_OFFSET));

    /* offset within tolerance: accepted */
    h_reset();
    g_stub.adc_fast.ia = h_amps_to_counts(INV_I_OFFSET_MAX_A - 5.0f);
    h_run_ms(150);
    CHECK(app_current_cal_done() && !diag_is_set(FLT_CURRENT_OFFSET));

    /* offset out of tolerance: fault, persists over key cycle, no DRIVE */
    h_reset();
    g_stub.adc_fast.ib = h_amps_to_counts(INV_I_OFFSET_MAX_A + 5.0f);
    h_run_ms(150);
    CHECK(diag_is_set(FLT_CURRENT_OFFSET));
    g_stub.di[DI_KEY] = true;                         /* key-on edge clears latched faults */
    h_run_ms(5);
    CHECK(diag_is_set(FLT_CURRENT_OFFSET));           /* ... but not this one */
    g_h.vdc = 345.0f;
    g_stub.di[DI_BRAKE] = true;
    h_set_gear(GEAR_D);
    for (t = 0; t < 600u; t += 10u) { h_run_ms(10); }
    CHECK(app_vsm_state() != VSM_DRIVE);
    CHECK(g_stub.pwm == STUB_PWM_OFF);
    vt_note("calibration completes after 1024 samples with PWM off; %.0f A offset accepted, %.0f A rejected and persistent",
            (double)(INV_I_OFFSET_MAX_A - 5.0f), (double)(INV_I_OFFSET_MAX_A + 5.0f));
}

/* SVC-COM-004: periodic VCU messages */
void svc_com_004(void)
{
    unsigned c0, i, n_status = 0u, n_diag = 0u;
    int last_alive = -1;
    bool alive_ok = true;

    h_reset();
    h_run_ms(55);
    c0 = g_stub.tx_count;
    h_run_ms(100);
    for (i = c0; i < g_stub.tx_count; i++) {
        const bsp_can_frame_t *f = &g_stub.tx_log[i % STUB_TX_LOG];
        CHECK(f->dlc == 8u);
        if (f->id == CAN_ID_VCU_STATUS) {
            n_status++;
            CHECK(f->data[0] == (uint8_t)VSM_STANDBY);
            CHECK(f->data[1] == (uint8_t)GEAR_P);
            if (last_alive >= 0 && f->data[7] != (uint8_t)((last_alive + 1) & 0x0F)) { alive_ok = false; }
            last_alive = f->data[7];
        } else if (f->id == CAN_ID_VCU_DIAG) {
            n_diag++;
        }
    }
    CHECK(n_status == 10u);
    CHECK(n_diag == 10u);
    CHECK(alive_ok);
    vt_note("100 ms: %u VCU_STATUS + %u VCU_DIAG frames, alive counter increments by 1",
            n_status, n_diag);
}
