#include <math.h>
#include <string.h>
#include "harness.h"
#include "app.h"
#include "board_cfg.h"
#include "ev_config.h"

harness_t g_h;

uint16_t h_volts_to_counts(float v)
{
    float c = v / ADC_V_PER_COUNT + 0.5f;
    if (c < 0.0f) { c = 0.0f; }
    if (c > ADC_FULL_SCALE) { c = ADC_FULL_SCALE; }
    return (uint16_t)c;
}

uint16_t h_amps_to_counts(float a)
{
    float c = CUR_ZERO_COUNTS + a / CUR_A_PER_COUNT + 0.5f;
    if (c < 0.0f) { c = 0.0f; }
    if (c > ADC_FULL_SCALE) { c = ADC_FULL_SCALE; }
    return (uint16_t)c;
}

uint16_t h_temp_to_counts(float deg_c)
{
    /* inverse of the NTC beta model in a pull-up divider */
    float t = deg_c + 273.15f;
    float r = NTC_R25_OHM * expf(NTC_BETA * (1.0f / t - 1.0f / 298.15f));
    return h_volts_to_counts(ADC_VREF_V * r / (r + NTC_PULLUP_OHM));
}

void h_make_bms_frames(bsp_can_frame_t *st, bsp_can_frame_t *lim, uint8_t alive,
                       float pack_v, bool ready, bool fault)
{
    uint16_t v = (uint16_t)(pack_v * 10.0f);
    memset(st, 0, sizeof(*st));
    st->id = CAN_ID_BMS_STATUS;
    st->dlc = 8u;
    st->data[0] = (uint8_t)v;
    st->data[1] = (uint8_t)(v >> 8);
    st->data[4] = 160u;                              /* 80 % */
    st->data[5] = (uint8_t)((ready ? 1u : 0u) | (fault ? 2u : 0u));
    st->data[6] = (uint8_t)(alive & 0x0Fu);
    st->data[7] = can_xor_checksum(st->data, 7u);

    memset(lim, 0, sizeof(*lim));
    lim->id = CAN_ID_BMS_LIMITS;
    lim->dlc = 8u;
    lim->data[0] = (uint8_t)(4000u & 0xFFu);         /* 400 A discharge */
    lim->data[1] = (uint8_t)(4000u >> 8);
    lim->data[2] = (uint8_t)(2000u & 0xFFu);         /* 200 A charge */
    lim->data[3] = (uint8_t)(2000u >> 8);
    lim->data[6] = (uint8_t)(alive & 0x0Fu);
    lim->data[7] = can_xor_checksum(lim->data, 7u);
}

void h_set_pedal_volts(float v1, float v2)
{
    g_stub.adc_slow.apps1 = h_volts_to_counts(v1);
    g_stub.adc_slow.apps2 = h_volts_to_counts(v2);
}

void h_set_pedal(float frac)
{
    h_set_pedal_volts(APPS1_MIN_V + frac * (APPS1_MAX_V - APPS1_MIN_V),
                      APPS2_MIN_V + frac * (APPS2_MAX_V - APPS2_MIN_V));
}

void h_set_gear(gear_t g)
{
    g_stub.di[DI_GEAR_P] = (g == GEAR_P);
    g_stub.di[DI_GEAR_R] = (g == GEAR_R);
    g_stub.di[DI_GEAR_N] = (g == GEAR_N);
    g_stub.di[DI_GEAR_D] = (g == GEAR_D);
}

void h_reset(void)
{
    bsp_stub_reset();
    memset(&g_h, 0, sizeof(g_h));
    g_h.bms_tx = true;
    g_h.bms_ready = true;
    g_h.bms_pack_v = 350.0f;
    g_h.fast_per_ms = (unsigned)(PWM_FREQ_HZ / 1000.0f);
    g_h.slow_enabled = true;

    h_set_pedal(0.0f);
    g_stub.adc_slow.t_inv = h_temp_to_counts(25.0f);
    g_stub.adc_slow.t_mot = h_temp_to_counts(25.0f);
    h_set_gear(GEAR_P);
    app_init();
}

static void fast_once(void)
{
    g_h.angle += (double)g_h.rpm / 60.0 * 65536.0 * (double)RESOLVER_POLE_PAIRS * (double)TS_FAST;
    g_h.angle = fmod(g_h.angle, 65536.0);
    if (g_h.angle < 0.0) { g_h.angle += 65536.0; }
    g_stub.pos.angle = (uint16_t)g_h.angle;
    g_stub.adc_fast.vdc = (uint16_t)(g_h.vdc / VDC_V_PER_COUNT + 0.5f);
    app_fast_isr();
}

void h_run_fast(unsigned n)
{
    while (n-- > 0u) { fast_once(); }
}

void h_run_ms(unsigned ms)
{
    unsigned t;
    static unsigned s_ms;
    for (t = 0; t < ms; t++) {
        h_run_fast(g_h.fast_per_ms);
        if (g_h.bms_tx && (s_ms++ % 10u) == 0u) {
            bsp_can_frame_t st, lim;
            g_h.alive = (uint8_t)((g_h.alive + 1u) & 0x0Fu);
            h_make_bms_frames(&st, &lim, g_h.alive, g_h.bms_pack_v, g_h.bms_ready, g_h.bms_fault);
            bsp_stub_push_rx(&st);
            bsp_stub_push_rx(&lim);
        }
        if (g_h.slow_enabled) { app_task_1ms(); }
        app_background();
    }
}

bool h_reach_ready(void)
{
    unsigned t;
    h_reset();
    h_run_ms(150);                               /* current sensor calibration */
    g_stub.di[DI_KEY] = true;
    h_run_ms(30);                                /* precharge starts */
    g_h.vdc = 345.0f;                            /* DC link charged */
    for (t = 0; t < 500u && app_vsm_state() != VSM_READY; t += 10u) { h_run_ms(10); }
    return app_vsm_state() == VSM_READY;
}

bool h_reach_drive(void)
{
    if (!h_reach_ready()) { return false; }
    g_stub.di[DI_BRAKE] = true;
    h_set_gear(GEAR_D);
    h_run_ms(30);
    g_stub.di[DI_BRAKE] = false;
    h_run_ms(10);
    return app_vsm_state() == VSM_DRIVE && app_gear() == GEAR_D;
}
