/**
 * @file bsp_stub.c
 * @brief Host-side fake of the BSP for unit/integration tests.
 */
#include <string.h>
#include "bsp_stub.h"

bsp_stub_t g_stub;

void bsp_stub_reset(void)
{
    memset(&g_stub, 0, sizeof(g_stub));
    g_stub.adc_fast.ia = g_stub.adc_fast.ib = g_stub.adc_fast.ic = 2048u;
    g_stub.duty[0] = g_stub.duty[1] = g_stub.duty[2] = 0.5f;
}

void bsp_adc_read_fast(bsp_adc_fast_t *out) { *out = g_stub.adc_fast; }
void bsp_adc_read_slow(bsp_adc_slow_t *out) { *out = g_stub.adc_slow; }
void bsp_pos_read(bsp_pos_t *out)           { *out = g_stub.pos; }

void bsp_pwm_set_duty(const float duty[3])
{
    g_stub.duty[0] = duty[0];
    g_stub.duty[1] = duty[1];
    g_stub.duty[2] = duty[2];
}
void bsp_pwm_enable(void)  { g_stub.pwm = STUB_PWM_RUN; }
void bsp_pwm_disable(void) { g_stub.pwm = STUB_PWM_OFF; }
void bsp_pwm_asc(void)     { g_stub.pwm = STUB_PWM_ASC; }
bool bsp_pwm_hw_fault(void) { return g_stub.hw_fault; }
void bsp_pwm_clear_hw_fault(void) { g_stub.hw_fault = false; }

bool bsp_di_read(bsp_di_t ch) { return g_stub.di[ch]; }
void bsp_do_write(bsp_do_t ch, bool on) { g_stub.dout[ch] = on; }

bool bsp_can_receive(bsp_can_frame_t *f)
{
    if (g_stub.rx_head == g_stub.rx_tail) { return false; }
    *f = g_stub.rx[g_stub.rx_tail % STUB_RX_DEPTH];
    g_stub.rx_tail++;
    return true;
}

bool bsp_can_send(const bsp_can_frame_t *f)
{
    g_stub.last_tx = *f;
    g_stub.tx_count++;
    return true;
}

void bsp_stub_push_rx(const bsp_can_frame_t *f)
{
    g_stub.rx[g_stub.rx_head % STUB_RX_DEPTH] = *f;
    g_stub.rx_head++;
}

void bsp_init(void) { }
void bsp_start(void) { }
void bsp_wdg_kick(void) { g_stub.wdg_kicks++; }
