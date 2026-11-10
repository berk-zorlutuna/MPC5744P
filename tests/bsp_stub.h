#ifndef BSP_STUB_H
#define BSP_STUB_H

#include "bsp.h"

#define STUB_RX_DEPTH 32u
#define STUB_TX_LOG   256u

typedef enum { STUB_PWM_OFF = 0, STUB_PWM_RUN, STUB_PWM_ASC } stub_pwm_t;

typedef struct {
    bsp_adc_fast_t adc_fast;
    bsp_adc_slow_t adc_slow;
    bsp_pos_t pos;
    float duty[3];
    stub_pwm_t pwm;
    bool hw_fault;
    bool di[DI_COUNT];
    bool dout[DO_COUNT];
    bsp_can_frame_t rx[STUB_RX_DEPTH];
    unsigned rx_head, rx_tail;
    bsp_can_frame_t last_tx;
    bsp_can_frame_t tx_log[STUB_TX_LOG];
    unsigned tx_count;
    unsigned wdg_kicks;
} bsp_stub_t;

extern bsp_stub_t g_stub;

void bsp_stub_reset(void);
void bsp_stub_push_rx(const bsp_can_frame_t *f);

#endif
