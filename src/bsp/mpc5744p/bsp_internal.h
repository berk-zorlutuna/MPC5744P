/**
 * @file bsp_internal.h
 * @brief Private declarations shared between the MPC5744P BSP files.
 */
#ifndef BSP_INTERNAL_H
#define BSP_INTERNAL_H

#include <stdint.h>
#include "MPC5744P.h"      /* from S32 Design Studio / NXP header package */
#include "board_cfg.h"
#include "ev_config.h"

/* PWM period in FlexPWM counts (signed, center aligned) */
#define PWM_MODULO   ((int32_t)((float)MOTC_CLK_HZ / PWM_FREQ_HZ))
#define PWM_HALF     (PWM_MODULO / 2)

void bsp_clock_init(void);
void bsp_wdg_init(void);
void bsp_gpio_init(void);
void bsp_pwm_init(void);
void bsp_pwm_start(void);
void bsp_adc_init(void);
void bsp_ctu_init(void);
void bsp_rdc_init(void);
void bsp_can_init(void);
void bsp_pit_init(void);
void bsp_irq_init(void);

/* Interrupt handlers - enter these in the INTC vector table of the
 * S32DS project (IntcIsrVectorTable[] in intc_SW_mode_isr_vectors_MPC5744P.c)
 * at the vector numbers given in board_cfg.h. */
void bsp_isr_adc1_eoc(void);
void bsp_isr_pit0_ch0(void);
void bsp_isr_flexpwm0_fault(void);

#endif /* BSP_INTERNAL_H */
