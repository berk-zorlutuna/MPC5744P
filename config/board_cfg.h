/**
 * @file board_cfg.h
 * @brief Board-specific hardware mapping for the MPC5744P BSP.
 *
 * Pin numbers, ADC channels, interrupt vector numbers and sensor scaling
 * depend on YOUR schematic. The values below are placeholders laid out for
 * a custom inverter board. Check every line against:
 *   - MPC5744P Reference Manual (RM), "Interrupt vector assignments"
 *   - MPC5744P IO signal description / pinout spreadsheet (MSCR/IMCR + SSS)
 *   - your inverter / VCU schematic
 * Then define BOARD_CFG_VERIFIED in the build to silence the warning.
 */
#ifndef BOARD_CFG_H
#define BOARD_CFG_H

#if !defined(BOARD_CFG_VERIFIED) && !defined(HOST_BUILD)
#warning "board_cfg.h: pin/IRQ/scaling values are placeholders - verify against your schematic and the MPC5744P RM"
#endif

/* ------------------------------------------------------------------ */
/* Clocks (see bsp/mpc5744p/clock.c)                                   */
/* ------------------------------------------------------------------ */
#define XOSC_HZ                 40000000u
#define SYS_CLK_HZ              200000000u   /* PLL1 -> system clock */
#define MOTC_CLK_HZ             160000000u   /* FlexPWM / CTU clock */
#define PBRIDGE_CLK_HZ          50000000u    /* PIT, DSPI, peripheral bus */
#define ADC_CLK_HZ              80000000u
#define CAN_CLK_HZ              XOSC_HZ      /* FlexCAN protocol clock = oscillator */

/* ------------------------------------------------------------------ */
/* Interrupt vector numbers (RM table "Interrupt vector assignments")  */
/* ------------------------------------------------------------------ */
#define IRQ_PIT0_CH0            226u
#define IRQ_ADC1_EOC            209u   /* ADC_1 end of conversion (EOCTU) */
#define IRQ_FLEXPWM0_FAULT      255u
#define IRQ_PRIO_FAST           12u    /* current loop: highest */
#define IRQ_PRIO_PWM_FAULT      15u
#define IRQ_PRIO_1MS            8u

/* ------------------------------------------------------------------ */
/* FlexPWM_0: SM0 = phase A, SM1 = phase B, SM2 = phase C              */
/* (PWMA = high side, PWMB = low side). MSCR index and SSS value.     */
/* ------------------------------------------------------------------ */
#define PIN_PWM_A_HI            0u    /* MSCR index */
#define PIN_PWM_A_LO            1u
#define PIN_PWM_B_HI            2u
#define PIN_PWM_B_LO            3u
#define PIN_PWM_C_HI            4u
#define PIN_PWM_C_LO            5u
#define PIN_PWM_SSS             1u    /* alternate function selecting FlexPWM_0 */
#define PIN_PWM_FAULT0          6u    /* gate driver desat / OC comparator, active low */
#define IMCR_PWM_FAULT0         0u
#define IMCR_PWM_FAULT0_SSS     1u

/* ------------------------------------------------------------------ */
/* ADC channels                                                        */
/* Fast (CTU-triggered, dual simultaneous conversion on ADC_0/ADC_1)   */
/* ------------------------------------------------------------------ */
#define ADC0_CH_IA              0u
#define ADC1_CH_IB              0u
#define ADC0_CH_IC              1u
#define ADC1_CH_VDC             1u
/* Slow (continuous scan on ADC_2) */
#define ADC2_CH_APPS1           0u
#define ADC2_CH_APPS2           1u
#define ADC2_CH_T_INV           2u
#define ADC2_CH_T_MOT           3u
#define ADC2_CH_MASK            0x0Fu

/* CTU trigger delay after the PWM reload (counter start = zero vector,
 * all low sides on). Gives the current sensors time to settle. */
#define CTU_SAMPLE_DELAY_NS     500u

/* ------------------------------------------------------------------ */
/* Sensor scaling                                                      */
/* ------------------------------------------------------------------ */
/* Component values below must match the hardware design
 * (tools/hw_design.py, docs/06_hardware_design.md). `make hw-report`
 * checks them against each other. */
#define ADC_VREF_V              3.3f         /* VREFH = precision analog 3.3 V */
#define ADC_FULL_SCALE          4095.0f
#define ADC_V_PER_COUNT         (ADC_VREF_V / ADC_FULL_SCALE)

/* Phase current: open-loop Hall sensor on a precision 5 V supply,
 * 2.5 V at 0 A, 2.5 mV/A, then a resistive divider to the 3.3 V ADC range */
#define CUR_SENSOR_ZERO_V       2.5f
#define CUR_SENSOR_V_PER_A      0.0025f
#define CUR_DIV_R_TOP_OHM       10000.0f
#define CUR_DIV_R_BOT_OHM       19600.0f
#define CUR_DIV_RATIO           (CUR_DIV_R_BOT_OHM / (CUR_DIV_R_TOP_OHM + CUR_DIV_R_BOT_OHM))
#define CUR_A_PER_COUNT         (ADC_V_PER_COUNT / (CUR_SENSOR_V_PER_A * CUR_DIV_RATIO))
#define CUR_ZERO_COUNTS         (CUR_SENSOR_ZERO_V * CUR_DIV_RATIO / ADC_V_PER_COUNT)

/* DC link voltage: high-voltage resistor string -> reinforced isolated
 * amplifier (gain 1) -> differential-to-single-ended op amp -> ADC */
#define VDC_DIV_R_TOP_OHM       5000000.0f   /* 5 x 1 MOhm */
#define VDC_DIV_R_BOT_OHM       20000.0f
#define VDC_ISO_GAIN            1.0f
#define VDC_OPAMP_GAIN          1.5f
#define VDC_V_PER_COUNT         (ADC_V_PER_COUNT * (VDC_DIV_R_TOP_OHM + VDC_DIV_R_BOT_OHM) \
                                 / (VDC_DIV_R_BOT_OHM * VDC_ISO_GAIN * VDC_OPAMP_GAIN))

/* Accelerator pedal tracks: 5 V sensor output -> divider -> ADC.
 * APPS_V_PER_COUNT converts counts to volts at the sensor. */
#define APPS_DIV_R_TOP_OHM      22000.0f     /* limits clamp current on a short to battery */
#define APPS_DIV_R_BOT_OHM      43000.0f
#define APPS_V_PER_COUNT        (ADC_V_PER_COUNT * (APPS_DIV_R_TOP_OHM + APPS_DIV_R_BOT_OHM) \
                                 / APPS_DIV_R_BOT_OHM)

/* NTC with pull-up to the ADC reference (ratiometric) */
#define NTC_PULLUP_OHM          4700.0f      /* resolution at 165 C; -40 C stays below open threshold */
#define NTC_R25_OHM             10000.0f
#define NTC_BETA                3435.0f

/* ------------------------------------------------------------------ */
/* Resolver-to-digital converter on DSPI (e.g. AD2S1210-class device)  */
/* ------------------------------------------------------------------ */
#define RDC_SPI                 SPI_2        /* named DSPI_2 in some header versions */
#define RDC_SPI_CS              0u           /* PCS0 */
#define PIN_RDC_SAMPLE          20u          /* GPIO: SAMPLE strobe, active low */
#define PIN_RDC_FAULT           21u          /* GPIO: DOS/LOT fault input, active low */

/* ------------------------------------------------------------------ */
/* Digital I/O (SIUL2 pin numbers = MSCR index)                        */
/* ------------------------------------------------------------------ */
#define PIN_DI_KEY              30u
#define PIN_DI_BRAKE            31u
#define PIN_DI_GEAR_P           32u
#define PIN_DI_GEAR_R           33u
#define PIN_DI_GEAR_N           34u
#define PIN_DI_GEAR_D           35u
#define PIN_DO_K_NEG            40u          /* HV- contactor */
#define PIN_DO_K_PRE            41u          /* precharge relay */
#define PIN_DO_K_POS            42u          /* HV+ contactor */
#define PIN_DO_DISCHARGE        43u          /* active discharge switch */
#define PIN_DO_GATE_EN          44u          /* gate driver enable */

/* ------------------------------------------------------------------ */
/* FlexCAN_0: 500 kbit/s, vehicle CAN                                  */
/* ------------------------------------------------------------------ */
#define PIN_CAN0_TX             16u
#define PIN_CAN0_TX_SSS         1u
#define PIN_CAN0_RX             17u
#define IMCR_CAN0_RX            0u
#define IMCR_CAN0_RX_SSS        1u

#endif /* BOARD_CFG_H */
