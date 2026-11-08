/**
 * @file bsp_adc.c
 * @brief SAR ADCs + CTU: PWM-synchronous current sampling, slow scan.
 *
 * Fast path: FlexPWM_0 master reload -> CTU trigger 0 (after settling
 * delay) -> command list with two dual (simultaneous) conversions on
 * ADC_0 / ADC_1: {Ia, Ib}, {Ic, Vdc}. ADC_1 end-of-CTU-conversion IRQ runs
 * the current loop once the Vdc result is valid.
 *
 * Slow path: ADC_2 continuous scan of pedal and temperature channels.
 */
#include "bsp.h"
#include "bsp_internal.h"

/* CTU command list entry (CLR, dual conversion format)            (VERIFY) */
#define CTU_CMD_CIR         (1u << 15)
#define CTU_CMD_LC          (1u << 14)   /* first command of the next list */
#define CTU_CMD_DUAL        (1u << 13)
#define CTU_CMD_FIFO(n)     (((uint32_t)(n) & 7u) << 10)
#define CTU_CMD_CH_B(c)     (((uint32_t)(c) & 0x1Fu) << 5)
#define CTU_CMD_CH_A(c)     ((uint32_t)(c) & 0x1Fu)

#define CTU_DELAY_CNT       ((uint16_t)((uint64_t)CTU_SAMPLE_DELAY_NS * MOTC_CLK_HZ / 1000000000uLL))

#define ADC_ISR_EOCTU       (1uL << 4)   /* ADC ISR.EOCTU flag (w1c)      (VERIFY) */

static void adc_calibrate(volatile struct ADC_tag *adc)
{
    uint32_t guard = 1000000u;
    adc->MCR.B.PWDN = 1u;
    adc->MCR.B.ADCLKSEL = 0u;                   /* ADC_CLK/2 for calibration */
    adc->MCR.B.PWDN = 0u;
    adc->CALBISTREG.B.TEST_EN = 1u;
    while (adc->CALBISTREG.B.C_T_BUSY && --guard) { }
    adc->MCR.B.PWDN = 1u;
    adc->MCR.B.ADCLKSEL = 1u;                   /* full ADC_CLK for operation */
}

void bsp_adc_init(void)
{
    adc_calibrate(&ADC_0);
    adc_calibrate(&ADC_1);
    adc_calibrate(&ADC_2);

    /* ADC_0 / ADC_1: CTU controlled, results overwrite, power up */
    ADC_0.MCR.B.OWREN = 1u;
    ADC_0.MCR.B.CTUEN = 1u;
    ADC_0.MCR.B.PWDN = 0u;

    ADC_1.MCR.B.OWREN = 1u;
    ADC_1.MCR.B.CTUEN = 1u;
    ADC_1.IMR.R = 0u;
    ADC_1.IMR.B.MSKEOCTU = 1u;                  /* IRQ at end of each CTU conversion */
    ADC_1.MCR.B.PWDN = 0u;

    /* ADC_2: continuous scan of slow channels */
    ADC_2.MCR.B.OWREN = 1u;
    ADC_2.MCR.B.MODE = 1u;                      /* scan */
    ADC_2.NCMR0.R = ADC2_CH_MASK;
    ADC_2.MCR.B.PWDN = 0u;
    ADC_2.MCR.B.NSTART = 1u;
}

void bsp_ctu_init(void)
{
    CTU_0.TGSISR.R = 0u;
    CTU_0.TGSISR.B.I0_RE = 1u;                  /* input 0: FlexPWM master reload */
    CTU_0.TGSCR.R = 0u;                         /* triggered mode, prescaler /1 */
    CTU_0.TGSCRR.R = 0u;                        /* counter reload value */
    CTU_0.TGSCCR.R = 0xFFFFu;                   /* counter compare (free) */
    CTU_0.T0CR.R = CTU_DELAY_CNT;               /* trigger 0 after settling delay */

    CTU_0.CLCR1.R = 0u;                         /* T0_INDEX = 0 */
    CTU_0.CLR[0].R = CTU_CMD_DUAL | CTU_CMD_FIFO(0) | CTU_CMD_CH_B(ADC1_CH_IB)  | CTU_CMD_CH_A(ADC0_CH_IA);
    CTU_0.CLR[1].R = CTU_CMD_DUAL | CTU_CMD_FIFO(0) | CTU_CMD_CH_B(ADC1_CH_VDC) | CTU_CMD_CH_A(ADC0_CH_IC);
    CTU_0.CLR[2].R = CTU_CMD_LC;                /* end of list */

    CTU_0.THCR1.R = 0u;
    CTU_0.THCR1.B.T0_E = 1u;                    /* trigger 0 enabled */
    CTU_0.THCR1.B.T0_ADCE = 1u;                 /* ... and starts the ADC command list */

    CTU_0.CR.B.TGSISR_RE = 1u;                  /* load double-buffered registers */
    CTU_0.CR.B.GRE = 1u;
}

void bsp_isr_adc1_eoc(void)
{
    ADC_1.ISR.R = ADC_ISR_EOCTU;
    /* Two CTU conversions per period on ADC_1 (Ib, then Vdc). Run the loop
     * when the last one (Vdc) is valid. Reading CDR clears VALID. */
    if (ADC_1.CDR[ADC1_CH_VDC].B.VALID) {
        app_fast_isr();
    }
}

void bsp_adc_read_fast(bsp_adc_fast_t *out)
{
    out->ia  = (uint16_t)ADC_0.CDR[ADC0_CH_IA].B.CDATA;
    out->ib  = (uint16_t)ADC_1.CDR[ADC1_CH_IB].B.CDATA;
    out->ic  = (uint16_t)ADC_0.CDR[ADC0_CH_IC].B.CDATA;
    out->vdc = (uint16_t)ADC_1.CDR[ADC1_CH_VDC].B.CDATA;
}

void bsp_adc_read_slow(bsp_adc_slow_t *out)
{
    out->apps1 = (uint16_t)ADC_2.CDR[ADC2_CH_APPS1].B.CDATA;
    out->apps2 = (uint16_t)ADC_2.CDR[ADC2_CH_APPS2].B.CDATA;
    out->t_inv = (uint16_t)ADC_2.CDR[ADC2_CH_T_INV].B.CDATA;
    out->t_mot = (uint16_t)ADC_2.CDR[ADC2_CH_T_MOT].B.CDATA;
}
