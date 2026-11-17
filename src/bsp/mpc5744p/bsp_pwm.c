/**
 * @file bsp_pwm.c
 * @brief FlexPWM_0: 3-phase complementary, center aligned, hardware fault input.
 *
 * Submodule 0/1/2 = phase A/B/C. PWMA = high side, PWMB = low side
 * (complementary with dead time). SM1/SM2 are synchronised to SM0.
 * Signed center-aligned counting: INIT = -HALF, VAL1 = HALF-1,
 * PWMA is high between VAL2 = -d*HALF and VAL3 = +d*HALF.
 * The counter starts (reload) in the middle of the low-side-on zero vector,
 * which is where the CTU triggers the current sampling.
 */
#include "bsp.h"
#include "bsp_internal.h"

#define DEADTIME_CNT    ((uint16_t)((uint64_t)INV_DEADTIME_NS * MOTC_CLK_HZ / 1000000000uLL))
#define SM_MASK         0x7u                    /* submodules 0..2 */
#define OUT_AB_MASK     ((SM_MASK << 8) | (SM_MASK << 4))   /* A[11:8], B[7:4] */
#define DTSRC_SWCOUT_23 ((2u << 2) | (2u << 6) | (2u << 10)) /* SM0..2 SEL23 = SWCOUT */

static void pin_out(uint32_t mscr)
{
    SIUL2.MSCR[mscr].B.SSS = PIN_PWM_SSS;
    SIUL2.MSCR[mscr].B.OBE = 1u;
    SIUL2.MSCR[mscr].B.SRC = 3u;                /* full drive strength (VERIFY field) */
}

static void force_update(void)
{
    /* FORCE on SM0 propagates to SM1/SM2 (FORCE_SEL = master force) and
     * loads MASK / DTSRCSEL / SWCOUT immediately. */
    FlexPWM_0.SUB[0].CTRL2.B.FORCE = 1u;
}

void bsp_pwm_init(void)
{
    unsigned sm;

    /* Fault input from gate drivers (desaturation / overcurrent), active low */
    SIUL2.MSCR[PIN_PWM_FAULT0].B.IBE = 1u;
    SIUL2.IMCR[IMCR_PWM_FAULT0].B.SSS = IMCR_PWM_FAULT0_SSS;

    for (sm = 0; sm < 3u; sm++) {
        FlexPWM_0.SUB[sm].CTRL1.R = 0u;
        FlexPWM_0.SUB[sm].CTRL1.B.FULL = 1u;     /* reload every full cycle */
        FlexPWM_0.SUB[sm].CTRL1.B.PRSC = 0u;     /* MOTC_CLK / 1 */

        FlexPWM_0.SUB[sm].CTRL2.R = 0u;
        if (sm == 0u) {
            FlexPWM_0.SUB[sm].CTRL2.B.INIT_SEL = 0u;   /* local sync */
            FlexPWM_0.SUB[sm].CTRL2.B.CLK_SEL = 0u;    /* IPBus clock */
            FlexPWM_0.SUB[sm].CTRL2.B.FORCE_SEL = 0u;  /* local FORCE bit */
        } else {
            FlexPWM_0.SUB[sm].CTRL2.B.INIT_SEL = 2u;   /* master sync from SM0 */
            FlexPWM_0.SUB[sm].CTRL2.B.CLK_SEL = 2u;    /* AUX_CLK from SM0 */
            FlexPWM_0.SUB[sm].CTRL2.B.RELOAD_SEL = 1u; /* master reload */
            FlexPWM_0.SUB[sm].CTRL2.B.FORCE_SEL = 1u;  /* master force */
        }
        FlexPWM_0.SUB[sm].CTRL2.B.INDEP = 0u;    /* complementary pair */
        FlexPWM_0.SUB[sm].CTRL2.B.DBGEN = 1u;    /* keep running in debug (gate drivers
                                                    are masked by FAULT handling) */

        FlexPWM_0.SUB[sm].INIT.R = (uint16_t)(int16_t)(-PWM_HALF);
        FlexPWM_0.SUB[sm].VAL0.R = 0u;
        FlexPWM_0.SUB[sm].VAL1.R = (uint16_t)(int16_t)(PWM_HALF - 1);
        FlexPWM_0.SUB[sm].VAL2.R = (uint16_t)(int16_t)(-PWM_HALF / 2);
        FlexPWM_0.SUB[sm].VAL3.R = (uint16_t)(int16_t)(PWM_HALF / 2);
        FlexPWM_0.SUB[sm].VAL4.R = 0u;
        FlexPWM_0.SUB[sm].VAL5.R = 0u;

        FlexPWM_0.SUB[sm].DTCNT0.R = DEADTIME_CNT;
        FlexPWM_0.SUB[sm].DTCNT1.R = DEADTIME_CNT;

        FlexPWM_0.SUB[sm].OCTRL.R = 0u;          /* active high, fault state = 0 */
        FlexPWM_0.SUB[sm].DISMAP.R = 0x0011u;    /* FAULT0 disables PWMA and PWMB */
    }

    /* Fault 0: active low, manual clear, safe mode, interrupt enabled */
    FlexPWM_0.FFILT.R = (2u << 8) | 10u;         /* glitch filter: 3 samples, period 10 */
    FlexPWM_0.FCTRL.R = 0x0011u;                 /* FSAFE0 | FIE0, FLVL0 = 0 (active low) */
    FlexPWM_0.FSTS.R = 0x000Fu;                  /* clear fault flags */

    FlexPWM_0.DTSRCSEL.R = 0u;                   /* generated PWM */
    FlexPWM_0.SWCOUT.R = 0u;
    FlexPWM_0.MASK.R = OUT_AB_MASK;              /* outputs forced low */
    FlexPWM_0.OUTEN.R = OUT_AB_MASK;

    pin_out(PIN_PWM_A_HI); pin_out(PIN_PWM_A_LO);
    pin_out(PIN_PWM_B_HI); pin_out(PIN_PWM_B_LO);
    pin_out(PIN_PWM_C_HI); pin_out(PIN_PWM_C_LO);

    FlexPWM_0.MCTRL.B.LDOK = SM_MASK;
}

void bsp_pwm_start(void)
{
    FlexPWM_0.MCTRL.B.LDOK = SM_MASK;
    FlexPWM_0.MCTRL.B.RUN = SM_MASK;
    force_update();
}

void bsp_pwm_set_duty(const float duty[3])
{
    unsigned sm;
    for (sm = 0; sm < 3u; sm++) {
        float d = duty[sm];
        int32_t on;
        if (d < 0.0f) { d = 0.0f; }
        if (d > 1.0f) { d = 1.0f; }
        on = (int32_t)(d * (float)PWM_HALF);
        FlexPWM_0.SUB[sm].VAL2.R = (uint16_t)(int16_t)(-on);
        FlexPWM_0.SUB[sm].VAL3.R = (uint16_t)(int16_t)(on);
    }
    FlexPWM_0.MCTRL.B.LDOK = SM_MASK;            /* latched at next reload */
}

void bsp_pwm_enable(void)
{
    FlexPWM_0.DTSRCSEL.R = 0u;
    FlexPWM_0.MASK.R = 0u;
    force_update();
}

void bsp_pwm_disable(void)
{
    FlexPWM_0.MASK.R = OUT_AB_MASK;
    FlexPWM_0.DTSRCSEL.R = 0u;
    force_update();
}

void bsp_pwm_asc(void)
{
    /* Software output 0 on PWM23 for each phase: high side off, the
     * complementary low side on (with dead time). */
    FlexPWM_0.SWCOUT.R = 0u;
    FlexPWM_0.DTSRCSEL.R = DTSRC_SWCOUT_23;
    FlexPWM_0.MASK.R = 0u;
    force_update();
}

bool bsp_pwm_hw_fault(void)
{
    return (FlexPWM_0.FSTS.R & 0x0001u) != 0u;   /* FFLAG0 */
}

void bsp_pwm_clear_hw_fault(void)
{
    FlexPWM_0.FSTS.R = 0x0001u;                  /* w1c; only clears if pin inactive */
    FlexPWM_0.FCTRL.R |= 0x0001u;                /* re-arm fault interrupt */
}

void bsp_isr_flexpwm0_fault(void)
{
    /* Hardware already disabled the outputs. Keep them masked too, so they
     * stay off after the fault pin releases until the application decides. */
    FlexPWM_0.MASK.R = OUT_AB_MASK;
    force_update();
    FlexPWM_0.FCTRL.R &= (uint16_t)~0x0001u;     /* disable fault IRQ; flag stays latched */
}
