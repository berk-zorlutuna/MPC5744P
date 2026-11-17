/**
 * @file bsp.h
 * @brief Hardware abstraction used by the application.
 *
 * The application and control code only talk to hardware through this
 * interface. The MPC5744P implementation is in bsp/mpc5744p/; the host
 * unit tests use a stub (tests/bsp_stub.c).
 */
#ifndef BSP_H
#define BSP_H

#include <stdbool.h>
#include <stdint.h>

/* ---------------- Interrupt locking ---------------- */
#if defined(__PPC__) || defined(__ppc__) || defined(__powerpc__)
static inline uint32_t bsp_irq_save(void)
{
    uint32_t msr;
    __asm__ volatile("mfmsr %0\n\twrteei 0" : "=r"(msr) : : "memory");
    return msr;
}
static inline void bsp_irq_restore(uint32_t msr)
{
    __asm__ volatile("wrtee %0" : : "r"(msr) : "memory");
}
#else
static inline uint32_t bsp_irq_save(void) { return 0u; }
static inline void bsp_irq_restore(uint32_t msr) { (void)msr; }
#endif

/* ---------------- Analog ---------------- */
typedef struct {
    uint16_t ia, ib, ic;   /* phase current sensors */
    uint16_t vdc;          /* DC link voltage */
} bsp_adc_fast_t;

typedef struct {
    uint16_t apps1, apps2; /* accelerator pedal tracks */
    uint16_t t_inv, t_mot; /* NTCs */
} bsp_adc_slow_t;

void bsp_adc_read_fast(bsp_adc_fast_t *out);  /* called in current loop ISR */
void bsp_adc_read_slow(bsp_adc_slow_t *out);

/* ---------------- Rotor position ---------------- */
typedef struct {
    uint16_t angle;        /* 0..65535 = 0..2*pi resolver angle */
    bool fault;            /* RDC loss of signal / tracking fault */
} bsp_pos_t;

void bsp_pos_read(bsp_pos_t *out);

/* ---------------- PWM ---------------- */
void bsp_pwm_set_duty(const float duty[3]);   /* 0..1, takes effect next period */
void bsp_pwm_enable(void);                    /* outputs active */
void bsp_pwm_disable(void);                   /* all six switches off */
void bsp_pwm_asc(void);                       /* active short circuit: low sides on */
bool bsp_pwm_hw_fault(void);                  /* hardware fault input latched */
void bsp_pwm_clear_hw_fault(void);

/* ---------------- Digital I/O ---------------- */
typedef enum {
    DI_KEY = 0, DI_BRAKE, DI_GEAR_P, DI_GEAR_R, DI_GEAR_N, DI_GEAR_D, DI_COUNT
} bsp_di_t;

typedef enum {
    DO_K_NEG = 0, DO_K_PRE, DO_K_POS, DO_DISCHARGE, DO_GATE_EN, DO_COUNT
} bsp_do_t;

bool bsp_di_read(bsp_di_t ch);
void bsp_do_write(bsp_do_t ch, bool on);

/* ---------------- CAN ---------------- */
typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} bsp_can_frame_t;

bool bsp_can_receive(bsp_can_frame_t *f);      /* non-blocking */
bool bsp_can_send(const bsp_can_frame_t *f);   /* non-blocking */

/* ---------------- System ---------------- */
void bsp_init(void);
void bsp_start(void);      /* start PWM timebase, interrupts */
void bsp_wdg_kick(void);

/* Callbacks implemented by the application, called from BSP ISRs */
void app_fast_isr(void);   /* every PWM period, after current sampling */
void app_task_1ms(void);   /* 1 ms periodic timer */

#endif /* BSP_H */
