/**
 * @file main.c
 * @brief Entry point: EV traction inverter + vehicle control on MPC5744P.
 *
 * Startup code, linker script, the INTC vector table and MPC5744P.h come
 * from an S32 Design Studio "MPC5744P" project (see README.md).
 */
#include "bsp.h"
#include "app.h"

int main(void)
{
    bsp_init();      /* clocks, watchdog, I/O, PWM (outputs off), ADC/CTU, CAN, RDC */
    app_init();      /* control, state machines, diagnostics */
    bsp_start();     /* start PWM timebase -> CTU -> ADC -> current loop, 1 ms timer, IRQs */

    for (;;) {
        app_background();
    }
}
