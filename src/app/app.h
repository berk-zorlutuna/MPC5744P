/**
 * @file app.h
 * @brief Application: task scheduling and glue between sensors, control,
 *        vehicle logic, diagnostics and the BSP.
 *
 * Execution model
 *   app_fast_isr()   PWM rate (10 kHz), highest priority: sampling, fast
 *                    protections, position, FOC, PWM update.
 *   app_task_1ms()   1 kHz timer ISR: inputs, CAN RX, diagnostics, torque
 *                    manager; every 10 ms: vehicle state machine, CAN TX.
 *   app_background() main loop: task supervision + watchdog service.
 */
#ifndef APP_H
#define APP_H

#include <stdbool.h>
#include <stdint.h>
#include "vehicle_sm.h"
#include "ev_types.h"

void app_init(void);
void app_background(void);

/* Read-only accessors (debug, tests, calibration tool) */
vsm_state_t app_vsm_state(void);
gear_t app_gear(void);
float app_torque_cmd(void);
float app_speed_rpm(void);
float app_vdc(void);
bool app_current_cal_done(void);

#endif /* APP_H */
