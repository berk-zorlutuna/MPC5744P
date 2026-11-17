/**
 * @file diag.h
 * @brief Fault manager: latched faults, debouncing and fault reactions.
 *
 * Faults are latched until diag_clear_all() is called (on a key cycle).
 * Each fault maps to a reaction; the system acts on the most severe one.
 */
#ifndef DIAG_H
#define DIAG_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    REACT_NONE = 0,
    REACT_WARN,        /* report only */
    REACT_LIMP,        /* reduced torque */
    REACT_TORQUE_OFF,  /* inverter to safe state (freewheel or ASC), HV stays on */
    REACT_HV_OFF       /* torque off + open contactors + discharge */
} diag_reaction_t;

typedef enum {
    FLT_HW_OVERCURRENT = 0,  /* gate driver desat / comparator (FlexPWM fault input) */
    FLT_SW_OVERCURRENT,
    FLT_CURRENT_SUM,
    FLT_CURRENT_OFFSET,
    FLT_HV_OVERVOLT,
    FLT_HV_UNDERVOLT,
    FLT_INV_OVERTEMP,
    FLT_MOT_OVERTEMP,
    FLT_POS_SENSOR,
    FLT_OVERSPEED,
    FLT_APPS_RANGE,
    FLT_APPS_PLAUS,
    FLT_BMS_TIMEOUT,
    FLT_BMS_FAULT,
    FLT_PRECHARGE,
    FLT_DISCHARGE,
    FLT_COUNT
} diag_fault_t;

void diag_init(void);
void diag_set(diag_fault_t f);                 /* ISR safe */
bool diag_is_set(diag_fault_t f);
uint32_t diag_mask(void);
diag_reaction_t diag_reaction(void);
diag_reaction_t diag_fault_reaction(diag_fault_t f);

/**
 * @brief Debounced fault detection. Call periodically with a condition;
 * the fault is latched after the condition was true for set_ms.
 */
void diag_debounce(diag_fault_t f, bool cond, uint16_t set_ms, uint16_t dt_ms);

void diag_clear_all(void);

#endif /* DIAG_H */
