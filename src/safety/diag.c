#include "diag.h"
#include "bsp.h"

static volatile uint32_t s_mask;
static uint16_t s_cnt[FLT_COUNT];

static const diag_reaction_t s_reaction[FLT_COUNT] = {
    [FLT_HW_OVERCURRENT] = REACT_TORQUE_OFF,
    [FLT_SW_OVERCURRENT] = REACT_TORQUE_OFF,
    [FLT_CURRENT_SUM]    = REACT_TORQUE_OFF,
    [FLT_CURRENT_OFFSET] = REACT_TORQUE_OFF,
    [FLT_HV_OVERVOLT]    = REACT_TORQUE_OFF,
    [FLT_HV_UNDERVOLT]   = REACT_TORQUE_OFF,
    [FLT_INV_OVERTEMP]   = REACT_TORQUE_OFF,
    [FLT_MOT_OVERTEMP]   = REACT_TORQUE_OFF,
    [FLT_POS_SENSOR]     = REACT_TORQUE_OFF,
    [FLT_OVERSPEED]      = REACT_TORQUE_OFF,
    [FLT_APPS_RANGE]     = REACT_LIMP,
    [FLT_APPS_PLAUS]     = REACT_LIMP,
    [FLT_BMS_TIMEOUT]    = REACT_HV_OFF,
    [FLT_BMS_FAULT]      = REACT_HV_OFF,
    [FLT_PRECHARGE]      = REACT_HV_OFF,
    [FLT_DISCHARGE]      = REACT_WARN,
};

void diag_init(void)
{
    diag_clear_all();
}

void diag_set(diag_fault_t f)
{
    uint32_t key;
    if ((unsigned)f >= (unsigned)FLT_COUNT) { return; }
    key = bsp_irq_save();
    s_mask |= (1uL << (unsigned)f);
    bsp_irq_restore(key);
}

bool diag_is_set(diag_fault_t f)
{
    return ((s_mask >> (unsigned)f) & 1u) != 0u;
}

uint32_t diag_mask(void)
{
    return s_mask;
}

diag_reaction_t diag_fault_reaction(diag_fault_t f)
{
    return ((unsigned)f < (unsigned)FLT_COUNT) ? s_reaction[f] : REACT_HV_OFF;
}

diag_reaction_t diag_reaction(void)
{
    diag_reaction_t r = REACT_NONE;
    uint32_t m = s_mask;
    unsigned i;
    for (i = 0; i < (unsigned)FLT_COUNT; i++) {
        if (((m >> i) & 1u) && (s_reaction[i] > r)) {
            r = s_reaction[i];
        }
    }
    return r;
}

void diag_debounce(diag_fault_t f, bool cond, uint16_t set_ms, uint16_t dt_ms)
{
    if ((unsigned)f >= (unsigned)FLT_COUNT) { return; }
    if (cond) {
        if (s_cnt[f] < set_ms) { s_cnt[f] = (uint16_t)(s_cnt[f] + dt_ms); }
        if (s_cnt[f] >= set_ms) { diag_set(f); }
    } else {
        s_cnt[f] = 0u;
    }
}

void diag_clear_all(void)
{
    uint32_t key = bsp_irq_save();
    unsigned i;
    s_mask = 0u;
    for (i = 0; i < (unsigned)FLT_COUNT; i++) { s_cnt[i] = 0u; }
    bsp_irq_restore(key);
}
