#include "can_msgs.h"
#include "ev_config.h"

static uint16_t rd_u16(const uint8_t *d) { return (uint16_t)(d[0] | ((uint16_t)d[1] << 8)); }
static int16_t  rd_s16(const uint8_t *d) { return (int16_t)rd_u16(d); }
static void wr_u16(uint8_t *d, uint16_t v) { d[0] = (uint8_t)v; d[1] = (uint8_t)(v >> 8); }
static void wr_s16(uint8_t *d, int16_t v)  { wr_u16(d, (uint16_t)v); }

static int16_t sat_s16(float x)
{
    if (x > 32767.0f)  { return 32767; }
    if (x < -32768.0f) { return -32768; }
    return (int16_t)x;
}

static uint8_t sat_u8(float x)
{
    if (x > 255.0f) { return 255u; }
    if (x < 0.0f)   { return 0u; }
    return (uint8_t)x;
}

uint8_t can_xor_checksum(const uint8_t *d, unsigned n)
{
    uint8_t x = 0u;
    unsigned i;
    for (i = 0; i < n; i++) { x ^= d[i]; }
    return x;
}

void bms_info_init(bms_info_t *b)
{
    b->pack_v = 0.0f;
    b->pack_i = 0.0f;
    b->soc = 0.0f;
    b->ready = false;
    b->fault = false;
    b->dis_i_max = 0.0f;
    b->chg_i_max = 0.0f;
    b->status_age_ms = BMS_TIMEOUT_MS;
    b->limits_age_ms = BMS_TIMEOUT_MS;
    b->status_alive = 0u;
    b->limits_alive = 0u;
    b->status_seen = false;
    b->limits_seen = false;
}

/* A frame is accepted when the checksum matches and the alive counter moved
 * (a stuck sender is treated like a missing one). */
static bool alive_ok(bool *seen, uint8_t *last, uint8_t now)
{
    bool ok = !(*seen) || (now != *last);
    *seen = true;
    *last = now;
    return ok;
}

bool can_decode_bms(bms_info_t *b, const bsp_can_frame_t *f)
{
    const uint8_t *d = f->data;

    if (f->dlc != 8u) { return false; }
    if (f->id != CAN_ID_BMS_STATUS && f->id != CAN_ID_BMS_LIMITS) { return false; }
    if (can_xor_checksum(d, 7u) != d[7]) { return false; }

    if (f->id == CAN_ID_BMS_STATUS) {
        if (!alive_ok(&b->status_seen, &b->status_alive, (uint8_t)(d[6] & 0x0Fu))) { return false; }
        b->pack_v = (float)rd_u16(&d[0]) * 0.1f;
        b->pack_i = (float)rd_s16(&d[2]) * 0.1f;
        b->soc    = (float)d[4] * 0.5f;
        b->ready  = (d[5] & 0x01u) != 0u;
        b->fault  = (d[5] & 0x02u) != 0u;
        b->status_age_ms = 0u;
    } else {
        if (!alive_ok(&b->limits_seen, &b->limits_alive, (uint8_t)(d[6] & 0x0Fu))) { return false; }
        b->dis_i_max = (float)rd_u16(&d[0]) * 0.1f;
        b->chg_i_max = (float)rd_u16(&d[2]) * 0.1f;
        b->limits_age_ms = 0u;
    }
    return true;
}

void bms_info_tick(bms_info_t *b, uint32_t dt_ms)
{
    if (b->status_age_ms < 0xFFFF0000u) { b->status_age_ms += dt_ms; }
    if (b->limits_age_ms < 0xFFFF0000u) { b->limits_age_ms += dt_ms; }
}

bool bms_info_timeout(const bms_info_t *b)
{
    return (b->status_age_ms >= BMS_TIMEOUT_MS) || (b->limits_age_ms >= BMS_TIMEOUT_MS);
}

void can_encode_vcu_status(const vcu_info_t *v, uint8_t alive, bsp_can_frame_t *f)
{
    f->id = CAN_ID_VCU_STATUS;
    f->dlc = 8u;
    f->data[0] = v->state;
    f->data[1] = v->gear;
    wr_s16(&f->data[2], sat_s16(v->speed_rpm));
    wr_s16(&f->data[4], sat_s16(v->torque_nm * 10.0f));
    f->data[6] = v->reaction;
    f->data[7] = alive;
}

void can_encode_vcu_diag(const vcu_info_t *v, bsp_can_frame_t *f)
{
    f->id = CAN_ID_VCU_DIAG;
    f->dlc = 8u;
    f->data[0] = (uint8_t)(v->fault_mask);
    f->data[1] = (uint8_t)(v->fault_mask >> 8);
    f->data[2] = (uint8_t)(v->fault_mask >> 16);
    f->data[3] = (uint8_t)(v->fault_mask >> 24);
    wr_u16(&f->data[4], (uint16_t)sat_s16(v->vdc * 10.0f));
    f->data[6] = sat_u8(v->t_inv + 40.0f);
    f->data[7] = sat_u8(v->t_mot + 40.0f);
}
