/**
 * @file can_msgs.h
 * @brief Vehicle CAN message layout (example DBC, little-endian signals).
 *
 * RX 0x101 BMS_STATUS  : [0..1] pack V (0.1 V), [2..3] pack I (0.1 A, signed),
 *                        [4] SOC (0.5 %), [5] flags (b0 ready, b1 fault),
 *                        [6] alive counter (0..15), [7] XOR checksum of [0..6]
 * RX 0x102 BMS_LIMITS  : [0..1] max discharge I (0.1 A), [2..3] max charge I (0.1 A),
 *                        [6] alive counter, [7] XOR checksum
 * TX 0x301 VCU_STATUS  : [0] state, [1] gear, [2..3] motor rpm (signed),
 *                        [4..5] torque cmd (0.1 Nm, signed), [6] reaction, [7] alive
 * TX 0x302 VCU_DIAG    : [0..3] fault mask, [4..5] Vdc (0.1 V), [6] T_inv+40, [7] T_mot+40
 */
#ifndef CAN_MSGS_H
#define CAN_MSGS_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp.h"

#define CAN_ID_BMS_STATUS   0x101u
#define CAN_ID_BMS_LIMITS   0x102u
#define CAN_ID_VCU_STATUS   0x301u
#define CAN_ID_VCU_DIAG     0x302u

typedef struct {
    float pack_v;
    float pack_i;
    float soc;
    bool ready;
    bool fault;
    float dis_i_max;
    float chg_i_max;
    uint32_t status_age_ms;   /* time since last valid BMS_STATUS */
    uint32_t limits_age_ms;
    uint8_t status_alive;
    uint8_t limits_alive;
    bool status_seen;
    bool limits_seen;
} bms_info_t;

typedef struct {
    uint8_t state;
    uint8_t gear;
    float speed_rpm;
    float torque_nm;
    uint8_t reaction;
    uint32_t fault_mask;
    float vdc;
    float t_inv;
    float t_mot;
} vcu_info_t;

void bms_info_init(bms_info_t *b);
/** @return true if the frame was a valid BMS frame and was consumed */
bool can_decode_bms(bms_info_t *b, const bsp_can_frame_t *f);
void bms_info_tick(bms_info_t *b, uint32_t dt_ms);
bool bms_info_timeout(const bms_info_t *b);

void can_encode_vcu_status(const vcu_info_t *v, uint8_t alive, bsp_can_frame_t *f);
void can_encode_vcu_diag(const vcu_info_t *v, bsp_can_frame_t *f);

uint8_t can_xor_checksum(const uint8_t *d, unsigned n);

#endif /* CAN_MSGS_H */
