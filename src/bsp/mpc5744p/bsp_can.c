/**
 * @file bsp_can.c
 * @brief FlexCAN_0, 500 kbit/s, standard IDs, polled mailboxes.
 *
 * MB0 = RX BMS_STATUS, MB1 = RX BMS_LIMITS (exact ID match),
 * MB8/MB9 = TX.
 */
#include "bsp.h"
#include "bsp_internal.h"
#include "can_msgs.h"

#define MB_RX_STATUS    0u
#define MB_RX_LIMITS    1u
#define MB_TX_FIRST     8u
#define MB_TX_LAST      9u
#define MB_COUNT        16u

#define CODE_RX_EMPTY   0x4u
#define CODE_RX_FULL    0x2u
#define CODE_RX_OVERRUN 0x6u
#define CODE_TX_INACT   0x8u
#define CODE_TX_DATA    0xCu

static void mb_rx_setup(uint32_t mb, uint32_t id)
{
    CAN_0.MB[mb].CS.R = 0u;
    CAN_0.MB[mb].ID.R = (id & 0x7FFu) << 18;
    CAN_0.RXIMR[mb].R = 0x1FFFFFFFuL;               /* all ID bits must match */
    CAN_0.MB[mb].CS.R = (uint32_t)CODE_RX_EMPTY << 24;
}

void bsp_can_init(void)
{
    uint32_t mb;

    SIUL2.MSCR[PIN_CAN0_TX].B.SSS = PIN_CAN0_TX_SSS;
    SIUL2.MSCR[PIN_CAN0_TX].B.OBE = 1u;
    SIUL2.MSCR[PIN_CAN0_RX].B.IBE = 1u;
    SIUL2.IMCR[IMCR_CAN0_RX].B.SSS = IMCR_CAN0_RX_SSS;

    CAN_0.MCR.B.MDIS = 1u;
    CAN_0.CTRL1.B.CLKSRC = 0u;                      /* oscillator clock (40 MHz) */
    CAN_0.MCR.B.MDIS = 0u;
    while (!CAN_0.MCR.B.FRZACK) { }                 /* leaves reset in freeze mode */

    /* 40 MHz / (PRESDIV+1 = 5) = 8 MHz = 16 tq per bit:
     * SYNC 1 + PROP 7 + PSEG1 6 + PSEG2 2 -> sample point 87.5 % */
    CAN_0.CTRL1.R = (4uL << 24)    /* PRESDIV */
                  | (1uL << 22)    /* RJW = 2 tq */
                  | (5uL << 19)    /* PSEG1 = 6 tq */
                  | (1uL << 16)    /* PSEG2 = 2 tq */
                  | (6uL << 0);    /* PROPSEG = 7 tq; CLKSRC (bit 13) = 0 */

    for (mb = 0; mb < MB_COUNT; mb++) {
        CAN_0.MB[mb].CS.R = 0u;
        CAN_0.MB[mb].ID.R = 0u;
    }
    mb_rx_setup(MB_RX_STATUS, CAN_ID_BMS_STATUS);
    mb_rx_setup(MB_RX_LIMITS, CAN_ID_BMS_LIMITS);
    for (mb = MB_TX_FIRST; mb <= MB_TX_LAST; mb++) {
        CAN_0.MB[mb].CS.R = (uint32_t)CODE_TX_INACT << 24;
    }

    CAN_0.MCR.B.IRMQ = 1u;                          /* individual RX masks */
    CAN_0.MCR.B.SRXDIS = 1u;                        /* no self reception */
    CAN_0.MCR.B.MAXMB = MB_COUNT - 1u;
    CAN_0.MCR.B.HALT = 0u;
    CAN_0.MCR.B.FRZ = 0u;
    while (CAN_0.MCR.B.FRZACK) { }
    while (CAN_0.MCR.B.NOTRDY) { }
}

static bool mb_read(uint32_t mb, bsp_can_frame_t *f)
{
    uint32_t cs = CAN_0.MB[mb].CS.R;                /* locks the mailbox */
    uint32_t code = (cs >> 24) & 0xFu;
    unsigned i;

    if (code != CODE_RX_FULL && code != CODE_RX_OVERRUN) { return false; }

    f->dlc = (uint8_t)((cs >> 16) & 0xFu);
    if (f->dlc > 8u) { f->dlc = 8u; }
    f->id = (CAN_0.MB[mb].ID.R >> 18) & 0x7FFu;
    for (i = 0; i < 8u; i++) { f->data[i] = CAN_0.MB[mb].DATA.B[i]; }

    (void)CAN_0.TIMER.R;                            /* unlock */
    CAN_0.IFLAG1.R = 1uL << mb;
    return true;
}

bool bsp_can_receive(bsp_can_frame_t *f)
{
    if (mb_read(MB_RX_STATUS, f)) { return true; }
    return mb_read(MB_RX_LIMITS, f);
}

bool bsp_can_send(const bsp_can_frame_t *f)
{
    uint32_t mb;
    unsigned i;

    for (mb = MB_TX_FIRST; mb <= MB_TX_LAST; mb++) {
        if (((CAN_0.MB[mb].CS.R >> 24) & 0xFu) == CODE_TX_INACT) {
            CAN_0.IFLAG1.R = 1uL << mb;
            CAN_0.MB[mb].ID.R = (f->id & 0x7FFu) << 18;
            for (i = 0; i < 8u; i++) { CAN_0.MB[mb].DATA.B[i] = f->data[i]; }
            CAN_0.MB[mb].CS.R = ((uint32_t)CODE_TX_DATA << 24)
                              | ((uint32_t)(f->dlc & 0xFu) << 16);
            return true;
        }
    }
    return false;                                   /* all TX mailboxes busy */
}
