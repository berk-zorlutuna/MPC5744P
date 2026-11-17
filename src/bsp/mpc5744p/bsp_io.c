/**
 * @file bsp_io.c
 * @brief SIUL2 digital I/O and the resolver-to-digital converter on DSPI.
 */
#include "bsp.h"
#include "bsp_internal.h"

static const uint16_t s_di_pin[DI_COUNT] = {
    [DI_KEY] = PIN_DI_KEY,       [DI_BRAKE] = PIN_DI_BRAKE,
    [DI_GEAR_P] = PIN_DI_GEAR_P, [DI_GEAR_R] = PIN_DI_GEAR_R,
    [DI_GEAR_N] = PIN_DI_GEAR_N, [DI_GEAR_D] = PIN_DI_GEAR_D,
};

static const uint16_t s_do_pin[DO_COUNT] = {
    [DO_K_NEG] = PIN_DO_K_NEG,   [DO_K_PRE] = PIN_DO_K_PRE,
    [DO_K_POS] = PIN_DO_K_POS,   [DO_DISCHARGE] = PIN_DO_DISCHARGE,
    [DO_GATE_EN] = PIN_DO_GATE_EN,
};

void bsp_gpio_init(void)
{
    unsigned i;
    /* Outputs: drive low (contactors open, gate drivers disabled) BEFORE
     * enabling the output buffer. */
    for (i = 0; i < (unsigned)DO_COUNT; i++) {
        SIUL2.GPDO[s_do_pin[i]].R = 0u;
        SIUL2.MSCR[s_do_pin[i]].B.SSS = 0u;    /* GPIO */
        SIUL2.MSCR[s_do_pin[i]].B.OBE = 1u;
    }
    for (i = 0; i < (unsigned)DI_COUNT; i++) {
        SIUL2.MSCR[s_di_pin[i]].B.SSS = 0u;
        SIUL2.MSCR[s_di_pin[i]].B.IBE = 1u;
    }
    /* Resolver converter strobe (output, idle high) and fault (input) */
    SIUL2.GPDO[PIN_RDC_SAMPLE].R = 1u;
    SIUL2.MSCR[PIN_RDC_SAMPLE].B.OBE = 1u;
    SIUL2.MSCR[PIN_RDC_FAULT].B.IBE = 1u;
}

bool bsp_di_read(bsp_di_t ch)
{
    if ((unsigned)ch >= (unsigned)DI_COUNT) { return false; }
    return SIUL2.GPDI[s_di_pin[ch]].R != 0u;
}

void bsp_do_write(bsp_do_t ch, bool on)
{
    if ((unsigned)ch >= (unsigned)DO_COUNT) { return; }
    SIUL2.GPDO[s_do_pin[ch]].R = on ? 1u : 0u;
}

/* ======================================================================= */
/* Resolver-to-digital converter (16-bit angle, SPI mode 1)                */
/* ======================================================================= */
#define SPI_SR_TCF      (1uL << 31)
#define SPI_SR_EOQF     (1uL << 28)
#define SPI_SR_RFDF     (1uL << 17)
#define SPI_PUSHR_EOQ   (1uL << 27)
#define SPI_PUSHR_PCS(n) (1uL << (16u + (n)))

void bsp_rdc_init(void)
{
    /* DSPI master, PCS idle high, halted while configuring */
    RDC_SPI.MCR.R = 0x80010001uL;            /* MSTR | PCSIS0 | HALT */
    /* CTAR0: 16-bit frame (FMSZ = 15), CPOL = 0, CPHA = 1,
     * baud = 50 MHz / PBR(2) / BR(4) = 6.25 MHz                       (VERIFY) */
    RDC_SPI.MODE.CTAR[0].R = (15uL << 27) | (1uL << 25) | (1uL << 0);
    RDC_SPI.MCR.B.HALT = 0u;
}

void bsp_pos_read(bsp_pos_t *out)
{
    uint32_t guard = 200u;
    volatile uint32_t dly;

    /* Latch the position in the converter */
    SIUL2.GPDO[PIN_RDC_SAMPLE].R = 0u;
    for (dly = 0u; dly < 20u; dly++) { }
    SIUL2.GPDO[PIN_RDC_SAMPLE].R = 1u;

    RDC_SPI.SR.R = SPI_SR_TCF | SPI_SR_EOQF | SPI_SR_RFDF;
    RDC_SPI.PUSHR.PUSHR.R = SPI_PUSHR_EOQ | SPI_PUSHR_PCS(RDC_SPI_CS) | 0x0000u;
    while (((RDC_SPI.SR.R & SPI_SR_RFDF) == 0u) && (--guard > 0u)) { }

    if (guard == 0u) {
        out->angle = 0u;
        out->fault = true;                       /* SPI timeout */
        return;
    }
    out->angle = (uint16_t)(RDC_SPI.POPR.R & 0xFFFFu);
    RDC_SPI.SR.R = SPI_SR_TCF | SPI_SR_EOQF | SPI_SR_RFDF;
    out->fault = (SIUL2.GPDI[PIN_RDC_FAULT].R == 0u);  /* active low */
}
