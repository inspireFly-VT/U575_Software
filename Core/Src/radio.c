/*
 * radio.c
 *
 * GFSK TX and RX on the LR1121 using the Semtech SWDR001 driver.
 *
 * The link settings come from Michael's working TX code (lr1121_dev_michael,
 * Oct 14). What changed from that version:
 *  - packet length is set for every packet. in variable length mode the length
 *    given to SetPacketParams is what TX sends and the most RX will accept.
 *    the old code always said 11 bytes
 *  - RX added. continuous mode, checked by polling the DIO9 pin
 *  - RX bandwidth widened from 19.5 kHz to 29.3 kHz (reason at mod_params)
 *  - image calibration for 430 to 440 MHz turned back on. it matters for RX
 *  - init checks the chip version and prints the calibration errors by name
 *  - init figures out crystal vs TCXO on its own (see radio_init)
 *
 * How the LR1121 reports events:
 * The radio keeps a 32 bit IRQ status register (TX done, RX done, CRC error...).
 * We pick which of those flags drive the DIO9 pin. DIO9 goes high while any
 * picked flag is set and drops when we clear the flags over SPI. So reading
 * DIO9 is a cheap "did anything happen" check that needs no SPI traffic.
 */

#include "radio.h"

#include <stdio.h>
#include "main.h"
#include "lr11xx_hal.h"
#include "lr11xx_system.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"

// Run a driver call. If it fails, print which call and line, then return -1.
#define CHECK(x) do { if ((x) != LR11XX_STATUS_OK) { printf("radio: FAIL %s line %d\r\n", #x, __LINE__); return -1; } } while (0)

// IRQ flags routed to DIO9. Anything else the radio flags is ignored.
#define RADIO_IRQ_MASK ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT | \
                         LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR )

// RX timeout value meaning "continuous". The radio stays in RX after each packet
// instead of dropping to standby. (0 would mean single RX, any other value is a
// timeout in 30.5 us steps.)
#define RX_CONTINUOUS 0xFFFFFF

// How the bits are put on the air
static const lr11xx_radio_mod_params_gfsk_t mod_params = {
    .br_in_bps    = RADIO_BITRATE_BPS,
    .pulse_shape  = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1,   // gaussian filter, BT = 1.0
    // RX filter width. The signal itself is about 2 * (fdev + bitrate / 2) = 14.8 kHz wide.
    // The two boards' crystals won't be on exactly the same frequency, so the filter needs
    // room on each side. 29.3 kHz allows about 7 kHz of offset (16 ppm at 435 MHz).
    // Michael's 19.5 kHz only allowed about 2 kHz, which two crystals can easily exceed.
    .bw_dsb_param = LR11XX_RADIO_GFSK_BW_29300,
    .fdev_in_hz   = RADIO_FDEV_HZ,
};

// How a packet is framed: preamble, sync word, length byte, payload, CRC.
// pld_len_in_bytes gets overwritten before every TX and every RX start.
static lr11xx_radio_pkt_params_gfsk_t pkt_params = {
    .preamble_len_in_bits  = 32,   // 1010... pattern the receiver locks onto
    .preamble_detector     = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS,
    .sync_word_len_in_bits = 32,   // marks where the packet starts. see sync[] in radio_init
    .address_filtering     = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE,
    .header_type           = LR11XX_RADIO_GFSK_PKT_VAR_LEN,   // length byte sent with each packet
    .pld_len_in_bytes      = RADIO_MAX_PAYLOAD,
    .crc_type              = LR11XX_RADIO_GFSK_CRC_2_BYTES,
    .dc_free               = LR11XX_RADIO_GFSK_DC_FREE_WHITENING,   // scrambles long runs of 0s or 1s
};

static bool irq_pin_high(const lr11xx_ctx_t* ctx)
{
    return HAL_GPIO_ReadPin(ctx->irq_port, ctx->irq_pin) == GPIO_PIN_SET;
}

// Print the radio's error register with each set bit spelled out
static void print_errors(uint16_t e)
{
    printf("radio: errors=0x%04X", e);
    if (e == 0)                                      printf(" (none)");
    if (e & LR11XX_SYSTEM_ERRORS_LF_RC_CALIB_MASK)   printf(" LF_RC_CALIB");
    if (e & LR11XX_SYSTEM_ERRORS_HF_RC_CALIB_MASK)   printf(" HF_RC_CALIB");
    if (e & LR11XX_SYSTEM_ERRORS_ADC_CALIB_MASK)     printf(" ADC_CALIB");
    if (e & LR11XX_SYSTEM_ERRORS_PLL_CALIB_MASK)     printf(" PLL_CALIB");
    if (e & LR11XX_SYSTEM_ERRORS_IMG_CALIB_MASK)     printf(" IMG_CALIB");
    if (e & LR11XX_SYSTEM_ERRORS_HF_XOSC_START_MASK) printf(" HF_XOSC_START");
    if (e & LR11XX_SYSTEM_ERRORS_LF_XOSC_START_MASK) printf(" LF_XOSC_START");
    if (e & LR11XX_SYSTEM_ERRORS_PLL_LOCK_MASK)      printf(" PLL_LOCK");
    printf("\r\n");
}

// Steps 1 to 7 of bring-up: reset, identify, power, RF switch, clock, calibration.
// Split out so radio_init can run it a second time with the other clock setting.
// errs gets the radio's error register after calibration.
static int bringup(const lr11xx_ctx_t* ctx, bool use_tcxo, uint16_t* errs)
{
    // 1. hardware reset. leaves the radio in standby with default settings
    CHECK((lr11xx_status_t)lr11xx_hal_reset(ctx));

    // 2. make sure SPI works and this really is an LR1121 before configuring anything
    lr11xx_system_version_t v;
    CHECK(lr11xx_system_get_version(ctx, &v));
    printf("radio: hw=0x%02X type=0x%02X fw=0x%04X\r\n", v.hw, v.type, v.fw);
    if (v.type != LR11XX_SYSTEM_VERSION_TYPE_LR1121) {
        printf("radio: expected type 0x03 (LR1121). check SPI wiring, NSS and BUSY pins\r\n");
        return -1;
    }

    // 3. power the radio's core from its DC-DC converter. more efficient than the LDO,
    //    and the dev board has the inductor it needs
    CHECK(lr11xx_system_set_reg_mode(ctx, LR11XX_SYSTEM_REG_MODE_DCDC));

    // 4. the board's RF switch (antenna to RX path or TX path) is driven by the
    //    radio's DIO5 to DIO7 pins (RFSW0 to RFSW2 here). this table says which pins
    //    go high in each mode. values are from Michael's working TX config.
    //    VERIFY against the dev board schematic if RX range is poor
    const lr11xx_system_rfswitch_cfg_t rf_sw = {
        .enable  = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH | LR11XX_SYSTEM_RFSW2_HIGH,
        .standby = 0,
        .rx      = LR11XX_SYSTEM_RFSW0_HIGH,
        .tx      = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH,   // low power PA (what we use)
        .tx_hp   = LR11XX_SYSTEM_RFSW1_HIGH,                              // high power PA
        .tx_hf   = 0,                                                     // 2.4 GHz PA, unused
    };
    CHECK(lr11xx_system_set_dio_as_rf_switch(ctx, &rf_sw));

    // 5. reference clock. a crystal board needs nothing here. on a TCXO board the radio
    //    has to power the TCXO from its VTCXO pin: 1.8 V, then wait 164 * 30.52 us = 5 ms
    //    for it to start
    if (use_tcxo)
        CHECK(lr11xx_system_set_tcxo_mode(ctx, LR11XX_SYSTEM_TCXO_CTRL_1_8V, 164));

    // 6. calibrate the radio's internal oscillators, PLL and ADC (0x3F = all six blocks).
    //    this also starts the reference clock, so a clock problem shows up as an error here
    CHECK(lr11xx_system_clear_errors(ctx));
    CHECK(lr11xx_system_calibrate(ctx, 0x3F));

    // 7. image calibration for our band. improves rejection of the image frequency,
    //    which matters for RX. not fatal if it fails, TX works without it
    if (lr11xx_system_calibrate_image_in_mhz(ctx, 430, 440) != LR11XX_STATUS_OK)
        printf("radio: image calibration command failed\r\n");

    CHECK(lr11xx_system_get_errors(ctx, errs));
    return 0;
}

int radio_init(const lr11xx_ctx_t* ctx)
{
    // Which reference clock the board has. Starts as RADIO_USE_TCXO, then remembers
    // whatever worked so later re-inits don't have to guess again.
    static bool use_tcxo = RADIO_USE_TCXO;
    uint16_t errs = 0;

    if (bringup(ctx, use_tcxo, &errs) != 0) return -1;

    // HF_XOSC_START means the 32 MHz reference never started, so the radio is running
    // off its internal RC oscillator. It will still transmit, but off frequency by tens
    // of kHz, and two boards will almost never hear each other. Usually it means the
    // TCXO setting is wrong for this board, so try the other one.
    if (errs & LR11XX_SYSTEM_ERRORS_HF_XOSC_START_MASK) {
        printf("radio: clock didn't start as %s, retrying as %s\r\n",
               use_tcxo ? "TCXO" : "crystal", use_tcxo ? "crystal" : "TCXO");
        use_tcxo = !use_tcxo;
        if (bringup(ctx, use_tcxo, &errs) != 0) return -1;
    }

    print_errors(errs);
    printf("radio: reference clock = %s\r\n", use_tcxo ? "TCXO" : "crystal");
    if (errs & (LR11XX_SYSTEM_ERRORS_HF_XOSC_START_MASK | LR11XX_SYSTEM_ERRORS_PLL_LOCK_MASK)) {
        // not fatal so you can still look at TX on an SDR, but expect RX to fail
        printf("radio: WARNING clock or PLL error. carrier will be off frequency\r\n");
    }
    CHECK(lr11xx_system_clear_errors(ctx));

    // 8. pick which IRQ flags drive DIO9 (second argument would be DIO11, unused)
    CHECK(lr11xx_system_set_dio_irq_params(ctx, RADIO_IRQ_MASK, 0));
    CHECK(lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK));

    // 9. modem settings. packet type has to come first, the rest depend on it
    CHECK(lr11xx_radio_set_pkt_type(ctx, LR11XX_RADIO_PKT_TYPE_GFSK));
    CHECK(lr11xx_radio_set_rf_freq(ctx, RADIO_FREQ_HZ));

    // 10. power amplifier. LP = low power PA (good to about +14 dBm), fed from the
    //     internal regulator. duty cycle and hp_sel are the datasheet values for that PA
    const lr11xx_radio_pa_cfg_t pa = {
        .pa_sel        = LR11XX_RADIO_PA_SEL_LP,
        .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG,
        .pa_duty_cycle = 0x04,
        .pa_hp_sel     = 0x00,
    };
    CHECK(lr11xx_radio_set_pa_cfg(ctx, &pa));
    CHECK(lr11xx_radio_set_tx_params(ctx, RADIO_TX_POWER_DBM, LR11XX_RADIO_RAMP_48_US));

    // 11. modulation and packet framing (structs at the top of this file)
    CHECK(lr11xx_radio_set_gfsk_mod_params(ctx, &mod_params));
    CHECK(lr11xx_radio_set_gfsk_pkt_params(ctx, &pkt_params));

    // 12. sync word, CRC and whitening. all three must match on both ends or
    //     packets are silently ignored (sync) or fail CRC (crc, whitening)
    const uint8_t sync[8] = { 0x12, 0xAD, 0x34, 0xCB, 0, 0, 0, 0 };   // only the first 4 bytes are used (32 bits)
    CHECK(lr11xx_radio_set_gfsk_sync_word(ctx, sync));
    CHECK(lr11xx_radio_set_gfsk_crc_params(ctx, 0x1D0F, 0x1021));     // CRC-16 CCITT seed and polynomial
    CHECK(lr11xx_radio_set_gfsk_whitening_seed(ctx, 0x01FF));

    // 13. where the radio goes after a TX or a single RX finishes
    CHECK(lr11xx_radio_set_rx_tx_fallback_mode(ctx, LR11XX_RADIO_FALLBACK_STDBY_RC));

    printf("radio: ready, GFSK %lu Hz, %d bps, %d dBm\r\n",
           (unsigned long)RADIO_FREQ_HZ, RADIO_BITRATE_BPS, RADIO_TX_POWER_DBM);
    return 0;
}

int radio_send(const lr11xx_ctx_t* ctx, const uint8_t* buf, uint8_t len)
{
    if (len == 0) return -1;

    // tell the radio how long this packet is
    pkt_params.pld_len_in_bytes = len;
    CHECK(lr11xx_radio_set_gfsk_pkt_params(ctx, &pkt_params));

    // clear old flags so DIO9 starts low, load the payload, start TX
    CHECK(lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK));
    CHECK(lr11xx_regmem_write_buffer8(ctx, buf, len));
    CHECK(lr11xx_radio_set_tx(ctx, 0));   // 0 = no radio side timeout, we time it ourselves below

    // wait for DIO9 (TX_DONE). allow the packet's time on air plus 100 ms of slack
    uint32_t timeout = lr11xx_radio_get_gfsk_time_on_air_in_ms(&pkt_params, &mod_params) + 100;
    uint32_t start = HAL_GetTick();
    while (!irq_pin_high(ctx)) {
        if (HAL_GetTick() - start > timeout) {
            printf("radio: TX timed out after %lu ms\r\n", (unsigned long)timeout);
            radio_standby(ctx);   // make sure the radio isn't left stuck in TX
            return -1;
        }
    }

    // DIO9 went high. read which flag it was and clear it
    lr11xx_system_irq_mask_t irq = 0;
    CHECK(lr11xx_system_get_and_clear_irq_status(ctx, &irq));
    return (irq & LR11XX_SYSTEM_IRQ_TX_DONE) ? 0 : -1;
}

int radio_standby(const lr11xx_ctx_t* ctx)
{
    // RC = run from the internal RC oscillator. lowest power standby that still takes commands
    CHECK(lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_RC));
    CHECK(lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK));
    return 0;
}

int radio_start_rx(const lr11xx_ctx_t* ctx)
{
    // settings should only change in standby. this also makes it safe to call
    // radio_start_rx() again while the radio is already in RX
    CHECK(lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_RC));

    // on RX the length field means "biggest packet to accept". allow the max
    pkt_params.pld_len_in_bytes = RADIO_MAX_PAYLOAD;
    CHECK(lr11xx_radio_set_gfsk_pkt_params(ctx, &pkt_params));
    CHECK(lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK));
    // the driver also sets the sub-GHz LNA path and applies Semtech's ACP workaround here
    CHECK(lr11xx_radio_set_rx_with_timeout_in_rtc_step(ctx, RX_CONTINUOUS));
    return 0;
}

radio_rx_result_t radio_poll_rx(const lr11xx_ctx_t* ctx, uint8_t* buf, uint16_t buf_size, radio_rx_info_t* info)
{
    // cheap check first. no SPI traffic unless the radio flagged something
    if (!irq_pin_high(ctx)) return RADIO_RX_NONE;

    // find out what happened and clear it so DIO9 drops again
    lr11xx_system_irq_mask_t irq = 0;
    if (lr11xx_system_get_and_clear_irq_status(ctx, &irq) != LR11XX_STATUS_OK) return RADIO_RX_ERROR;

    // DIO9 says a flag is set but the register read came back empty. The IRQ read
    // doesn't go through the Stat1 check, so this is how a dead MISO line shows up
    // here. Without this check DIO9 never clears and RX silently stops working.
    if (irq == 0) return RADIO_RX_ERROR;

    // a packet came in (good or bad). grab its RSSI, length and error flags
    lr11xx_radio_pkt_status_gfsk_t st = { 0 };
    if (irq & (LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_CRC_ERROR)) {
        if (lr11xx_radio_get_gfsk_pkt_status(ctx, &st) != LR11XX_STATUS_OK) return RADIO_RX_ERROR;
        if (info) {
            info->rssi_dbm = st.rssi_sync_in_dbm;
            info->len      = st.rx_len_in_bytes;
        }
    }

    if ((irq & LR11XX_SYSTEM_IRQ_CRC_ERROR) || st.is_crc_err) return RADIO_RX_CRC_ERROR;
    if (irq & (LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR | LR11XX_SYSTEM_IRQ_TIMEOUT)) return RADIO_RX_ERROR;
    if (!(irq & LR11XX_SYSTEM_IRQ_RX_DONE)) return RADIO_RX_NONE;   // some other flag, nothing to read

    // good packet. ask where it sits in the radio's 256 byte RX buffer, then copy it out
    lr11xx_radio_rx_buffer_status_t bs;
    if (lr11xx_radio_get_rx_buffer_status(ctx, &bs) != LR11XX_STATUS_OK) return RADIO_RX_ERROR;

    uint8_t n = bs.pld_len_in_bytes;
    if (n > buf_size) n = (uint8_t)buf_size;   // never write past the caller's buffer
    if (n > 0 && lr11xx_regmem_read_buffer8(ctx, buf, bs.buffer_start_pointer, n) != LR11XX_STATUS_OK)
        return RADIO_RX_ERROR;

    if (info) info->len = n;
    return RADIO_RX_OK;
}
