#ifndef RADIO_H
#define RADIO_H

/*
 * radio.h
 *
 * Small wrapper around the Semtech LR11xx driver for our GFSK link.
 * main.c only talks to the radio through these functions.
 *
 * Both boards must run the same link settings below. Only the role
 * (TX or RX) differs, and main.c picks that.
 *
 * Typical use:
 *   radio_init(&ctx);                        once at startup
 *   radio_send(&ctx, buf, len);              TX board, once per packet
 *   radio_start_rx(&ctx);                    RX board, once
 *   radio_poll_rx(&ctx, buf, size, &info);   RX board, every loop pass
 */

#include <stdbool.h>
#include <stdint.h>
#include "lr1121_hal_stm32.h"

/* ---- link settings. must match on both boards ------------------------- */
#define RADIO_FREQ_HZ        435000000UL   // carrier frequency
#define RADIO_BITRATE_BPS    4800          // GFSK bit rate
#define RADIO_FDEV_HZ        5000          // GFSK frequency deviation
#define RADIO_TX_POWER_DBM   0             // low power PA. keep it low on the bench with the boards close together
#define RADIO_MAX_PAYLOAD    255           // GFSK variable length packets have a 1 byte length field, so 255 max

/* 1 if the LR1121 board has a TCXO, 0 for a plain crystal.
 * This is only the first guess. If the clock fails to start, radio_init
 * retries with the other setting and prints which one worked. Set this to
 * match so startup doesn't need the retry. */
#define RADIO_USE_TCXO       0

/* What radio_poll_rx() found */
typedef enum {
    RADIO_RX_NONE = 0,   // nothing new
    RADIO_RX_OK,         // good packet copied into the caller's buffer
    RADIO_RX_CRC_ERROR,  // a packet was heard but its CRC failed (noise, weak signal, collision)
    RADIO_RX_ERROR,      // SPI error, length error or the radio left RX. call radio_start_rx() again
} radio_rx_result_t;

/* Extra info about a received packet */
typedef struct {
    uint8_t len;          // payload length in bytes
    int8_t  rssi_dbm;     // signal strength measured when the sync word was detected
} radio_rx_info_t;

/* Reset the radio and load all the link settings.
 * Returns 0 on success, -1 on any failure (the reason is printed). */
int radio_init(const lr11xx_ctx_t* ctx);

/* Send one packet and block until it is fully transmitted.
 * Takes about 35 ms for a short packet at 4800 bps. Returns 0 on success. */
int radio_send(const lr11xx_ctx_t* ctx, const uint8_t* buf, uint8_t len);

/* Stop whatever the radio is doing and park it in standby. Returns 0 on success. */
int radio_standby(const lr11xx_ctx_t* ctx);

/* Start continuous RX. The radio keeps listening after every packet,
 * so this only needs calling once (or again after RADIO_RX_ERROR).
 * Safe to call while already in RX. Returns 0 on success. */
int radio_start_rx(const lr11xx_ctx_t* ctx);

/* Non-blocking check for a received packet. Call it every loop pass.
 * If a good packet arrived it is copied into buf (up to buf_size bytes)
 * and info gets the length and RSSI. info may be NULL. */
radio_rx_result_t radio_poll_rx(const lr11xx_ctx_t* ctx, uint8_t* buf, uint16_t buf_size, radio_rx_info_t* info);

#endif /* RADIO_H */
