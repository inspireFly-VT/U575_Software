/*
 * lr1121_hal.c
 *
 * Hardware glue between the Semtech LR11xx driver (SWDR001) and the STM32 HAL.
 *
 * The Semtech driver builds the command bytes for every radio operation, then
 * calls the functions in this file to actually move those bytes over SPI.
 * The prototypes come from lr11xx_hal.h. We only supply the bodies.
 *
 * Every SPI transaction follows the same rule from the LR1121 datasheet:
 *   1. wait for BUSY to go low (radio ready for a command)
 *   2. pull NSS low
 *   3. clock the bytes
 *   4. release NSS high
 * The radio raises BUSY while it processes a command, so the next
 * transaction always starts by waiting on it again.
 */

#include "lr11xx_hal.h"
#include "lr1121_hal_stm32.h"

#include "main.h"
#include <stdio.h>
#include <string.h>

#define BUSY_TIMEOUT_MS 1000   // calibration is the slowest command and takes well under this
#define SPI_TIMEOUT_MS  100    // per HAL_SPI call

// Sent on MOSI while reading. The radio treats 0x00 as a NOP, so nothing we
// clock out during a read can be mistaken for a command.
static const uint8_t spi_zeros[32] = { 0 };

// Set to 0 to skip the Stat1 check in lr11xx_hal_read (see the comment there).
#define LR11XX_HAL_CHECK_STAT1 1

// The radio raises BUSY a fraction of a microsecond after NSS goes high, not
// instantly. At our current 4 MHz clock the code is slow enough that this never
// matters. At 160 MHz (Tristian's power modes branch) we could read BUSY before
// it rises, think the radio is ready, and send the next command too early.
// This waits about 1 us at any clock speed. It costs nothing noticeable at 4 MHz.
static void busy_settle(void)
{
    for (volatile uint32_t i = SystemCoreClock / 4000000U + 1U; i > 0U; i--) {}
}

// Spin until BUSY is low. Returns false if the radio stays busy too long,
// which usually means it is unpowered, held in reset, or BUSY is miswired.
static bool wait_busy_low(const lr11xx_ctx_t* c)
{
    busy_settle();
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(c->busy_port, c->busy_pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() - start > BUSY_TIMEOUT_MS) return false;
    }
    return true;
}

// Read len bytes into data while clocking out 0x00.
// Done in 32 byte chunks so any length works without a big buffer.
// HAL_SPI_Receive is avoided on purpose. It switches SPI to receive-only mode,
// which stops driving MOSI, so the radio could see random bits as a command.
static HAL_StatusTypeDef spi_read_zeros(SPI_HandleTypeDef* spi, uint8_t* data, uint16_t len)
{
    HAL_StatusTypeDef s = HAL_OK;
    while (len > 0 && s == HAL_OK) {
        uint16_t n = (len > sizeof(spi_zeros)) ? sizeof(spi_zeros) : len;
        s = HAL_SPI_TransmitReceive(spi, spi_zeros, data, n, SPI_TIMEOUT_MS);
        data += n;
        len -= n;
    }
    return s;
}

/*
 * Write: one transaction. Command bytes (opcode + arguments), then optional
 * data bytes (for example the packet payload for WriteBuffer8).
 */
lr11xx_hal_status_t lr11xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    const lr11xx_ctx_t* c = context;
    if (!wait_busy_low(c)) return LR11XX_HAL_STATUS_ERROR;

    HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef s = HAL_SPI_Transmit(c->spi, command, command_length, SPI_TIMEOUT_MS);
    if (s == HAL_OK && data_length > 0)
        s = HAL_SPI_Transmit(c->spi, data, data_length, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_SET);

    return (s == HAL_OK) ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

/*
 * Read: two transactions.
 *   phase 1  send the command. the radio raises BUSY while it prepares the answer
 *   phase 2  once BUSY drops, clock the answer out. the first byte is always
 *            the Stat1 status byte, then the requested data follows
 */
lr11xx_hal_status_t lr11xx_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    const lr11xx_ctx_t* c = context;

    // phase 1
    if (!wait_busy_low(c)) return LR11XX_HAL_STATUS_ERROR;
    HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef s = HAL_SPI_Transmit(c->spi, command, command_length, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_SET);
    if (s != HAL_OK) return LR11XX_HAL_STATUS_ERROR;

    // phase 2
    // (the old version copied through a fixed 32 byte buffer and failed any
    //  read over 31 bytes, which would have broken reading RX packets)
    if (!wait_busy_low(c)) return LR11XX_HAL_STATUS_ERROR;
    uint8_t stat1 = 0;
    HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_RESET);
    s = spi_read_zeros(c->spi, &stat1, 1);
    if (s == HAL_OK && data_length > 0)
        s = spi_read_zeros(c->spi, data, data_length);
    HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_SET);
    if (s != HAL_OK) return LR11XX_HAL_STATUS_ERROR;

#if LR11XX_HAL_CHECK_STAT1
    // Stat1 bits 3:1 say how the radio handled our read command:
    //   0 = FAIL, 1 = PERR (bad parameter), 2 = OK, 3 = DATA (answer follows)
    // Anything but OK or DATA means the radio rejected the command, or the SPI
    // link is dead (MISO stuck low reads 0x00 = FAIL, stuck high reads 0xFF = invalid).
    // Without this check a dead link returns all zeros and reports success.
    uint8_t cmd_status = (uint8_t)((stat1 >> 1) & 0x07);
    if (cmd_status != 2 && cmd_status != 3) {
        printf("lr1121: read cmd 0x%02X%02X rejected, stat1=0x%02X\r\n", command[0], command[1], stat1);
        return LR11XX_HAL_STATUS_ERROR;
    }
#endif
    return LR11XX_HAL_STATUS_OK;
}

/*
 * Direct read: one transaction with no command phase. The radio shifts its
 * status bytes out while we clock zeros in. Used by lr11xx_system_get_status.
 */
lr11xx_hal_status_t lr11xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length )
{
    const lr11xx_ctx_t* c = context;
    if (!wait_busy_low(c)) return LR11XX_HAL_STATUS_ERROR;

    HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef s = spi_read_zeros(c->spi, data, data_length);
    HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_SET);

    return (s == HAL_OK) ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

/*
 * Hardware reset. Pulse NRESET low, then wait for the radio to boot.
 * The radio holds BUSY high until it is ready, so we finish by waiting on it.
 */
lr11xx_hal_status_t lr11xx_hal_reset( const void* context )
{
    const lr11xx_ctx_t* c = context;
    HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_SET);   // NSS has to idle high while the radio boots
    HAL_GPIO_WritePin(c->reset_port, c->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(c->reset_port, c->reset_pin, GPIO_PIN_SET);
    HAL_Delay(5);
    return wait_busy_low(c) ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

/*
 * Wake from sleep. A falling edge on NSS wakes the radio.
 * Not used yet, since we never put the radio to sleep.
 */
lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context )
{
    const lr11xx_ctx_t* c = context;
    HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_SET);
    return wait_busy_low(c) ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

/*
 * Only the GNSS and WiFi scan commands use this. We don't use either,
 * so it just reports that it was called.
 */
lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd( const void* context )
{
    printf("lr1121 abort blocking command (not implemented)\r\n");
    return LR11XX_HAL_STATUS_ERROR;
}

//julia wuz here
