
#include "lr11xx_hal.h"
#include "lr1121_hal_stm32.h"

#include "main.h"   // for HAL and pin defines
#include <string.h>


#define BUSY_TIMEOUT_MS 100

static bool wait_busy_low(const lr11xx_ctx_t* c) {
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(c->busy_port, c->busy_pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() - start > BUSY_TIMEOUT_MS) return false;
    }
    return true;
}


lr11xx_hal_status_t lr11xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
	const lr11xx_ctx_t* c = context;
	if (!wait_busy_low(c)) return LR11XX_HAL_STATUS_ERROR;

	HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef s = HAL_SPI_Transmit(c->spi, (uint8_t*)command, command_length, 100);
	if (s == HAL_OK && data_length > 0)
		s = HAL_SPI_Transmit(c->spi, (uint8_t*)data, data_length, 100);
	HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_SET);

	return (s == HAL_OK) ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
	const lr11xx_ctx_t* c = context;

	// Phase 1: send the command
	if (!wait_busy_low(c)) return LR11XX_HAL_STATUS_ERROR;
	HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef s = HAL_SPI_Transmit(c->spi, (uint8_t*)command, command_length, 100);
	HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_SET);
	if (s != HAL_OK) return LR11XX_HAL_STATUS_ERROR;

	// Phase 2
	if (!wait_busy_low(c)) return LR11XX_HAL_STATUS_ERROR;
	uint8_t tx[32] = {0};
	uint8_t rx[32];
	if (data_length + 1 > sizeof(tx)) return LR11XX_HAL_STATUS_ERROR;

	HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_RESET);
	s = HAL_SPI_TransmitReceive(c->spi, tx, rx, data_length + 1, 100);
	HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_SET);

	if (s != HAL_OK) return LR11XX_HAL_STATUS_ERROR;
	memcpy(data, &rx[1], data_length);   // rx[0] is stat1
	return LR11XX_HAL_STATUS_OK;
}


lr11xx_hal_status_t lr11xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length )
{
    const lr11xx_ctx_t* c = context;

    if (!wait_busy_low(c)) return LR11XX_HAL_STATUS_ERROR;

    HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef s = HAL_SPI_Receive(c->spi, data, data_length, 100);
    HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_SET);

    return (s == HAL_OK) ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_reset( const void* context )
{
	const lr11xx_ctx_t* c = context;
	HAL_GPIO_WritePin(c->reset_port, c->reset_pin, GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(c->reset_port, c->reset_pin, GPIO_PIN_SET);
	HAL_Delay(5);
	return wait_busy_low(c) ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context )
{

	const lr11xx_ctx_t* c = context;
	HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_SET);
	return wait_busy_low(c) ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd( const void* context )
{
	printf("lr1121 abort blocking command\r\n");

	return LR11XX_HAL_STATUS_ERROR;
}

//julia wuz here





