
#include "lr11xx_hal.h"
#include "lr1121_hal_stm32.h"

#include "main.h"   // for HAL and pin defines


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
	printf("lr1121 write\r\n");

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
	printf("lr1121 read\r\n");

	const lr11xx_ctx_t* c = context;

	// Phase 1: send the command
	if (!wait_busy_low(c)) return LR11XX_HAL_STATUS_ERROR;
	HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef s = HAL_SPI_Transmit(c->spi, (uint8_t*)command, command_length, 100);
	HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_SET);
	if (s != HAL_OK) return LR11XX_HAL_STATUS_ERROR;

	// Phase 2: wait for BUSY, then clock out a dummy byte (stat1) + data
	if (!wait_busy_low(c)) return LR11XX_HAL_STATUS_ERROR;
	HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_RESET);
	uint8_t dummy = 0x00;
	s = HAL_SPI_Transmit(c->spi, &dummy, 1, 100);   // NOP; response status byte is discarded here
	if (s == HAL_OK && data_length > 0)
		s = HAL_SPI_Receive(c->spi, data, data_length, 100);
	HAL_GPIO_WritePin(c->nss_port, c->nss_pin, GPIO_PIN_SET);

	return (s == HAL_OK) ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}


lr11xx_hal_status_t lr11xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length )
{
	printf("lr1121 direct read\r\n");
	return LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_reset( const void* context )
{
	printf("lr1121 reset \r\n");

	const lr11xx_ctx_t* c = context;
	HAL_GPIO_WritePin(c->reset_port, c->reset_pin, GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(c->reset_port, c->reset_pin, GPIO_PIN_SET);
	HAL_Delay(5);
	return wait_busy_low(c) ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context )
{
	printf("lr1121 wake up\r\n");

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





