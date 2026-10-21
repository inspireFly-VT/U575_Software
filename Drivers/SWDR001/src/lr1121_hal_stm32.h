#ifndef LR1121_HAL_STM32_H
#define LR1121_HAL_STM32_H

/*
 * lr1121_hal_stm32.h
 *
 * The Semtech driver never touches hardware directly. Every driver call takes
 * a "context" pointer and hands it to the functions in lr1121_hal.c, which
 * cast it back to this struct to find the SPI bus and pins.
 *
 * Fill one of these in per radio (see radio_ctx in main.c).
 */

#include "main.h"

typedef struct {
    SPI_HandleTypeDef* spi;                         // SPI bus the radio is on (SPI1)
    GPIO_TypeDef* nss_port;   uint16_t nss_pin;     // chip select. MCU -> radio, active low
    GPIO_TypeDef* busy_port;  uint16_t busy_pin;    // BUSY. radio -> MCU, high while the radio can't take a command
    GPIO_TypeDef* reset_port; uint16_t reset_pin;   // NRESET. MCU -> radio, active low
    GPIO_TypeDef* irq_port;   uint16_t irq_pin;     // DIO9. radio -> MCU, high while an enabled IRQ flag is set
} lr11xx_ctx_t;

#endif /* LR1121_HAL_STM32_H */
