
#include "main.h"

typedef struct {
    SPI_HandleTypeDef* spi;
    GPIO_TypeDef* nss_port;   uint16_t nss_pin;
    GPIO_TypeDef* busy_port;  uint16_t busy_pin;
    GPIO_TypeDef* reset_port; uint16_t reset_pin;
} lr11xx_ctx_t;
