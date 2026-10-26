#ifndef LR1121_DRIVER_H
#define LR1121_DRIVER_H

#include <stdio.h>
#include "lr11xx_system.h"
#include "lr1121_hal_stm32.h"
#include "lr11xx_hal.h"

#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"


int radio_init(const void* ctx);

int radio_set_uhf(const void* ctx);

int radio_set_sband(const void* ctx);

int radio_set_gfsk(const void* ctx);

int radio_set_lora(const void* ctx);



int radio_send(const void* ctx, const uint8_t* message);

int radio_receive(const void* ctx, uint8_t* buf, uint8_t max_len, uint32_t timeout_ms);

int set_send_packet_length(const void* ctx, uint8_t length);


#endif
