
#include "../Inc/lr1121_driver.h"
#include <string.h>

#define CHECK(x) do { if ((x) != LR11XX_STATUS_OK) { printf("FAIL line %d\r\n", __LINE__); return -1; } } while (0)
#define SHOW_ERR(tag) do { uint16_t e; lr11xx_system_get_errors(ctx, &e); printf(tag ": errors=0x%04X\r\n", e); } while (0)


int radio_init(const void* ctx) {
    lr11xx_hal_reset(ctx);  // Reset hal

    lr11xx_system_set_reg_mode(ctx, LR11XX_SYSTEM_REG_MODE_DCDC); // Set power delivery type

    lr11xx_system_rfswitch_cfg_t rf_sw = {
		.enable = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH | LR11XX_SYSTEM_RFSW2_HIGH,
		.standby = 0,
		.rx = LR11XX_SYSTEM_RFSW0_HIGH,
		.tx = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH,
		.tx_hp = LR11XX_SYSTEM_RFSW1_HIGH,
		.tx_hf = 0,
	};  // Define default rfswitch settings
    lr11xx_system_set_dio_as_rf_switch(ctx, &rf_sw); // Load rfswitch settings

    lr11xx_system_clear_errors(ctx);
    lr11xx_system_calibrate(ctx, 0x3F);  // Preparing radio

    lr11xx_system_set_dio_irq_params(ctx, LR11XX_SYSTEM_IRQ_TX_DONE, 0);
    lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK); // Prepare interrupt functionality

    lr11xx_radio_set_pkt_type(ctx, LR11XX_RADIO_PKT_TYPE_GFSK);  // Set modulation
    lr11xx_radio_set_rf_freq(ctx, 915000000);  // Set Frequency

    lr11xx_radio_pa_cfg_t pa = {
		.pa_sel = LR11XX_RADIO_PA_SEL_LP,
		.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG,
		.pa_duty_cycle = 0x04,
		.pa_hp_sel = 0x00,
	};  // Create default packet configurations

    lr11xx_radio_set_pa_cfg(ctx, &pa);
    lr11xx_radio_set_tx_params(ctx, 0, LR11XX_RADIO_RAMP_48_US);   // 0 dBm

    lr11xx_radio_mod_params_gfsk_t mod = {
		.br_in_bps = 4800,
		.pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1,
		.bw_dsb_param = LR11XX_RADIO_GFSK_BW_19500,
		.fdev_in_hz = 5000,
	};
	CHECK(lr11xx_radio_set_gfsk_mod_params(ctx, &mod));


	lr11xx_radio_pkt_params_gfsk_t pkt = {
		.preamble_len_in_bits = 32,
		.preamble_detector = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS,
		.sync_word_len_in_bits = 32,
		.address_filtering = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE,
		.header_type = LR11XX_RADIO_GFSK_PKT_VAR_LEN,
		.pld_len_in_bytes = 255,
		.crc_type = LR11XX_RADIO_GFSK_CRC_2_BYTES,
		.dc_free = LR11XX_RADIO_GFSK_DC_FREE_WHITENING,
	};
	CHECK(lr11xx_radio_set_gfsk_pkt_params(ctx, &pkt));

	uint8_t sync[8] = { 0x12, 0xAD, 0x34, 0xCB, 0, 0, 0, 0 };
	CHECK(lr11xx_radio_set_gfsk_sync_word(ctx, sync));
	CHECK(lr11xx_radio_set_gfsk_crc_params(ctx, 0x1D0F, 0x1021));
	CHECK(lr11xx_radio_set_gfsk_whitening_seed(ctx, 0x01FF));
	CHECK(lr11xx_radio_set_rx_tx_fallback_mode(ctx, LR11XX_RADIO_FALLBACK_STDBY_RC));
	return 0;

}


int radio_send(const void* ctx, const uint8_t* buf) {

	size_t len = strlen((const char*)buf);
	CHECK(lr11xx_regmem_write_buffer8(ctx, buf, len));


	CHECK(lr11xx_radio_set_tx(ctx, 0));

	for (int ms = 0; ms < 200; ms += 10) {
		printf("t=%d BUSY=%d DIO9=%d\r\n", ms,
			   HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13),
			   HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11));

		if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_SET) break;
		HAL_Delay(10);
	}

	lr11xx_system_irq_mask_t irq = 0;
	CHECK(lr11xx_system_get_and_clear_irq_status(ctx, &irq));
	printf("irq=0x%08lX\r\n", (unsigned long)irq);
	return (irq & LR11XX_SYSTEM_IRQ_TX_DONE) ? 0 : -1;
}

// Returns: >0 = bytes received, 0 = timeout, -1 = SPI/driver error, -2 = CRC/header error
int radio_receive(const void* ctx, uint8_t* buf, uint8_t max_len, uint32_t timeout_ms)
{
    const lr11xx_system_irq_mask_t mask = LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT |
                                          LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_HEADER_ERROR;
    CHECK(lr11xx_system_set_dio_irq_params(ctx, mask, 0));
    CHECK(lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK));
    CHECK(lr11xx_radio_set_rx(ctx, timeout_ms));

    uint32_t start = HAL_GetTick();
    lr11xx_system_irq_mask_t irq = 0;
    while (HAL_GetTick() - start < timeout_ms + 100) {
        CHECK(lr11xx_system_get_and_clear_irq_status(ctx, &irq));

        if (irq & (LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_HEADER_ERROR)) return -2;
        if (irq & LR11XX_SYSTEM_IRQ_RX_DONE) {
            lr11xx_radio_rx_buffer_status_t st;
            CHECK(lr11xx_radio_get_rx_buffer_status(ctx, &st));
            uint8_t n = (st.pld_len_in_bytes > max_len) ? max_len : st.pld_len_in_bytes;
            CHECK(lr11xx_regmem_read_buffer8(ctx, buf, st.buffer_start_pointer, n));
            return n;
        }
        if (irq & LR11XX_SYSTEM_IRQ_TIMEOUT) return 0;
        HAL_Delay(5);
    }
    return 0;
}

