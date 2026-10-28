
#include "../Inc/lr1121_driver.h"
#include <string.h>

#define CHECK(x) do { if ((x) != LR11XX_STATUS_OK) { printf("FAIL line %d\r\n", __LINE__); return -1; } } while (0)


enum Band {
	S_BAND,
	UHF
};

static enum Band current_band = UHF;

static uint32_t uhf_frequency = 915000000UL;  // Default value
static uint32_t sband_frequency = 2450000000UL; // Default value

static lr11xx_radio_pkt_type_t g_pkt_type;

static lr11xx_radio_pkt_params_gfsk_t g_gfsk_pkt;
static lr11xx_radio_pkt_params_lora_t g_lora_pkt;



int radio_init(const void* ctx) {
    lr11xx_hal_reset(ctx);  // Reset hal

    CHECK(lr11xx_system_set_reg_mode(ctx, LR11XX_SYSTEM_REG_MODE_DCDC)); // Set power delivery type

    lr11xx_system_rfswitch_cfg_t rf_sw = {
		.enable = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH | LR11XX_SYSTEM_RFSW2_HIGH,
		.standby = 0,
		.rx = LR11XX_SYSTEM_RFSW0_HIGH,
		.tx = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH,
		.tx_hp = LR11XX_SYSTEM_RFSW1_HIGH,
		.tx_hf = 0,
	};  // Define default rfswitch settings
    CHECK(lr11xx_system_set_dio_as_rf_switch(ctx, &rf_sw)); // Load rfswitch settings

    CHECK(lr11xx_system_clear_errors(ctx));
    CHECK(lr11xx_system_calibrate(ctx, 0x3F));  // Preparing radio

    printf("Calibrating image in mhz. May crash radio, remove here if bugging \r\n");
    CHECK(lr11xx_system_calibrate_image_in_mhz(ctx, 902, 928));

//    CHECK(lr11xx_system_set_dio_irq_params(ctx,
//              LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT |
//              LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_HEADER_ERROR, 0));
    CHECK(lr11xx_system_set_dio_irq_params(ctx, LR11XX_SYSTEM_IRQ_TX_DONE, 0));
    CHECK(lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK)); // Prepare interrupt functionality
	CHECK(lr11xx_radio_set_rx_tx_fallback_mode(ctx, LR11XX_RADIO_FALLBACK_STDBY_RC)); // Set fallback mode

	return 0;

	// End of default init


//
//    CHECK(lr11xx_radio_set_pkt_type(ctx, LR11XX_RADIO_PKT_TYPE_GFSK));  // Set modulation
//    CHECK(lr11xx_radio_set_rf_freq(ctx, 915000000));  // Set Frequency
//
//    lr11xx_radio_pa_cfg_t pa = {
//		.pa_sel = LR11XX_RADIO_PA_SEL_LP, // Low power
//		.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG,
//		.pa_duty_cycle = 0x04,
//		.pa_hp_sel = 0x00,
//	};  // Create default power amplifier configurations
//
//    CHECK(lr11xx_radio_set_pa_cfg(ctx, &pa)); // Load default power amplifier configurations
//    CHECK(lr11xx_radio_set_tx_params(ctx, 0, LR11XX_RADIO_RAMP_48_US));   // 0 dBm (transmit parameters)
//
//    lr11xx_radio_mod_params_gfsk_t mod = {
//		.br_in_bps = 4800,
//		.pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1,
//		.bw_dsb_param = LR11XX_RADIO_GFSK_BW_19500,
//		.fdev_in_hz = 5000,
//	};  // Create modulation parameters
//	CHECK(lr11xx_radio_set_gfsk_mod_params(ctx, &mod)); // Load in modulation parameters
//
////	g_pkt.preamble_len_in_bits = 32;
////	g_pkt.preamble_detector = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS;
////	g_pkt
//
//	lr11xx_radio_pkt_params_gfsk_t default_g_pkt = {
//		.preamble_len_in_bits = 32,
//		.preamble_detector = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS,
//		.sync_word_len_in_bits = 32,
//		.address_filtering = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE,
//		.header_type = LR11XX_RADIO_GFSK_PKT_VAR_LEN,
//		.pld_len_in_bytes = 255,
//		.crc_type = LR11XX_RADIO_GFSK_CRC_2_BYTES,
//		.dc_free = LR11XX_RADIO_GFSK_DC_FREE_WHITENING,
//	};  // Create GFSK packet parameters
//	g_pkt = default_g_pkt;
//	CHECK(lr11xx_radio_set_gfsk_pkt_params(ctx, &g_pkt)); // Load GFSK packet parameters
//
//	uint8_t sync[8] = { 0x12, 0xAD, 0x34, 0xCB, 0, 0, 0, 0 };  // Create sync word
//	CHECK(lr11xx_radio_set_gfsk_sync_word(ctx, sync));  // Load sync word
//	CHECK(lr11xx_radio_set_gfsk_crc_params(ctx, 0x1D0F, 0x1021));  // Set gfsk cyclic redundancy check parameters
//	CHECK(lr11xx_radio_set_gfsk_whitening_seed(ctx, 0x01FF));  // Set whitening seed
//	CHECK(lr11xx_radio_set_rx_tx_fallback_mode(ctx, LR11XX_RADIO_FALLBACK_STDBY_RC)); // Set fallback mode
//	return 0;
}

int radio_set_uhf(const void* ctx) {
    CHECK(lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_RC));
    //CHECK(lr11xx_system_calibrate_image_in_mhz(ctx, 902, 928));   // image cal is for the sub-GHz path
    CHECK(lr11xx_radio_set_rf_freq(ctx, uhf_frequency));

    lr11xx_radio_pa_cfg_t pa = {
		.pa_sel = LR11XX_RADIO_PA_SEL_LP,
		.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG,
		.pa_duty_cycle = 0x04,
		.pa_hp_sel = 0x00,
	};
    CHECK(lr11xx_radio_set_pa_cfg(ctx, &pa));  // Define and load power configuration
    CHECK(lr11xx_radio_set_tx_params(ctx, 0, LR11XX_RADIO_RAMP_48_US));   // 0 dBm

    current_band = UHF;
    return 0;
}

int radio_set_sband(const void* ctx) {
    CHECK(lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_RC));
    CHECK(lr11xx_radio_set_rf_freq(ctx, sband_frequency));

    lr11xx_radio_pa_cfg_t pa = {      // from the -2 dBm entry in your shield file. Copy the entry for your target power.
		.pa_sel = LR11XX_RADIO_PA_SEL_HF,
		.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG,
		.pa_duty_cycle = 0x03,
		.pa_hp_sel = 0x00,
	};
    CHECK(lr11xx_radio_set_pa_cfg(ctx, &pa));  // Set and load power config for sband
    CHECK(lr11xx_radio_set_tx_params(ctx, -2, LR11XX_RADIO_RAMP_48_US));

    return 0;
}

int radio_set_gfsk(const void* ctx) {
	CHECK(lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_RC));
	CHECK(lr11xx_radio_set_pkt_type(ctx, LR11XX_RADIO_PKT_TYPE_GFSK));

	lr11xx_radio_mod_params_gfsk_t mod = {
		.br_in_bps = 4800,
		.pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1,
		.bw_dsb_param = LR11XX_RADIO_GFSK_BW_19500,
		.fdev_in_hz = 5000,
	};
	CHECK(lr11xx_radio_set_gfsk_mod_params(ctx, &mod));  // Define and load modulation params

	g_gfsk_pkt = (lr11xx_radio_pkt_params_gfsk_t){
		.preamble_len_in_bits = 32,
		.preamble_detector = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS,
		.sync_word_len_in_bits = 32,
		.address_filtering = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE,
		.header_type = LR11XX_RADIO_GFSK_PKT_VAR_LEN,
		.pld_len_in_bytes = 255,
		.crc_type = LR11XX_RADIO_GFSK_CRC_2_BYTES,
		.dc_free = LR11XX_RADIO_GFSK_DC_FREE_WHITENING,
	};
	CHECK(lr11xx_radio_set_gfsk_pkt_params(ctx, &g_gfsk_pkt)); // Define and load packet params

	uint8_t sync[8] = { 0x12, 0xAD, 0x34, 0xCB, 0, 0, 0, 0 };
	CHECK(lr11xx_radio_set_gfsk_sync_word(ctx, sync));
	CHECK(lr11xx_radio_set_gfsk_crc_params(ctx, 0x1D0F, 0x1021));
	CHECK(lr11xx_radio_set_gfsk_whitening_seed(ctx, 0x01FF));  // Set sync word, crc, and whitening params
	g_pkt_type = LR11XX_RADIO_PKT_TYPE_GFSK;

	return 0;
}

int radio_set_lora(const void* ctx) {
	CHECK(lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_RC));
	CHECK(lr11xx_radio_set_pkt_type(ctx, LR11XX_RADIO_PKT_TYPE_LORA));

	lr11xx_radio_mod_params_lora_t mod = {
		.sf = LR11XX_RADIO_LORA_SF7,
		.bw = (current_band == S_BAND) ? LR11XX_RADIO_LORA_BW_400 : LR11XX_RADIO_LORA_BW_125,
		.cr = LR11XX_RADIO_LORA_CR_4_5,
		.ldro = 0
	};
	CHECK(lr11xx_radio_set_lora_mod_params(ctx, &mod));

	g_lora_pkt = (lr11xx_radio_pkt_params_lora_t){
		.preamble_len_in_symb = 8,
		.header_type = LR11XX_RADIO_LORA_PKT_EXPLICIT,
		.pld_len_in_bytes = 255,
		.crc = LR11XX_RADIO_LORA_CRC_ON,
		.iq = LR11XX_RADIO_LORA_IQ_STANDARD
	};
	CHECK(lr11xx_radio_set_lora_pkt_params(ctx, &g_lora_pkt));
	CHECK(lr11xx_radio_set_lora_public_network(ctx, LR11XX_RADIO_LORA_NETWORK_PRIVATE));
	g_pkt_type = LR11XX_RADIO_PKT_TYPE_LORA;

	return 0;
}


int radio_send(const void* ctx, const uint8_t* buf) {
    CHECK(lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK));

	size_t len = strlen((const char*)buf);
	CHECK(lr11xx_regmem_write_buffer8(ctx, buf, len)); // Write message into buffer

	CHECK(lr11xx_radio_set_tx(ctx, 0)); // Set radio to transmit mode

	uint32_t start = HAL_GetTick();
	lr11xx_system_irq_mask_t irq = 0;
	while (HAL_GetTick() - start < 500) {
		CHECK(lr11xx_system_get_and_clear_irq_status(ctx, &irq));
		if (irq & LR11XX_SYSTEM_IRQ_TX_DONE) return 0;
		HAL_Delay(2);
	}
	return -1;
}

int set_send_packet_length(const void* ctx, uint8_t length) {

	printf("Updating packet length to %d \r\n", length);
	if(	g_pkt_type == LR11XX_RADIO_PKT_TYPE_GFSK ) {
		g_gfsk_pkt.pld_len_in_bytes = length;
		CHECK(lr11xx_radio_set_gfsk_pkt_params(ctx, &g_gfsk_pkt));
	}
	else if( g_pkt_type == LR11XX_RADIO_PKT_TYPE_LORA ) {
		g_lora_pkt.pld_len_in_bytes = length;
		CHECK(lr11xx_radio_set_lora_pkt_params(ctx, &g_lora_pkt));
	}
	else {
		return 1;
	}

    CHECK(lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK));
	return 0;
}

// Returns: >0 = bytes received, 0 = timeout, -1 = SPI/driver error, -2 = CRC/header error
int radio_receive(const void* ctx, uint8_t* buf, uint8_t max_len, uint32_t timeout_ms)
{
//    const lr11xx_system_irq_mask_t mask = LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT |
//                                          LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_HEADER_ERROR;
//    CHECK(lr11xx_system_set_dio_irq_params(ctx, mask, 0)); // CHECK IRQ
    CHECK(lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK)); // Clear IRQ status
    CHECK(lr11xx_radio_set_rx(ctx, timeout_ms)); // Set receive timeout

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

