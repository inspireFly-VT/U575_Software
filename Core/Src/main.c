/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lr11xx_system.h"
#include "lr1121_hal_stm32.h"
#include "lr11xx_hal.h"

#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define CHECK(x) do { if ((x) != LR11XX_STATUS_OK) { printf("FAIL line %d\r\n", __LINE__); return -1; } } while (0)
#define SHOW_ERR(tag) do { uint16_t e; lr11xx_system_get_errors(ctx, &e); printf(tag ": errors=0x%04X\r\n", e); } while (0)


int radio_init(const void* ctx)
{
    lr11xx_hal_reset(ctx);

    // Chip setup (in standby RC)
    CHECK(lr11xx_system_set_reg_mode(ctx, LR11XX_SYSTEM_REG_MODE_DCDC));
    SHOW_ERR("after reg_mode");


    lr11xx_system_rfswitch_cfg_t rf_sw = {
        // VERIFY: copy from the LR1121MB1DIS shield file
        .enable = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH | LR11XX_SYSTEM_RFSW2_HIGH,
		.standby = 0,
		.rx = LR11XX_SYSTEM_RFSW0_HIGH,
		.tx = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH,
		.tx_hp = LR11XX_SYSTEM_RFSW1_HIGH,
		.tx_hf = 0,
    };
    CHECK(lr11xx_system_set_dio_as_rf_switch(ctx, &rf_sw));

    // VERIFY: TCXO voltage and timeout (timeout is in 30.52 us steps)
    //CHECK(lr11xx_system_set_tcxo_mode(ctx, LR11XX_SYSTEM_TCXO_CTRL_1_8V, 164)); //Maybe optional function
    SHOW_ERR("after tcxo");


    CHECK(lr11xx_system_clear_errors(ctx));
    CHECK(lr11xx_system_calibrate(ctx, 0x3F));              // all blocks

    uint16_t errs;
    lr11xx_system_get_errors(ctx, &errs);
    printf("after calibrate: errors=0x%04X\r\n", errs);

    //CHECK(lr11xx_system_calibrate_image_in_mhz(ctx, 430, 440));
    CHECK(lr11xx_system_set_dio_irq_params(ctx, LR11XX_SYSTEM_IRQ_TX_DONE, 0));
    CHECK(lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK));

    // Radio config
    CHECK(lr11xx_radio_set_pkt_type(ctx, LR11XX_RADIO_PKT_TYPE_GFSK));
    CHECK(lr11xx_radio_set_rf_freq(ctx, 435000000));

    lr11xx_radio_pa_cfg_t pa = {
        .pa_sel = LR11XX_RADIO_PA_SEL_LP,
        .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG,
        .pa_duty_cycle = 0x04,
        .pa_hp_sel = 0x00,
    };
    CHECK(lr11xx_radio_set_pa_cfg(ctx, &pa));
    CHECK(lr11xx_radio_set_tx_params(ctx, 0, LR11XX_RADIO_RAMP_48_US));   // 0 dBm

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
        .pld_len_in_bytes = 11,
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

int radio_send(const void* ctx, const uint8_t* buf, uint8_t len)
{
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  /*
   * Pins for LR1121 Dev Board Perspective
   * SCK = PA5
   * MISO = PA6
   * MOSI = PA7
   * NSS(CS) = PF13
   * Reset = PA0 (or PA3?)
   * Busy = PE13
   * DIO9(IRQ) = PE11
   */


  // Radio Init

  lr11xx_ctx_t radio_context = {
		  .spi = &hspi1,
		  .nss_port = GPIOF, .nss_pin = GPIO_PIN_13,
		  .reset_port = GPIOA, .reset_pin = GPIO_PIN_3,
		  .busy_port = GPIOE, .busy_pin = GPIO_PIN_13,
  };
  lr11xx_system_version_t version;


  // Radio Set up
  printf("Setting NSS high \r\n");
  HAL_GPIO_WritePin(radio_context.nss_port, radio_context.nss_pin, GPIO_PIN_SET);

  //static const uint8_t message[12] = "Hello World!";


  /* Function to use:
   *
   *
   * Set Frequency: lr11xx_radio_set_rf_freq( const void* context, const uint32_t freq_in_hz );
   *
   */
//  lr11xx_hal_reset(&radio_context); //Reset radio
//
//  lr11xx_system_set_reg_mode(&radio_context, LR11XX_SYSTEM_REG_MODE_DCDC);
//
//  lr11xx_system_rfswitch_cfg_t rf_sw = {
//          // VERIFY: copy from the LR1121MB1DIS shield file
//      .enable = 0, .standby = 0, .rx = 0, .tx = 0, .tx_hp = 0, .tx_hf = 0,
//  };
//
//  lr11xx_system_set_dio_as_rf_switch(&radio_context, &rf_sw); // Init DIO for the RX switch
//
//  lr11xx_system_set_tcxo_mode(&radio_context, LR11XX_SYSTEM_TCXO_CTRL_1_8V, 164); // Set power?
//
//  lr11xx_system_clear_errors(&radio_context);  // Clear Errors
//
//  lr11xx_system_calibrate(&radio_context, 0x3F); // Calibrate Radio
//
//  lr11xx_system_calibrate_image_in_mhz(&radio_context, 430, 440); //Calibrate for our frequency range
//
//  lr11xx_system_set_dio_irq_params(&radio_context, LR11XX_SYSTEM_IRQ_TX_DONE, 0); // More configuring for interrupts
//
//  lr11xx_system_clear_irq_status(&radio_context, LR11XX_SYSTEM_IRQ_ALL_MASK); // More IRQ stuff
//
//  lr11xx_radio_set_pkt_type(&radio_context, LR11XX_RADIO_PKT_TYPE_GFSK); // Set packet type to GFSK;
//
//  lr11xx_radio_set_rf_freq(&radio_context, 435000000);

//  lr11xx_radio_pa_cfg_t pa = {
//          .pa_sel = LR11XX_RADIO_PA_SEL_LP,
//          .pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG,
//          .pa_duty_cycle = 0x04,
//          .pa_hp_sel = 0x00,
//      };
//
//
//  lr11xx_radio_set_pa_cfg(&radio_context, &pa); // Load in packet configurations
//
//  lr11xx_radio_set_tx_params(&radio_context, 0, LR11XX_RADIO_RAMP_48_US); // Set transmit parameters
//
//  lr11xx_radio_mod_params_gfsk_t mod = {
//          .br_in_bps = 4800,
//          .pulse_shape = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_1,
//          .bw_dsb_param = LR11XX_RADIO_GFSK_BW_19500,
//          .fdev_in_hz = 5000,
//      };
//
//  lr11xx_radio_set_gfsk_mod_params(&radio_context, &mod); //Load modulation parameters
//
//  lr11xx_radio_pkt_params_gfsk_t pkt = {
//          .preamble_len_in_bits = 32,
//          .preamble_detector = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_8BITS,
//          .sync_word_len_in_bits = 32,
//          .address_filtering = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE,
//          .header_type = LR11XX_RADIO_GFSK_PKT_VAR_LEN,
//          .pld_len_in_bytes = 11,
//          .crc_type = LR11XX_RADIO_GFSK_CRC_2_BYTES,
//          .dc_free = LR11XX_RADIO_GFSK_DC_FREE_WHITENING,
//      };
//
//  lr11xx_radio_set_gfsk_pkt_params(&radio_context, &pkt); //  Packet parameters
//
//  uint8_t sync[8] = { 0x12, 0xAD, 0x34, 0xCB, 0, 0, 0, 0 }; // Create sync word
//  lr11xx_radio_set_gfsk_sync_word(&radio_context, sync); // Load sync word
//  lr11xx_radio_set_gfsk_crc_params(&radio_context, 0x1D0F, 0x1021); // Create CRC (Cyclic redundancy check) parameters
//  lr11xx_radio_set_gfsk_whitening_seed(&radio_context, 0x01FF); // More weird radio stuff (whitening)
//  lr11xx_radio_set_rx_tx_fallback_mode(&radio_context, LR11XX_RADIO_FALLBACK_STDBY_RC); // Radio fallback to standby
//

  if (radio_init(&radio_context) != 0) { printf("init failed\r\n"); }
//  lr11xx_hal_reset(&radio_context);

  int on_off = 1;

  int counter = 0;
  while (on_off)
  {
	  printf("Hello World %d\r\n", counter);
	  counter++;


	  printf("BUSY=%d\r\n", HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13));
	  //printf("Toggling Reset... \r\n");
	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);

	  lr11xx_system_get_version(&radio_context, &version);
	  printf("Version Test: hw=0x%02X type=0x%02X fw=0x%04X\r\n \n\n", version.hw, version.type, version.fw);

	  HAL_Delay(500);

	  // Send message

//	  for(int i = 0; i < 50; i++){
//		  lr11xx_regmem_write_buffer8(&radio_context, message, 12);  // Load message
//		  lr11xx_radio_set_tx(&radio_context, 0); //Set radio to transmit mode
//
//		  uint32_t start_time = HAL_GetTick();
//		  lr11xx_system_irq_mask_t irq = 0;
//		  while (HAL_GetTick() - start_time < 500) {
//			  lr11xx_system_get_and_clear_irq_status(&radio_context, &irq);
//			  if (irq & LR11XX_SYSTEM_IRQ_TX_DONE){
//				  break;
//			  }
//		  }
//	  }


	  printf("Attempting to send message... \r\n");

	  radio_send(&radio_context, (const uint8_t*)"Hello World", 11);
	  HAL_Delay(500);

	  printf("Transmitting done \r\n");



	  //Verify Radio is still working
//	  GPIO_PinState busy_state = HAL_GPIO_ReadPin(radio_context.busy_port, radio_context.busy_pin);
//	  printf("Busy status: %d \r\n", busy_state);
//
//	  lr11xx_status_t status_return = lr11xx_system_get_version(&radio_context, &version);
//	  printf("Get version return: %d \r\n", status_return);
//
//	  printf("hw=0x%02X type=0x%02X fw=0x%04X\r\n \n\n", version.hw, version.type, version.fw);
//
//
//	  printf("Toggling busy pin... \r\n");
//	  HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
//	  HAL_Delay(500);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x7;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi1.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi1, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA3 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PE11 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//#ifdef __GNUC__
//#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif
//
//PUTCHAR_PROTOTYPE
//{
//  // Replace &huart2 with your specific UART handle (e.g., &huart1, &huart3)
//  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
//  return ch;
//}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
