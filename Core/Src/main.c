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
#include <string.h>
#include <stdlib.h>
#include "radio.h"   // our LR1121 wrapper. everything radio related goes through it
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * Which job this board does.
 * Both boards can run the exact same build. BOARD_ROLE is only the role it
 * starts in. Press the blue USER button at any time to swap TX <-> RX.
 */
#define ROLE_TX         0
#define ROLE_RX         1
#define BOARD_ROLE      ROLE_TX

#define TX_PERIOD_MS    1000   // TX board sends one packet this often
#define INIT_RETRY_MS   2000   // if the radio fails to start, retry this often
#define TX_FAIL_LIMIT   3      // this many failed sends in a row -> reset and re-init the radio
#define RX_ERR_LIMIT    3      // this many RX errors in a row -> reset and re-init the radio
#define BUTTON_HOLD_MAX 2000   // give up waiting for the button release after this long
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
// Set by the button interrupt, cleared by the main loop. volatile because the
// interrupt changes it behind the main loop's back.
static volatile uint8_t button_pressed = 0;
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
/*
 * Program overview
 * ----------------
 * Two Nucleo-U575 boards, each with an LR1121 dev board, make a one way GFSK link.
 *  TX role  sends "PING <n>" once a second, n counting up from 0.
 *  RX role  listens continuously and prints each packet with its RSSI, plus
 *           running counts of good packets, CRC errors and missed sequence numbers.
 *
 * LEDs (so you can tell what is going on without a terminal)
 *  green  toggles on every packet sent
 *  blue   toggles on every good packet received
 *  red    steady on = the last send failed or the last packet had a CRC error.
 *         goes off again at the next good send or packet
 *  red    blinking fast = radio not responding, retrying init
 *  all 3  steady on = fatal MCU error (Error_Handler or a crash)
 *
 * Serial output goes out the ST-LINK USB port (COM1 = USART1) at 115200 8N1.
 */

/*
 * LR1121 dev board wiring (Michael's bench setup, verified on TX)
 *   SCK  = PA5    MISO = PA6    MOSI = PA7   (SPI1)
 *   NSS  = PF13   RESET = PA3
 *   BUSY = PE13   DIO9 (IRQ) = PE11
 * If any of these move, change them here and in the .ioc.
 */
static const lr11xx_ctx_t radio_ctx = {
    .spi        = &hspi1,
    .nss_port   = GPIOF, .nss_pin   = GPIO_PIN_13,
    .reset_port = GPIOA, .reset_pin = GPIO_PIN_3,
    .busy_port  = GPIOE, .busy_pin  = GPIO_PIN_13,
    .irq_port   = GPIOE, .irq_pin   = GPIO_PIN_11,
};

// The BSP calls this from the button's EXTI13 interrupt.
// Keep interrupt code short. Just raise the flag and let the main loop act on it.
void BSP_PB_Callback(Button_TypeDef Button)
{
    if (Button == BUTTON_USER) button_pressed = 1;
}

// Put the radio in the right mode for this role and print a banner.
// Returns 0 on success, -1 if the radio didn't respond.
static int enter_role(int role)
{
    if (radio_standby(&radio_ctx) != 0) return -1;

    if (role == ROLE_RX) {
        if (radio_start_rx(&radio_ctx) != 0) return -1;
        printf("\r\n=== ROLE: RX (listening on %lu Hz) ===\r\n", (unsigned long)RADIO_FREQ_HZ);
    } else {
        printf("\r\n=== ROLE: TX (one packet every %d ms) ===\r\n", TX_PERIOD_MS);
    }
    return 0;
}

// Print a payload as text. Anything that isn't a printable character shows as '.'
// so binary junk can't mess up the terminal.
static void print_payload(const uint8_t* buf, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) putchar((buf[i] >= 32 && buf[i] < 127) ? buf[i] : '.');
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

  printf("\r\nU575 + LR1121 GFSK link test\r\n");

  int role = BOARD_ROLE;
  bool radio_ok = (radio_init(&radio_ctx) == 0) && (enter_role(role) == 0);

  uint8_t  buf[RADIO_MAX_PAYLOAD + 1];   // +1 so a received payload can be null terminated
  uint32_t last_tx   = HAL_GetTick() - TX_PERIOD_MS;   // tick of the last send. starts one period back so the first PING goes right away
  uint32_t last_init = HAL_GetTick();    // tick of the last init attempt
  uint32_t last_blink = 0;               // for the fast red blink while the radio is down

  // TX bookkeeping
  uint32_t tx_seq = 0;                   // sequence number for the next PING
  uint32_t tx_fail = 0;                  // total failed sends
  uint32_t tx_fail_streak = 0;           // failed sends in a row. too many -> re-init

  // RX bookkeeping. "missed" counts gaps in the PING sequence numbers, so it is every
  // PING that never arrived intact. a PING that failed its CRC is counted there too
  uint32_t rx_good = 0, rx_crc = 0, rx_missed = 0, rx_last_seq = 0;
  bool     rx_have_seq = false;          // false until the first PING arrives
  uint32_t rx_err_streak = 0;            // RX errors in a row. too many -> re-init

  while (1)
  {
    // ---- recovery: radio not up (bad wiring, no power, or it stopped answering) ----
    // Fast red blink and a fresh reset + init every INIT_RETRY_MS until it comes back.
    if (!radio_ok) {
      if (HAL_GetTick() - last_blink >= 125) { last_blink = HAL_GetTick(); BSP_LED_Toggle(LED_RED); }
      if (HAL_GetTick() - last_init >= INIT_RETRY_MS) {
        last_init = HAL_GetTick();
        printf("retrying radio init...\r\n");
        radio_ok = (radio_init(&radio_ctx) == 0) && (enter_role(role) == 0);
        if (radio_ok) {
          BSP_LED_Off(LED_RED);
          tx_fail_streak = 0;
          rx_err_streak = 0;
        }
      }
      continue;
    }

    // ---- button: swap TX <-> RX ----
    if (button_pressed) {
      // The interrupt fires on the press. Wait for the release, then let the
      // contacts settle, so bounce on the release can't count as a second press.
      uint32_t t0 = HAL_GetTick();
      while (BSP_PB_GetState(BUTTON_USER) && (HAL_GetTick() - t0 < BUTTON_HOLD_MAX)) {}
      HAL_Delay(50);
      button_pressed = 0;

      role = (role == ROLE_TX) ? ROLE_RX : ROLE_TX;
      tx_fail_streak = 0;
      rx_err_streak = 0;
      BSP_LED_Off(LED_GREEN); BSP_LED_Off(LED_BLUE); BSP_LED_Off(LED_RED);
      if (enter_role(role) != 0) {
        printf("radio didn't respond while switching roles\r\n");
        radio_ok = false;
        continue;
      }
    }

    // ---- TX role ----
    if (role == ROLE_TX) {
      if (HAL_GetTick() - last_tx >= TX_PERIOD_MS) {
        last_tx = HAL_GetTick();
        int n = snprintf((char*)buf, sizeof(buf), "PING %lu", (unsigned long)tx_seq);

        if (radio_send(&radio_ctx, buf, (uint8_t)n) == 0) {
          tx_fail_streak = 0;
          BSP_LED_Toggle(LED_GREEN);
          BSP_LED_Off(LED_RED);
          printf("TX #%lu \"%s\"\r\n", (unsigned long)tx_seq, (char*)buf);
        } else {
          tx_fail++;
          tx_fail_streak++;
          BSP_LED_On(LED_RED);
          printf("TX #%lu FAILED (total fails: %lu)\r\n", (unsigned long)tx_seq, (unsigned long)tx_fail);
          if (tx_fail_streak >= TX_FAIL_LIMIT) {
            printf("%d fails in a row, resetting the radio\r\n", TX_FAIL_LIMIT);
            radio_ok = false;
          }
        }
        tx_seq++;
      }
    }

    // ---- RX role ----
    else {
      radio_rx_info_t info = { 0 };
      // pass RADIO_MAX_PAYLOAD, not sizeof(buf), to keep the last byte free for the '\0'
      radio_rx_result_t r = radio_poll_rx(&radio_ctx, buf, RADIO_MAX_PAYLOAD, &info);

      if (r != RADIO_RX_ERROR && r != RADIO_RX_NONE) rx_err_streak = 0;   // the radio is answering

      if (r == RADIO_RX_OK) {
        rx_good++;
        BSP_LED_Toggle(LED_BLUE);
        BSP_LED_Off(LED_RED);
        buf[info.len] = 0;

        // If it's one of our PINGs, use the sequence number to spot dropped packets.
        // If the TX board resets, its count goes back to 0. That just restarts the tracking.
        if (info.len > 5 && memcmp(buf, "PING ", 5) == 0) {
          uint32_t seq = strtoul((char*)buf + 5, NULL, 10);
          if (rx_have_seq && seq > rx_last_seq + 1) rx_missed += seq - rx_last_seq - 1;
          rx_last_seq = seq;
          rx_have_seq = true;
        }

        printf("RX \"");
        print_payload(buf, info.len);
        printf("\" len=%u rssi=%d dBm | good=%lu crc_err=%lu missed=%lu\r\n",
               info.len, info.rssi_dbm, (unsigned long)rx_good, (unsigned long)rx_crc, (unsigned long)rx_missed);
      }
      else if (r == RADIO_RX_CRC_ERROR) {
        // heard something but it was corrupted. the radio is still listening
        rx_crc++;
        BSP_LED_On(LED_RED);
        printf("RX CRC error rssi=%d dBm (crc_err=%lu)\r\n", info.rssi_dbm, (unsigned long)rx_crc);
      }
      else if (r == RADIO_RX_ERROR) {
        // radio dropped out of RX or SPI failed. restart RX. if it keeps happening,
        // or the restart itself fails, do a full reset + re-init (which re-checks SPI)
        rx_err_streak++;
        printf("RX error, restarting RX\r\n");
        if (radio_start_rx(&radio_ctx) != 0 || rx_err_streak >= RX_ERR_LIMIT) {
          printf("RX keeps failing, resetting the radio\r\n");
          radio_ok = false;
        }
      }
      // RADIO_RX_NONE: nothing new, loop around
    }
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
  // Keep the SPI pins driven while the peripheral is disabled.
  // HAL turns SPI off after every transfer. With this left disabled, SCK and MOSI
  // stop being driven in the gap between two HAL calls made while NSS is still low
  // (command then data), and a glitch on SCK could clock a stray bit into the radio.
  // This is the "Master Keep IO State" setting in CubeMX. Set here so a code
  // regenerate can't undo it.
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
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
  // Solid red, green and blue together = the MCU itself hit a fatal error
  // (clock, SPI or UART init failed, or a HardFault). The radio-only error blinks red instead.
  // If this runs before BSP_LED_Init the pins aren't outputs yet and nothing lights up.
  BSP_LED_On(LED_RED);
  BSP_LED_On(LED_GREEN);
  BSP_LED_On(LED_BLUE);
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
