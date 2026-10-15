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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Steps, visited in this order each time the user button is pressed.
 * First a Run-mode frequency sweep (the CPU cycle counter is printed each
 * second), then the low-power modes at 160 MHz.
 * STANDBY and SHUTDOWN reset the MCU on wake-up (use the black RESET button),
 * so they are last; after reset the sequence restarts from the beginning. */
typedef enum
{
  PM_RUN_160 = 0,
  PM_RUN_80,
  PM_RUN_40,
  PM_RUN_4,
  PM_SLEEP,
  PM_STOP0,
  PM_STOP1,
  PM_STOP2,
  PM_STOP3,
  PM_STANDBY,
  PM_SHUTDOWN,
  PM_COUNT
} PowerMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Some HAL versions only define the SLEEPENTRY name; both have the same value. */
#ifndef PWR_STOPENTRY_WFI
#define PWR_STOPENTRY_WFI PWR_SLEEPENTRY_WFI
#endif

#define BUTTON_SETTLE_MS  300U   /* ignore contact bounce after a press */

/* Mode the demo starts in after power-up/reset. Set to e.g. PM_STOP0 to jump
 * straight to the low-current modes when using the multimeter's uA range. */
#define START_MODE        PM_RUN_160
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile uint8_t button_pressed = 0;

static const char *const mode_names[PM_COUNT] =
{
  "RUN @ 160 MHz", "RUN @ 80 MHz", "RUN @ 40 MHz", "RUN @ 4 MHz",
  "SLEEP", "STOP0", "STOP1", "STOP2", "STOP3", "STANDBY", "SHUTDOWN"
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Wait_In_Mode(PowerMode_t mode);
static void Restore_After_Stop(PowerMode_t mode);
static void Reinit_COM(void);
static void Set_RunFrequency(uint32_t mhz);
static void Run_At_Frequency(uint32_t mhz);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Enable the Cortex-M33 cycle counter (counts CPU clock cycles). */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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

  /* LEDs draw far more current than the MCU does in low-power modes. */
  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_BLUE);
  BSP_LED_Off(LED_RED);

  int counter = 0;                 /* number of completed passes through the modes */
  PowerMode_t mode = START_MODE;

  printf("\r\n=== Power mode demo (SMPS, Range 1) ===\r\n");
  printf("Press the blue user button to move to the next mode.\r\n");

  while (1)
  {
	  printf("\r\n[pass %d] Entering %s mode. Press the button to leave it.\r\n",
	         counter, mode_names[mode]);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Wait_In_Mode(mode);          /* STANDBY / SHUTDOWN never return */

	  printf("Left %s mode.\r\n", mode_names[mode]);

	  mode++;
	  if (mode >= PM_COUNT)
	  {
	    mode = PM_RUN_160;
	    counter++;
	  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE9
                           PE10 PE11 PE12 PE13
                           PE14 PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 PF3
                           PF4 PF5 PF6 PF7
                           PF8 PF9 PF10 PF11
                           PF12 PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC6 PC8 PC9 PC10
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5
                           PA6 PA7 PA8 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB13 PB14 PB15
                           PB4 PB5 PB6 PB8
                           PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 PG3 PG4
                           PG5 PG6 PG7 PG8
                           PG9 PG10 PG12 PG13
                           PG14 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

/* Button interrupt: only set a flag. Printing from an ISR is slow and would
 * keep the MCU awake longer than necessary. */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    button_pressed = 1;
  }
}

/* Re-initialise the virtual COM port. The USART baud-rate divider is derived
 * from the current clock, so call this after every clock change. */
static void Reinit_COM(void)
{
  BSP_COM_DeInit(COM1);
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
}

/* Change the CPU frequency while running:
 *   160 / 80 / 40 MHz : PLL fed from MSI (48 MHz), only the PLL R divider changes
 *   4 MHz             : MSI alone (PLL off)
 * The voltage range stays at Range 1 so the sweep isolates frequency. */
static void Set_RunFrequency(uint32_t mhz)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  uint32_t pllr    = 1U;
  uint32_t latency = FLASH_LATENCY_4;
  uint32_t use_pll = 1U;

  switch (mhz)
  {
  case 160U: pllr = 1U; latency = FLASH_LATENCY_4; break;
  case 80U:  pllr = 2U; latency = FLASH_LATENCY_2; break;
  case 40U:  pllr = 4U; latency = FLASH_LATENCY_1; break;
  case 4U:   use_pll = 0U; latency = FLASH_LATENCY_0; break;
  default:   return;
  }

  if (HAL_RCC_GetHCLKFreq() == (mhz * 1000000U))
  {
    return;                        /* already there */
  }

  /* 1. Move SYSCLK onto MSI so the PLL can be safely reconfigured. */
  clk.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                | RCC_CLOCKTYPE_PCLK3;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  clk.APB3CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /* 2. Set the MSI range and the PLL (same settings as SystemClock_Config,
        except for the R divider). */
  osc.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  osc.MSIState            = RCC_MSI_ON;
  osc.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  if (use_pll != 0U)
  {
    osc.MSIClockRange      = RCC_MSIRANGE_0;          /* 48 MHz */
    osc.PLL.PLLState       = RCC_PLL_ON;
    osc.PLL.PLLSource      = RCC_PLLSOURCE_MSI;
    osc.PLL.PLLMBOOST      = RCC_PLLMBOOST_DIV4;
    osc.PLL.PLLM           = 3;
    osc.PLL.PLLN           = 10;
    osc.PLL.PLLP           = 2;
    osc.PLL.PLLQ           = 2;
    osc.PLL.PLLR           = pllr;
    osc.PLL.PLLRGE         = RCC_PLLVCIRANGE_1;
    osc.PLL.PLLFRACN       = 0;
  }
  else
  {
    osc.MSIClockRange      = RCC_MSIRANGE_4;          /* 4 MHz */
    osc.PLL.PLLState       = RCC_PLL_OFF;
  }
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    Error_Handler();
  }

  /* 3. Select the final clock source with the matching flash latency. */
  clk.SYSCLKSource = (use_pll != 0U) ? RCC_SYSCLKSOURCE_PLLCLK
                                     : RCC_SYSCLKSOURCE_MSI;
  if (HAL_RCC_ClockConfig(&clk, latency) != HAL_OK)
  {
    Error_Handler();
  }

  Reinit_COM();                    /* UART baud rate depends on the clock */
}

/* Run at the given frequency, printing how many CPU clock cycles elapse in
 * each second of real time (taken from the HAL tick, which the HAL retunes
 * after every clock change). The result should match the frequency. */
static void Run_At_Frequency(uint32_t mhz)
{
  Set_RunFrequency(mhz);

  printf("CPU running at %lu MHz (HCLK reports %lu Hz).\r\n",
         (unsigned long)mhz, (unsigned long)HAL_RCC_GetHCLKFreq());

  while (!button_pressed)
  {
    uint32_t t0 = HAL_GetTick();
    uint32_t c0 = DWT->CYCCNT;

    while (!button_pressed && ((HAL_GetTick() - t0) < 1000U))
    {
      __NOP();                     /* busy loop: this is the Run-mode workload */
    }

    if (!button_pressed)
    {
      uint32_t cycles = DWT->CYCCNT - c0;
      printf("  %lu cycles in ~1 s  (~%lu.%03lu MHz)\r\n",
             (unsigned long)cycles,
             (unsigned long)(cycles / 1000000U),
             (unsigned long)((cycles / 1000U) % 1000U));
    }
  }
}

/* After a Stop mode the clock tree falls back to the default oscillator, and
 * (especially after STOP3) peripheral registers may be lost, so re-apply the
 * configuration the rest of the program expects. */
static void Restore_After_Stop(PowerMode_t mode)
{
  SystemClock_Config();

  if (mode == PM_STOP3)
  {
    MX_GPIO_Init();
    if ((ICACHE->CR & ICACHE_CR_EN) == 0U)
    {
      MX_ICACHE_Init();
    }
    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_BLUE);
    BSP_LED_Init(LED_RED);
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  }

  Reinit_COM();
}

/* Sit in the requested power mode until the user button is pressed.
 * Interrupts are masked while checking the flag so a press that arrives just
 * before the WFI instruction cannot be missed (a pending interrupt still wakes
 * the core even when PRIMASK is set). */
static void Wait_In_Mode(PowerMode_t mode)
{
  HAL_Delay(BUTTON_SETTLE_MS);   /* let the previous press finish bouncing */
  button_pressed = 0;

  switch (mode)
  {
  case PM_RUN_160:
    Run_At_Frequency(160U);
    break;

  case PM_RUN_80:
    Run_At_Frequency(80U);
    break;

  case PM_RUN_40:
    Run_At_Frequency(40U);
    break;

  case PM_RUN_4:
    Run_At_Frequency(4U);
    Set_RunFrequency(160U);      /* back to full speed for the low-power modes */
    break;

  case PM_SLEEP:
    while (!button_pressed)
    {
      __disable_irq();
      if (!button_pressed)
      {
        HAL_SuspendTick();
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        HAL_ResumeTick();
      }
      __enable_irq();
    }
    break;

  case PM_STOP0:
  case PM_STOP1:
  case PM_STOP2:
  case PM_STOP3:
    while (!button_pressed)
    {
      __disable_irq();
      if (!button_pressed)
      {
        HAL_SuspendTick();
        if      (mode == PM_STOP0) { HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI); }
        else if (mode == PM_STOP1) { HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI); }
        else if (mode == PM_STOP2) { HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI); }
        else                       { HAL_PWREx_EnterSTOP3Mode(PWR_STOPENTRY_WFI); }
        Restore_After_Stop(mode);
        HAL_ResumeTick();
      }
      __enable_irq();
    }
    break;

  case PM_STANDBY:
    /* No wake-up pin is configured: press the black RESET button to leave. */
    HAL_PWR_EnterSTANDBYMode();
    break;

  case PM_SHUTDOWN:
    /* Lowest-power mode. Press the black RESET button to leave. */
    HAL_PWREx_EnterSHUTDOWNMode();
    break;

  default:
    break;
  }
}
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
