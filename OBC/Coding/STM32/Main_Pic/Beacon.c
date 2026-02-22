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
#include "stdio.h"
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- Requested pin mapping --- */

#define BC_EN_Pin         GPIO_PIN_0          // PD0
#define BC_ADC_CHANNEL    ADC_CHANNEL_0       // PA0 -> ADC1_IN0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* --- Beacon globals (PIC-like) --- */
static uint16_t BC_TEMP_RAW      = 0;
static float    BC_TEMP_C        = 0.0f;
static float    BC_INITIALTEMP_C = 0.0f;
static float    BC_MAXTEMP_C     = 0.0f;

static uint8_t  BC_ATTEMPT_FLAG = 0;  // 0..4
static uint8_t  ANT_DEP_STATUS  = 0;  // 0=not deployed, 1=deployed
static float    BC_LAST_DELTA_C = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* ---------------- UART helper ---------------- */
static void PC_Printf(const char *fmt, ...)
{
  char buf[220];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)strlen(buf), 100);
}

static void UartSend(const char *s)
{
  // Large timeout for Proteus + long strings
  (void)HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 1000);
  // tiny gap helps some Proteus terminals
  HAL_Delay(2);
}

static void PrintMenu(void)
{
  UartSend("\r\n==== BEACON DEMO (STM32F401VE / Proteus) ====\r\n");
  UartSend("1) Read temp once\r\n");
  UartSend("2) Stream temp (1 Hz, 10 samples)\r\n");
  UartSend("3) BC_OPERATION (ON 30s + delta check)\r\n");
  UartSend("4) Beacon ON\r\n");
  UartSend("5) Beacon OFF\r\n");
  UartSend("6) Beacon OFF\r\n");
  UartSend("7) MAKE_BC_FLAG_1\r\n");
  UartSend("8) MAKE_BC_FLAG_2\r\n");
  UartSend("9) MAKE_BC_FLAG_3\r\n");
  UartSend("0) MAKE_BC_FLAG_4\r\n");
  UartSend("a) Antenna_Deploy\r\n");
  UartSend("c) CLEAR_BC_FLAG\r\n");
  UartSend("m) Menu\r\n");
  UartSend("=============================================\r\n");
}

static int UART_ReadChar(uint8_t *ch)
{
  return (HAL_UART_Receive(&huart2, ch, 1, 0) == HAL_OK);
}

/* ---------------- Beacon functions (condensed) ---------------- */

/* Ensure ADC1 configured for channel PA0 (IN0). CubeMX does most, but this keeps it robust. */
static void BC_SETUP(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel      = BC_ADC_CHANNEL;
  sConfig.Rank         = 1;
  /* Sampling time: increase if your source impedance is high */
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  (void)HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static uint16_t BC_ReadAdcRaw(void)
{
  BC_SETUP();

  (void)HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 20) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return 0;
  }

  uint16_t raw = (uint16_t)(HAL_ADC_GetValue(&hadc1) & 0x0FFF); // 12-bit
  (void)HAL_ADC_Stop(&hadc1);
  return raw;
}

/* PIC-like DEMO conversion: TempC = (V*100) - 50, where V is sensor voltage */
static float BC_RawToTempC(uint16_t raw)
{
  const float VDD = 3.30f;
  float v = ((float)raw / 4096.0f) * VDD;
  return (v * 100.0f) - 50.0f;
}

static void MEASURE_BC_TEMP(void)
{
  BC_TEMP_RAW = BC_ReadAdcRaw();
  BC_TEMP_C   = BC_RawToTempC(BC_TEMP_RAW);
}

static void BC_READ_TO_PC(void)
{
  MEASURE_BC_TEMP();
  PC_Printf("RAW=%u  Temp=%6.2f C\r\n", BC_TEMP_RAW, BC_TEMP_C);
}

static void CHECK_BC_TEMP(void)
{
  MEASURE_BC_TEMP();
  if (BC_TEMP_C > BC_MAXTEMP_C) BC_MAXTEMP_C = BC_TEMP_C;
  PC_Printf("ADC=%4u  Temp=%6.2f C\r\n", BC_TEMP_RAW, BC_TEMP_C);
}

static void Turn_ON_BC(void)
{
  HAL_GPIO_WritePin(BC_EN_GPIO_Port, BC_EN_Pin, GPIO_PIN_SET);
  PC_Printf("BC: ON  (PD0=1)\r\n");
}

static void Turn_OFF_BC(void)
{
  HAL_GPIO_WritePin(BC_EN_GPIO_Port, BC_EN_Pin, GPIO_PIN_RESET);
  PC_Printf("BC: OFF (PD0=0)\r\n");
}

static void BC_OPERATION(void)
{
  PC_Printf("\r\n--- BC_OPERATION START ---\r\n");

  /* baseline */
  BC_MAXTEMP_C = -1000.0f;
  CHECK_BC_TEMP();
  BC_INITIALTEMP_C = BC_TEMP_C;
  PC_Printf("Initial Temp: %6.2f C\r\n", BC_INITIALTEMP_C);

  /* ON + monitor 30s (sample each 1s) */
  Turn_ON_BC();

  uint32_t t0 = HAL_GetTick();
  uint32_t last = t0;

  while ((HAL_GetTick() - t0) < 30000UL)
  {
    if ((HAL_GetTick() - last) >= 1000UL)
    {
      last += 1000UL;
      CHECK_BC_TEMP();
    }
    HAL_Delay(5);
  }

  Turn_OFF_BC();

  float delta = BC_MAXTEMP_C - BC_INITIALTEMP_C;
  BC_LAST_DELTA_C = delta;

  PC_Printf("Max Temp:     %6.2f C\r\n", BC_MAXTEMP_C);
  PC_Printf("Delta:        %6.2f C\r\n", delta);

  if (delta > 5.0f)
    PC_Printf("Result: PASS (delta > 5C)\r\n");
  else
    PC_Printf("Result: FAIL (delta <= 5C)\r\n");

  PC_Printf("--- BC_OPERATION END ---\r\n\r\n");
}

static void CLEAR_BC_FLAG(void)
{
  BC_ATTEMPT_FLAG = 0;
  PC_Printf("\r\nBC_ATTEMPT_FLAG cleared -> %u\r\n", BC_ATTEMPT_FLAG);
}

static void MAKE_BC_FLAG_1(void)
{
  BC_ATTEMPT_FLAG = 1;
  PC_Printf("\r\nBC_ATTEMPT_FLAG -> 1\r\n");
}

static void MAKE_BC_FLAG_2(void)
{
  BC_ATTEMPT_FLAG = 2;
  PC_Printf("\r\nBC_ATTEMPT_FLAG -> 2\r\n");
}

static void MAKE_BC_FLAG_3(void)
{
  BC_ATTEMPT_FLAG = 3;
  PC_Printf("\r\nBC_ATTEMPT_FLAG -> 3\r\n");
}

static void MAKE_BC_FLAG_4(void)
{
  BC_ATTEMPT_FLAG = 4;
  PC_Printf("\r\nBC_ATTEMPT_FLAG -> 4\r\n");
}

static void Antenna_Deploy(void)
{
  PC_Printf("\r\n=== Antenna_Deploy() ===\r\n");

  if (ANT_DEP_STATUS)
  {
    PC_Printf("Already deployed (ANT_DEP_STATUS=1). No action.\r\n");
    return;
  }

  if (BC_ATTEMPT_FLAG >= 4)
  {
    PC_Printf("Max attempts reached (BC_ATTEMPT_FLAG=%u). Aborting.\r\n", BC_ATTEMPT_FLAG);
    return;
  }

  // Mark that we are attempting (PIC often increments / tracks attempts)
  BC_ATTEMPT_FLAG++;
  PC_Printf("Attempt #%u\r\n", BC_ATTEMPT_FLAG);

  // Run the beacon heating/verification routine
  BC_OPERATION();

  // Decide deployment success based on delta
  if (BC_LAST_DELTA_C > 5.0f)
  {
    ANT_DEP_STATUS = 1;
    PC_Printf("DEPLOY SUCCESS: delta=%6.2f C -> ANT_DEP_STATUS=1\r\n", BC_LAST_DELTA_C);
  }
  else
  {
    PC_Printf("DEPLOY FAIL: delta=%6.2f C -> ANT_DEP_STATUS=0\r\n", BC_LAST_DELTA_C);
  }
}


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

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  Turn_OFF_BC();
  PrintMenu();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint8_t ch;
	      if (UART_ReadChar(&ch))
	      {
	        switch (ch)
	        {
	          case '1':
	            BC_READ_TO_PC();
	            break;

	          case '2':
	            for (int i = 0; i < 10; i++)
	            {
	              CHECK_BC_TEMP();
	              HAL_Delay(1000);
	            }
	            break;

	          case '3':
	            BC_OPERATION();
	            break;

	          case '4':
	            Turn_ON_BC();
	            break;

	          case '5':
	            Turn_OFF_BC();
	            break;

	          case 'm':
	          case 'M':
	            PrintMenu();
	            break;

	          case 'c':
	        	  CLEAR_BC_FLAG();
	        	  break;
	          case 'a':
	        	  Antenna_Deploy();
	        	  break;
	          case '7':
	        	  MAKE_BC_FLAG_1();
	        	  break;
	          case '8':
	        	  MAKE_BC_FLAG_2();
	        	  break;
	          case '9':
	        	  MAKE_BC_FLAG_3();
	        	  break;
	          case '0':
	        	  MAKE_BC_FLAG_4();
	        	  break;

	          default:
	            PC_Printf("Unknown: '%c' (press 'm' for menu)\r\n", ch);
	            break;
	        }
	      }

	      HAL_Delay(5);


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BC_EN_GPIO_Port, BC_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BC_EN_Pin */
  GPIO_InitStruct.Pin = BC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BC_EN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
