/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_DIO_Pin GPIO_PIN_13
#define LED_DIO_GPIO_Port GPIOC
#define RTC_CLK_Pin GPIO_PIN_14
#define RTC_CLK_GPIO_Port GPIOC
#define SPI_MISS_SDI_Pin GPIO_PIN_1
#define SPI_MISS_SDI_GPIO_Port GPIOC
#define SPI_MISS_SDO_Pin GPIO_PIN_2
#define SPI_MISS_SDO_GPIO_Port GPIOC
#define SPI_MISS_CS_Pin GPIO_PIN_3
#define SPI_MISS_CS_GPIO_Port GPIOC
#define UART_EOBC2DEB_OUT_Pin GPIO_PIN_0
#define UART_EOBC2DEB_OUT_GPIO_Port GPIOA
#define UART_DEB2EOBC_BUFIN_Pin GPIO_PIN_1
#define UART_DEB2EOBC_BUFIN_GPIO_Port GPIOA
#define SPI_MF_CS_Pin GPIO_PIN_4
#define SPI_MF_CS_GPIO_Port GPIOA
#define SPI_MF_SCK_Pin GPIO_PIN_5
#define SPI_MF_SCK_GPIO_Port GPIOA
#define SPI_MF_SDI_Pin GPIO_PIN_6
#define SPI_MF_SDI_GPIO_Port GPIOA
#define SPI_MF_SD0_Pin GPIO_PIN_7
#define SPI_MF_SD0_GPIO_Port GPIOA
#define ANALOG_ANT_DEP_Pin GPIO_PIN_4
#define ANALOG_ANT_DEP_GPIO_Port GPIOC
#define GPIO_AIEO_STAT_Pin GPIO_PIN_5
#define GPIO_AIEO_STAT_GPIO_Port GPIOC
#define ANALOG_DIO4_TRX_TEMP_Pin GPIO_PIN_0
#define ANALOG_DIO4_TRX_TEMP_GPIO_Port GPIOB
#define ANALOG_DIO5_TRX_RSS_Pin GPIO_PIN_1
#define ANALOG_DIO5_TRX_RSS_GPIO_Port GPIOB
#define SPI_SDI_CPLD_EOBC_Pin GPIO_PIN_2
#define SPI_SDI_CPLD_EOBC_GPIO_Port GPIOB
#define SPI_MISS_SCK_Pin GPIO_PIN_10
#define SPI_MISS_SCK_GPIO_Port GPIOB
#define DIO3_CP2TRX_CWKEY_Pin GPIO_PIN_13
#define DIO3_CP2TRX_CWKEY_GPIO_Port GPIOB
#define EOBC_DIO0_Pin GPIO_PIN_14
#define EOBC_DIO0_GPIO_Port GPIOB
#define LED_STM32_Pin GPIO_PIN_13
#define LED_STM32_GPIO_Port GPIOD
#define UART_STM2RP_Pin GPIO_PIN_6
#define UART_STM2RP_GPIO_Port GPIOC
#define UART_RP2STM_Pin GPIO_PIN_7
#define UART_RP2STM_GPIO_Port GPIOC
#define UART_CPLD2EOBC_BUFIN_Pin GPIO_PIN_9
#define UART_CPLD2EOBC_BUFIN_GPIO_Port GPIOA
#define UART_EOBC2CPLD_Pin GPIO_PIN_10
#define UART_EOBC2CPLD_GPIO_Port GPIOA
#define DEV_SWDIO_EOBC_Pin GPIO_PIN_13
#define DEV_SWDIO_EOBC_GPIO_Port GPIOA
#define DEV_SWCLK_EOBC_Pin GPIO_PIN_14
#define DEV_SWCLK_EOBC_GPIO_Port GPIOA
#define UART_EOBC2TRX_OUT_Pin GPIO_PIN_10
#define UART_EOBC2TRX_OUT_GPIO_Port GPIOC
#define UART_TRX2EOBC_BUFIN_Pin GPIO_PIN_11
#define UART_TRX2EOBC_BUFIN_GPIO_Port GPIOC
#define CPLD_CTRL1_Pin GPIO_PIN_1
#define CPLD_CTRL1_GPIO_Port GPIOD
#define CPLD_CTRL2_Pin GPIO_PIN_2
#define CPLD_CTRL2_GPIO_Port GPIOD
#define CPLD_CTRL3_Pin GPIO_PIN_3
#define CPLD_CTRL3_GPIO_Port GPIOD
#define SPI_SCK_CPLD_EOBC_Pin GPIO_PIN_3
#define SPI_SCK_CPLD_EOBC_GPIO_Port GPIOB
#define SPI_SDO_CPLD_EOBC_Pin GPIO_PIN_4
#define SPI_SDO_CPLD_EOBC_GPIO_Port GPIOB
#define SPI_CS_CPLD_EOBC_Pin GPIO_PIN_5
#define SPI_CS_CPLD_EOBC_GPIO_Port GPIOB
#define DIO_ANT_DEP1_Pin GPIO_PIN_6
#define DIO_ANT_DEP1_GPIO_Port GPIOB
#define DIO_ANT_DEP2_Pin GPIO_PIN_7
#define DIO_ANT_DEP2_GPIO_Port GPIOB
#define DIO1_CP_TRX_ONOFF_Pin GPIO_PIN_8
#define DIO1_CP_TRX_ONOFF_GPIO_Port GPIOB
#define DIO2_CP_TRX_CW_Pin GPIO_PIN_9
#define DIO2_CP_TRX_CW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
