/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define CS5361_RST_Pin GPIO_PIN_13
#define CS5361_RST_GPIO_Port GPIOB
#define CS5361_SA_Pin GPIO_PIN_14
#define CS5361_SA_GPIO_Port GPIOB
#define CS5361_HPF_Pin GPIO_PIN_15
#define CS5361_HPF_GPIO_Port GPIOB
#define CS5361_MDIV_Pin GPIO_PIN_8
#define CS5361_MDIV_GPIO_Port GPIOD
#define CS5361_M1_Pin GPIO_PIN_9
#define CS5361_M1_GPIO_Port GPIOD
#define CS5361_M0_Pin GPIO_PIN_10
#define CS5361_M0_GPIO_Port GPIOD
#define CS5361_MS_Pin GPIO_PIN_11
#define CS5361_MS_GPIO_Port GPIOD
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MIC_FILTER_RESULT_LENGTH 16
#define PCM_OUT_SIZE MIC_FILTER_RESULT_LENGTH
#define INTERNAL_BUFF_SIZE      64
extern int16_t audiodata[64];
extern uint32_t AudioRecInited;
extern int16_t RecBuf0[MIC_FILTER_RESULT_LENGTH];
extern int16_t RecBuf1[MIC_FILTER_RESULT_LENGTH];
extern uint8_t buffer_ready;

extern volatile uint16_t Mic_DMA_PDM_Buffer0[INTERNAL_BUFF_SIZE];
extern volatile uint16_t Mic_DMA_PDM_Buffer1[INTERNAL_BUFF_SIZE];
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
