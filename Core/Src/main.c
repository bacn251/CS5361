/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "usb_device.h"
#include "CS5361.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "usbd_audio_if.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef SPI3
#define SPI3 SPI3_BASE
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_rx;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
// Buffer cho dữ liệu từ CS5361 - đủ lớn để chứa dữ liệu stereo 24-bit
int32_t adc_buffer[AUDIO_IN_PACKET] = {0};

// Hệ số khuếch đại cho tín hiệu âm thanh
uint8_t volume_gain = 4; // Khuếch đại mặc định là 4 lần

// Biến theo dõi trạng thái
volatile uint8_t i2s_active = 0;
volatile uint8_t usb_active = 0;
volatile uint32_t last_i2s_activity = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_CRC_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Điều chỉnh âm lượng từ CS5361
 * @param gain: Hệ số khuếch đại (1-10)
 */
void CS5361_Set_Volume(uint8_t gain)
{
  if (gain > 0 && gain <= 10)
  {
    volume_gain = gain;
  }
}

/**
 * @brief Bắt đầu nhận dữ liệu từ CS5361 qua I2S DMA
 */
void I2S_TO_CS5361(void)
{
  // Đèn báo bắt đầu truyền dữ liệu
  HAL_GPIO_TogglePin(LD6_GPIO_Port, LD6_Pin);
  
  // Đánh dấu I2S đang hoạt động
  i2s_active = 1;
  last_i2s_activity = HAL_GetTick();
  
  // Bắt đầu nhận dữ liệu qua DMA
  HAL_StatusTypeDef status = HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)adc_buffer, AUDIO_IN_PACKET * 2);
  if (status != HAL_OK)
  {
    // Báo lỗi nếu không khởi tạo được DMA
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    i2s_active = 0;
  }
}

/**
 * @brief Callback khi nhận được nửa buffer DMA
 */
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == hi2s3.Instance)
  {
    // Đèn báo callback được gọi
    HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    last_i2s_activity = HAL_GetTick();
    
    // Lấy con trỏ đến USB audio buffer
    USBD_AUDIO_HandleTypeDef *haudio = (USBD_AUDIO_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (haudio != NULL && usb_active)
    {
      // Xác định phần buffer cần xử lý
      int16_t *buf_part = haudio->in_buffer;
      
      // Xử lý dữ liệu từ CS5361 (24-bit) sang USB Audio (16-bit)
      // CS5361 gửi dữ liệu theo định dạng I2S, mỗi frame gồm 2 kênh (L+R)
      // Chúng ta chỉ lấy kênh trái (Left)
      for (uint16_t i = 0; i < (AUDIO_IN_PACKET / 2); i++)
      {
        // Lấy mẫu từ kênh trái (chẵn) từ buffer DMA
        int32_t sample = adc_buffer[i*2]; // Chỉ lấy kênh trái (index chẵn)
        
        // Chuyển đổi từ 24-bit sang 16-bit và khuếch đại
        // CS5361 trả về dữ liệu 24-bit được căn lề trái trong int32_t
        buf_part[i] = (int16_t)((sample >> 8) * volume_gain);
      }
      
      // Nếu buffer đã sẵn sàng và đây là nửa đầu tiên, bắt đầu truyền USB
      if (!haudio->in_buffer_half)
      {
        haudio->in_buffer_half = 1;
        USBD_LL_Transmit(&hUsbDeviceFS, AUDIO_IN_EP, (uint8_t*)haudio->in_buffer, AUDIO_IN_PACKET);
      }
    }
  }
}

/**
 * @brief Callback khi nhận đủ buffer DMA
 */
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  if (hi2s->Instance == hi2s3.Instance)
  {
    // Đèn báo callback được gọi
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    last_i2s_activity = HAL_GetTick();
    
    // Lấy con trỏ đến USB audio buffer
    USBD_AUDIO_HandleTypeDef *haudio = (USBD_AUDIO_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (haudio != NULL && usb_active)
    {
      // Xác định phần buffer cần xử lý
      int16_t *buf_part = haudio->in_buffer + (AUDIO_IN_PACKET / 2);
      
      // Xử lý dữ liệu từ CS5361 (24-bit) sang USB Audio (16-bit)
      // Chỉ lấy kênh trái
      for (uint16_t i = 0; i < (AUDIO_IN_PACKET / 2); i++)
      {
        // Lấy mẫu từ kênh trái (chẵn) từ nửa sau của buffer DMA
        int32_t sample = adc_buffer[(i + (AUDIO_IN_PACKET / 2))*2]; // Chỉ lấy kênh trái
        
        // Chuyển đổi từ 24-bit sang 16-bit và khuếch đại
        buf_part[i] = (int16_t)((sample >> 8) * volume_gain);
      }
      
      // Nếu buffer đã sẵn sàng và đây là nửa thứ hai, bắt đầu truyền USB
      if (haudio->in_buffer_half)
      {
        haudio->in_buffer_half = 0;
        USBD_LL_Transmit(&hUsbDeviceFS, AUDIO_IN_EP, (uint8_t*)haudio->in_buffer, AUDIO_IN_PACKET);
      }
    }
  }
}

/**
 * @brief Kiểm tra USB Audio đã sẵn sàng chưa
 */
uint8_t USB_Audio_Ready(void)
{
  if (hUsbDeviceFS.pClassData != NULL)
  {
    usb_active = 1;
    return 1;
  }
  usb_active = 0;
  return 0;
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

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_CRC_Init();
  MX_USB_DEVICE_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */
  // Tắt tất cả đèn LED
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // Đèn xanh
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // Đèn xanh lá
  HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // Đèn đỏ
  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // Đèn màu xanh dương

  // Khởi động tuần tự LED để kiểm tra
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
  
  // Khởi tạo CS5361
  cs5361_init();
  HAL_Delay(50);
  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
  
  // Chờ USB khởi tạo
  uint32_t usb_timeout = HAL_GetTick() + 3000; // 3 second timeout
  while (!USB_Audio_Ready() && HAL_GetTick() < usb_timeout)
  {
    HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
    HAL_Delay(100);
  }
  
  if (USB_Audio_Ready())
  {
    // USB đã sẵn sàng, bắt đầu nhận dữ liệu từ CS5361
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
    I2S_TO_CS5361();
  }
  else
  {
    // USB chưa sẵn sàng, báo lỗi
    for (int i = 0; i < 5; i++)
    {
      HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
      HAL_Delay(100);
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Kiểm tra trạng thái USB và I2S định kỳ
    static uint32_t last_check = 0;
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - last_check > 1000) // Kiểm tra mỗi giây
    {
      last_check = current_time;
      
      // Kiểm tra USB
      if (USB_Audio_Ready())
      {
        // Hiển thị trạng thái hoạt động
        HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
        
        // Kiểm tra I2S
        if (!i2s_active || (current_time - last_i2s_activity > 500))
        {
          // I2S không hoạt động hoặc đã lâu không có dữ liệu mới
          // Khởi động lại I2S
          I2S_TO_CS5361();
        }
        
        // Kiểm tra tín hiệu âm thanh
        static uint32_t silent_count = 0;
        int32_t signal_level = 0;
        
        // Tính mức tín hiệu trung bình
        for (int i = 0; i < 10; i++)
        {
          signal_level += abs(adc_buffer[i*2]); // Chỉ kiểm tra kênh trái
        }
        signal_level /= 10;
        
        // Nếu tín hiệu quá nhỏ, tăng bộ đếm im lặng
        if (signal_level < 1000)
        {
          silent_count++;
          if (silent_count > 5)
          {
            // Tăng khuếch đại nếu tín hiệu quá nhỏ trong thời gian dài
            if (volume_gain < 10)
            {
              volume_gain++;
            }
            silent_count = 0;
            
            // Báo hiệu đã điều chỉnh khuếch đại
            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
          }
        }
        else
        {
          silent_count = 0;
          
          // Nếu tín hiệu quá lớn, giảm khuếch đại để tránh biến dạng
          if (signal_level > 20000 && volume_gain > 1)
          {
            volume_gain--;
            
            // Báo hiệu đã điều chỉnh khuếch đại
            HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET);
          }
        }
      }
      else
      {
        // USB không sẵn sàng, báo hiệu
        HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
      }
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */
}

/**
 * @brief I2S3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 6;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 249;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS5361_RST_Pin | CS5361_SA_Pin | CS5361_HPF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CS5361_MDIV_Pin | CS5361_M1_Pin | CS5361_M0_Pin | CS5361_MS_Pin | LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS5361_RST_Pin CS5361_SA_Pin CS5361_HPF_Pin */
  GPIO_InitStruct.Pin = CS5361_RST_Pin | CS5361_SA_Pin | CS5361_HPF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS5361_MDIV_Pin CS5361_M1_Pin CS5361_M0_Pin CS5361_MS_Pin
                           LD4_Pin LD3_Pin LD5_Pin LD6_Pin */
  GPIO_InitStruct.Pin = CS5361_MDIV_Pin | CS5361_M1_Pin | CS5361_M0_Pin | CS5361_MS_Pin | LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
//{
//   /* Call CS5361 transfer complete callback */
//   if (hi2s->Instance == SPI3)
//   {
//     cs5361_transfer_complete_callback();
//   }
// }
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
