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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "usb_device.h"
#include "CS5361.h"
#include "usbd_cdc.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_rx;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* Definitions for init */
osThreadId_t initHandle;
const osThreadAttr_t init_attributes = {
    .name = "init",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityRealtime1,
};
/* Definitions for audioprocess */
osThreadId_t audioprocessHandle;
const osThreadAttr_t audioprocess_attributes = {
    .name = "audioprocess",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for usbtransmit */
osThreadId_t usbtransmitHandle;
const osThreadAttr_t usbtransmit_attributes = {
    .name = "usbtransmit",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal7,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
    .name = "myQueue01"};
/* Definitions for usbTransmitEvent */
osEventFlagsId_t usbTransmitEventHandle;
const osEventFlagsAttr_t usbTransmitEvent_attributes = {
    .name = "usbTransmitEvent"};
/* USER CODE BEGIN PV */
#define USB_BUFFER_SIZE 2048
uint8_t usbTxBuffer[USB_BUFFER_SIZE];
volatile uint8_t usbTxReady = 1;
volatile uint32_t current_total_size = 0;

// init rtos
#define PACKET_START_MARKER1 0xAA
#define PACKET_START_MARKER2 0x55
#define PACKET_HEADER_SIZE 4
// Giảm kích thước buffer để tránh tràn
#define AUDIO_BUFFER_SIZE 48 // Giảm từ 128 xuống 64
#define AUDIO_CHANNELS 2     // left and right (2), left or right (1)
#define SAMPLE_RATE 48000    // 48kHz sampling rate
volatile uint8_t buffer_ready_flag = 0;
volatile uint8_t current_buffer = 0; // 0 = ping, 1 = pong

// init rtos
#define PACKET_START_MARKER1 0xAA
#define PACKET_START_MARKER2 0x55
#define PACKET_HEADER_SIZE 4
typedef struct
{
  uint16_t *data;
  uint16_t size;
} AudioBuffer_t;
uint16_t audio_buffer[AUDIO_BUFFER_SIZE * 4];
#define HALF_BUFFER_SIZE (AUDIO_BUFFER_SIZE * 2)
uint16_t *audio_buffer_ping = &audio_buffer[0];
uint16_t *audio_buffer_pong = &audio_buffer[HALF_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_CRC_Init(void);
static void MX_I2S3_Init(void);
static void MX_USART2_UART_Init(void);
void InitTask(void *argument);
void AudioProcessTask(void *argument);
void UsbTransmitTask(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_CRC_Init();
  MX_I2S3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew(4, sizeof(AudioBuffer_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of init */
  initHandle = osThreadNew(InitTask, NULL, &init_attributes);

  /* creation of audioprocess */
  audioprocessHandle = osThreadNew(AudioProcessTask, NULL, &audioprocess_attributes);

  /* creation of usbtransmit */
  usbtransmitHandle = osThreadNew(UsbTransmitTask, NULL, &usbtransmit_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of usbTransmitEvent */
  usbTransmitEventHandle = osEventFlagsNew(&usbTransmitEvent_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
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
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
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
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1000);
  return ch;
}
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
  AudioBuffer_t queueItem;
  queueItem.data = audio_buffer_ping;
  queueItem.size = HALF_BUFFER_SIZE;
 // printf("HAL_I2S_RxHalfCpltCallback\n");
  osMessageQueuePut(myQueue01Handle, &queueItem, 0, 0);
}
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s)
{
  AudioBuffer_t queueItem;
  queueItem.data = audio_buffer_pong;
  queueItem.size = HALF_BUFFER_SIZE;
 // printf("HAL_I2S_RxCpltCallback\n");
  osMessageQueuePut(myQueue01Handle, &queueItem, 0, 0);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_InitTask */
/**
 * @brief  Function implementing the init thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_InitTask */
void InitTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET); // èn xanh
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET); // èn xanh lá
    HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET); // èn đ
    HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, GPIO_PIN_RESET); // èn màu xanh dương
    osDelay(200);
    cs5361_init();
    osDelay(1);
    HAL_I2S_Receive_DMA(&hi2s3, audio_buffer, AUDIO_BUFFER_SIZE * 4);
    osThreadId_t myTaskId = osThreadGetId();
    uint32_t stack_free = osThreadGetStackSpace(myTaskId);
    printf("Free stack: %lu bytes\r\n", stack_free);
    osThreadTerminate(osThreadGetId());
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_AudioProcessTask */
/**
 * @brief Function implementing the audioprocess thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_AudioProcessTask */
void AudioProcessTask(void *argument)
{
  /* USER CODE BEGIN AudioProcessTask */
  osStatus_t status;
  AudioBuffer_t bufferItem;
  /* Infinite loop */
  for (;;)
  {
    status = osMessageQueueGet(myQueue01Handle, &bufferItem, NULL, osWaitForever);
    if (status == osOK)
    {
      printf("DMMTask\n");
      osThreadFlagsSet(usbTransmitEventHandle, 0x01);
    }
    else
    {
      printf("osMessageQueueGet failed with status: %d\n", status);
    }
  }

  //   // Kiểm tra xem bufferItem có dữ liệu hay không
  //   if (osMessageQueueGet(myQueue01Handle, &bufferItem, NULL, 0U) == osOK)
  //   {
  //     uint16_t *buffer = bufferItem.data;
  //     uint16_t size = bufferItem.size;
  //     uint16_t sample_count = size;
  //     uint16_t offset_idx = 0;
  //     uint32_t total_size = sample_count * 3 + PACKET_HEADER_SIZE;
  //     current_total_size = total_size;
  //     usbTxBuffer[0] = 0xAA;
  //     usbTxBuffer[1] = 0x55;
  //     usbTxBuffer[2] = (sample_count >> 8) & 0xFF;
  //     usbTxBuffer[3] = sample_count & 0xFF;
  //     for (uint16_t i = 0; i < sample_count * 2; i += 4)
  //     {
  //       uint16_t left_low = buffer[i];      // LSB
  //       uint16_t left_high = buffer[i + 1]; // MSB
  //       int32_t left_24bit = ((int32_t)left_high << 16) | left_low;
  //       left_24bit = (left_24bit << 8) >> 8; // Sign extend

  //       uint32_t offset = PACKET_HEADER_SIZE + (offset_idx * 3);
  //       usbTxBuffer[offset] = left_24bit & 0xFF;
  //       usbTxBuffer[offset + 1] = (left_24bit >> 8) & 0xFF;
  //       usbTxBuffer[offset + 2] = (left_24bit >> 16) & 0xFF;

  //       offset_idx++;
  //     }
  //     osThreadFlagsSet(usbTransmitEventHandle, 0x01);
  //   }
  // }
  /* USER CODE END AudioProcessTask */
}

/* USER CODE BEGIN Header_UsbTransmitTask */
/**
 * @brief Function implementing the usbtransmit thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UsbTransmitTask */
void UsbTransmitTask(void *argument)
{
  /* USER CODE BEGIN UsbTransmitTask */
  /* Infinite loop */
  for (;;)
  {
   // osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
    // if (usbTxReady)
    // {
    //   // Gửi qua USB CDC với đúng kích thước đã tính toán
    //   CDC_Transmit_FS(usbTxBuffer, current_total_size);
    //   usbTxReady = 0;

  }
  /* USER CODE END UsbTransmitTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM3 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
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
