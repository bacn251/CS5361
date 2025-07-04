/*
 * CS5361.c
 *
 *  Created on: Apr 17, 2025
 *      Author: Bacnk
 */
#include "CS5361.h"

/* External variable declaration for I2S handle */
extern I2S_HandleTypeDef hi2s3;
/* External variable declaration for USB Device handle */

void cs5361_set_mode(cs5361_mode_t mode)
{
  switch (mode)
  {
  case CS5361_MASTER_MODE:
    HAL_GPIO_WritePin(CS5361_MS_GPIO_Port, CS5361_MS_Pin, 1);
    break;
  case CS5361_SLAVE_MODE:
    HAL_GPIO_WritePin(CS5361_MS_GPIO_Port, CS5361_MS_Pin, 0);
    break;
  default:
    break;
  }
}
void cs5361_set_speed_mode(cs5361_speed_mode_t speed)
{
  switch (speed)
  {
  case CS5361_SINGLE_SPEED:
    HAL_GPIO_WritePin(CS5361_M0_GPIO_Port, CS5361_M0_Pin, 0);
    HAL_GPIO_WritePin(CS5361_M1_GPIO_Port, CS5361_M1_Pin, 0);
    break;
  case CS5361_DOUBLE_SPEED:
    HAL_GPIO_WritePin(CS5361_M0_GPIO_Port, CS5361_M0_Pin, 1);
    HAL_GPIO_WritePin(CS5361_M1_GPIO_Port, CS5361_M1_Pin, 0);
    break;
  case CS5361_QUAD_SPEED:
    HAL_GPIO_WritePin(CS5361_M0_GPIO_Port, CS5361_M0_Pin, 0);
    HAL_GPIO_WritePin(CS5361_M1_GPIO_Port, CS5361_M1_Pin, 1);
    break;
  default:
    break;
  }
}
void cs5361_set_mdiv(cs5361_mdiv_mode_t mdiv)
{
  switch (mdiv)
  {
  case CS5361_MDIV_1:
    HAL_GPIO_WritePin(CS5361_MDIV_GPIO_Port, CS5361_MDIV_Pin, 0);
    break;
  case CS5361_MDIV_2:
    HAL_GPIO_WritePin(CS5361_MDIV_GPIO_Port, CS5361_MDIV_Pin, 1);
    break;
  default:
    break;
  }
}
void cs5361_set_hpf(cs5361_hpf_mode_t hpf)
{
  switch (hpf)
  {
  case CS5361_HPF_DISABLE:
    HAL_GPIO_WritePin(CS5361_HPF_GPIO_Port, CS5361_HPF_Pin, 1);
    break;
  case CS5361_HPF_ENABLE:
    HAL_GPIO_WritePin(CS5361_HPF_GPIO_Port, CS5361_HPF_Pin, 0);
    break;
  default:
    break;
  }
}
void cs5361_set_sa(cs5361_sa_mode_t sa)
{
  switch (sa)
  {
  case CS5361_I2S_MODE:
    HAL_GPIO_WritePin(CS5361_SA_GPIO_Port, CS5361_SA_Pin, 1);
    break;
  case CS5361_LEFT_JUSTIFIED_MODE:
    HAL_GPIO_WritePin(CS5361_SA_GPIO_Port, CS5361_SA_Pin, 0);
    break;
  default:
    break;
  }
}
void cs5361_powerdown(void)
{
  HAL_GPIO_WritePin(CS5361_RST_GPIO_Port, CS5361_RST_Pin, 0);
}
void cs5361_powerup(void)
{
  HAL_GPIO_WritePin(CS5361_RST_GPIO_Port, CS5361_RST_Pin, 1);
}
void cs5361_init(void)
{
  cs5361_set_mode(CS5361_SLAVE_MODE);
  cs5361_set_speed_mode(CS5361_QUAD_SPEED); //CS5361_SINGLE_SPEED  //CS5361_QUAD_SPEED 96khz
  cs5361_set_mdiv(CS5361_MDIV_2);
  cs5361_set_hpf(CS5361_HPF_ENABLE);
  cs5361_set_sa(CS5361_I2S_MODE);
  cs5361_powerup();
}

//static uint16_t cs5361_dma_buffer[CS5361_BUFFER_SIZE * 2];
//static uint8_t cs5361_is_running = 0;
//static uint8_t *cs5361_pcm_byte_buffer = NULL;
//static uint16_t cs5361_num_samples = 0;

/**
 * @brief  Start reading data from CS5361 using DMA
 * @param  buffer: Pointer to buffer where processed PCM byte data will be stored (3 bytes per sample)
 * @param  size: Size of the buffer in *samples*
 * @retval HAL status
 */
//HAL_StatusTypeDef cs5361_start_dma_read(uint8_t *buffer, uint16_t size)
//{
//  HAL_StatusTypeDef status = HAL_OK;
//
////    if (buffer == NULL || size == 0 || cs5361_is_running || size != CS5361_BUFFER_SIZE)
////    {
////        return HAL_ERROR; /* Ensure buffer is valid, not running, and size matches definition */
////    }
//
//    cs5361_pcm_byte_buffer = buffer;
//    cs5361_num_samples = size; /* Store size in samples */
//
//    /* Start I2S DMA reception in circular mode */
//    /* Size is size * 2 because HAL_I2S_Receive_DMA takes size in half-words (16-bit) */
//    /* We are reading 32-bit data frames (containing 24-bit audio), so each sample is 2 half-words */
//   // status = HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)cs5361_dma_buffer, size * 2);
//    HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)cs5361_dma_buffer, size * 2);
//
//
////    if (status == HAL_OK)
////    {
//      cs5361_is_running = 1;
////    }
//
////    return status;
//  }
//
//  /**
//   * @brief  Stop reading data from CS5361
//   * @param  None
//   * @retval HAL status
//   */
//  HAL_StatusTypeDef cs5361_stop_dma_read(void)
//  {
//    HAL_StatusTypeDef status = HAL_OK;
//
//    if (!cs5361_is_running)
//    {
//      return HAL_ERROR;
//    }
//
//    /* Stop I2S DMA reception */
//    status = HAL_I2S_DMAStop(&hi2s3);
//
//    if (status == HAL_OK)
//    {
//      cs5361_is_running = 0;
//      cs5361_pcm_byte_buffer = NULL;
//      cs5361_num_samples = 0;
//    }
//
//    return status;
//  }
//
//  /**
//   * @brief  Process raw I2S data from CS5361 to 24-bit PCM byte array
//   * @param  raw_data: Pointer to raw 32-bit data from I2S (cast from uint16_t buffer)
//   * @param  pcm_byte_data: Pointer to output 24-bit PCM data buffer (3 bytes per sample)
//   * @param  num_samples: Number of samples to process
//   * @retval None
//   */
//  void cs5361_process_data(uint32_t *raw_data, uint8_t *pcm_byte_data, uint16_t num_samples)
//  {
//    for (uint16_t i = 0; i < num_samples; i++)
//    {
//      /* Assuming I2S data is 32-bit frame, 24-bit data, MSB aligned */
//      /* Raw data from DMA buffer (uint16_t) needs to be combined for 32-bit access if necessary, */
//      /* but HAL_I2S_Receive_DMA with uint16_t* might handle 32-bit transfers correctly depending on config. */
//      /* Let's assume raw_data points correctly to 32-bit aligned samples for now. */
//      uint32_t raw_sample = raw_data[i];
//
//      /* Extract 3 bytes (24 bits) - Assuming data is in bits 31-8 (MSB aligned) */
//      /* Adjust byte order if necessary for USB endpoint (Little Endian is common) */
//      pcm_byte_data[i * 3 + 0] = (uint8_t)(raw_sample >> 8);  /* Least significant byte of 24-bit sample */
//      pcm_byte_data[i * 3 + 1] = (uint8_t)(raw_sample >> 16); /* Middle byte */
//      pcm_byte_data[i * 3 + 2] = (uint8_t)(raw_sample >> 24); /* Most significant byte */
//    }
//  }
//
//  /**
//   * @brief  Half transfer complete callback
//   * @param  None
//   * @retval None
//   */
//  void cs5361_half_transfer_complete_callback(void)
//  {
//    if (cs5361_is_running && cs5361_pcm_byte_buffer != NULL)
//    {
//      uint16_t samples_to_process = cs5361_num_samples / 2;
//      uint16_t bytes_to_send = samples_to_process * 3; /* 3 bytes per sample */
//
//      /* Process first half of DMA buffer */
//      /* Cast the uint16_t DMA buffer to uint32_t for processing */
//      cs5361_process_data((uint32_t *)cs5361_dma_buffer, cs5361_pcm_byte_buffer, samples_to_process);
//
//      /* Send the processed data to USB Host */
//    }
//  }
//
//  /**
//   * @brief  Transfer complete callback
//   * @param  None
//   * @retval None
//   */
//  void cs5361_transfer_complete_callback(void)
//  {
//    if (cs5361_is_running && cs5361_pcm_byte_buffer != NULL)
//    {
//      uint16_t samples_to_process = cs5361_num_samples / 2;
//      uint16_t bytes_to_send = samples_to_process * 3;                                  /* 3 bytes per sample */
//      uint8_t *buffer_offset = &cs5361_pcm_byte_buffer[bytes_to_send];                  /* Pointer to the second half of the user buffer */
//      uint32_t *dma_buffer_offset = (uint32_t *)&cs5361_dma_buffer[cs5361_num_samples]; /* Pointer to the second half of the DMA buffer (as uint32_t) */
//
//      /* Process second half of DMA buffer */
//      cs5361_process_data(dma_buffer_offset, buffer_offset, samples_to_process);
//
//      /* Send the processed data to USB Host */
//    }
//  }

  /**
   * @brief  Start the CS5361 ADC with default configuration
   * @param  None
   * @retval None
   */
  void cs5361_start(void)
  {
    /* Initialize the CS5361 with default configuration */
    cs5361_init();

    /* Allow some time for the CS5361 to stabilize after power-up */
    HAL_Delay(10);
  }
