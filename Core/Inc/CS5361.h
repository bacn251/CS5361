/*
 * CS5361.h
 *
 *  Created on: Apr 17, 2025
 *      Author: Bacnk
 */

#ifndef INC_CS5361_H_
#define INC_CS5361_H_
#include "main.h"

/* Buffer size for CS5361 DMA transfers */
#define CS5361_BUFFER_SIZE   64  /* Number of samples in DMA buffer */

typedef enum
{
 CS5361_MASTER_MODE,
 CS5361_SLAVE_MODE
}cs5361_mode_t;

typedef enum {
   CS5361_SINGLE_SPEED,
   CS5361_DOUBLE_SPEED,
   CS5361_QUAD_SPEED
} cs5361_speed_mode_t;

typedef enum {
   CS5361_MDIV_1,
   CS5361_MDIV_2
} cs5361_mdiv_mode_t;
typedef enum {
   CS5361_HPF_DISABLE,
   CS5361_HPF_ENABLE
} cs5361_hpf_mode_t;

typedef enum {
   CS5361_I2S_MODE,
   CS5361_LEFT_JUSTIFIED_MODE
} cs5361_sa_mode_t;

void cs5361_set_mode(cs5361_mode_t mode);
void cs5361_set_speed_mode(cs5361_speed_mode_t speed);
void cs5361_set_mdiv(cs5361_mdiv_mode_t mdiv);
void cs5361_set_hpf(cs5361_hpf_mode_t hpf);
void cs5361_set_sa(cs5361_sa_mode_t sa);
void cs5361_powerdown(void);
void cs5361_powerup(void);
void cs5361_init(void);
void cs5361_start(void);

/* Data acquisition functions */
/* buffer: Pointer to buffer where processed PCM byte data will be stored (3 bytes per sample) */
/* size: Size of the buffer in *samples* */
HAL_StatusTypeDef cs5361_start_dma_read(uint8_t* buffer, uint16_t size);
HAL_StatusTypeDef cs5361_stop_dma_read(void);
/* raw_data: Pointer to raw 32-bit data from I2S */
/* pcm_byte_data: Pointer to output 24-bit PCM data buffer (3 bytes per sample) */
/* num_samples: Number of samples to process */
void cs5361_process_data(uint32_t* raw_data, uint8_t* pcm_byte_data, uint16_t num_samples);

/* Callback functions (to be called from HAL_I2S_RxHalfCpltCallback and HAL_I2S_RxCpltCallback) */
void cs5361_half_transfer_complete_callback(void);
void cs5361_transfer_complete_callback(void);

#endif /* INC_CS5361_H_ */
