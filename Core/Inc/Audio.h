///*
// * Audio.h
// *
// *  Created on: Dec 14, 2024
// *      Author: Bacnk
// */
//
//#ifndef INC_AUDIO_H_
//#define INC_AUDIO_H_
//#include "main.h"
//#include "usbd_audio_if.h"
//#include <string.h>
//#include <stdlib.h>
//#include <stdbool.h>
//extern USBD_HandleTypeDef hUsbDeviceFS;
//extern I2S_HandleTypeDef hi2s2;
//typedef struct
//{
//    volatile int32_t *sampleBuffer; // Con trỏ đến bộ nhớ chứa mẫu 64-bit
//    int16_t *processBuffer;         // Con trỏ đến bộ nhớ tạm xử lý 16-bit
//    int16_t *sendBuffer;            // Con trỏ đến bộ nhớ tạm gửi 16-bit
//    bool running;                   // Trạng thái chạy
//    uint8_t zeroCounter;            // Bộ đếm cho mute
//} Check;
//
//
//extern Check audioCheck;
//void Audio(void);
//int8_t Start(void);
//int8_t Stop(void);
//int8_t Pause(void);
//int8_t Resume(void);
//void SetLed(void);
//void I2S_Complete(void);
//void I2S_HalfComplete();
//void SetVolume(int16_t volume);
//void sendData(volatile int32_t *data_in, int16_t *data_out);
//#endif /* INC_AUDIO_H_ */
