/*
 * Application.h
 *
 *  Created on: Dec 14, 2024
 *      Author: Bacnk
 */

#pragma once

extern "C" {
#include "main.h"
#include "usbd_audio_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern I2S_HandleTypeDef hi2s2;
}

// include our classes

#include "Audio.h"
#include "Program.h"

// for the Debug_Semihosting configuration

#ifdef SEMIHOSTING
#include <stdio.h>
#endif
