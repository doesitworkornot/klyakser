/**
  ******************************************************************************
  * @file    BSP/Inc/main.h 
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "stdlib.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_ts.h"
#include "stm32f429i_discovery_io.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_gyroscope.h"
#ifdef EE_M24LR64
#include "stm32f429i_discovery_eeprom.h"
#endif /*EE_M24LR64*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
  void   (*DemoFunc)(void);
  uint8_t DemoName[50]; 
  uint32_t DemoIndex;
}BSP_DemoTypedef;

typedef struct platform_pos{
	int enabled;
	int x;
	int y;
}platforms;

typedef struct doodler_s{
	int x;
	int y;
	const unsigned char* img;
}doodler_s;


/* Exported constants --------------------------------------------------------*/
#define NUM_PLATFORMS 2
#define JUMP_HEIGHT 4

/* Exported macro ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
void Joystick_demo (void);
void Touchscreen_demo (void);
void LCD_demo (void);
void MEMS_demo (void);
void Log_demo(void);
#ifdef EE_M24LR64
void EEPROM_demo (void);
#endif /*EE_M24LR64*/
void SD_demo (void);
void Touchscreen_Calibration (void);
uint16_t Calibration_GetX(uint16_t x);
uint16_t Calibration_GetY(uint16_t y);
uint8_t IsCalibrationDone(void);
uint8_t CheckForUserInput(void);
void Toggle_Leds(void);
static void SystemClock_Config(void);
static void Display_DemoDescription(void);
void Display_field(platforms*, doodler_s);
void move_down(platforms*, int);

#endif /* __MAIN_H */
