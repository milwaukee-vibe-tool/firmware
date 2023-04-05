/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

// NOTE: static const values should be optimized in place, even at -O1
// NOTE: we can use defines, but this gives type safety
static const uint16_t ADXL343_ADDR = 0x53 << 1;

// data registers for ADXL343
static const uint16_t ADXL343_REG_DEVID = 0x00;
static const uint16_t ADXL343_REG_BWRATE = 0x2c;
static const uint16_t ADXL343_REG_PWRCTL= 0x2d;
static const uint16_t ADXL343_REG_INTENABLE= 0x2e;
static const uint16_t ADXL343_REG_DATAFORMAT = 0x31;
static const uint16_t ADXL343_REG_DATAX0 = 0x32;
static const uint16_t ADXL343_REG_DATAX1 = 0x33;
static const uint16_t ADXL343_REG_DATAY0 = 0x34;
static const uint16_t ADXL343_REG_DATAY1 = 0x35;
static const uint16_t ADXL343_REG_DATAZ0 = 0x36;
static const uint16_t ADXL343_REG_DATAZ1 = 0x37;
static const uint16_t ADXL343_REG_FIFOCTL = 0x38;

// scale factor at +/-8g
// this number is 0.00156 g/LSB * 9.81
static const float ADXL343_SCALE_8G = 0.15306;


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
