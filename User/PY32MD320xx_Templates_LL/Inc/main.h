/**
  ******************************************************************************
  * @file    main.h
  * @author  MCU Application Team
  * @brief   Header for main.c file.
  *          This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 Puya Semiconductor Co.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by Puya under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "py32md3xx_ll_rcc.h"
#include "py32md3xx_ll_bus.h"
#include "py32md3xx_ll_system.h"
#include "py32md3xx_ll_cortex.h"
#include "py32md3xx_ll_utils.h"
#include "py32md3xx_ll_pwr.h"
#include "py32md3xx_ll_dma.h"
#include "py32md3xx_ll_gpio.h"
#include "py32md3xx_ll_spi.h"
#include "py32md3xx_ll_tim.h"
#include "py32md3xx_ll_usart.h"
#include "py32md3xx_ll_flash.h"
#include "FOC.h"
#if defined(USE_FULL_ASSERT)
#include "py32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define FLASH_CONFIG_ADDR       ((uint32_t)0x0800F000U) 
#define FLASH_MAGIC_WORD        0xDEADBEEF // ?????????
#define FLASH_PROG_SIZE         8       // PY32 ??? 64 ?(8 ??)?????
/* Exported variables prototypes ---------------------------------------------*/


#pragma pack(push, 1) 
typedef struct {
    uint8_t Header;  // 1 ??
    uint8_t ID;  // 1 ??
		uint8_t Command;
		uint8_t BookedData;
    int16_t Value;  // 2 ??
} CommandStruct;


typedef struct {
		uint8_t mode;
		uint16_t readHeader;
		uint16_t pos;
		int16_t speed;
		int16_t target_pos;
		int16_t target_speed;
		int16_t Iq;
		int16_t V_alpha;
		int16_t V_beta;

} MotorParams;


#pragma pack(pop)

typedef struct {
    uint8_t ID;  // 1 ??
    uint16_t offset;
		int16_t Kp_spd;
		int16_t Ki_spd;
		int16_t Kp_pos;
		int16_t Ki_pos;
		uint32_t magic_word;
}	MotorConf;

typedef struct {
		int16_t PI;
		int16_t P;
		int16_t I;
		int16_t	Kp;
		int16_t Ki;
} PI_Controller;


/* Exported functions prototypes ---------------------------------------------*/
void APP_ErrorHandler(void);
void Save_MotorConf(const MotorConf *config);
int Load_MotorConf(MotorConf *config);
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
