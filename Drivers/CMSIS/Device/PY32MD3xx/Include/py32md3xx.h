/**
  ******************************************************************************
  * @file    py32md3xx.h
  * @brief   CMSIS PY32MD3xx Device Peripheral Access Layer Header File.
  * @version v1.0.0
  *
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

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup py32md3xx
  * @{
  */

#ifndef __PY32MD3XX_H
#define __PY32MD3XX_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @addtogroup Library_configuration_section
  * @{
  */

/**
  * @brief PY32 Family
  */
#if !defined  (PY32MD3)
#define PY32MD3
#endif /* PY32MD3 */

#if defined(PY32MD320x8)
#define PY32MD320PRE
#endif

/* Uncomment the line below according to the target PY32 device used in your
   application
  */

#if !defined (PY32MD320x8)
/* #define PY32MD320x8  */  /*!< PY32MD320x8  Devices (PY32MD320xx  microcontrollers where the Flash memory is 64 Kbytes)              */
#endif

/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */
#if !defined  (USE_HAL_DRIVER)
/**
 * @brief Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will
   be based on direct access to peripherals registers
   */
/*#define USE_HAL_DRIVER */
#endif /* USE_HAL_DRIVER */

/**
  * @brief CMSIS Device version number V1.0.0
  */
#define __PY32MD3_DEVICE_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __PY32MD3_DEVICE_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __PY32MD3_DEVICE_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __PY32MD3_DEVICE_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __PY32MD3_DEVICE_VERSION        ((__PY32MD3_DEVICE_VERSION_MAIN << 24)\
                                        |(__PY32MD3_DEVICE_VERSION_SUB1 << 16)\
                                        |(__PY32MD3_DEVICE_VERSION_SUB2 << 8 )\
                                        |(__PY32MD3_DEVICE_VERSION_RC))

/**
  * @}
  */

/** @addtogroup Device_Included
  * @{
  */

#if defined(PY32MD320x8)
#include "py32md320x8.h"
#else
#error "Please select first the target PY32MD3xx device used in your application (in py32md3xx.h file)"
#endif

/**
  * @}
  */

/** @addtogroup Exported_types
  * @{
  */
typedef enum
{
  RESET = 0,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0,
  ENABLE = !DISABLE
} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum
{
  ERROR = 0,
  SUCCESS = !ERROR
} ErrorStatus;

/**
  * @}
  */


/** @addtogroup Exported_macros
  * @{
  */
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define CLEAR_REG(REG)        ((REG) = (0x0))

#define WRITE_REG(REG, VAL)   ((REG) = (VAL))

#define READ_REG(REG)         ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define HW32_REG(ADDRESS)     ( * ((volatile unsigned           int * )(ADDRESS)))

#define HW16_REG(ADDRESS)     ( * ((volatile unsigned short     int * )(ADDRESS)))

#define HW8_REG(ADDRESS)      ( * ((volatile unsigned          char * )(ADDRESS)))

/**
  * @}
  */

#if defined (USE_HAL_DRIVER)
 #include "py32md3xx_hal.h"
#endif /* USE_HAL_DRIVER */


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PY32MD3XX_H */
/**
  * @}
  */

/**
  * @}
  */


/************************ (C) COPYRIGHT Puya *****END OF FILE******************/

