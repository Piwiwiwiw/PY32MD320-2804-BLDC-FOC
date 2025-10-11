#ifndef __FOC_H
#define __FOC_H


#define SPI1_CS_PORT 				GPIOA
#define SPI1_CS_PIN 				LL_GPIO_PIN_15
#define SPI1_CS_ENABLE()    LL_GPIO_ResetOutputPin(SPI1_CS_PORT, SPI1_CS_PIN)
#define SPI1_CS_DISABLE()   LL_GPIO_SetOutputPin(SPI1_CS_PORT, SPI1_CS_PIN)

#define PWM_TOP  		1500


#include "main.h"
#include "trig_q15_func.h"


uint16_t Read_encoder(void);
void set_PWM_level(Q15 Ualpha, Q15 Ubeta);
void setZero(void);
#endif