/**
  ******************************************************************************
  * @file    py32md3xx_it.c
  * @author  MCU Application Team
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "py32md3xx_it.h"
#include "FOC.h"
/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/



/* Private define ------------------------------------------------------------*/
#define Q15_IRPM   			8947
#define speed_limit     100



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
	CommandStruct CommandBuffer[2] = {0};
	uint8_t BufferActivated = 1;
	uint8_t BufferRecieve = 0;
	
	

	uint16_t encoder_prev = 0;
	uint8_t  Speed_subCounter = 0; 
	
	int16_t d_raw_angle = 0;
	int8_t sign = 0;
	uint16_t electric_angle = 0;
	
	MotorParams motor = {0};
	MotorConf conf = {0};
	
	PI_Controller speed = {0};
	PI_Controller position = {0};
/* Private function prototypes -----------------------------------------------*/
/* Private user code ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
}

/**
  * @brief This function handles timer1 update.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void){
	if(LL_TIM_IsActiveFlag_UPDATE(TIM1)){
		LL_TIM_ClearFlag_UPDATE(TIM1);
		//encoder proc
		motor.pos = Read_encoder();
		Speed_subCounter ++;
		if(Speed_subCounter == 32){
			Speed_subCounter = 0;
			motor.speed = motor.pos - encoder_prev;
			encoder_prev = motor.pos;
			motor.speed += (motor.speed < -8192) ? 16383 : (motor.speed > 8192)  ? -16383: 0;	
			sign = (motor.speed > 0) ? 1 : -1;
			d_raw_angle = (motor.speed > 0) ? motor.speed : - motor.speed;
			d_raw_angle = sign * (d_raw_angle >> 5);
		//CMD proc;
			
			
		switch(CommandBuffer[BufferActivated].Command){
			case 0xC2: 
				position.P = motor.target_pos - motor.pos;
				position.P += (position.P < - 8192) ? 16383 : (position.P > 8192) ? -16383 : 0;
				if(! ( (position.PI >= speed_limit && position.P > 0) || (position.PI <= -speed_limit && position.P < 0) )){
					position.I += Q15_Multiply(position.Ki, position.P);
				}
				position.P = Q15_Multiply(position.P, position.Kp);
				position.PI = position.P + position.I;
				position.PI = (position.PI > speed_limit) ? speed_limit : (position.PI < -speed_limit) ? -speed_limit : position.PI; 
				motor.target_speed = position.PI;  
				//Fall-through
			case 0xC1:
				speed.P = motor.speed - motor.target_speed;
				if(! ( (speed.PI >= Q15_P577 && speed.P > 0) || (speed.PI <= -Q15_P577 && speed.P < 0) )){
					speed.I += speed.Ki * speed.P;
				}
				speed.P *= speed.Kp;
				speed.PI = speed.P + speed.I;
				speed.PI = (speed.PI > Q15_P577) ? Q15_P577 : (speed.PI < -Q15_P577) ? -Q15_P577 : speed.PI;
				motor.Iq = speed.PI;
				break;
			case 0xC0:
				motor.Iq = Q15_Multiply(CommandBuffer[BufferActivated].Value, Q15_P577);
				break;
			default:
				break;
			}
		
		}

		//set PWM
		electric_angle = (7  * (motor.pos + d_raw_angle + conf.offset)) & 0x3FFF;
		motor.V_alpha= Q15_Multiply(motor.Iq,  - fast_sin_q15_raw(electric_angle >> 5));
		motor.V_beta = Q15_Multiply(motor.Iq,  fast_cos_q15_raw(electric_angle >> 5));
		set_PWM_level(motor.V_alpha, motor.V_beta);
	}
}



/**
  * @brief This function handles timer1 update.
  */
void USART2_IRQHandler(void){
	if (LL_USART_IsActiveFlag_IDLE(USART2))
    {
        LL_USART_ClearFlag_IDLE(USART2);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2); 
				if(	CommandBuffer[BufferRecieve].Header == 0XAA && CommandBuffer[BufferRecieve].ID == conf.ID){
				//Config mode, send a pack of config			
					motor.mode = CommandBuffer[BufferRecieve].Command;
				  switch(motor.mode){
						//position mode
						case 0XC2:
							motor.target_pos = (CommandBuffer[BufferRecieve].Value > 16383) ? 16383 : (CommandBuffer[BufferRecieve].Value < 0) ? 0 : CommandBuffer[BufferRecieve].Value;
							BufferRecieve = BufferRecieve ? 0 : 1;
							BufferActivated = BufferActivated ? 0 : 1;
							break;
						//speed mode
						case 0XC1:
							motor.target_speed = Q15_Multiply(Q15_IRPM, CommandBuffer[BufferRecieve].Value);
							BufferRecieve = BufferRecieve ? 0 : 1;
							BufferActivated = BufferActivated ? 0 : 1;
							break;
						//Torque mode
						case 0xC0:
							BufferRecieve = BufferRecieve ? 0 : 1;
							BufferActivated = BufferActivated ? 0 : 1;
							break;
						
						//ID config
						case 0xF0:
							conf.ID = CommandBuffer[BufferRecieve].Value & 0xFF;
							break;
						//offset config
						case 0XF1:
							conf.offset = CommandBuffer[BufferRecieve].Value & 0x3FFF;
							break;
						//K config
						case 0xD0:
							conf.Kp_pos = CommandBuffer[BufferRecieve].Value;
							break;
						case 0xD1:
							conf.Ki_pos = CommandBuffer[BufferRecieve].Value;
							break;
						case 0xE0:
							conf.Kp_spd = CommandBuffer[BufferRecieve].Value;
							break;
						case 0xE1:
							conf.Kp_spd = CommandBuffer[BufferRecieve].Value;
							break;
						case 0xA0:
							LL_TIM_DisableAllOutputs(TIM1);
							LL_TIM_DisableIT_UPDATE(TIM1);
							break;
					  case 0xA1:
							LL_TIM_EnableAllOutputs(TIM1);
							LL_TIM_EnableIT_UPDATE(TIM1);
							break;
						//save Config
						case 0xB0:
							LL_TIM_DisableAllOutputs(TIM1);
							LL_TIM_DisableIT_UPDATE(TIM1);
							conf.Kp_pos = position.Kp;
							conf.Ki_pos = position.Ki;
							conf.Kp_spd = speed.Kp;
							conf.Ki_spd = speed.Ki;
							Save_MotorConf(&conf);
							break;
						default:
							break;
					}

					LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_1);
				}
				LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)(&CommandBuffer[BufferRecieve]));
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 6);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
    }
    

    if (LL_USART_IsActiveFlag_ORE(USART2))
		{
        LL_USART_ClearFlag_ORE(USART2);
    }
	
}
/******************************************************************************/
/* PY32MD3xx Peripheral Interrupt Handlers                                     */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file.                                          */
/******************************************************************************/

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
