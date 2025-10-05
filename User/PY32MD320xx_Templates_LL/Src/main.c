/**
  ******************************************************************************
  * @file    main.c
  * @author  MCU Application Team
  * @brief   Main program body
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

/* Private define ------------------------------------------------------------*/
#define SPI1_CS_PORT 				GPIOA
#define SPI1_CS_PIN 				LL_GPIO_PIN_15


#define SPI1_CS_ENABLE()    LL_GPIO_ResetOutputPin(SPI1_CS_PORT, SPI1_CS_PIN)
#define SPI1_CS_DISABLE()   LL_GPIO_SetOutputPin(SPI1_CS_PORT, SPI1_CS_PIN)
/* Private variables ---------------------------------------------------------*/
extern MotorParams motor;
extern CommandStruct CommandBuffer[];
extern MotorConf conf;
extern PI_Controller speed;
extern PI_Controller position;
/* Private user code ---------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void APP_SystemClockConfig(void);
static void APP_GPIOConfig(void);
static void APP_SPI1Config(void);
static void APP_USARTConfig(void);
static void APP_DMAConfig(void);
static void APP_PWMChannelConfig(void);
static void APP_TIM1BaseConfig(void);
static void NVIC_Config(void);

uint8_t flash_buffer[FLASH_PAGE_SIZE];




void Save_MotorConf(const MotorConf *config){
	LL_FLASH_Unlock(FLASH);
	memset(flash_buffer, 0XFF, FLASH_PAGE_SIZE);
	memcpy(flash_buffer, &conf, sizeof(MotorConf));
	LL_FLASH_EnableIT_EOP(FLASH);
  LL_FLASH_EnablePageProgram(FLASH);
	LL_FLASH_PageProgram(FLASH, FLASH_CONFIG_ADDR, (uint32_t*)flash_buffer);
  while(LL_FLASH_IsActiveFlag_BUSY(FLASH)==1);
  while(LL_FLASH_IsActiveFlag_EOP(FLASH)==0);
  LL_FLASH_ClearFlag_EOP(FLASH);
  LL_FLASH_DisableIT_EOP(FLASH);
  LL_FLASH_DisablePageProgram(FLASH);
	LL_FLASH_Lock(FLASH);
}


int Load_MotorConf(MotorConf *config){
	const MotorConf *flash_src = (const MotorConf*)FLASH_CONFIG_ADDR;
	if(flash_src->magic_word != FLASH_MAGIC_WORD){
		return -1;
	}
	memcpy(config, flash_src, sizeof(MotorConf));
		return 0;
}






/**
  * @brief  Main program.
  * @param  None
  * @retval int
  */
int main(void)
{
  /* Configure system clock */
  APP_SystemClockConfig();
	NVIC_Config();
	
	APP_GPIOConfig();
	APP_SPI1Config();
	APP_USARTConfig();

	 
	APP_PWMChannelConfig();
	APP_TIM1BaseConfig();
	
	if(Load_MotorConf(&conf)){
		conf.ID = 1;
		conf.offset = 11975;
		conf.Kp_pos = 1500;
		conf.Ki_pos = 0;
		conf.Kp_spd = 110;
		conf.Ki_spd = 9;
		conf.magic_word = FLASH_MAGIC_WORD;
	}
	position.Kp = conf.Kp_pos;
	position.Ki = conf.Ki_pos;
	speed.Kp = conf.Kp_spd;
	speed.Ki = conf.Ki_spd;
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
	LL_TIM_EnableIT_UPDATE(TIM1);
  while (1)
  {	
  }
}

/**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
static void APP_SystemClockConfig(void)
{
	LL_UTILS_ClkInitTypeDef UTILS_ClkInitStruct = {LL_RCC_SYSCLK_DIV_1, LL_RCC_APB1_DIV_1};
  /* Enable HSI */
  LL_RCC_HSI_Enable();
	LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_24MHz);
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }
	LL_PLL_ConfigSystemClock_HSI(&UTILS_ClkInitStruct);

	
  LL_Init1msTick(48000000);
  /* Update system clock global variable SystemCoreClock (can also be updated by calling SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(48000000);

	
	
  /* Set AHB prescaler */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
		/* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);


}


/**
  * @brief  Configure PA08 as MCO alternate function
  * @param  None
  * @retval None
  */
static void APP_GPIOConfig(void)
{
  /* Enable GPIOA clock */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  
  /* Configure PA01 as LED PIN */
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_1; 
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; 
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;   
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;                
  
  /* Initialize GPIOA */
  LL_GPIO_Init(GPIOA,&GPIO_InitStruct);
}

/**
  * @brief  SPI1 configuration function
  * @param  None
  * @retval None
  */
static void APP_SPI1Config(void){
	LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable clock */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  
  
  /**SPI1 pin configuration
  PB3   ------> SPI1_SCK
  PA11   ------> SPI1_MISO
  PA15   ------> SPI1_SOFT_NSS
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_0;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15);
	
	
	
	  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_16BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
	
	LL_SPI_Enable(SPI1);
}

/**
  * @brief  Configure TIM base
  * @param  None
  * @retval None
  */
static void APP_USARTConfig(void){
	  /* Enable clock, initialize pins, enable NVIC interrupt */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    
    /** GPIOA configuration 
				PA2  ->  TX
				PA5  <-  RX      */
	
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF4_USART2;
    LL_GPIO_Init(GPIOA,&GPIO_InitStruct);
    

    GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
    GPIO_InitStruct.Alternate = LL_GPIO_AF9_USART2;
    LL_GPIO_Init(GPIOA,&GPIO_InitStruct);
    
    /* Set USART1 interrupt priority */
    NVIC_SetPriority(USART2_IRQn,0);
    /* Enable USART1 interrupt */
    NVIC_EnableIRQ(USART2_IRQn);

  /* USART configuration */
		LL_USART_InitTypeDef USART_InitStruct = {0};
		USART_InitStruct.BaudRate = 115200;
		USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
		USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
		USART_InitStruct.Parity = LL_USART_PARITY_NONE;
		USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
		USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
		USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
		LL_USART_Init(USART2, &USART_InitStruct);
  
  /* Configure as full duplex asynchronous mode */
		LL_USART_ConfigAsyncMode(USART2);
		APP_DMAConfig();
		LL_USART_EnableIT_IDLE(USART2);
  /* Enable UART module */
		LL_USART_Enable(USART2);
	
}
/**
  * @brief  Configure TIM1 PWM related GPIO
  * @param  None
  * @retval None
  */
static void APP_DMAConfig(void){
  /* Enable DMA clock and syscfg clock */
		LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
		LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

  
  /* Configure DMA channel LL_DMA_CHANNEL_1 for transmission */
		LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_MEMORY_TO_PERIPH | \
                      LL_DMA_MODE_NORMAL                   | \
                      LL_DMA_PERIPH_NOINCREMENT  | \
                      LL_DMA_MEMORY_INCREMENT  | \
                      LL_DMA_PDATAALIGN_BYTE | \
                      LL_DMA_MDATAALIGN_BYTE | \
                      LL_DMA_PRIORITY_LOW);
  
 
  /* Configure DMA channel LL_DMA_CHANNEL_2 for reception */
		LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY | \
                      LL_DMA_MODE_NORMAL                   | \
                      LL_DMA_PERIPH_NOINCREMENT  | \
                      LL_DMA_MEMORY_INCREMENT  | \
                      LL_DMA_PDATAALIGN_BYTE | \
                      LL_DMA_MDATAALIGN_BYTE | \
                      LL_DMA_PRIORITY_LOW);
	  LL_SYSCFG_SetDMARemap_CH1(LL_SYSCFG_DMA_MAP_USART2_TX);
    LL_SYSCFG_SetDMARemap_CH2(LL_SYSCFG_DMA_MAP_USART2_RX);
		
		
		
		
		LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&CommandBuffer[0]);
		LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&(USART2->DR));
		LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 6);
		LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
		LL_USART_ClearFlag_ORE(USART2);
		LL_USART_EnableDMAReq_RX(USART2);
}
/**
  * @brief  Configure TIM1 PWM related GPIO
  * @param  None
  * @retval None
  */

static void APP_PWMChannelConfig(void)
{
	LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	
	
  LL_GPIO_InitTypeDef TIM1CHMapInit= {0};
  LL_TIM_OC_InitTypeDef TIM_OC_Initstruct ={0};

  /* Configure PA8/PA9/PA10 as TIM1_CH1/TIM1_CH2/TIM1_CH3 */
  TIM1CHMapInit.Pin        = LL_GPIO_PIN_7|LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  TIM1CHMapInit.Mode       = LL_GPIO_MODE_ALTERNATE;
  TIM1CHMapInit.Alternate  = LL_GPIO_AF_2; 
  TIM1CHMapInit.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
  TIM1CHMapInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  TIM1CHMapInit.Pull       = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA,&TIM1CHMapInit);
  /* Configure PA7/PB0/PB1 as TIM1_CH1N/TIM1_CH2N/TIM1_CH3N */
	TIM1CHMapInit.Pin        = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  TIM1CHMapInit.Mode       = LL_GPIO_MODE_ALTERNATE;
  TIM1CHMapInit.Alternate  = LL_GPIO_AF_2; 
  TIM1CHMapInit.Speed      = LL_GPIO_SPEED_FREQ_HIGH;
  TIM1CHMapInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  TIM1CHMapInit.Pull       = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB,&TIM1CHMapInit);
	
	
  TIM_OC_Initstruct.OCMode        = LL_TIM_OCMODE_PWM2;     /* Mode: PWM2 */
	
  TIM_OC_Initstruct.OCState       = LL_TIM_OCSTATE_ENABLE;  /* Channel enable */
	TIM_OC_Initstruct.OCNState       = LL_TIM_OCSTATE_ENABLE;  
	
  TIM_OC_Initstruct.OCPolarity    = LL_TIM_OCPOLARITY_HIGH; /* Active polarity: high level */
	TIM_OC_Initstruct.OCNPolarity    = LL_TIM_OCPOLARITY_HIGH;
	
  TIM_OC_Initstruct.OCIdleState   = LL_TIM_OCIDLESTATE_LOW; /* Complementary channel active polarity: high level */
	TIM_OC_Initstruct.OCNIdleState   = LL_TIM_OCIDLESTATE_LOW;
	
  TIM_OC_Initstruct.CompareValue  = 500;
  /* Configure channel 1 */
  LL_TIM_OC_Init(TIM1,LL_TIM_CHANNEL_CH1,&TIM_OC_Initstruct);
  /* Configure channel 2 */
  LL_TIM_OC_Init(TIM1,LL_TIM_CHANNEL_CH2,&TIM_OC_Initstruct);
  /* Configure channel 3 */
  LL_TIM_OC_Init(TIM1,LL_TIM_CHANNEL_CH3,&TIM_OC_Initstruct);
}

/**
  * @brief  Configure TIM base
  * @param  None
  * @retval None
  */
static void APP_TIM1BaseConfig(void)
{
  /* Configure TIM1 */


	
	
	LL_TIM_InitTypeDef TIM1CountInit = {0};
	
  TIM1CountInit.ClockDivision       = LL_TIM_CLOCKDIVISION_DIV1;   /* No clock division */
  TIM1CountInit.CounterMode         = LL_TIM_COUNTERMODE_CENTER_DOWN;      /* Up counting mode */
  TIM1CountInit.Prescaler           = 0;                      /* Prescaler value: 2400 */
  TIM1CountInit.Autoreload          = PWM_TOP;                           /* Repetition counter value: 0 */
  TIM1CountInit.RepetitionCounter 	= 0;
	
  /* Initialize TIM1 */
  LL_TIM_Init(TIM1,&TIM1CountInit);

  /* Enable main output */
  LL_TIM_EnableAllOutputs(TIM1);

  /* Enable TIM1 counter */
  LL_TIM_EnableCounter(TIM1);
}

/**
  * @brief  System clock configuration function
  * @param  None
  * @retval None
  */
static void NVIC_Config(void){
	
	NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0);
	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
  //NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
  //NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  //NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1);
  //NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	NVIC_SetPriority(USART2_IRQn, 2);
	NVIC_EnableIRQ(USART2_IRQn);
}

/**
 * @brief ?? SPI ?????????? (????)
 * * @param SPIx ???? SPI ??
 * @param TxData ??????(????????? 0x00 ? 0xFF)
 * @return uint8_t ??????
 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void APP_ErrorHandler(void)
{
  /* Infinite loop */
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     for example: printf("Wrong parameters value: file %s on line %d\r\n", file, line)  */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT Puya *****END OF FILE******************/
