#include "FOC.h"


static const uint8_t sector_lookup[8] = {0, 2, 6, 1, 4, 3 ,5 ,0};
static uint16_t level_u, level_v, level_w; 
static uint16_t factor, X, Y ,Z, T1, T2, T0;
static uint8_t sector, final_sector;
extern MotorConf conf;
extern MotorParams motor;



uint16_t Read_encoder(void){
	uint16_t ret = 0;
	SPI1_CS_ENABLE();
  while(!LL_SPI_IsActiveFlag_TXE(SPI1));
  LL_SPI_TransmitData16(SPI1, 0XFFFF);
  while(!LL_SPI_IsActiveFlag_RXNE(SPI1));
  ret = LL_SPI_ReceiveData16(SPI1)>>1;
	ret &= 0x3fff;
	SPI1_CS_DISABLE();
	return ret;
}




void set_PWM_level(Q15 Ualpha, Q15 Ubeta){
	if(!Ualpha && !Ubeta){
		LL_TIM_OC_SetCompareCH1(TIM1, PWM_TOP);
		LL_TIM_OC_SetCompareCH2(TIM1, PWM_TOP);
		LL_TIM_OC_SetCompareCH3(TIM1, PWM_TOP);
		return;
	}
	Q15 U1,U2,U3;
	U1 = Ubeta;
	U2 =  - Q15_Multiply(Ubeta, Q15_half) + Q15_Multiply(Ualpha, Q15_half_SQRT3);
	U3 =  - Q15_Multiply(Ubeta, Q15_half) - Q15_Multiply(Ualpha, Q15_half_SQRT3);
	
	sector = 0;
	if(U1 > 0) sector |= 1;
	if(U2 > 0) sector |= 2;
	if(U3 > 0) sector |= 4;
	final_sector = sector_lookup[sector];
	factor = Q14_Multiply(PWM_TOP, Q14_SQRT3);
	X = Q15_Multiply(factor, Ubeta);
	
	Y = Q15_Multiply(factor,
							(Q15_Multiply(  Q15_half_SQRT3, Ualpha)+ 
							Q15_Multiply(  Q15_half, 			 Ubeta)));
	
	Z = Q15_Multiply(factor, 
							(-Q15_Multiply(  Q15_half_SQRT3, Ualpha)+ 
							Q15_Multiply(  Q15_half, 			 Ubeta)));
  switch(final_sector) {
      case 1: T1 = -Z; T2 = X; break;
      case 2: T1 = Z;  T2 = Y; break;
      case 3: T1 = X; T2 = -Y; break;
      case 4: T1 = -X; T2 = Z; break;
      case 5: T1 = -Y;  T2 = -Z; break;
      case 6: T1 = Y; T2 = -X; break;
    }
	T0 = (PWM_TOP - T1 - T2) >> 2;
	switch (final_sector) {
      case 1: level_u = T1+T2+T0; 
              level_v = T2+T0; 
              level_w = T0; 
              break;
      case 2: level_u = T2+T0; 
              level_v = T1+T2+T0; 
              level_w = T0; 
              break;
      case 3: level_u = T0; 
              level_v = T1+T2+T0; 
              level_w = T2+T0; 
              break;
      case 4: level_u = T0; 
              level_v = T2+T0; 
              level_w = T1+T2+T0; 
              break;
      case 5: level_u = T2+T0; 
              level_v = T0; 
              level_w = T1+T2+T0; 
              break;
      case 6: level_u = T1+T2+T0; 
              level_v = T0; 
              level_w = T2+T0; 
              break;
    }
	
		LL_TIM_OC_SetCompareCH1(TIM1, level_u);
		LL_TIM_OC_SetCompareCH2(TIM1, level_v);
		LL_TIM_OC_SetCompareCH3(TIM1, level_w);
	
}

void setZero(void){
		motor.pos = Read_encoder();
		motor.pos_Prev = motor.pos;
		motor.step_dpos = motor.pos - conf.m_zero; 
		motor.total_pos = motor.step_dpos > 8192 ? motor.step_dpos - 0x3FFF : motor.step_dpos < - 8192 ? motor.step_dpos + 0x3FFF :motor.step_dpos;
		motor.total_pos_prev = motor.total_pos_prev;
}
