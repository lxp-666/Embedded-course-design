#ifndef __TB6612_H
#define __TB6612_H
#include "stdint.h"
#include "ti_msp_dl_config.h"

 extern volatile	 int16_t last_sum,last_error;	

#define AIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_AIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN1_PIN)) )
#define AIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_AIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN2_PIN)) )

#define BIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_BIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN1_PIN)) )
#define BIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_BIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN2_PIN)) )


void TB6612_Motor_Stop(void);
void AO_Control(uint8_t dir, uint32_t speed);
void BO_Control(uint8_t dir, uint32_t speed);

void CCR_PWMA_SET( uint32_t speed);
void CCR_PWMB_SET( uint32_t speed);
void PID_Speed_Change_L(int16_t Taget_speed,int16_t Measure,float Kp,float Ki,float Kd) ;
void PID_Speed_Change_R(int16_t Taget_speed,int16_t Measure,float Kp,float Ki,float Kd) ;


#endif