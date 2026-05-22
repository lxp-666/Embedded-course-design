#include "TB6612.h"
#include "ti_msp_dl_config.h"
#include "stdint.h"
#include "stdio.h"

volatile  unsigned int CompareA=0; 
volatile  unsigned int CompareB=0; 
volatile  int16_t last_sum,last_error;	
/******************************************************************
 * 函 数 名 称：TB6612_Motor_Stop
 * 函 数 说 明：A端和B端电机停止
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
void TB6612_Motor_Stop(void)
{
    AIN1_OUT(1);
    AIN2_OUT(1);
    BIN1_OUT(1);
    BIN2_OUT(1);
}

/******************************************************************
 * 函 数 名 称：AO_Control
 * 函 数 说 明：A端口电机控制
 * 函 数 形 参：dir旋转方向 1正转0反转   speed旋转速度，范围（0 ~ per-1）
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：speed 0--最快，9999--停止
******************************************************************/
void AO_Control(uint8_t dir, uint32_t speed)
{

    if( dir == 1 )
    {
        AIN1_OUT(0);
        AIN2_OUT(1);
    }
    else
    {
        AIN1_OUT(1);
        AIN2_OUT(0);
    }

    DL_TimerG_setCaptureCompareValue(PWM_0_INST, speed, GPIO_PWM_0_C1_IDX);
}



/******************************************************************
 * 函 数 名 称：BO_Control
 * 函 数 说 明：B端口电机控制
 * 函 数 形 参：dir旋转方向 1正转0反转   speed旋转速度，范围（0 ~ per-1）
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：speed 0-9999   0--最快，9999--停止
******************************************************************/
void BO_Control(uint8_t dir, uint32_t speed)
{

    if( dir == 1 )
    {
        BIN1_OUT(0);
        BIN2_OUT(1);
    }
    else
    {
        BIN1_OUT(1);
        BIN2_OUT(0);
    }

    DL_TimerG_setCaptureCompareValue(PWM_0_INST, speed, GPIO_PWM_0_C0_IDX);
}

void GPIO_Drection(void)
{
  DL_GPIO_setPins(TB6612_PORT,TB6612_AIN2_PIN);
  DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN1_PIN);
	
  DL_GPIO_setPins(TB6612_PORT,TB6612_BIN1_PIN);
  DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN2_PIN);
}

void CCR_PWMA_SET( uint32_t speed)
{
//	if(speed>300) speed=300;   //限制占空比改变
	DL_TimerG_setCaptureCompareValue(PWM_0_INST, speed, GPIO_PWM_0_C1_IDX);
}

void CCR_PWMB_SET( uint32_t speed)
{
//	if(speed>300) speed=300;
	DL_TimerG_setCaptureCompareValue(PWM_0_INST, speed, GPIO_PWM_0_C0_IDX);
}


/////*  	PID控制函数		*/////
void PID_Speed_Change_L(int16_t Taget_speed,int16_t Measure,float Kp,float Ki,float Kd) 
	/*	通过设定目标速度和测量编码器速度来维持特定速度	*/
{
	int16_t Error,temp;
	Error=Taget_speed-Measure;//pwm数值量（MAX9999）
	static int16_t last_sum=0,last_error;
	temp=(int16_t)( Kp*Error + Ki*last_sum + Kd*(Error-last_error) );
	if(temp>1000)    temp=1000;     //限制幅度改变速度
	if(temp<-1000)   temp=-1000;
	if(temp>0)
	{
		CompareA+= (uint16_t)temp;
	}
	else
	{
		CompareA-=(uint16_t)(-temp);
	}			
	if(Error<200 && Error>-200)
	{
		last_sum+=Error;
		last_error=Error;	
	}			
	if(CompareA>6000) CompareA=6000;
	BO_Control(1,CompareA);
//	CCR_PWMB_SET(CompareA); 		//调节占空比函数
}
//
void PID_Speed_Change_R(int16_t Taget_speed,int16_t Measure,float Kp,float Ki,float Kd) 
/*	通过设定目标速度和测量编码器速度来维持特定速度	*/
{
	int16_t Error,temp;	
	Error=Taget_speed-Measure;
	static int16_t last_sum=0,last_error;	
	temp=(int16_t)( Kp*Error + Ki*last_sum + Kd*(Error-last_error) );	
	if(temp>1000) temp=1000;     //限制幅度改变速度
	if(temp<-1000) temp=-1000;
	if(temp>0)
	{
		CompareB+= (uint16_t)temp;	
	}
	else
	{
		CompareB-=(uint16_t)(-temp);
	}
	if(Error<200 && Error>-200)
	{
		last_sum+=Error;
		last_error=Error;			
	}
	if(CompareB>6000) CompareB=6000 ;    //限幅
	AO_Control(1,CompareB);
//	CCR_PWMA_SET(CompareB); 		//调节占空比函数
}