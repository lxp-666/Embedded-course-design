#include "encoder.h"
#include "ti_msp_dl_config.h"

static ENCODER_RES motor_encoder_L;
static ENCODER_RES motor_encoder_R;


//编码器初始化
void encoder_init(void)
{
	//编码器引脚外部中断
	NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
	NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

//获取左编码器的值
int get_encoder_count_L(void)
{
	return motor_encoder_R.count;
}
//获取右编码器的值
int get_encoder_count_R(void)
{
	return motor_encoder_L.count;
}
//获取左编码器的方向
ENCODER_DIR get_encoder_dir_L(void)
{
	return motor_encoder_R.dir;
}
//获取右编码器的方向
ENCODER_DIR get_encoder_dir_R(void)
{
	return motor_encoder_L.dir;
}

//编码器数据更新
//间隔一定时间更新
void encoder_update_L(void)
{
	motor_encoder_L.count = motor_encoder_L.temp_count;

	//确定方向
	motor_encoder_L.dir = ( motor_encoder_L.count >= 0 ) ? FORWARD : REVERSAL;

	motor_encoder_L.temp_count = 0;//编码器计数值清零
}

void encoder_update_R(void)
{
	motor_encoder_R.count = motor_encoder_R.temp_count;

	//确定方向
	motor_encoder_R.dir = ( motor_encoder_R.count >= 0 ) ? FORWARD : REVERSAL;

	motor_encoder_R.temp_count = 0;//编码器计数值清零
}

//外部中断处理函数
void GROUP1_IRQHandler(void)
{
	uint32_t gpio_status;

	//获取中断信号情况
	gpio_status = DL_GPIO_getEnabledInterruptStatus(GPIO_ENCODER_PORT, GPIO_ENCODER_PIN_L_A_PIN | GPIO_ENCODER_PIN_L_B_PIN | GPIO_ENCODER_PIN_R_A_PIN | GPIO_ENCODER_PIN_R_B_PIN);
	
	
	//编码器A相上升沿触发 （左）
	if((gpio_status & GPIO_ENCODER_PIN_L_A_PIN) == GPIO_ENCODER_PIN_L_A_PIN)
	{
		//如果在A相上升沿下，B相为低电平
		if(!DL_GPIO_readPins(GPIO_ENCODER_PORT,GPIO_ENCODER_PIN_L_B_PIN))
		{
			motor_encoder_L.temp_count--;
		}
		else
		{
			motor_encoder_L.temp_count++;
			DL_GPIO_togglePins(GPIO_LED1_PORT,GPIO_LED1_PIN_22_PIN);
		}
	}//编码器B相上升沿触发 （左）
	  else if((gpio_status & GPIO_ENCODER_PIN_L_B_PIN)==GPIO_ENCODER_PIN_L_B_PIN)
	{
		//如果在B相上升沿下，A相为低电平
		if(!DL_GPIO_readPins(GPIO_ENCODER_PORT,GPIO_ENCODER_PIN_L_A_PIN))
		{
			motor_encoder_L.temp_count++;
			DL_GPIO_togglePins(GPIO_LED1_PORT,GPIO_LED1_PIN_22_PIN);
		}
		else
		{
			motor_encoder_L.temp_count--;
		}
	}
	
	//编码器A相上升沿触发 （右）
	if((gpio_status & GPIO_ENCODER_PIN_R_A_PIN) == GPIO_ENCODER_PIN_R_A_PIN)
	{
		//如果在A相上升沿下，B相为低电平
		if(!DL_GPIO_readPins(GPIO_ENCODER_PORT,GPIO_ENCODER_PIN_R_B_PIN))
		{
			motor_encoder_R.temp_count--;
		}
		else
		{
			motor_encoder_R.temp_count++;
		}
	}//编码器B相上升沿触发 （右）
	 else if((gpio_status & GPIO_ENCODER_PIN_R_B_PIN)==GPIO_ENCODER_PIN_R_B_PIN)
	{
		//如果在B相上升沿下，A相为低电平
		if(!DL_GPIO_readPins(GPIO_ENCODER_PORT,GPIO_ENCODER_PIN_R_A_PIN))
		{
			motor_encoder_R.temp_count++;
			
		}
		else
		{
			motor_encoder_R.temp_count--;
		}
	}
	//清除状态
	DL_GPIO_clearInterruptStatus(GPIO_ENCODER_PORT,GPIO_ENCODER_PIN_L_A_PIN | GPIO_ENCODER_PIN_L_B_PIN | GPIO_ENCODER_PIN_R_A_PIN | GPIO_ENCODER_PIN_R_B_PIN);
}
