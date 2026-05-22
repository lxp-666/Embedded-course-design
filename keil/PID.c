#include "ti_msp_dl_config.h"
#include "PID.h"

float err_x = 0.0;											//x误差
float err_y = 0.0;											//y误差
float err_left = 0.0;										//x误差
float err_right = 0.0;											//y误差

float integral_x = 0.0;									//x积分
float integral_y = 0.0;									//y积分
float integral_left = 0.0;									//左边积分
float integral_right = 0.0;									//右边边积分

float err_last_x = 0.0;									//x上一误差值
float err_last_y = 0.0;									//y上一误差值
float err_last_left = 0.0;									//左边上一误差值
float err_last_right = 0.0;									//右边上一误差值

float actual_value = 0.0;								//驱动值
float actual_value2 = 0.0;								//驱动值2

/*************************************
*函  数  名	：PID_x()
*函数功能	：将X轴的各个参数进行PID运算
*输	入	：目标值，实际值 ,kp_x ,ki_x ,kd_x
*输	出	：PID运算后的驱动值
*************************************/
float PID_x(float target_value, float temp_val, float kp_x, float ki_x, float kd_x)
{
	/*位置式PID*/
	/*误差 = 目标值 - 输入的实际值*/
	err_x = target_value - temp_val;
	
	/*误差累计*/
	integral_x += err_x;
	
	
	/*积分限幅*/
	if(integral_x > 3000)
		integral_x = 3000;
	else if(integral_x < -3000)
		integral_x = -3000;
	
	/*PID算法实现*/
	actual_value = kp_x*err_x + ki_x*integral_x + kd_x*(err_x - err_last_x);
	
	/*记录上一误差值*/
	err_last_x = err_x;
	
	
	/*返回当前实际值*/
	return actual_value;
}



/**
  * @brief  设置目标值
  * @param  val		目标值
  *	@note 	无
  * @retval 无
  */
void set_PID_target(int temp_val,PID *pid)
{  
	pid->target_val = temp_val;          // 设置当前的目标值
}

/**
  * @brief  获取目标值
  * @param  无
  *	@note 	无
  * @retval 目标值
  */
int get_pid_target(PID *pid)
{
  return pid->target_val;    // 获取当前的目标值
}

/**
  * @brief  设置比例、积分、微分系数
  * @param  p：比例系数 P
  * @param  i：积分系数 i
  * @param  d：微分系数 d
  *	@note 	无
  * @retval 无
  */
void set_PID(float p, float i, float d,PID *pid)
{
	pid->Kp = p;    // 设置比例系数 P
	pid->Ki = i;    // 设置积分系数 I
	pid->Kd = d;    // 设置微分系数 D
}

void PID_param_init(PID *pid,float kp,float ki,float kd,float limit,float outmax)
{

	/* 初始化参数(连接上位机后, 可实时修改PID参数) */
	pid->target_val=0;				
	pid->output_val=0.0;
	pid->Error=0.0;
	pid->LastError=0.0;
	pid->PrevError=0.0;
	pid->integral=0.0;
	pid->integral_limit=limit;
	pid->outmax=outmax;

	pid->Kp = kp; //0.2
	pid->Ki = ki;
	pid->Kd = kd;
}

/**
  * @brief  速度PID算法实现
  * @param  actual_val:实际值
  *	@note 	无
  * @retval 通过PID计算后的输出
  */
void addPID_realize(PID *pid, float actual_val)
{
	/*计算目标值与实际值的误差*/
	pid->Error = pid->target_val - actual_val;
	
	/*积分项*/
	pid->integral += pid->Error;
	
	/*PID算法实现*/
	pid->output_val += pid->Kp * (pid->Error - pid-> LastError) +
                       pid->Ki * pid->Error +
	                   pid->Kd *(pid->Error -2*pid->LastError+pid->PrevError);
	/*误差传递*/
	pid-> PrevError = pid->LastError;
	pid-> LastError = pid->Error;

	/*返回当前实际值*/
	
}

void PosionPID_realize(PID *pid, float actual_val)
{
	/*计算目标值与实际值的误差*/
	pid->Error = pid->target_val - actual_val;
	
	/*积分项*/
	pid->integral += pid->Error;
	/*积分项限幅*/
	if(pid->integral>pid->integral_limit) pid->integral=pid->integral_limit;
	if(pid->integral<-(pid->integral_limit))pid->integral=-(pid->integral_limit);
	/*PID算法实现*/
	pid->output_val = pid->Kp * pid->Error +
	                  pid->Ki * pid->integral +
	                  pid->Kd *(pid->Error -pid->LastError);
	if(pid->output_val>pid->outmax) pid->output_val=pid->outmax;
	/*误差传递*/
	pid-> LastError = pid->Error;
	
	/*返回当前实际值*/
//	return pid->output_val;
}
