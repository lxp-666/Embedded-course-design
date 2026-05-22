#include "ti_msp_dl_config.h"
#include "string.h"
#include "LCD.h"
#include "TB6612.h"
#include "encoder.h"
#include "stdio.h"
#include "timer.h"
#include "UART.h"
#include "time.h"
#include "gw_grayscale_sensor.h"
#include "PID.h"
#include "hardware_iic.h"
#include "IIC.h"
#include "gyro.h"
#include "math.h"
#include "Kalman.h" 


///////////////函数声明////////////////////////////////////////
#define STOP 0 			//停车
#define NAVIGATION 1  	//惯性导航状态
#define LINE 2  		//循线状态

#define u8 uint8_t                                     
#define u16 uint16_t                                   
#define u32 uint32_t 

extern uint8_t IIC_ReadBytes(uint8_t Salve_Address, uint8_t Reg_Address, 
                          uint8_t *Result, uint8_t len);

void ui_encoder_value_update(void);//编码器的数据屏幕显示
uint16_t Get_speed_L(void);//左电机速度获取函数
uint16_t Get_speed_R(void);//右电机速度获取函数
float Track_Err(uint8_t car_state);//获取灰度误差值
void PID_position(float target_value, float temp_val, float kp, float ki, float kd);
void PID_angle(float target, float current, float kp, float ki, float kd); 

void tast1();	//走直线
void tast2();	//绕一圈
void tast3();	//走8字
void tast4();	//走4圈8字


///////////////全局变量////////////////////////////////////////
int mode=1;
int begin_flag=0;

volatile int16_t Taget_speed_L;
volatile int16_t Taget_speed_R;
volatile int16_t Taget_speed_L_angle;
volatile int16_t Taget_speed_R_angle;
volatile int16_t Taget_speed_L_angle_zw;
volatile int16_t Taget_speed_R_angle_zw;


volatile int L_speed_actual;//左电机速度
volatile int R_speed_actual;//右电机速度

volatile uint16_t glfage_1ms=0;
volatile uint8_t glfage_5ms=0;
volatile uint8_t glfage_10ms=0;
volatile uint8_t glfage_20ms=0;
volatile uint8_t glfage_500ms=0;
volatile uint8_t glfage_1000ms=0;

unsigned char Digtal;
uint8_t huidu[8]={0,0,0,0,0,0,0,0};//0-7路灰度数据
#define SENSOR_COUNT 8
float Error_huidu;
float bias;

#define DEFAULT_DUTY_L 2300// 	定义电机默认直走占空比(左)
#define DEFAULT_DUTY_R 2380// 	定义电机默认直走占空比(右)

unsigned char Digtal;

//速度环
float Kp_L=0.17; 	
float Ki_L=0.0;
float Kd_L=0;
float Kp_R=0.17; 	
float Ki_R=0.0;
float Kd_R=0;

//位置环
float Kp_P=0.8; 
float Ki_P=0;
float Kd_P=0;

//角度环
float Kp_A=33;
float Ki_A=0.0;
float Kd_A=0.0;

u8 count_Set=0;//设定点数
u8 count=0;//计点数

Gyro_Struct *JY61P_Data;
static float gyro_offset = 0;
static uint8_t gyro_calibrated = 0;

void calibrate_gyro() {
    if (!gyro_calibrated) {
        float sum = 0;
        for (int i = 0; i < 100; i++) { // 采集100次求平均
            sum += JY61P_Data->z;
            delay_ms(10);
        }
        gyro_offset = sum / 100; // 零漂补偿值
        gyro_calibrated = 1;
    }
}

///////////////////主程序/////////////////////////////////////////////
int main(void)
{
	int time = 0;
	SYSCFG_DL_init();	    
	uart_init();	//串口初始化	
	encoder_init();	//编码器初始化		
	timer_init();	//定时器初始化
	jy61pInit();	//陀螺仪初始化
	lcd_init();		//LCD初始化
	calibrate_gyro(); // 增加校准
	
	AO_Control(1,DEFAULT_DUTY_R);
	BO_Control(1,DEFAULT_DUTY_L);
	
	begin_flag=1;
	while (1) 
	{
		
		mode=1;
		if(mode==1)
		{
			if(begin_flag==1)
			{		
				tast1();
			}
		
		}
//		else if(mode==2)
//		{
//			if(begin_flag==1)
//			{
//				tast2();
//			}
//		
//		}
//		else if(mode==3)
//		{
//			if(begin_flag==1)
//			{
//				tast3();
//			}
//		
//		}
//		else if(mode==4)
//		{
//			if(begin_flag==1)
//			{
//				tast4();
//			}
//		
//		}
//		PID_position(0,Error_huidu,Kp_P,Ki_P,Kd_P);
///////////////////每10ms读取灰度值和角度////////////////////////
		if(glfage_10ms)
		{
			glfage_10ms=0;
			Digtal=IIC_Get_Digtal();
			huidu[0]=(Digtal>>0)&0x01;
			huidu[1]=(Digtal>>1)&0x01;
			huidu[2]=(Digtal>>2)&0x01;
			huidu[3]=(Digtal>>3)&0x01;
			huidu[4]=(Digtal>>4)&0x01;
			huidu[5]=(Digtal>>5)&0x01;
			huidu[6]=(Digtal>>6)&0x01;
			huidu[7]=(Digtal>>7)&0x01;
			JY61P_Data = get_angle();
			bias=JY61P_Data->z;
			Error_huidu = Track_Err(Digtal);//计算位置误差值
//			L_speed_actual=get_encoder_count_L();//读取左电机实际速度
//			R_speed_actual=get_encoder_count_R();//读取右电机实际速度
		}
//		Digtal=IIC_Get_Digtal();
//		printf("%d\r\n",Digtal);

//		Gyro_Struct *JY61P_Data = get_angle();
//		Error_huidu = Track_Err(Digtal);//计算位置误差值
//		PID_position(0,Error_huidu,Kp_P,Ki_P,Kd_P);//设定左右电机的目标速度
//		
//		L_speed_actual=get_encoder_count_L();//读取左电机实际速度
//		R_speed_actual=get_encoder_count_R();//读取右电机实际速度

//		PID_Speed_Change_L(Taget_speed_L,(-L_speed_actual)*125,Kp_L,Ki_L,Kd_L);//控制左电机速度
//		PID_Speed_Change_R(Taget_speed_R,R_speed_actual*125,Kp_R,Ki_R,Kd_R);//控制右电机速度
//		printf("%d,%d,%d,%d\r\n",Taget_speed_L,L_speed_actual,Taget_speed_R,R_speed_actual);
//		printf("%d,%d\r\n",CompareA,CompareB);

//		delay_ms(5);
		
	}
}

void tast1()   // 直走: 黑线循迹 → 白区陀螺直行 → 黑线循迹 → 停车
  {
      static uint8_t phase = 0;           // 0=第一段黑线, 1=白区直行, 2=第二段黑线, 3=停车
      static uint8_t debounce = 0;        // 去抖计数
      static float lock_heading = 0;      // 进入白区时锁定的航向
      static uint8_t init_done = 0;

      if (!init_done) {
          phase = 0;
          debounce = 0;
          init_done = 1;
      }

      // 读陀螺仪并补偿零漂
      JY61P_Data = get_angle();
      float bias = JY61P_Data->z - gyro_offset;

      // 判断传感器状态
      uint8_t all_white = (huidu[0]==1 && huidu[1]==1 && huidu[2]==1 && huidu[3]==1
                        && huidu[4]==1 && huidu[5]==1 && huidu[6]==1 && huidu[7]==1);

      switch (phase) {

      case 0:   // 第一段黑线：循迹
          PID_position(0, Error_huidu, Kp_P, Ki_P, Kd_P);

          if (all_white) {
              debounce++;
              if (debounce >= 50) {           // 连续5次(50ms)全白才确认
                  debounce = 0;
                  lock_heading = bias;       // ★ 锁定当前航向角
                  phase = 1;
              }
          } else {
              debounce = 0;
          }
          break;

      case 1:   // 白区：陀螺仪直行
          PID_angle(lock_heading, bias, Kp_A, Ki_A, Kd_A);

          if (!all_white) {                  // 检测到黑线
              debounce++;
              if (debounce >= 50) {
                  debounce = 0;
                  phase = 2;
              }
          } else {
              debounce = 0;
          }
          break;

      case 2:   // 第二段黑线：循迹
          PID_position(0, Error_huidu, Kp_P, Ki_P, Kd_P);

          if (all_white) {                   // 走完第二段黑线
              debounce++;
              if (debounce >= 50) {
                  debounce = 0;
                  phase = 3;
              }
          } else {
              debounce = 0;
          }
          break;

      case 3:   // 停车
          TB6612_Motor_Stop();
          begin_flag = 0;
          phase = 0;
          init_done = 0;                     // 重置，下次可再启动
          break;
      }
  }
void tast2()//绕一圈
{
	unsigned char flag=0;//状态,导航--0，循迹--1
	unsigned char flag_last=0;//状态,导航--0，循迹--1

	if (huidu[0]!=1 || huidu[1]!=1 || huidu[2]!=1 || huidu[3]!=1 || huidu[4]!=1 || huidu[5]!=1 || huidu[6]!=1 || huidu[7]!=1 )  flag=1;  //判断循迹还是到导航
	else  flag=0;
	
	if(flag!=flag_last) count++;
	
	if(count == 4)                                //count为模式切换计数值，用于判断是否跑完全程
    {
        TB6612_Motor_Stop();
		begin_flag=0;
		count=0;
    }
	
	if(flag==0)
	{	
		flag_last=0;
		if(bias>165||bias<-165)			
		{
			if(bias>165)				PID_angle(0,fabs(bias)-180,Kp_A,Ki_A,Kd_A);
			if(bias<-165)				PID_angle(0,180-fabs(bias),Kp_A,Ki_A,Kd_A);
		}	
		if(bias>-15&&bias<15) 			PID_angle(0,bias,Kp_A,Ki_A,Kd_A);
		delay_ms(10);
	}
	
	else if(flag==1)
	{
		flag_last=1;
		PID_position(0,Error_huidu,Kp_P-1.1,Ki_P,Kd_P);//设定左右电机的目标速度
		PID_Speed_Change_L(Taget_speed_L,(-L_speed_actual)*125,Kp_L,Ki_L,Kd_L);//控制左电机速度
		PID_Speed_Change_R(Taget_speed_R,R_speed_actual*125,Kp_R,Ki_R,Kd_R);//控制右电机速度
		delay_ms(10);
	}
}
void tast3()//走8字
{
	static unsigned char flag;	//状态,导航--0，循迹--1
	float bias;			//实际角度
	bias = JY61P_Data->z  - gyro_offset;
	
	if ((bias > -80 && bias < 80) && (huidu[0] != 1 || huidu[1] != 1 || huidu[2] != 1 || huidu[3] != 1))
	{
		flag = 1; // 右侧区域，仅左侧4个灰度传感器触发循迹
	}
	else if (((bias > -180 && bias < -100) || (bias > 100 && bias < 180)) && (huidu[4] != 1 || huidu[5] != 1 || huidu[6] != 1 || huidu[7] != 1))
	{
		flag = 1; // 左侧区域，仅右侧4个灰度传感器触发循迹
	}    
	else if(huidu[0]==1 && huidu[1]==1 && huidu[2]==1 && huidu[3]==1 && huidu[4]==1 && huidu[5]==1 && huidu[6]==1 && huidu[7]==1 )   flag=0;  // 导航状态
	
	if(flag==0)
	{
		if(bias>-80&&bias<80)									PID_angle(-40.5,bias,Kp_A+2,Ki_A,Kd_A);
		else if((bias>-180&&bias<-100)||(bias>100&&bias<180))	PID_angle(-144,bias,Kp_A+2,Ki_A,Kd_A);
		
		PID_Speed_Change_L(Taget_speed_L_angle,(-L_speed_actual)*125,Kp_L,Ki_L,Kd_L);//控制左电机速度
		PID_Speed_Change_R(Taget_speed_R_angle,R_speed_actual*125,Kp_R,Ki_R,Kd_R);//控制右电机速度
		
		delay_ms(10);
	}
	else if(flag==1)
	{
		PID_position(0,Error_huidu,Kp_P-0.1,Ki_P,Kd_P);//设定左右电机的目标速度
		PID_Speed_Change_L(Taget_speed_L,(-L_speed_actual)*125,Kp_L,Ki_L,Kd_L);//控制左电机速度
		PID_Speed_Change_R(Taget_speed_R,R_speed_actual*125,Kp_R,Ki_R,Kd_R);//控制右电机速度
		delay_ms(10);
	}
}
void tast4()//走4圈8字
{
	unsigned char flag;//状态,导航--0，循迹--1
	float bias;//目标角度
	bias=JY61P_Data->z;	
	
	
	if (huidu[0]!=1 || huidu[1]!=1 || huidu[2]!=1 || huidu[3]!=1 || huidu[4]!=1 || huidu[5]!=1 || huidu[6]!=1 || huidu[7]!=1 )  flag=1; 
    else   flag=0;  // 导航状态

    if(count == 4)                                //count为模式切换计数值，用于判断是否跑完全程
    {
		TB6612_Motor_Stop();
		begin_flag=0;
		count=0;
    }
	
	
	if(flag==0)
	{
		
//		if((bias>-180&&bias<-100)||(bias>100&&bias<180))	
//		{
//			if(bias>-180&&bias<-100)		PID_angle(-138,bias,Kp_A,Ki_A,Kd_A);
//			else if(bias>100&&bias<180)		PID_angle(-138,-276-bias,Kp_A,Ki_A,Kd_A);
//		}
		
		if((bias>-180&&bias<-100)||(bias>100&&bias<180))	PID_angle(-139,bias,Kp_A,Ki_A,Kd_A);
		else if(bias>-80&&bias<80)							PID_angle(-40,bias,Kp_A,Ki_A,Kd_A);
		
		PID_Speed_Change_L(Taget_speed_L_angle,(-L_speed_actual)*125,Kp_L,Ki_L,Kd_L);//控制左电机速度
		PID_Speed_Change_R(Taget_speed_R_angle,R_speed_actual*125,Kp_R,Ki_R,Kd_R);//控制右电机速度
		
		delay_ms(10);
	}
	else if(flag==1)
	{
		PID_position(0,Error_huidu,Kp_P-0.36,Ki_P,Kd_P);//设定左右电机的目标速度
		PID_Speed_Change_L(Taget_speed_L,(-L_speed_actual)*125,Kp_L,Ki_L,Kd_L);//控制左电机速度
		PID_Speed_Change_R(Taget_speed_R,R_speed_actual*125,Kp_R,Ki_R,Kd_R);//控制右电机速度
		delay_ms(10);
	}
}

//定时器的中断服务函数 已配置为1ms的周期
void TIMER_0_INST_IRQHandler(void)
{
    switch( DL_TimerG_getPendingInterrupt(TIMER_0_INST) )//如果产生了定时器中断
    {
        case DL_TIMER_IIDX_ZERO://如果是0溢出中断
		glfage_1ms++;
		if(glfage_1ms>60000) glfage_1ms = 0;
		if(glfage_1ms%5 == 0)glfage_5ms = 1;
		if(glfage_1ms%10 == 0)glfage_10ms = 1;
		if(glfage_1ms%20 == 0)glfage_20ms = 1;
		if(glfage_1ms%500 == 0)glfage_500ms = 1;
		if(glfage_1ms%1000 == 0)glfage_1000ms = 1; 
        break;
        default:break;//其他的定时器中断  
    }
}

//#define SENSOR_COUNT 8
// const int weights[SENSOR_COUNT] = {
//    -70, -60, -5, -2, 2, 5, 60, 70
//};


float Track_Err(uint8_t car_state) 
{
	float Err;
    switch (car_state) 
    {
		case 0xe7: Err = 0; break;      // 1110 0111 - 正中间
        case 0xcf: Err = 3.5; break;    // 1100 1111 - 右侧小偏差 - 减小值使转向更平缓
        case 0x9f: Err = 4.0; break;    // 1001 1111 - 右侧中等偏差
        case 0x3f: Err = 5.0; break;    // 0011 1111 - 右侧大偏差 - 增大值使转向更果断
        case 0xf3: Err = -3.5; break;   // 1111 0011 - 左侧小偏差 - 减小值使转向更平缓
        case 0xf9: Err = -4.0; break;   // 1111 1001 - 左侧中等偏差
        case 0xfc: Err = -5.0; break;   // 1111 1100 - 左侧大偏差 - 增大值使转向更果断
        case 0xef: Err = 2.0; break;    // 1110 1111 - 右侧轻微偏差
        case 0xdf: Err = 3.0; break;    // 1101 1111 - 右侧中小偏差
        case 0xbf: Err = 3.5; break;    // 1011 1111 - 右侧中小偏差
        case 0x7f: Err = 5.5; break;    // 0111 1111 - 右侧极大偏差 - 大幅增大值
        case 0xf7: Err = -2.0; break;   // 1111 0111 - 左侧轻微偏差
        case 0xfb: Err = -3.0; break;   // 1111 1011 - 左侧中小偏差
        case 0xfd: Err = -3.5; break;   // 1111 1101 - 左侧中偏差
        case 0xfe: Err = -5.5; break;   // 1111 1110 - 左侧极大偏差 - 大幅增大值
        case 0x1f: Err = 8.0; break;    // 0001 1111 - 右侧极限偏差 - 特别大的值
        case 0xf8: Err = -3.0; break;   // 1111 1000 - 左侧中小偏差
        case 0x8f: Err = 9.0; break;    // 1000 1111 - 右侧极限偏差 - 特别大的值
        default: Err = 0; break;        // 未定义情况
    }
    return Err;
}


void PID_position(float target_value, float temp_val, float kp, float ki, float kd)
{	
	float err_x = 0.0;										//x误差
	float integral = 0.0;									//x积分
	float err_last = 0.0;									//x上一误差值
	float actual_value = 0.0;								//驱动值
	/*位置式PID*/
	/*误差 = 目标值 - 输入的实际值*/
	err_x = target_value - temp_val;

	/*误差累计*/
	integral += err_x;

	/*积分限幅*/
	if(integral > 1000)
		integral = 1000;
	else if(integral < -1000)
		integral = -1000;

	/*PID算法实现*/
	actual_value = kp*err_x + ki*integral + kd*(err_x - err_last);
	
	/*记录上一误差值*/
	err_last = err_x;
	
	/*计算目标速度*/
	Taget_speed_L=DEFAULT_DUTY_L-actual_value*100;
	Taget_speed_R=DEFAULT_DUTY_R+actual_value*100;
	
	BO_Control(1,Taget_speed_L);
	AO_Control(1,Taget_speed_R);
	
}

void PID_angle(float target, float current, float kp, float ki, float kd) 
{
    static float integral = 0, last_err = 0;
    
    // 角度误差规范化（处理-180°到+180°跳变）
    float err = target - current;
    if(err > 180) err -= 360;
    else if(err < -180) err += 360;
    
    // 积分限幅
    integral += err;
    if(integral > 100) integral = 100;
    else if(integral < -100) integral = -100;
    
    // 完整PID计算
    float output = kp * err + ki * integral + kd * (err - last_err);
    last_err = err;
    
    // 输出限幅
    output = (output > 1000) ? 1000 : (output < -1000) ? -1000 : output;
    
    Taget_speed_L_angle = DEFAULT_DUTY_L - output;
    Taget_speed_R_angle = DEFAULT_DUTY_R + output;
	
	BO_Control(1,Taget_speed_L_angle);
	AO_Control(1,Taget_speed_R_angle);
}	