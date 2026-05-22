#ifndef __pid_H
#define __pid_H

typedef struct
{
	float target_val;     //目标值
	float Error;        //第 k 次偏差 
	float LastError;    //Error[-1],第 k-1 次偏差 
	float PrevError;    //Error[-2],第 k-2 次偏差 
	float Kp,Ki,Kd;     //比例、积分、微分系数
	float integral;     //积分值
	float output_val;   //输出值
	float integral_limit;//积分限幅值
	float outmax;		//最大输出值
}PID;

void PID_param_init(PID *pid,float kp,float ki,float kd,float limit,float outmax);
void addPID_realize(PID *pid, float actual_val);
void PosionPID_realize(PID *pid, float actual_val);
void set_PID(float p, float i, float d,PID *pid);
int get_pid_target(PID *pid);
void set_PID_target(int temp_val,PID *pid);
float PID_x(float target_value_x, float temp_val_x, float kp_x, float ki_x, float kd_x);
float PID_y(float target_value_y, float temp_val_y, float kp_y, float ki_y, float kd_y);

#endif
