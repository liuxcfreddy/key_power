#ifndef _PID_H_
#define _PID_H_
struct _pid
{
	float SetSpeed; //定义设定值
	float ActualSpeed; //定义实际值
	float err; //定义偏差值
	float err_next; //定义上一个偏差值
	float err_last; //定义最上前的偏差值
	float Kp,Ki,Kd; //定义比例、积分、微分系数
	
	float incrementSpeed;
}pid;

void PID_init();
float PID_realize(float speed);