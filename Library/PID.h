#ifndef _PID_H_
#define _PID_H_
struct _pid
{
	float SetSpeed; //�����趨ֵ
	float ActualSpeed; //����ʵ��ֵ
	float err; //����ƫ��ֵ
	float err_next; //������һ��ƫ��ֵ
	float err_last; //��������ǰ��ƫ��ֵ
	float Kp,Ki,Kd; //������������֡�΢��ϵ��
	
	float incrementSpeed;
}pid;

void PID_init();
float PID_realize(float speed);