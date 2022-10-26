
#include "PID.h"
extern	float incrementSpeed;
void PID_init()
{
	pid.SetSpeed=0.0;
	pid.ActualSpeed=0.0;
	pid.err=0.0;
	pid.err_last=0.0;
	pid.err_next=0.0;
	pid.Kp=0.2;//��������
	pid.Ki=0.015;//����ʱ�䳣��
	pid.Kd=0.2;//΢��ʱ�䳣��
}
 
float PID_realize(float speed)
{
	pid.SetSpeed=speed;
	pid.err=pid.SetSpeed-pid.ActualSpeed;
	incrementSpeed=pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last);
	pid.ActualSpeed+=incrementSpeed;
	pid.err_last=pid.err_next;
	pid.err_next=pid.err;
 
return pid.ActualSpeed;
}