
#include "intrins.h"
#include "EncoderEC11.h"
#include <STC89C5xRC.H>
#include "adc0832.h"
#include "bin.h"
#define Smg_IO P2

/**************字符库********************/
code unsigned char smg_ca[14] = { //共阳极
    0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x90, 0xff, 0xc6, 0x8c, 0x88};
code unsigned char smg_ck[16] = { //共阴极
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71};
code unsigned char smgdot_ca[10] = { //带点
    0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x00, 0x10};
//---------------IO定义-----------------//
sbit Volt_Pwm = P1 ^ 6;
sbit A0 = P4 ^ 0;
sbit A1 = P4 ^ 1;
sbit A2 = P4 ^ 2;
//---------------变量定义---------------//
unsigned int Volt_Time = 600;   //周期1000us
unsigned int Volt_OutPut = 300; //目标电压
unsigned int Volt_Chek;         //检测电压
void Smg_Show(unsigned int Temp, unsigned int Temp1);
unsigned int output;
unsigned int chek;
long int change;
/***************/
float SetVoltage;    //定义设定值
float ActualVoltage; //定义实际值
float err;           //定义偏差值
float err_last;      //定义上一个偏差值
float Kp, Ki, Kd;    //定义比例、积分、微分系数
float result;        // pid计算结果
float voltage;       //定义电压值（控制执行器的变量）0-5v右转 5-10v左转
float integral;      //定义积分值
/*********************************  */
void Delay500us() //@12.000MHz
{
    unsigned char i;

    _nop_();
    i = 247;
    while (--i)
        ;
}
/***********************************************************
* 名    称：InitTimer0()
* 功    能：电压控制
* 入口参数：无
* 出口参数：无
* 说    明：12M晶振，12分频，所以计数器每递增一个数就是1微秒
            因为定时器是TH0，TL0都要全部计数到0xFF后在计1个数就会产生中断，所以要想产生
            x毫秒的中断，那么TH0，TL0就应该赋值（0xFFFF-x） 从这个值开始计数产生定时中断
/**********************************************************/
void InitTimer0(void) // 1us
{

    TMOD &= 0xF0; //设置定时器模式
    TMOD |= 0x01; //设置定时器模式
    TL0 = 0xE0;   //设置定时初值
    TH0 = 0xB1;   //设置定时初值
    TF0 = 0;      //清除TF0标志
    TR0 = 1;      //定时器0开始计时
    ET0 = 1;      //开定时器0中断
    EA = 1;       //开总中断
}
void Volt_Init() //定时器初始化
{
    InitTimer0();
}

/***********************************************************
* 名    称：Timer0Value(uint16 pwm)
* 功    能：给定时器0计数器赋值产生定时中断
* 入口参数：pwm
* 出口参数：无
* 说    明：12M晶振，12分频，所以计数器每递增一个数就是1微秒
            因为定时器是TH0，TL0都要全部计数到0xFF后在计1个数就会产生中断，所以要想产生
            pwm毫秒的中断，那么TH0，TL0就应该赋值（0xFFFF-pwm） 从这个值开始计数产生定时中断
/**********************************************************/
void Timer0Value(unsigned int pwm)
{
    unsigned int value;
    value = 0xffff - pwm;
    TR0 = 0;
    TL0 = value;      // 16位数据给8位数据赋值默认将16位数据的低八位直接赋给八位数据
    TH0 = value >> 8; //将16位数据右移8位，也就是将高8位移到低八位，再赋值给8位数据
    TR0 = 1;
}

/***********************************************************
* 名    称： Timer0_isr() interrupt 1 using 1
* 功    能： 时钟0中断处理
* 入口参数： 无
* 出口参数： 无
* 说    明：
/**********************************************************/
void Timer0_isr(void) interrupt 1 using 1
{
    static unsigned char i = 1; //静态变量：每次调用函数时保持上一次所赋的值，
    //跟全局变量类似，不同是它只能用于此函数内部
    switch (i)
    {
    case 1:
        Volt_Pwm = 0; // PWM控制脚高电平
        //给定时器0赋值，计数Pwm0Duty个脉冲后产生中断，下次中断会进入下一个case语句
        Timer0Value(Volt_Time);
        break;
    case 2:
        Volt_Pwm = 1; // PWM控制脚低电平
        //高脉冲结束后剩下的时间(20000-Pwm0Duty)全是低电平了，Pwm0Duty + (20000-Pwm0Duty) = 20000个脉冲正好为一个周期20毫秒
        Timer0Value(1200 - Volt_Time);
        i = 0;
        break;
    }
    i++;
}

//编码器定时器部分

/***********************************************************
* 名    称：Timer1Init
* 功    能：编码器初始化
* 入口参数：无
* 出口参数：无
* 说    明：2ms一周期
/**********************************************************/
void Timer1Init(void) // 4毫秒@12.000MHz
{
    // TMOD &= 0x0F; //设置定时器模式
    // TMOD |= 0x10; //设置定时器模式
    AUXR &= 0xBF; //定时器时钟12T模式
    TMOD &= 0x0F; //设置定时器模式
    TL1 = 0x24;   //设置定时初始值
    TH1 = 0xFA;   //设置定时初始值
    TF1 = 0;      //清除TF1标志
    TR1 = 1;      //定时器1开始计时
    ET1 = 1;      //开定时器1中断
    EA = 1;       //开总中断
}
void EC11_Init(void) // EC11编码器初始化
{
    Timer1Init();
    Encoder_EC11_Init(0);
}
/********************************************************
* 名    称： Timer1_isr() interrupt 3 using 3
* 功    能： 时钟0中断处理
* 入口参数： 无
* 出口参数： 无
* 说    明： 无
/**********************************************************/

void Timer1_isr(void) interrupt 3 using 3
{
    Encoder_EC11_Analyze(Encoder_EC11_Scan());
}


void Smg_Show(unsigned int Temp, unsigned int Temp1)
{

    A0 = 0;
    A1 = 0;
    A2 = 0;
    Smg_IO = smg_ca[(Temp / 1000) % 10];
    Delay500us();
    Smg_IO = 0xff;

    A0 = 1;
    A1 = 0;
    A2 = 0;
    Smg_IO = smgdot_ca[(Temp / 100) % 10];
    Delay500us();
    Smg_IO = 0xff;

    A0 = 0;
    A1 = 1;
    A2 = 0;
    Smg_IO = smg_ca[(Temp / 10) % 10];
    Delay500us();
    Smg_IO = 0xff;

    A0 = 1;
    A1 = 1;
    A2 = 0;
    Smg_IO = smg_ca[(Temp) % 10];
    Delay500us();
    Smg_IO = 0xff;

    //检测电压显示
    A0 = 0;
    A1 = 0;
    A2 = 1;
    Smg_IO = smg_ca[(Temp1 / 1000) % 10];
    Delay500us();
    Smg_IO = 0xff;

    A0 = 1;
    A1 = 0;
    A2 = 1;
    Smg_IO = smgdot_ca[(Temp1 / 100) % 10];
    Delay500us();
    Smg_IO = 0xff;

    A0 = 0;
    A1 = 1;
    A2 = 1;
    Smg_IO = smg_ca[(Temp1 / 10) % 10];
    Delay500us();
    Smg_IO = 0xff;

    A0 = 1;
    A1 = 1;
    A2 = 1;
    Smg_IO = smg_ca[(Temp1) % 10];
    Delay500us();
    Smg_IO = 0xff;
}
void PID()
{
    if ((output) > chek) // output>chek
    {
        if (((chek - output) < -2) || ((chek - output) > 2))
        {

            if (Volt_Time >= 1200)
            {
                Volt_Time = 1200;
            }
            else
            {
                Volt_Time++;
            }
        }
    }
    else
    {
        if (((chek - output) < -2) || ((chek - output) > 2))
        {

            if (Volt_Time <= 10)
            {
                Volt_Time = 10;
            }
            else
            {
                Volt_Time--;
            }
        }
    }

    //----------防止超限----------//
    if (Volt_OutPut >= 1200)
    {
        Volt_OutPut = 1200;
    }
    if (Volt_OutPut <= 100)
    {
        Volt_OutPut = 100;
    }
}
/*********PID参数初始化********/
void PID_Init()
{
    SetVoltage = 0.0;    // 设定的预期电压值
    ActualVoltage = 0.0; // adc实际电压值
    err = 0.0;           // 当前次实际与理想的偏差
    err_last = 0.0;      // 上一次的偏差
    voltage = 0.0;       // 控制电压值
    integral = 0.0;      // 积分值
    Kp = 0.03;           // 比例系数
    Ki = 0.000001;       // 积分系数
    Kd = 0.001;          // 微分系数
}
float PID_realize(float v, float v_r)
{
    SetVoltage = v;                                            // 固定电压值传入
    ActualVoltage = v_r;                                       // 实际电压传入 = ADC_Value * 3.3f/ 4096
    err = SetVoltage - ActualVoltage;                          //计算偏差
    integral += err;                                           //积分求和
    result = Kp * err + Ki * integral + Kd * (err - err_last); //位置式公式
    err_last = err;                                            //留住上一次误差
    return result;
}
void main()
{

    Volt_Init();
    EC11_Init();
    PID_Init();    
    while (1)
    {
        Volt_Chek = read0832();
        chek = (Volt_Chek * 1.96);
        output = (Volt_OutPut / 2);
        Smg_Show(Volt_OutPut, chek * 2);
        change = PID_realize(Volt_OutPut, Volt_Chek * 4);
        Volt_Time += change;
        // PID();
        if (Volt_Time >= 1200)
        {
            Volt_Time = 1200;
        }
        if (Volt_Time <= 10)
        {
            Volt_Time = 10;
        }
    }
}
