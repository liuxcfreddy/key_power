
#include "intrins.h"
#include  "EncoderEC11.h"
#include <STC89C5xRC.H>
#include "adc0832.h"
#include "bin.h"
/**************字符库********************/
code unsigned char smg_ca[14] = {    //共阳极
    0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x90, 0xff, 0xc6, 0x8c, 0x88};
code unsigned  char smg_ck[16]={     //共阴极
    0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x39,0x5e,0x79,0x71};
code unsigned char smgdot_ca[10] = {//带点
    0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x00, 0x10};
//---------------IO定义-----------------//
sbit Volt_Pwm = P1^6;
#define Smg_IO P2
//---------------变量定义---------------//
unsigned int Volt_Time =100; //周期1000us
unsigned int Volt_OutPut=330;//目标电压
unsigned int Volt_Chek; //检测电压
void Smg_Show(unsigned int Temp , unsigned int Temp1);
int output;
int chek;

void Delay500us()		//@12.000MHz
{
	unsigned char i;

	_nop_();
	i = 247;
	while (--i);
}



/***********************************************************
* 名    称：InitTimer0()
* 功    能：电压控制
* 入口参数：无
* 出口参数：无
* 说    明：12M晶振，12分频，所以计数器每递增一个数就是1微秒，完全满足舵机控制的精度要求
            因为定时器是TH0，TL0都要全部计数到0xFF后在计1个数就会产生中断，所以要想产生
            x毫秒的中断，那么TH0，TL0就应该赋值（0xFFFF-x）	从这个值开始计数产生定时中断
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
void Volt_Init() //舵机初始化
{
    InitTimer0();
}

/***********************************************************
* 名    称：Timer0Value(uint16 pwm)
* 功    能：给定时器0计数器赋值产生定时中断
* 入口参数：pwm
* 出口参数：无
* 说    明：12M晶振，12分频，所以计数器每递增一个数就是1微秒，完全满足舵机控制的精度要求
            因为定时器是TH0，TL0都要全部计数到0xFF后在计1个数就会产生中断，所以要想产生
            pwm毫秒的中断，那么TH0，TL0就应该赋值（0xFFFF-pwm）	从这个值开始计数产生定时中断
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
        Timer0Value(1200-Volt_Time);
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
    //TMOD &= 0x0F; //设置定时器模式
    //TMOD |= 0x10; //设置定时器模式
    AUXR &= 0xBF;		//定时器时钟12T模式
	TMOD &= 0x0F;		//设置定时器模式
	TL1 = 0x24;		//设置定时初始值
	TH1 = 0xFA;		//设置定时初始值
    TF1 = 0;      //清除TF1标志
    TR1 = 1;      //定时器1开始计时
    ET1 = 1;      //开定时器1中断
    EA = 1;       //开总中断
}
void EC11_Init(void)//EC11编码器初始化
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

void Timer2Init(void)		//100微秒@12.000MHz
{
	T2MOD = 0;     //初始化模式寄存器
    T2CON = 0;     //初始化控制寄存器
    TL2 = 0x9C;		//设置定时初始值
	TH2 = 0xFF;		//设置定时初始值
	RCAP2L = 0x9C;		//设置定时重载值
	RCAP2H = 0xFF;		//设置定时重载值
    TR2 = 1;       //定时器2开始计时
    PT2 = 0;
    ET2 = 1;
    EA = 1;
}
void Timer2sir() interrupt 5
{
    TR2=0;
    RCAP2L = 0x9C;		//设置定时重载值
	RCAP2H = 0xFF;		//设置定时重载值
    TR2=1;
    
    
}



sbit A0 = P4^0;
sbit A1 = P4^1;
sbit A2 = P4^2;
void Smg_Show(unsigned int Temp , unsigned int Temp1)
{

    A0 = 0;
    A1=0;
    A2=0;
    Smg_IO=smg_ca[(Temp/1000)%10];Delay500us();
    Smg_IO=0xff;

    A0=1;
    A1=0;
    A2=0;   
    Smg_IO=smgdot_ca[(Temp/100)%10];Delay500us();
    Smg_IO=0xff;

    A0=0;
    A1=1;
    A2=0; 
    Smg_IO=smg_ca[(Temp/10)%10];Delay500us();
    Smg_IO=0xff;

    A0=1;
    A1=1;
    A2=0;
    Smg_IO=smg_ca[(Temp)%10];Delay500us();
    Smg_IO=0xff;


//检测电压显示
    A0=0;
    A1=0;
    A2=1; 
    Smg_IO=smg_ca[(Temp1/1000)%10];Delay500us();
    Smg_IO=0xff;
	
    A0=1;
    A1=0;
    A2=1;   
    Smg_IO=smgdot_ca[(Temp1/100)%10];Delay500us();
    Smg_IO=0xff;

    A0=0;
    A1=1;
    A2=1; 
    Smg_IO=smg_ca[(Temp1/10)%10];Delay500us();
    Smg_IO=0xff;

    A0=1;
    A1=1;
    A2=1;
    Smg_IO=smg_ca[(Temp1)%10];Delay500us();
    Smg_IO=0xff;

}

void main()
{
	
    Volt_Init();
    EC11_Init(); 
    //Timer2Init();  
    while (1)
    {
    Smg_Show(output,chek);
    Volt_Chek=read0832();
    output=Volt_OutPut;
    chek=(Volt_Chek*100/51);
    if((output)>chek)//output>chek
    {
        if(((chek-Volt_OutPut)<-2)||((chek-Volt_OutPut)>2))
        {
            Volt_Time++;
            if(Volt_Time<=10)
            {Volt_Time=10;}
        } 
        
    }
    else
    {
        if(((chek-Volt_OutPut)<-2)||((chek-Volt_OutPut)>2))
        {
            Volt_Time--;
            if(Volt_Time>=1200)
            {Volt_Time=1200;}
        } 
    }
    }
    
}
