
#include "intrins.h"
#include "EncoderEC11.h"
#include <STC89C5xRC.H>
#include "adc0832.h"
#include "bin.h"
#define Smg_IO P2

/**************�ַ���********************/
code unsigned char smg_ca[14] = { //������
    0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x90, 0xff, 0xc6, 0x8c, 0x88};
code unsigned char smg_ck[16] = { //������
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71};
code unsigned char smgdot_ca[10] = { //����
    0x40, 0x79, 0x24, 0x30, 0x19, 0x12, 0x02, 0x78, 0x00, 0x10};
//---------------IO����-----------------//
sbit Volt_Pwm = P1 ^ 6;
sbit A0 = P4 ^ 0;
sbit A1 = P4 ^ 1;
sbit A2 = P4 ^ 2;
//---------------��������---------------//
unsigned int Volt_Time = 600;   //����1000us
unsigned int Volt_OutPut = 300; //Ŀ���ѹ
unsigned int Volt_Chek;         //����ѹ
void Smg_Show(unsigned int Temp, unsigned int Temp1);
unsigned int output;
unsigned int chek;
long int change;
/***************/
float SetVoltage;    //�����趨ֵ
float ActualVoltage; //����ʵ��ֵ
float err;           //����ƫ��ֵ
float err_last;      //������һ��ƫ��ֵ
float Kp, Ki, Kd;    //������������֡�΢��ϵ��
float result;        // pid������
float voltage;       //�����ѹֵ������ִ�����ı�����0-5v��ת 5-10v��ת
float integral;      //�������ֵ
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
* ��    �ƣ�InitTimer0()
* ��    �ܣ���ѹ����
* ��ڲ�������
* ���ڲ�������
* ˵    ����12M����12��Ƶ�����Լ�����ÿ����һ��������1΢��
            ��Ϊ��ʱ����TH0��TL0��Ҫȫ��������0xFF���ڼ�1�����ͻ�����жϣ�����Ҫ�����
            x������жϣ���ôTH0��TL0��Ӧ�ø�ֵ��0xFFFF-x�� �����ֵ��ʼ����������ʱ�ж�
/**********************************************************/
void InitTimer0(void) // 1us
{

    TMOD &= 0xF0; //���ö�ʱ��ģʽ
    TMOD |= 0x01; //���ö�ʱ��ģʽ
    TL0 = 0xE0;   //���ö�ʱ��ֵ
    TH0 = 0xB1;   //���ö�ʱ��ֵ
    TF0 = 0;      //���TF0��־
    TR0 = 1;      //��ʱ��0��ʼ��ʱ
    ET0 = 1;      //����ʱ��0�ж�
    EA = 1;       //�����ж�
}
void Volt_Init() //��ʱ����ʼ��
{
    InitTimer0();
}

/***********************************************************
* ��    �ƣ�Timer0Value(uint16 pwm)
* ��    �ܣ�����ʱ��0��������ֵ������ʱ�ж�
* ��ڲ�����pwm
* ���ڲ�������
* ˵    ����12M����12��Ƶ�����Լ�����ÿ����һ��������1΢��
            ��Ϊ��ʱ����TH0��TL0��Ҫȫ��������0xFF���ڼ�1�����ͻ�����жϣ�����Ҫ�����
            pwm������жϣ���ôTH0��TL0��Ӧ�ø�ֵ��0xFFFF-pwm�� �����ֵ��ʼ����������ʱ�ж�
/**********************************************************/
void Timer0Value(unsigned int pwm)
{
    unsigned int value;
    value = 0xffff - pwm;
    TR0 = 0;
    TL0 = value;      // 16λ���ݸ�8λ���ݸ�ֵĬ�Ͻ�16λ���ݵĵͰ�λֱ�Ӹ�����λ����
    TH0 = value >> 8; //��16λ��������8λ��Ҳ���ǽ���8λ�Ƶ��Ͱ�λ���ٸ�ֵ��8λ����
    TR0 = 1;
}

/***********************************************************
* ��    �ƣ� Timer0_isr() interrupt 1 using 1
* ��    �ܣ� ʱ��0�жϴ���
* ��ڲ����� ��
* ���ڲ����� ��
* ˵    ����
/**********************************************************/
void Timer0_isr(void) interrupt 1 using 1
{
    static unsigned char i = 1; //��̬������ÿ�ε��ú���ʱ������һ��������ֵ��
    //��ȫ�ֱ������ƣ���ͬ����ֻ�����ڴ˺����ڲ�
    switch (i)
    {
    case 1:
        Volt_Pwm = 0; // PWM���ƽŸߵ�ƽ
        //����ʱ��0��ֵ������Pwm0Duty�����������жϣ��´��жϻ������һ��case���
        Timer0Value(Volt_Time);
        break;
    case 2:
        Volt_Pwm = 1; // PWM���ƽŵ͵�ƽ
        //�����������ʣ�µ�ʱ��(20000-Pwm0Duty)ȫ�ǵ͵�ƽ�ˣ�Pwm0Duty + (20000-Pwm0Duty) = 20000����������Ϊһ������20����
        Timer0Value(1200 - Volt_Time);
        i = 0;
        break;
    }
    i++;
}

//��������ʱ������

/***********************************************************
* ��    �ƣ�Timer1Init
* ��    �ܣ���������ʼ��
* ��ڲ�������
* ���ڲ�������
* ˵    ����2msһ����
/**********************************************************/
void Timer1Init(void) // 4����@12.000MHz
{
    // TMOD &= 0x0F; //���ö�ʱ��ģʽ
    // TMOD |= 0x10; //���ö�ʱ��ģʽ
    AUXR &= 0xBF; //��ʱ��ʱ��12Tģʽ
    TMOD &= 0x0F; //���ö�ʱ��ģʽ
    TL1 = 0x24;   //���ö�ʱ��ʼֵ
    TH1 = 0xFA;   //���ö�ʱ��ʼֵ
    TF1 = 0;      //���TF1��־
    TR1 = 1;      //��ʱ��1��ʼ��ʱ
    ET1 = 1;      //����ʱ��1�ж�
    EA = 1;       //�����ж�
}
void EC11_Init(void) // EC11��������ʼ��
{
    Timer1Init();
    Encoder_EC11_Init(0);
}
/********************************************************
* ��    �ƣ� Timer1_isr() interrupt 3 using 3
* ��    �ܣ� ʱ��0�жϴ���
* ��ڲ����� ��
* ���ڲ����� ��
* ˵    ���� ��
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

    //����ѹ��ʾ
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

    //----------��ֹ����----------//
    if (Volt_OutPut >= 1200)
    {
        Volt_OutPut = 1200;
    }
    if (Volt_OutPut <= 100)
    {
        Volt_OutPut = 100;
    }
}
/*********PID������ʼ��********/
void PID_Init()
{
    SetVoltage = 0.0;    // �趨��Ԥ�ڵ�ѹֵ
    ActualVoltage = 0.0; // adcʵ�ʵ�ѹֵ
    err = 0.0;           // ��ǰ��ʵ���������ƫ��
    err_last = 0.0;      // ��һ�ε�ƫ��
    voltage = 0.0;       // ���Ƶ�ѹֵ
    integral = 0.0;      // ����ֵ
    Kp = 0.03;           // ����ϵ��
    Ki = 0.000001;       // ����ϵ��
    Kd = 0.001;          // ΢��ϵ��
}
float PID_realize(float v, float v_r)
{
    SetVoltage = v;                                            // �̶���ѹֵ����
    ActualVoltage = v_r;                                       // ʵ�ʵ�ѹ���� = ADC_Value * 3.3f/ 4096
    err = SetVoltage - ActualVoltage;                          //����ƫ��
    integral += err;                                           //�������
    result = Kp * err + Ki * integral + Kd * (err - err_last); //λ��ʽ��ʽ
    err_last = err;                                            //��ס��һ�����
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
