#include "adc0832.h"

#include "intrins.h"
void Delay50us()		//@12.000MHz
{
	unsigned char i;

	_nop_();
	i = 22;
	while (--i);
}

void pulse0832(){ 
	Delay50us();
	CLK_0832=1;
	Delay50us();
	CLK_0832=0;
}  //���庯��

//��ģ���ѹֵת����8λ��������������
unsigned char read0832()
{
	unsigned char i, ch = 0, ch1 = 0;
	CS_0832=0;		// Ƭѡ��DOΪ����̬
	
	DI_0832=1;
	// �˴���ͣT-SetUp: 250ns (��pulse0832���)
	pulse0832();	// ��һ�����壬��ʼλ��DI�ø�
	
	DI_0832=1;
	pulse0832();	// �ڶ������壬DI=1��ʾ˫ͨ������������
	
	DI_0832=0;
	pulse0832();	// ���������壬DI=1��ʾѡ��ͨ��1��CH2��
	
	// 51��Ƭ��Ϊ׼˫��IO�ڣ�Ӧ��д��1�ٶ�ȡ
	DI_0832=1;
	
	// MSB FIRST DATA
	for(i = 0; i < 8; ++i) {
		pulse0832();
		ch <<= 1;
		if(DO_0832==1)
			ch |= 0x01;
	}
	
	// MSB FIRST��������һλ��LSB FIRST����ĵ�һλ����
	// ͬһ��ʱ���½���֮�󣬹ʴ˴���ִ�ж�ȡ����ִ��pulse
	// LSB FIRST DATA
	for(i = 0; i < 8; ++i) {
		ch1 >>= 1;
		if(DO_0832==1)
			ch1 |= 0x80;
		pulse0832();
	}
	
	CS_0832=1;		// ȡ��Ƭѡ��һ��ת�����ڽ���
	return (ch==ch1) ? ch : 0;		// ����ת�����
}