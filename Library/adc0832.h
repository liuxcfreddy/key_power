#ifndef __adc0832_h_
#define __adc0832_h_

#include <STC89C5xRC.H>

sbit CS_0832 = P1^3;  //ʹ�ܶ�
sbit CLK_0832 = P3^5;//ʱ�ӽӿ�
sbit DO_0832 = P1^5;	// �������DI��DO��ͬʱ��Ч���ɹ���һ���ӿ�
sbit DI_0832 = P1^4;  //ͨ��ѡ��

unsigned char read0832();

#endif