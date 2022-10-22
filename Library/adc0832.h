#ifndef __adc0832_h_
#define __adc0832_h_

#include <STC89C5xRC.H>

sbit CS_0832 = P1^3;  //使能端
sbit CLK_0832 = P3^5;//时钟接口
sbit DO_0832 = P1^5;	// 数据输出DI、DO不同时有效，可共用一个接口
sbit DI_0832 = P1^4;  //通道选择

unsigned char read0832();

#endif