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
}  //定义函数

//把模拟电压值转换成8位二进制数并返回
unsigned char read0832()
{
	unsigned char i, ch = 0, ch1 = 0;
	CS_0832=0;		// 片选，DO为高阻态
	
	DI_0832=1;
	// 此处暂停T-SetUp: 250ns (由pulse0832完成)
	pulse0832();	// 第一个脉冲，起始位，DI置高
	
	DI_0832=0;
	pulse0832();	// 第二个脉冲，DI=1（DI第一位）表示双通道单极性输入
	
	DI_0832=0;
	pulse0832();	// 第三个脉冲，DI=1（DI第二位）表示选择通道1（CH2）
					/*当此2位数据为“1”、“0”时，只对CH0进行单通道转换。

					当2位数据为“1”、“1”时，只对CH1进行单通道转换。

					当2位数据为“0”、“0”时，将CH0作为正输入端IN+，CH1作为负输入端IN-进行输入。

					当2位数据为“0”、“1”时，将CH0作为负输入端IN-，CH1作为正输入端IN+进行输入。
					*/
	
	
	// 51单片机为准双向IO口：应先写入1再读取
	DI_0832=1;
	
	// MSB FIRST DATA
	for(i = 0; i < 8; ++i) {
		pulse0832();
		ch <<= 1;
		if(DO_0832==1)
			ch |= 0x01;
	}
	
	// MSB FIRST输出的最后一位与LSB FIRST输出的第一位是在
	// 同一个时钟下降沿之后，故此处先执行读取，后执行pulse
	// LSB FIRST DATA
	for(i = 0; i < 8; ++i) {
		ch1 >>= 1;
		if(DO_0832==1)
			ch1 |= 0x80;
		pulse0832();
	}
	
	CS_0832=1;		// 取消片选，一个转换周期结束
	return (ch==ch1) ? ch : 0;		// 返回转换结果
}