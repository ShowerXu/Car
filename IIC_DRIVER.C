#include "iic_driver.h"
#include "sysdef.h"
/*
需要外部加上拉电阻
*/
void IIC_Init(void)
{
	CLRWDT();
	SET_SCL_OUT;
	SET_SDA_OUT;
	// WPUC2 = 1;		//内部若上拉，依据外部电路决定是否开启
	// WPUC4 = 1;
	SDA_H();
	SCL_H();
	__delay_us(I2C_SPEED);

}


void IIC_Start(void)
{
	CLRWDT();
	SET_SDA_OUT;
	//SET_SCL_OUT;
	SDA_H();
	asm("nop");
	SCL_H();
	__delay_us(I2C_SPEED);
	SDA_L();
	__delay_us(I2C_SPEED);	
	SCL_L();
}

void IIC_Stop(void)
{
	SET_SDA_OUT;
	//SET_SCL_OUT;
	CLRWDT();
	SCL_L();
	SDA_L();
	__delay_us(I2C_SPEED);
	SCL_H();
	SDA_H();
	__delay_us(I2C_SPEED);
}

void IIC_Ack(void)
{
	SCL_L();
	SET_SDA_OUT;
	//SET_SCL_OUT;
	CLRWDT();
	SDA_L();
	__delay_us(I2C_SPEED);
	SCL_H();
	__delay_us(I2C_SPEED);
	SCL_L();
}

void IIC_NAck(void)
{
	SCL_L();
	SET_SDA_OUT;
	//SET_SCL_OUT;
	CLRWDT();
	SDA_H();
	__delay_us(I2C_SPEED);
	SCL_H();
	__delay_us(I2C_SPEED);
	SCL_L();
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
unsigned char IIC_Wait_Ack(void)		
{  
	unsigned char ErrorCount=50;	//定义超时量为255次

	SET_SDA_IN;						//SDA as Input
	//SET_SCL_OUT;
	SDA_H();
	CLRWDT();
	__delay_us(I2C_SPEED);
	SCL_H();
	__delay_us(I2C_SPEED);
	while(SDA_READ)					//在一段时间内检测到IIC_SDA=0的话认为是应答信号  
	{
		if(ErrorCount==0)  
		{  
			IIC_Stop();
			return 1;
		}  
		ErrorCount--;  
	}
	SCL_L();						//钳住总线,为下1次通信做准备   
	SET_SDA_OUT;
	return 0;						//成功处理应答信号
}
//IIC发送一个字节
void IIC_Send_Byte(unsigned char WriteByte)
{
	unsigned char BitCount;

	SET_SDA_OUT;
	//SET_SCL_OUT;	
	CLRWDT();
	SCL_L();
	for(BitCount=0;BitCount<8;BitCount++)
	{
		
		if((WriteByte&0x80)==0x80) SDA_H();
		else SDA_L();
		WriteByte<<=1;
		asm("nop");
		SCL_H();
		asm("nop");
		__delay_us(I2C_SPEED);
		SCL_L();
		__delay_us(I2C_SPEED);
	}
}
//IIC读一个字节
//ack=1 发送ACK
//ack=0 发送NACK
unsigned char IIC_Read_Byte(unsigned char ack)
{
	unsigned char TempData=0,BitCount;

	SET_SDA_IN;				//SDA as Input
	//SET_SCL_OUT;
	CLRWDT();

	asm("nop");
	for(BitCount=0;BitCount<8;BitCount++)
	{
		SCL_L();
		__delay_us(I2C_SPEED);
		SCL_H();
		asm("nop");
		TempData<<=1;		
		if(SDA_READ) TempData++;
		__delay_us(I2C_SPEED);
	}
	if(!ack) IIC_NAck();
	else IIC_Ack();
	return(TempData);
}
