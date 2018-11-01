#include "iic_driver.h"
#include "sysdef.h"
/*
��Ҫ�ⲿ����������
*/
void IIC_Init(void)
{
	CLRWDT();
	SET_SCL_OUT;
	SET_SDA_OUT;
	// WPUC2 = 1;		//�ڲ��������������ⲿ��·�����Ƿ���
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
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
unsigned char IIC_Wait_Ack(void)		
{  
	unsigned char ErrorCount=50;	//���峬ʱ��Ϊ255��

	SET_SDA_IN;						//SDA as Input
	//SET_SCL_OUT;
	SDA_H();
	CLRWDT();
	__delay_us(I2C_SPEED);
	SCL_H();
	__delay_us(I2C_SPEED);
	while(SDA_READ)					//��һ��ʱ���ڼ�⵽IIC_SDA=0�Ļ���Ϊ��Ӧ���ź�  
	{
		if(ErrorCount==0)  
		{  
			IIC_Stop();
			return 1;
		}  
		ErrorCount--;  
	}
	SCL_L();						//ǯס����,Ϊ��1��ͨ����׼��   
	SET_SDA_OUT;
	return 0;						//�ɹ�����Ӧ���ź�
}
//IIC����һ���ֽ�
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
//IIC��һ���ֽ�
//ack=1 ����ACK
//ack=0 ����NACK
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