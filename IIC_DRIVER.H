#ifndef __IIC_DRIVER_H
#define __IIC_DRIVER_H

#include "syscfg.h"
#include "MS82Fxx02.h"

#define SDA_READ		RC4
#define SCL_READ		RC2

#define SCL_H()			SCL_READ = 1
#define SCL_L()			SCL_READ = 0
#define SDA_H()			SDA_READ = 1
#define SDA_L()			SDA_READ = 0

#define SET_SCL_OUT		TRISC2 = 0
#define SET_SDA_OUT		TRISC4 = 0
#define SET_SDA_IN		TRISC4 = 1

#define I2C_SPEED		2

//IIC操作函数
void IIC_Init(void);//初始化IIC的IO�?
void IIC_Start(void); //IIC开始信�?
void IIC_Stop(void);//IIC结束信号
void IIC_Ack(void);//产生ACK应答信号
void IIC_NAck(void);//不产生ACK应答信号
unsigned char IIC_Wait_Ack(void); //等待应答信号
void IIC_Send_Byte(unsigned char txd);//IIC发送一个字�?
unsigned char IIC_Read_Byte(unsigned char ack);//IIC读一个字�?

#endif

