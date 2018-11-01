/**********************************************************
*文件名:MS82F_WPWM_TEST.C
*功能:MS82Fxx02 CCP1模块PWM半桥输出
*器件型号:MS82F1402A
*振荡器:内部RC 4MHz 2T
*引脚定义:
*                 ----------------
*  VDD-----------|1(VDD)   (VSS)14|------------GND
*  SSW-----------|2(PA7)   (PA0)13|-------------NC
*  PSW-----------|3(PA6)   (PA1)12|---------BATCHK
*  NC------------|4(PA4)   (PA2)11|----------DEBUG
*  PWM-----------|5(PC3)   (PA3)10|-------------TX-9600
*  SCL-----------|6(PC2)   (PC0)09|-------------NC
*  SDA-----------|7(PC4)   (PC1)08|------------LSW
*                 ----------------
*                 MS82F1402A SOP14
*说明:懒得说
**********************************************************/
#include "syscfg.h"
#include "MS82Fxx02.h"
#include "sysdef.h"
#include "EPWM.h"
#include "mpu6050.h"
#include "uart.h"
#include "other.h"

//#define	ENDEBUG	1

#ifdef ENDEBUG
//串口调试参数
unsigned char str[7];
#endif
//PWM参数
volatile unsigned char pwmPer=0;
//传感器数值
//#define N=10
int Z10[N];
int z,zz;
#define SMS_SLEEP	0
#define SMS_INIT	1
#define SMS_RUN		2
//状态机变量
volatile unsigned char SMS=0;


void zz2pwm(int zz1);
/*====================================================
*函数名:interrupt ISR
*功能:中断服务函数
*输入参数:无
*返回参数:无
====================================================*/
void interrupt ISR(void)
{
	if(T0IE&&T0IF)
	{
		T0IF = 0;
		Uart_TX_Int();
	}
    IOCA_int();
}
/*====================================================
*函数名:DEVICE_INIT
*功能:上电器件初始化
*输入参数:无
*返回参数:无
====================================================*/
void DEVICE_INIT(void)
{
	OSCCON = 0B01110001;   //Bit7    >>>  LFMOD=0 >>> WDT振荡器频率=32KHz
                           //Bit6:4 >>> IRCF[2:0]=111 >>> 内部RC频率=16MHz
                           //Bit0   >>> SCS=1      >>> 系统时钟选择为内部振荡器
	INTCON = 0B00000000; //暂禁止所有中断
	PIE1 = 0B00000000;
	PIR1 = 0B00000000;
	PORTA = 0B00000000;
	TRISA = 0B00000000;  //所有PORTA口都为输出口
	WPUA = 0B00000000;
	PORTC = 0b00000000;
	TRISC = 0B00000000;  //所有PORTAC口都为输出口
	WPUC = 0B00000000;
	OPTION = 0B00000000;
}

//=======================MainRoutine======================
void main(void)
{
#asm
	MOVLW		0x07			// 系统设置不可以删除和修改
	MOVWF		0x19			// 系统设置不可以删除和修改
#endasm
	uchar i=0;
	DEVICE_INIT();
	key_init();
	SMS = SMS_SLEEP;
	do{
		switch(SMS)
		{
			case SMS_SLEEP:
			#ifdef	ENDEBUG	
				UART_TX_init();	
				GIE =1;			
				UART_TX_print("\r\n --Sleep\r\n");
			#endif
				pwmPer=0;
				SET_EPWM_TO(pwmPer);
				SET_EPWM_OFF();
				goto_sleep();
				GIE =1;
				SLEEP();
				CLRWDT();
				key_init();
				key_power_time=0;
				for(i=0;i<10;i++){
					__delay_ms(10);
					key_scan();
					if(key_power_time>5){
						SMS= SMS_INIT;
						break;
					}
				}
				CLRWDT();
			break;
			
			case SMS_INIT:
				SET_EPWM_INIT();			//初始化pwm接口 
				MPU_Init();					//初始化MPU6050
	#ifdef	ENDEBUG			
				UART_TX_init();			
				UART_TX_print("\r\n --wakup\r\n");
	#endif	
				key_init();					//按键初始化
				key_power_time=10;			//初始值
				GIE = 1;  					//开启总中断
				__delay_ms(100);			//等待sensor稳定
	#ifdef	ENDEBUG				
				UART_TX_print("\r\nMS81Fxx02 UART-9600-8-N-1\r\n");
	#endif			
				pwmPer=0;   
				SET_EPWM_TO(pwmPer);
				SMS = SMS_RUN;
				CLRWDT();
			break;
				
			case SMS_RUN:
				while(1){	
					for(i=0;i<N;i++)
                    {
					  MPU_Get_Gyr_Z(&z);  // This will update x, y, and z with new values    
					  Z10[i]=z;
					  key_scan_H();
					  CLRWDT();
					  __delay_ms(5);
					}
					if(key_power_time>240)	
                    { 
                    	SMS=SMS_SLEEP;
                        break;
                    }
					zz=filter(Z10);
	#ifdef	ENDEBUG			
    				//zz=z;	
					int2str(str,zz);
					UART_TX_print(str);
					UART_TX_print("-");
	#endif				
					zz2pwm(zz);
					SET_EPWM_TO(pwmPer);
	#ifdef	ENDEBUG			
    				//zz=z;	
					int2str(str,pwmPer);
					UART_TX_print(str);
					UART_TX_print("\r\n");
	#endif	
				}
			break;
			
			default: 
				SMS=SMS_SLEEP;
				CLRWDT();
			break;
		}
	}while(1);
}



/*====================================================
*函数名:zz2pwm
*功能:转换角速度到马达占空比
*输入参数:角速度
*返回参数:无
====================================================*/
void zz2pwm(int zz1){
  unsigned int a,b,c=0;
  if(zz1>=MID_VALUE){
    a=zz1-MID_VALUE;
    c=1;
  }else{
    a=MID_VALUE-zz1;
    c=0;
  }
  b=a>>6; //(0~7)
  if(c<1){
    if(pwmPer<=246) pwmPer+=b;
    }
  else {
    if(pwmPer>b) pwmPer-=b;
    }
  return;
}