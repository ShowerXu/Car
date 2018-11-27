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
*PID3_ok是去掉调试输出，代以延时，需观察812F
*401：延时打开到尾功能,延时5S 8CCD
*	  陀螺仪初始化检测.	  8E13
*	  陀螺仪监测放到第一次上电,检测失败RA输出高电平3秒
*402：增加低电压检测	  5B70
*403：增加测试模式		4218
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
volatile unsigned int pwmPer=0;
//传感器数值
int Z10[N];
int z,zz;
volatile unsigned char fnum=0;
//unsigned char hnum,lnum=0;
#define SMS_SLEEP	0
#define SMS_INIT	1
#define SMS_RUN		2
//状态机变量
volatile unsigned char SMS=0;
unsigned int can_close=0;

void zz2pwm(int zz1);



struct _pid{
	int SetSpeed; //定义设定值
	int ActualSpeed; //定义实际值
	int err; //定义偏差值
	int err_next; //定义上一个偏差值
	int err_last; //定义最上前的偏差值
	int Kp,Ki,Kd; //定义比例、积分、微分系数
	long voltage;
}pid;
void PID_realize(int curSpeed);
void PID_init(void);
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
		#ifdef	ENDEBUG			
		Uart_TX_Int();
		#endif
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
    test_init();
    __delay_ms(100);
    CLRWDT();
    if(RA3==0)
    {
	    i=MPU_Init();					//check MPU6050
	    if(i==0)	RA2=1;
        RC3=1;							//check motor
        i=250;
        key_stop_time=0;
        while(i--){						//check stop switch
	    	CLRWDT();
	        __delay_ms(80);
	        key_scan_H();
	        if(key_stop_time>10) RC3=0;
            else	RC3=1;
        }
        RC3=0;
        RA2=0;
    }
    //DEVICE_INIT();
	key_init();
	SMS = SMS_SLEEP;
	do{
		switch(SMS)
		{
			case SMS_SLEEP:
			#ifdef	ENDEBUG	
				//UART_TX_init();	
				//GIE =1;			
				//UART_TX_print("\r\n --Sleep");
			#endif
            	GIE =1;	
				pwmPer=0;
                RA2=0;
				SET_EPWM_TO(pwmPer);
				SET_EPWM_OFF();
				goto_sleep();
				GIE =1;
				SLEEP();
				CLRWDT();
				key_init();
				key_power_time=0;
				key_stop_time=0;
				key_speed_time=0;
				for(i=0;i<10;i++){
					__delay_ms(10);
					key_scan();
					if(key_power_time>5){
						SMS= SMS_INIT;
						speed_mode=0;
						if(key_speed_time>5) speed_mode=1;
						break;
					}
				}
				CLRWDT();
			break;
			
			case SMS_INIT:
				SET_EPWM_INIT();			//初始化pwm接口 
				i=MPU_Init();					//初始化MPU6050
                PID_init();
	#ifdef	ENDEBUG			
				UART_TX_init();
                GIE=1;			
				//UART_TX_print("--wakup\r\n");
	#endif	
				key_init();					//按键初始化
				key_power_time=5;			//初始值
				key_stop_time=5;
                can_close=0;
				GIE = 1;  					//开启总中断
				__delay_ms(1000);			//等待sensor稳定
	#ifdef	ENDEBUG				
				//UART_TX_print("\r\nUART-9600-8-N-1\r\n");
	#endif			
				pwmPer=0;   
				SET_EPWM_TO(pwmPer);
                SET_EPWM_ON();
				SMS = SMS_RUN;
				CLRWDT();
			break;
			case SMS_RUN:
                pwmPer=600;   
				SET_EPWM_TO(pwmPer);
                SET_EPWM_ON();
                __delay_ms(20);
				while(1){	                    
					for(i=0;i<N;i++)
                    {
					 MPU_Get_Gyr_Z(&z);  // This will update x, y, and z with new values  
                      //MPU_Get_Gyroscope(&x,&y,&z);
                      if(z<0) z=0;
					  Z10[i]=z;
					  key_scan_H();
					  CLRWDT();
					  //__delay_ms(2);
					}
					if(key_power_time>40)	
                    { 
                    	SMS=SMS_SLEEP;
                        break;
                    }
                    if(can_close<250){
                    	can_close++;
                    	key_stop_time=1;
                    }                   
					if(key_stop_time>40)	
                    { 
                    	SMS=SMS_SLEEP;
                       break;
                    }
					zz=filter(Z10);		
					zz2pwm(zz);
                    SET_EPWM_TO(pwmPer);
                    if(can_close>160)	
                    { 
                    	if((zz<1000)&&(pwmPer>900)){SMS=SMS_SLEEP;break;}                     
                    }                     
    #ifdef	ENDEBUG				
					int2StrB(str,zz);
					UART_TX_print(str);
					UART_TX_print("|");
					int2StrB(str,pwmPer);
					UART_TX_print(str);
					UART_TX_print("|");                                       
                    
	#endif		               
	#ifdef	ENDEBUG			                   
					int2StrB(str,pid.ActualSpeed);
					UART_TX_print(str);
					UART_TX_print("\r\n");
	#endif	
    				__delay_ms(15);
                    __delay_ms(5);
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
*函数名:PID
*功能:转换角速度到马达占空比
*输入参数:角速度
*返回参数:无
===================================================*/
int abss(int a)
{
	if(a < 0)
		return -a;
	return a;
}

void PID_init(void){
	pid.SetSpeed=MID_VALUE_33;
	pid.ActualSpeed=0;
	pid.err=0;
	pid.err_last=0;
	pid.err_next=0;
	pid.Kp=1;
	pid.Ki=4;
	pid.Kd=2;
}
void PID_realize(int curSpeed){
	long incrementSpeed;
    char index=0;
	pid.SetSpeed=gyro[speed_mode][1];
	pid.err=pid.SetSpeed-curSpeed;
	if(curSpeed>1000)
	{
		incrementSpeed=pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last);
		incrementSpeed>>1;
        pid.ActualSpeed=incrementSpeed;
		if(abss(incrementSpeed)>680)	pid.voltage=4;
		//else if(abss(incrementSpeed)>300)	pid.voltage=4;
		else if(abss(incrementSpeed)>280)	pid.voltage=3;
		else if(abss(incrementSpeed)>80)	pid.voltage=2; 
		else if(abss(incrementSpeed)>30)	pid.voltage=1;
		else pid.voltage=0;
		if(incrementSpeed<0) pid.voltage=0-pid.voltage; 
		pid.err_last=pid.err_next;
		pid.err_next=pid.err;
	}
	else
	{
		if(pid.err) incrementSpeed=1;
		else incrementSpeed=-1;
		pid.voltage=incrementSpeed;
	}
	return;
}

/*====================================================
*函数名:zz2pwm
*功能:转换角速度到马达占空比
*输入参数:角速度
*返回参数:无
====================================================*/
void zz2pwm(int zz1){
	
	PID_realize(zz);
	pid.voltage+=pwmPer;
	pwmPer=pid.voltage;
}


