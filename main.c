/**********************************************************
*�ļ���:MS82F_WPWM_TEST.C
*����:MS82Fxx02 CCP1ģ��PWM�������
*�����ͺ�:MS82F1402A
*����:�ڲ�RC 4MHz 2T
*���Ŷ���:
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
*˵��:����˵
*PID3_ok��ȥ�����������������ʱ����۲�812F
*401����ʱ�򿪵�β����,��ʱ5S 8CCD
*	  �����ǳ�ʼ�����.	  8E13
*	  �����Ǽ��ŵ���һ���ϵ�,���ʧ��RA����ߵ�ƽ3��
*402�����ӵ͵�ѹ���	  5B70
*403�����Ӳ���ģʽ		4218
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
//���ڵ��Բ���
unsigned char str[7];
#endif
//PWM����
volatile unsigned int pwmPer=0;
//��������ֵ
int Z10[N];
int z,zz;
volatile unsigned char fnum=0;
//unsigned char hnum,lnum=0;
#define SMS_SLEEP	0
#define SMS_INIT	1
#define SMS_RUN		2
//״̬������
volatile unsigned char SMS=0;
unsigned int can_close=0;

void zz2pwm(int zz1);



struct _pid{
	int SetSpeed; //�����趨ֵ
	int ActualSpeed; //����ʵ��ֵ
	int err; //����ƫ��ֵ
	int err_next; //������һ��ƫ��ֵ
	int err_last; //��������ǰ��ƫ��ֵ
	int Kp,Ki,Kd; //������������֡�΢��ϵ��
	long voltage;
}pid;
void PID_realize(int curSpeed);
void PID_init(void);
/*====================================================
*������:interrupt ISR
*����:�жϷ�����
*�������:��
*���ز���:��
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
*������:DEVICE_INIT
*����:�ϵ�������ʼ��
*�������:��
*���ز���:��
====================================================*/
void DEVICE_INIT(void)
{
	OSCCON = 0B01110001;   //Bit7    >>>  LFMOD=0 >>> WDT����Ƶ��=32KHz
                           //Bit6:4 >>> IRCF[2:0]=111 >>> �ڲ�RCƵ��=16MHz
                           //Bit0   >>> SCS=1      >>> ϵͳʱ��ѡ��Ϊ�ڲ�����
	INTCON = 0B00000000; //�ݽ�ֹ�����ж�
	PIE1 = 0B00000000;
	PIR1 = 0B00000000;
	PORTA = 0B00000000;
	TRISA = 0B00000000;  //����PORTA�ڶ�Ϊ�����
	WPUA = 0B00000000;
	PORTC = 0b00000000;
	TRISC = 0B00000000;  //����PORTAC�ڶ�Ϊ�����
	WPUC = 0B00000000;
	OPTION = 0B00000000;
}

//=======================MainRoutine======================
void main(void)
{
#asm
	MOVLW		0x07			// ϵͳ���ò�����ɾ�����޸�
	MOVWF		0x19			// ϵͳ���ò�����ɾ�����޸�
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
				SET_EPWM_INIT();			//��ʼ��pwm�ӿ� 
				i=MPU_Init();					//��ʼ��MPU6050
                PID_init();
	#ifdef	ENDEBUG			
				UART_TX_init();
                GIE=1;			
				//UART_TX_print("--wakup\r\n");
	#endif	
				key_init();					//������ʼ��
				key_power_time=5;			//��ʼֵ
				key_stop_time=5;
                can_close=0;
				GIE = 1;  					//�������ж�
				__delay_ms(1000);			//�ȴ�sensor�ȶ�
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
*������:PID
*����:ת�����ٶȵ����ռ�ձ�
*�������:���ٶ�
*���ز���:��
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
*������:zz2pwm
*����:ת�����ٶȵ����ռ�ձ�
*�������:���ٶ�
*���ز���:��
====================================================*/
void zz2pwm(int zz1){
	
	PID_realize(zz);
	pid.voltage+=pwmPer;
	pwmPer=pid.voltage;
}


