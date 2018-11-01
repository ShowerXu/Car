/********************************
1.��������	-- RA6
2.ֹͣ����	-- RC1
3.�ٶȿ���	-- RA7


*********************************/
#include "syscfg.h"
#include "MS82Fxx02.h"
#include "other.h"
#include "iic_driver.h"

#define	PA_KEY_MASK	0B11000000
#define PC_KEY_MASK	0B00000010
#define key_power	RA6
#define key_stop	RC1
#define key_speed	RA7

volatile unsigned char Temp_Read;

void key_init(void)
{
	//PORTA |= PA_KEY_MASK;
	TRISA |= PA_KEY_MASK;  
	WPUA |= PA_KEY_MASK;   
	//PORTC |= PC_KEY_MASK;
	TRISC |= PC_KEY_MASK; 
	WPUC |= PC_KEY_MASK;    	
}

void key_scan(void)
{
	if(!key_power){ if(key_power_time<250)	key_power_time++;}
	else key_power_time=0;
	if(!key_stop) {if(key_stop_time<250)		key_stop_time++;}
	else key_stop_time=0;
	if(!key_speed){if(key_speed_time<250)	key_speed_time++;}
	else key_speed_time=0;
}

void key_scan_H(void)
{
	if(key_power){ if(key_power_time<250)	key_power_time++;}
	else key_power_time=0;
	if(key_stop) {if(key_stop_time<250)		key_stop_time++;}
	else key_stop_time=0;
	if(key_speed){if(key_speed_time<250)	key_speed_time++;}
	else key_speed_time=0;
}

/*
 goto_sleep(void)
 ֻ�ܱ�RA6��ioca�жϻ���
*/ 
	
void goto_sleep(void)
{
	
	PORTA=0;
    PORTC=0;
    IIC_Init();
    
	SWDTEN = 0;	//�����رտ��Ź�
	Temp_Read = PORTA; //���ʧ��״̬
	PAIF = 0;
	IOCA6 = 1;
	PAIE = 1;
	//SLEEP();
	//SWDTEN = 1; //�����������Ź�

}

/*====================================================
*������:IOCA interrupt ISR
*����:�жϷ�����
*�������:��
*���ز���:��
====================================================*/
void IOCA_int(void)
{
	if(PAIE&&PAIF)
	{
		Temp_Read = PORTA; //���ʧ��״̬
		PAIF = 0;          //���־λ
		IOCA6 = 0;
		PAIE = 0;          //�ر��ж�
	}
}


/*====================================================
*������:filter
*����:��λֵƽ���˲�
*�������:����
*���ز���:�����˲������ֵ
====================================================*/
unsigned int filter(int *arr)
{
  unsigned char i=0;
  unsigned int sum=0;
  for(i=0;i<(N-2);i++){
    if(*(arr+i)>MAX_VALUE) *(arr+i)=1000;
    if(*(arr+i)<MIN_VALUE) *(arr+i)=0;
    sum+=*(arr+i);
  }
  sum=sum>>3;
  sum+=MIN_VALUE;
  *(arr+N-1)=sum;
  return *(arr+N-1);
}