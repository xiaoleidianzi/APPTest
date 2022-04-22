/**
	************************************************************
	************************************************************
	************************************************************
	*	�ļ����� 	led.c
	*
	*	���ߣ� 		�ż���
	*
	*	���ڣ� 		2016-11-23
	*
	*	�汾�� 		V1.0
	*
	*	˵���� 		LED��ʼ��������LED
	*
	*	�޸ļ�¼��	
	************************************************************
	************************************************************
	************************************************************
**/

//��Ƭ��ͷ�ļ�
#include "stm32f10x.h"
#include "string.h"
//LEDͷ�ļ�
#include "led.h"


LED_STATUS led_status;
SCB_struct scb_struct;


/*
************************************************************
*	�������ƣ�	Led_Init
*
*	�������ܣ�	LED��ʼ��
*
*	��ڲ�����	��
*
*	���ز�����	��
*
*	˵����		LED4-PC7	LED5-PC8	LED6-PA12	LED7-PC10
				�ߵ�ƽ�ص�		�͵�ƽ����
************************************************************
*/
void Led_Init(void)
{
	
	GPIO_InitTypeDef gpio_initstruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOC, ENABLE);	//��GPIOA��GPIOC��ʱ��
	
	gpio_initstruct.GPIO_Mode = GPIO_Mode_Out_PP;									//����Ϊ�������ģʽ
	gpio_initstruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 |GPIO_Pin_8 | GPIO_Pin_9 |GPIO_Pin_10;					//��ʼ��Pin7��8��10
	gpio_initstruct.GPIO_Speed = GPIO_Speed_50MHz;									//���ص����Ƶ��
	GPIO_Init(GPIOF, &gpio_initstruct);												//��ʼ��GPIOC
	
	gpio_initstruct.GPIO_Pin = GPIO_Pin_12;											//��ʼ��Pin12
	GPIO_Init(GPIOA, &gpio_initstruct);												//��ʼ��GPIOA
  
	GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_SET);	//status���������LED_ON�򷵻�Bit_SET�����򷵻�Bit_RESET����ͬ  
	GPIO_WriteBit(GPIOF, GPIO_Pin_7, Bit_SET);	//status���������LED_ON�򷵻�Bit_SET�����򷵻�Bit_RESET����ͬ 
	GPIO_WriteBit(GPIOF, GPIO_Pin_8, Bit_SET);	//status���������LED_ON�򷵻�Bit_SET�����򷵻�Bit_RESET����ͬ 
	GPIO_WriteBit(GPIOF, GPIO_Pin_9, Bit_SET);	//status���������LED_ON�򷵻�Bit_SET�����򷵻�Bit_RESET����ͬ 
//	Led4_Set(LED_OFF);
//	Led5_Set(LED_OFF);
//	Led6_Set(LED_OFF);
//	Led7_Set(LED_OFF);

}

void TESEREADinterrupt(int a,int b,int c,int d,int e,int f,int g,int h,int i,int j,int k,int l,int m,int n,int o,int p,int q,int r,int s,int t,int w,int v,int u,int x,int y,int z)
{
	
	//register unsigned int msp_reg __asm("msp");
	//msp_reg = 0X20008000 ;
	
	

//	__asm {
//	MOV R13 ,#0X20008000
//	PUSH{R0-R3}}
//	
//	//memcpy((unsigned int *)scb_struct.CPUID,(unsigned int *)0xF000ED0C,sizeof(SCB_struct));
//	//scb_struct.CPUID = SCB->CPUID;
//	//scb_struct.VTOR = SCB->VTOR;
//	
//	
//	POP{R0-R3}	
	
}

__asm void TESTINIT(void)
{
	//L1��L2Ϊ��ţ�ע�ⲻҪ��:
 
	//PUSH{R0-R3}
	//BKPT(0)
	//PUSH{LR}	
//L1
//	NOP
//	//BKPT(0)
//	CMP R0, #1
//	//BNE L2
//	//MOVS R3, #3
//	//MOVS R4, #4
//	ITE EQ 
//	MOVEQ R4, #1
//	MOVNE R4, #2
//L2 
//	MOV R3, #2
	
	//POP{PC}
}
/*
************************************************************
*	�������ƣ�	Led4_Set
*
*	�������ܣ�	LED4����
*
*	��ڲ�����	status��LED_ON-����	LED_OFF-�ص�
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void Led4_Set(LED_ENUM status)
{

	GPIO_WriteBit(GPIOF, GPIO_Pin_7, status != LED_ON ? Bit_SET : Bit_RESET);	//status���������LED_ON�򷵻�Bit_SET�����򷵻�Bit_RESET����ͬ
	led_status.Led4Sta = status;

}

/*
************************************************************
*	�������ƣ�	Led5_Set
*
*	�������ܣ�	LED5����
*
*	��ڲ�����	status��LED_ON-����	LED_OFF-�ص�
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void Led5_Set(LED_ENUM status)
{

	GPIO_WriteBit(GPIOC, GPIO_Pin_8, status != LED_ON ? Bit_SET : Bit_RESET);
	led_status.Led5Sta = status;

}

/*
************************************************************
*	�������ƣ�	Led6_Set
*
*	�������ܣ�	LED6����
*
*	��ڲ�����	status��LED_ON-����	LED_OFF-�ص�
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void Led6_Set(LED_ENUM status)
{

	GPIO_WriteBit(GPIOA, GPIO_Pin_12, status != LED_ON ? Bit_SET : Bit_RESET);
	led_status.Led6Sta = status;

}

/*
************************************************************
*	�������ƣ�	Led7_Set
*
*	�������ܣ�	LED7����
*
*	��ڲ�����	status��LED_ON-����	LED_OFF-�ص�
*
*	���ز�����	��
*
*	˵����		
************************************************************
*/
void Led7_Set(LED_ENUM status)
{

	GPIO_WriteBit(GPIOC, GPIO_Pin_10, status != LED_ON ? Bit_SET : Bit_RESET);
	led_status.Led7Sta = status;

}
