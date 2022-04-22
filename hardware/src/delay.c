/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	delay.c
	*
	*	作者： 		张继瑞
	*
	*	日期： 		2016-11-23
	*
	*	版本： 		V1.0
	*
	*	说明： 		利用systick做阻塞式延时
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/

//单片机头文件
#include "stm32f10x.h"

//delay头文件
#include "delay.h"

uint32_t SystickTime=0;
//延时系数
unsigned char UsCount = 0;
unsigned int MsCount = 0;
unsigned int a;

/*
************************************************************
*	函数名称：	Delay_Init
*
*	函数功能：	systick初始化
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void Delay_Init(void)
{

	SysTick->CTRL |= ~(1 << 2);	//选择时钟为HCLK		103--9MHz
	
	
	a = *(unsigned int *)(SysTick_BASE);
	
	UsCount = 9;								//微秒级延时系数9000000/1000000
	
	MsCount = UsCount * 1000;		//毫秒级延时系数9000000/1000

}

/*
************************************************************
*	函数名称：	DelayUs
*
*	函数功能：	微秒级延时
*
*	入口参数：	us：延时的时长
*
*	返回参数：	无
*
*	说明：		此时钟(21MHz)最大延时798915us
************************************************************
*/
void DelayUs(unsigned short us)
{

	unsigned int ctrlResult = 0;
	
	us &= 0x00FFFFFF;											//取低24位
	
	SysTick->LOAD = us * UsCount;					//装载数据
	
	SysTick->VAL = 0;
	SysTick->CTRL |= 1;										//使能倒计数器
	
	do
	{
		ctrlResult = SysTick->CTRL;
	}
	while((ctrlResult & 0x01) && !(ctrlResult & (1 << 16)));	//保证在运行、检查是否倒计数到0
	
	SysTick->CTRL = 0;											//关闭倒计数器
	SysTick->VAL &= 0;

}

/*
************************************************************
*	函数名称：	DelayXms
*
*	函数功能：	毫秒级延时
*
*	入口参数：	ms：延时的时长
*
*	返回参数：	无
*
*	说明：		
************************************************************
*/
void DelayXms(unsigned short ms)
{
	if((ms == 0)|(ms * MsCount > 0xFFFFFF))//取低24位装载值	
		return;	
	
	SysTick->LOAD = ms * MsCount;								//装载数据
	SysTick->VAL = 0;														//计数器清零
	SysTick->CTRL |= 0x03;											//使能倒计数器+计数为零时触发异常+使用系统时钟(72M)
}

/*
************************************************************
*	函数名称：	DelayMs
*
*	函数功能：	微秒级长延时
*
*	入口参数：	ms：延时的时长
*
*	返回参数：	无
*
*	说明：		多次调用DelayXms，做到长延时
************************************************************
*/
void DelayMs(unsigned short ms)
{

	unsigned char repeat = 0;
	unsigned short remain = 0;
	
	repeat = ms / 500;
	remain = ms % 500;
	
	while(repeat)
	{
		DelayXms(500);
		repeat--;
	}
	
	if(remain)
		DelayXms(remain);

}
