/**
	************************************************************
	************************************************************
	************************************************************
	*	文件名： 	main.c
	*
	*	作者： 		张继瑞
	*
	*	日期： 		2017-05-08
	*
	*	版本： 		V1.0
	*
	*	说明： 		接入onenet，上传数据和命令控制
	*
	*	修改记录：	
	************************************************************
	************************************************************
	************************************************************
**/

//单片机头文件
#include "stm32f10x.h"

//网络协议层
#include "onenet.h"

//网络设备
#include "esp8266.h"

//硬件驱动
#include "delay.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "usart.h"
#include "i2c_ee.h"
//C库
#include <string.h>
#include <stdio.h>

#include "version.h"


//const char Compiler_Time[] __attribute__((at(VERINFO_ADDR_BASE + 0x60))) = "Time: "__TIME__;

extern uint32_t SystickTime;

volatile uint32_t systick_count = 0;

//OS使用的数据
long long task0_stack[32],task1_stack[32],task2_stack[32],task3_stack[32];

volatile uint32_t curr_task = 0;
uint32_t next_task = 1;
uint32_t PSP_array[4];

void task0(void);
void task1(void);
void task2(void);
void task3(void);


int __svc(0x00) Svc_service_add(int x, int y);//服务#0:加法
int __svc(0x01) Svc_service_sub(int x, int y);//服务#1:减法
int __svc(0x02) Svc_service_incr(int x);			//服务#2:自加	
void SVC_Handler_Main(unsigned int * svc_args);
int x,y,z;
void RCC_Configuration(void);
void I2CInit(void);
void I2C(void);



typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private typedef -----------------------------------------------------------*/
//typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define EEPROM_WriteAddress1    0xA0
#define EEPROM_ReadAddress1     0xA1
#define BufferSize1             (countof(Tx1_Buffer)-1)
#define BufferSize2             (countof(Tx2_Buffer)-1)
#define EEPROM_WriteAddress2    (EEPROM_WriteAddress1 + BufferSize1)
#define EEPROM_ReadAddress2     (EEPROM_ReadAddress1 + BufferSize1)

/* Private macro -------------------------------------------------------------*/
#define countof(a) (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
u8 Tx1_Buffer[] = "STM32F103";
u8 Rx1_Buffer[BufferSize1];
TestStatus TransferStatus1 = FAILED, TransferStatus2 = FAILED;
TestStatus Buffercmp(u8* pBuffer1, u8* pBuffer2, u16 BufferLength);
//extern void I2C_Configuration(void);
/*
************************************************************
*	函数名称：	Hardware_Init
*
*	函数功能：	硬件初始化
*
*	入口参数：	无
*
*	返回参数：	无
*
*	说明：		初始化单片机功能以及外接设备
************************************************************
*/
void Hardware_Init(void)
{
	//__WFE();休眠
	//NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);							 //中断控制器分组设置
	//NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  //设置systick NVIC中断
		
//	__asm{
//		MOVS r13 ,#0x200008A0 
//		PUSH{R0-R3}		
//	}
	
//	TESEREADinterrupt(1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26);

	RCC_Configuration();
	
	//Delay_Init();									//systick初始化
	SysTick_Config(7199999);					//100ms
	
	Usart1_Init(115200);							//串口1，打印信息用
	
	Usart2_Init(115200);							//串口2，驱动ESP8266用
	
	Led_Init();										//LED初始化
	
	I2C_EE_Init();  
	
	UsartPrintf(USART2, " Hardware init OK\r\n");
	
}

/*
************************************************************
*	函数名称：	main
*
*	函数功能：	
*
*	入口参数：	无
*
*	返回参数：	0
*
*	说明：		
************************************************************
*/
int main(void)
{
			
	int timestart = 0;	//开始计时
	int timeend   = 0;	//结束计时
	int time1   = 0;//总用时

	Hardware_Init();				//初始化外围硬件
	unsigned int version; 
	version	= *(unsigned int *)(VERINFO_ADDR_BASE + 0x00);
	UsartPrintf(USART1, "%4x\r\n",version);		//串口调试用--输出版本信息
	

	NVIC_SetPriority (SysTick_IRQn,0xFF);  //设置systick NVIC中断	
	//NVIC_SetPriority (SVCall_IRQn,0xFF-1);  //设置systick NVIC中断
	//SysTick_Config(72000000/1000000);
	
//	x = 5;
//	y = 8;
//	z=Svc_service_add(x,y);
//	z=Svc_service_sub(x,y);//服务#0:减法
//	z=Svc_service_incr(x); //服务#0:自加
	
//	//创建任务0的堆栈
//	PSP_array[0] = ((unsigned int)task0_stack) + sizeof task0_stack - 16*4;						//栈指针
//	*(volatile unsigned long *)(PSP_array[0] + (14<<2))         =(unsigned long)task0;
//	*(volatile unsigned long *)(PSP_array[0] + (15<<2))         =(unsigned long)0x01000000;//初始化xPSR
//	
//	//创建任务1的堆栈
//	PSP_array[1] = ((unsigned int)task1_stack) + sizeof task1_stack - 16*4;
//	*(volatile unsigned long *)(PSP_array[1] + (14<<2))         =(unsigned long)task1;
//	*(volatile unsigned long *)(PSP_array[1] + (15<<2))         =(unsigned long)0x01000000;//初始化xPSR
//	
//	//创建任务2的堆栈
//	PSP_array[2] = ((unsigned int)task2_stack) + sizeof task2_stack - 16*4;
//	*(volatile unsigned long *)(PSP_array[2] + (14<<2))         =(unsigned long)task2;
//	*(volatile unsigned long *)(PSP_array[2] + (15<<2))         =(unsigned long)0x01000000;//初始化xPSR
//	
//	//创建任务3的堆栈
//	PSP_array[3] = ((unsigned int)task3_stack) + sizeof task3_stack - 16*4;
//	*(volatile unsigned long *)(PSP_array[3] + (14<<2))         =(unsigned long)task3;
//	*(volatile unsigned long *)(PSP_array[3] + (15<<2))         =(unsigned long)0x01000000;//初始化xPSR
//	//初始化程序计数器
//	curr_task = 1;
//	
//	__set_PSP(PSP_array[curr_task] + 16*4);//设置PSP为任务0的栈顶
//	//__set_CONTROL(0X03);
//	__ISB();
//	task1();
	while(1)
	{	
			if(SystickTime % 20 > 10)		
			{
				timestart = SysTick->VAL;
				GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_SET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同
				GPIO_WriteBit(GPIOF, GPIO_Pin_7, Bit_RESET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同
				GPIO_WriteBit(GPIOF, GPIO_Pin_8, Bit_SET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同
				GPIO_WriteBit(GPIOF, GPIO_Pin_9, Bit_RESET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同
				timeend = SysTick->VAL;
				time1 = (timestart - timeend);
				time1 = time1;
				//I2C();
			}
			else
			{
				GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_RESET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同	
				GPIO_WriteBit(GPIOF, GPIO_Pin_7, Bit_SET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同	
				GPIO_WriteBit(GPIOF, GPIO_Pin_8, Bit_RESET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同	
				GPIO_WriteBit(GPIOF, GPIO_Pin_9, Bit_SET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同					
			}
			//DelayXms(1000);
	}

}


void RCC_Configuration(void)
{   

		ErrorStatus HSEstatue;
		//1 复位时钟
		RCC_DeInit();
	
		//2 HSE使能并等待其就绪
		RCC_HSEConfig(RCC_HSE_ON);
		HSEstatue = RCC_WaitForHSEStartUp();
	
		if(HSEstatue==SUCCESS)
		{
			//3 HSE使能预取值，配置等待周期
			FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
			FLASH_SetLatency(FLASH_Latency_2);
			
			//4 配置时钟来源和倍频系数8M×9=72M
			RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);
			
			//5 使能PLL并等待其稳定
			RCC_PLLCmd(ENABLE);
			while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);
			
			//6 选择系统时钟
			RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
			while(RCC_GetSYSCLKSource() != 0x08);
			
			//7 设置HCLK,PCLK2,PCLK2时钟
			RCC_HCLKConfig(RCC_SYSCLK_Div1);
			//8 设置APB1
			RCC_PCLK1Config(RCC_HCLK_Div2);
			//9 设置APB2
			RCC_PCLK2Config(RCC_HCLK_Div1);
			
		}
		else
		{
			//配置错误执行的代码块
		}

      /* Enable DMA1 clock */
      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
      /* Enable ADC1, ADC2, ADC3 and GPIOC clocks */
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 |RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOF | RCC_APB2Periph_ADC1 , ENABLE);
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 |RCC_APB1Periph_I2C1  | RCC_APB1Periph_I2C2 , ENABLE);
			//GPIO重映射
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		//内部时钟配置
		
//    RCC_DeInit();
//    // Enable HSI 
//    RCC_HSICmd(ENABLE);
//    //Wait till HSE is ready 
//    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET){;}
//    if(1)
//    {
//      /* Enable Prefetch Buffer */
//      //FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

//      /* Flash 2 wait state */
//      //FLASH_SetLatency(FLASH_Latency_2);
// 
//      /* HCLK = SYSCLK */
//      RCC_HCLKConfig(RCC_SYSCLK_Div1); 
//  
//      /* PCLK2 = HCLK */
//      RCC_PCLK2Config(RCC_HCLK_Div1); 

//      /* PCLK1 = HCLK/2 */
//      RCC_PCLK1Config(RCC_HCLK_Div2);
//      /* ADCCLK = PCLK2/4 */
//      RCC_ADCCLKConfig(RCC_PCLK2_Div4);

//      /* PLLCLK = 8MHz /2* 14 = 64 MHz 内部晶振*/
//      RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_14);//RCC_PLLMul_14
//      /* Enable PLL */ 
//      RCC_PLLCmd(ENABLE);

//      /* Wait till PLL is ready */
//      while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
//      {
//      }
//      /* Select PLL as system clock source */
//      RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
//  
//      /* Wait till PLL is used as system clock source */
//      while(RCC_GetSYSCLKSource() != 0x08)
//      {
//      }
//    }

}

void task0(void)
{
	while(1)
	{	
			if(SystickTime % 20 > 10)		
			{
				GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_SET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同
			}
			else
			{
				GPIO_WriteBit(GPIOF, GPIO_Pin_6, Bit_RESET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同		
			}
	}
}

void task1(void)
{
	while(1)
	{	
			if(SystickTime % 20 > 10)		
			{
				GPIO_WriteBit(GPIOF, GPIO_Pin_7, Bit_SET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同
			}
			else
			{
				GPIO_WriteBit(GPIOF, GPIO_Pin_7, Bit_RESET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同		
			}
	}
}


void task2(void)
{
	while(1)
	{	
			if(SystickTime % 20 > 10)		
			{
				GPIO_WriteBit(GPIOF, GPIO_Pin_8, Bit_SET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同
			}
			else
			{
				GPIO_WriteBit(GPIOF, GPIO_Pin_8, Bit_RESET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同		
			}
	}
}


void task3(void)
{
	while(1)
	{	
			if(SystickTime % 20 > 10)		
			{
				GPIO_WriteBit(GPIOF, GPIO_Pin_9, Bit_SET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同
			}
			else
			{
				GPIO_WriteBit(GPIOF, GPIO_Pin_9, Bit_RESET);	//status如果不等于LED_ON则返回Bit_SET，否则返回Bit_RESET。下同		
			}
	}
}

void SysTick_Handler(void)
{
    //SystickTime_Increase();
    SystickTime++;
//		curr_task = 2;
//		//aa = SystickTime / 4;
//		switch(curr_task)
//		{
//			case (0): next_task = 1;break;
//			case (1): next_task = 2;break;
//			case (2): next_task = 3;break;
//			case (3): next_task = 0;break;
//			default:curr_task = 0;break;
//		}
//	  SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
		//SysTick->CTRL = 0;											//关闭倒计数器
}

//void SVC_Handler_Main(unsigned int * svc_args)
//{
//		//栈帧中包含:r0, r1, r2, r3, r12, r14，返回地址和xPSR
//		//压栈RO  = svc_args[0]
//		//压栈R1  = svc_args[1]
//		//压栈R2  = svc_args[2]
//		//压栈R3  = svc_args[3]
//		//压栈R12 = svc_args[4]
//		//压栈LR  = svc_args[5]
//		//压栈PC  = svc_args[6]
//		//压栈xPSR= svc_args[7]
//	
//		unsigned int svc_number;
//		svc_number =((char *)svc_args[6])[-2];
//		switch(svc_number)
//		{
//				case 0:svc_args[0]=svc_args[0]+svc_args[1];
//					break;
//				case 1:svc_args[0]=svc_args[0]-svc_args[1];
//					break;
//				case 2:svc_args[0]=svc_args[0]+1;
//					break;
//				default:
//					break;
//		}
//		return ;

//}
void I2CInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  /* Configure I2C1 pins: SCL and SDA ----------------------------------------*/
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure I2C2 pins: SCL and SDA ----------------------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* 1 bit for pre-emption priority, 3 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
   
  /* Configure and enable I2C1 interrupt -------------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C2 interrupt -------------------------------------*/  
  NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}

void I2C(void)
{
	/* Initialize the I2C EEPROM driver ----------------------------------------*/
  //write(0x00,5);
	//u8 X = read(0X00);

  /* First write in the memory followed by a read of the written data --------*/
  /* Write on I2C EEPROM from EEPROM_WriteAddress1 */
  //I2C_EE_BufferWrite(Tx1_Buffer, EEPROM_WriteAddress1, BufferSize1); 

	I2C_EE_PageWrite(Tx1_Buffer, EEPROM_WriteAddress1, 6);
  I2C_EE_WaitEepromStandbyState();
	
  /* Read from I2C EEPROM from EEPROM_ReadAddress1 */
  I2C_EE_BufferRead(Rx1_Buffer, EEPROM_ReadAddress1, 6); 

  /* Check if the data written to the memory is read correctly */
  TransferStatus1 = Buffercmp(Tx1_Buffer, Rx1_Buffer, BufferSize1);
  /* TransferStatus1 = PASSED, if the transmitted and received data 
     to/from the EEPROM are the same */
  /* TransferStatus1 = FAILED, if the transmitted and received data 
     to/from the EEPROM are different */

  /* Wait for EEPROM standby state */
  I2C_EE_WaitEepromStandbyState();


	
//	
//	
//	
//	I2C_InitTypeDef  I2C_InitStructure;	
//	/* Enable I2C1 and I2C2 ----------------------------------------------------*/
//  I2C_Cmd(I2C1, ENABLE);//使能或者失能 I2C 外设
//  I2C_Cmd(I2C2, ENABLE);

////  /* Enable I2C1 and I2C2 event and buffer interrupt */
////  I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_BUF, ENABLE);
////  I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_BUF, ENABLE);

//  /* I2C1 configuration ------------------------------------------------------*/
//  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;							//设置 I2C 为 I2C 模式
//  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;			//I2C 快速模式 Tlow / Thigh = 2 
//  I2C_InitStructure.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7;//该参数用来设置第一个设备自身地址，它可以是一个 7 位地址或者一个 10 位地址。
//  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;							//使能应答（ACK）
//  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//定义了应答 7 位地址还是 10 位地址
//  I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;					//该参数用来设置时钟频率，这个值不能高于 400KHz。
//  I2C_Init(I2C1, &I2C_InitStructure);
//  /* I2C2 configuration ------------------------------------------------------*/
//  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_10bit;
//  I2C_InitStructure.I2C_OwnAddress1 = I2C2_SLAVE_ADDRESS10;
//  I2C_Init(I2C2, &I2C_InitStructure);
//   
//  /*----- Transmission Phase -----*/
//  /* Send I2C1 START condition */
//  I2C_GenerateSTART(I2C1, ENABLE);//产生 I2Cx 传输 START 条件

//  /* Send data */
//   /* Test on I2C1 EV5 and clear it */
//  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));  //检查最近一次 I2C 事件是否是输入的事件->SB=1，读SR1然后将地址写入DR寄存器将清除该事件。
//  /* Send Header to I2C2 for write */
//  I2C_SendData(I2C1, HeaderAddressWrite);//通过外设 I2Cx 发送一个数据
//  /* Test on I2C1 EV9 and clear it */
//  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_ADDRESS10)); //ADDR10=1，读SR1然后写入DR寄存器将清除该事件。
//  /* Send I2C2 slave Address for write */
//  I2C_Send7bitAddress(I2C1, I2C2_SLAVE_ADDRESS7, I2C_Direction_Transmitter);//向指定的从 I2C 设备传送地址字
//  /* Test on I2C2 EV1 and clear it */
//  while(!I2C_CheckEvent(I2C2, I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED));  
//  /* Test on I2C1 EV6 and clear it */
//  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  

//  /* Send data */
//  while (Rx_Idx < BufferSize)
//  {
//    /* Send I2C1 data */
//    I2C_SendData(I2C1, I2C1_Buffer_Tx[Tx_Idx++]);
//   /* Test on I2C2 EV2 and clear it */
//    while(!I2C_CheckEvent(I2C2, I2C_EVENT_SLAVE_BYTE_RECEIVED));  
//    /* Store received data on I2C2 */
//    I2C2_Buffer_Rx[Rx_Idx++] = I2C_ReceiveData(I2C2);//返回通过 I2Cx 最近接收的数据
//    /* Test on I2C1 EV8 and clear it */
//    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); 
//  }
//  /* Send I2C1 STOP Condition */
//  I2C_GenerateSTOP(I2C1, ENABLE);
//  /* Test on I2C2 EV4 and clear it */
//  while(!I2C_CheckEvent(I2C2, I2C_EVENT_SLAVE_STOP_DETECTED)); 
//  /* Clear I2C2 STOPF flag */
//  I2C_ClearFlag(I2C2, I2C_FLAG_STOPF);

//  /* Check the corectness of written data */
//  TransferStatus = Buffercmp(I2C1_Buffer_Tx, I2C2_Buffer_Rx, BufferSize);
//  /* TransferStatus = PASSED, if the transmitted and received data
//     are equal */
//  /* TransferStatus = FAILED, if the transmitted and received data 
//     are different */

//  while (1)
//  {
//  }
}

/*******************************************************************************
* Function Name  : Buffercmp
* Description    : Compares two buffers.
* Input          : - pBuffer1, pBuffer2: buffers to be compared.
*                : - BufferLength: buffer's length
* Output         : None
* Return         : PASSED: pBuffer1 identical to pBuffer2
*                  FAILED: pBuffer1 differs from pBuffer2
*******************************************************************************/
TestStatus Buffercmp(u8* pBuffer1, u8* pBuffer2, u16 BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }
    
    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;  
}

