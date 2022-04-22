/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "usart.h"
extern volatile uint32_t curr_task;
extern uint32_t next_task;
extern uint32_t PSP_array[4];

extern uint32_t SystickTime;
extern __IO uint32_t TimeDisplay;
extern void SVC_Handler_Main(unsigned int * svc_args);
extern vu8 Tx_Idx,Rx_Idx,PEC_Value;
extern u8 I2C1_Buffer_Tx[],I2C2_Buffer_Rx[];

#define BufferSize 4
#define I2C2_SLAVE_ADDRESS7   0x30

int aa = 0;
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
	UsartPrintf(USART_DEBUG, "\r\n*******************HardFault_Handler*******************\r\n");
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
	UsartPrintf(USART_DEBUG, "\r\n*******************MemManage_Handler*******************\r\n");
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
	UsartPrintf(USART_DEBUG, "\r\n*******************BusFault_Handler*******************\r\n");
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
	UsartPrintf(USART_DEBUG, "\r\n*******************UsageFault_Handler*******************\r\n");
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//__asm void SVC_Handler(void)
//{
//	TST LR,#4				//测试EXC RETURN的第2位
//	ITE EQ
//	MRSEQ R0,MSP   //若为0,压栈使用的是MSP，复制到R0
//	MRSNE R0,PSP		//若为1,压栈使用的是PSP，复制到R0
//	
//	//LDR R0,[R0,#24] //从栈帧中得到压栈的PC
//	//(压栈的PC=SVC后指令的地址)
//	//LDRB R0,[R0,#-2]//读取SVC指令的第一个字节  SVC编号目前位于RO中
//	B  __cpp(SVC_Handler_Main)
//	ALIGN 4
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void) //UCOS系统中不写在这里
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//    //SystickTime_Increase();
//    SystickTime++;
////		curr_task = 2;
////		//aa = SystickTime / 4;
////		switch(curr_task)
////		{
////			case (0): next_task = 1;break;
////			case (1): next_task = 2;break;
////			case (2): next_task = 3;break;
////			case (3): next_task = 0;break;
////			default:curr_task = 0;break;
////		}
////	  SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
//		//SysTick->CTRL = 0;											//关闭倒计数器
//}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/


void EXTI0_IRQHandler(void)
{
    
}

/**
  * @brief  This function handles usart1 global interrupt request.
  * @param  None
  * @retval : None
  */
//void USART1_IRQHandler(void)
//{
//		unsigned int data;

//    if(USART1->SR & 0x0F)
//    {
//        // See if we have some kind of error
//        // Clear interrupt (do nothing about it!)
//        data = USART1->DR;
//    }
//    else if(USART1->SR & USART_FLAG_RXNE)      //Receive Data Reg Full Flag
//    {		
//        data = USART1->DR;
//				//usart1_putrxchar(data);     //Insert received character into buffer                     
//    }
//		else
//		{;}
//}

/**
  * @brief  This function handles usart2 global interrupt request.
  * @param  None
  * @retval : None
  */
//void USART2_IRQHandler(void)
//{
//		unsigned int data;

//    if(USART2->SR & 0x0F)
//    {
//        // See if we have some kind of error
//        // Clear interrupt (do nothing about it!)
//        data = USART2->DR;
//    }
//		else if(USART2->SR & USART_FLAG_RXNE)   //Receive Data Reg Full Flag
//    {		
//        data = USART2->DR;
//				usart2_rcv_buf[usart2_rcv_len++]=data;
//				
//				if(data=='{') //约定平台下发的控制命令以'{'为开始符，‘}’为控制命令结束符，读者可以自定义自己的开始符合结束符
//				{
//						rcv_cmd_start=1;
//				}
//				if(rcv_cmd_start==1)
//				{
//						usart2_cmd_buf[usart2_cmd_len++]=data;
//						if((data=='}')||(usart2_cmd_len>=MAX_CMD_LEN-1))
//						{
//								rcv_cmd_start=0;
//								LED_CmdCtl();
//								memset(usart2_cmd_buf,0,usart2_cmd_len);
//        				usart2_cmd_len=0;
//						}
//				}	  
//    }
//		else
//		{
//				;
//		}
//}

/**
  * @brief  This function handles RTC global interrupt request.
  * @param  None
  * @retval : None
  */
void RTC_IRQHandler(void)
{
   
}

//* 描    述 : 在cm3内核下,真正的任务文本切换是靠本函数实现
;//                 |     ....        |
;//                 |-----------------|
;//                 |     ....        |
;//                 |-----------------|
;//                 |     ....        |
;//                 |-----------------|    
;//     Low Memory  |     ....        |     
;//                 |-----------------|      
;//        ^        |       R4        |  
;//        ^        |-----------------|     
;//        ^        |       R5        |          
;//        |        |-----------------|         
;//        |        |       R6        |          
;//        |        |-----------------|          
;//        |        |       R7        |         
;//        |        |-----------------|            
;//        |        |       R8        |        
;//        |        |-----------------|         
;//        |        |       R9        |
;//        |        |-----------------|
;//        |        |      R10        |
;//      Stack      |-----------------|
;//      Growth     |      R11        |
;//       = 1       |-----------------|
;//        |        |    R0 = p_arg   |  <-------- 异常时的PSP (向下生长的满栈)
;//        |        |-----------------|
;//        |        |       R1        |
;//        |        |-----------------|
;//        |        |       R2        |
;//        |        |-----------------|
;//        |        |       R3        |
;//        |        |-----------------|
;//        |        |       R12       |
;//        |        |-----------------|
;//        |        |       LR        |
;//        |        |-----------------|
;//        |        |    PC = task    |
;//        |        |-----------------|
;//        |        |      xPSR       |
;//    High Memory  |-----------------|
;//                 |      ....       |

__asm void PendSV_Handler(void)
{
//	MRS R0,PSP 								//读取当前进程栈指针数值 中断已经将R0~R3 LR PSP XPSR自动压栈
//	MOV R4,#4									//测试代码
//	MOV R5,#5									//测试代码
//	MOV R6,#6									//测试代码
//	MOV R7,#7									//测试代码
	
	STMDB R0!,{R4-R11}				//将R4~R11保存到任务栈中
	LDR R1,=__cpp(&curr_task)
	LDR R2,[R1]								//获得当前任务ID
	LDR R3,=__cpp(&PSP_array)
	
	LSL R2,R2,#2	 	
	STR R0,[R3,R2]			//将PSP数值保存到PSP_array

	//加载下一个上下文
	LDR R4,=__cpp(&next_task)
	LDR R4,[R4]									//得到下一个任务ID
	STR R4,[R1]									//设置curr_task =  next_task
	//LDR R0,[R3,R4,LSL,#2]			
	LSL R4,R4,#2	 
	LDR R0,[R3,R4]							
	
	LDMIA R0!,{R4-R11}				//从任务栈中加载R4~R11寄存器
	MSR PSP,R0								//设置PSP为下一个任务
	BX LR											//返回
	ALIGN 4										//数据对齐
}
/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
