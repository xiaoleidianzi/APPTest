#ifndef _LED_H_
#define _LED_H_





typedef struct
{

	_Bool Led4Sta;
	_Bool Led5Sta;
	_Bool Led6Sta;
	_Bool Led7Sta;

} LED_STATUS;

extern LED_STATUS led_status;


typedef enum
{

	LED_OFF = 0,
	LED_ON

} LED_ENUM;

typedef struct
{
	unsigned int CPUID;  //可用于识别处理器类型和版本的ID代码
	unsigned int ICSR;	 //系统异常的控制和状态
	unsigned int VTOR;	 //使能向量表重定位到其他的地址
	unsigned int AIRCR;  //优先级分组配置和自复位控制
	unsigned int SCR;		 //休眠模式和低功耗特性的配置
	unsigned int CCR;		 //高级特性的配置
	unsigned int SHP[12];//系统异常的优先级设置
	unsigned int SHCSR;	 //使能错误异常和系统异常状态的控制
	unsigned int CFSR;	 //引起错误异常的提示信息
	unsigned int HFSR;	 //引起硬件错误异常的提示信息
	unsigned int DFSR;	 //引起调试事件的提示信息
	unsigned int MMFAR;	 //存储器管理错误的地址值
	unsigned int BFAR;	 //总线错误的地址值
	unsigned int AFSR;	 //设备相关错误状态的信息
	unsigned int PFR[2]; //可用处理器特性的只读信息
	unsigned int DFR;		 //可用调试特性的只读信息
	unsigned int AFR;		 //可用辅助特性的只读信息
	unsigned int MMFR[4];//可用存储器模块特性的只读信息
	unsigned int ISAR[5];//指令集特性的只读信息
	unsigned int CPACR;	 //使能浮点特性的寄存器，只存在于具有浮点单元的Cortex-M4
	
}SCB_struct;
extern SCB_struct scb_struct;


void Led_Init(void);

void Led4_Set(LED_ENUM status);

void Led5_Set(LED_ENUM status);

void Led6_Set(LED_ENUM status);

void Led7_Set(LED_ENUM status);
void TESEREADinterrupt(int a,int b,int c,int d,int e,int f,int g,int h,int i,int j,int k,int l,int m,int n,int o,int p,int q,int r,int s,int t,int w,int v,int u,int x,int y,int z);

#endif
