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
	unsigned int CPUID;  //������ʶ���������ͺͰ汾��ID����
	unsigned int ICSR;	 //ϵͳ�쳣�Ŀ��ƺ�״̬
	unsigned int VTOR;	 //ʹ���������ض�λ�������ĵ�ַ
	unsigned int AIRCR;  //���ȼ��������ú��Ը�λ����
	unsigned int SCR;		 //����ģʽ�͵͹������Ե�����
	unsigned int CCR;		 //�߼����Ե�����
	unsigned int SHP[12];//ϵͳ�쳣�����ȼ�����
	unsigned int SHCSR;	 //ʹ�ܴ����쳣��ϵͳ�쳣״̬�Ŀ���
	unsigned int CFSR;	 //��������쳣����ʾ��Ϣ
	unsigned int HFSR;	 //����Ӳ�������쳣����ʾ��Ϣ
	unsigned int DFSR;	 //��������¼�����ʾ��Ϣ
	unsigned int MMFAR;	 //�洢���������ĵ�ֵַ
	unsigned int BFAR;	 //���ߴ���ĵ�ֵַ
	unsigned int AFSR;	 //�豸��ش���״̬����Ϣ
	unsigned int PFR[2]; //���ô��������Ե�ֻ����Ϣ
	unsigned int DFR;		 //���õ������Ե�ֻ����Ϣ
	unsigned int AFR;		 //���ø������Ե�ֻ����Ϣ
	unsigned int MMFR[4];//���ô洢��ģ�����Ե�ֻ����Ϣ
	unsigned int ISAR[5];//ָ����Ե�ֻ����Ϣ
	unsigned int CPACR;	 //ʹ�ܸ������ԵļĴ�����ֻ�����ھ��и��㵥Ԫ��Cortex-M4
	
}SCB_struct;
extern SCB_struct scb_struct;


void Led_Init(void);

void Led4_Set(LED_ENUM status);

void Led5_Set(LED_ENUM status);

void Led6_Set(LED_ENUM status);

void Led7_Set(LED_ENUM status);
void TESEREADinterrupt(int a,int b,int c,int d,int e,int f,int g,int h,int i,int j,int k,int l,int m,int n,int o,int p,int q,int r,int s,int t,int w,int v,int u,int x,int y,int z);

#endif
