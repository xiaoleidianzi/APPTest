//#include "system_stm32f10x.h"

#define VERINFO_Length   		0x08 			 							 //存放版本号FLASH的长度

#define VERINFO_ADDR_BASE  (0x08008000  - VERINFO_Length) /* Vector Table Relocation in Internal FLASH. */
