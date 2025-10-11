#ifndef __STMFLASH_H
#define __STMFLASH_H
#include "stdint.h"

//FLASH起始地址
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
#define FLASH_WAITETIME  50000          //FLASH等待超时时间



#define ADDR_FLASH_PAGE_60 (0x08000000+2048*60) // 128kB 1page=2K
#define ADDR_FLASH_PAGE_61 (0x08000000+2048*61) // 128kB 1page=2K
 
uint32_t STMFLASH_ReadWord(uint32_t faddr);		  	//读出字  
void STMFLASH_Write(uint32_t WriteAddr,uint64_t *pBuffer,uint32_t NumToWrite,uint8_t Page);		//从指定地址开始写入指定长度的数据
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead);   		//从指定地址开始读出指定长度的数据
//测试写入
void Test_Write(uint32_t WriteAddr,uint32_t WriteData);	
#endif
