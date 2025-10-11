#include "stmflash.h"
#include "stm32g0xx_hal.h"
//#include "stm32l0xx_hal_flash_ex.h"
//#include "stm32_hal_legacy.h"
//读取指定地址的字(32位数据) 
//faddr:读地址 
//返回值:对应数据.
uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
	return *(__IO uint32_t*)faddr; 
}

static uint32_t GetPage(uint32_t Addr)
{
  return (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
}


uint32_t endaddr=0;	
void STMFLASH_Write(uint32_t WriteAddr,uint64_t *pBuffer,uint32_t NumToWrite,uint8_t Page)	
{ 
	FLASH_EraseInitTypeDef FlashEraseInit;
	HAL_StatusTypeDef FlashStatus=HAL_OK;
	uint32_t PageError=0;
	uint32_t addrx=0;
	uint32_t FirstPage = 0;
	uint32_t NbOfPages = 0;

	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//非法地址
    
	HAL_FLASH_Unlock();             //解锁	
	addrx=WriteAddr;				//写入的起始地址
	//FLASH->ACR&=~(1<<10);  //FLASH擦除期间,必须禁止数据fetch
	endaddr=WriteAddr+NumToWrite;	//写入的结束地址
    /* Get the 1st page to erase */
  FirstPage = GetPage(addrx);

	FlashEraseInit.TypeErase=FLASH_TYPEERASE_PAGES;       //擦除类型，页擦除 

	FlashEraseInit.Page=FirstPage;
	FlashEraseInit.NbPages =Page;                         //一次擦除x页

	HAL_FLASHEx_Erase(&FlashEraseInit,&PageError) ;


	FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME);            //等待上次操作完成
	if(FlashStatus==HAL_OK)
	{
		 while(WriteAddr<endaddr)//写数据
		 {
			 if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,WriteAddr,*pBuffer)== HAL_OK)//写入数据
			 {
					pBuffer++;
					WriteAddr+=8;
			 }
			 else
			 {
					break;
			 }

		}
	}
	FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME); 
	//FLASH->ACR|=1<<10;                //FLASH擦除结束,开启数据fetch
	HAL_FLASH_Lock();           //上锁
} 

//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToRead:字(32位)数
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
	uint32_t i;
	uint32_t buf;
//	*pBuffer=*(__IO uint64_t*)ReadAddr; 
	for(i=0;i<NumToRead;i++)
	{
		buf=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		pBuffer[i]=buf;
		ReadAddr+=4;//偏移4个字节.	
	}
}


