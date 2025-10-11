#include "stmflash.h"
#include "stm32g0xx_hal.h"
//#include "stm32l0xx_hal_flash_ex.h"
//#include "stm32_hal_legacy.h"
//��ȡָ����ַ����(32λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
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

	if(WriteAddr<STM32_FLASH_BASE||WriteAddr%4)return;	//�Ƿ���ַ
    
	HAL_FLASH_Unlock();             //����	
	addrx=WriteAddr;				//д�����ʼ��ַ
	//FLASH->ACR&=~(1<<10);  //FLASH�����ڼ�,�����ֹ����fetch
	endaddr=WriteAddr+NumToWrite;	//д��Ľ�����ַ
    /* Get the 1st page to erase */
  FirstPage = GetPage(addrx);

	FlashEraseInit.TypeErase=FLASH_TYPEERASE_PAGES;       //�������ͣ�ҳ���� 

	FlashEraseInit.Page=FirstPage;
	FlashEraseInit.NbPages =Page;                         //һ�β���xҳ

	HAL_FLASHEx_Erase(&FlashEraseInit,&PageError) ;


	FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME);            //�ȴ��ϴβ������
	if(FlashStatus==HAL_OK)
	{
		 while(WriteAddr<endaddr)//д����
		 {
			 if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,WriteAddr,*pBuffer)== HAL_OK)//д������
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
	//FLASH->ACR|=1<<10;                //FLASH��������,��������fetch
	HAL_FLASH_Lock();           //����
} 

//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToRead:��(32λ)��
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
	uint32_t i;
	uint32_t buf;
//	*pBuffer=*(__IO uint64_t*)ReadAddr; 
	for(i=0;i<NumToRead;i++)
	{
		buf=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		pBuffer[i]=buf;
		ReadAddr+=4;//ƫ��4���ֽ�.	
	}
}


