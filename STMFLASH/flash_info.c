#include "flash_info.h"  

#include "stmflash.h" 



System_Para_TypeDef sys_para;



//====================================================================//
// 语法格式：void Flash_Configuration(void)
// 实现功能：Flash记忆检测
// 参    数：无
// 返 回 值：无
// 备    注：无
//====================================================================//
#define FLAH_BUFF0_len 58
uint32_t FLAH_BUFF0[FLAH_BUFF0_len];
uint32_t FLAH_BUFF1[4];
uint16_t twl_address[4];
uint32_t baudrate=0;//01 23 45 67 89 1011 1213 1415 1617 1819 2021 2223 2425 2627 2829 3031 3233
void flash_config(void)
{

	  HAL_Delay(100);	
	  STMFLASH_Read(ADDR_FLASH_PAGE_60, FLAH_BUFF0, 42*4);	
		STMFLASH_Read(ADDR_FLASH_PAGE_60+42*4, &FLAH_BUFF0[42], (FLAH_BUFF0_len-42)*4);	
		HAL_Delay(100);	
	  STMFLASH_Read(ADDR_FLASH_PAGE_61, FLAH_BUFF1, sizeof(FLAH_BUFF1));

		sys_para.flag = FLAH_BUFF0[0];
		if(sys_para.flag != 0xAAAA)//If it is the new device
		{
			FLAH_BUFF0[1] = 1; // the first the to start
			sys_para.start_count = FLAH_BUFF0[1];	
			
			sys_para.baud    =FLAH_BUFF0[5]  = 0x0002;
			sys_para.gain1   =FLAH_BUFF0[6]  = 0x3939;
			sys_para.gain2   =FLAH_BUFF0[7]  = 0x3939;

			sys_para.TAG_NUM =FLAH_BUFF0[14] = DEFAULT_MAX_TAG_SIZE;
			sys_para.TAG_SLOT=FLAH_BUFF0[15] = DEFAULT_T_Slot;
			sys_para.twlt		 =FLAH_BUFF0[48] = 0x0000;
			sys_para.uwbid   =FLAH_BUFF1[0]  = 0xFFFF;
			FLAH_BUFF1[1]=0xAAAA;

			app_flash_write();	
		}
		else if(sys_para.flag == 0xAAAA)//如果不是第一次写Flash
		{
			
			FLAH_BUFF0[1]++;
			FLAH_BUFF1[1]=0xAAAA;
			sys_para.start_count  =  FLAH_BUFF0[1];
			sys_para.role         =  FLAH_BUFF0[2];
			sys_para.channel 	  =  FLAH_BUFF0[3];
			sys_para.datarate     =  FLAH_BUFF0[4];
			sys_para.baud		  =  FLAH_BUFF0[5];
			sys_para.gain1		  =	 FLAH_BUFF0[6];
			sys_para.gain2		  =	 FLAH_BUFF0[7];
			sys_para.TAG_NUM	  =	 FLAH_BUFF0[14];
			sys_para.TAG_SLOT	  =  FLAH_BUFF0[15];
			sys_para.twlt		  =  FLAH_BUFF0[48];

			sys_para.uwbid        = FLAH_BUFF1[0];	
			//app_flash_write();	
		}

		baudrate=460800;
}



void app_flash_write(void)
{
	STMFLASH_Write(ADDR_FLASH_PAGE_60, (uint64_t *)FLAH_BUFF0, sizeof(FLAH_BUFF0),1);//PAGE61，用作平时存储记录
	HAL_Delay(100);
	STMFLASH_Write(ADDR_FLASH_PAGE_61, (uint64_t *)FLAH_BUFF1, sizeof(FLAH_BUFF1),1);
	HAL_Delay(100);


}
