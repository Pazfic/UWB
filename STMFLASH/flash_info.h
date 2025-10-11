#ifndef _FLASH_INFO_H_
#define _FLASH_INFO_H_
#include "stm32g0xx_hal.h"



#define DEFAULT_MAX_TAG_SIZE 8
#define DEFAULT_T_Slot			 10


typedef struct
{
	uint16_t flag;
	uint16_t start_count;
	uint16_t role;
	uint16_t channel;
	uint16_t datarate;
	uint16_t baud;	
	uint16_t gain1;	
	uint16_t gain2;
	uint16_t TAG_NUM;
	uint16_t TAG_SLOT;
	uint16_t twlt;
	uint16_t uwbid;
	
} System_Para_TypeDef; 

extern uint32_t FLAH_BUFF0[58];
extern uint32_t FLAH_BUFF1[4];
extern uint16_t twl_address[4];
extern uint32_t baudrate;
extern System_Para_TypeDef sys_para;


//u8 Check_cmd(u8* RXbuff);
uint8_t Get_UniqueID(void);
uint8_t RouterInfo_Write_to_Flash(uint8_t OBJ, uint8_t* buffer);
uint16_t GetRecSwtich(uint8_t* RXbuff);
void flash_config(void);
void app_flash_write(void);

#endif

