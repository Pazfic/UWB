#ifndef __ATCMD_H
#define __ATCMD_H
#include "flash_info.h"
#include "stdint.h"
#include "usart.h"

#include "usart.h"
#define STAR 0x41
#define RSET 0x42

#define BEEp 0x10
#define LEDe 0x11
#define ACCe 0x12
#define INFO 0x13
#define VBAT 0x14
#define BAUd 0x15
#define QSEt 0x16
#define GAIn 0x17
#define SOSe 0x18
#define UCHg 0x19
#define LBAe 0x1A

#define BEEP 0x20
#define LEDE 0x21
#define ACCE 0x22
#define BAUD 0x23
#define QSET 0x24
#define GAIN 0x25
#define SOSE 0x26
#define UCHG 0x27
#define LBAE 0x28
#define TCPE 0x29
#define TCPS 0x2A
#define SPED 0x2B
#define TWLT 0x2C
#define ACMD 0x2D
#define LIAP 0x2E
#define EROR 0x80

#define INFO_VERSION 0x01
#define INFO_SENSOR 0x02
#define INFO_UWB 0x04
#define INFO_ENET 0x10
#define INFO_BATTERY 0x20
#define INFO_OTHER 0x40
// 0x01 INFO_VERSION(�汾��Ϣ)
// 0x02 INFO_SENSOR(��������Ϣ)
// 0x04 INFO_UWB(UWB��Ϣ)
// 0x08 INFO_ENET(��̫�������Ϣ)
void display_deviceinfo(System_Para_TypeDef *syspara, uint16_t type);

void Display_Searching(void);
void Display_ATCMDInfo(void);

// u8 Check_Switch_Status(u16 value);

void Process_ATCMD(void);
#endif
