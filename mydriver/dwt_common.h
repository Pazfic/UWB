/*! ----------------------------------------------------------------------------
 * @file    dwt_common.h
 * @brief   Defines PDOA Tag related Common Macros, structures, function
 * definitions
 *
 * @author  Decawave
 *
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @modified Path Partner 2018
 */

#ifndef __DWT_COMMON__H__
#define __DWT_COMMON__H__

#ifdef __cplusplus
extern "C" {
#endif
#include "atcmd.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "flash_info.h"
#include "instance.h"
#include "main.h"
#include "port.h"
#include "string.h"
#include "tim.h"
/**********************************************************************************************/

extern int MAX_TAG_SIZE; // ???????
extern int N_Slot;       // ????????
extern int T_Slot;       // 1??????????? ms
extern int T_Round;      // ??????????
/**********************************************************************************************/

#define FINAL_MSG_TS_LEN 4
#define POLL_MSG_LEN     4
#define RES_MSG_LEN      4
#define UUS_TO_DWT_TIME  65536

#define TX_ANT_DLY       16436
#define RX_ANT_DLY       16436

#define SPEED_LIGHT      299702547 // ???? m/s

/* System mode of operation. used to
 *
 * 1. indicate in which mode of operation system is running
 * 2. configure the access rights to command handler in control mode
 * */

// TX power and PG delay configuration structure
typedef struct {
    uint8 PGdelay;

    // TX POWER
    // 31:24     BOOST_0.125ms_PWR
    // 23:16     BOOST_0.25ms_PWR-TX_SHR_PWR
    // 15:8      BOOST_0.5ms_PWR-TX_PHR_PWR
    // 7:0       DEFAULT_PWR-TX_DATA_PWR
    uint32 txPwr[2][2]; //
} tx_structr;

// uwb ?????????
typedef struct {
    uint8 channel;
    uint8 prf;
    uint8 datarate;
    uint8 preambleCode;
    uint8 preambleLength;
    uint8 pacSize;
    uint8 nsSFD;
    uint16 sfdTO;

} chConfig_t;

// typedef struct
//{
//	uint16 POLL_TX_TO_RESP_RX_DLY_UUS;//poll?????????????response?????
//
//	uint16  RESP_RX_TO_FINAL_TX_DLY_UUS ;//final????????????

//	uint16  RESP_RX_TIMEOUT_UUS ;//respon???????????

//	uint16 POLL_RX_TO_RESP_TX_DLY_UUS ;//response????????????

// 	uint16 RESP_TX_TO_FINAL_RX_DLY_UUS ;//response?????????????final?????

// 	uint16 FINAL_RX_TIMEOUT_UUS ;//final???????????

//}timeConfig_t;

typedef struct {
    uint32 status;     // initial value of register as ISR is entered
    uint8 event;       // event type
    uint8 aatset;      // auto ACK TX bit is set
    uint16 datalength; // length of frame
    uint8 fctrl[2];    // frame control bytes
    uint8 dblbuff;     // set if double buffer is enabled

} dwt_callback_data_t;

extern float RX_power_A;
extern chConfig_t chConfig[6];
extern const tx_structr txSpectrumConfigr[8];

// extern timeConfig_t timeconfig[6];
// extern timeConfig_t tconfig;
extern dwt_config_t config;
extern uint16 waittime;
extern int Anchpollingtime;

extern int USART_BUFF_LEN;
extern int T_ID;
extern int A_ID;
extern int MODULE_ROLE;
extern int rxdata_bit;

int deca_configure(void);

void twr_tx_tag_cb(const dwt_cb_data_t *txd);
void twr_rx_cb(const dwt_cb_data_t *rxd);
void twr_rx_timeout_cb(const dwt_cb_data_t *rxd);
void twr_rx_error_cb(const dwt_cb_data_t *rxd);
void testappruns(int modes);
void addressconfigure(int modes);
void insaddress(uint16 address);
void port_stop_all_UWB(void);

#ifdef __cplusplus
}
#endif

#endif /* __DW_PDOA_TAG_COMMON__H__ */
