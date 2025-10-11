/*! ----------------------------------------------------------------------------
 *  @file    instance.h
 *  @brief   DecaWave header for application level instance
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "deca_types.h"
#include "deca_device_api.h"
#include "dwt_common.h"
/******************************************************************************************************************
********************* NOTES on DW (MP) features/options ***********************************************************
*******************************************************************************************************************/
#define DEEP_SLEEP (1) //To enable deep-sleep set this to 1
//DEEP_SLEEP mode can be used, for example, by a Tag instance to put the DW1000 into low-power deep-sleep mode:
// when the Anchor is sending the range report back to the Tag, the Tag will enter sleep after a ranging exchange is finished
// once it receives a report or times out, before the next poll message is sent (before next ranging exchange is started).

#define CORRECT_RANGE_BIAS  (1)     // Compensate for small bias due to uneven accumulator growth at close up high power


#define ANCTOANCTWR (1) //if set to 1 then anchor to anchor TWR will be done in the last slot
/******************************************************************************************************************
*******************************************************************************************************************
*******************************************************************************************************************/

#define NUM_INST            1
#define SPEED_OF_LIGHT      (299702547.0)     // in m/s in air
#define MASK_40BIT			(0x00FFFFFFFFFF)  // DW1000 counter is 40 bits
#define MASK_32BIT			(0x0000FFFFFFFF) 
#define MASK_TXDTS			(0x00FFFFFFFE00)  //The TX timestamp will snap to 8 ns resolution - mask lower 9 bits.

//! callback events
#define DWT_SIG_RX_NOERR            0
#define DWT_SIG_TX_DONE             1       // Frame has been sent
#define DWT_SIG_RX_OKAY             2       // Frame Received with Good CRC
#define DWT_SIG_RX_ERROR            3       // Frame Received but CRC is wrong
#define DWT_SIG_RX_TIMEOUT          4       // Timeout on receive has elapsed
#define DWT_SIG_TX_AA_DONE          6       // ACK frame has been sent (as a result of auto-ACK)
#define DWT_SIG_RX_BLINK			7		// Received ISO EUI 64 blink message
#define DWT_SIG_RX_PHR_ERROR        8       // Error found in PHY Header
#define DWT_SIG_RX_SYNCLOSS         9       // Un-recoverable error in Reed Solomon Decoder
#define DWT_SIG_RX_SFDTIMEOUT       10      // Saw preamble but got no SFD within configured time
#define DWT_SIG_RX_PTOTIMEOUT       11      // Got preamble detection timeout (no preamble detected)

#define DWT_SIG_TX_PENDING          12      // TX is pending
#define DWT_SIG_TX_ERROR            13      // TX failed
#define DWT_SIG_RX_PENDING          14      // RX has been re-enabled
#define DWT_SIG_DW_IDLE             15      // DW radio is in IDLE (no TX or RX pending)
#define DWT_SIG_RX					16
#define SIG_RX_UNKNOWN			99		// Received an unknown frame

//DecaRTLS frame function codes
#define RTLS_DEMO_MSG_TAG_POLL              (0x81)          // Tag poll message
#define RTLS_DEMO_MSG_ANCH_RESP             (0x70)          // Anchor response to poll
#define RTLS_DEMO_MSG_ANCH_POLL				(0x71)			// Anchor to anchor poll message
#define RTLS_DEMO_MSG_ANCH_RESP2            (0x72)          // Anchor response to poll from anchor
#define RTLS_DEMO_MSG_ANCH_FINAL            (0x73)          // Anchor final massage back to Anchor
#define RTLS_DEMO_MSG_TAG_FINAL             (0x82)          // Tag final massage back to Anchor


//lengths including the Decaranging Message Function Code byte
#define TAG_POLL_MSG_LEN                    5			           // FunctionCode(1), Range Num (1),ADC(2),SOS(1)
#define ANCH_RESPONSE_MSG_LEN               11               // FunctionCode(1),Range Num (1), Sleep Correction Time (2), Measured_TOF_Time(4), aircmd (2),crc(1)
#define TAG_FINAL_MSG_LEN                   42              // FunctionCode(1), Range Num (1), Poll_TxTime(5),
															// Resp0_RxTime(5), Resp1_RxTime(5), Resp2_RxTime(5), Resp3_RxTime(5), Resp4_RxTime(5),Resp5_RxTime(5),Final_TxTime(5)

#define MAX_MAC_MSG_DATA_LEN                (TAG_FINAL_MSG_LEN) //max message len of the above

#define STANDARD_FRAME_SIZE         127
//#define STANDARD_FRAME_SIZE         80

#define ADDR_BYTE_SIZE_L            (8)
#define ADDR_BYTE_SIZE_S            (2)

#define FRAME_CONTROL_BYTES         2
#define FRAME_SEQ_NUM_BYTES         1
#define FRAME_PANID                 2
#define FRAME_CRC					2
#define FRAME_SOURCE_ADDRESS_S        (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S          (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L        (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L          (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP					(FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID) //5
#define FRAME_CRTL_AND_ADDRESS_L    (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP) //21 bytes for 64-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_S    (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //9 bytes for 16-bit addresses)
#define FRAME_CRTL_AND_ADDRESS_LS	(FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP) //15 bytes for 1 16-bit address and 1 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_L-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_S-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS     (STANDARD_FRAME_SIZE-FRAME_CRTL_AND_ADDRESS_LS-TAG_FINAL_MSG_LEN-FRAME_CRC) //127 - 15 - 16 - 2 = 94

//NOTE: the user payload assumes that there are only 88 "free" bytes to be used for the user message (it does not scale according to the addressing modes)
#define MAX_USER_PAYLOAD_STRING	MAX_USER_PAYLOAD_STRING_LL

#define MAX_TAG_LIST_SIZE				(32)
#define	MAX_TAG_SIZE2            (128)
#define MAX_ANCHOR_LIST_SIZE			(4) //this is limited to 4 in this application
#define NUM_EXPECTED_RESPONSES			(3) //e.g. MAX_ANCHOR_LIST_SIZE - 1
#define NUM_EXPECTED_RESPONSES_ANC		(1) //anchors A0, A1 and A2 are involved in anchor to anchor ranging
#define NUM_EXPECTED_RESPONSES_ANC0		(2) //anchor A0 expects response from A1 and A2

#define GATEWAY_ANCHOR_ADDR				(0x8000)
#define A1_ANCHOR_ADDR					(0x8001)
#define A2_ANCHOR_ADDR					(0x8002)
#define A3_ANCHOR_ADDR					(0x8003)


// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define INST_DONE_WAIT_FOR_NEXT_EVENT   	1   //this signifies that the current event has been processed and instance is ready for next one
#define INST_DONE_WAIT_FOR_NEXT_EVENT_TO    2   //this signifies that the current event has been processed and that instance is waiting for next one with a timeout
                                        		//which will trigger if no event coming in specified time
#define INST_NOT_DONE_YET               	0   //this signifies that the instance is still processing the current event

//application data message byte offsets
#define FCODE                               0         // Function code is 1st byte of messageData
#define POLL_RNUM                           1         // Poll message range number
#define VRESP                               32				// Mask of valid response times (e.g. if bit 1 = A0's response time is valid)
//FINAL message

#define PTXT                                2				 // Poll TX time
#define RRXT0                               7				 // A0 Response RX time
#define RRXT1                               12			 // A1 Response RX time
#define RRXT2                               17				// A2 Response RX time
#define RRXT3                               22				// A3 Response RX time
//27
//32
#if (MAX_ANCHOR_LIST_SIZE==6)
#define FTXT                                37				// Final TX time
#elif (MAX_ANCHOR_LIST_SIZE==4)
#define FTXT                                27				// Final TX time
#elif (MAX_ANCHOR_LIST_SIZE==5)
#define FTXT                                32				// Final TX time
#endif

//RESPONSE message
#define RES_TAG_SLP0                        2               // Response tag sleep correction LSB
#define RES_TAG_SLP1                        3               // Response tag sleep correction MSB
#define TOFR                                4				// ToF (n-1) 4 bytes
#define AIR_COMMAND0												8				
#define AIR_COMMAND1												9				
#define CRC_BIT_RES													10
//POLL message
#define BATTERY_VALUE0                      2
#define BATTERY_VALUE1                      3
#define SOS_CODE                            4
//Anch message
#define MAX_TAG_NUM_L                       2
#define MAX_TAG_NUM_H  											3
#define TAG_SLOTTIME_L											4
#define TAG_SLOTTIME_H											5
#define CRC_BIT															6


typedef enum instanceModes{ TAG, ANCHOR} INST_MODE;



typedef enum inst_states
{
    TA_INIT, //0

    TA_TXE_WAIT,                //1 - state in which the instance will enter sleep (if ranging finished) or proceed to transmit a message
    TA_TXPOLL_WAIT_SEND,        //2 - configuration and sending of Poll message
    TA_TXFINAL_WAIT_SEND,       //3 - configuration and sending of Final message
    TA_TXRESPONSE_WAIT_SEND,    //4 - a place holder - response is sent from call back
		TA_NULL,
    TA_TX_WAIT_CONF,            //6 - confirmation of TX done message

    TA_RXE_WAIT,                //7
    TA_RX_WAIT_DATA,            //8

    TA_SLEEP_DONE,               //9
    TA_TXRESPONSE_SENT_POLLRX,    //10
    TA_TXRESPONSE_SENT_FINALRX,    //11
		TA_SLEEP
} INST_STATES;

typedef enum BAT_states
{
	VBAT_INIT,
	VBAT_CHG,
	VBAT_DISCHG,
	VBAT_IDLE

}VBAT_STATES;

// This file defines data and functions for access to Parameters in the Device
//message structure for Poll, Response and Final message

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  13-20 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LL] ; //  22-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlsl ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  07-08
    uint8 messageData[MAX_USER_PAYLOAD_STRING_SS] ; //  09-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dsss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_L];             	//  05-12 using 64 bit addresses
    uint8 sourceAddr[ADDR_BYTE_SIZE_S];           	//  13-14
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dlss ;

typedef struct
{
    uint8 frameCtrl[2];                         	//  frame control bytes 00-01
    uint8 seqNum;                               	//  sequence_number 02
    uint8 panID[2];                             	//  PAN ID 03-04
    uint8 destAddr[ADDR_BYTE_SIZE_S];             	//  05-06
    uint8 sourceAddr[ADDR_BYTE_SIZE_L];           	//  07-14 using 64 bit addresses
    uint8 messageData[MAX_USER_PAYLOAD_STRING_LS] ; //  15-124 (application data and any user payload)
    uint8 fcs[2] ;                              	//  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes.
} srd_msg_dssl ;

typedef struct
{
	uint8  type;			// event type - if 0 there is no event in the queue
	uint8  type_save;		// holds the event type - does not clear (used to show what event has been processed)
	uint8  type_pend;	    // set if there is a pending event
	uint16 rxLength ;		// length of RX data (does not apply to TX events)

	uint64_t timeStamp ;		// last timestamp (Tx or Rx) - 40 bit DW1000 time

	uint32 timeStamp32l ;		   // last tx/rx timestamp - low 32 bits of the 40 bit DW1000 time
	uint32 timeStamp32h ;		   // last tx/rx timestamp - high 32 bits of the 40 bit DW1000 time

	uint32 uTimeStamp ;			  //32 bit system counter (ms) - STM32 tick time (at time of IRQ)

	union {
			//holds received frame (after a good RX frame event)
			uint8   frame[STANDARD_FRAME_SIZE];
    		srd_msg_dlsl rxmsg_ll ; //64 bit addresses
			srd_msg_dssl rxmsg_sl ;
			srd_msg_dlss rxmsg_ls ;
			srd_msg_dsss rxmsg_ss ; //16 bit addresses
	}msgu;

	//uint32 eventtime ;
	//uint32 eventtimeclr ;
	uint8 gotit;			//stores the instance function which processed the event (used for debug)
}event_data_t ;



typedef struct
{
    INST_MODE mode;				//instance mode (tag or anchor)

    INST_STATES testAppState ;			//state machine - current state
    INST_STATES nextState ;				//state machine - next state
    INST_STATES previousState ;			//state machine - previous state
		VBAT_STATES testBatState;
		VBAT_STATES nextBatState;
		VBAT_STATES previousBatState;
		srd_msg_dsss msg_f ;
		uint16_t  panID ;
		uint16_t instanceAddress16;
		uint8_t	wait4ack ;
		uint8_t eui64[8];	
		uint8_t  responseTO ;	
		uint8_t	rangeNum;				// incremented for each sequence of ranges (each slot)
		uint8_t   frameSN;				// modulo 256 frame sequence number - it is incremented for each new frame transmission
		uint32_t TOF[MAX_TAG_SIZE2];
		uint32_t TOFArray[MAX_ANCHOR_LIST_SIZE];
		uint32_t Tof[MAX_ANCHOR_LIST_SIZE];
		uint8  sos_state;
		int longTermRangeCount ; //total number of ranges
		uint8_t   rxResponseMask;
} instance_data_t ;

extern instance_data_t ins;
//-------------------------------------------------------------------------------------------------------------
//
//	Functions used in logging/displaying range and status data
//
//-------------------------------------------------------------------------------------------------------------
#define instance_process_irq(x) 	dwt_isr() 

#ifdef __cplusplus
}
#endif

#endif
